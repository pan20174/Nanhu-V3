package xiangshan.mem

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import utils._
import xs.utils._
import xiangshan._
import xs.utils.perf.HasPerfLogging
import xiangshan.backend.execute.fu.FuConfigs.lduCfg
import freechips.rocketchip.util.SeqToAugmentedSeq
import xiangshan.backend.issue.RsIdx
import xiangshan.backend.rob.RobPtr
import xiangshan.cache.{HasDCacheParameters, DcacheTLBypassLduIO}

object LoadReplayCauses {
  /*
    Replay has different priority causes.
    Lower the coding bits to give higher priority to replay.
  */
  /* Warning:
   * ************************************************************
   * *Do not alter the priority, as it could lead to a deadlock *
   * ************************************************************
   */
  // st-ld violation re-execute check
  val C_MA  = 0
  // tlb miss check
  val C_TM  = 1
  // store-to-load-forwarding check
  val C_FF  = 2
  // dcache replay check
  val C_DR  = 3
  // dcache miss check
  val C_DM  = 4
  // wpu predict fail
  val C_WF  = 5
  // dcache bank conflict check
  val C_BC  = 6
  // RAR queue accept check
  val C_RAR = 7
  // RAW queue accept check
  val C_RAW = 8
  // st-ld violation
  val C_NK  = 9
  // total causes
  val allCauses = 10
}
class ReplayInfo(implicit p: Parameters) extends XSBundle{
  val replayCause = Vec(LoadReplayCauses.allCauses, Bool())
  val schedIndex = UInt(log2Up(LoadReplayQueueSize).W)
  val isReplayQReplay = Bool()
  val full_fwd = Bool()
  // replay cause alias
  def mem_amb       = replayCause(LoadReplayCauses.C_MA)
  def tlb_miss      = replayCause(LoadReplayCauses.C_TM)
  def fwd_fail      = replayCause(LoadReplayCauses.C_FF)
  def dcache_rep    = replayCause(LoadReplayCauses.C_DR)
  def dcache_miss   = replayCause(LoadReplayCauses.C_DM)
  def wpu_fail      = replayCause(LoadReplayCauses.C_WF)
  def bank_conflict = replayCause(LoadReplayCauses.C_BC)
  def rar_nack      = replayCause(LoadReplayCauses.C_RAR)
  def raw_nack      = replayCause(LoadReplayCauses.C_RAW)
  def nuke          = replayCause(LoadReplayCauses.C_NK)
  def need_rep      = replayCause.asUInt.orR
}

class ReplayQUopInfo(implicit p: Parameters) extends XSBundle{
  val robIdx = new RobPtr
  val exceptionVec = ExceptionVec()
}



class LoadToReplayQueueBundle(implicit p: Parameters) extends XSBundle with HasDCacheParameters{
  val vaddr = UInt(VAddrBits.W)
  val paddr = UInt(PAddrBits.W)
  val mask = UInt(8.W)
  val uop = new MicroOp

  val tlbMiss = Bool()
  val tl_d_channel_wakeup = Input(new DcacheTLBypassLduIO)
  val mshrMissIDResp = Flipped(ValidIO(UInt(log2Up(cfg.nMissEntries).W)) )
  val replay = new ReplayInfo
}

class ReplayQueueIssueBundle(implicit p: Parameters) extends XSBundle {
  val vaddr = UInt(VAddrBits.W)
  val mask = UInt(8.W)
  val uop = new MicroOp
  val schedIndex = UInt(log2Up(LoadReplayQueueSize).W)
}
class RawDataModule[T <: Data](gen: T, numEntries: Int, numRead: Int, numWrite: Int)(implicit p: Parameters) extends XSModule{
  val io = IO(new Bundle(){
    val wen   = Input(Vec(numWrite, Bool()))
    val waddr = Input(Vec(numWrite, UInt(log2Up(numEntries).W)))
    val wdata = Input(Vec(numWrite, gen))

    val ren   = Input(Vec(numRead, Bool()))
    val raddr = Input(Vec(numRead, UInt(log2Up(numEntries).W)))
    val rdata = Output(Vec(numRead, gen))
  })
  val data = Reg(Vec(numEntries, gen))
  for(i <- 0 until(numRead)){
    io.rdata(i) := RegEnable(data(io.raddr(i)), io.ren(i))
  }
  for(i <- 0 until(numWrite)){
    when(io.wen(i)){
      data(io.waddr(i)) := io.wdata(i)
    }
  }
}

class ReplayQDebugBundle(implicit p: Parameters) extends XSBundle{
  val validNum = Output(UInt(log2Up(LoadReplayQueueSize).W))
  val issueNum = Output(UInt(log2Up(LoadReplayQueueSize).W))
  val debug_deqPtr = Input(new RobPtr)
  val debug_enqPtr = Input(new RobPtr)
}



class LoadReplayQueue(enablePerf: Boolean)(implicit p: Parameters) extends XSModule
  with HasLoadHelper
  with HasPerfLogging
  with HasDCacheParameters
{
  val io = IO(new Bundle() {
    val enq = Vec(LoadPipelineWidth, Flipped(DecoupledIO(new LoadToReplayQueueBundle)))
    val redirect = Flipped(ValidIO(new Redirect))
    val replayQIssue = Vec(LoadPipelineWidth, DecoupledIO(new ReplayQueueIssueBundle))
    val replayQFull = Output(Bool())
    val ldStop = Output(Bool())
    val tlDchannelWakeupDup = Input(new DcacheTLBypassLduIO)
    val degbugInfo = new ReplayQDebugBundle
    })

  val counterRegMax = 16
  val penaltyRegWidth = log2Up(counterRegMax)

  // replayQueue state signs define
  // allocated: the entry has been enqueued
  val allocatedReg = RegInit(VecInit(List.fill(LoadReplayQueueSize)(false.B)))
  // scheduled: the entry has beed issued
  val scheduledReg = RegInit(VecInit(List.fill(LoadReplayQueueSize)(false.B)))
  // blocking: the entry's reply cause has not been resolved yet
  val blockingReg = RegInit(VecInit(List.fill(LoadReplayQueueSize)(false.B)))
  // replayCause: the reason for replaying, monitor whether the exception is solved
  val causeReg = RegInit(VecInit(List.fill(LoadReplayQueueSize)(0.U(LoadReplayCauses.allCauses.W))))
  // uop: micro op per entry
  val uopReg = RegInit(VecInit(List.fill(LoadReplayQueueSize)(0.U.asTypeOf(new MicroOp))))
  // replay penalty, prevent dead block
  val penaltyReg = RegInit(VecInit(List.fill(LoadReplayQueueSize)(0.U(penaltyRegWidth.W))))
  val counterReg = RegInit(VecInit(List.fill(LoadReplayQueueSize)(0.U(log2Up(counterRegMax).W))))
  // hintIDReg: store the Hint ID of TLB miss or Dcache miss
  val hintIDReg = RegInit(VecInit(List.fill(LoadReplayQueueSize)(0.U(reqIdWidth.W))))
  // mshrIDreg: store the L1 MissQ mshr ID
  val mshrIDreg = RegInit(VecInit(List.fill(LoadReplayQueueSize)(0.U((log2Up(cfg.nMissEntries+1).W)))))
  
  // replayQueue enq\deq control
  val freeList = Module(new FreeList(
    size = LoadReplayQueueSize,
    allocWidth = LoadPipelineWidth,
    freeWidth = 4,
    enablePreAlloc = true,
    moduleName = "LoadReplayQueue freelist"
  ))
  val vaddrModule = Module(new RawDataModule(
    gen = UInt(VAddrBits.W),
    numEntries = LoadReplayQueueSize,
    numRead = LoadPipelineWidth,
    numWrite = LoadPipelineWidth))
  // replayQueue Full Backpressure Logic
  val lqFull = freeList.io.empty
  val lqFreeNums = freeList.io.validCount
  io.replayQFull := lqFull

  // enq req control
  val enqReqValid = io.enq.map(_.valid)
  val enqReqBits = io.enq.map(_.bits)
  val enqReqNeedReplay = io.enq.map(req => req.bits.replay.need_rep && req.valid)
  val cancelEnq = io.enq.map(enq => enq.bits.uop.robIdx.needFlush(io.redirect))
  val hasExceptions = io.enq.map(enq => ExceptionNO.selectByFu(enq.bits.uop.cf.exceptionVec, lduCfg).asUInt.orR && !enq.bits.tlbMiss)
  val needEnqueue = VecInit((0 until LoadPipelineWidth).map(i => {
    enqReqValid(i) && !cancelEnq(i) && enqReqNeedReplay(i) && !hasExceptions(i)
  }))
  val enqIndexOH =  WireInit(VecInit.fill(LoadPipelineWidth)(0.U(LoadReplayQueueSize.W)))
  val s1_robOldestSelOH = WireInit(VecInit.fill(LoadPipelineWidth)(0.U(LoadReplayQueueSize.W)))
  val robOldestSelOH = WireInit(VecInit.fill(LoadPipelineWidth)(0.U(LoadReplayQueueSize.W)))
  // Allocate logic
  val newEnqueue = (0 until LoadPipelineWidth).map(i => {
    needEnqueue(i) && !io.enq(i).bits.replay.isReplayQReplay
  })
  // freeList enq/deq logic + entry allocate logic
  val freeMaskVec = WireInit(VecInit.fill(LoadReplayQueueSize)(false.B))
  for ((enq, i) <- io.enq.zipWithIndex) {
    vaddrModule.io.wen(i) := false.B
    vaddrModule.io.waddr(i) := 0.U
    vaddrModule.io.wdata(i) := 0.U
    freeList.io.doAllocate(i) := false.B
    freeList.io.allocateReq(i) := true.B

    val offset = PopCount(newEnqueue.take(i))
    // freeList allocated ready
    val canAccept = Mux(enq.bits.replay.isReplayQReplay, true.B, freeList.io.canAllocate(offset))
    val enqIndex = Mux(enq.bits.replay.isReplayQReplay, enq.bits.replay.schedIndex, freeList.io.allocateSlot(offset))
    enq.ready := canAccept
    enqIndexOH(i) := UIntToOH(enqIndex)
    when(enqReqNeedReplay(i) && canAccept ){
      // freelist actually allocate new entry
      freeList.io.doAllocate(i) := !enq.bits.replay.isReplayQReplay
      // allocate new entry
      allocatedReg(enqIndex) := true.B
      scheduledReg(enqIndex) := false.B
      uopReg(enqIndex)       := enq.bits.uop
      causeReg(enqIndex)     := enq.bits.replay.replayCause.asUInt
      penaltyReg(enqIndex)   := 0.U
      blockingReg(enqIndex)  := true.B
      hintIDReg(enqIndex)    := 0.U
      when(enq.bits.replay.replayCause(LoadReplayCauses.C_BC)){
        counterReg(enqIndex) := 1.U << penaltyReg(enqIndex)
        penaltyReg(enqIndex)  := penaltyReg(enqIndex) + 1.U
        blockingReg(enqIndex) := true.B
      }
      when(enq.bits.replay.replayCause(LoadReplayCauses.C_TM)){
        hintIDReg(enqIndex) := (1.U << reqIdWidth) - 1.U
      }
      when(enq.bits.replay.replayCause(LoadReplayCauses.C_DM)){
        blockingReg(enqIndex) := (!enq.bits.replay.full_fwd) || 
        (!(enq.bits.tl_d_channel_wakeup.valid && enq.bits.tl_d_channel_wakeup.hitInflightDcacheResp))
        when(enq.bits.mshrMissIDResp.valid){
          mshrIDreg(enqIndex) := enq.bits.mshrMissIDResp.bits
        }
      }

      vaddrModule.io.wen(i)   := true.B
      vaddrModule.io.waddr(i) := enqIndex
      vaddrModule.io.wdata(i) := enq.bits.vaddr
      if(enablePerf){for(i <- 0 until(LoadReplayQueueSize)){
        XSPerfAccumulate(s"LoadReplayQueue_entry_${i}_used", enqIndex === i.asUInt)
      }}
    }

    // Upon replay success, need to deallocate the entry; otherwise, need to replay again
    when(enq.valid && enq.bits.replay.isReplayQReplay){
      when(!enqReqNeedReplay(i) || hasExceptions(i)){
        allocatedReg(enq.bits.replay.schedIndex) := false.B
        freeMaskVec(enq.bits.replay.schedIndex) := true.B
        penaltyReg(enq.bits.replay.schedIndex) := 0.U
      }.otherwise{
        scheduledReg(enq.bits.replay.schedIndex) := false.B
      }
    }
  }

  (0 until LoadReplayQueueSize).map(i => {
    // case TLB MSS
    when (causeReg(i)(LoadReplayCauses.C_TM)) {
      blockingReg(i) := Mux(hintIDReg(i) =/= 0.U, true.B, false.B)
    }
    // case Dcache MSS
    when (causeReg(i)(LoadReplayCauses.C_DM)) {
      blockingReg(i) :=  Mux(io.tlDchannelWakeupDup.valid && io.tlDchannelWakeupDup.mshrid === mshrIDreg(i), false.B, blockingReg(i))
    }
  })

  allocatedReg.zipWithIndex.foreach({case(valid,idx) => {
    when(valid){
      counterReg(idx) := counterReg(idx) - 1.U
      when(counterReg(idx) === 0.U && blockingReg(idx)){
        blockingReg(idx) := false.B
      }
    }
  }})

  // misprediction recovery / exception redirect
  val needCancel = WireInit(VecInit.fill(LoadReplayQueueSize)(false.B))
  for (i <- 0 until LoadReplayQueueSize) {
    needCancel(i) := uopReg(i).robIdx.needFlush(io.redirect) && allocatedReg(i)
    when (needCancel(i)) {
      allocatedReg(i) := false.B
      freeMaskVec(i) := true.B
    }
  }
  freeList.io.free := freeMaskVec.asUInt

  // replay entry select logic
  def getRemBits(input: UInt, rem: Int): UInt = {
    VecInit((0 until LoadReplayQueueSize / LoadPipelineWidth).map(i =>
      { input(LoadPipelineWidth * i + rem) } )).asUInt
  }
  def getRemUop[T <: Data](input: Vec[T], rem: Int): Vec[T] = {
    VecInit((0 until LoadReplayQueueSize / LoadPipelineWidth).map(i =>
      input(LoadPipelineWidth * i + rem)
    ))
  }
  val s1_selResSeq = Wire(Vec(LoadPipelineWidth, Valid(UInt((LoadReplayQueueSize / LoadPipelineWidth).W))))
  dontTouch(s1_selResSeq)
  val s0_readyToReplay_mask = VecInit((0 until LoadReplayQueueSize).map(i => {
    allocatedReg(i) && !scheduledReg(i) && !blockingReg(i)
  }))
  dontTouch(s0_readyToReplay_mask)
  s1_selResSeq := (0 until LoadPipelineWidth).map{ rem =>
    val s0_remReadyToReplay_uop = getRemUop(uopReg, rem)
    val s0_remReadyToReplay_mask = getRemBits(s0_readyToReplay_mask.asUInt, rem)
    dontTouch(s0_remReadyToReplay_mask)
    val s0_remReadyToReplay_seq = (0 until LoadReplayQueueSize/LoadPipelineWidth).map{ i =>
      val valid = s0_remReadyToReplay_mask(i) && !(Mux(s1_selResSeq(rem).valid, s1_selResSeq(rem).bits(i).asBool, false.B))
      val uop = s0_remReadyToReplay_uop(i)
      val validUop = Wire(Valid(new MicroOp))
      validUop.valid := valid && !uop.robIdx.needFlush(io.redirect)
      validUop.bits := uop
      validUop
    }
    SelectPolicy((s0_remReadyToReplay_seq), true, true, LoadReplayQueueSize/LoadPipelineWidth, io.redirect, p)
  }
  s1_robOldestSelOH := VecInit((0 until LoadPipelineWidth).map(rem => {
    val oldestBitsVec = WireInit(VecInit.fill(LoadReplayQueueSize)(false.B))
    for (i <- 0 until LoadReplayQueueSize / LoadPipelineWidth) {
      oldestBitsVec(i * LoadPipelineWidth + rem) := s1_selResSeq(rem).bits(i)
    }
    oldestBitsVec.asUInt
  }))
  val debug_robOldestSelOH = WireInit(VecInit.fill(LoadPipelineWidth)(0.U(log2Up(LoadReplayQueueSize).W)))
  for(i <- 0 until LoadPipelineWidth){
    debug_robOldestSelOH(i) := OHToUInt(s1_robOldestSelOH(i))
    dontTouch(debug_robOldestSelOH(i))
  }
  for (i <- 0 until LoadPipelineWidth) {
    for (j <- 0 until LoadReplayQueueSize) {
      when (s1_selResSeq(i).valid && s1_robOldestSelOH(i)(j)) {
        scheduledReg(j) := true.B
      }
    }
  }
  // replay issue logic
  val s1_SelReplayIdx = WireInit(VecInit.fill(LoadPipelineWidth)(0.U(log2Up(LoadReplayQueueSize).W)))
  val s2_replay_req = Wire(Vec(LoadPipelineWidth, DecoupledIO(new ReplayQueueIssueBundle)))
  for (i <- 0 until LoadPipelineWidth) {
    s1_SelReplayIdx(i) := OHToUInt(s1_robOldestSelOH(i))
    dontTouch(s1_SelReplayIdx)
    vaddrModule.io.ren(i) := s1_selResSeq(i).valid
    vaddrModule.io.raddr(i) := s1_SelReplayIdx(i)

    val s2_replay_req_schedIndex = RegEnable(s1_SelReplayIdx(i), s1_selResSeq(i).valid)
    val s2_replay_req_uop = uopReg(s2_replay_req_schedIndex)

    s2_replay_req(i).valid := RegNext(s1_selResSeq(i).valid) && !s2_replay_req_uop.robIdx.needFlush(io.redirect) // s2 out valid
    s2_replay_req(i).bits.vaddr := vaddrModule.io.rdata(i)   // s2 read vaddr
    s2_replay_req(i).bits.schedIndex := s2_replay_req_schedIndex   // s2 out idx
    s2_replay_req(i).bits.uop := s2_replay_req_uop // s2 read uop reg

    s2_replay_req(i).bits.mask := 0.U

    io.replayQIssue(i) <> s2_replay_req(i)
    dontTouch(io.replayQIssue(i))
  }
  io.ldStop := s1_selResSeq.map(seq => seq.valid).reduce(_ | _)
  //  perf cnt
  val enqNumber               = PopCount(io.enq.map(enq => enq.fire && !enq.bits.replay.isReplayQReplay))
  val deqNumber               = PopCount(io.replayQIssue.map(_.fire))
  val deqBlockCount           = PopCount(io.replayQIssue.map(r => r.valid && !r.ready))
  val replayTlbMissCount      = PopCount(io.enq.map(enq => enq.fire && !enq.bits.replay.isReplayQReplay && enq.bits.replay.replayCause(LoadReplayCauses.C_TM)))
  val replayMemAmbCount       = PopCount(io.enq.map(enq => enq.fire && !enq.bits.replay.isReplayQReplay && enq.bits.replay.replayCause(LoadReplayCauses.C_MA)))
  val replayNukeCount         = PopCount(io.enq.map(enq => enq.fire && !enq.bits.replay.isReplayQReplay && enq.bits.replay.replayCause(LoadReplayCauses.C_NK)))
  val replayRARRejectCount    = PopCount(io.enq.map(enq => enq.fire && !enq.bits.replay.isReplayQReplay && enq.bits.replay.replayCause(LoadReplayCauses.C_RAR)))
  val replayRAWRejectCount    = PopCount(io.enq.map(enq => enq.fire && !enq.bits.replay.isReplayQReplay && enq.bits.replay.replayCause(LoadReplayCauses.C_RAW)))
  val replayBankConflictCount = PopCount(io.enq.map(enq => enq.fire && !enq.bits.replay.isReplayQReplay && enq.bits.replay.replayCause(LoadReplayCauses.C_BC)))
  val replayDCacheReplayCount = PopCount(io.enq.map(enq => enq.fire && !enq.bits.replay.isReplayQReplay && enq.bits.replay.replayCause(LoadReplayCauses.C_DR)))
  val replayForwardFailCount  = PopCount(io.enq.map(enq => enq.fire && !enq.bits.replay.isReplayQReplay && enq.bits.replay.replayCause(LoadReplayCauses.C_FF)))
  val replayDCacheMissCount   = PopCount(io.enq.map(enq => enq.fire && !enq.bits.replay.isReplayQReplay && enq.bits.replay.replayCause(LoadReplayCauses.C_DM)))

  val validNum = PopCount(allocatedReg)
  val issueNum = PopCount(scheduledReg)
  io.degbugInfo.validNum := validNum
  io.degbugInfo.issueNum := issueNum
  dontTouch(io.degbugInfo)

  io.enq.foreach({ case enq =>
    when(enq.valid) {
      assert(enq.bits.uop.robIdx >= io.degbugInfo.debug_deqPtr &&
              enq.bits.uop.robIdx < io.degbugInfo.debug_enqPtr,"illegal rob enqueue replay Queue!!!")
    }
  })


  if(enablePerf){ XSPerfAccumulate("enq", enqNumber)
  XSPerfAccumulate("deq", deqNumber)
  XSPerfAccumulate("deq_block", deqBlockCount)
  XSPerfAccumulate("replay_full", io.replayQFull)
  XSPerfAccumulate("replay_rar_nack", replayRARRejectCount)
  XSPerfAccumulate("replay_raw_nack", replayRAWRejectCount)
  XSPerfAccumulate("replay_nuke", replayNukeCount)
  XSPerfAccumulate("replay_mem_amb", replayMemAmbCount)
  XSPerfAccumulate("replay_tlb_miss", replayTlbMissCount)
  XSPerfAccumulate("replay_bank_conflict", replayBankConflictCount)
  XSPerfAccumulate("replay_dcache_replay", replayDCacheReplayCount)
  XSPerfAccumulate("replay_forward_fail", replayForwardFailCount)
  XSPerfAccumulate("replay_dcache_miss", replayDCacheMissCount)}

}