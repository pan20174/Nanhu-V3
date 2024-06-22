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
import xiangshan.cache.{DCacheTLDBypassLduIO, HasDCacheParameters}
import xiangshan.cache.mmu.HasTlbConst

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
  // dcache bank conflict check
  val C_BC  = 5
  // RAR queue accept check
  val C_RAR = 6
  // RAW queue accept check
  val C_RAW = 7
  // st-ld violation
  val C_NK  = 8
  // total causes
  val allCauses = 9
}
class ReplayInfo(implicit p: Parameters) extends XSBundle{
  val replayCause = Vec(LoadReplayCauses.allCauses, Bool())
  val schedIndex = UInt(log2Up(LoadReplayQueueSize).W)
  val isReplayQReplay = Bool()
  val full_fwd = Bool()
  val fwd_data_sqIdx = new SqPtr
  // replay cause alias
  def mem_amb       = replayCause(LoadReplayCauses.C_MA)
  def tlb_miss      = replayCause(LoadReplayCauses.C_TM)
  def fwd_fail      = replayCause(LoadReplayCauses.C_FF)
  def dcache_rep    = replayCause(LoadReplayCauses.C_DR)
  def dcache_miss   = replayCause(LoadReplayCauses.C_DM)
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

class LoadTLBWakeUpBundle(implicit p: Parameters) extends XSBundle with HasTlbConst{
  val vpn = UInt(vpnLen.W)
}


class LoadToReplayQueueBundle(implicit p: Parameters) extends XSBundle with HasDCacheParameters{
  val vaddr = UInt(VAddrBits.W)
  val paddr = UInt(PAddrBits.W)
  val mask = UInt(8.W)
  val uop = new MicroOp

  val tlbMiss = Bool()
  val mshrMissIDResp = (UInt(log2Up(cfg.nMissEntries).W))
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

  for(i <- 0 until numWrite){
    for(j <- (i + 1) until numWrite){
      when(io.wen(i) && io.wen(j)){
        assert(!(io.waddr(i) === io.waddr(j)),"can not write the same address")
      }
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
    val tlDchannelWakeup = Input(new DCacheTLDBypassLduIO)
    val stDataReadyVec = Input(Vec(StoreQueueSize, Bool()))
    val stDataReadySqPtr = Input(new SqPtr)
    val sqEmpty= Input(Bool())
    val storeDataWbPtr = Vec(StorePipelineWidth, Flipped(Valid(new SqPtr)))
    val tlbWakeup = Flipped(ValidIO(new LoadTLBWakeUpBundle))
    val degbugInfo = new ReplayQDebugBundle
    val mshrFull = Input(Bool())
    val rawIsFull = Input(Bool())
    val loadDeqPtr = Input(new LqPtr)
  })

  val counterRegMax = 32
  val penaltyRegWidth = log2Up(counterRegMax) + 1

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
  // blockSqIdxReg: store the sqIdx which block load forward data
  val blockSqIdxReg = RegInit(VecInit(List.fill(LoadReplayQueueSize)(0.U.asTypeOf(new SqPtr))))


  //tmp: use vaddr
  val vaddrReg = RegInit(VecInit(List.fill(LoadReplayQueueSize)(0.U(VAddrBits.W))))

  // replayQueue enq\deq control
  val freeList = Module(new LsqFreeList(
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
  val enqReqFire = io.enq.map(_.fire)
  val enqReqBits = io.enq.map(_.bits)
  val enqReqNeedReplay = io.enq.map(req => req.bits.replay.need_rep && req.valid)
  val cancelEnq = io.enq.map(enq => enq.bits.uop.robIdx.needFlush(io.redirect))
  val hasExceptions = io.enq.map(enq => ExceptionNO.selectByFu(enq.bits.uop.cf.exceptionVec, lduCfg).asUInt.orR && !enq.bits.tlbMiss)
  val needEnqueue = VecInit((0 until LoadPipelineWidth).map(i => {
    enqReqFire(i) && !cancelEnq(i) && enqReqNeedReplay(i) && !hasExceptions(i)
  }))
  val enqIndexOH =  WireInit(VecInit.fill(LoadPipelineWidth)(0.U(LoadReplayQueueSize.W)))
  val enqIndex =  WireInit(VecInit.fill(LoadPipelineWidth)(0.U(log2Up(LoadReplayQueueSize).W)))
  val enqNeedAlloacteNew =  WireInit(VecInit.fill(LoadPipelineWidth)(false.B))
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
    enqIndex(i) := Mux(enq.bits.replay.isReplayQReplay, enq.bits.replay.schedIndex, freeList.io.allocateSlot(offset))
    enq.ready := canAccept
    enqIndexOH(i) := UIntToOH(enqIndex(i))
    enqNeedAlloacteNew(i) := enqReqNeedReplay(i) && canAccept
    when(enqNeedAlloacteNew(i)){
      // freelist actually allocate new entry
      freeList.io.doAllocate(i) := !enq.bits.replay.isReplayQReplay
      // allocate new entry
      allocatedReg(enqIndex(i)) := true.B
      scheduledReg(enqIndex(i)) := false.B
      uopReg(enqIndex(i))       := enq.bits.uop
      causeReg(enqIndex(i))     := enq.bits.replay.replayCause.asUInt
      hintIDReg(enqIndex(i))    := 0.U
      vaddrReg(enqIndex(i)) := enq.bits.vaddr
      vaddrModule.io.wen(i)   := true.B
      vaddrModule.io.waddr(i) := enqIndex(i)
      vaddrModule.io.wdata(i) := enq.bits.vaddr
      if(enablePerf){for(j <- 0 until(LoadReplayQueueSize)) {
        XSPerfAccumulate(s"LoadReplayQueue_entry_${i}_used", enqIndex(i) === j.asUInt)
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

  // val storeAddrInSameCycleVec = Wire(Vec(LoadReplayQueueSize, Bool()))
  val storeDataInSameCycleVec = Wire(Vec(LoadReplayQueueSize, Bool()))
  // val addrNotBlockVec = Wire(Vec(LoadReplayQueueSize, Bool()))
  val dataNotBlockVec = Wire(Vec(LoadReplayQueueSize, Bool()))
  // val storeAddrValidVec = addrNotBlockVec.asUInt | storeAddrInSameCycleVec.asUInt
  val storeDataValidVec = dataNotBlockVec.asUInt | storeDataInSameCycleVec.asUInt
  // store data valid check
  // val stAddrReadyVec = io.stAddrReadyVec
  val stDataReadyVec = io.stDataReadyVec
  for (i <- 0 until LoadReplayQueueSize) {
    // dequeue
    dataNotBlockVec(i) := (io.stDataReadySqPtr > blockSqIdxReg(i)) || stDataReadyVec(blockSqIdxReg(i).value) || io.sqEmpty // for better timing
    // store data execute
    storeDataInSameCycleVec(i) := VecInit((0 until StorePipelineWidth).map(w => {
      io.storeDataWbPtr(w).valid &&
      blockSqIdxReg(i) === io.storeDataWbPtr(w).bits
    })).asUInt.orR
  }

  // store data issue check
  val stDataDeqVec = Wire(Vec(LoadReplayQueueSize, Bool()))
  (0 until LoadReplayQueueSize).map(i => {
    stDataDeqVec(i) := allocatedReg(i) && storeDataValidVec(i)
  })

  (0 until LoadReplayQueueSize).foreach(i => {
    (0 until LoadPipelineWidth).foreach(j => {
      when(enqNeedAlloacteNew(j) && (enqIndex(j) === i.asUInt)){
        // case TLB MISS
        when(io.enq(j).bits.replay.tlb_miss){
          counterReg(i) := 1.U << penaltyReg(i)
          penaltyReg(i) := penaltyReg(i) + 1.U
          blockingReg(i) := true.B
        }
        // case Forward Fail
        when(io.enq(j).bits.replay.fwd_fail){
          blockingReg(i) := true.B
          blockSqIdxReg(i) := io.enq(j).bits.replay.fwd_data_sqIdx
        }
        // case Dcache no mshr
        when(io.enq(j).bits.replay.dcache_rep){
          blockingReg(i) := true.B
        }
        // case Dcache MISS
        when(io.enq(j).bits.replay.dcache_miss){
          blockingReg(i) := (!io.enq(j).bits.replay.full_fwd) && !(io.tlDchannelWakeup.valid && io.tlDchannelWakeup.mshrid === io.enq(j).bits.mshrMissIDResp)
          mshrIDreg(i) := io.enq(j).bits.mshrMissIDResp
        }
        // case Bank Conflict
        when(io.enq(j).bits.replay.bank_conflict){
          counterReg(i) := 1.U << penaltyReg(i)
          penaltyReg(i)  := penaltyReg(i) + 1.U
          blockingReg(i) := true.B
        }
        // case read after read
        when(io.enq(j).bits.replay.raw_nack) {
          blockingReg(i) := Mux(io.rawIsFull,true.B,false.B)
        }
      }
    })
  })

  (0 until LoadReplayQueueSize).foreach(i => {
    // otherwise listening the casue whether has been solved
    // case TLB MISS
    when(causeReg(i)(LoadReplayCauses.C_TM)) {
      blockingReg(i) := !(counterReg(i) === 0.U)
    }
    // case Forward Fail
    when(causeReg(i)(LoadReplayCauses.C_FF)) {
      when(stDataDeqVec(i)){
        blockingReg(i) := false.B
      }
    }
    // case Dcache no mshr
    when(causeReg(i)(LoadReplayCauses.C_DR)) {
      when((io.tlDchannelWakeup.valid && (io.tlDchannelWakeup.mshrid <= 15.U)) || !io.mshrFull){
        blockingReg(i) := false.B
      }
    }
    // case Dcache MISS
    when(causeReg(i)(LoadReplayCauses.C_DM)) {
      when(io.tlDchannelWakeup.valid && io.tlDchannelWakeup.mshrid === mshrIDreg(i)){
        blockingReg(i) := false.B
      }
    }
    // case Bank Conflict
    when(causeReg(i)(LoadReplayCauses.C_BC)) {
      blockingReg(i) := !(counterReg(i) === 0.U)
    }
    // case read after read
    when(causeReg(i)(LoadReplayCauses.C_RAW)) {
      blockingReg(i) := Mux(!io.rawIsFull || (io.loadDeqPtr === uopReg(i).lqIdx), false.B, true.B)
    }
  })


  allocatedReg.zipWithIndex.foreach({case(valid,idx) => {
    when(valid && (counterReg(idx) =/= 0.U)){
      counterReg(idx) := counterReg(idx) - 1.U
    }
  }})

  // misprediction recovery / exception redirect
  val needCancel = WireInit(VecInit.fill(LoadReplayQueueSize)(false.B))
  for (i <- 0 until LoadReplayQueueSize) {
    needCancel(i) := uopReg(i).robIdx.needFlush(io.redirect) && allocatedReg(i)
    when (needCancel(i)) {
      allocatedReg(i) := false.B
      freeMaskVec(i) := true.B
      causeReg(i) := 0.U
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
    ReplayQueueSelectPolicy((s0_remReadyToReplay_seq), true, true, LoadReplayQueueSize/LoadPipelineWidth, io.redirect, p)
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
        causeReg(j)     := 0.U
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
//    assert(vaddrReg(s2_replay_req_schedIndex) === vaddrModule.io.rdata(i),"the vaddr must be equal!!!")
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