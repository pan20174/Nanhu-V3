/***************************************************************************************
* Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
* Copyright (c) 2020-2021 Peng Cheng Laboratory
*
* XiangShan is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*          http://license.coscl.org.cn/MulanPSL2
*
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
*
* See the Mulan PSL v2 for more details.
***************************************************************************************/

package xiangshan.mem

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import utils._
import xs.utils._
import xiangshan._
import xiangshan.backend.execute.fu.fpu.FPU
import xiangshan.backend.rob.RobPtr
import xiangshan.cache._
import xiangshan.frontend.FtqPtr
import xiangshan.ExceptionNO._
import xiangshan.backend.execute.fu.FuConfigs
import xiangshan.backend.issue.SelectPolicy
import xiangshan.mem.lsqueue.{LSQExceptionGen, LoadRAWQueueDataModule}
import xs.utils.perf.HasPerfLogging

class LqPtr(implicit p: Parameters) extends CircularQueuePtr[LqPtr](
  p => p(XSCoreParamsKey).LoadQueueSize
){
}

object LqPtr {
  def apply(f: Bool, v: UInt)(implicit p: Parameters): LqPtr = {
    val ptr = Wire(new LqPtr)
    ptr.flag := f
    ptr.value := v
    ptr
  }
}

trait HasLoadHelper { this: XSModule =>
  def rdataHelper(uop: MicroOp, rdata: UInt): UInt = {
    val fpWen = uop.ctrl.fpWen
    LookupTree(uop.ctrl.fuOpType, List(
      LSUOpType.lb   -> SignExt(rdata(7, 0) , XLEN),
      LSUOpType.lh   -> SignExt(rdata(15, 0), XLEN),
      /*
          riscv-spec-20191213: 12.2 NaN Boxing of Narrower Values
          Any operation that writes a narrower result to an f register must write
          all 1s to the uppermost FLEN−n bits to yield a legal NaN-boxed value.
      */
      LSUOpType.lw   -> Mux(fpWen, FPU.box(rdata, FPU.S), SignExt(rdata(31, 0), XLEN)),
      LSUOpType.ld   -> Mux(fpWen, FPU.box(rdata, FPU.D), SignExt(rdata(63, 0), XLEN)),
      LSUOpType.lbu  -> ZeroExt(rdata(7, 0) , XLEN),
      LSUOpType.lhu  -> ZeroExt(rdata(15, 0), XLEN),
      LSUOpType.lwu  -> ZeroExt(rdata(31, 0), XLEN),
    ))
  }
}

class LqEnqIO(implicit p: Parameters) extends XSBundle {
  val canAccept = Output(Bool())
  val sqCanAccept = Input(Bool())
  val needAlloc = Vec(exuParameters.LsExuCnt, Input(Bool()))
  val req = Vec(exuParameters.LsExuCnt, Flipped(ValidIO(new MicroOp)))
  val resp = Vec(exuParameters.LsExuCnt, Output(new LqPtr))
  val reqNum = Input((UInt(exuParameters.LsExuCnt.W)))
}

class LqPaddrWriteBundle(implicit p: Parameters) extends XSBundle {
  val paddr = Output(UInt(PAddrBits.W))
  val lqIdx = Output(new LqPtr)
}

class LqTriggerIO(implicit p: Parameters) extends XSBundle {
  val hitLoadAddrTriggerHitVec = Input(Vec(TriggerNum, Bool()))
  val lqLoadAddrTriggerHitVec = Output(Vec(TriggerNum, Bool()))
}

// Load Queue
class LoadQueue(implicit p: Parameters) extends XSModule
  with HasDCacheParameters
  with HasCircularQueuePtrHelper
  with HasLoadHelper
  with HasPerfEvents
  with HasPerfLogging
{
  val io = IO(new Bundle() {
    val enq = new LqEnqIO
    val brqRedirect = Flipped(ValidIO(new Redirect))
    val loadIn = Vec(LoadPipelineWidth, Flipped(Valid(new LqWriteBundle)))  //from loadUnit S2
    val storeIn = Vec(StorePipelineWidth, Flipped(Valid(new LsPipelineBundle)))
    val stLdViolationQuery = Vec(StorePipelineWidth, Flipped(Valid(new storeRAWQueryBundle)))
    val s2_load_data_forwarded = Vec(LoadPipelineWidth, Input(Bool()))
    val loadViolationQuery = Vec(LoadPipelineWidth, Flipped(new LoadViolationQueryIO))
    val lqSafeDeq = Input(new RobPtr)
    val robHead = Input(new RobPtr)
    val rollback = Output(Valid(new Redirect)) // replay now starts from load instead of store
    val release = Flipped(ValidIO(new Release))
    val uncache = new UncacheWordIO
    val exceptionAddr = new ExceptionAddrIO
    val lqFull = Output(Bool())
    val lqCancelCnt = Output(UInt(log2Up(LoadQueueSize + 1).W))
    val trigger = Vec(LoadPipelineWidth, new LqTriggerIO)
    val lqDeq = Output(UInt(log2Up(CommitWidth + 1).W))
    val replayQEnq = Vec(LoadPipelineWidth, Flipped(DecoupledIO(new LoadToReplayQueueBundle)))
    val ldStop = Output(Bool())
    val replayQIssue = Vec(LoadPipelineWidth, DecoupledIO(new ReplayQueueIssueBundle))
    val replayQFull = Output(Bool())
    val mmioWb = DecoupledIO(new ExuOutput)
    val stAddrReadyPtr = Input(new SqPtr)
    val stAddrAllReady = Input(Bool())
    val tlbWakeup = Flipped(ValidIO(new LoadTLBWakeUpBundle))
    val debug_deqPtr = Input(new RobPtr)
    val debug_enqPtr = Input(new RobPtr)

    //RAW
    val loadEnqRAW = Vec(LoadPipelineWidth, Flipped(new LoadEnqRAWBundle)) //Load S2 enq
  })

  private val replayQueue = Module(new LoadReplayQueue(enablePerf = true))
  replayQueue.io.redirect := io.brqRedirect
  replayQueue.io.enq <> io.replayQEnq
  replayQueue.io.replayQIssue(0).ready := true.B
  replayQueue.io.replayQIssue(1).ready := true.B
  replayQueue.io.tlbWakeup := io.tlbWakeup
  io.ldStop := replayQueue.io.ldStop
  io.replayQIssue <> replayQueue.io.replayQIssue
  io.replayQFull := replayQueue.io.replayQFull
  replayQueue.io.degbugInfo.debug_deqPtr := io.debug_deqPtr
  replayQueue.io.degbugInfo.debug_enqPtr := io.debug_enqPtr
  //  val debugReplayQ = Seq.fill(LoadPipelineWidth)(RegInit(0.U.asTypeOf(new ReplayQueueIssueBundle)))
  //  for(i <- 0 until LoadPipelineWidth){
  //    debugReplayQ(i) <> replayQueue.io.replayQIssue(i).bits
  //    dontTouch(debugReplayQ(i))
  //  }
  XSPerfAccumulate("replayq", replayQueue.io.replayQIssue(0).fire || replayQueue.io.replayQIssue(1).fire)
  println("LoadQueue: size:" + LoadQueueSize)

  private val rawQueue = Module(new LoadRAWQueue)
  rawQueue.io.redirect := io.brqRedirect
  rawQueue.io.stAddrReadyPtr := io.stAddrReadyPtr
  rawQueue.io.storeQuery := io.stLdViolationQuery
  rawQueue.io.loadEnq <> io.loadEnqRAW
  rawQueue.io.stAddrAllReady := io.stAddrAllReady
  io.rollback := rawQueue.io.rollback

  private val uop = Reg(Vec(LoadQueueSize, new MicroOp))
  private val dataModule = Module(new LoadQueueDataWrapper(LoadQueueSize, wbNumRead = LoadPipelineWidth, wbNumWrite = LoadPipelineWidth))
  dataModule.io := DontCare

  private val vaddrTriggerResultModule = Module(new vaddrTriggerResultDataModule(Vec(TriggerNum, Bool()), LoadQueueSize, numRead = LoadPipelineWidth, numWrite = LoadPipelineWidth, "LqTrigger"))
  vaddrTriggerResultModule.io := DontCare

  private val exceptionGen = Module(new LSQExceptionGen(LoadPipelineWidth, FuConfigs.lduCfg))
  exceptionGen.io.redirect := io.brqRedirect
  private val exceptionInfo = exceptionGen.io.out

  private val allocated = RegInit(VecInit(List.fill(LoadQueueSize)(false.B))) // lq entry has been allocated
  private val readyToLeave = RegInit(VecInit(List.fill(LoadQueueSize)(false.B))) // lq entry is ready to leave
  private val datavalid = RegInit(VecInit(List.fill(LoadQueueSize)(false.B))) // data is valid
  private val writebacked = RegInit(VecInit(List.fill(LoadQueueSize)(false.B))) // inst has been writebacked to CDB
  private val released = RegInit(VecInit(List.fill(LoadQueueSize)(false.B))) // load data has been released by dcache
  private val error = RegInit(VecInit(List.fill(LoadQueueSize)(false.B))) // load data has been corrupted
  private val miss = RegInit(VecInit(List.fill(LoadQueueSize)(false.B))) // load inst missed, waiting for miss queue to accept miss request
  private val pending = RegInit(VecInit(List.fill(LoadQueueSize)(false.B))) // mmio pending: inst is an mmio inst, it will not be executed until it reachs the end of rob

  private val debug_mmio = RegInit(VecInit(List.fill(LoadQueueSize)(false.B))) // mmio: inst is an mmio inst
  private val debug_paddr = RegInit(VecInit(List.fill(LoadQueueSize)(0.U(PAddrBits.W)))) // mmio: inst is an mmio inst

  private val enqPtrExt = RegInit(VecInit((0 until io.enq.req.length).map(_.U.asTypeOf(new LqPtr))))
  private val deqPtrExt = RegInit(0.U.asTypeOf(new LqPtr))
  private val deqPtrExtNext = Wire(new LqPtr)
  dontTouch(deqPtrExt)

  private val enqPtr = enqPtrExt(0).value
  private val deqPtr = deqPtrExt.value

  private val validCount = distanceBetween(enqPtrExt(0), deqPtrExt)
  private val allowEnqueue = validCount <= (LoadQueueSize - 2).U

  private val deqMask = UIntToMask(deqPtr, LoadQueueSize)
  private val enqMask = UIntToMask(enqPtr, LoadQueueSize)

  private val release1cycle = io.release
  private val release2cycle_valid = RegNext(io.release.valid)
  private val release2cycle_paddr = RegEnable(io.release.bits.paddr, io.release.valid)
  private val release2cycle_paddr_dup_lsu = RegEnable(io.release.bits.paddr, io.release.valid)

  /**
    * Enqueue at dispatch
    *
    * Currently, LoadQueue only allows enqueue when #emptyEntries > EnqWidth
    */
  io.enq.canAccept := allowEnqueue

  //for uop(index).robidx clock-gating
  private val wen_robIdx = Wire(Vec(LoadQueueSize, Bool()))
  private val canEnqueue = io.enq.req.map(_.valid)
  private val enqCancel = io.enq.req.map(_.bits.robIdx.needFlush(io.brqRedirect))

  (0 until LoadQueueSize).foreach(i => {
    val req_valid = Wire(Vec(exuParameters.LsExuCnt, Bool()))
    (0 until exuParameters.LsExuCnt).foreach(j => {
      req_valid(j) := canEnqueue(j) && !enqCancel(j) && (io.enq.req(j).bits.lqIdx.value === i.U)
    })
    wen_robIdx(i) := req_valid.asUInt.orR
    val w_addr = OHToUInt(req_valid)
    val w_data = io.enq.req(w_addr).bits.robIdx
    when(wen_robIdx(i)) {
      uop(i).robIdx := w_data
    }
  })

  for (i <- 0 until io.enq.req.length) {
    val offset = if (i == 0) 0.U else PopCount(io.enq.needAlloc.take(i))
    val lqIdx = enqPtrExt(offset)
    val index = io.enq.req(i).bits.lqIdx.value
    when (canEnqueue(i) && !enqCancel(i)) {
      allocated(index) := true.B
      readyToLeave(index) := false.B
      datavalid(index) := false.B
      writebacked(index) := false.B
      released(index) := false.B
      miss(index) := false.B
      pending(index) := false.B
      error(index) := false.B
      XSError(!io.enq.canAccept || !io.enq.sqCanAccept, s"must accept $i\n")
      XSError(index =/= lqIdx.value, s"must be the same entry $i\n")
    }
    io.enq.resp(i) := lqIdx
  }
  XSDebug(p"(ready, valid): ${io.enq.canAccept}, ${Binary(Cat(io.enq.req.map(_.valid)))}\n")


  private val lastCycleRedirect_valid = RegNext(io.brqRedirect.valid)
  private val lastCycleRedirect_bits = RegEnable(io.brqRedirect.bits, io.brqRedirect.valid)
  private val lastlastCycleRedirect_valid = RegNext(lastCycleRedirect_valid)
  private val lastlastCycleRedirect_bits = RegEnable(lastCycleRedirect_bits, lastCycleRedirect_valid)

  private val lastCycleRedirect = Wire(io.brqRedirect.cloneType)
  lastCycleRedirect.valid := lastCycleRedirect_valid
  lastCycleRedirect.bits := lastCycleRedirect_bits

  private val lastlastCycleRedirect = Wire(io.brqRedirect.cloneType)
  lastlastCycleRedirect.valid := lastlastCycleRedirect_valid
  lastlastCycleRedirect.bits := lastlastCycleRedirect_bits
  /**
    * Writeback load from load units
    *
    * Most load instructions writeback to regfile at the same time.
    * However,
    *   (1) For an mmio instruction with exceptions, it writes back to ROB immediately.
    *   (2) For an mmio instruction without exceptions, it does not write back.
    * The mmio instruction will be sent to lower level when it reaches ROB's head.
    * After uncache response, it will write back through arbiter with loadUnit.
    *   (3) For cache misses, it is marked miss and sent to dcache later.
    * After cache refills, it will write back through arbiter with loadUnit.
    */
  for (i <- 0 until LoadPipelineWidth) {
    dataModule.io.wb.wen(i) := false.B
    dataModule.io.paddr.wen(i) := false.B
    vaddrTriggerResultModule.io.wen(i) := false.B
    val loadWbIndex = io.loadIn(i).bits.uop.lqIdx.value

    // most lq status need to be updated immediately after load writeback to lq
    // flag bits in lq needs to be updated accurately
    when(io.loadIn(i).fire) {
      when(io.loadIn(i).bits.miss) {
        XSInfo(io.loadIn(i).valid, "load miss write to lq idx %d pc 0x%x vaddr %x paddr %x mask %x forwardData %x forwardMask: %x mmio %x\n",
          io.loadIn(i).bits.uop.lqIdx.asUInt,
          io.loadIn(i).bits.uop.cf.pc,
          io.loadIn(i).bits.vaddr,
          io.loadIn(i).bits.paddr,
          io.loadIn(i).bits.mask,
          io.loadIn(i).bits.forwardData.asUInt,
          io.loadIn(i).bits.forwardMask.asUInt,
          io.loadIn(i).bits.mmio
        )
      }.otherwise {
        XSInfo(io.loadIn(i).valid, "load hit write to cbd lqidx %d pc 0x%x vaddr %x paddr %x mask %x forwardData %x forwardMask: %x mmio %x\n",
        io.loadIn(i).bits.uop.lqIdx.asUInt,
        io.loadIn(i).bits.uop.cf.pc,
        io.loadIn(i).bits.vaddr,
        io.loadIn(i).bits.paddr,
        io.loadIn(i).bits.mask,
        io.loadIn(i).bits.forwardData.asUInt,
        io.loadIn(i).bits.forwardMask.asUInt,
        io.loadIn(i).bits.mmio
      )}
      //EnableFastForward is false
      datavalid(loadWbIndex) := (!io.loadIn(i).bits.miss || io.s2_load_data_forwarded(i)) &&
        !io.loadIn(i).bits.mmio // mmio data is not valid until we finished uncache access

      writebacked(loadWbIndex) := !io.loadIn(i).bits.miss && !io.loadIn(i).bits.mmio

      debug_mmio(loadWbIndex) := io.loadIn(i).bits.mmio
      debug_paddr(loadWbIndex) := io.loadIn(i).bits.paddr

      val dcacheMissed = io.loadIn(i).bits.miss && !io.loadIn(i).bits.mmio
      if(EnableFastForward){
        miss(loadWbIndex) := dcacheMissed && !io.s2_load_data_forwarded(i)
      } else {
        miss(loadWbIndex) := dcacheMissed && !io.s2_load_data_forwarded(i)
      }
      pending(loadWbIndex) := io.loadIn(i).bits.mmio
      released(loadWbIndex) := release2cycle_valid &&
        io.loadIn(i).bits.paddr(PAddrBits-1, DCacheLineOffset) === release2cycle_paddr(PAddrBits-1, DCacheLineOffset) ||
        release1cycle.valid &&
        io.loadIn(i).bits.paddr(PAddrBits-1, DCacheLineOffset) === release1cycle.bits.paddr(PAddrBits-1, DCacheLineOffset)
    }

    // dirty code to reduce load_s2.valid fanout
    when(io.loadIn(i).bits.lq_data_wen_dup(0)){
      val loadWbData = Wire(new LQDataEntry)
      loadWbData.paddr := io.loadIn(i).bits.paddr
      loadWbData.mask := io.loadIn(i).bits.mask
      loadWbData.data := io.loadIn(i).bits.forwardData.asUInt // fwd data
      loadWbData.fwdMask := io.loadIn(i).bits.forwardMask
      dataModule.io.wbWrite(i, loadWbIndex, loadWbData)
      dataModule.io.wb.wen(i) := true.B
    }
    // dirty code for load instr
    when(io.loadIn(i).bits.lq_data_wen_dup(1)){
      uop(loadWbIndex).pdest := io.loadIn(i).bits.uop.pdest
      uop(loadWbIndex).loadStoreEnable := io.loadIn(i).bits.uop.loadStoreEnable
      uop(loadWbIndex).uopIdx := io.loadIn(i).bits.uop.uopIdx
      uop(loadWbIndex).uopNum := io.loadIn(i).bits.uop.uopNum
      uop(loadWbIndex).segIdx := io.loadIn(i).bits.uop.segIdx
      uop(loadWbIndex).mergeIdx := io.loadIn(i).bits.uop.mergeIdx
    }
    when(io.loadIn(i).bits.lq_data_wen_dup(2)){
      uop(loadWbIndex).cf := io.loadIn(i).bits.uop.cf
    }
    when(io.loadIn(i).bits.lq_data_wen_dup(3)){
      uop(loadWbIndex).ctrl := io.loadIn(i).bits.uop.ctrl
    }
    when(io.loadIn(i).bits.lq_data_wen_dup(4)){
      uop(loadWbIndex).debugInfo := io.loadIn(i).bits.uop.debugInfo
      uop(loadWbIndex).vCsrInfo := io.loadIn(i).bits.uop.vCsrInfo
      uop(loadWbIndex).vctrl := io.loadIn(i).bits.uop.vctrl
    }
    when(io.loadIn(i).bits.lq_data_wen_dup(5)){
      vaddrTriggerResultModule.io.waddr(i) := loadWbIndex
      vaddrTriggerResultModule.io.wdata(i) := io.trigger(i).hitLoadAddrTriggerHitVec
      vaddrTriggerResultModule.io.wen(i) := true.B
    }
  }

  /**
    * Load commits
    *
    * When load commited, mark it as !allocated and move deqPtrExt forward.
    */
  //when scalar load is committed and index-ordered instruction is committed, allocated flag is false
  private val readyToDeq = Reg(Vec(LoadQueueSize, Bool()))
  for (i <- 0 until LoadQueueSize) {
    readyToDeq(i) := readyToLeave(i) & allocated(i) & writebacked(i) & !io.brqRedirect.valid
  }
  private val deqWindow = Seq.tabulate(CommitWidth)(idx => (deqPtrExt + idx.U).value)
  private val deqVec = deqWindow.map(addr => readyToDeq(addr) & !io.brqRedirect.valid)
  private val deqBlocked = deqVec.map(!_) :+ true.B
  private val deqNum = PriorityEncoder(deqBlocked)
  io.lqDeq := RegNext(deqNum)
  uop.zip(readyToLeave).zipWithIndex.foreach({case((u, r), idx) =>
    val updateReadyToLeave = Wire(Bool())
    updateReadyToLeave := u.robIdx <= io.lqSafeDeq && allocated(idx) && !r
    when(updateReadyToLeave){
      r := true.B
    }
  })
  (0 until CommitWidth).map(i => {
    when(deqNum > i.U){
      allocated((deqPtrExt+i.U).value) := false.B
      XSError(!allocated((deqPtrExt+i.U).value), s"why release invalid entry $i?\n")
    }
  })


  when(io.rollback.valid) {
    // XSDebug("Mem rollback: pc %x robidx %d\n", io.rollback.bits.cfi, io.rollback.bits.robIdx.asUInt)
  }

  // Load-Load Memory violation query
  (0 until LoadPipelineWidth).foreach(i => {
    val ldldViolationReqValid = io.loadViolationQuery(i).s1_req.valid
    dataModule.io.release_violation(i).paddr := io.loadViolationQuery(i).s1_req.bits.paddr
    io.loadViolationQuery(i).s1_req.ready := true.B
    io.loadViolationQuery(i).s2_resp.valid := RegNext(io.loadViolationQuery(i).s1_req.fire)
    // Generate real violation mask
    // Note that we use UIntToMask.rightmask here
    val startIndex = io.loadViolationQuery(i).s1_req.bits.uop.lqIdx.value
    val lqIdxMask = UIntToMask(startIndex, LoadQueueSize)
    val xorMask = lqIdxMask ^ enqMask
    val sameFlag = io.loadViolationQuery(i).s1_req.bits.uop.lqIdx.flag === enqPtrExt(0).flag
    val ldToEnqPtrMask = Mux(sameFlag, xorMask, ~xorMask)
    val ldld_violation_mask_gen_1 = WireInit(VecInit((0 until LoadQueueSize).map(j => {
      ldToEnqPtrMask(j) && // the load is younger than current load
      allocated(j) && // entry is valid
      released(j) && // cacheline is released
      (datavalid(j) || miss(j)) // paddr is valid
    })))
    val ldld_violation_mask_gen_2 = WireInit(VecInit((0 until LoadQueueSize).map(j => {
      dataModule.io.release_violation(i).match_mask(j)// addr match
      // addr match result is slow to generate, we RegNext() it
    })))
//    val ldld_violation_mask = RegNext(ldld_violation_mask_gen_1).asUInt & RegNext(ldld_violation_mask_gen_2).asUInt
    val llvMaskReg1 = RegEnable(ldld_violation_mask_gen_1, ldldViolationReqValid)
    val llvMaskReg2 = RegEnable(ldld_violation_mask_gen_2, ldldViolationReqValid)
    val llvMask = llvMaskReg1.asUInt & llvMaskReg2.asUInt
    dontTouch(llvMask)
    dontTouch(llvMaskReg1)
    dontTouch(llvMaskReg2)
    dontTouch(ldld_violation_mask_gen_1)
    dontTouch(ldld_violation_mask_gen_2)
    llvMask.suggestName("ldldViolationMask_" + i)
    io.loadViolationQuery(i).s2_resp.bits.have_violation := llvMask.orR
  })

  // "released" flag update
  //
  // When io.release.valid (release1cycle.valid), it uses the last ld-ld paddr cam port to
  // update release flag in 1 cycle

  when(release1cycle.valid){
    // Take over ld-ld paddr cam port
    dataModule.io.release_violation.takeRight(1)(0).paddr := release1cycle.bits.paddr
    io.loadViolationQuery.takeRight(1)(0).s1_req.ready := false.B
  }

  when(release2cycle_valid){
    // If a load comes in that cycle, we can not judge if it has ld-ld violation
    // We replay that load inst from RS
    io.loadViolationQuery.map(i => i.s1_req.ready :=
      // use lsu side release2cycle_dup_lsu paddr for better timing
      !(i.s1_req.bits.paddr(PAddrBits-1, DCacheLineOffset) === release2cycle_paddr_dup_lsu(PAddrBits-1, DCacheLineOffset))
    )
    // io.loadViolationQuery.map(i => i.req.ready := false.B) // For better timing
  }

  private val release_flag = WireInit(VecInit((0 until LoadQueueSize).map(i =>{
    RegNext(dataModule.io.release_violation.takeRight(1)(0).match_mask(i) &&
      allocated(i) &&
      datavalid(i) &&
      release1cycle.valid)
  })).asUInt)
  dontTouch(release_flag)
  (0 until LoadQueueSize).map(i => {
    when(release_flag(i)){
      // Note: if a load has missed in dcache and is waiting for refill in load queue,
      // its released flag still needs to be set as true if addr matches.
      released(i) := true.B
    }
  })

  /**
    * Memory mapped IO / other uncached operations
    *
    * States:
    * (1) writeback from store units: mark as pending
    * (2) when they reach ROB's head, they can be sent to uncache channel
    * (3) response from uncache channel: mark as datavalid
    * (4) writeback to ROB (and other units): mark as writebacked
    * (5) ROB commits the instruction: same as normal instructions
    */
  //(2) when they reach ROB's head, they can be sent to uncache channel
  dataModule.io.uncache.ren := false.B

  private val lqTailMmioPending = WireInit(pending(deqPtr))
  private val lqTailAllocated = WireInit(allocated(deqPtr))
  private val ldTailReadyToLeave = WireInit(readyToLeave(deqPtr))
  private val ldTailWritebacked = WireInit(writebacked(deqPtr))
  private val pendingHead = uop(deqPtrExt.value).robIdx === io.robHead
  val s_idle :: s_req :: s_resp :: s_wait :: Nil = Enum(4)
  private val uncache_Order_State = RegInit(s_idle)
  switch(uncache_Order_State) {
    is(s_idle) {
      when(RegNext(ldTailReadyToLeave && lqTailMmioPending && lqTailAllocated && pendingHead && !ldTailWritebacked)) {
        dataModule.io.uncache.ren := true.B
        uncache_Order_State := s_req
      }
    }
    is(s_req) {
      when(io.uncache.req.fire) {
        uncache_Order_State := s_resp
      }
    }
    is(s_resp) {
      when(io.uncache.resp.fire) {
        uncache_Order_State := s_wait
      }
    }
    is(s_wait) {
      when(RegNext(ldTailWritebacked)) {
        uncache_Order_State := s_idle // ready for next mmio
      }
    }
  }

  //S0
  private val mmioLdWbSel = Wire(UInt(log2Up(LoadQueueSize).W))
  private val mmioLdWbSelV = Wire(Bool())
  mmioLdWbSel := deqPtrExt.value
  mmioLdWbSelV := !writebacked(mmioLdWbSel) && !(io.mmioWb.fire) && uncache_Order_State === s_wait
  dataModule.io.wb.raddr(0) := mmioLdWbSel


  //S1
  private val s1_mmioLdWbSel = RegNext(mmioLdWbSel)
  private val s1_mmioLdWbSelV = RegNext(mmioLdWbSelV, false.B)
  private val s1_mmioData = dataModule.io.wb.rdata(0).data
  private val s1_mmioPaddr = dataModule.io.wb.rdata(0).paddr
  private val s1_mmioUop = uop(s1_mmioLdWbSel)

  private val s1_mmioDataSel = LookupTree(s1_mmioPaddr(2,0), List(
    "b000".U -> s1_mmioData(63, 0),
    "b001".U -> s1_mmioData(63, 8),
    "b010".U -> s1_mmioData(63, 16),
    "b011".U -> s1_mmioData(63, 24),
    "b100".U -> s1_mmioData(63, 32),
    "b101".U -> s1_mmioData(63, 40),
    "b110".U -> s1_mmioData(63, 48),
    "b111".U -> s1_mmioData(63, 56)
  ))

  private val s1_mmioDataPartialLoad = rdataHelper(s1_mmioUop, s1_mmioDataSel)

  private val defaultEVec = Wire(ExceptionVec())
  defaultEVec.foreach(_ := false.B)
  private val excptCond = exceptionInfo.valid && s1_mmioUop.robIdx === exceptionInfo.bits.robIdx && s1_mmioUop.uopIdx === exceptionInfo.bits.uopIdx

  io.mmioWb.valid := s1_mmioLdWbSelV && !uop(deqPtrExt.value).robIdx.needFlush(lastCycleRedirect)
  io.mmioWb.bits := DontCare
  io.mmioWb.bits.uop := s1_mmioUop
  io.mmioWb.bits.uop.cf.exceptionVec := Mux(excptCond, exceptionInfo.bits.eVec, defaultEVec)
  io.mmioWb.bits.data := s1_mmioDataPartialLoad
  io.mmioWb.bits.redirectValid := false.B
  io.mmioWb.bits.debug.isMMIO := debug_mmio(s1_mmioLdWbSel)
  io.mmioWb.bits.debug.paddr := debug_paddr(s1_mmioLdWbSel)

  when(io.mmioWb.fire){
    writebacked(s1_mmioLdWbSel) := true.B
  }

  io.uncache.req.valid := uncache_Order_State === s_req

  dataModule.io.uncache.raddr := deqPtrExtNext.value  //todo

  io.uncache.req.bits.cmd  := MemoryOpConstants.M_XRD
  io.uncache.req.bits.addr := dataModule.io.uncache.rdata.paddr
  io.uncache.req.bits.data := dataModule.io.uncache.rdata.data
  io.uncache.req.bits.mask := dataModule.io.uncache.rdata.mask
  io.uncache.req.bits.robIdx := DontCare
  io.uncache.req.bits.id   := DontCare
  io.uncache.req.bits.instrtype := DontCare

  io.uncache.resp.ready := true.B

  when (io.uncache.req.fire) {
    pending(deqPtr) := false.B

    XSDebug("uncache req: pc %x addr %x data %x op %x mask %x\n",
      uop(deqPtr).cf.pc,
      io.uncache.req.bits.addr,
      io.uncache.req.bits.data,
      io.uncache.req.bits.cmd,
      io.uncache.req.bits.mask
    )
  }

  // (3) response from uncache channel: mark as datavalid
  dataModule.io.uncache.wen := false.B
  when(io.uncache.resp.fire){
    datavalid(deqPtr) := true.B
    dataModule.io.uncacheWrite(deqPtr, io.uncache.resp.bits.data(XLEN-1, 0))
    dataModule.io.uncache.wen := true.B

//    XSDebug("uncache resp: data %x\n", io.dcache.bits.data)
  }

  // Read vaddr for mem exception
  // no inst will be commited 1 cycle before tval update
  exceptionGen.io.in.zipWithIndex.foreach({ case (d, i) =>
    val validCond = io.loadIn(i).valid && !io.loadIn(i).bits.uop.robIdx.needFlush(io.brqRedirect)
    val writebackCond = !io.loadIn(i).bits.miss && !io.loadIn(i).bits.mmio
    val ffIgnoreCond = io.loadIn(i).bits.uop.vctrl.ff && io.loadIn(i).bits.uop.segIdx =/= 0.U && writebackCond
    d.bits.robIdx := RegEnable(io.loadIn(i).bits.uop.robIdx, validCond)
    d.bits.vaddr := RegEnable(io.loadIn(i).bits.vaddr, validCond)
    d.bits.eVec := RegEnable(io.loadIn(i).bits.uop.cf.exceptionVec, validCond)
    d.bits.uopIdx := RegEnable(io.loadIn(i).bits.uop.uopIdx, validCond)
    d.valid := RegNext(validCond & !ffIgnoreCond, false.B)
  })
  private val mmioEvec = Wire(ExceptionVec())
  mmioEvec.foreach(_ := false.B)
  mmioEvec(loadAccessFault) := true.B
  exceptionGen.io.mmioUpdate.valid := io.uncache.resp.fire && io.uncache.resp.bits.error
  exceptionGen.io.mmioUpdate.bits.eVec := mmioEvec
  exceptionGen.io.mmioUpdate.bits.robIdx := io.robHead
  exceptionGen.io.mmioUpdate.bits.vaddr := dataModule.io.uncache.rdata.paddr
  exceptionGen.io.mmioUpdate.bits.uopIdx := uop(deqPtr).uopIdx

//  private val ffCleanConds = io.ldout.map(lo => {
//    val wbCond = lo.fire && exceptionInfo.valid
//    val excptHitCond = lo.bits.uop.uopIdx === exceptionInfo.bits.uopIdx && lo.bits.uop.robIdx === exceptionInfo.bits.robIdx
//    val ffIgnoreCond = lo.bits.uop.vctrl.ff && lo.bits.uop.segIdx =/= 0.U
//    wbCond && excptHitCond && ffIgnoreCond
//  })
//  exceptionGen.io.clean := ffCleanConds.reduce(_ || _)
  exceptionGen.io.clean := false.B  //todo
  io.exceptionAddr.vaddr := exceptionInfo.bits.vaddr

  (0 until LoadPipelineWidth).foreach(i => {
    io.trigger(i).lqLoadAddrTriggerHitVec := DontCare //todo
  })

  // misprediction recovery / exception redirect
  // invalidate lq term using robIdx
  private val needCancel = Wire(Vec(LoadQueueSize, Bool()))
  for (i <- 0 until LoadQueueSize) {
    needCancel(i) := uop(i).robIdx.needFlush(io.brqRedirect) && allocated(i)
    when (needCancel(i)) {
      allocated(i) := false.B
    }
  }

  private val splitNum = 5
  require(LoadQueueSize % splitNum == 0)
  private val needCancelCntSub = Seq.fill(splitNum)(RegInit(0.U(log2Up(LoadQueueSize).W)))

  (0 until splitNum).foreach({ case i => {
    needCancelCntSub(i) := PopCount(needCancel.asUInt((i + 1) * (LoadQueueSize / splitNum) - 1, i * (LoadQueueSize / splitNum)))
  }})
  private val needCancelCnt = needCancelCntSub.reduce(_ + _)
  private val lastCycleCancelCount = needCancelCnt


  /**
    * update pointers
    */
  private val lastEnqCancel = PopCount(RegNext(VecInit(canEnqueue.zip(enqCancel).map(x => x._1 && x._2))))
  private val enqNumber = Mux(io.enq.canAccept && io.enq.sqCanAccept, PopCount(io.enq.req.map(_.valid)), 0.U)
  private val enqNumber_enq = Mux(io.enq.canAccept && io.enq.sqCanAccept, io.enq.reqNum, 0.U)

  XSError(enqNumber =/= enqNumber_enq,"enqNumber and enqNumber_enq must be equal\n")

  when (lastCycleRedirect.valid) {
    // we recover the pointers in the next cycle after redirect
    enqPtrExt := VecInit(enqPtrExt.map(_ - (lastCycleCancelCount + lastEnqCancel)))
  }.otherwise {
    enqPtrExt := VecInit(enqPtrExt.map(_ + enqNumber_enq))
  }

  deqPtrExtNext := deqPtrExt + deqNum
  deqPtrExt := deqPtrExtNext

  io.lqCancelCnt := RegNext(lastCycleCancelCount + lastEnqCancel)

  /**
    * misc
    */
  // perf counter
  QueuePerf(LoadQueueSize, validCount, !allowEnqueue)
  io.lqFull := !allowEnqueue
  XSPerfAccumulate("rollback", io.rollback.valid) // rollback redirect generated
  XSPerfAccumulate("mmioCycle", uncache_Order_State =/= s_idle) // lq is busy dealing with uncache req
  XSPerfAccumulate("mmioCnt", io.uncache.req.fire)
//  XSPerfAccumulate("refill", io.dcache.valid)
//  XSPerfAccumulate("writeback_success", PopCount(VecInit(io.ldout.map(i => i.fire))))
//  XSPerfAccumulate("writeback_blocked", PopCount(VecInit(io.ldout.map(i => i.valid && !i.ready))))
  XSPerfAccumulate("utilization_miss", PopCount((0 until LoadQueueSize).map(i => allocated(i) && miss(i))))

  val perfValidCount = RegNext(validCount)

  val perfEvents = Seq(
    ("rollback         ", io.rollback.valid),
    ("mmioCycle        ", uncache_Order_State =/= s_idle),
    ("mmio_Cnt         ", io.uncache.req.fire),
//    ("refill           ", io.dcache.valid),
//    ("writeback_success", PopCount(VecInit(io.ldout.map(i => i.fire)))),
//    ("writeback_blocked", PopCount(VecInit(io.ldout.map(i => i.valid && !i.ready)))),
    ("ltq_1_4_valid    ", (perfValidCount < (LoadQueueSize.U/4.U))),
    ("ltq_2_4_valid    ", (perfValidCount > (LoadQueueSize.U/4.U)) & (perfValidCount <= (LoadQueueSize.U/2.U))),
    ("ltq_3_4_valid    ", (perfValidCount > (LoadQueueSize.U/2.U)) & (perfValidCount <= (LoadQueueSize.U*3.U/4.U))),
    ("ltq_4_4_valid    ", (perfValidCount > (LoadQueueSize.U*3.U/4.U)))
  )
  generatePerfEvent()

  // debug info
  XSDebug("enqPtrExt %d:%d deqPtrExt %d:%d\n", enqPtrExt(0).flag, enqPtr, deqPtrExt.flag, deqPtr)

  def PrintFlag(flag: Bool, name: String): Unit = {
    when(flag) {
      XSDebug(false, true.B, name)
    }.otherwise {
      XSDebug(false, true.B, " ")
    }
  }

  for (i <- 0 until LoadQueueSize) {
    XSDebug(i + " pc %x pa %x ", uop(i).cf.pc, debug_paddr(i))
    PrintFlag(allocated(i), "a")
    PrintFlag(allocated(i) && datavalid(i), "v")
    PrintFlag(allocated(i) && writebacked(i), "w")
    PrintFlag(allocated(i) && miss(i), "m")
    PrintFlag(allocated(i) && pending(i), "p")
    XSDebug(false, true.B, "\n")
  }

}
