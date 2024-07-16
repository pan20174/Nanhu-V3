package xiangshan.backend
import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import xiangshan._
import xiangshan.backend.decode.FusionDecodeInfo
import xs.utils.CircularQueuePtr
import xiangshan.backend.rob.RobPtr
import xs.utils._

class RouterQueueEntry(implicit p: Parameters) extends XSBundle {
  val uop = new MicroOp
  val robidx = new RobPtr
  def initMicroOp: MicroOp = {
    val umop = Wire(new MicroOp)
    umop := uop
    umop.srcState(0) := DontCare
    umop.srcState(1) := DontCare
    umop.srcState(2) := DontCare
    umop.robIdx := robidx
    dontTouch(umop.robIdx)
    umop.debugInfo := DontCare
    umop.lqIdx := DontCare
    umop.sqIdx := DontCare
    umop.lpv := DontCare
    umop.vCsrInfo := DontCare
    umop.vctrl := DontCare
    umop.vctrl.ordered := false.B
    umop.uopIdx := 0.U
    umop.vmState := DontCare
    umop.isTail := DontCare
    umop.isPrestart := DontCare
    umop.vtypeRegIdx := DontCare
    umop.uopNum := 0.U
    umop.vm := DontCare
    umop.canRename := DontCare
    umop.mergeIdx := DontCare
    umop.loadStoreEnable := DontCare
    umop.segIdx := 0.U
    umop.elmIdx := 0.U
    umop
  }
}

class RouterQueue(vecLen:Int, outNum:Int, size:Int)(implicit p: Parameters) extends XSModule with HasCircularQueuePtrHelper {
  val io = IO(new Bundle {
    val in = Flipped(Vec(vecLen, Decoupled(new CfCtrl)))
    val out = Vec(outNum, Vec(vecLen, Decoupled(new MicroOp)))
    val allowOut = Output(Vec(outNum, Bool()))
    val redirect = Flipped(ValidIO(new Redirect))
    val flush = Input(Bool())
    val robempty = Input(Bool())
    val singleStep = Input(Bool())
  })

  //reg
  private class RouterQueuePtr extends CircularQueuePtr[RouterQueuePtr](size)

  private val dataModule = Reg(Vec(size, new RouterQueueEntry))

  private val enqPtrVec = Seq.tabulate(vecLen)(i => RegInit(i.U.asTypeOf(new RouterQueuePtr)))
  private val enqPtr = enqPtrVec.head
  private val enqPtrVecNext = enqPtrVec.map(WireInit(_))

  private val deqPtrVec = Seq.tabulate(vecLen)(i => RegInit(i.U.asTypeOf(new RouterQueuePtr)))
  private val deqPtr = deqPtrVec.head
  private val deqPtrVecNext = deqPtrVec.map(WireInit(_))

  private val singleStepStatus = RegInit(false.B)

  //robidx allocate
  private val validCount = PopCount(io.in.map(_.valid)) // number of instructions waiting to enter rob (from decode)
  private val hasValid = io.in.map(_.valid).reduce(_ || _)
  private val robIdxHead = RegInit(0.U.asTypeOf(new RobPtr))
  dontTouch(robIdxHead)
  dontTouch(io.redirect)
  private val lastCycleMisprediction = RegNext((io.redirect.valid) && !io.redirect.bits.flushItself())
  dontTouch(lastCycleMisprediction)
  private val allowOut = Wire(Vec(outNum, Bool()))
  allowOut.zipWithIndex.foreach({ case (a, i) =>
    a := io.out.zipWithIndex.filterNot(_._2 == i).map(_._1.head.ready).reduce(_ & _)
  })
  private val canOut = allowOut.reduce(_ && _)
  private val canAccept = io.in.map(_.ready).reduce(_ || _)
  private val robIdxHeadNext = Mux(io.redirect.valid, io.redirect.bits.robIdx, // redirect: move ptr to given rob index
    Mux(lastCycleMisprediction, robIdxHead + 1.U, // mis-predict: not flush robIdx itself
      Mux(canAccept, robIdxHead + validCount, // instructions successfully entered next stage: increase robIdx
        /* default */ robIdxHead))) // no instructions passed by this cycle: stick to old value
  dontTouch(robIdxHeadNext)
  robIdxHead := robIdxHeadNext

  //enq
  //  private val allocatePtrVec = VecInit((0 until vecLen).map(i => enqPtrVec(PopCount(io.in.map(_.fire))).value))
  io.in.zipWithIndex.zip(enqPtrVec).map { case ((in, i), enqaddr) =>
    when(in.fire && !(io.redirect.valid || io.flush)) {
      dataModule(enqaddr.value).uop.ctrl := in.bits.ctrl
      dataModule(enqaddr.value).uop.cf := in.bits.cf
      dataModule(enqaddr.value).uop.vCsrInfo := in.bits.vCsrInfo
      dataModule(enqaddr.value).uop.vctrl := in.bits.vctrl
      dataModule(enqaddr.value).robidx := robIdxHead + PopCount(io.in.take(i).map(_.valid))
      dataModule(enqaddr.value).uop.robIdx := robIdxHead + PopCount(io.in.take(i).map(_.valid))
      dataModule(enqaddr.value).initMicroOp
    }
  }
  private val enqCount    = PopCount(io.in.map(_.valid))
  private val numValidEntries = distanceBetween(enqPtr, deqPtr)
  private val allowEnqueue = RegNext(numValidEntries + enqCount <= (size - RenameWidth).U, true.B)
  // enq ready
  io.in.map(_.ready := allowEnqueue)

  // singelStepStatus
  private val inst0actualOut = io.out(0)(0).valid
  when(io.redirect.valid) {
    singleStepStatus := false.B
  }.elsewhen(io.singleStep && io.in(0).fire && inst0actualOut) {
    singleStepStatus := true.B
  }

  //deq
  private val deqEntries = WireInit(VecInit.fill(vecLen)(0.U.asTypeOf(new RouterQueueEntry)))
  deqEntries.zip(deqPtrVec).foreach({case (deqdata, addr) =>
    deqdata := dataModule(addr.value)
  })
  val isBlockBackward = VecInit(deqEntries.map(req => req.uop.ctrl.blockBackward))
  val isNoSpecExec = VecInit(deqEntries.map(req => req.uop.ctrl.noSpecExec))
  val thisIsBlocked = VecInit((0 until vecLen).map(i => {
    // for i > 0, when Rob is empty but dispatch1 have valid instructions to enqueue, it's blocked
    if (i > 0) isNoSpecExec(i) && !io.robempty
    else isNoSpecExec(i) && !io.robempty
  }))
  val nextCanOut = VecInit((0 until vecLen).map(i =>
    (!isNoSpecExec(i) && !isBlockBackward(i))
  ))
  val notBlockedByPrevious = VecInit((0 until vecLen).map(i =>
    if (i == 0) true.B
    else Cat((0 until i).map(j => nextCanOut(j))).andR
  ))
  val thisCanActualOut = (0 until RenameWidth).map(i => !thisIsBlocked(i) && notBlockedByPrevious(i))
  io.out.zipWithIndex.foreach({case(out, deq) =>
    out.zipWithIndex.zip(deqEntries).foreach({case ((o, i), deq) =>
      o.valid := thisCanActualOut(i) && (i.U < numValidEntries) && !(io.redirect.valid || io.flush)
      o.bits := deq.uop
      o.bits.robIdx := deq.robidx
      o.bits.ctrl.singleStep := io.singleStep && (if (i == 0) singleStepStatus else true.B)
    })
  })
  io.allowOut := allowOut

  //pointers update
  private val enqMask = UIntToMask(enqPtr.value, size)
  private val deqMask = UIntToMask(deqPtr.value, size)
  private val enqXorDeq = enqMask ^ deqMask
  private val validsMask = Mux(deqPtr.value < enqPtr.value || deqPtr === enqPtr, enqXorDeq, (~enqXorDeq).asUInt)
  private val redirectHits = dataModule.map(_.uop.robIdx.needFlush(io.redirect))
  private val flushVec = Cat(redirectHits.reverse)
  private val redirectMask = validsMask & flushVec
  private val flushNum = PopCount(redirectMask)
  private val actualEnqNum = Mux(io.in(0).fire, PopCount(io.in.map(_.valid)), 0.U)
  enqPtrVec.zipWithIndex.foreach({case (ptr, i) =>
    when(io.redirect.valid) {
      ptr := i.U.asTypeOf(new RouterQueuePtr)
    }.elsewhen(io.flush) {
      ptr := i.U.asTypeOf(new RouterQueuePtr)
    }.elsewhen(actualEnqNum =/= 0.U) {
      ptr := ptr + actualEnqNum
    }
  })
  deqPtrVec.zipWithIndex.foreach({ case (ptr, i) =>
    when(io.redirect.valid) {
      ptr := i.U.asTypeOf(new RouterQueuePtr)
    }.elsewhen(io.out(0).map(_.fire).reduce(_ || _)) {
      ptr := ptr + PopCount(io.out(0).map(_.fire))
    }
  })

}