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
  val fusion = new FusionDecodeInfo
  val uop = new MicroOp()
}

class RouterQueue(vecLen:Int, outNum:Int, size:Int)(implicit p: Parameters) extends XSModule with HasCircularQueuePtrHelper {
  val io = IO(new Bundle {
    val in = Flipped(Vec(vecLen, Decoupled(new CfCtrl)))
    val fusionInfoIn = Vec(DecodeWidth, Flipped(new FusionDecodeInfo))
    val out = Vec(outNum, Vec(vecLen, Decoupled(new MicroOp)))
    val allowOut = Output(Vec(outNum, Bool()))
    val redirect = Flipped(ValidIO(new Redirect))
    val robempty = Input(Bool())
    val fusionInfoOut = Vec(DecodeWidth, Flipped(new FusionDecodeInfo))
    val singleStep = Input(Bool())
  })

  //reg
  private class RouterQueuePtr extends CircularQueuePtr[RouterQueuePtr](size)

  private val dataModule = Reg(Vec(size, new RouterQueueEntry))
  private val enqPtrVec = Wire(Vec(vecLen, new RouterQueuePtr))
  private val enqPtr = enqPtrVec.head
  private val enqPtrVecNext = Wire(enqPtrVec.cloneType)

  private val deqPtrVec = Wire(Vec(vecLen, new RouterQueuePtr))
  private val deqPtrVecNext = deqPtrVec.map(WireInit(_))
  private val deqPtr = deqPtrVec.head

  private val singleStepStatus = RegInit(false.B)

  //enq
  private val validCount = PopCount(io.in.map(_.valid)) // number of instructions waiting to enter rob (from decode)
  private val robIdxHead = RegInit(0.U.asTypeOf(new RobPtr))
  private val lastCycleMisprediction = RegNext(io.redirect.valid && !io.redirect.bits.flushItself())
  private val allowOut = Wire(Vec(outNum, Bool()))
  allowOut.zipWithIndex.foreach({ case (a, i) =>
    a := io.out.zipWithIndex.filterNot(_._2 == i).map(_._1.head.ready).reduce(_ & _)
  })
  private val canOut = allowOut.reduce(_ && _)
  private val robIdxHeadNext = Mux(io.redirect.valid, io.redirect.bits.robIdx, // redirect: move ptr to given rob index
    Mux(lastCycleMisprediction, robIdxHead + 1.U, // mis-predict: not flush robIdx itself
      Mux(canOut && allowEnqueue, robIdxHead + validCount, // instructions successfully entered next stage: increase robIdx
        /* default */ robIdxHead))) // no instructions passed by this cycle: stick to old value
  robIdxHead := robIdxHeadNext
  private val allocatePtrVec = VecInit((0 until vecLen).map(i => enqPtrVec(PopCount(io.in.map(_.fire))).value))
  io.in.zipWithIndex.zip(allocatePtrVec).map { case ((in, i), enqaddr) =>
    when(in.fire) {
      dataModule(enqaddr).uop := in.bits
      dataModule(enqaddr).uop.robIdx := robIdxHead + PopCount(io.in.take(i).map(_.valid))
      dataModule(enqaddr).fusion := io.fusionInfoIn(i)
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
      o.valid := thisCanActualOut(i)
      o.bits := deq.uop
      o.bits.ctrl.singleStep := io.singleStep && (if (i == 0) singleStepStatus else true.B)
      io.fusionInfoOut(i) := deq.fusion
    })
  })

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
  private val enqPtrNext = Mux(io.redirect.valid, enqPtr - flushNum,
    Mux(actualEnqNum =/= 0.U, enqPtr + actualEnqNum, enqPtr))
  enqPtr := enqPtrNext
  enqPtrVecNext.zipWithIndex.map { case (ptr, i) => ptr := enqPtrNext + i.U }
  enqPtrVec := enqPtrVecNext

  deqPtrVecNext.zip(deqPtrVec).foreach({ case (dn, d) =>
    dn := d + PopCount(io.out(0).map(_.fire))
    when(!io.redirect.valid && io.out(0).map(_.fire).reduce(_ || _)) {
      d := dn
    }
  })

}