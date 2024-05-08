package xiangshan.vector.writeback
import chisel3._
import chisel3.util._
import freechips.rocketchip.diplomacy.LazyModule
import freechips.rocketchip.diplomacy._
import org.chipsalliance.cde.config.Parameters
import xiangshan.backend.rob.RobPtr
import xiangshan.{ExceptionVec, ExuOutput, MicroOp, Redirect, RedirectLevel, TriggerCf, XSBundle, XSCoreParamsKey}
import xiangshan.backend.writeback.{WriteBackSinkNode, WriteBackSinkParam, WriteBackSinkType}
import xiangshan.vector.HasVectorParameters
import xs.utils.{CircularQueuePtr, HasCircularQueuePtrHelper, UIntToMask}
import xiangshan.vector.viwaitqueue.SplitCtrlIO

class VmbPtr(implicit p: Parameters) extends CircularQueuePtr[VmbPtr](
  p => p(XSCoreParamsKey).vectorParameters.vMergeBufferDepth
) with HasCircularQueuePtrHelper

class VmbAlloc(implicit p: Parameters) extends XSBundle {
  val req = Input(Vec(VIDecodeWidth, Valid(new RobPtr)))
  val resp = Output(Vec(VIDecodeWidth, Valid(new VmbPtr)))
}
class WbMergeBufferV2(implicit p: Parameters) extends LazyModule {
  val wbNodeParam = WriteBackSinkParam(name = "MergeBuffer", sinkType = WriteBackSinkType.vecMs)
  val writebackNode = new WriteBackSinkNode(wbNodeParam)
  lazy val module = new WbMergeBufferV2Impl(this)
}
class WbMergeBufferV2Impl(outer: WbMergeBufferV2) extends LazyModuleImp(outer) with HasVectorParameters with HasCircularQueuePtrHelper {
  private val writebackIn = outer.writebackNode.in.head._2._1 zip outer.writebackNode.in.head._1
  private val vectorWbNodeNum = writebackIn.length
  private val size = VectorMergeBufferDepth
  private val allocWidth = VIDecodeWidth
  private val deqWidth = VectorMergeWbWidth

  println(s"wbMergePortsNum: $vectorWbNodeNum")
  println("=================WbMergeBuffer Ports=================")
  for (wb <- writebackIn) {
    val name: String = wb._1.name
    val id = wb._1.id
    println(s"wbMergeNodes: $name, id: $id")
  }

  private val wbHasException = writebackIn.filter(wb => wb._1.hasException)
  private val wbHasNoException = writebackIn.filter(wb => !wb._1.hasException)
  println("=================WbMergeBuffer Exception Gen Port=================")
  for (wb <- wbHasException) {
    val name: String = wb._1.name
    val id = wb._1.id
    println(s"wbMergeNodes: $name, id: $id")
  }

  val io = IO(new Bundle {
    val allocate = new VmbAlloc
    val rob = Vec(deqWidth, ValidIO(new ExuOutput))
    val vmbInit = Flipped(ValidIO(new MicroOp))
    val vlUpdate = Output(Valid(UInt(log2Ceil(VLEN + 1).W)))
    val ffOut = Output(Valid(new ExuOutput))
    val redirect = Flipped(Valid(new Redirect))
    val splitCtrl = Flipped(new SplitCtrlIO)
  })
  private val allWritebacks = writebackIn.map(_._2)
  private val exceptionGen = Module(new VmbExceptionGen(wbHasException.length))
  exceptionGen.io.wb := wbHasException.map(_._2)
  exceptionGen.io.vmbInit := io.vmbInit
  exceptionGen.io.redirect := io.redirect

  private val table = Reg(Vec(size, new ExuOutput))
  private val valids = RegInit(VecInit(Seq.fill(size)(false.B)))
  private val wbCnts = RegInit(VecInit(Seq.fill(size)((0.U((log2Ceil(VLEN + 1) + 1).W)))))
  private val allocPtrVec = RegInit(VecInit(Seq.tabulate(allocWidth)(i => i.U.asTypeOf(new VmbPtr))))
  private val cmtPtrVec = RegInit(VecInit(Seq.tabulate(deqWidth)(i => i.U.asTypeOf(new VmbPtr))))
  assert(cmtPtrVec.head <= allocPtrVec.head)

  private val validEntriesNum = distanceBetween(allocPtrVec.head, cmtPtrVec.head)
  private val emptyEntriesNum = size.U - validEntriesNum
  private val flushVec = Cat(table.map(_.uop.robIdx.needFlush(io.redirect)).reverse)
  private val redirectMask = valids.asUInt & flushVec
  private val flushNum = PopCount(redirectMask)

  io.allocate.resp.zip(io.allocate.req).zipWithIndex.foreach({case((resp, req), idx) =>
    val ptr = allocPtrVec(idx)
    resp.valid := req.valid && idx.U < emptyEntriesNum && !io.redirect.valid
    resp.bits := ptr
  })

  for(i <- table.indices){
    val allocHits = io.allocate.resp.map(r => r.valid && r.bits.value === i.U)
    when(allocHits.reduce(_ | _)){
      table(i).uop.robIdx := Mux1H(allocHits, io.allocate.req.map(_.bits))
      table(i).uop.uopNum := VLEN.U
      table(i).vxsat := false.B
      table(i).fflags := false.B
      wbCnts(i) := 0.U
      valids(i) := true.B
    }
  }

  private val enqNum = PopCount(io.allocate.resp.map(_.valid))
  when(io.redirect.valid){
    allocPtrVec.foreach(ptr => ptr := ptr - flushNum)
  }.elsewhen(io.allocate.resp.map(_.valid).reduce(_|_)){
    allocPtrVec.foreach(ptr => ptr := ptr + enqNum)
  }
  valids.zip(redirectMask.asBools).foreach({case(v, r) =>
    when(v & r & io.redirect.valid){
      v := false.B
    }
  })

  private val deqTableEntry = table(cmtPtrVec.head.value)
  private val deqException = exceptionGen.io.current.valid && exceptionGen.io.current.bits.vmbIdx === cmtPtrVec.head
  private val flushSendCounter = RegInit(0.U(2.W))
  private val ff = exceptionGen.io.current.bits.ff
  private val uopIdx = exceptionGen.io.current.bits.uopIdx
  private val sendFlush = deqException && ff && uopIdx =/= 0.U && wbCnts(cmtPtrVec.head.value) === deqTableEntry.uop.uopNum
  //when fault-only-first instruction has exception, block deq
  private val blockDeq = sendFlush || flushSendCounter.orR
  when(sendFlush){
    flushSendCounter := 3.U
  }.elsewhen(flushSendCounter =/= 0.U){
    flushSendCounter := flushSendCounter - 1.U
  }
  io.ffOut.valid := flushSendCounter === 3.U
  io.ffOut.bits := DontCare
  io.ffOut.bits := table(exceptionGen.io.current.bits.vmbIdx.value)
  io.ffOut.bits.redirect.robIdx := exceptionGen.io.current.bits.robIdx
  io.ffOut.bits.redirect.ftqIdx := exceptionGen.io.current.bits.ftqIdx
  io.ffOut.bits.redirect.ftqOffset := exceptionGen.io.current.bits.ftqOffset
  io.ffOut.bits.redirect.level := RedirectLevel.flushAfter
  io.ffOut.bits.redirect.isFlushPipe := true.B
  io.ffOut.bits.redirect.isException := false.B
  io.ffOut.bits.redirect.isLoadStore := false.B
  io.ffOut.bits.redirect.isLoadLoad := false.B
  io.ffOut.bits.redirect.isXRet := false.B
  exceptionGen.io.clean := io.ffOut.valid

  io.vlUpdate.valid := io.ffOut.valid
  io.vlUpdate.bits := exceptionGen.io.current.bits.uopIdx

  private val deqCandidates = cmtPtrVec.map(ptr => table(ptr.value))
  private val onlyAllowDeqOne = cmtPtrVec.map(ptr => {
    ptr === exceptionGen.io.current.bits.vmbIdx && exceptionGen.io.current.valid
  }).reduce(_|_)
  private val canDeq = Wire(Vec(deqWidth, Bool()))
  canDeq.zipWithIndex.foreach {case(w, i) =>
    if(i == 0){
      w := !blockDeq
    } else {
      w := PopCount(io.rob.take(i).map(_.valid)) === i.U && !onlyAllowDeqOne && !blockDeq
    }
  }

  val hasOrderedLS = RegInit(false.B)
  when(io.vmbInit.valid && !io.vmbInit.bits.ctrl.blockBackward && io.vmbInit.bits.ctrl.noSpecExec) {
    hasOrderedLS := true.B
  }.elsewhen(io.splitCtrl.allDone) {
    hasOrderedLS := false.B
  }

  val deqEntry = deqCandidates.head
  val deqPtr = cmtPtrVec.head
  val deqEntryIsOrder = (!deqEntry.uop.ctrl.blockBackward) && deqEntry.uop.ctrl.noSpecExec

  val wbVec = allWritebacks.map(_.valid)
  val wbExceptionVec = allWritebacks.map(_.bits.uop.cf.exceptionVec)
  val wbException = Mux1H(wbVec, wbExceptionVec)

  io.splitCtrl.allowNext := hasOrderedLS && wbVec.reduce(_||_) && !(wbException.reduce(_||_))
  io.splitCtrl.allDone := hasOrderedLS && ((wbCnts(deqPtr.value) === deqEntry.uop.uopNum) || deqEntry.uop.uopNum === 0.U || (wbVec.reduce(_||_) && (wbException.reduce(_||_))))

  io.rob.zipWithIndex.foreach({case(deq, idx) =>
    val ptr = cmtPtrVec(idx).value
    deq.valid := (deqCandidates(idx).uop.uopNum === wbCnts(ptr) || deqCandidates(idx).uop.uopNum === 0.U) && valids(ptr) && canDeq(idx) && !io.redirect.valid
    deq.bits := deqCandidates(idx)
    when(deq.valid){
      valids(ptr) := false.B
    }
  })
  io.rob.head.bits.uop.uopIdx := Mux(deqException, exceptionGen.io.current.bits.uopIdx, 0.U)
  io.rob.head.bits.uop.cf.exceptionVec := Mux(deqException, exceptionGen.io.current.bits.exceptionVec, 0.U.asTypeOf(ExceptionVec()))
  io.rob.head.bits.uop.cf.trigger := Mux(deqException, exceptionGen.io.current.bits.trigger, 0.U.asTypeOf(new TriggerCf))

  private val deqNum = PopCount(io.rob.map(_.valid))
  when(io.rob.map(_.valid).reduce(_|_)){
    cmtPtrVec.foreach(ptr => ptr := ptr + deqNum)
  }
  private val vmbInitDelay = Pipe(io.vmbInit)
  when(vmbInitDelay.valid){
    table(vmbInitDelay.bits.mergeIdx.value) := DontCare
    table(vmbInitDelay.bits.mergeIdx.value).uop := vmbInitDelay.bits
    table(vmbInitDelay.bits.mergeIdx.value).vxsat := false.B
    table(vmbInitDelay.bits.mergeIdx.value).fflags := 0.U
  }

  private def checkWbHit(wb:Valid[ExuOutput], idx:Int, reg:Boolean):Bool = {
    if(!reg) {
      wb.valid && wb.bits.uop.mergeIdx.value === idx.U
    } else {
      RegNext(wb.valid && wb.bits.uop.mergeIdx.value === idx.U)
    }
  }

  for((c, idx) <- wbCnts.zipWithIndex){
    val hitVecNoException = wbHasNoException.map(_._2).map(checkWbHit(_, idx, false))
    val hitVecHasException = wbHasException.map(_._2).map(checkWbHit(_, idx, true))
    val hitVec = hitVecNoException ++ hitVecHasException
    when(hitVec.reduce(_|_)) {
      c := c + PopCount(hitVec)
    }
  }
  for((t, idx) <- table.zipWithIndex){
    val hitVecNoException = wbHasNoException.map(_._2).map(checkWbHit(_, idx, false))
    val hitVecHasException = wbHasException.map(_._2).map(checkWbHit(_, idx, true))
    val hitVec = hitVecNoException ++ hitVecHasException

    val vxHasNoExceptionVec = Wire(Vec(wbHasNoException.length, Bool()))
    val ffHasNoExceptionVec = Wire(Vec(wbHasNoException.length, UInt(5.W)))
    wbHasNoException.map(_._2).zip(hitVecNoException).zipWithIndex.foreach {
      case ((wb, hit), i) => {
        vxHasNoExceptionVec(i) := hit && wb.bits.vxsat
        ffHasNoExceptionVec(i) := wb.bits.fflags & VecInit(Seq.fill(5)(hit)).asUInt
      }
    }

    val vxHasExceptionVec = Wire(Vec(wbHasException.length, Bool()))
    val ffHasExceptionVec = Wire(Vec(wbHasException.length, Bool()))
    wbHasException.map(_._2).zip(hitVecHasException).zipWithIndex.foreach {
      case ((wb, hit), i) => {
        vxHasExceptionVec(i) := hit && RegNext(wb.bits.vxsat)
        ffHasExceptionVec(i) := RegNext(wb.bits.fflags) & VecInit(Seq.fill(5)(hit)).asUInt
      }
    }

    val vxVec = vxHasExceptionVec ++ vxHasNoExceptionVec
    val ffVec = ffHasExceptionVec ++ ffHasNoExceptionVec

    when(hitVec.reduce(_||_)) {
      t.vxsat := t.vxsat || vxVec.reduce(_||_)
      t.fflags := t.fflags | ffVec.reduce(_|_)
    }

    wbHasException.map(_._2).zipWithIndex.foreach {
      case (wb, i) => {
        when(checkWbHit(wb, idx, true)) {
          assert(t.uop.robIdx === RegNext(wb.bits.uop.robIdx), s"table ${idx} robIdx not matched!")
        }
      }
    }

    wbHasNoException.map(_._2).zipWithIndex.foreach {
      case (wb, i) => {
        when(checkWbHit(wb, idx, false)) {
          assert(t.uop.robIdx === wb.bits.uop.robIdx, s"table ${idx} robIdx not matched!")
        }
      }
    }

    when(valids(idx)) {
      assert(wbCnts(idx) <= t.uop.uopNum, s"Too many writebacks in vmb entry $idx!")
    }
  }
}
