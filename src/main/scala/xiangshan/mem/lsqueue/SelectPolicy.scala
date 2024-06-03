package xiangshan.mem

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import utils._
import xs.utils._
import xiangshan._
import xs.utils.perf.HasPerfLogging
import xiangshan.backend.rob.RobPtr

class SelectResp(entryIdxWidth:Int)(implicit p: Parameters) extends XSBundle {
  val info = new SelectInfo
  val entryIdxOH = UInt(entryIdxWidth.W)
}

class SelectInfo(implicit p: Parameters) extends XSBundle{
  val fuType = FuType()
  val lpv = Vec(loadUnitNum, UInt(LpvLength.W))
  val pdest = UInt(PhyRegIdxWidth.W)
  val rfWen = Bool()
  val fpWen = Bool()
  val isVector = Bool()
  val robPtr = new RobPtr
  val psrc = Vec(3, UInt(PhyRegIdxWidth.W))
  val vm = UInt(PhyRegIdxWidth.W)
  val isFma = Bool()
  val isSgOrStride = Bool()
  val ftqOffset = UInt(log2Up(PredictWidth).W)
}

class SelectPolicy(width:Int, oldest:Boolean, haveEqual:Boolean)(implicit p: Parameters) extends Module{
  val io = IO(new Bundle{
    val in = Input(Vec(width, Valid(new RobPtr)))
    val out = Output(Valid(UInt(width.W)))
  })
  private val ostr = if(oldest)"o" else "p"
  private val estr = if(haveEqual) "e" else ""
  override val desiredName:String = s"LoadReplayQueueSelectPolicy_w${width}" + ostr + estr
  if(oldest) {
    val onlyOne = PopCount(io.in.map(_.valid)) === 1.U
    val oldestOHMatrix = io.in.zipWithIndex.map({ case (self, idx) =>
      io.in.zipWithIndex.filterNot(_._2 == idx).map(i => (i._1.valid && self.valid && (self.bits <= i._1.bits)) ^ i._1.valid)
    })
    val oldestOHSeq = oldestOHMatrix.map(_.reduce(_|_)).map(!_)
    val oldestOH = if(haveEqual) PriorityEncoderOH(Cat(oldestOHSeq.reverse)) else Cat(oldestOHSeq.reverse)
    val defaultValue = Cat(io.in.map(_.valid).reverse)
    io.out.valid := io.in.map(_.valid).reduce(_ | _)
    io.out.bits := Mux(onlyOne, defaultValue, oldestOH)

    val selRobPtr = Mux1H(io.out.bits, io.in.map(_.bits))
    for(i <- io.in.indices) {
      if(haveEqual) {
        when(io.out.valid && !io.out.bits(i) && io.in(i).valid) {assert(selRobPtr <= io.in(i).bits)}
      } else {
        when(io.out.valid && !io.out.bits(i) && io.in(i).valid) {assert(selRobPtr < io.in(i).bits)}
      }
    }

  } else {
    io.out.valid := io.in.map(_.valid).reduce(_ | _)
    io.out.bits := PriorityEncoderOH(Cat(io.in.map(_.valid).reverse))
  }
  when(io.out.valid) {
    assert(PopCount(io.out.bits) === 1.U)
  }
}
object SelectPolicy {
  def apply(in:Seq[Valid[MicroOp]], oldest:Boolean, haveEqual:Boolean, entryNum:Int, redirect: Valid[Redirect], p:Parameters) :Valid[UInt] = {
    val selector = Module(new SelectPolicy(in.length, oldest, haveEqual)(p))
    selector.io.in.zip(in).foreach({case(a, b) =>
      a.valid := b.valid
      a.bits := b.bits.robIdx
    })
    val res = Reg(Valid(UInt(entryNum.W)))
    res.valid := selector.io.out.valid && !redirect.valid
    res.bits := selector.io.out.bits
    val res_out = Wire(Valid(UInt(entryNum.W)))
    res_out.valid := res.valid && !redirect.valid
    res_out.bits := res.bits
    res_out
  }
}