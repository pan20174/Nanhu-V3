package xiangshan.mem

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import utils._
import xs.utils._
import xiangshan._
import xiangshan.backend.issue.SelectPolicy
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

object ReplayQueueSelectPolicy {
  def apply(in:Seq[Valid[MicroOp]], oldest:Boolean, haveEqual:Boolean, entryNum:Int, redirect: Valid[Redirect], p:Parameters) :Valid[UInt] = {
    val selector = Module(new SelectPolicy(in.length, oldest, haveEqual)(p))
    selector.io.in.zip(in).foreach({case(a, b) =>
      a.valid := b.valid
      a.bits := b.bits.robIdx
    })
    val resReg = Reg(Valid(UInt(entryNum.W)))
    resReg.valid := selector.io.out.valid && !redirect.valid
    resReg.bits := selector.io.out.bits
    val res = Wire(Valid(UInt(entryNum.W)))
    res.valid := resReg.valid && !redirect.valid
    res.bits  := resReg.bits
    res
  }
}