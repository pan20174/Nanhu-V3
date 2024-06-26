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
  def apply(in:Seq[Valid[RobPtr]], oldest:Boolean, haveEqual:Boolean, entryNum:Int, redirect: Valid[Redirect], p:Parameters) :Valid[UInt] = {
    val selector = Module(new SelectPolicy(in.length, oldest, haveEqual)(p))
    selector.io.in.zip(in).foreach({case(a, b) =>
      a.valid := b.valid
      a.bits := b.bits
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

class RAWQueueSelectPolicy(inputNum:Int, haveEqual:Boolean, idx: Int)(implicit p: Parameters) extends XSModule {
  val io = IO(new Bundle {
    val in = Input(Vec(inputNum, Valid(new RobPtr)))
    val out = Output(Valid(UInt(inputNum.W)))
  })
  override val desiredName: String = s"RAWQueueSelectPolicy_${idx}"
  private def ReductionFunc(in: Seq[(Valid[RobPtr], UInt)]):(Valid[RobPtr], UInt) = {
    val selectPolicy = Module(new SelectPolicy(in.length, true, haveEqual))
    val interRes = Wire(Valid(new RobPtr))
    selectPolicy.io.in.zip(in).foreach({case(a, b) =>
      a.valid := b._1.valid
      a.bits := b._1.bits
    })
    interRes.valid := selectPolicy.io.out.valid
    interRes.bits := Mux1H(selectPolicy.io.out.bits, in.map(_._1.bits))
    val idx = Mux1H(selectPolicy.io.out.bits, in.map(_._2))
    (interRes, idx)//((valid,rob),idx)
  }

  private val res = ParallelOperationN(io.in.zipWithIndex.map(in => (in._1, (1L << in._2).U(inputNum.W))), 8, ReductionFunc)
  io.out.valid := res._1.valid
  io.out.bits := res._2
}

class ViolationSelector(inNum:Int, haveEqual:Boolean)(implicit p: Parameters) extends XSModule{
  val io = IO(new Bundle{
    val in = Vec(inNum, Flipped(ValidIO(new RobPtr)))
    val out = ValidIO(new RobPtr)
    val chosen = Output(UInt(inNum.W))
  })

  private val selector = Module(new SelectPolicy(inNum, true, haveEqual))
  selector.io.in.zip(io.in).foreach({case(si, in) =>
    si.valid := in.valid
    si.bits := in.bits  //rob
  })
  io.out.valid := selector.io.out.valid
  io.out.bits := Mux1H(selector.io.out.bits, io.in.map(_.bits)) //rob
  io.chosen := selector.io.out.bits
}