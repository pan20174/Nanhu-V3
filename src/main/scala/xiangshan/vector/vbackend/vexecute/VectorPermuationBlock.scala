package xiangshan.vector.vbackend.vexecute
import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
import freechips.rocketchip.diplomacy.{LazyModule, LazyModuleImp, LazyModuleImpLike}
import xiangshan.{HasXSParameter, Redirect, SrcType, XSBundle, XSModule}
import xiangshan.backend.execute.exu.{ExuConfig, ExuOutputNode, ExuType}
import xiangshan.backend.execute.fu.FuConfigs
import xiangshan.backend.regfile.ScalarRfReadPort
import xiangshan.vector.HasVectorParameters
import xiangshan.vector.vbackend.vexecute.vfu.permutation.Permutation
import xiangshan.vector.vbackend.vexecute.vfu.uopToVuop
import xiangshan.vector.vbackend.vissue.vprs.{VpReservationStation, VprsIssueBundle}
import xiangshan.vector.vbackend.vregfile.VectorRfReadPort

class PermutationRegfileReadPort(implicit p: Parameters) extends XSBundle{
  val vrf = Flipped(new VectorRfReadPort)
  val srf = Flipped(new ScalarRfReadPort)
}

class VectorPermutationBlock(implicit p: Parameters) extends LazyModule{
  private val cfg = ExuConfig(
    name = "VectorPermutationExu",
    id = 0,
    complexName = "VectorPermuationComplex",
    fuConfigs = Seq(FuConfigs.vpermCfg),
    exuType = ExuType.vperm
  )
  val writebackNode = new ExuOutputNode(cfg)
  val vprs = LazyModule(new VpReservationStation)
  lazy val module = new LazyModuleImp(this) with HasXSParameter with HasVectorParameters {
    val io = IO(new Bundle{
      val redirect = Input(Valid(new Redirect))
      val intAllocPregs = Vec(RenameWidth, Flipped(ValidIO(UInt(PhyRegIdxWidth.W))))
      val fpAllocPregs = Vec(RenameWidth, Flipped(ValidIO(UInt(PhyRegIdxWidth.W))))
      val vecAllocPregs = Vec(vectorParameters.vRenameWidth, Flipped(ValidIO(UInt(PhyRegIdxWidth.W))))
      val vstart = Input(UInt(log2Up(VLEN).W))
      val vcsr = Input(UInt(3.W))
      val frm = Input(UInt(3.W))
      val rfReadPort = new PermutationRegfileReadPort
    })

    vprs.module.io.redirect := io.redirect
    vprs.module.io.intAllocPregs := io.intAllocPregs
    vprs.module.io.fpAllocPregs := io.fpAllocPregs
    vprs.module.io.vecAllocPregs := io.vecAllocPregs

    private val permutation = Module(new Permutation)

    io.rfReadPort.srf.addr := vprs.module.io.issue.bits.prs
    io.rfReadPort.srf.isFp := vprs.module.io.issue.bits.prsType === SrcType.fp

    private val issueDataReg = Reg(new VprsIssueBundle)
    private val issueScalarDataReg = Reg(UInt(XLEN.W))
    private val issueValidReg = RegInit(false.B)
    private val allowPipe = !issueValidReg || permutation.io.out.perm_busy || (issueValidReg && issueDataReg.uop.robIdx.needFlush(io.redirect))
    when(allowPipe){
      issueValidReg := vprs.module.io.issue.valid
    }
    when(allowPipe && vprs.module.io.issue.valid){
      issueDataReg := vprs.module.io.issue.bits
      issueScalarDataReg := io.rfReadPort.srf.data
    }
    vprs.module.io.issue.ready := allowPipe

    private val rfReqValid = RegNext(permutation.io.out.rd_en, false.B)
    private val rfReqAddr = RegEnable(permutation.io.out.rd_preg_idx, permutation.io.out.rd_en)
    io.rfReadPort.vrf.addr := rfReqAddr
    private val rfRespValid = RegNext(rfReqValid, false.B)
    private val rfRespData = RegEnable(io.rfReadPort.vrf.data, rfReqValid)

    permutation.io.in.uop := uopToVuop(issueDataReg.uop, p)
    permutation.io.in.uop.info.vstart := io.vstart
    permutation.io.in.uop.info.vxrm := io.vcsr(2,1)
    permutation.io.in.uop.info.frm := io.frm
    permutation.io.in.rs1 := issueScalarDataReg
    permutation.io.in.vs1_preg_idx.zip(issueDataReg.pvs1).foreach({case(a,b) => a := b})
    permutation.io.in.vs2_preg_idx.zip(issueDataReg.pvs2).foreach({case(a,b) => a := b})
    permutation.io.in.old_vd_preg_idx.zip(issueDataReg.pov).foreach({case(a,b) => a := b})
    permutation.io.in.mask_preg_idx := issueDataReg.pvm
    permutation.io.in.uop_valid := issueValidReg
    permutation.io.in.uop_rob_idx := issueDataReg.uop.robIdx.value
    permutation.io.in.rdata := rfRespData
    permutation.io.in.rvalid := rfRespValid
    permutation.io.in.flush_vld := io.redirect.valid
    permutation.io.in.flush_rob_idx := io.redirect.bits.robIdx.value

    private val wb = writebackNode.out.head._1
    wb.valid := permutation.io.out.wb_vld
    wb.bits.data := permutation.io.out.wb_data
  }
}
