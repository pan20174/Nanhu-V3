package xiangshan.vector.vbackend.vexecute
import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import darecreek.exu.vfu.perm.Permutation
import freechips.rocketchip.diplomacy.{LazyModule, LazyModuleImp, LazyModuleImpLike}
import xiangshan.{HasXSParameter, Redirect, SrcType, XSBundle, XSModule}
import xiangshan.backend.execute.exu.{ExuConfig, ExuOutputNode, ExuType}
import xiangshan.backend.execute.fu.FuConfigs
import xiangshan.backend.regfile.ScalarRfReadPort
import xiangshan.vector.HasVectorParameters
import xiangshan.vector.vbackend.vexecute.vfu.uopToVuop
import xiangshan.vector.vbackend.vissue.vprs.{VpReservationStation, VprsIssueBundle}
import xiangshan.vector.vbackend.vregfile.VectorRfReadPort
import xs.utils.DelayN

class PermutationRegfileReadPort(implicit p: Parameters) extends XSBundle{
  val vrf = Flipped(new VectorRfReadPort)
  val srf = Flipped(new ScalarRfReadPort)
}

class VectorPermutationBlock(implicit p: Parameters) extends LazyModule{
  private val cfg = ExuConfig(
    name = "VPermExu",
    id = 0,
    complexName = "VectorPermuationComplex",
    fuConfigs = Seq(FuConfigs.vpermCfg),
    writebackToRob = false,
    writebackToVms = true,
    exuType = ExuType.vperm
  )
  val writebackNode = new ExuOutputNode(cfg)
  val vprs = LazyModule(new VpReservationStation)
  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) with HasXSParameter with HasVectorParameters {
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
    private val idata = RegEnable(io.rfReadPort.srf.idata, vprs.module.io.issue.bits.rsRen)
    private val fdata = RegEnable(io.rfReadPort.srf.fdata, vprs.module.io.issue.bits.rsRen)
    private val rsData = Mux(RegEnable(vprs.module.io.issue.bits.prsType === SrcType.fp, vprs.module.io.issue.bits.rsRen), fdata, idata)

    private val fuReady = !permutation.io.out.perm_busy
    private val issueDataReg = Reg(new VprsIssueBundle)
    private val issueScalarDataReg = Reg(UInt(XLEN.W))
    private val issueValidReg = RegInit(false.B)
    private val allowPipe = !issueValidReg || fuReady || (issueValidReg && issueDataReg.uop.robIdx.needFlush(io.redirect))
    when(allowPipe){
      issueValidReg := vprs.module.io.issue.valid && !vprs.module.io.issue.bits.uop.robIdx.needFlush(io.redirect)
    }
    when(vprs.module.io.issue.fire){
      issueDataReg := vprs.module.io.issue.bits
      issueScalarDataReg := rsData
    }
    vprs.module.io.issue.ready := allowPipe

    private val rfReqValid = RegNext(permutation.io.out.rd_en, false.B)
    private val rfReqAddr = RegEnable(permutation.io.out.rd_preg_idx, permutation.io.out.rd_en)
    io.rfReadPort.vrf.addr := rfReqAddr
    private val rfRespValid = RegNext(rfReqValid, false.B)
    private val rfRespData = RegEnable(io.rfReadPort.vrf.data, rfReqValid)
    private val isVgei16 = issueDataReg.uop.vctrl.funct6 === "b001110".U && issueDataReg.uop.vctrl.funct3 === "b000".U
    private val isE8 = issueDataReg.uop.vCsrInfo.vsew === 0.U

    permutation.io.in.uop := uopToVuop(issueDataReg.uop, issueValidReg, io.vstart, io.vcsr(2,1), io.frm, p)
    permutation.io.in.uop.info.vstart := io.vstart
    permutation.io.in.uop.info.vxrm := io.vcsr(2,1)
    permutation.io.in.uop.info.frm := io.frm
    permutation.io.in.rs1 := issueScalarDataReg
    permutation.io.in.vs1_preg_idx := issueDataReg.pvs1
    permutation.io.in.vs2_preg_idx := issueDataReg.pvs2
    permutation.io.in.old_vd_preg_idx := issueDataReg.pov
    permutation.io.in.mask_preg_idx := issueDataReg.pvm
    permutation.io.in.uop_valid := issueValidReg && !issueDataReg.uop.robIdx.needFlush(io.redirect)
    permutation.io.in.rdata := rfRespData
    permutation.io.in.rvalid := rfRespValid
    permutation.io.redirect := io.redirect
    when(isVgei16 && isE8) {
      permutation.io.in.vs2_preg_idx(1) := issueDataReg.pvs2(2)
      permutation.io.in.vs2_preg_idx(2) := issueDataReg.pvs2(4)
      permutation.io.in.vs2_preg_idx(3) := issueDataReg.pvs2(6)
    }

    private val fuInFire = issueValidReg && fuReady
    private val pdestReg = RegEnable(issueDataReg.pdest, fuInFire)
    //TODO: This is ugly, only for vrgatherei16.vv
    private val cntm2 = RegEnable(issueDataReg.uop.vctrl.isWidden && issueDataReg.uop.uopNum > 1.U, fuInFire)
    private val wbCounter = RegInit(0.U(4.W))
    private val wb = writebackNode.out.head._1
    wb.valid := permutation.io.out.wb_vld
    wb.bits.data := permutation.io.out.wb_data
    wb.bits.uop := permutation.io.out.uop.sysUop
    wb.bits.wakeupMask := ((1 << (VLEN / 8)) - 1).U((VLEN / 8).W)
    wb.bits.writeDataMask := ((1 << (VLEN / 8)) - 1).U((VLEN / 8).W)

    when(fuInFire) {
      wbCounter := 0.U
    }.elsewhen(wb.valid) {
      wbCounter := Mux(cntm2, wbCounter + 2.U, wbCounter + 1.U)
      assert(!(cntm2 && wb.bits.uop.uopNum(0)))
    }

    wb.bits.uop.uopIdx := wbCounter
    wb.bits.uop.pdest := pdestReg(wbCounter)
  }
}
