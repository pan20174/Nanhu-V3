package xiangshan.vector.videcode
import chisel3.util.BitPat
import freechips.rocketchip.rocket.Instructions._
import freechips.rocketchip.util.uintToBitPat
import xiangshan.vector._
abstract trait VDecodeConstants {
  def VLEN: Int = 128

  def X = BitPat("b?")

  def N = BitPat("b0")

  def Y = BitPat("b1")

  def decodeDefault: List[BitPat] = // illegal instruction
  //     eew0       eew1       eew2       eewType0    eewType1    eewType2    emul        emulType       ordered
  //     |          |          |          |           |           |           |           |              |  isLs
  //     |          |          |          |           |           |           |           |              |  |  ff/vcpop
  //     |          |          |          |           |           |           |           |              |  |  |  maskOp
  //     |          |          |          |           |           |           |           |              |  |  |  |  widden
  //     |          |          |          |           |           |           |           |              |  |  |  |  |  narrow
  //     |          |          |          |           |           |           |           |              |  |  |  |  |  |
    List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.dc, EewType.dc, EmulVal.dc, EmulType.lmul, X, X, X, X, X, X, X)

  val table: Array[(BitPat, List[BitPat])]
}

object VLDecode extends VDecodeConstants {
  val table: Array[(BitPat, List[BitPat])] = Array(
    VL1RE8_V -> List(EewVal.dc, EewVal.dc, EewVal.byte, EewType.dc, EewType.dc, EewType.const, EmulVal.r1, EmulType.const, N, Y, N, N, N, N, N),
    VL1RE16_V -> List(EewVal.dc, EewVal.dc, EewVal.hword, EewType.dc, EewType.dc, EewType.const, EmulVal.r1, EmulType.const, N, Y, N, N, N, N, N),
    VL1RE32_V -> List(EewVal.dc, EewVal.dc, EewVal.word, EewType.dc, EewType.dc, EewType.const, EmulVal.r1, EmulType.const, N, Y, N, N, N, N, N),
    VL1RE64_V -> List(EewVal.dc, EewVal.dc, EewVal.dword, EewType.dc, EewType.dc, EewType.const, EmulVal.r1, EmulType.const, N, Y, N, N, N, N, N),

    VL2RE8_V -> List(EewVal.dc, EewVal.dc, EewVal.byte, EewType.dc, EewType.dc, EewType.const, EmulVal.r2, EmulType.const, N, Y, N, N, N, N, N),
    VL2RE16_V -> List(EewVal.dc, EewVal.dc, EewVal.hword, EewType.dc, EewType.dc, EewType.const, EmulVal.r2, EmulType.const, N, Y, N, N, N, N, N),
    VL2RE32_V -> List(EewVal.dc, EewVal.dc, EewVal.word, EewType.dc, EewType.dc, EewType.const, EmulVal.r2, EmulType.const, N, Y, N, N, N, N, N),
    VL2RE64_V -> List(EewVal.dc, EewVal.dc, EewVal.dword, EewType.dc, EewType.dc, EewType.const, EmulVal.r2, EmulType.const, N, Y, N, N, N, N, N),

    VL4RE8_V -> List(EewVal.dc, EewVal.dc, EewVal.byte, EewType.dc, EewType.dc, EewType.const, EmulVal.r4, EmulType.const, N, Y, N, N, N, N, N),
    VL4RE16_V -> List(EewVal.dc, EewVal.dc, EewVal.hword, EewType.dc, EewType.dc, EewType.const, EmulVal.r4, EmulType.const, N, Y, N, N, N, N, N),
    VL4RE32_V -> List(EewVal.dc, EewVal.dc, EewVal.word, EewType.dc, EewType.dc, EewType.const, EmulVal.r4, EmulType.const, N, Y, N, N, N, N, N),
    VL4RE64_V -> List(EewVal.dc, EewVal.dc, EewVal.dword, EewType.dc, EewType.dc, EewType.const, EmulVal.r4, EmulType.const, N, Y, N, N, N, N, N),

    VL8RE8_V -> List(EewVal.dc, EewVal.dc, EewVal.byte, EewType.dc, EewType.dc, EewType.const, EmulVal.r8, EmulType.const, N, Y, N, N, N, N, N),
    VL8RE16_V -> List(EewVal.dc, EewVal.dc, EewVal.hword, EewType.dc, EewType.dc, EewType.const, EmulVal.r8, EmulType.const, N, Y, N, N, N, N, N),
    VL8RE32_V -> List(EewVal.dc, EewVal.dc, EewVal.word, EewType.dc, EewType.dc, EewType.const, EmulVal.r8, EmulType.const, N, Y, N, N, N, N, N),
    VL8RE64_V -> List(EewVal.dc, EewVal.dc, EewVal.dword, EewType.dc, EewType.dc, EewType.const, EmulVal.r8, EmulType.const, N, Y, N, N, N, N, N),

    VLE8_V -> List(EewVal.dc, EewVal.dc, EewVal.byte, EewType.dc, EewType.dc, EewType.const, EmulVal.dc, EmulType.lmul, N, Y, N, N, N, N, N),
    VLE16_V -> List(EewVal.dc, EewVal.dc, EewVal.hword, EewType.dc, EewType.dc, EewType.const, EmulVal.dc, EmulType.lmul, N, Y, N, N, N, N, N),
    VLE32_V -> List(EewVal.dc, EewVal.dc, EewVal.word, EewType.dc, EewType.dc, EewType.const, EmulVal.dc, EmulType.lmul, N, Y, N, N, N, N, N),
    VLE64_V -> List(EewVal.dc, EewVal.dc, EewVal.dword, EewType.dc, EewType.dc, EewType.const, EmulVal.dc, EmulType.lmul, N, Y, N, N, N, N, N),

    VLE8FF_V -> List(EewVal.dc, EewVal.dc, EewVal.byte, EewType.dc, EewType.dc, EewType.const, EmulVal.dc, EmulType.lmul, N, Y, Y, N, N, N, N),
    VLE16FF_V -> List(EewVal.dc, EewVal.dc, EewVal.hword, EewType.dc, EewType.dc, EewType.const, EmulVal.dc, EmulType.lmul, N, Y, Y, N, N, N, N),
    VLE32FF_V -> List(EewVal.dc, EewVal.dc, EewVal.word, EewType.dc, EewType.dc, EewType.const, EmulVal.dc, EmulType.lmul, N, Y, Y, N, N, N, N),
    VLE64FF_V -> List(EewVal.dc, EewVal.dc, EewVal.dword, EewType.dc, EewType.dc, EewType.const, EmulVal.dc, EmulType.lmul, N, Y, Y, N, N, N, N),

    VLM_V -> List(EewVal.dc, EewVal.dc, EewVal.byte, EewType.dc, EewType.dc, EewType.const, EmulVal.r1, EmulType.const, N, Y, N, Y, N, N, N),

    VLSE8_V -> List(EewVal.dc, EewVal.dc, EewVal.byte, EewType.dc, EewType.dc, EewType.const, EmulVal.dc, EmulType.lmul, N, Y, N, N, N, N, N),
    VLSE16_V -> List(EewVal.dc, EewVal.dc, EewVal.hword, EewType.dc, EewType.dc, EewType.const, EmulVal.dc, EmulType.lmul, N, Y, N, N, N, N, N),
    VLSE32_V -> List(EewVal.dc, EewVal.dc, EewVal.word, EewType.dc, EewType.dc, EewType.const, EmulVal.dc, EmulType.lmul, N, Y, N, N, N, N, N),
    VLSE64_V -> List(EewVal.dc, EewVal.dc, EewVal.dword, EewType.dc, EewType.dc, EewType.const, EmulVal.dc, EmulType.lmul, N, Y, N, N, N, N, N),

    VLOXEI8_V -> List(EewVal.dc, EewVal.byte, EewVal.dc, EewType.dc, EewType.const, EewType.sew, EmulVal.dc, EmulType.lmul, Y, Y, N, N, N, N, N),
    VLOXEI16_V -> List(EewVal.dc, EewVal.hword, EewVal.dc, EewType.dc, EewType.const, EewType.sew, EmulVal.dc, EmulType.lmul, Y, Y, N, N, N, N, N),
    VLOXEI32_V -> List(EewVal.dc, EewVal.word, EewVal.dc, EewType.dc, EewType.const, EewType.sew, EmulVal.dc, EmulType.lmul, Y, Y, N, N, N, N, N),
    VLOXEI64_V -> List(EewVal.dc, EewVal.dword, EewVal.dc, EewType.dc, EewType.const, EewType.sew, EmulVal.dc, EmulType.lmul, Y, Y, N, N, N, N, N),

    VLUXEI8_V -> List(EewVal.dc, EewVal.byte, EewVal.dc, EewType.dc, EewType.const, EewType.sew, EmulVal.dc, EmulType.lmul, N, Y, N, N, N, N, N),
    VLUXEI16_V -> List(EewVal.dc, EewVal.hword, EewVal.dc, EewType.dc, EewType.const, EewType.sew, EmulVal.dc, EmulType.lmul, N, Y, N, N, N, N, N),
    VLUXEI32_V -> List(EewVal.dc, EewVal.word, EewVal.dc, EewType.dc, EewType.const, EewType.sew, EmulVal.dc, EmulType.lmul, N, Y, N, N, N, N, N),
    VLUXEI64_V -> List(EewVal.dc, EewVal.dword, EewVal.dc, EewType.dc, EewType.const, EewType.sew, EmulVal.dc, EmulType.lmul, N, Y, N, N, N, N, N),
  )
}

object VSDecode extends VDecodeConstants {
  val table: Array[(BitPat, List[BitPat])] = Array(
    VS1R_V -> List(EewVal.dc, EewVal.dc, EewVal.byte, EewType.dc, EewType.dc, EewType.const, EmulVal.r1, EmulType.const, N, Y, N, N, N, N, N),
    VS2R_V -> List(EewVal.dc, EewVal.dc, EewVal.byte, EewType.dc, EewType.dc, EewType.const, EmulVal.r2, EmulType.const, N, Y, N, N, N, N, N),
    VS4R_V -> List(EewVal.dc, EewVal.dc, EewVal.byte, EewType.dc, EewType.dc, EewType.const, EmulVal.r4, EmulType.const, N, Y, N, N, N, N, N),
    VS8R_V -> List(EewVal.dc, EewVal.dc, EewVal.byte, EewType.dc, EewType.dc, EewType.const, EmulVal.r8, EmulType.const, N, Y, N, N, N, N, N),

    VSE8_V -> List(EewVal.dc, EewVal.dc, EewVal.byte, EewType.dc, EewType.dc, EewType.const, EmulVal.dc, EmulType.lmul, N, Y, N, N, N, N, N),
    VSE16_V -> List(EewVal.dc, EewVal.dc, EewVal.hword, EewType.dc, EewType.dc, EewType.const, EmulVal.dc, EmulType.lmul, N, Y, N, N, N, N, N),
    VSE32_V -> List(EewVal.dc, EewVal.dc, EewVal.word, EewType.dc, EewType.dc, EewType.const, EmulVal.dc, EmulType.lmul, N, Y, N, N, N, N, N),
    VSE64_V -> List(EewVal.dc, EewVal.dc, EewVal.dword, EewType.dc, EewType.dc, EewType.const, EmulVal.dc, EmulType.lmul, N, Y, N, N, N, N, N),

    VSM_V -> List(EewVal.byte, EewVal.dc, EewVal.byte, EewType.dc, EewType.dc, EewType.const, EmulVal.r1, EmulType.const, N, Y, N, Y, N, N, N),

    VSSE8_V -> List(EewVal.dc, EewVal.dc, EewVal.byte, EewType.dc, EewType.dc, EewType.const, EmulVal.dc, EmulType.lmul, N, Y, N, N, N, N, N),
    VSSE16_V -> List(EewVal.dc, EewVal.dc, EewVal.hword, EewType.dc, EewType.dc, EewType.const, EmulVal.dc, EmulType.lmul, N, Y, N, N, N, N, N),
    VSSE32_V -> List(EewVal.dc, EewVal.dc, EewVal.word, EewType.dc, EewType.dc, EewType.const, EmulVal.dc, EmulType.lmul, N, Y, N, N, N, N, N),
    VSSE64_V -> List(EewVal.dc, EewVal.dc, EewVal.dword, EewType.dc, EewType.dc, EewType.const, EmulVal.dc, EmulType.lmul, N, Y, N, N, N, N, N),

    VSUXEI8_V -> List(EewVal.dc, EewVal.byte, EewVal.dc, EewType.dc, EewType.const, EewType.sew, EmulVal.dc, EmulType.lmul, N, Y, N, N, N, N, N),
    VSUXEI16_V -> List(EewVal.dc, EewVal.hword, EewVal.dc, EewType.dc, EewType.const, EewType.sew, EmulVal.dc, EmulType.lmul, N, Y, N, N, N, N, N),
    VSUXEI32_V -> List(EewVal.dc, EewVal.word, EewVal.dc, EewType.dc, EewType.const, EewType.sew, EmulVal.dc, EmulType.lmul, N, Y, N, N, N, N, N),
    VSUXEI64_V -> List(EewVal.dc, EewVal.dword, EewVal.dc, EewType.dc, EewType.const, EewType.sew, EmulVal.dc, EmulType.lmul, N, Y, N, N, N, N, N),

    VSOXEI8_V -> List(EewVal.dc, EewVal.byte, EewVal.dc, EewType.dc, EewType.const, EewType.sew, EmulVal.dc, EmulType.lmul, Y, Y, N, N, N, N, N),
    VSOXEI16_V -> List(EewVal.dc, EewVal.hword, EewVal.dc, EewType.dc, EewType.const, EewType.sew, EmulVal.dc, EmulType.lmul, Y, Y, N, N, N, N, N),
    VSOXEI32_V -> List(EewVal.dc, EewVal.word, EewVal.dc, EewType.dc, EewType.const, EewType.sew, EmulVal.dc, EmulType.lmul, Y, Y, N, N, N, N, N),
    VSOXEI64_V -> List(EewVal.dc, EewVal.dword, EewVal.dc, EewType.dc, EewType.const, EewType.sew, EmulVal.dc, EmulType.lmul, Y, Y, N, N, N, N, N),
  )
}

object VADecode extends VDecodeConstants {
  val table: Array[(BitPat, List[BitPat])] = Array(
    //arithmetic instruction
    VADC_VIM -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VADC_VVM -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VADC_VXM -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSBC_VVM -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSBC_VXM -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VADD_VI -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VADD_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VADD_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VAND_VI -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VAND_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VAND_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSLL_VI -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSLL_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSLL_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSRA_VI -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSRA_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSRA_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSRL_VI -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSRL_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSRL_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSUB_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSUB_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VRSUB_VI -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, Y, N, N, N, N, N, N),
    VRSUB_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, Y, N, N, N, N, N, N),
    VOR_VI -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VOR_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VOR_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VXOR_VI -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VXOR_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VXOR_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VMAX_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VMAX_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VMAXU_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VMAXU_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VMERGE_VIM -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VMERGE_VVM -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VMERGE_VXM -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VMIN_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VMIN_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VMINU_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VMINU_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),

    VMV_V_I -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.dc, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VMV_V_V -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.dc, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VMV_V_X -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.dc, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),

    VSEXT_VF2 -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sewd2, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSEXT_VF4 -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sewd4, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSEXT_VF8 -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sewd8, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VZEXT_VF2 -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sewd2, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VZEXT_VF4 -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sewd4, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VZEXT_VF8 -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sewd8, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),

    // fixed-point Instruction
    VASUB_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VASUB_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VASUBU_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VASUBU_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VAADD_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VAADD_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VAADDU_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VAADDU_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSSRA_VI -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSSRA_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSSRA_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSSRL_VI -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSSRL_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSSRL_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSSUB_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSSUB_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSSUBU_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSSUBU_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSADD_VI -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSADD_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSADD_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSADDU_VI -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSADDU_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSADDU_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),

    //mac
    VMUL_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VMUL_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VMULH_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VMULH_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VMULHSU_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VMULHSU_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VMULHU_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VMULHU_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VMACC_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VMACC_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VNMSAC_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VNMSAC_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VMADD_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VMADD_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VNMSUB_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VNMSUB_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSMUL_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSMUL_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),

    //float
    VFADD_VF -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFADD_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFMACC_VF -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFMACC_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFMADD_VF -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFMADD_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFMAX_VF -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFMAX_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFMERGE_VFM -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFMIN_VF -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFMIN_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFMSAC_VF -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFMSAC_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFMSUB_VF -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFMSUB_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFMUL_VF -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFMUL_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFNMACC_VF -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFNMACC_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFNMADD_VF -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFNMADD_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFNMSAC_VF -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFNMSAC_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFNMSUB_VF -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFNMSUB_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFRSUB_VF -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFSGNJ_VF -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFSGNJ_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFSGNJN_VF -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFSGNJN_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFSGNJX_VF -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFSGNJX_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFSUB_VF -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFSUB_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),

    VFREC7_V -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFRSQRT7_V -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFCLASS_V -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFCVT_F_X_V -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFCVT_F_XU_V -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFCVT_RTZ_X_F_V -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFCVT_RTZ_XU_F_V -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFCVT_X_F_V -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFCVT_XU_F_V -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),

    VFMV_F_S -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.scalar, EewType.dc, EmulVal.r1, EmulType.const, N, N, N, N, N, N, N),
    VFMV_S_F -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.dc, EewType.scalar, EmulVal.r1, EmulType.const, N, N, N, N, N, N, N),
    VFMV_V_F -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.dc, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),

    //reduction-float
    VFREDMAX_VS -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.scalar, EewType.sew, EewType.scalar, EmulVal.dc, EmulType.lmul, Y, N, N, N, N, N, N),
    VFREDMIN_VS -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.scalar, EewType.sew, EewType.scalar, EmulVal.dc, EmulType.lmul, Y, N, N, N, N, N, N),
    VFREDOSUM_VS -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.scalar, EewType.sew, EewType.scalar, EmulVal.dc, EmulType.lmul, Y, N, N, N, N, N, N),
    VFREDUSUM_VS -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.scalar, EewType.sew, EewType.scalar, EmulVal.dc, EmulType.lmul, Y, N, N, N, N, N, N),

    //div
    VFDIV_VF -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFDIV_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFRDIV_VF -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFSQRT_V -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VDIV_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VDIV_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VDIVU_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VDIVU_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VREM_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VREM_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VREMU_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VREMU_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),

    //mask
    VMXNOR_MM -> List(EewVal.mask, EewVal.mask, EewVal.mask, EewType.const, EewType.const, EewType.const, EmulVal.r1, EmulType.const, N, N, N, Y, N, N, N),
    VMXOR_MM -> List(EewVal.mask, EewVal.mask, EewVal.mask, EewType.const, EewType.const, EewType.const, EmulVal.r1, EmulType.const, N, N, N, Y, N, N, N),
    VMAND_MM -> List(EewVal.mask, EewVal.mask, EewVal.mask, EewType.const, EewType.const, EewType.const, EmulVal.r1, EmulType.const, N, N, N, Y, N, N, N),
    VMANDN_MM -> List(EewVal.mask, EewVal.mask, EewVal.mask, EewType.const, EewType.const, EewType.const, EmulVal.r1, EmulType.const, N, N, N, Y, N, N, N),
    VMNAND_MM -> List(EewVal.mask, EewVal.mask, EewVal.mask, EewType.const, EewType.const, EewType.const, EmulVal.r1, EmulType.const, N, N, N, Y, N, N, N),
    VMNOR_MM -> List(EewVal.mask, EewVal.mask, EewVal.mask, EewType.const, EewType.const, EewType.const, EmulVal.r1, EmulType.const, N, N, N, Y, N, N, N),
    VMOR_MM -> List(EewVal.mask, EewVal.mask, EewVal.mask, EewType.const, EewType.const, EewType.const, EmulVal.r1, EmulType.const, N, N, N, Y, N, N, N),
    VMORN_MM -> List(EewVal.mask, EewVal.mask, EewVal.mask, EewType.const, EewType.const, EewType.const, EmulVal.r1, EmulType.const, N, N, N, Y, N, N, N),

    VFIRST_M -> List(EewVal.dc, EewVal.mask, EewVal.dc, EewType.dc, EewType.const, EewType.dc, EmulVal.r1, EmulType.const, N, N, N, N, N, N, N),
    VCPOP_M -> List(EewVal.dc, EewVal.mask, EewVal.dc, EewType.dc, EewType.const, EewType.dc, EmulVal.dc, EmulType.lmul, Y, N, Y, N, N, N, N),
    VMSBF_M -> List(EewVal.dc, EewVal.mask, EewVal.mask, EewType.dc, EewType.const, EewType.const, EmulVal.r1, EmulType.const, N, N, N, N, N, N, Y),
    VMSIF_M -> List(EewVal.dc, EewVal.mask, EewVal.mask, EewType.dc, EewType.const, EewType.const, EmulVal.r1, EmulType.const, N, N, N, N, N, N, Y),
    VMSOF_M -> List(EewVal.dc, EewVal.mask, EewVal.mask, EewType.dc, EewType.const, EewType.const, EmulVal.r1, EmulType.const, N, N, N, N, N, N, Y),
    VID_V -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.dc, EewType.sew, EmulVal.dc, EmulType.lmul, Y, N, N, N, N, N, N),
    VIOTA_M -> List(EewVal.dc, EewVal.mask, EewVal.dc, EewType.dc, EewType.const, EewType.sew, EmulVal.dc, EmulType.lmul, Y, N, N, N, N, N, Y),

    //reduction
    VREDAND_VS -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.scalar, EewType.sew, EewType.scalar, EmulVal.dc, EmulType.lmul, Y, N, N, N, N, N, N),
    VREDMAX_VS -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.scalar, EewType.sew, EewType.scalar, EmulVal.dc, EmulType.lmul, Y, N, N, N, N, N, N),
    VREDMAXU_VS -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.scalar, EewType.sew, EewType.scalar, EmulVal.dc, EmulType.lmul, Y, N, N, N, N, N, N),
    VREDMIN_VS -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.scalar, EewType.sew, EewType.scalar, EmulVal.dc, EmulType.lmul, Y, N, N, N, N, N, N),
    VREDMINU_VS -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.scalar, EewType.sew, EewType.scalar, EmulVal.dc, EmulType.lmul, Y, N, N, N, N, N, N),
    VREDOR_VS -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.scalar, EewType.sew, EewType.scalar, EmulVal.dc, EmulType.lmul, Y, N, N, N, N, N, N),
    VREDSUM_VS -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.scalar, EewType.sew, EewType.scalar, EmulVal.dc, EmulType.lmul, Y, N, N, N, N, N, N),
    VREDXOR_VS -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.scalar, EewType.sew, EewType.scalar, EmulVal.dc, EmulType.lmul, Y, N, N, N, N, N, N),

    //permutation instructions
    VCOMPRESS_VM -> List(EewVal.mask, EewVal.dc, EewVal.dc, EewType.const, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, Y),
    VSLIDE1DOWN_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSLIDE1UP_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, Y),
    VSLIDEDOWN_VI -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSLIDEDOWN_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VSLIDEUP_VI -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, Y),
    VSLIDEUP_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, Y),
    VFSLIDE1DOWN_VF -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, N),
    VFSLIDE1UP_VF -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, Y),
    VRGATHER_VI -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, Y),
    VRGATHER_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, Y),
    VRGATHER_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, Y),
    VRGATHEREI16_VV -> List(EewVal.hword, EewVal.dc, EewVal.dc, EewType.const, EewType.sew, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, N, Y),
    
    VMV_S_X -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.dc,  EewType.scalar,  EmulVal.r1, EmulType.const, N, N, N, N, N, N, N),
    VMV_X_S -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.scalar,  EewType.dc,    EmulVal.r1, EmulType.const, N, N, N, N, N, N, N),
    VMV1R_V -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew,    EmulVal.r1, EmulType.const, N, N, N, N, N, N, N),
    VMV2R_V -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew,    EmulVal.r2, EmulType.const, N, N, N, N, N, N, N),
    VMV4R_V -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew,    EmulVal.r4, EmulType.const, N, N, N, N, N, N, N),
    VMV8R_V -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sew,    EmulVal.r8, EmulType.const, N, N, N, N, N, N, N),

  )
}

object VWDecode extends VDecodeConstants{
  val table: Array[(BitPat, List[BitPat])] = Array(
    VWADD_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VWADD_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VWADD_WV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sewm2, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VWADD_WX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sewm2, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VWADDU_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VWADDU_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VWADDU_WV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sewm2, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VWADDU_WX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sewm2, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VWSUB_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VWSUB_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VWSUB_WV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sewm2, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VWSUB_WX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sewm2, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VWSUBU_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VWSUBU_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VWSUBU_WV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sewm2, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VWSUBU_WX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sewm2, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VWMUL_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VWMUL_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VWMULSU_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VWMULSU_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VWMULU_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VWMULU_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VWMACC_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VWMACC_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VWMACCSU_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VWMACCSU_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VWMACCU_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VWMACCU_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VWMACCUS_VX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),

    VFWADD_VF -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VFWADD_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VFWADD_WF -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sewm2, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VFWADD_WV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sewm2, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VFWSUB_VF -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VFWSUB_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VFWSUB_WF -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sewm2, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VFWSUB_WV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sewm2, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VFWMUL_VF -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VFWMUL_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VFWMACC_VF -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VFWMACC_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VFWNMACC_VF -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VFWNMACC_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VFWMSAC_VF -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VFWMSAC_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VFWNMSAC_VF -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VFWNMSAC_VV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),

    VFWCVT_F_F_V -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VFWCVT_F_X_V -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VFWCVT_F_XU_V -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VFWCVT_RTZ_X_F_V -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VFWCVT_RTZ_XU_F_V -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VFWCVT_X_F_V -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),
    VFWCVT_XU_F_V -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sew, EewType.sewm2, EmulVal.dc, EmulType.lmul, N, N, N, N, Y, N, N),

    VWREDSUM_VS -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.scalar, EewType.sew, EewType.scalar, EmulVal.dc, EmulType.lmul, Y, N, N, N, Y, N, N),
    VWREDSUMU_VS -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.scalar, EewType.sew, EewType.scalar, EmulVal.dc, EmulType.lmul, Y, N, N, N, Y, N, N),
    VFWREDOSUM_VS -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.scalar, EewType.sew, EewType.scalar, EmulVal.dc, EmulType.lmul, Y, N, N, N, Y, N, N),
    VFWREDUSUM_VS -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.scalar, EewType.sew, EewType.scalar, EmulVal.dc, EmulType.lmul, Y, N, N, N, Y, N, N),
  )
}

object VNDecode extends VDecodeConstants{
  val table: Array[(BitPat, List[BitPat])] = Array(
    //narrowing
    VNSRA_WI -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sewm2, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, Y, N),
    VNSRA_WV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sewm2, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, Y, N),
    VNSRA_WX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sewm2, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, Y, N),
    VNSRL_WI -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sewm2, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, Y, N),
    VNSRL_WV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sewm2, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, Y, N),
    VNSRL_WX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sewm2, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, Y, N),
    VNCLIP_WI -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sewm2, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, Y, N),
    VNCLIP_WV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sewm2, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, Y, N),
    VNCLIP_WX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sewm2, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, Y, N),
    VNCLIPU_WI -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sewm2, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, Y, N),
    VNCLIPU_WV -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.sew, EewType.sewm2, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, Y, N),
    VNCLIPU_WX -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sewm2, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, Y, N),
    VFNCVT_F_F_W -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sewm2, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, Y, N),
    VFNCVT_F_X_W -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sewm2, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, Y, N),
    VFNCVT_F_XU_W -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sewm2, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, Y, N),
    VFNCVT_ROD_F_F_W -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sewm2, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, Y, N),
    VFNCVT_RTZ_X_F_W -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sewm2, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, Y, N),
    VFNCVT_RTZ_XU_F_W -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sewm2, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, Y, N),
    VFNCVT_X_F_W -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sewm2, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, Y, N),
    VFNCVT_XU_F_W -> List(EewVal.dc, EewVal.dc, EewVal.dc, EewType.dc, EewType.sewm2, EewType.sew, EmulVal.dc, EmulType.lmul, N, N, N, N, N, Y, N),

    VMADC_VI -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.dc, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMADC_VIM -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.dc, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMADC_VV -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.sew, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMADC_VVM -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.sew, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMADC_VX -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.dc, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMADC_VXM -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.dc, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMSBC_VV -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.sew, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMSBC_VVM -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.sew, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMSBC_VX -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.dc, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMSBC_VXM -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.dc, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),

    VMSEQ_VI -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.dc, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMSEQ_VV -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.sew, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMSEQ_VX -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.dc, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMSGT_VI -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.dc, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMSGT_VX -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.dc, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMSGTU_VI -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.dc, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMSGTU_VX -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.dc, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMSLE_VI -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.dc, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMSLE_VV -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.sew, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMSLE_VX -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.dc, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMSLEU_VI -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.dc, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMSLEU_VV -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.sew, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMSLEU_VX -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.dc, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMSLT_VV -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.sew, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMSLT_VX -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.dc, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMSLTU_VV -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.sew, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMSLTU_VX -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.dc, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMSNE_VI -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.dc, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMSNE_VV -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.sew, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMSNE_VX -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.dc, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMFEQ_VF -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.dc, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMFEQ_VV -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.sew, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMFGE_VF -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.dc, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMFGT_VF -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.dc, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMFLE_VF -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.dc, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMFLE_VV -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.sew, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMFLT_VF -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.dc, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMFLT_VV -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.sew, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMFNE_VF -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.dc, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
    VMFNE_VV -> List(EewVal.dc, EewVal.dc, EewVal.mask, EewType.sew, EewType.sew, EewType.const, EmulVal.dc, EmulType.lmul, Y, N, N, Y, N, Y, N),
  )
}

