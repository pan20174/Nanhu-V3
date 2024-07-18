package xiangshan.backend.rob
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import xiangshan.{ExuOutput, Redirect, XSBundle, XSModule}
class WbReq(implicit p: Parameters) extends XSBundle {
  val robIdx = new RobPtr
  val data = UInt(log2Ceil(RenameWidth + 1).W)
}
class WbStateArray(enqNum:Int, wbNum:Int, redirectNum:Int)(implicit p: Parameters) extends XSModule{
  val io = IO(new Bundle {
    val enqIn = Input(Vec(enqNum, Valid(new WbReq)))
    val wbIn = Input(Vec(wbNum, Valid(new WbReq)))
    val redirectIn = Input(Vec(redirectNum, Valid(new WbReq)))
    val out = Output(Vec(RobSize, Bool()))
  })
  private val enq = io.enqIn
  private val wb = io.wbIn
  private val rdc = io.redirectIn
  private val maxWb = (1 << (RenameWidth + 1) - 1).asUInt
  private val writebackNum = RegInit(VecInit(Seq.fill(RobSize)(maxWb)))
  private val currentWbNum = writebackNum
  private val writebacked = Wire(Vec(RobSize, Bool()))
  private val mayBeFlushed = RegInit(VecInit(Seq.fill(RobSize)(false.B)))

  for(i <- writebacked.indices) {
    val enqSel = enq.map(r => r.valid && r.bits.robIdx.value === i.U)
    val enqData = Mux1H(enqSel, enq.map(_.bits.data))
    val wbSel = wb.map(r => r.valid && r.bits.robIdx.value === i.U && (r.bits.data === 1.U))
    val wbDataNum = PopCount(wbSel)
    val rdcSel = rdc.map(r => r.valid && r.bits.robIdx.value === i.U)
    val rdcData = Mux1H(rdcSel, rdc.map(_.bits.data))
    val enqHit = Cat(enqSel).orR
    val wbHit = Cat(wbSel).orR
    val rdcHit = Cat(rdcSel).orR
    when(enqHit) {
      writebackNum(i) := enqData
      mayBeFlushed(i) := false.B
    }.elsewhen(rdcHit) {
      writebackNum(i) := rdcData
      mayBeFlushed(i) := true.B
    }.elsewhen(wbHit) {
      writebackNum(i) := Mux(mayBeFlushed(i), maxWb, currentWbNum(i) - wbDataNum)
    }
    writebacked(i) := writebackNum(i) === 0.U
  }

  io.out.zip(writebacked).foreach({case(a, b) => a := b})

  private var enqIdx = 0
  private var wbIdx = 0
  private var rdcIdx = 0

  def enqueue(v: Bool, addr: RobPtr, block: Bool, data: UInt): Unit = {
    this.io.enqIn(enqIdx).valid := v
    this.io.enqIn(enqIdx).bits.robIdx := addr
    this.io.enqIn(enqIdx).bits.data := Mux(block, maxWb, data)
    enqIdx = enqIdx + 1
  }

  def writeback(v:Bool, addr:RobPtr, data:Bool):Unit = {
    this.io.wbIn(wbIdx).valid := v
    this.io.wbIn(wbIdx).bits.robIdx := addr
    this.io.wbIn(wbIdx).bits.data := data.asUInt
    wbIdx = wbIdx + 1
  }

  def redirect(v:Bool, addr:RobPtr, data:Bool):Unit = {
    this.io.redirectIn(rdcIdx).valid := v
    this.io.redirectIn(rdcIdx).bits.robIdx := addr
    this.io.redirectIn(rdcIdx).bits.data := Mux(data, 0.U, maxWb)
    rdcIdx = rdcIdx + 1
  }
}
