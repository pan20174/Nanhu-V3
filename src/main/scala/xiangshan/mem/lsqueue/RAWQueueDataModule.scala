package xiangshan.mem.lsqueue
import org.chipsalliance.cde.config.Parameters
import xiangshan._
import chisel3._
import chisel3.util.{Mux1H, PopCount, RegEnable, UIntToOH, isPow2, log2Up}
import xiangshan.cache.HasDCacheParameters
import xs.utils._
/***************************************************************************************
 * Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
 * Copyright (c) 2020-2021 Peng Cheng Laboratory
 *
 * XiangShan is licensed under Mulan PSL v2.
 * You can use this software according to the terms and conditions of the Mulan PSL v2.
 * You may obtain a copy of Mulan PSL v2 at:
 *          http://license.coscl.org.cn/MulanPSL2
 *
 * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
 * EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
 * MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
 *
 * See the Mulan PSL v2 for more details.
 ***************************************************************************************/

class LoadRAWQueueDataModule(queryWidth: Int, entrySize: Int, writeNum: Int, numWBanks: Int = 1)(implicit p: Parameters) extends XSModule {
  val offsetWidth = 3
  val io = IO(new Bundle(){
    val write = Vec(writeNum, new Bundle(){
      val wen = Input(Bool())
      val entryAddr = Input(UInt(log2Up(entrySize).W))
      val paddr = Input(UInt((PAddrBits - offsetWidth).W))
//      val wen_mask = Input(Bool())    ///tmp
//      val entryAddr_mask = Input(UInt(log2Up(entrySize).W)) ///tmp
      val mask = Input(UInt(8.W))
    })

    val violation = Vec(queryWidth,new Bundle(){
      val paddr = Input(UInt((PAddrBits - offsetWidth).W))
      val mask = Input(UInt((XLEN/8).W))
      val violationMask = Output(Vec(entrySize,Bool()))
    })
  })

  val paddrModule = Module(new RAWQPAddrModule(numEntries = entrySize, numRead = queryWidth, numWrite = writeNum, numWBanks = numWBanks))
  val maskModule = Module(new RAWQMaskModule(numEntries = entrySize, numRead = queryWidth, numWrite = writeNum, numWBanks = 1))

  io.write.zip(paddrModule.io.write).foreach({ case (io, p) =>
    require(io.paddr.getWidth == p.paddr.getWidth)
  })

  maskModule.io.write.zip(paddrModule.io.write).zip(io.write).foreach({ case ((maskSink, paddrSink), source) =>
    maskSink.wen := source.wen
    maskSink.entryAddr := source.entryAddr
    maskSink.mask := source.mask

    paddrSink.wen := source.wen
    paddrSink.entryAddr := source.entryAddr
    paddrSink.paddr := source.paddr
  })

  maskModule.io.violation.zip(paddrModule.io.violation).zip(io.violation).foreach({ case ((m, p), req) =>
    m.mask := req.mask
    p.paddr := req.paddr

    req.violationMask := (p.violationMask.asUInt & m.violationMask.asUInt).asBools
  })
}



// load queue load mask module
class RAWQMaskModule(numEntries: Int, numRead: Int, numWrite: Int, numWBanks: Int)(implicit p: Parameters) extends XSModule {
  val io = IO(new Bundle {
    val write = Vec(numWrite, new Bundle() {
      val wen = Input(Bool())
      val entryAddr = Input(UInt(log2Up(numEntries).W))
      val mask = Input(UInt(8.W))
    })

    val violation = Vec(numRead, new Bundle() {
      val mask = Input(UInt((XLEN / 8).W))
      val violationMask = Output(Vec(numEntries, Bool()))
    })

    val debugInfoMask = Output(Vec(numEntries,UInt(8.W)))
  })
  //todo: init is tmp
  //  val maskReg = Reg(Vec(numEntries, UInt(8.W)))
  val maskReg = RegInit(VecInit(List.fill(numEntries)(0.U(8.W))))

  val waddrOH = io.write.map(a => UIntToOH(a.entryAddr))
  for (i <- 0 until numEntries) {
    val wen = io.write.zip(waddrOH).map(w => w._1.wen && w._2(i))
    val wMask = io.write.map(d => d.mask)
    when(wen.reduce(_|_)){
      maskReg(i) := Mux1H(wen,wMask)
    }
  }

  //violation response
  for (i <- 0 until numRead) {
    for (j <- 0 until numEntries) {
      io.violation(i).violationMask(j) := (io.violation(i).mask & maskReg(j)).orR
    }
  }

  //cannot write the same address
  io.write.zipWithIndex.foreach({ case (self, idx) =>
    io.write.zipWithIndex.filterNot(_._2 == idx).foreach({ other =>
      assert(!(other._1.wen && self.wen && (self.entryAddr === other._1.entryAddr)),"can not write the same address!!")
    })
  })

  io.debugInfoMask := maskReg
  dontTouch(io.debugInfoMask)
}


class RAWQPAddrModule(numEntries: Int, numRead: Int, numWrite: Int, numWBanks: Int)(implicit p: Parameters) extends
  XSModule with HasDCacheParameters {
  val offsetWidth = 3

  val io = IO(new Bundle {
    val write = Vec(numWrite, new Bundle() {
      val wen = Input(Bool())
      val entryAddr = Input(UInt(log2Up(numEntries).W))
      val paddr = Input(UInt((PAddrBits - offsetWidth).W))
    })
    val violation = Vec(numRead, new Bundle() {
      val paddr = Input(UInt((PAddrBits - offsetWidth).W))
      val violationMask = Output(Vec(numEntries, Bool()))
    })

    val debugInfoPaddr = Output(Vec(numEntries,UInt((PAddrBits - offsetWidth).W)))
  })
  //todo: init is tmp
//  val paddrReg = Reg(Vec(numEntries, UInt((PAddrBits - offsetWidth).W)))
  val paddrReg = RegInit(VecInit(List.fill(numEntries)(0.U((PAddrBits - offsetWidth).W))))

  //write data: use 2 cycle
  val s0_wen = io.write.map(_.wen)
  val s0_waddrOH = io.write.map(a => UIntToOH(a.entryAddr))
  val s0_wdata = io.write.map(_.paddr)

  val s1_wen = s0_wen.map(RegNext(_))
  val s1_waddrOH = s0_waddrOH.zip(s0_wen).map({case(a, en) =>
    RegEnable(a,en)
  })
  val s1_wdata = s0_wdata.zip(s0_wen).map({ case (d, en) =>
    RegEnable(d, en)
  })


  for (i <- 0 until numEntries) {
    val wen = s1_wen.zip(s1_waddrOH).map(w => w._1 && w._2(i))
    val data = s1_wdata
    when(wen.reduce(_ | _)) {
      paddrReg(i) := Mux1H(wen, data)
    }
  }


  // content addressed match
  for (i <- 0 until numRead) {
    for (j <- 0 until numEntries) {
      io.violation(i).violationMask(j) := io.violation(i).paddr === paddrReg(j)
    }
  }

  //cannot write the same address
  io.write.zipWithIndex.map({ case (self, idx) =>
    io.write.zipWithIndex.filterNot(_._2 == idx).map(other => other._1.wen && self.wen && (self.entryAddr === other._1.entryAddr))
  })

  io.debugInfoPaddr := paddrReg
  dontTouch(io.debugInfoPaddr)
}

