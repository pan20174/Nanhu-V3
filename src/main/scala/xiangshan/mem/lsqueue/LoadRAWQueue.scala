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
package xiangshan.mem

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import xiangshan._
import xiangshan.backend.rob.RobPtr
import xiangshan.mem.{SqPtr,LsqFreeList}
import xiangshan.mem.lsqueue.LoadRAWQueueDataModule
import xs.utils.HasCircularQueuePtrHelper
import xs.utils.perf.HasPerfLogging


class RAWEnqBundle(implicit p: Parameters) extends XSBundle {
  val robIdx = new RobPtr
  val sqIdx = new SqPtr
  val mask = UInt((XLEN/8).W)
  val paddr = UInt((PAddrBits - 3).W)
//  val data_valid = Bool()
}

class LoadEnqRAWBundle(implicit p: Parameters){
  val s2_enq = Output(Valid(new RAWEnqBundle))
  val s2_enqSuccess = Input(Bool())
  val s3_cancel = Input(Bool())
}


class storeRAWQueryBundle(implicit p: Parameters) extends XSBundle {
  val paddr = UInt((PAddrBits - 3).W)
  val mask = UInt(8.W)
  val robIdx = new RobPtr
  val lqIdx = new LqPtr
}

class RAWInfoUop(implicit p: Parameters) extends XSBundle {
//  val uop = new MicroOp
  val robIdx = new RobPtr
}


class LoadRAWQueue(implicit p: Parameters) extends XSModule with HasPerfLogging with HasCircularQueuePtrHelper{
  val io = IO(new Bundle() {
    val redirect = Flipped(ValidIO(new Redirect))
    val loadEnq = Vec(LoadPipelineWidth, Flipped(new LoadEnqRAWBundle)) //Load S2 enq //todo
    val stAddrReadyPtr = Input(new SqPtr)
    val storeQuery = Vec(StorePipelineWidth,Flipped(ValidIO(new storeRAWQueryBundle))) //store S1 query
    val violationRes = ValidIO(new RobPtr)

    val loadEnq_paddr = Vec(LoadPipelineWidth, new Bundle(){
      val valid = Input(Bool())
      val paddr = Input(UInt((PAddrBits - 3).W))
      val lqIdx = Input(UInt(log2Up(LoadRAWQueueSize).W))
    }) //tmp
    val loadEnq_mask = Vec(LoadPipelineWidth, new Bundle() {
      val valid = Input(Bool())
      val mask = Input(UInt(8.W))
      val lqIdx = Input(UInt(log2Up(LoadRAWQueueSize).W))
    }) //tmp

//    val violation = Vec(StorePipelineWidth,new Bundle() {
//      val paddr = Input(UInt((PAddrBits - 3).W))
//      val mask = Input(UInt((XLEN / 8).W))
//      val violationMask = Output(Vec(LoadRAWQueueSize,Bool()))
//    }) //tmp

  })

  val freeWidth = 4

  val allocatedReg = RegInit(VecInit(List.fill(LoadRAWQueueSize)(false.B)))
  val uopReg = Reg(Vec(LoadRAWQueueSize,new RAWInfoUop))
  val dataModule = Module(new LoadRAWQueueDataModule(
    queryWidth = StorePipelineWidth,
    entrySize = LoadRAWQueueSize,
    writeNum = LoadPipelineWidth,
    numWBanks = 1))
  dataModule.io := DontCare
//  dataModule.io.write.foreach(_.wen := false.B)


//  dataModule.io.write.zip(io.loadEnq_paddr).zip(io.loadEnq_mask).foreach({case((sink, paddr),mask) =>
//    sink.wen := paddr.valid
//    sink.entryAddr := paddr.lqIdx.asUInt
//    sink.paddr := paddr.paddr
//    sink.mask := mask.mask
//  })

//  dataModule.io.violation.zip(io.violation).foreach({case(module,query) =>
//    module.mask := query.mask
//    module.paddr := query.paddr
//    query.violationMask := module.violationMask
//  })

  val freeList = Module(new LsqFreeList(
    size = LoadRAWQueueSize,
    allocWidth = LoadPipelineWidth,
    freeWidth = freeWidth,
    enablePreAlloc = true,
    moduleName = "LoadRAWQueue freelist"
  ))

  val reqValidVec = io.loadEnq.map(_.s2_enq.valid)
  val needAllocateEntryVec = Wire(Vec(LoadPipelineWidth,Valid(UInt(LoadRAWQueueSize.W))))
  val freeMaskVec = Wire(UInt(LoadRAWQueueSize.W))

  needAllocateEntryVec.map(_.valid).zip(io.loadEnq).foreach({case (valid,req) =>
    valid := req.s2_enq.valid && !req.s2_enq.bits.robIdx.needFlush(io.redirect) &&
      (io.stAddrReadyPtr <= req.s2_enq.bits.sqIdx)
  })

  for ((enq, idx) <- io.loadEnq.zipWithIndex) {
    freeList.io.doAllocate(idx) := false.B
    freeList.io.allocateReq(idx) := true.B

    val offset = PopCount(needAllocateEntryVec.map(_.valid).take(idx))
    val canAccept = freeList.io.canAllocate(offset)

    needAllocateEntryVec(idx).bits := freeList.io.allocateSlot(offset)
    enq.s2_enqSuccess := Mux(needAllocateEntryVec(idx).valid, canAccept, true.B)
  }

  val reqWriteValidVec = Wire(Vec(LoadPipelineWidth,Bool()))
  reqWriteValidVec.zipWithIndex.foreach({case(reqWriteValid,idx) =>
    reqWriteValid := reqValidVec(idx) && io.loadEnq(idx).s2_enqSuccess

    when(reqWriteValid){
      dataModule.io.write.zip(io.loadEnq.map(_.s2_enq.bits)).foreach({ case (sink, source) =>
        sink.wen := true.B
        sink.entryAddr := needAllocateEntryVec(idx).bits
        sink.paddr := source.paddr
        sink.mask := source.mask
      })
    }
  })

  (0 until LoadRAWQueueSize).foreach({ rawIdx =>
    val redirecValid = uopReg(rawIdx).robIdx.needFlush(io.redirect)
    val w_enVec = Wire(Vec(StorePipelineWidth,Bool()))
    w_enVec.zipWithIndex.foreach({case (en,idx) =>
      en := reqWriteValidVec(idx) && !redirecValid &&  needAllocateEntryVec(idx).bits === rawIdx.U
    })
    val w_addr = OHToUInt(w_enVec)
    val w_data = io.loadEnq(w_addr).s2_enq.bits
    val w_en = w_enVec.reduce(_|_)

    when(w_en) {
      allocatedReg(rawIdx) := true.B
      uopReg(rawIdx).robIdx := w_data.robIdx
    }
  })


  needAllocateEntryVec.zipWithIndex.foreach({ case (self, idx) =>
    needAllocateEntryVec.zipWithIndex.filterNot(_._2 == idx).foreach({ other =>
      assert(!(other._1.valid && self.valid && self.bits === other._1.bits), "needAllocateEntryVec must be different!")
    })
  })

  //detect store load violation
  val violationOldestVec = Wire(Vec(StorePipelineWidth,Valid(new RobPtr)))
  val violationRes = Wire(Valid(new RobPtr))
  require(io.storeQuery.length == dataModule.io.violation.length)
  io.storeQuery.zipWithIndex.foreach({case (query,idx) =>
    //S0: store_s1 req
    dataModule.io.violation(idx).paddr := query.bits.paddr
    dataModule.io.violation(idx).mask := query.bits.mask
    val s0_needCheck = VecInit((0 until LoadRAWQueueSize).map(j => {
      allocatedReg(j) && !uopReg(j).robIdx.needFlush(io.redirect) && isAfter(uopReg(j).robIdx,query.bits.robIdx)
    }))
    val s0_addrMaskMatch = dataModule.io.violation(idx).violationMask.asUInt

    //S1: store_s2 resp
    val s1_addrMaskMatch = RegEnable(s0_addrMaskMatch,query.valid)
    val s1_needCheck = RegEnable(s0_needCheck,query.valid)
    val s1_violationValid = VecInit((0 until LoadRAWQueueSize).map(j => {
      s1_addrMaskMatch(j) && s1_needCheck(j)
    }))
    val uopRob = uopReg.map(_.robIdx)

    val selModule = Module(new RAWQueueSelectPolicy(LoadRAWQueueSize, true, idx))
    selModule.io.in.zipWithIndex.foreach({case (in,i) =>
      in.valid := s1_violationValid(i)
      in.bits := uopRob(i)
    })
    violationOldestVec(idx) := selModule.io.out
  })


  val violationSel = Module(new ViolationSelector(StorePipelineWidth,true))
  require(violationSel.io.in.length == violationOldestVec.length)

  violationSel.io.in := violationOldestVec
  violationRes := violationSel.io.out
  io.violationRes := violationRes
}