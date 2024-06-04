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
import utils._
import xs.utils._
import xiangshan.ExceptionNO._
import xiangshan._
import xiangshan.backend.execute.fu.FuConfigs.lduCfg
import xiangshan.backend.execute.fu._
import xiangshan.backend.execute.fu.csr.SdtrigExt
import xiangshan.backend.issue.{RSFeedback, RSFeedbackType, RsIdx}
import xiangshan.backend.rob.RobPtr
import xiangshan.cache._
import xiangshan.cache.mmu.{TlbCmd, TlbReq, TlbRequestIO, TlbResp}
import xs.utils.perf.HasPerfLogging

class LoadToLsqIO(implicit p: Parameters) extends XSBundle {
  val s1_lduUpdateLQ = ValidIO(new LqPaddrWriteBundle)

  val s2_lduUpdateLQ = ValidIO(new LqWriteBundle)
  val s2_load_data_forwarded = Output(Bool())
  val s2_dcache_require_replay = Output(Bool())

//  val s3_lq_wb = Flipped(DecoupledIO(new ExuOutput))
//  val s3_lq_wbLdRawData = Input(new LoadDataFromLQBundle)
  val s3_delayed_load_error = Output(Bool())
  val s3_replay_from_fetch = Output(Bool()) // update uop.ctrl.replayInst in load queue in s3

  val forwardFromSQ = new PipeLoadForwardFromSQ
  val loadViolationQuery = new LoadViolationQueryIO
  val trigger = Flipped(new LqTriggerIO)
}

class LoadUnitTriggerIO(implicit p: Parameters) extends XSBundle {
  val tdata2 = Input(UInt(64.W))
  val matchType = Input(UInt(2.W))
  val tEnable = Input(Bool()) // timing is calculated before this
  val addrHit = Output(Bool())
  val lastDataHit = Output(Bool())
}
class ReplayQueueLoadInBundle(implicit p: Parameters) extends XSBundle{
  val tmp = Bool()
}

class LoadUnit(implicit p: Parameters) extends XSModule with HasLoadHelper with HasPerfEvents with HasDCacheParameters with SdtrigExt with HasPerfLogging {
  val io = IO(new Bundle() {
    val rsIssueIn = Flipped(Decoupled(new ExuInput))
    val replayQIssueIn = Flipped(Decoupled(new ReplayQueueIssueBundle))
    val auxValid = Input(Bool())
    val ldout = Decoupled(new ExuOutput)
    val redirect = Flipped(ValidIO(new Redirect))
    val feedbackSlow = ValidIO(new RSFeedback)
    val feedbackFast = ValidIO(new RSFeedback)
    val rsIdx = Input(new RsIdx)
    val dcache = new DCacheLoadIO
    val forwardFromSBuffer = new LoadForwardQueryIO
    val lsq = new LoadToLsqIO
    val trigger = Vec(TriggerNum, new LoadUnitTriggerIO)
    val vmEnable = Input(Bool())
    val tlb = new TlbRequestIO(2)
    val pmp = Flipped(new PMPRespBundle()) // arrive same to tlb now

    //FDI
    val fdiReq = ValidIO(new  FDIReqBundle())
    val fdiResp = Flipped(new FDIRespBundle())

    // provide prefetch info
    val prefetch_train = ValidIO(new LsPipelineBundle())

    val s3_delayed_load_error = Output(Bool()) // load ecc error
    // Note that io.s3_delayed_load_error and io.lsq.s3_delayed_load_error is different

    val csrCtrl = Flipped(new CustomCSRCtrlIO)
    val cancel = Output(Bool())

    val s3_enq_replqQueue = DecoupledIO(new LoadToReplayQueueBundle)
    val ldStop = Input(Bool())
    val replayQFull = Input(Bool())
    val mmioWb = Flipped(DecoupledIO(new ExuOutput))
  })
  io.rsIssueIn.ready := true.B
  io.replayQIssueIn.ready := true.B
  val rsIssueIn = WireInit(io.rsIssueIn)
  val replayIssueIn = WireInit(io.replayQIssueIn)
  assert(!(rsIssueIn.valid && replayIssueIn.valid))

  val s0_in_fromRs = Wire(DecoupledIO(new LoadPipelineBundleS0))
  val s0_in_fromReplayQ = Wire(DecoupledIO(new LoadPipelineBundleS0))
  s0_in_fromRs.valid := rsIssueIn.valid
  s0_in_fromRs.ready := true.B

  s0_in_fromReplayQ.valid := replayIssueIn.valid
  s0_in_fromReplayQ.ready := true.B

  s0_in_fromRs.bits.fromRsToS0Bundle(rsIssueIn.bits, io.rsIdx)
  s0_in_fromReplayQ.bits.fromRQToS0Bundle(replayIssueIn.bits)

  val s0_req_tlb = io.tlb.req
  val s0_req_dcache = io.dcache.req
  //  val s0_in = s0_in_fromRs
  val s0_in = Wire(DecoupledIO(new LoadPipelineBundleS0))
  val s0_out = Wire(Decoupled(new LsPipelineBundle))

  s0_in.valid := s0_in_fromRs.valid || s0_in_fromReplayQ.valid
  s0_in.bits := Mux(s0_in_fromReplayQ.valid, s0_in_fromReplayQ.bits, s0_in_fromRs.bits)

  val s0_imm12 = s0_in.bits.uop.ctrl.imm(11,0)
  val s0_vaddr = Mux(s0_in_fromReplayQ.valid,s0_in.bits.vaddr_replay,
                WireInit(s0_in.bits.src(0) + SignExt(s0_imm12, VAddrBits)))
  val s0_mask = WireInit(genWmask(s0_vaddr, s0_in.bits.uop.ctrl.fuOpType(1,0)))
  val s0_uop = WireInit(s0_in.bits.uop)

  val s0_auxValid = s0_in.valid

  val s0_isSoftPrefetch = Mux(s0_in_fromReplayQ.valid,false.B,LSUOpType.isPrefetch(s0_uop.ctrl.fuOpType))
  val s0_isSoftPrefetchRead = s0_uop.ctrl.fuOpType === LSUOpType.prefetch_r
  val s0_isSoftPrefetchWrite = s0_uop.ctrl.fuOpType === LSUOpType.prefetch_w
  val s0_EnableMem = s0_in.bits.uop.loadStoreEnable

  s0_req_tlb := DontCare
  s0_req_tlb.valid := s0_auxValid
  s0_req_tlb.bits.vaddr := s0_vaddr
  s0_req_tlb.bits.cmd := TlbCmd.read
  s0_req_tlb.bits.size := LSUOpType.size(s0_uop.ctrl.fuOpType)
  s0_req_tlb.bits.robIdx := s0_uop.robIdx
  s0_req_tlb.bits.debug.pc := s0_uop.cf.pc

  s0_req_dcache.valid := s0_auxValid
  when (s0_isSoftPrefetchRead) {
    s0_req_dcache.bits.cmd  := MemoryOpConstants.M_PFR
  }.elsewhen (s0_isSoftPrefetchWrite) {
    s0_req_dcache.bits.cmd  := MemoryOpConstants.M_PFW
  }.otherwise {
    s0_req_dcache.bits.cmd  := MemoryOpConstants.M_XRD
  }

  s0_req_dcache.bits.robIdx := s0_in.bits.uop.robIdx
  s0_req_dcache.bits.addr := s0_vaddr
  s0_req_dcache.bits.mask := s0_mask
  s0_req_dcache.bits.data := DontCare
  s0_req_dcache.bits.instrtype := Mux(s0_isSoftPrefetch,SOFT_PREFETCH.U,LOAD_SOURCE.U)
  s0_req_dcache.bits.id := DontCare

  val s0_addrAligned = LookupTree(s0_uop.ctrl.fuOpType(1, 0), List(
    "b00".U   -> true.B,                   //b
    "b01".U   -> (s0_vaddr(0)    === 0.U), //h
    "b10".U   -> (s0_vaddr(1, 0) === 0.U), //w
    "b11".U   -> (s0_vaddr(2, 0) === 0.U)  //d
  ))

  s0_out.valid := s0_in.valid
  s0_out.bits := DontCare
  s0_out.bits.isReplayQReplay := s0_in.bits.isReplayQReplay
  s0_out.bits.schedIndex := s0_in.bits.schedIndex
  s0_out.bits.vaddr := s0_vaddr
  s0_out.bits.mask := s0_mask
  s0_out.bits.uop := s0_uop

  private val s0_vaddr2 = s0_in.bits.src(0) + SignExt(s0_imm12, XLEN)
  dontTouch(s0_vaddr)
  private val illegalAddr = s0_vaddr2(XLEN - 1, VAddrBits - 1) =/= 0.U && s0_vaddr2(XLEN - 1, VAddrBits - 1) =/= Fill(XLEN - VAddrBits + 1, 1.U(1.W))
  s0_out.bits.uop.cf.exceptionVec(loadAddrMisaligned) := Mux(s0_EnableMem, !s0_addrAligned, false.B)
  s0_out.bits.uop.cf.exceptionVec(loadPageFault) := Mux(s0_EnableMem & io.vmEnable, illegalAddr, false.B)

  s0_out.bits.rsIdx := io.rsIdx
  s0_out.bits.replayCause.foreach(_ := false.B)
  s0_out.bits.isSoftPrefetch := s0_isSoftPrefetch

  s0_in.ready := !s0_in.valid || s0_out.ready

  val s0_cancel = !s0_req_dcache.ready


  val s1_in = Wire(Decoupled(new LsPipelineBundle))
  val s1_out = Wire(Decoupled(new LsPipelineBundle))

  PipelineConnect(s0_out, s1_in, true.B, s0_out.bits.uop.robIdx.needFlush(io.redirect))

  s1_out.bits := s1_in.bits
  val s1_dtlbResp = io.tlb.resp
  val s1_cancel_inner = RegEnable(s0_cancel,s0_out.fire)

  s1_dtlbResp.ready := true.B

  val s1_uop = s1_in.bits.uop
  val s1_paddr_dup_lsu = s1_dtlbResp.bits.paddr(0)
  val s1_paddr_dup_dcache = s1_dtlbResp.bits.paddr(1)

  io.dcache.s1_paddr_dup_lsu := s1_paddr_dup_lsu
  io.dcache.s1_paddr_dup_dcache := s1_paddr_dup_dcache
  val s1_enableMem = s1_in.bits.uop.loadStoreEnable


  // af & pf exception were modified below.
  val s1_exception = Mux(s1_enableMem && s1_in.valid, ExceptionNO.selectByFu(s1_out.bits.uop.cf.exceptionVec, lduCfg).asUInt.orR, false.B)
  val s1_tlb_miss = s1_dtlbResp.bits.miss
  val s1_mask = s1_in.bits.mask
  val s1_bank_conflict = io.dcache.s1_bank_conflict

  val s1_dcacheKill = s1_in.valid && (s1_tlb_miss || s1_exception || s1_cancel_inner || (!s1_enableMem))

  val s1_sbufferForwardReq = io.forwardFromSBuffer
  val s1_lsqForwardReq = io.lsq.forwardFromSQ
  val s1_ldViolationQueryReq = io.lsq.loadViolationQuery.s1_req
  val s1_fdiReq = io.fdiReq

  s1_sbufferForwardReq.valid := s1_in.valid && !(s1_exception || s1_tlb_miss) && s1_enableMem
  s1_sbufferForwardReq.vaddr := s1_in.bits.vaddr
  s1_sbufferForwardReq.paddr := s1_paddr_dup_lsu
  s1_sbufferForwardReq.uop := s1_uop
  s1_sbufferForwardReq.sqIdx := s1_uop.sqIdx
  s1_sbufferForwardReq.mask := s1_mask
  s1_sbufferForwardReq.pc := s1_uop.cf.pc

  s1_lsqForwardReq.valid := s1_in.valid && !(s1_exception || s1_tlb_miss) && s1_enableMem
  s1_lsqForwardReq.vaddr := s1_in.bits.vaddr
  s1_lsqForwardReq.paddr := s1_paddr_dup_lsu
  s1_lsqForwardReq.uop := s1_uop
  s1_lsqForwardReq.sqIdx := s1_uop.sqIdx
  s1_lsqForwardReq.sqIdxMask := DontCare
  s1_lsqForwardReq.mask := s1_mask
  s1_lsqForwardReq.pc := s1_uop.cf.pc

  s1_ldViolationQueryReq.valid := s1_in.valid && !(s1_exception || s1_tlb_miss) && s1_enableMem
  s1_ldViolationQueryReq.bits.paddr := s1_paddr_dup_lsu
  s1_ldViolationQueryReq.bits.uop := s1_uop

  s1_fdiReq.valid := s1_out.fire
  s1_fdiReq.bits.addr := s1_out.bits.vaddr
  s1_fdiReq.bits.inUntrustedZone := s1_out.bits.uop.fdiUntrusted
  s1_fdiReq.bits.operation := FDIOp.read


  // Generate feedback signal caused by:
  // * dcache bank conflict
  // * need redo ld-ld violation check
  val s1_csrCtrl_ldld_vio_check_enable = io.csrCtrl.ldld_vio_check_enable
  val s1_needLdVioCheckRedo = s1_ldViolationQueryReq.valid &&
    !s1_ldViolationQueryReq.ready &&
    RegNext(s1_csrCtrl_ldld_vio_check_enable)
//  val s1_rsFeedback = Wire(ValidIO(new RSFeedback))

//  s1_rsFeedback.valid := s1_in.valid && (s1_bank_conflict || s1_needLdVioCheckRedo || s1_cancel_inner) && s1_enableMem
//  s1_rsFeedback.bits.rsIdx := s1_in.bits.rsIdx
//  s1_rsFeedback.bits.sourceType := Mux(s1_bank_conflict, RSFeedbackType.bankConflict, RSFeedbackType.ldVioCheckRedo)

  s1_out.bits.replayCause(LoadReplayCauses.C_BC) := s1_in.valid && (s1_cancel_inner || s1_bank_conflict || s1_needLdVioCheckRedo) //todo: for debug
  //disable bank_conflict
//  s1_rsFeedback.valid := s1_in.valid && (s1_needLdVioCheckRedo || s1_cancel_inner) && s1_enableMem && (!s1_in.bits.isReplayQReplay)
//  s1_rsFeedback.bits.rsIdx := s1_in.bits.rsIdx
//  s1_rsFeedback.bits.sourceType := RSFeedbackType.ldVioCheckRedo

  // if replay is detected in load_s1,
  // load inst will be canceled immediately
//  s1_out.valid := s1_in.valid && (!s1_rsFeedback.valid || !s1_enableMem)
  s1_out.valid := s1_in.valid && s1_enableMem
  s1_out.bits.paddr := s1_paddr_dup_lsu
  s1_out.bits.tlbMiss := s1_tlb_miss

  // current ori test will cause the case of ldest == 0, below will be modifeid in the future.
  // af & pf exception were modified
  s1_out.bits.uop.cf.exceptionVec(loadPageFault) := (s1_dtlbResp.bits.excp(0).pf.ld || s1_in.bits.uop.cf.exceptionVec(loadPageFault)) && s1_enableMem
  s1_out.bits.uop.cf.exceptionVec(loadAccessFault) := s1_dtlbResp.bits.excp(0).af.ld && s1_enableMem
  s1_out.bits.ptwBack := s1_dtlbResp.bits.ptwBack
  s1_out.bits.rsIdx := s1_in.bits.rsIdx
  s1_out.bits.isSoftPrefetch := s1_in.bits.isSoftPrefetch

  s1_in.ready := !s1_in.valid || s1_out.ready

  assert(s0_in.ready)

  val s2_in = Wire(Decoupled(new LsPipelineBundle))
  val s2_out = Wire(Decoupled(new LsPipelineBundle))

  val s2_cancel_inner = RegEnable(s1_cancel_inner,s1_out.fire)

  val s2_pmp = WireInit(io.pmp)
  val s2_static_pm = RegEnable(io.tlb.resp.bits.static_pm,io.tlb.resp.valid)
  when(s2_static_pm.valid) {
    s2_pmp.ld := false.B
    s2_pmp.st := false.B
    s2_pmp.instr := false.B
    s2_pmp.mmio := s2_static_pm.bits
  }

  val s2_enableMem = s2_in.bits.uop.loadStoreEnable && s2_in.valid
  val s2_is_prefetch = s2_in.bits.isSoftPrefetch

  val s2_exception_vec = WireInit(s2_in.bits.uop.cf.exceptionVec)
  s2_exception_vec(loadAccessFault) := (s2_in.bits.uop.cf.exceptionVec(loadAccessFault) || s2_pmp.ld) && s2_enableMem
  when(s2_is_prefetch) {
    s2_exception_vec := 0.U.asTypeOf(s2_exception_vec.cloneType)
  }
  val s2_exception = Mux(s2_enableMem, ExceptionNO.selectByFu(s2_exception_vec, lduCfg).asUInt.orR,false.B)
  //FDI load access fault
  s2_exception_vec(fdiULoadAccessFault) := io.fdiResp.fdi_fault === FDICheckFault.UReadDascisFault

  ///more resp input:
  val s2_dcacheResp = io.dcache.resp
  val s2_LSQ_LoadForwardQueryIO = Wire(new LoadForwardQueryIO)
  val s2_SB_LoadForwardQueryIO = Wire(new LoadForwardQueryIO)
  val s2_loadViolationQueryResp = Wire(ValidIO(new LoadViolationQueryResp))

  s2_LSQ_LoadForwardQueryIO := DontCare
  s2_SB_LoadForwardQueryIO := DontCare
  s2_dcacheResp.ready := true.B

  s2_LSQ_LoadForwardQueryIO.forwardData := io.lsq.forwardFromSQ.forwardData
  s2_LSQ_LoadForwardQueryIO.forwardMask := io.lsq.forwardFromSQ.forwardMask
//  s2_LSQ_LoadForwardQueryIO.forwardMaskFast := io.lsq.forwardFromSQ.forwardMaskFast
  s2_LSQ_LoadForwardQueryIO.dataInvalid := io.lsq.forwardFromSQ.dataInvalid
  s2_LSQ_LoadForwardQueryIO.matchInvalid := io.lsq.forwardFromSQ.matchInvalid

  s2_SB_LoadForwardQueryIO.forwardData := io.forwardFromSBuffer.forwardData
  s2_SB_LoadForwardQueryIO.forwardMask := io.forwardFromSBuffer.forwardMask
//  s2_SB_LoadForwardQueryIO.forwardMaskFast := io.forwardFromSBuffer.forwardMaskFast
  s2_SB_LoadForwardQueryIO.dataInvalid := io.forwardFromSBuffer.dataInvalid // always false
  s2_SB_LoadForwardQueryIO.matchInvalid := io.forwardFromSBuffer.matchInvalid

  s2_loadViolationQueryResp := io.lsq.loadViolationQuery.s2_resp

  ///more output: dataForwarded
  val s2_actually_mmio = s2_pmp.mmio
  val s2_mask = s2_in.bits.mask
  val s2_paddr = s2_in.bits.paddr
  val s2_tlb_miss = s2_in.bits.tlbMiss
  val s2_mmio = !s2_is_prefetch && s2_actually_mmio && !s2_exception
  val s2_cache_miss = s2_dcacheResp.bits.miss
  val s2_cache_replay = s2_dcacheResp.bits.replay
  val s2_cache_tag_error = 0.U.asTypeOf(s2_dcacheResp.bits.tag_error.cloneType)
  val s2_ldld_violation = s2_loadViolationQueryResp.valid &&
    s2_loadViolationQueryResp.bits.have_violation &&
    RegNext(io.csrCtrl.ldld_vio_check_enable)
  val s2_data_invalid = s2_LSQ_LoadForwardQueryIO.dataInvalid && !s2_ldld_violation && !s2_exception


  ///more output info:
  val s2_dcache_kill = s2_pmp.ld || s2_pmp.mmio // move pmp resp kill to outside
  val s2_dcacheShouldResp = !(s2_tlb_miss || s2_exception || s2_mmio || s2_is_prefetch)
  assert(!(s2_enableMem && (!s2_cancel_inner && s2_dcacheShouldResp && !s2_dcacheResp.valid)), "DCache response got lost")

  // merge forward result
  // lsq has higher priority than sbuffer
  val s2_forwardMask = Wire(Vec(8, Bool()))
  val s2_forwardData = Wire(Vec(8, UInt(8.W)))
  for (i <- 0 until XLEN / 8) {
    s2_forwardMask(i) := s2_LSQ_LoadForwardQueryIO.forwardMask(i) || s2_SB_LoadForwardQueryIO.forwardMask(i)
    s2_forwardData(i) := Mux(s2_LSQ_LoadForwardQueryIO.forwardMask(i), s2_LSQ_LoadForwardQueryIO.forwardData(i), s2_SB_LoadForwardQueryIO.forwardData(i))
  }

  val s2_fullForward = ((~s2_forwardMask.asUInt).asUInt & s2_mask) === 0.U && !s2_LSQ_LoadForwardQueryIO.dataInvalid

  when(s2_enableMem) {
//    s2_out.valid := s2_in.valid && !s2_tlb_miss && !s2_data_invalid
    //load must go to S2 to return feedback
    s2_out.valid := s2_in.valid && !s2_out.bits.uop.robIdx.needFlush(io.redirect)
  }.otherwise {
    s2_out.valid := s2_in.valid && !s2_out.bits.uop.robIdx.needFlush(io.redirect)
  }
  s2_out.bits := s2_in.bits
  s2_out.bits.data := 0.U

  //EnableFastForward is false
  s2_out.bits.miss := s2_cache_miss &&
    !s2_exception &&
    !s2_fullForward
    !s2_ldld_violation &&
    !s2_is_prefetch && s2_enableMem

  s2_out.bits.uop.ctrl.fpWen := s2_in.bits.uop.ctrl.fpWen && !s2_exception

  ///output
  val s2_loadDataFromDcache = Wire(new LoadDataFromDcacheBundle)
  s2_loadDataFromDcache.load_data := Mux(s2_dcacheResp.bits.miss, 0.U, s2_dcacheResp.bits.load_data) //to cut X-prop
  s2_loadDataFromDcache.forwardMask := s2_forwardMask
  s2_loadDataFromDcache.forwardData := s2_forwardData
  s2_loadDataFromDcache.uop := s2_out.bits.uop
  s2_loadDataFromDcache.addrOffset := s2_paddr(2, 0)

  val s2_can_replay_from_fetch = !s2_mmio && !s2_is_prefetch && !s2_tlb_miss

  val s2_lpvCancel = s2_in.valid && (s2_tlb_miss || s2_mmio || s2_LSQ_LoadForwardQueryIO.dataInvalid || s2_cache_miss)
  s2_out.bits.mmio := s2_mmio && s2_enableMem
  s2_out.bits.uop.ctrl.flushPipe := false.B ///flushPipe logic is useless
  s2_out.bits.uop.cf.exceptionVec := s2_exception_vec // cache error not included

  val s2_dataForwarded = s2_cache_miss && !s2_exception &&
    (s2_fullForward || io.csrCtrl.cache_error_enable && s2_cache_tag_error)
  // io.out.bits.forwardX will be send to lq
  s2_out.bits.forwardMask := s2_forwardMask
  // data from dcache is not included in io.out.bits.forwardData
  s2_out.bits.forwardData := s2_forwardData

  s2_in.ready := s2_out.ready || !s2_in.valid

  val s2_need_replay_from_rs = Wire(Bool())

//  s2_need_replay_from_rs := s2_tlb_miss || // replay if dtlb miss
//      s2_cache_replay && !s2_is_prefetch && !s2_mmio && !s2_exception && !s2_dataForwarded || // replay if dcache miss queue full / busy
//      s2_data_invalid && !s2_is_prefetch // replay if store to load forward data is not ready

  s2_need_replay_from_rs := s2_tlb_miss || // replay if dtlb miss
    s2_cache_miss && !s2_is_prefetch && !s2_mmio && !s2_exception && !s2_dataForwarded || // replay if dcache miss queue full / busy
    s2_data_invalid && !s2_is_prefetch // replay if store to load forward data is not ready


  val s2_rsFeedback = Wire(ValidIO(new RSFeedback))
//  s2_rsFeedback.valid := Mux(s2_in.valid && RegNext(s1_bank_conflict,false.B),false.B,s2_in.valid && (s2_need_replay_from_rs && !s2_in.bits.isReplayQReplay || ((io.s3_enq_replqQueue.ready && !s2_out.bits.isReplayQReplay))) && s2_enableMem)
//  s2_rsFeedback.valid := s2_in.valid && Mux(s2_in.bits.isReplayQReplay,false.B,
//                                            Mux(RegNext(s1_bank_conflict,false.B),false.B,
//                                                s2_need_replay_from_rs || !io.s3_enq_replqQueue.ready))

  s2_rsFeedback.valid := s2_in.valid && Mux(s2_in.bits.isReplayQReplay, false.B, true.B)
  s2_rsFeedback.bits.rsIdx := s2_in.bits.rsIdx
//  s2_rsFeedback.bits.sourceType := Mux(s2_tlb_miss, RSFeedbackType.tlbMiss,
//    Mux(s2_cache_replay,
//      RSFeedbackType.mshrFull,
//      RSFeedbackType.dataInvalid
//    )
//  )
  s2_rsFeedback.bits.sourceType := Mux(!io.s3_enq_replqQueue.ready, RSFeedbackType.replayQFull,RSFeedbackType.success)

  val s2_dcache_require_replay = s2_cache_replay &&
    s2_rsFeedback.valid &&
    !s2_dataForwarded &&
    !s2_is_prefetch &&
    s2_out.bits.miss

  io.tlb.req_kill := false.B
  io.dcache.s1_kill := s1_dcacheKill
  assert(s1_in.ready)

  // provide paddr for lq
  io.lsq.s1_lduUpdateLQ.valid := s1_out.valid
  io.lsq.s1_lduUpdateLQ.bits.lqIdx := s1_out.bits.uop.lqIdx
  io.lsq.s1_lduUpdateLQ.bits.paddr := s1_paddr_dup_lsu

  PipelineConnect(s1_out, s2_in, true.B, s1_out.bits.uop.robIdx.needFlush(io.redirect))

  // load s2
//  io.s2IsPointerChasing := DontCare
  io.prefetch_train.bits := s2_in.bits
  // override miss bit
  io.prefetch_train.bits.miss := io.dcache.resp.bits.miss
  io.prefetch_train.valid := s2_in.fire && !s2_out.bits.mmio && !s2_in.bits.tlbMiss
  io.dcache.s2_kill := s2_dcache_kill // to kill mmio resp which are redirected

  io.lsq.s2_load_data_forwarded := s2_dataForwarded

  assert(s2_in.ready)

  // feedback bank conflict / ld-vio check struct hazard to rs
//  io.feedbackFast.valid := RegNext(s1_rsFeedback.valid && !s1_out.bits.uop.robIdx.needFlush(io.redirect), false.B)
//  io.feedbackFast.bits := RegNext(s1_rsFeedback.bits) //remove clock-gating for timing
  io.feedbackFast.valid := false.B
  io.feedbackFast.bits := DontCare


  // pre-calcuate sqIdx mask in s0, then send it to lsq in s1 for forwarding
  val sqIdxMaskReg = RegEnable(UIntToMask(s0_in.bits.uop.sqIdx.value, StoreQueueSize), s0_in.valid)
  // to enable load-load, sqIdxMask must be calculated based on rsIssueIn.uop
  // If the timing here is not OK, load-load forwarding has to be disabled.
  // Or we calculate sqIdxMask at RS??
  io.lsq.forwardFromSQ.sqIdxMask := sqIdxMaskReg

  // writeback to LSQ
  // Current dcache use MSHR
  // Load queue will be updated at s2 for both hit/miss int/fp load
  io.lsq.s2_lduUpdateLQ.valid := s2_out.valid
  // generate LqWriteBundle from LsPipelineBundle
  io.lsq.s2_lduUpdateLQ.bits.fromLsPipelineBundle(s2_out.bits)
  // generate duplicated load queue data wen
  val load_s2_valid_vec = RegInit(0.U(6.W))
  val load_s2_leftFire = s1_out.valid && s2_in.ready
  load_s2_valid_vec := 0x0.U(6.W)
  when (load_s2_leftFire) { load_s2_valid_vec := 0x3f.U(6.W)}
  when (s1_out.bits.uop.robIdx.needFlush(io.redirect)) { load_s2_valid_vec := 0x0.U(6.W) }
  assert(RegNext(s2_in.valid === load_s2_valid_vec(0)))
  io.lsq.s2_lduUpdateLQ.bits.lq_data_wen_dup := load_s2_valid_vec.asBools

  // s2_dcache_require_replay signal will be RegNexted, then used in s3
  io.lsq.s2_dcache_require_replay := s2_dcache_require_replay

  // write to rob and writeback bus
  val s2_wb_valid = s2_out.valid && !s2_tlb_miss && !s2_data_invalid && !s2_cancel_inner && !s2_out.bits.miss &&
      !s2_out.bits.mmio &&
      !s2_out.bits.uop.robIdx.needFlush(io.redirect)

  // Int load, if hit, will be writebacked at s2
  val hitLoadOut = Wire(Valid(new ExuOutput))
  hitLoadOut := DontCare
  hitLoadOut.valid := s2_wb_valid
  hitLoadOut.bits.uop := s2_out.bits.uop
  hitLoadOut.bits.data := s2_out.bits.data
  hitLoadOut.bits.redirectValid := false.B
  hitLoadOut.bits.redirect := DontCare
  hitLoadOut.bits.debug.isMMIO := s2_out.bits.mmio
  hitLoadOut.bits.debug.isPerfCnt := false.B
  hitLoadOut.bits.debug.paddr := s2_out.bits.paddr
  hitLoadOut.bits.debug.vaddr := s2_out.bits.vaddr
  hitLoadOut.bits.fflags := DontCare

  s2_out.ready := true.B

  // load s3
//  val s3_load_wb_meta_reg = RegEnable(Mux(hitLoadOut.valid, hitLoadOut.bits, io.lsq.s3_lq_wb.bits), hitLoadOut.valid | io.lsq.s3_lq_wb.valid)
//  val s3_load_wb_meta_reg = RegEnable(hitLoadOut.bits, hitLoadOut.valid)
  val s3_load_wb_meta_reg = RegEnable(Mux(hitLoadOut.valid,hitLoadOut.bits,io.mmioWb.bits), hitLoadOut.valid | io.mmioWb.valid)

  // data from load queue refill
//  val s3_loadDataFromLQ = RegEnable(io.lsq.s3_lq_wbLdRawData, io.lsq.s3_lq_wb.valid)
//  val s3_rdataLQ = s3_loadDataFromLQ.mergedData()
  val s3_mmioDataFromLq = RegEnable(io.mmioWb.bits.data, io.mmioWb.valid)
//  val s3_mmioData = LookupTree()

  // data from dcache hit
  val s3_loadDataFromDcache = RegEnable(s2_loadDataFromDcache, s2_in.valid)
  val s3_rdataDcache = s3_loadDataFromDcache.mergedData()

  private val hitLoadOutValidReg = RegNext(hitLoadOut.valid, false.B)
//  val hitLoadOutValidReg_dup = Seq.fill(8)(RegNext(hitLoadOut.valid, false.B))

//  val s3_uop = Mux(hitLoadOutValidReg,s3_loadDataFromDcache.uop,s3_loadDataFromLQ.uop)
//  val s3_offset = Mux(hitLoadOutValidReg,s3_loadDataFromDcache.addrOffset,s3_loadDataFromLQ.addrOffset)
  val s3_uop = s3_loadDataFromDcache.uop
  val s3_offset = s3_loadDataFromDcache.addrOffset

  val s3_rdata_dup = WireInit(VecInit(List.fill(8)(0.U(64.W))))
  s3_rdata_dup.zipWithIndex.foreach({case(d,i) => {
//    d := Mux(hitLoadOutValidReg_dup(i),s3_rdataDcache,s3_rdataLQ)
    d := s3_rdataDcache
  }})

  val s3_sel_rdata = LookupTree(s3_offset,List(
    "b000".U -> s3_rdata_dup(0)(63, 0),
    "b001".U -> s3_rdata_dup(1)(63, 8),
    "b010".U -> s3_rdata_dup(2)(63, 16),
    "b011".U -> s3_rdata_dup(3)(63, 24),
    "b100".U -> s3_rdata_dup(4)(63, 32),
    "b101".U -> s3_rdata_dup(5)(63, 40),
    "b110".U -> s3_rdata_dup(6)(63, 48),
    "b111".U -> s3_rdata_dup(7)(63, 56)
  ))
  val s3_rdataPartialLoad = rdataHelper(s3_uop,s3_sel_rdata)


//  private val lsqOutputValidReg = RegNext(io.lsq.s3_lq_wb.valid && (!io.lsq.s3_lq_wb.bits.uop.robIdx.needFlush(io.redirect)),false.B)
  private val s3_lsqMMIOOutputValid = RegNext(io.mmioWb.valid && (!io.mmioWb.bits.uop.robIdx.needFlush(io.redirect)),false.B)
//  io.ldout.valid := hitLoadOutValidReg || lsqOutputValidReg
  io.ldout.valid := hitLoadOutValidReg || s3_lsqMMIOOutputValid
  io.ldout.bits := s3_load_wb_meta_reg
  when(hitLoadOutValidReg) {
    io.ldout.bits.data := s3_rdataPartialLoad
  }
  io.ldout.bits.uop.cf.exceptionVec(loadAccessFault) := s3_load_wb_meta_reg.uop.cf.exceptionVec(loadAccessFault) //||

  // feedback tlb miss / dcache miss queue full
//  io.feedbackSlow.valid := RegNext(s2_out.valid &&(!s2_out.bits.isReplayQReplay) && !s2_out.bits.uop.robIdx.needFlush(io.redirect), false.B)
//  io.feedbackSlow.bits.rsIdx := RegNext(s2_rsFeedback.bits.rsIdx)
//  io.feedbackSlow.bits.sourceType := RegNext(Mux(s2_rsFeedback.valid,s2_rsFeedback.bits.sourceType,RSFeedbackType.success))


  io.feedbackSlow.valid := RegNext(s2_rsFeedback.valid && !s2_out.bits.uop.robIdx.needFlush(io.redirect), false.B)
  io.feedbackSlow.bits.rsIdx := RegNext(s2_rsFeedback.bits.rsIdx)
  io.feedbackSlow.bits.sourceType := RegNext(s2_rsFeedback.bits.sourceType)


  // If replay is reported at load_s1, inst will be canceled (will not enter load_s2),
  // in that case:
  // * replay should not be reported twice
  assert(!(RegNext(io.feedbackFast.valid) && io.feedbackSlow.valid))
  // * io.fastUop.valid should not be reported
//  assert(!RegNext(io.feedbackFast.valid && io.fastUop.valid))

  // load forward_fail/ldld_violation check
  // check for inst in load pipeline
  val s3_forward_fail = RegNext(io.lsq.forwardFromSQ.matchInvalid || io.forwardFromSBuffer.matchInvalid)
  val s3_ldld_violation = RegNext(
    io.lsq.loadViolationQuery.s2_resp.valid &&
    io.lsq.loadViolationQuery.s2_resp.bits.have_violation &&
    RegNext(io.csrCtrl.ldld_vio_check_enable)
  )
  val s3_need_replay_from_fetch = s3_forward_fail || s3_ldld_violation
  val s3_can_replay_from_fetch = RegEnable(s2_can_replay_from_fetch, s2_out.valid)


  // 1) use load pipe check result generated in load_s3 iff load_hit
  when (RegNext(hitLoadOut.valid)) {
    io.ldout.bits.uop.ctrl.replayInst := s3_need_replay_from_fetch
  }
  // 2) otherwise, write check result to load queue
  io.lsq.s3_replay_from_fetch := s3_need_replay_from_fetch && s3_can_replay_from_fetch

  // s3_delayed_load_error path is not used for now, as we writeback load result in load_s3
  // but we keep this path for future use
  io.s3_delayed_load_error := false.B
  io.lsq.s3_delayed_load_error := false.B //load_s2.io.s3_delayed_load_error

//  io.lsq.s3_lq_wb.ready := !hitLoadOut.valid
  io.mmioWb.ready := !hitLoadOut.valid

  val lastValidData = RegEnable(io.ldout.bits.data, io.ldout.fire)
  val hitLoadAddrTriggerHitVec = Wire(Vec(TriggerNum, Bool()))
  val lqLoadAddrTriggerHitVec = io.lsq.trigger.lqLoadAddrTriggerHitVec
  (0 until TriggerNum).foreach{ i => {
    val tdata2 = io.trigger(i).tdata2
    val matchType = io.trigger(i).matchType
    val tEnable = io.trigger(i).tEnable

    hitLoadAddrTriggerHitVec(i) := TriggerCmp(s2_out.bits.vaddr, tdata2, matchType, tEnable)
    io.trigger(i).addrHit := RegNext(Mux(hitLoadOut.valid, hitLoadAddrTriggerHitVec(i), lqLoadAddrTriggerHitVec(i)))
    io.trigger(i).lastDataHit := TriggerCmp(lastValidData, tdata2, matchType, tEnable)
  }}
  io.lsq.trigger.hitLoadAddrTriggerHitVec := hitLoadAddrTriggerHitVec

  private val s1_cancel = RegInit(false.B)
  s1_cancel := s1_in.valid && (!s1_out.valid)
  io.cancel := s1_cancel || s2_lpvCancel

//  val temp_replay_record_s1 = s1_rsFeedback.valid && !s1_out.bits.uop.robIdx.needFlush(io.redirect)
//  val temp_replay_record_s1_reg = RegNext(temp_replay_record_s1,false.B)
//  val temp_replay_record_s2 = s2_rsFeedback.valid && !s2_out.bits.uop.robIdx.needFlush(io.redirect)
//  val replayHasOtherCause = (temp_replay_record_s2 || temp_replay_record_s1_reg) && s2_out.bits.isReplayQReplay

  val hasOtherCause = s2_out.valid && (s2_need_replay_from_rs)
  //tmp:
  when(s2_out.valid){
    assert(PopCount(s2_out.bits.replayCause.asUInt) <= 1.U)
  }
//  val s2_needEnqReplayQ = s2_out.valid && (replayHasOtherCause || s2_out.bits.replayCause.reduce(_|_))
//  val s2_needEnqReplayQ = s2_out.valid && (replayHasOtherCause ||
//                                          !s2_out.bits.isReplayQReplay && s2_out.bits.replayCause.reduce(_|_))

  //tmp: use S2
  io.s3_enq_replqQueue.valid := s2_out.valid && !s2_out.bits.uop.robIdx.needFlush(io.redirect)
  io.s3_enq_replqQueue.bits.vaddr := s2_out.bits.vaddr
  io.s3_enq_replqQueue.bits.paddr := DontCare
  io.s3_enq_replqQueue.bits.isReplayQReplay := s2_out.bits.isReplayQReplay
  io.s3_enq_replqQueue.bits.replayCause := DontCare
//  io.s3_enq_replqQueue.bits.replayCause(LoadReplayCauses.C_BC) := s2_out.bits.replayCause(LoadReplayCauses.C_BC) || hasOtherCause
  io.s3_enq_replqQueue.bits.replayCause(LoadReplayCauses.C_BC) := Mux(s2_wb_valid,false.B,true.B)
  io.s3_enq_replqQueue.bits.schedIndex := s2_out.bits.schedIndex
  io.s3_enq_replqQueue.bits.uop := s2_out.bits.uop
  io.s3_enq_replqQueue.bits.mask := s2_mask
  io.s3_enq_replqQueue.bits.tlbMiss := false.B
  assert(!(hitLoadOut.valid && io.s3_enq_replqQueue.bits.replayCause.reduce(_|_)),"when load wb,replayCause must be 0!!")


  val perfEvents = Seq(
    ("load_s0_in_fire         ", s0_in.fire),
    ("load_to_load_forward    ", s1_out.valid),
    ("stall_dcache            ", s0_out.valid && s0_out.ready && !s0_req_dcache.ready),
    ("load_s1_in_fire         ", s1_in.fire),
    ("load_s1_tlb_miss        ", s1_in.fire && s1_dtlbResp.bits.miss),
    ("load_s2_in_fire         ", s2_in.fire),
    ("load_s2_dcache_miss     ", s2_in.fire && s2_dcacheResp.bits.miss),
    ("load_s2_replay          ", s2_rsFeedback.valid),
    ("load_s2_replay_tlb_miss ", s2_rsFeedback.valid && s2_in.bits.tlbMiss),
    ("load_s2_replay_cache    ", s2_rsFeedback.valid && !s2_in.bits.tlbMiss && s2_dcacheResp.bits.miss),
  )
  generatePerfEvent()

  when(io.ldout.fire){
    XSDebug("ldout %x\n", io.ldout.bits.uop.cf.pc)
  }

  val debugModule = Module(new LoadUnitDebugInfo)
  debugModule.io.infoIn.s0_valid := s0_in.valid
  debugModule.io.infoIn.s1_valid := s1_in.valid
  debugModule.io.infoIn.s2_valid := s2_in.valid
//  debugModule.io.infoIn.s3_valid :=
  debugModule.io.infoIn.wb_valid := io.ldout.valid
  debugModule.io.infoIn.rsIdx := io.rsIdx
  debugModule.io.infoIn.robIdx := s0_in.bits.uop.robIdx
}

class LdDebugBundle(implicit p: Parameters) extends XSBundle {
  val s0_valid = Bool()
  val s1_valid = Bool()
  val s2_valid = Bool()
//  val s3_valid = Bool()
  val wb_valid = Bool()
  val rsIdx = new RsIdx()
  val robIdx = new RobPtr()
}

class LoadUnitDebugInfo(implicit p: Parameters) extends XSModule{
  val io = IO(new Bundle(){
    val infoIn = Input(new LdDebugBundle)
  })

  val debugReg = RegInit(0.U.asTypeOf(new LdDebugBundle))
  debugReg := io.infoIn
  dontTouch(debugReg)
}

