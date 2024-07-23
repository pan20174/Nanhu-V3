/***************************************************************************************
 * Copyright (c) 2020-2023 Institute of Computing Technology, Chinese Academy of Sciences
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
/***************************************************************************************
 * Author: Liang Sen
 * E-mail: liangsen20z@ict.ac.cn
 * Date: 2023-06-19
 ****************************************************************************************/
package xiangshan.backend.issue.MemRs

import chisel3._
import chisel3.util._
import chisel3.experimental.prefix

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy.{LazyModule, LazyModuleImp, ValName}

import utils._
import xs.utils._
import xs.utils.perf._

import xiangshan.{ExuOutput, FuType, HasXSParameter, MicroOp, Redirect, SrcState, SrcType, XSCoreParamsKey}
import xiangshan.backend.rename.BusyTable
import xiangshan.backend.issue._
import xiangshan.backend.execute.exu.ExuType
import xiangshan.backend.execute.fu.FuConfigs
import xiangshan.backend.writeback.{WriteBackSinkNode, WriteBackSinkParam, WriteBackSinkType}
import xiangshan.backend.rob.RobPtr
import xiangshan.mem.SqPtr

class IssueInfo(bankIdxWidth:Int, entryIdxWidth:Int)(implicit p: Parameters) extends Bundle{
  val entryIdxOH = UInt(entryIdxWidth.W)
  val bankIdxOH = UInt(bankIdxWidth.W)
}

object MemRsHelper {
  def WbToWkp(in:Valid[ExuOutput], p:Parameters):Valid[WakeUpInfo] = {
    val wkp = Wire(Valid(new WakeUpInfo()(p)))
    wkp.valid := in.valid && in.bits.wakeupValid
    wkp.bits.pdest := in.bits.uop.pdest
    wkp.bits.robPtr := in.bits.uop.robIdx
    wkp.bits.lpv := 0.U.asTypeOf(wkp.bits.lpv)
    wkp.bits.destType := MuxCase(SrcType.default, Seq(
      in.bits.uop.ctrl.rfWen -> SrcType.reg,
      in.bits.uop.ctrl.fpWen -> SrcType.fp,
      in.bits.uop.ctrl.vdWen -> SrcType.vec
    ))
    wkp
  }
}

class MemoryReservationStation(implicit p: Parameters) extends LazyModule{
  private val entryNum = p(XSCoreParamsKey).memRsDepth
  private val wbNodeParam = WriteBackSinkParam(name = "Memory RS", sinkType = WriteBackSinkType.memRs)
  private val rsParam = RsParam(name = "Memory RS", RsType.mem, entryNum)
  require(entryNum % rsParam.bankNum == 0)
  val issueNode = new RsIssueNode(rsParam)
  val wakeupNode = new WriteBackSinkNode(wbNodeParam)
  val dispatchNode = new RsDispatchNode(rsParam)

  lazy val module = new MemoryReservationStationImpl(this, rsParam)
}

class MemoryReservationStationImpl(outer:MemoryReservationStation, param:RsParam) extends LazyModuleImp(outer)
  with HasXSParameter with HasPerfLogging{
  require(param.bankNum == 4)
  require(param.entriesNum % param.bankNum == 0)
  private val rawIssue = outer.issueNode.out.head._1 zip outer.issueNode.out.head._2._2
  private val wbIn = outer.wakeupNode.in.head
  private val wakeup = wbIn._1.zip(wbIn._2._1)
  rawIssue.foreach(elm => elm._2.exuConfigs.foreach(elm0 => require(ExuType.memTypes.contains(elm0.exuType))))
  private val issue = rawIssue.filterNot(_._2.isSpecialLoad)
  private val stIssue = rawIssue.filter(_._2.isSta)
  private val ldIssue = rawIssue.filter(_._2.isLdu)
//  private val specialLoadIssue = rawIssue.filter(_._2.isSpecialLoad)
  println("\nMemory Reservation Issue Ports Config:")
  for ((iss, issuePortIdx) <- rawIssue.zipWithIndex) {
    println(s"Issue Port $issuePortIdx ${iss._2}")
  }

  private val staIssue = issue.filter(_._2.hasSta)
  private val stdIssue = issue.filter(_._2.hasStd)
  private val lduIssue = issue.filter(_._2.hasLoad)

  private val staIssuePortNum = issue.count(_._2.hasSta)
  private val stdIssuePortNum = issue.count(_._2.hasStd)
  private val lduIssuePortNum = issue.count(_._2.hasLoad)

  require(staIssuePortNum == stdIssuePortNum)
  require(staIssue.nonEmpty && staIssue.length <= param.bankNum && (param.bankNum % staIssue.length) == 0)
  require(stdIssue.nonEmpty && stdIssue.length <= param.bankNum && (param.bankNum % stdIssue.length) == 0)
  require(lduIssue.nonEmpty && lduIssue.length <= param.bankNum && (param.bankNum % lduIssue.length) == 0)

  private val entriesNumPerBank = param.entriesNum / param.bankNum

  val io = IO(new Bundle{
    val redirect = Input(Valid(new Redirect))
    val mulSpecWakeup = Input(Vec(p(XSCoreParamsKey).exuParameters.mulNum, Valid(new WakeUpInfo)))
    val aluJmpSpecWakeup = Input(Vec(p(XSCoreParamsKey).exuParameters.aluNum + p(XSCoreParamsKey).exuParameters.JmpCnt, Valid(new WakeUpInfo)))
    val fmaSpecWakeup = Input(Vec(p(XSCoreParamsKey).exuParameters.fmaNum, Valid(new WakeUpInfo)))
    val loadEarlyWakeup = Output(Vec(loadUnitNum, Valid(new EarlyWakeUpInfo)))
    val earlyWakeUpCancel = Input(Vec(loadUnitNum, Bool()))
    val stLastCompelet = Input(new SqPtr)
    val integerAllocPregs = Vec(RenameWidth, Flipped(ValidIO(UInt(PhyRegIdxWidth.W))))
    val floatingAllocPregs = Vec(RenameWidth, Flipped(ValidIO(UInt(PhyRegIdxWidth.W))))
    val vectorAllocPregs = Vec(coreParams.vectorParameters.vRenameWidth, Flipped(ValidIO(UInt(PhyRegIdxWidth.W))))
    val ldStopMemRS = Input(Vec(LoadPipelineWidth, Bool()))
    val lduEarlyWakeUpIn = Input(Vec(loadUnitNum, Valid(new EarlyWakeUpInfo)))
    val fromMemBlkInfo = new Bundle() {
      val ldValidNum = Input(UInt())
      val replayQFreeNum = Input(UInt(log2Up(LoadReplayQueueSize).W))
    }
  })
  require(outer.dispatchNode.in.length == 1)
  private val replayPortNum = 6

  private val enq = outer.dispatchNode.in.map(_._1).head

  private val aluJmpSpecWakeup = Wire(Vec(io.aluJmpSpecWakeup.length, Valid(new WakeUpInfo)))
  aluJmpSpecWakeup.zip(io.aluJmpSpecWakeup).foreach({case(a, b) =>
    val canceled = b.bits.lpv.zip(io.earlyWakeUpCancel).map({case(l,c) => l(0) && c}).reduce(_||_)
    a.valid := b.valid && !canceled
    a.bits := b.bits
    a.bits.lpv.foreach(_ := 0.U)
  })
  private val fmaSpecWakeup = io.fmaSpecWakeup.map(Pipe(_))
  private val allWkpIns = wakeup.map(w => (MemRsHelper.WbToWkp(w._1, p), w._2))
  private val wkpToRsBank = allWkpIns.map(_._1) ++ fmaSpecWakeup ++ io.mulSpecWakeup ++ aluJmpSpecWakeup
  private val wbWkpLen = allWkpIns.length

  private val regWkpIdx = allWkpIns.zipWithIndex.filter(e => e._1._2.writeIntRf && !e._1._2.isVldu).map(_._2) ++
    Seq.tabulate((io.mulSpecWakeup ++ aluJmpSpecWakeup).length)(_ + wbWkpLen + fmaSpecWakeup.length)

  private val fpWkpIdx = allWkpIns.zipWithIndex.filter(e => e._1._2.writeFpRf && !e._1._2.isVldu).map(_._2) ++
    Seq.tabulate((fmaSpecWakeup ++ io.mulSpecWakeup).length)(_ + wbWkpLen)

  private val vecWkpIdx = allWkpIns.zipWithIndex.filter(e => e._1._2.writeVecRf && e._1._2.throughVectorRf).map(_._2)

  private val stIssuedWires = Wire(Vec(staIssuePortNum, Valid(new RobPtr)))

  private val rsBankSeq = Seq.tabulate(param.bankNum)( _ => {
    val mod = Module(new MemoryReservationBank(entriesNumPerBank,
      staIssuePortNum,
      wkpToRsBank.length,
      regWkpIdx,
      fpWkpIdx,
      vecWkpIdx,
      replayPortNum))

    mod.io.redirect := io.redirect
    mod.io.wakeups := wkpToRsBank
    mod.io.earlyWakeUpCancel := io.earlyWakeUpCancel
    mod.io.stIssued := stIssuedWires
    mod.io.stLastCompelet := RegNext(io.stLastCompelet)
    mod
  })

//  private val internalEarlyWakeup = Wire(Vec(loadUnitNum, Valid(new EarlyWakeUpInfo)))
//  io.loadEarlyWakeup.zip(internalEarlyWakeup).foreach({case(a, b) =>
//    a.valid := b.valid
//    a.bits := b.bits
//  })

  private val loadUnitEarlyWakeup = io.lduEarlyWakeUpIn
  //wake up other rs
  io.loadEarlyWakeup.zip(loadUnitEarlyWakeup).foreach({ case (a, b) =>
    a.valid := b.valid
    a.bits := b.bits
  })

  //wake up memRs
  rsBankSeq.foreach(rb => {
    rb.io.loadEarlyWakeup.zip(loadUnitEarlyWakeup).foreach({case(a, b) =>
      a.valid := b.valid && b.bits.destType === SrcType.reg
      a.bits := b.bits
    })
  })

  private val allocateNetwork = Module(new AllocateNetwork(param.bankNum, entriesNumPerBank, Some("MemAllocNetwork")))

  private val regWkpIns = wakeup.filter(_._2.writeIntRf).map(_._1).map(MemRsHelper.WbToWkp(_, p))
  private val fpWkpIns = wakeup.filter(_._2.writeFpRf).map(_._1).map(MemRsHelper.WbToWkp(_, p))
  private val vecWkpIns = wakeup.filter(e => e._2.writeVecRf && e._2.throughVectorRf).map(_._1).map(MemRsHelper.WbToWkp(_, p))

  private val floatingBusyTable = Module(new BusyTable(NRPhyRegs, param.bankNum, (fpWkpIns ++ io.mulSpecWakeup ++ io.fmaSpecWakeup).length, RenameWidth))
  floatingBusyTable.io.allocPregs := io.floatingAllocPregs
  floatingBusyTable.io.wbPregs.zip(fpWkpIns ++ io.mulSpecWakeup ++ fmaSpecWakeup).foreach({ case (bt, wb) =>
    bt.valid := wb.valid && wb.bits.destType === SrcType.fp
    bt.bits := wb.bits.pdest
  })
  private val integerBusyTable = Module(new BusyTable(NRPhyRegs, param.bankNum * 2, regWkpIns.length + io.mulSpecWakeup.length + aluJmpSpecWakeup.length, RenameWidth))
  integerBusyTable.io.allocPregs := io.integerAllocPregs
  integerBusyTable.io.wbPregs.zip(regWkpIns ++ aluJmpSpecWakeup ++ io.mulSpecWakeup).foreach({ case (bt, wb) =>
    bt.valid := wb.valid && wb.bits.destType === SrcType.reg
    bt.bits := wb.bits.pdest
  })
  private val vectorRfSize = coreParams.vectorParameters.vPhyRegsNum
  private val vRenameWidth = coreParams.vectorParameters.vRenameWidth
  private val vectorBusyTable = Module(new BusyTable(vectorRfSize, param.bankNum * 3, vecWkpIns.length + io.mulSpecWakeup.length, vRenameWidth))
  vectorBusyTable.io.allocPregs := io.vectorAllocPregs
  vectorBusyTable.io.wbPregs.zip(vecWkpIns ++ io.mulSpecWakeup).foreach({ case (bt, wb) =>
    bt.valid := wb.valid && wb.bits.destType === SrcType.vec
    bt.bits := wb.bits.pdest
  })

  private val staExuCfg = staIssue.flatMap(_._2.exuConfigs).filter(_.exuType == ExuType.sta).head
  private val stdExuCfg = stdIssue.flatMap(_._2.exuConfigs).filter(_.exuType == ExuType.std).head
  private val lduExuCfg = lduIssue.flatMap(_._2.exuConfigs).filter(_.exuType == ExuType.ldu).head

  private val staSelectNetwork = Module(new HybridSelectNetwork(param.bankNum, entriesNumPerBank, staIssuePortNum, staExuCfg, true, Some(s"MemStaSelNetwork")))
  private val stdSelectNetwork = Module(new HybridSelectNetwork(param.bankNum, entriesNumPerBank, stdIssuePortNum, stdExuCfg, true, Some(s"MemStdSelNetwork")))
  private val lduSelectNetwork = Module(new HybridSelectNetwork(param.bankNum, entriesNumPerBank, lduIssuePortNum, lduExuCfg, true, Some(s"MemLduSelNetwork")))

  private val uopReadNum = stIssue.length + ldIssue.length
  require(uopReadNum == 4)

  staSelectNetwork.io.selectInfo.zip(rsBankSeq).foreach({ case (sink, source) =>
    sink := source.io.staSelectInfo
  })
  staSelectNetwork.io.earlyWakeUpCancel := io.earlyWakeUpCancel
  staSelectNetwork.io.redirect := io.redirect

  stdSelectNetwork.io.selectInfo.zip(rsBankSeq).foreach({ case (sink, source) =>
    sink := source.io.stdSelectInfo
  })
  stdSelectNetwork.io.earlyWakeUpCancel := io.earlyWakeUpCancel
  stdSelectNetwork.io.redirect := io.redirect

  lduSelectNetwork.io.selectInfo.zip(rsBankSeq).foreach({ case (sink, source) =>
    sink := source.io.lduSelectInfo
  })
  lduSelectNetwork.io.earlyWakeUpCancel := io.earlyWakeUpCancel
  lduSelectNetwork.io.redirect := io.redirect

  private var fpBusyTableReadIdx = 0
  private var intBusyTableReadIdx = 0
  private var vectorBusyTableReadIdx = 0
  allocateNetwork.io.enqFromDispatch.zip(enq).foreach({case(sink, source) =>
    val fdRead = floatingBusyTable.io.read(fpBusyTableReadIdx)
    val baseRead = integerBusyTable.io.read(intBusyTableReadIdx)
    val strdOrIdRead = integerBusyTable.io.read(intBusyTableReadIdx + 1)
    val offRead = vectorBusyTable.io.read(vectorBusyTableReadIdx + 0)
    val vdRead = vectorBusyTable.io.read(vectorBusyTableReadIdx + 1)
    val vmRead = vectorBusyTable.io.read(vectorBusyTableReadIdx + 2)
    val type0 = source.bits.ctrl.srcType(0)
    val type1 = source.bits.ctrl.srcType(1)
    sink.valid := source.valid
    sink.bits := source.bits
    source.ready := sink.ready

    baseRead.req := source.bits.psrc(0)
    sink.bits.srcState(0) := baseRead.resp

    strdOrIdRead.req := source.bits.psrc(1)
    fdRead.req := source.bits.psrc(1)
    offRead.req := source.bits.psrc(1)
    sink.bits.srcState(1) := MuxCase(SrcState.rdy, Seq(
      (type1 === SrcType.reg) -> strdOrIdRead.resp,
      (type1 === SrcType.fp) -> fdRead.resp,
      (type1 === SrcType.vec) -> offRead.resp,
    ))

    vdRead.req := source.bits.psrc(2)
    sink.bits.srcState(2) := vdRead.resp

    vmRead.req := source.bits.vm
    sink.bits.vmState := vmRead.resp

    fpBusyTableReadIdx = fpBusyTableReadIdx + 1
    intBusyTableReadIdx = intBusyTableReadIdx + 2
    vectorBusyTableReadIdx = vectorBusyTableReadIdx + 3
    //assert(type0 === SrcType.reg)
    when(source.valid) {
      assert(FuType.memoryTypes.map(_ === source.bits.ctrl.fuType).reduce(_||_))
    }
  })

  private val timer = GTimer()
  for(((fromAllocate, toAllocate), rsBank) <- allocateNetwork.io.enqToRs
    .zip(allocateNetwork.io.allocVec)
    .zip(rsBankSeq)){
    toAllocate := rsBank.io.alloc
    rsBank.io.enq.valid := fromAllocate.valid && !io.redirect.valid
    rsBank.io.enq.bits := fromAllocate.bits
    rsBank.io.enq.bits.debugInfo.enqRsTime := timer + 1.U
  }


  private val loadIssResps = Wire(Vec(lduIssuePortNum, Decoupled(new SelectResp(param.bankNum, entriesNumPerBank))))
  private val ldIssuePayloads = Wire(Vec(ldIssue.length, new MicroOp))
  private val ldFastReleaseRsEntry = Wire(Vec(ldIssue.length, ValidIO(new RSFeedback)))
  private var uopReadPortIdx = 0

  for ((iss, ldIssuePortIdx) <- ldIssue.zipWithIndex) {
    prefix(iss._2.name + "_" + iss._2.id) {
      val loadIssueDriver = Module(new MemoryIssuePipelineBlock(1, param.bankNum, entriesNumPerBank, true))
      loadIssueDriver.io.redirect := io.redirect
      loadIssueDriver.io.earlyWakeUpCancel := io.earlyWakeUpCancel

      val scalarLoadSel = lduSelectNetwork.io.issueInfo(ldIssuePortIdx).valid
      loadIssResps(ldIssuePortIdx).valid := scalarLoadSel
      loadIssResps(ldIssuePortIdx).bits := lduSelectNetwork.io.issueInfo(ldIssuePortIdx).bits
      loadIssResps(ldIssuePortIdx).ready := loadIssueDriver.io.enq.ready
      lduSelectNetwork.io.issueInfo(ldIssuePortIdx).ready := loadIssResps(ldIssuePortIdx).fire

      ldFastReleaseRsEntry(ldIssuePortIdx) := loadIssueDriver.io.earlyFeedback

      //todo
      def getSlice[T <: Object](in: Seq[T]): Seq[T] = in.slice(ldIssuePortIdx * 2, ldIssuePortIdx * 2 + 2)
      val selectedBanks = getSlice(rsBankSeq)

//      val selectedBanks = rsBankSeq
      val bankEns = getSlice(lduSelectNetwork.io.issueInfo(ldIssuePortIdx).bits.bankIdxOH.asBools).map(_ && loadIssueDriver.io.enq.fire)
      //      val bankPayloads = Wire(Vec(3, new MicroOp))
      for ((b, en) <- selectedBanks.zip(bankEns)) {
        b.io.loadIssue.valid := en && loadIssueDriver.io.enq.fire
        b.io.loadIssue.bits := lduSelectNetwork.io.issueInfo(ldIssuePortIdx).bits.entryIdxOH
        b.io.auxLoadIssValid := en && loadIssueDriver.io.enq.fire

        b.io.specialIssue.valid := false.B
        b.io.specialIssue.bits := DontCare

        b.io.auxSLoadIssValid := false.B
      }

      rsBankSeq.foreach(_.io.issueRead(uopReadPortIdx) := loadIssueDriver.io.deq.bits.entryIdxOH)
      ldIssuePayloads(ldIssuePortIdx) := Mux1H(loadIssueDriver.io.deq.bits.bankIdxOH, rsBankSeq.map(_.io.issueResp(uopReadPortIdx)))

      loadIssueDriver.io.enq.bits.chosen := DontCare

      lduSelectNetwork.io.issueInfo(ldIssuePortIdx).ready := loadIssueDriver.io.enq.ready
      loadIssueDriver.io.enq.valid := lduSelectNetwork.io.issueInfo(ldIssuePortIdx).valid
      loadIssueDriver.io.enq.bits.uop := ldIssuePayloads(ldIssuePortIdx)
      loadIssueDriver.io.enq.bits.selectResp := lduSelectNetwork.io.issueInfo(ldIssuePortIdx).bits
      loadIssueDriver.io.enq.bits.canFeedback := (io.fromMemBlkInfo.replayQFreeNum - io.fromMemBlkInfo.ldValidNum) >= 4.U

      iss._1.issue.valid := loadIssueDriver.io.deq.valid
      iss._1.issue.bits.uop := loadIssueDriver.io.deq.bits.uop
      iss._1.issue.bits.src := DontCare
      iss._1.rsIdx.bankIdxOH := loadIssueDriver.io.deq.bits.bankIdxOH
      iss._1.rsIdx.entryIdxOH := loadIssueDriver.io.deq.bits.entryIdxOH
      iss._1.hold := false.B
      iss._1.hasFeedback := loadIssueDriver.io.earlyFeedback.valid
      iss._1.auxValid := loadIssueDriver.io.deq.fire
      iss._1.specialPsrc := DontCare
      iss._1.specialPsrcType := DontCare
      iss._1.specialPsrcRen := false.B
      loadIssueDriver.io.deq.ready := iss._1.issue.ready
      loadIssueDriver.io.ldStop := io.ldStopMemRS(ldIssuePortIdx)

      uopReadPortIdx = uopReadPortIdx + 1
    }
  }


//  private val issBankNum = param.bankNum / issue.length
  private val issBankNum = 2 //todo
  private val issueDriverStdHasIssue = Wire(Vec(issBankNum,ValidIO(new IssueInfo(param.bankNum,param.entriesNum))))  //todo


  private val stIssuePayloads = Wire(Vec(stIssue.length, new MicroOp))
  for((iss, issuePortIdx) <- stIssue.zipWithIndex) {
    prefix(iss._2.name + "_" + iss._2.id) {
      val issueDriver = Module(new MemoryIssuePipelineBlock(3, param.bankNum, entriesNumPerBank, false))
      issueDriver.io.redirect := io.redirect
      issueDriver.io.earlyWakeUpCancel := io.earlyWakeUpCancel

      val respArbiter = Module(new SelectRespArbiter(param.bankNum, entriesNumPerBank, 2, true))
      respArbiter.io.in(0) <> stdSelectNetwork.io.issueInfo(issuePortIdx)
      respArbiter.io.in(1) <> staSelectNetwork.io.issueInfo(issuePortIdx)
//      respArbiter.io.in(2) <> lduSelectNetwork.io.issueInfo(issuePortIdx)
      XSPerfAccumulate(s"iss_${issuePortIdx}_${iss._2.name}_conflict", Cat(respArbiter.io.in.take(2).map(_.valid)).andR)
      XSPerfAccumulate(s"iss_${issuePortIdx}_${iss._2.name}_issue", respArbiter.io.out.fire)

      val selResp = respArbiter.io.out

      def getSlice[T <: Object](in: Seq[T]): Seq[T] = in.slice(issuePortIdx * issBankNum, issuePortIdx * issBankNum + issBankNum)

      val selectedBanks = getSlice(rsBankSeq)
      val bankEns = getSlice(selResp.bits.bankIdxOH.asBools).map(_ && selResp.fire)
      //      val bankPayloads = Wire(Vec(3, new MicroOp))
      for ((b, en) <- selectedBanks.zip(bankEns)) {
        b.io.staIssue.valid := en && respArbiter.io.in(1).fire
        b.io.auxStaIssValid := respArbiter.io.in(1).valid & !issueDriver.io.hold
        b.io.staIssue.bits := staSelectNetwork.io.issueInfo(issuePortIdx).bits.entryIdxOH

        b.io.stdIssue.valid := en && respArbiter.io.in(0).fire
        b.io.auxStdIssValid := respArbiter.io.in(0).valid & !issueDriver.io.hold
        b.io.stdIssue.bits := stdSelectNetwork.io.issueInfo(issuePortIdx).bits.entryIdxOH
      }

      rsBankSeq.foreach(_.io.issueRead(uopReadPortIdx) := issueDriver.io.deq.bits.entryIdxOH)
      stIssuePayloads(issuePortIdx) := Mux1H(issueDriver.io.deq.bits.bankIdxOH, rsBankSeq.map(_.io.issueResp(uopReadPortIdx)))

      issueDriver.io.enq.bits.chosen := respArbiter.io.chosen

      stIssuedWires(issuePortIdx).valid := issueDriver.io.deq.fire && issueDriver.io.deq.bits.uop.ctrl.fuType === FuType.stu
      stIssuedWires(issuePortIdx).bits := issueDriver.io.deq.bits.uop.robIdx

      selResp.ready := issueDriver.io.enq.ready
      issueDriver.io.enq.valid := selResp.valid
      issueDriver.io.enq.bits.uop := stIssuePayloads(issuePortIdx) //todo
      issueDriver.io.enq.bits.selectResp := selResp.bits
      issueDriver.io.enq.bits.canFeedback := false.B

      iss._1.issue.valid := issueDriver.io.deq.valid
      iss._1.issue.bits.uop := issueDriver.io.deq.bits.uop
      iss._1.issue.bits.src := DontCare
      iss._1.rsIdx.bankIdxOH := issueDriver.io.deq.bits.bankIdxOH
      iss._1.rsIdx.entryIdxOH := issueDriver.io.deq.bits.entryIdxOH
      iss._1.hold := issueDriver.io.hold
      iss._1.hasFeedback := false.B
      iss._1.auxValid := issueDriver.io.deq.fire
      iss._1.specialPsrc := DontCare
      iss._1.specialPsrcType := DontCare
      iss._1.specialPsrcRen := false.B
      issueDriver.io.deq.ready := iss._1.issue.ready
      issueDriver.io.ldStop := false.B

      issueDriverStdHasIssue(issuePortIdx).valid := issueDriver.io.deq.fire && issueDriver.io.deq.bits.uop.ctrl.fuType === FuType.std
      issueDriverStdHasIssue(issuePortIdx).bits.bankIdxOH := issueDriver.io.deq.bits.bankIdxOH
      issueDriverStdHasIssue(issuePortIdx).bits.entryIdxOH := issueDriver.io.deq.bits.entryIdxOH

      rsBankSeq.zipWithIndex.foreach({ case (bank, idx) => {
        bank.io.stdHasIssue(issuePortIdx).valid := issueDriver.io.deq.fire && issueDriver.io.deq.bits.uop.ctrl.fuType === FuType.std &&
          issueDriver.io.deq.bits.bankIdxOH(idx)
        bank.io.stdHasIssue(issuePortIdx).bits := issueDriver.io.deq.bits.entryIdxOH
      }})
      uopReadPortIdx = uopReadPortIdx + 1
    }
  }
  require(uopReadPortIdx == 4)


  val bankReplayPort = rsBankSeq.map(_.io.replay)
  val feedbackSeq = Seq(stIssue.map(_._1.rsFeedback.feedbackSlowStore),
    ldIssue.map(_._1.rsFeedback.feedbackSlowLoad),
    ldFastReleaseRsEntry).flatten

  require(feedbackSeq.length == replayPortNum)
  bankReplayPort.zipWithIndex.foreach(bankReplays => {
    bankReplays._1.zip(feedbackSeq).foreach({ case (sink, source) =>
      val bankIdx = bankReplays._2
      sink.valid := source.valid && source.bits.rsIdx.bankIdxOH(bankIdx)
      sink.bits.entryIdxOH := source.bits.rsIdx.entryIdxOH
      sink.bits.status.waitVal := source.bits.sourceType
      sink.bits.status.hit := Mux(source.bits.sourceType === RSFeedbackType.success, true.B, false.B)
    })
  })


  println("\nMemory Reservation Wake Up Ports Config:")
  wakeup.zipWithIndex.foreach({ case ((_, cfg), idx) =>
    println(s"Wake Port $idx ${cfg.name} of ${cfg.complexName} #${cfg.id}")
  })
  XSPerfHistogram("issue_num", PopCount(issue.map(_._1.issue.fire)), true.B, 1, issue.length, 1)
}

