package xiangshan.cache.prefetch

import chisel3._
import chisel3.util._
import xiangshan._
import xiangshan.cache._
import utils._

case class StreamPrefetchParameters(
  streamCnt: Int,
  streamSize: Int,
  ageWidth: Int,
  blockBytes: Int,
  reallocStreamOnMissInstantly: Boolean
) {
  def streamWidth = log2Up(streamCnt)
  def idxWidth = log2Up(streamSize)
  def totalWidth = streamWidth + idxWidth
}

class StreamPrefetchReq(p: StreamPrefetchParameters) extends PrefetchReq {
  val id = UInt(p.totalWidth.W)

  def stream = id(p.totalWidth - 1, p.totalWidth - p.streamWidth)
  def idx = id(p.idxWidth - 1, 0)

  override def cloneType: this.type = (new StreamPrefetchReq(p)).asInstanceOf[this.type]
}

class StreamPrefetchResp(p: StreamPrefetchParameters) extends PrefetchResp {
  val id = UInt(p.totalWidth.W)
  
  def stream = id(p.totalWidth - 1, p.totalWidth - p.streamWidth)
  def idx = id(p.idxWidth - 1, 0)

  override def cloneType: this.type = (new StreamPrefetchResp(p)).asInstanceOf[this.type]
}

class StreamPrefetchIO(p: StreamPrefetchParameters) extends PrefetchBundle {
  val train = Flipped(ValidIO(new PrefetchTrain))
  val req = DecoupledIO(new StreamPrefetchReq(p))
  val resp = Flipped(DecoupledIO(new StreamPrefetchResp(p)))

  override def cloneType: this.type = (new StreamPrefetchIO(p)).asInstanceOf[this.type]
}

class StreamBufferUpdate(p: StreamPrefetchParameters) extends PrefetchBundle {
  val hitIdx = UInt(log2Up(p.streamSize).W)

  override def cloneType: this.type = (new StreamBufferUpdate(p)).asInstanceOf[this.type]
}

class StreamBufferAlloc(p: StreamPrefetchParameters) extends StreamPrefetchReq(p) {
  override def cloneType: this.type = (new StreamBufferAlloc(p)).asInstanceOf[this.type]
}


class StreamBuffer(p: StreamPrefetchParameters) extends PrefetchModule {
  val io = IO(new Bundle {
    val addrs = Vec(p.streamSize, ValidIO(UInt(PAddrBits.W)))
    val update = Flipped(ValidIO(new StreamBufferUpdate(p)))
    val alloc = Flipped(ValidIO(new StreamBufferAlloc(p)))
    // prefetch req
    val req = DecoupledIO(new StreamPrefetchReq(p))
    val resp = Flipped(DecoupledIO(new StreamPrefetchResp(p)))
  })

  def streamSize = p.streamSize
  def streamCnt = p.streamCnt
  def blockBytes = p.blockBytes
  def getBlockAddr(addr: UInt) = addr & ~((blockBytes - 1).U)

  val baseReq = RegInit(0.U.asTypeOf(Valid(new PrefetchReq)))
  val nextReq = RegInit(0.U.asTypeOf(new PrefetchReq))
  val buf = RegInit(VecInit(Seq.fill(streamSize)(0.U.asTypeOf(new PrefetchReq))))
  val valid = RegInit(VecInit(Seq.fill(streamSize)(false.B)))
  val head = RegInit(0.U(log2Up(streamSize).W))
  val tail = RegInit(0.U(log2Up(streamCnt).W))
  val full = head === tail && valid(head)
  val empty = head === tail && !valid(head)

  val s_idle :: s_req :: s_resp :: Nil = Enum(3)
  val state = RegInit(VecInit(Seq.fill(streamSize)(s_idle)))

  val isPrefetching = VecInit(state.map(_ =/= s_idle))
  val deqLater = RegInit(VecInit(Seq.fill(streamSize)(false.B)))

  // dequeue
  val hitIdx = io.update.bits.hitIdx
  when (io.update.valid && !empty && valid(hitIdx)) {
    val headBeforehitIdx = head <= hitIdx && (hitIdx < tail || tail <= head)
    val hitIdxBeforeHead = hitIdx < tail && tail <= head
    when (headBeforehitIdx) {
      (0 until streamSize).foreach(i => deqLater(i) := Mux(i.U >= head && i.U <= hitIdx, true.B, deqLater(i)))
    }

    when (hitIdxBeforeHead) {
      (0 until streamSize).foreach(i => deqLater(i) := Mux(i.U >= head || i.U <= hitIdx, true.B, deqLater(i)))
    }
  }

  val deqValid = WireInit(VecInit(Seq.fill(streamSize)(false.B)))
  deqValid(head) := deqLater(head) && !isPrefetching(head)
  var deq = deqLater(head) && !isPrefetching(head)
  for (i <- 1 until streamSize) {
    val idx = head + i.U
    deq = deq && deqLater(idx) && !isPrefetching(idx)
    deqValid(idx) := deq
  }

  (0 until streamSize).foreach(i => valid(i) := valid(i) && !deqValid(i))
  (0 until streamSize).foreach(i => deqLater(i) := deqLater(i) && !deqValid(i))
  val nextHead = head + PopCount(deqValid)
  when (deqValid.asUInt.orR) {
    head := nextHead
    baseReq.valid := true.B
    baseReq.bits := buf(nextHead - 1.U)
  }

  // enqueue
  when (!full && baseReq.valid) {
    state(tail) := s_req
    tail := tail + 1.U
    buf(tail) := nextReq
    nextReq.addr := nextReq.addr + blockBytes.U
  }

  val reqs = Wire(Vec(streamSize, Decoupled(new StreamPrefetchReq(p))))
  val resps = Wire(Vec(streamSize, Decoupled(new StreamPrefetchResp(p))))
  (0 until streamSize).foreach{ i =>
    when (state(i) === s_req) {
      when (reqs(i).fire()) {
        state(i) := s_resp
      }
    }

    when (state(i) === s_resp) {
      when (resps(i).fire()) {
        state(i) := s_idle
        valid(i) := true.B
      }
    }

    reqs(i).valid := state(i) === s_req
    reqs(i).bits.addr := buf(i).addr
    reqs(i).bits.write := buf(i).write
    reqs(i).bits.id := Cat(0.U(p.streamWidth.W), i.U(p.idxWidth.W))
    resps(i).ready := state(i) === s_resp
  }

  // send req sequentially
  val prefetchPrior = Wire(Vec(streamSize, UInt(log2Up(streamSize).W)))
  val reqArb = Module(new Arbiter(new StreamPrefetchReq(p), streamSize))
  for (i <- 0 until streamSize) {
    prefetchPrior(i) := head + i.U
    reqs(i).ready := false.B
    reqs(prefetchPrior(i)) <> reqArb.io.in(i)
    resps(i).bits := io.resp.bits
    resps(i).valid := io.resp.valid && io.resp.bits.idx === i.U
  }
  reqArb.io.out <> io.req
  io.resp.ready := VecInit(resps.zipWithIndex.map{ case (r, i) =>
    r.ready && i.U === io.resp.bits.idx}).asUInt.orR
  
  // realloc this stream buffer for a newly-found stream
  val reallocReq = RegInit(0.U.asTypeOf(new StreamBufferAlloc(p)))
  val needRealloc = RegInit(false.B)
  when (io.alloc.valid) {
    needRealloc := true.B
    reallocReq := io.alloc.bits
    reallocReq.addr := getBlockAddr(io.alloc.bits.addr)
  }.elsewhen (needRealloc && !isPrefetching.asUInt.orR) {
    baseReq.valid := true.B
    baseReq.bits := reallocReq
    nextReq.write := reallocReq.write
    nextReq.addr := reallocReq.addr + blockBytes.U
    head := 0.U
    tail := 0.U
    needRealloc := false.B
    valid.foreach(_ := false.B)
  }

  for (i <- 0 until streamSize) {
    io.addrs(i).valid := baseReq.valid && (valid(i) || state(i) =/= s_idle)
    io.addrs(i).bits := getBlockAddr(buf(i).addr)
  }
}

class CompareBundle(width: Int) extends PrefetchBundle {
  val bits = UInt(width.W)
  val idx = UInt()

  override def cloneType: this.type = (new CompareBundle(width)).asInstanceOf[this.type]
}

object ParallelMin {
  def apply[T <: Data](xs: Seq[CompareBundle]): CompareBundle = {
    ParallelOperation(xs, (a: CompareBundle, b: CompareBundle) => Mux(a.bits < b.bits, a, b))
  }
}

class StreamPrefetch(p: StreamPrefetchParameters) extends PrefetchModule {
  val io = IO(new StreamPrefetchIO(p))
  
  // TODO: implement this
  def streamCnt = p.streamCnt
  def streamSize = p.streamSize
  def ageWidth = p.ageWidth
  def getBlockAddr(addr: UInt) = addr & ~((p.blockBytes - 1).U)
  val streamBufs = Seq.fill(streamCnt) { Module(new StreamBuffer(p)) }
  val addrValids = Wire(Vec(streamCnt, Vec(streamSize, Bool())))
  for (i <- 0 until streamCnt) {
    for (j <- 0 until streamSize) {
      addrValids(i)(j) := streamBufs(i).io.addrs(j).valid
    }
  }
  val bufValids = WireInit(VecInit(addrValids.map(_.asUInt.orR)))
  val ages = Seq.fill(streamCnt)(RegInit(0.U(ageWidth.W)))
  val maxAge = -1.S(ageWidth.W).asUInt

  // assign default value
  for (i <- 0 until streamCnt) {
    streamBufs(i).io.update.valid := false.B
    streamBufs(i).io.update.bits := DontCare
    streamBufs(i).io.alloc.valid := false.B
    streamBufs(i).io.alloc.bits := DontCare
  }

  // 1. streamBufs hit while l1i miss
  val hit = WireInit(false.B)
  for (i <- 0 until streamCnt) {
    for (j <- 0 until streamSize) {
      when (io.train.valid && addrValids(i)(j) && getBlockAddr(io.train.bits.addr) === streamBufs(i).io.addrs(j).bits) {
        hit := true.B
        streamBufs(i).io.update.valid := true.B
        streamBufs(i).io.update.bits.hitIdx := j.U
        ages(i) := maxAge
      }
    }
  }

  // 2. streamBufs miss
  when (!hit && io.train.valid) {
    (0 until streamCnt).foreach(i => ages(i) := Mux(ages(i) =/= 0.U, ages(i) - 1.U, 0.U))

    // realloc an invalid or the eldest stream buffer with new one
    val idx = Wire(UInt(log2Up(streamCnt).W))
    when ((~bufValids.asUInt).orR) {
      idx := PriorityMux(~bufValids.asUInt, VecInit(List.tabulate(streamCnt)(_.U)))
    }.otherwise {
      val ageCmp = Seq.fill(streamCnt)(Wire(new CompareBundle(ageWidth)))
      (0 until streamCnt).foreach(i => ageCmp(i).bits := ages(i))
      (0 until streamCnt).foreach(i => ageCmp(i).idx := i.U)
      idx := ParallelMin(ageCmp).idx
    }

    for (i <- 0 until streamCnt) {
      streamBufs(i).io.alloc.valid := idx === i.U
      streamBufs(i).io.alloc.bits := DontCare
      streamBufs(i).io.alloc.bits.addr := io.train.bits.addr
      streamBufs(i).io.alloc.bits.write := io.train.bits.write
      when (idx === i.U) { ages(i) := maxAge }
    }
  }

  // 3. send reqs from streamBufs
  val reqArb = Module(new Arbiter(new StreamPrefetchReq(p), streamCnt))
  for (i <- 0 until streamCnt) {
    reqArb.io.in(i).valid := streamBufs(i).io.req.valid
    reqArb.io.in(i).bits := streamBufs(i).io.req.bits
    reqArb.io.in(i).bits.id := Cat(i.U(p.streamWidth.W), streamBufs(i).io.req.bits.id(p.idxWidth - 1, 0))
    streamBufs(i).io.req.ready := reqArb.io.in(i).ready

    streamBufs(i).io.resp.valid := io.resp.valid && i.U === io.resp.bits.stream
    streamBufs(i).io.resp.bits := io.resp.bits
  }
  io.req <> reqArb.io.out
  io.resp.ready := VecInit(streamBufs.zipWithIndex.map { case (buf, i) =>
    i.U === io.resp.bits.stream && buf.io.resp.ready}).asUInt.orR
}
