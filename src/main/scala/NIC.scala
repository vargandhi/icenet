package icenet

import chisel3._
import chisel3.util._
import chisel3.experimental.{IO}
import freechips.rocketchip.subsystem.{BaseSubsystem, TLBusWrapperLocation, PBUS, FBUS}
import freechips.rocketchip.config.{Field, Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.regmapper.{HasRegMap, RegField}
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._
import freechips.rocketchip.prci.{ClockSinkDomain}
import testchipip.{StreamIO, StreamChannel, TLHelper, ClockedIO}
import IceNetConsts._

/**
 * @inBufFlits How many flits in the input buffer(s)
 * @outBufFlits Number of flits in the output buffer
 * @nMemXacts Maximum number of transactions that the send/receive path can send to memory
 * @maxAcquireBytes Cache block size
 * @ctrlQueueDepth Depth of the MMIO control queues
 * @usePauser Hardware support for Ethernet pause frames
 * @checksumOffload TCP checksum offload engine
 * @packetMaxBytes Maximum number of bytes in a packet (header size + MTU)
 */

/*
 *  test commit NIC 1 
 *  test commit NIC 2
 */
case class NICConfig(
  inBufFlits: Int  = 2 * ETH_STANDARD_MAX_BYTES / NET_IF_BYTES,
  outBufFlits: Int = 2 * ETH_STANDARD_MAX_BYTES / NET_IF_BYTES,
  nMemXacts: Int = 8,
  maxAcquireBytes: Int = 64,
  ctrlQueueDepth: Int = 10,
  usePauser: Boolean = false,
  checksumOffload: Boolean = false,
  packetMaxBytes: Int = ETH_STANDARD_MAX_BYTES)

case class NICAttachParams(
  masterWhere: TLBusWrapperLocation = FBUS,
  slaveWhere: TLBusWrapperLocation = PBUS
)

case object NICKey extends Field[Option[NICConfig]](None)
case object NICAttachKey extends Field[NICAttachParams](NICAttachParams())

trait HasNICParameters {
  implicit val p: Parameters
  val nicExternal = p(NICKey).get
  val inBufFlits = nicExternal.inBufFlits
  val outBufFlits = nicExternal.outBufFlits
  val nMemXacts = nicExternal.nMemXacts
  val maxAcquireBytes = nicExternal.maxAcquireBytes
  val ctrlQueueDepth = nicExternal.ctrlQueueDepth
  val usePauser = nicExternal.usePauser
  val checksumOffload = nicExternal.checksumOffload
  val packetMaxBytes = nicExternal.packetMaxBytes
}

abstract class NICLazyModule(implicit p: Parameters)
  extends LazyModule with HasNICParameters

abstract class NICModule(implicit val p: Parameters)
  extends Module with HasNICParameters

abstract class NICBundle(implicit val p: Parameters)
  extends Bundle with HasNICParameters

class PacketArbiter(arbN: Int, rr: Boolean = false)
  extends HellaPeekingArbiter(
    new StreamChannel(NET_IF_WIDTH), arbN,
    (ch: StreamChannel) => ch.last, rr = rr)


class IceNicSendIO extends Bundle {
  val req = Decoupled(UInt(NET_IF_WIDTH.W))
  val comp = Flipped(Decoupled(Bool()))
}


class IceNicCoreSendReq extends Bundle {
  val req = Decoupled(UInt(NET_IF_WIDTH.W))
  val coreID = Decoupled(UInt(CORE_ID_BITS.W)) 
}

//TODO: check if trait works
trait IceNicGlobalSendIO extends Bundle {   
  
  val req = new IceNicCoreSendReq
  val comp = Flipped(Decoupled(Bool()))
  val comp2 = Flipped(Decoupled(Bool()))
  
}

class IceNicRecvIO extends Bundle {
  val req = Decoupled(UInt(NET_IF_WIDTH.W))
  val comp = Flipped(Decoupled(UInt(NET_LEN_BITS.W)))
}

class IceNicCoreRecvReq extends Bundle {
    val req = Decoupled(UInt(NET_IF_WIDTH.W))
    val coreID = Decoupled(UInt(CORE_ID_BITS.W))
}

//TODO: check if trait works
trait IceNicGlobalRecvIO extends Bundle {
  val req = new IceNicCoreRecvReq
  val comp = Flipped(Decoupled(UInt(NET_LEN_BITS.W)))
  val comp2 = Flipped(Decoupled(UInt(NET_LEN_BITS.W))) 
}


trait IceNicControllerBundle extends Bundle {
  val send = new IceNicSendIO
  val recv = new IceNicRecvIO
 
  val send2 = new IceNicSendIO
  val recv2 = new IceNicRecvIO

  val sendCoreReq1 = new IceNicCoreSendReq
  val sendCoreReq2 = new IceNicCoreSendReq  

  val recvCoreReq1 = new IceNicCoreRecvReq
  val recvCoreReq2 = new IceNicCoreRecvReq

  val sendMain = new IceNicGlobalSendIO
  val recvMain = new IceNicGlobalRecvIO
 
  
  val macAddr = Input(UInt(ETH_MAC_BITS.W))
  val macAddr2 = Input(UInt(ETH_MAC_BITS.W))

  val txcsumReq = Decoupled(new ChecksumRewriteRequest)
  val txcsumReq2 = Decoupled(new ChecksumRewriteRequest)
  
  val rxcsumRes = Flipped(Decoupled(new TCPChecksumOffloadResult))
  val rxcsumRes2 = Flipped(Decoupled(new TCPChecksumOffloadResult))   


  val csumEnable = Output(Bool())
  val csumEnable2 = Output(Bool())

  
  val coreSendMem1 = Input(UInt(BUF_REQ_BITS.W))
  val coreSendMem2 = Input(UInt(BUF_REQ_BITS.W))

  val coreRecvMem1 = Input(UInt(BUF_REQ_BITS.W))
  val coreRecvMem1 = Input(UInt(BUF_REQ_BITS.W))

}

trait IceNicControllerModule extends HasRegMap with HasNICParameters {
  implicit val p: Parameters
  val io: IceNicControllerBundle

  val sendCompDown = WireInit(false.B)
  val sendCompDown2 = WireInit(false.B)

  val qDepth = ctrlQueueDepth
  require(qDepth < (1 << 8))

  def queueCount[T <: Data](qio: QueueIO[T], depth: Int): UInt =
    TwoWayCounter(qio.enq.fire, qio.deq.fire, depth)

  // hold (len, addr) of packets that we need to send out
  val sendReqQueue = Module(new HellaQueue(qDepth)(UInt(NET_IF_WIDTH.W)))
  val sendReqCount = queueCount(sendReqQueue.io, qDepth)
   
 
  val sendReqQueue2 = Module(new HellaQueue(qDepth)(UInt(NET_IF_WIDTH.W)))
  val sendReqCount2 = queueCount(sendReqQueue2.io, qDepth)
  
  // hold addr of buffers we can write received packets into
  val recvReqQueue = Module(new HellaQueue(qDepth)(UInt(NET_IF_WIDTH.W)))
  val recvReqCount = queueCount(recvReqQueue.io, qDepth)
  
  val recvReqQueue2 = Module(new HellaQueue(qDepth)(UInt(NET_IF_WIDTH.W)))
  val recvReqCount2 = queueCount(recvReqQueue2.io, qDepth)

  // count number of sends completed
  val sendCompCount = TwoWayCounter(io.send.comp.fire, sendCompDown, qDepth)
  
  // count the number of sends completed on core2
  val sendCompCount2 = TwoWayCounter(io.send2.comp.fire, sendCompDown2, qDepth)

  // hold length of received packets
  val recvCompQueue = Module(new HellaQueue(qDepth)(UInt(NET_LEN_BITS.W)))
  val recvCompCount = queueCount(recvCompQueue.io, qDepth)

  // hold length of received packets for core 2
  val recvCompQueue2 = Module(new HellaQueue(qDepth)(UInt(NET_LEN_BITS.W)))
  val recvCompCount2 = queueCount(recvCompQueue2.io, qDepth)


  val sendCompValid = sendCompCount > 0.U
  val sendCompValid2 = sendCompCount2 > 0.U
  
  val intMask = RegInit(0.U(2.W))
  val intMask2 = RegInit(0.U(2.W))

  //val core1 = RegInit(1.U(CORE_ID_BITS.W))
  //val core2 = RegInit(2.U(CORE_ID_BITS.W))

  io.send.req <> sendReqQueue.io.deq
  io.send2.req <> sendReqQueue2.io.deq
  
  io.sendCoreReq1.req <> io.send.req
  io.sendCoreReq1.coreID := 1.U 

  io.sendCoreReq2.req <> io.send2.req
  io.sendCoreReq2.coreID := 2.U
 
  val sendReqArb = Module(new RRArbiter(IceNicCoreSendReq,2))
  
//TODO: Add counter logic as ready/valid bool bits
  sendReqArb.io.in(0) <> io.sendCoreReq1
  sendReqArb.io.in(1) <> io.sendCoreReq2

  io.sendMain.req <> sendReqArb.io.out
  io.sendMain.comp <> io.send.comp
  io.sendMain.comp2 <> io.send2.comp
 
  io.send.comp.ready := sendCompCount < qDepth.U
  io.send2.comp.ready := sendCompCount2 < qDepth.U
  

  io.recv.req <> recvReqQueue.io.deq 
  recvCompQueue.io.enq <> io.recv.comp

  io.recv2.req <> recvReqQueue2.io.deq
  recvCompQueue2.io.enq <> io.recv2.comp
  
  
  io.recvCoreReq1.req <> io.recv.req
  io.recvCoreReq1.coreID := 1.U

  io.recvCoreReq1.req <> io.recv.req
  io.recvCoreReq1.coreID := 2.U 

  val recvReqArb = Module(new RRArbiter(IceNicCoreRecvReq,2))

  recvReqArb.io.in(0) <> io.recvCoreReq1
  recvReqArb.io.in(1) <> io.recvCoreReq2
  
  io.recvMain.req <> recvReqArb.io.out
  io.recvMain.comp <> io.recv.comp
  io.recvMain.comp2 <> io.recv2.comp


  interrupts(0) := sendCompValid && intMask(0)
  interrupts(1) := recvCompQueue.io.deq.valid && intMask(1)

  interrupts(2) := sendCompValid2 && intMask2(0) 
  interrupts(3) := recvCompQueue2.io.deq.valid && intMask2(1)

  val sendReqSpace = (qDepth.U - sendReqCount)
  val recvReqSpace = (qDepth.U - recvReqCount)
     
  val sendReqSpace2 = (qDepth.U - sendReqCount2)
  val recvReqSpace2 = (qDepth.U - recvReqCount2)

  def sendCompRead = (ready: Bool) => {
    sendCompDown := sendCompValid && ready
    (sendCompValid, true.B)
  }

  def sendCompRead2 = (ready: Bool) => {                                                                                                       
    sendCompDown2 := sendCompValid2 && ready
    (sendCompValid2, true.B)
  }
  
  val txcsumReqQueue = Module(new HellaQueue(qDepth)(UInt(49.W)))
  val rxcsumResQueue = Module(new HellaQueue(qDepth)(UInt(2.W)))
  val csumEnable = RegInit(false.B)

  val txcsumReqQueue2 = Module(new HellaQueue(qDepth)(UInt(49.W)))
  val rxcsumResQueue2 = Module(new HellaQueue(qDepth)(UInt(2.W)))
  val csumEnable2 = RegInit(false.B)

  // initialize csum data structures for core1
  io.txcsumReq.valid := txcsumReqQueue.io.deq.valid
  io.txcsumReq.bits := txcsumReqQueue.io.deq.bits.asTypeOf(new ChecksumRewriteRequest)
  txcsumReqQueue.io.deq.ready := io.txcsumReq.ready

  rxcsumResQueue.io.enq.valid := io.rxcsumRes.valid
  rxcsumResQueue.io.enq.bits := io.rxcsumRes.bits.asUInt
  io.rxcsumRes.ready := rxcsumResQueue.io.enq.ready

  io.csumEnable := csumEnable
  
  io.coreMem1 := coreMem1

  // initialize csum data structures for core2 
  io.txcsumReq2.valid := txcsumReqQueue2.io.deq.valid
  io.txcsumReq2.bits := txcsumReqQueue2.io.deq.bits.asTypeOf(new ChecksumRewriteRequest)
  txcsumReqQueue2.io.deq.ready := io.txcsumReq2.ready
  
  rxcsumResQueue2.io.enq.valid := io.rxcsumRes2.valid
  rxcsumResQueue2.io.enq.bits := io.rxcsumRes2.bits.asUInt
  io.rxcsumRes2.ready := rxcsumResQueue2.io.enq.ready
  
  io.csumEnable2 := csumEnable2
  io.coreMem2 := coreMem2 
  
  regmap(
    0x00 -> Seq(RegField.w(NET_IF_WIDTH, sendReqQueue.io.enq)),
    0x08 -> Seq(RegField.w(NET_IF_WIDTH, recvReqQueue.io.enq)),
    0x10 -> Seq(RegField.r(1, sendCompRead)),
    0x12 -> Seq(RegField.r(NET_LEN_BITS, recvCompQueue.io.deq)),
    0x14 -> Seq(
      RegField.r(8, sendReqSpace),
      RegField.r(8, recvReqSpace),
      RegField.r(8, sendCompCount),
      RegField.r(8, recvCompCount)),
    0x18 -> Seq(RegField.r(ETH_MAC_BITS, io.macAddr)),
    0x20 -> Seq(RegField(2, intMask)),
    0x28 -> Seq(RegField.w(49, txcsumReqQueue.io.enq)),
    0x30 -> Seq(RegField.r(2, rxcsumResQueue.io.deq)),
    0x31 -> Seq(RegField(1, csumEnable)),
    0x32 -> Seq(RegField.w(BUF_REQ_BITS, coreSendMem1)),
    0x33 -> Seq(RegField.w(BUF_REQ_BITS, coreRecvMem1)),   
 
    // Set2 of MMIO registers
    0x40 -> Seq(RegField.w(NET_IF_WIDTH, sendReqQueue2.io.enq)),
    0x48 -> Seq(RegField.w(NET_IF_WIDTH, recvReqQueue2.io.enq)),
    0x50 -> Seq(RegField.r(1, sendCompRead2)),
    0x52 -> Seq(RegField.r(NET_LEN_BITS, recvCompQueue2.io.deq)),
    0x54 -> Seq(
      RegField.r(8, sendReqSpace2),
      RegField.r(8, recvReqSpace2),
      RegField.r(8, sendCompCount2),
      RegField.r(8, recvCompCount2)),       
   
    // TODO: update the lower regmap --- check if diff macAddr needed!!
    0x58 -> Seq(RegField.r(ETH_MAC_BITS, io.macAddr2)),
    0x60 -> Seq(RegField(2, intMask2)),
    0x68 -> Seq(RegField.w(49, txcsumReqQueue2.io.enq)),
    0x70 -> Seq(RegField.r(2, rxcsumResQueue2.io.deq)),
    0x71 -> Seq(RegField(1, csumEnable2)), 
    0x72 -> Seq(RegField.w(BUF_REQ_BITS, coreSendMem2)),
    0x73 -> Seq(RegField.w(BUF_REQ_BITS, coreRecvMem2))) 

}


case class IceNicControllerParams(address: BigInt, beatBytes: Int)

/*
 * Take commands from the CPU over TL2, expose as Queues
*/

class IceNicController(c: IceNicControllerParams)(implicit p: Parameters)
  extends TLRegisterRouter(
    c.address, "ice-nic", Seq("ucbbar,ice-nic"),

}

case class IceNicControllerParams(address: BigInt, beatBytes: Int)

/*
 * Take commands from the CPU over TL2, expose as Queues
 */
class IceNicController(c: IceNicControllerParams)(implicit p: Parameters)
  extends TLRegisterRouter(
    c.address, "ice-nic", Seq("ucbbar,ice-nic"),
    interrupts = 5, beatBytes = c.beatBytes)(
      new TLRegBundle(c, _)    with IceNicControllerBundle)(
      new TLRegModule(c, _, _) with IceNicControllerModule)

class IceNicSendPath(nInputTaps: Int = 0)(implicit p: Parameters)
    extends NICLazyModule {
  val reader = LazyModule(new StreamReader(
    nMemXacts, outBufFlits, maxAcquireBytes))
  val node = reader.node

  lazy val module = new LazyModuleImp(this) {
    val io = IO(new Bundle {
      //TODO-done: updated the send type to IceNicGlobalSendIO
      val send = Flipped(new IceNicGlobalSendIO)
      val tap = Flipped(Vec(nInputTaps, Decoupled(new StreamChannel(NET_IF_WIDTH))))
      val out = Decoupled(new StreamChannel(NET_IF_WIDTH))
      val rlimit = Input(new RateLimiterSettings)
      val csum = checksumOffload.option(new Bundle {
        val req = Flipped(Decoupled(new ChecksumRewriteRequest))
        val enable = Input(Bool())
      })
    })

    val readreq = reader.module.io.req
    val readreqCore = reader.module.io.coreID
    io.send.req.req.ready := readreq.ready
    readreq.valid := io.send.req.req.valid
    readreq.bits.address := io.send.req.req.bits(47, 0)
    readreq.bits.length  := io.send.req.req.bits(62, 48)
    readreq.bits.partial := io.send.req.req.bits(63)
    
  // Assign coreID of current send request to reader obj
    readreqCore := io.send.req.coreID
  
  // ---*** decide which comp signal to send based on req/core ***---

    when (readreqCore === 1.U){
        //TODO: set valid bits of comp2 to false?
        //TODO: decrement core1 counter 
        io.send.comp <> reader.module.io.resp
    }.elsewhen(readreqCore === 2.U){       
        io.send.comp2 <> reader.module.io.resp
        // TODO: set valid bits of comp to false?
        // TODO: decrement core2 counter
    }.otherwise{
        //do nothing
}   
  //---*** comp selection logic ends here ***----

    val preArbOut = if (checksumOffload) {
      val readerOut = reader.module.io.out
      val arb = Module(new PacketArbiter(2))
      val bufFlits = (packetMaxBytes - 1) / NET_IF_BYTES + 1
      val rewriter = Module(new ChecksumRewrite(NET_IF_WIDTH, bufFlits))
      val enable = io.csum.get.enable

      rewriter.io.req <> io.csum.get.req

      arb.io.in(0) <> rewriter.io.stream.out
      arb.io.in(1).valid := !enable && readerOut.valid
      arb.io.in(1).bits  := readerOut.bits
      rewriter.io.stream.in.valid := enable && readerOut.valid
      rewriter.io.stream.in.bits := readerOut.bits
      readerOut.ready := Mux(enable,
        rewriter.io.stream.in.ready, arb.io.in(1).ready)

      arb.io.out
    } else { reader.module.io.out }

    val unlimitedOut = if (nInputTaps > 0) {
      val bufWords = (packetMaxBytes - 1) / NET_IF_BYTES + 1
      val inputs = (preArbOut +: io.tap).map { in =>
        // The packet collection buffer doesn't allow sending the first flit
        // of a packet until the last flit is received.
        // This ensures that we don't lock the arbiter while waiting for data
        // to arrive, which could cause deadocks.
        val buffer = Module(new PacketCollectionBuffer(bufWords))
        buffer.io.in <> in
        buffer.io.out
      }
      val arb = Module(new PacketArbiter(inputs.size, rr = true))
      arb.io.in <> inputs
      arb.io.out
    } else { preArbOut }

    val limiter = Module(new RateLimiter(new StreamChannel(NET_IF_WIDTH)))
    limiter.io.in <> unlimitedOut
    limiter.io.settings := io.rlimit
    io.out <> limiter.io.out
  }
}

class IceNicWriter(implicit p: Parameters) extends NICLazyModule {
  val writer = LazyModule(new StreamWriter(nMemXacts, maxAcquireBytes))
  val node = writer.node

  lazy val module = new LazyModuleImp(this) {
    val io = IO(new Bundle {
      val recv = Flipped(new IceNicRecvIO)
      val in = Flipped(Decoupled(new StreamChannel(NET_IF_WIDTH)))
      val length = Flipped(Valid(UInt(NET_LEN_BITS.W)))
    })

    val streaming = RegInit(false.B)
    val byteAddrBits = log2Ceil(NET_IF_BYTES)
    val helper = DecoupledHelper(
      io.recv.req.valid,
      writer.module.io.req.ready,
      io.length.valid, !streaming)

    writer.module.io.req.valid := helper.fire(writer.module.io.req.ready)
    writer.module.io.req.bits.address := io.recv.req.bits
    writer.module.io.req.bits.length := io.length.bits
    io.recv.req.ready := helper.fire(io.recv.req.valid)

    writer.module.io.in.valid := io.in.valid && streaming
    writer.module.io.in.bits := io.in.bits
    io.in.ready := writer.module.io.in.ready && streaming

    io.recv.comp <> writer.module.io.resp

    when (io.recv.req.fire) { streaming := true.B }
    when (io.in.fire && io.in.bits.last) { streaming := false.B }
  }
}

/*
 * Recv frames
 */
class IceNicRecvPath(val tapFuncs: Seq[EthernetHeader => Bool] = Nil)
    (implicit p: Parameters) extends LazyModule {
  val writer = LazyModule(new IceNicWriter)
  val node = TLIdentityNode()
  node := writer.node
  lazy val module = new IceNicRecvPathModule(this)
}

class IceNicRecvPathModule(outer: IceNicRecvPath)
    extends LazyModuleImp(outer) with HasNICParameters {
  val io = IO(new Bundle {
    val recv = Flipped(new IceNicRecvIO)
    val in = Flipped(Decoupled(new StreamChannel(NET_IF_WIDTH))) // input stream
    val tap = Vec(outer.tapFuncs.length, Decoupled(new StreamChannel(NET_IF_WIDTH)))
    val csum = checksumOffload.option(new Bundle {
      val res = Decoupled(new TCPChecksumOffloadResult)
      val enable = Input(Bool())
    })
    val buf_free = Output(Vec(1 + outer.tapFuncs.length, UInt(8.W)))
  })

  def tapOutToDropCheck(tapOut: EthernetHeader => Bool) = {
    (header: EthernetHeader, ch: StreamChannel, update: Bool) => {
      val first = RegInit(true.B)
      val drop = tapOut(header) && first
      val dropReg = RegInit(false.B)

      when (update && first) { first := false.B; dropReg := drop }
      when (update && ch.last) { first := true.B; dropReg := false.B }

      drop || dropReg
    }
  }

  def duplicateStream(in: DecoupledIO[StreamChannel], outs: Seq[DecoupledIO[StreamChannel]]) = {
    outs.foreach { out =>
      out.valid := in.valid
      out.bits := in.bits
    }
    in.ready := outs.head.ready
    val outReadys = Cat(outs.map(_.ready))
    assert(outReadys.andR || !outReadys.orR,
      "Duplicated streams must all be ready simultaneously")
    outs
  }

  def invertCheck(check: (EthernetHeader, StreamChannel, Bool) => Bool) =
    (eth: EthernetHeader, ch: StreamChannel, up: Bool) => !check(eth, ch, up)

  val tapDropChecks = outer.tapFuncs.map(func => tapOutToDropCheck(func))
  val pauseDropCheck = if (usePauser) Some(PauseDropCheck(_, _, _)) else None
  val allDropChecks =
    // Drop checks for the primary buffer
    // Drop if the packet should be tapped out or is a pause frame
    Seq(tapDropChecks ++ pauseDropCheck.toSeq) ++
    // Drop checks for the tap buffers
    // For each tap, drop if the packet doesn't match the tap function or is a pause frame
    tapDropChecks.map(check => invertCheck(check) +: pauseDropCheck.toSeq)

  val buffers = allDropChecks.map(dropChecks =>
    Module(new NetworkPacketBuffer(
      inBufFlits,
      maxBytes = packetMaxBytes,
      dropChecks = dropChecks, dropless = usePauser)))
  duplicateStream(io.in, buffers.map(_.io.stream.in))

  io.buf_free := buffers.map(_.io.free)

  io.tap <> buffers.tail.map(_.io.stream.out)
  val bufout = buffers.head.io.stream.out
  val buflen = buffers.head.io.length

  val (csumout, recvreq) = (if (checksumOffload) {
    val offload = Module(new TCPChecksumOffload(NET_IF_WIDTH))
    val offloadReady = offload.io.in.ready || !io.csum.get.enable

    val out = Wire(Decoupled(new StreamChannel(NET_IF_WIDTH)))
    val recvreq = Wire(Decoupled(UInt(NET_IF_WIDTH.W)))
    val reqq = Module(new Queue(UInt(NET_IF_WIDTH.W), 1))

    val enqHelper = DecoupledHelper(
      io.recv.req.valid, reqq.io.enq.ready, recvreq.ready)
    val deqHelper = DecoupledHelper(
      bufout.valid, offloadReady, out.ready, reqq.io.deq.valid)

    reqq.io.enq.valid := enqHelper.fire(reqq.io.enq.ready)
    reqq.io.enq.bits := io.recv.req.bits
    io.recv.req.ready := enqHelper.fire(io.recv.req.valid)
    recvreq.valid := enqHelper.fire(recvreq.ready)
    recvreq.bits := io.recv.req.bits

    out.valid := deqHelper.fire(out.ready)
    out.bits  := bufout.bits
    offload.io.in.valid := deqHelper.fire(offloadReady, io.csum.get.enable)
    offload.io.in.bits := bufout.bits
    bufout.ready := deqHelper.fire(bufout.valid)
    reqq.io.deq.ready := deqHelper.fire(reqq.io.deq.valid, bufout.bits.last)

    io.csum.get.res <> offload.io.result

    (out, recvreq)
  } else { (bufout, io.recv.req) })

  val writer = outer.writer.module
  writer.io.recv.req <> Queue(recvreq, 1)
  io.recv.comp <> writer.io.recv.comp
  writer.io.in <> csumout
  writer.io.length.valid := buflen.valid
  writer.io.length.bits  := buflen.bits
}

class NICIO extends StreamIO(NET_IF_WIDTH) {
  val macAddr = Input(UInt(ETH_MAC_BITS.W))
  val rlimit = Input(new RateLimiterSettings)
  val pauser = Input(new PauserSettings)

}

/*
 * A simple NIC
 *
 * Expects ethernet frames (see below), but uses a custom transport
 * (see ExtBundle)
 *
 * Ethernet Frame format:
 *   2 bytes |  6 bytes  |  6 bytes    | 2 bytes  | 46-1500B
 *   Padding | Dest Addr | Source Addr | Type/Len | Data
 *
 * @address Starting address of MMIO control registers
 * @beatBytes Width of memory interface (in bytes)
 * @tapOutFuncs Sequence of functions for each output tap.
 *              Each function takes the header of an Ethernet frame
 *              and returns Bool that is true if matching and false if not.
 * @nInputTaps Number of input taps
 *
 */
class IceNIC(address: BigInt, beatBytes: Int = 8,
    tapOutFuncs: Seq[EthernetHeader => Bool] = Nil,
    nInputTaps: Int = 0)
    (implicit p: Parameters) extends NICLazyModule {

  val control = LazyModule(new IceNicController(
    IceNicControllerParams(address, beatBytes)))
  val sendPath = LazyModule(new IceNicSendPath(nInputTaps))
  val recvPath = LazyModule(new IceNicRecvPath(tapOutFuncs))

  val mmionode = TLIdentityNode()
  val dmanode = TLIdentityNode()
  val intnode = control.intnode

  control.node := TLAtomicAutomata() := mmionode
  dmanode := TLWidthWidget(NET_IF_BYTES) := sendPath.node
  dmanode := TLWidthWidget(NET_IF_BYTES) := recvPath.node

  lazy val module = new LazyModuleImp(this) {
    val io = IO(new Bundle {
      val ext = new NICIO
      val tapOut = Vec(tapOutFuncs.length, Decoupled(new StreamChannel(NET_IF_WIDTH)))
      val tapIn = Flipped(Vec(nInputTaps, Decoupled(new StreamChannel(NET_IF_WIDTH))))
    })
 
//TODO: updated sendpath and receive path connections with the controller queues
    sendPath.module.io.send <> control.module.io.sendMain
    recvPath.module.io.recv <> control.module.io.recv



    // connect externally
    if (usePauser) {
      val pauser = Module(new Pauser(inBufFlits, 1 + tapOutFuncs.length))
      pauser.io.int.out <> sendPath.module.io.out
      recvPath.module.io.in <> pauser.io.int.in
      io.ext.out <> pauser.io.ext.out
      pauser.io.ext.in <> io.ext.in
      pauser.io.in_free := recvPath.module.io.buf_free
      pauser.io.macAddr := io.ext.macAddr
      pauser.io.settings := io.ext.pauser
    } else {
      recvPath.module.io.in <> io.ext.in
      io.ext.out <> sendPath.module.io.out
    }

    control.module.io.macAddr := io.ext.macAddr
    sendPath.module.io.rlimit := io.ext.rlimit

    io.tapOut <> recvPath.module.io.tap
    sendPath.module.io.tap <> io.tapIn

    if (checksumOffload) {
      sendPath.module.io.csum.get.req <> control.module.io.txcsumReq
      sendPath.module.io.csum.get.enable := control.module.io.csumEnable
      control.module.io.rxcsumRes <> recvPath.module.io.csum.get.res
      recvPath.module.io.csum.get.enable := control.module.io.csumEnable
    } else {
      control.module.io.txcsumReq.ready := false.B
      control.module.io.rxcsumRes.valid := false.B
      control.module.io.rxcsumRes.bits := DontCare
    }
  }
}

class SimNetwork extends BlackBox with HasBlackBoxResource {
  val io = IO(new Bundle {
    val clock = Input(Clock())
    val reset = Input(Bool())
    val net = Flipped(new NICIOvonly)
  })
  addResource("/vsrc/SimNetwork.v")
  addResource("/csrc/SimNetwork.cc")
  addResource("/csrc/device.h")
  addResource("/csrc/device.cc")
  addResource("/csrc/switch.h")
  addResource("/csrc/switch.cc")
  addResource("/csrc/packet.h")
}


class NICIOvonly extends Bundle {
  val in = Flipped(Valid(new StreamChannel(NET_IF_WIDTH)))
  val out = Valid(new StreamChannel(NET_IF_WIDTH))
  val macAddr = Input(UInt(ETH_MAC_BITS.W))
  val rlimit = Input(new RateLimiterSettings)
  val pauser = Input(new PauserSettings)

}

object NICIOvonly {
  def apply(nicio: NICIO): NICIOvonly = {
    val vonly = Wire(new NICIOvonly)
    vonly.out.valid := nicio.out.valid
    vonly.out.bits  := nicio.out.bits
    nicio.out.ready := true.B
    nicio.in.valid  := vonly.in.valid
    nicio.in.bits   := vonly.in.bits
    assert(!vonly.in.valid || nicio.in.ready, "NIC input not ready for valid")
    nicio.macAddr := vonly.macAddr
    nicio.rlimit  := vonly.rlimit
    nicio.pauser  := vonly.pauser
    vonly
  }
}

object NICIO {
  def apply(vonly: NICIOvonly): NICIO = {
    val nicio = Wire(new NICIO)
    assert(!vonly.out.valid || nicio.out.ready)
    nicio.out.valid := vonly.out.valid
    nicio.out.bits  := vonly.out.bits
    vonly.in.valid  := nicio.in.valid
    vonly.in.bits   := nicio.in.bits
    nicio.in.ready  := true.B
    vonly.macAddr   := nicio.macAddr
    vonly.rlimit    := nicio.rlimit
    vonly.pauser    := nicio.pauser
    nicio
  }

}
trait CanHavePeripheryIceNIC  { this: BaseSubsystem =>
  private val address = BigInt(0x10016000)
  private val portName = "Ice-NIC"


  val icenicOpt = p(NICKey).map { params =>
    val manager = locateTLBusWrapper(p(NICAttachKey).slaveWhere)
    val client = locateTLBusWrapper(p(NICAttachKey).masterWhere)
    val domain = LazyModule(new ClockSinkDomain(name=Some(portName)))
    // TODO: currently the controller is in the clock domain of the bus which masters it
    // we assume this is same as the clock domain of the bus the controller masters
    domain.clockNode := manager.fixedClockNode

    val icenic = domain { LazyModule(new IceNIC(address, manager.beatBytes)) }

    manager.toVariableWidthSlave(Some(portName)) { icenic.mmionode }
    client.fromPort(Some(portName))() :=* icenic.dmanode
    ibus.fromSync := icenic.intnode

    val inner_io = domain { InModuleBody {
      val inner_io = IO(new NICIOvonly).suggestName("nic")
      inner_io <> NICIOvonly(icenic.module.io.ext)
      inner_io
    } }

    val outer_io = InModuleBody {
      val outer_io = IO(new ClockedIO(new NICIOvonly)).suggestName("nic")
      outer_io.bits <> inner_io
      outer_io.clock := domain.module.clock
      outer_io
    }
    outer_io
  }
      netio.rlimit.period := PlusArg("rlimit-period", 1)
      netio.rlimit.size := PlusArg("rlimit-size", 8)
      netio.pauser.threshold := PlusArg("pauser-threshold", 2 * packetWords + latency)
      netio.pauser.quanta := PlusArg("pauser-quanta", 2 * packetQuanta)
      netio.pauser.refresh := PlusArg("pauser-refresh", packetWords)

      if (nicConf.get.usePauser) {
        val pauser = Module(new PauserComplex(qDepth))
        pauser.io.ext.flipConnect(NetDelay(NICIO(netio), latency))
        pauser.io.int.out <> pauser.io.int.in
        pauser.io.macAddr := netio.macAddr + (1 << 40).U
        pauser.io.settings := netio.pauser
      } else {

        netio.in := Pipe(netio.out, latency)
      }
      netio.in.bits.keep := NET_FULL_KEEP
    }
  }

  def connect(net: Option[NICIOvonly], nicConf: Option[NICConfig]) {
    net.foreach { netio =>
      val packetWords = nicConf.get.packetMaxBytes / NET_IF_BYTES
      NicLoopback.connect(net, nicConf, 4 * packetWords)
    }
  }
}

object SimNetwork {
  def connect(net: Option[NICIOvonly], clock: Clock, reset: Bool) {
    net.foreach { netio =>
      val sim = Module(new SimNetwork)
      sim.io.clock := clock
      sim.io.reset := reset
      sim.io.net <> netio
    }
  }
}
