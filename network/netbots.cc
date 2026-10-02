// netbots ns-3 scenario: N robot nodes broadcasting state updates over UDP/IPv4.
//
// Medium: SimpleNetDevice (per-node DataRate; finite FIFO queue disc) attached to
// JitterChannel, a SimpleChannel that delivers each frame to every other device after
// delay + U(0, jitter), drawn independently per receiver. Loss: RateErrorModel (packet
// unit) on every receiving device. Load: per-robot CBR OnOff UDP broadcast sharing the
// robot's interface/queue. There is NO shared-medium contention (switched-LAN abstraction).
//
// Modes:
//   batch   (default) ns-3 timers emit periodic updates; state payload is a placeholder.
//   coupled (--coupled) stdin/stdout lockstep driven by Python:
//            TX <src> <t_ns> <x> <y> <th> <v> <w>   schedule an update at t_ns (>= now)
//            RUN <t_ns>                            advance; replies "RX ..." lines then "DONE"
//            END
// Both modes write the same CSV trace: event,t_ns,src,dst,seq,t_gen_ns,bytes,kind,reason

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/traffic-control-module.h"

#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace ns3;

static const uint16_t PORT = 5000;
static const uint32_t PAYLOAD = 56; // u32 src, u32 seq, i64 t_gen_ns, 5 x f64 state

// Identifies a state packet anywhere in the stack (also inside drop traces).
class NetbotsTag : public Tag
{
  public:
    uint32_t src = 0, seq = 0;
    int64_t tgen = 0;

    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("NetbotsTag").SetParent<Tag>().AddConstructor<NetbotsTag>();
        return tid;
    }

    TypeId GetInstanceTypeId() const override { return GetTypeId(); }

    uint32_t GetSerializedSize() const override { return 16; }

    void Serialize(TagBuffer i) const override
    {
        i.WriteU32(src);
        i.WriteU32(seq);
        i.WriteU64(tgen);
    }

    void Deserialize(TagBuffer i) override
    {
        src = i.ReadU32();
        seq = i.ReadU32();
        tgen = i.ReadU64();
    }

    void Print(std::ostream& os) const override { os << src << ":" << seq; }
};

class JitterChannel : public SimpleChannel
{
  public:
    Time delay;
    Ptr<UniformRandomVariable> jitter = CreateObject<UniformRandomVariable>();

    void Send(Ptr<Packet> p,
              uint16_t protocol,
              Mac48Address to,
              Mac48Address from,
              Ptr<SimpleNetDevice> sender) override
    {
        for (std::size_t i = 0; i < GetNDevices(); ++i)
        {
            auto dev = DynamicCast<SimpleNetDevice>(GetDevice(i));
            if (dev == sender)
            {
                continue;
            }
            Time d = delay + NanoSeconds(int64_t(jitter->GetValue()));
            Simulator::ScheduleWithContext(dev->GetNode()->GetId(),
                                           d,
                                           &SimpleNetDevice::Receive,
                                           dev,
                                           p->Copy(),
                                           protocol,
                                           to,
                                           from);
        }
    }
};

static std::ofstream g_trace;
static bool g_coupled = false;
static std::vector<Ptr<Socket>> g_sockets;
static std::vector<uint32_t> g_seq;

static void
Log(const char* ev, int src, int dst, uint32_t seq, int64_t tgen, uint32_t bytes,
    const char* kind, const char* reason)
{
    g_trace << ev << "," << Simulator::Now().GetNanoSeconds() << "," << src << "," << dst << ","
            << seq << "," << tgen << "," << bytes << "," << kind << "," << reason << "\n";
}

static void
LogDrop(int dst, const char* reason, Ptr<const Packet> p)
{
    NetbotsTag t;
    if (p->PeekPacketTag(t))
    {
        Log("drop", t.src, dst, t.seq, t.tgen, p->GetSize(), "state", reason);
    }
    else
    {
        Log("drop", -1, dst, 0, -1, p->GetSize(), "bg", reason);
    }
}

static void
LogQdiscDrop(Ptr<const QueueDiscItem> item)
{
    LogDrop(-1, "queue", item->GetPacket());
}

static void
SendState(uint32_t src, double x, double y, double th, double v, double w)
{
    uint8_t buf[PAYLOAD];
    uint32_t seq = g_seq[src]++;
    int64_t tgen = Simulator::Now().GetNanoSeconds();
    double st[5] = {x, y, th, v, w};
    std::memcpy(buf, &src, 4);
    std::memcpy(buf + 4, &seq, 4);
    std::memcpy(buf + 8, &tgen, 8);
    std::memcpy(buf + 16, st, 40);
    Ptr<Packet> p = Create<Packet>(buf, PAYLOAD);
    NetbotsTag tag;
    tag.src = src;
    tag.seq = seq;
    tag.tgen = tgen;
    p->AddPacketTag(tag);
    Log("tx", src, -1, seq, tgen, PAYLOAD, "state", "");
    g_sockets[src]->SendTo(p, 0, InetSocketAddress(Ipv4Address::GetBroadcast(), PORT));
}

static void
Periodic(uint32_t src, Time period, Time stop)
{
    SendState(src, 0, 0, 0, 0, 0);
    if (Simulator::Now() + period < stop)
    {
        Simulator::Schedule(period, &Periodic, src, period, stop);
    }
}

static void
Receive(uint32_t dst, Ptr<Socket> s)
{
    Address from;
    while (Ptr<Packet> p = s->RecvFrom(from))
    {
        if (p->GetSize() != PAYLOAD)
        {
            continue;
        }
        uint8_t buf[PAYLOAD];
        p->CopyData(buf, PAYLOAD);
        uint32_t src, seq;
        int64_t tgen;
        double st[5];
        std::memcpy(&src, buf, 4);
        std::memcpy(&seq, buf + 4, 4);
        std::memcpy(&tgen, buf + 8, 8);
        std::memcpy(st, buf + 16, 40);
        if (src == dst)
        {
            continue;
        }
        Log("rx", src, dst, seq, tgen, PAYLOAD, "state", "");
        if (g_coupled)
        {
            std::cout.precision(17);
            std::cout << "RX " << Simulator::Now().GetNanoSeconds() << " " << src << " " << dst
                      << " " << seq << " " << tgen << " " << st[0] << " " << st[1] << " " << st[2]
                      << " " << st[3] << " " << st[4] << "\n";
        }
    }
}

int
main(int argc, char* argv[])
{
    uint32_t n = 3;
    double duration = 60, rateHz = 10, offsetMs = 10, delayMs = 0.1, jitterMs = 0, loss = 0;
    double bgKbps = 0, linkMbps = 2;
    uint32_t queuePkts = 100, bgPktBytes = 1000, seed = 1, run = 1;
    std::string out = "trace.csv";

    CommandLine cmd;
    cmd.AddValue("n", "number of robots", n);
    cmd.AddValue("duration", "simulated seconds (batch mode)", duration);
    cmd.AddValue("rateHz", "periodic update rate (batch mode)", rateHz);
    cmd.AddValue("offsetMs", "start offset step between robots (batch mode)", offsetMs);
    cmd.AddValue("delayMs", "base one-way channel delay", delayMs);
    cmd.AddValue("jitterMs", "extra per-receiver delay ~ U(0, jitterMs)", jitterMs);
    cmd.AddValue("loss", "per-receiver packet error rate", loss);
    cmd.AddValue("bgKbps", "per-robot background CBR load (0 = off)", bgKbps);
    cmd.AddValue("bgPktBytes", "background packet size", bgPktBytes);
    cmd.AddValue("linkMbps", "per-device transmit rate", linkMbps);
    cmd.AddValue("queuePkts", "FIFO queue disc capacity [packets]", queuePkts);
    cmd.AddValue("seed", "RngSeedManager seed", seed);
    cmd.AddValue("run", "RngSeedManager run number", run);
    cmd.AddValue("out", "trace CSV path", out);
    cmd.AddValue("coupled", "lockstep stdin/stdout mode", g_coupled);
    cmd.Parse(argc, argv);

    RngSeedManager::SetSeed(seed);
    RngSeedManager::SetRun(run);

    NodeContainer nodes;
    nodes.Create(n);

    auto ch = CreateObject<JitterChannel>();
    ch->delay = NanoSeconds(int64_t(delayMs * 1e6));
    ch->jitter->SetAttribute("Max", DoubleValue(jitterMs * 1e6));

    SimpleNetDeviceHelper sh;
    sh.SetDeviceAttribute("DataRate", DataRateValue(DataRate(uint64_t(linkMbps * 1e6))));
    // Device queue of 1 packet + flow control: all queueing happens in the traced
    // FIFO queue disc below (otherwise Ipv4AddressHelper silently adds a 1000p pfifo_fast).
    sh.SetQueue("ns3::DropTailQueue<Packet>", "MaxSize", QueueSizeValue(QueueSize("1p")));
    NetDeviceContainer devs = sh.Install(nodes, ch);

    InternetStackHelper inet;
    inet.Install(nodes);
    TrafficControlHelper tch;
    tch.SetRootQueueDisc("ns3::FifoQueueDisc",
                         "MaxSize",
                         QueueSizeValue(QueueSize(QueueSizeUnit::PACKETS, queuePkts)));
    QueueDiscContainer qdiscs = tch.Install(devs);
    Ipv4AddressHelper ip;
    ip.SetBase("10.1.0.0", "255.255.255.0");
    ip.Assign(devs);

    g_trace.open(out);
    g_trace << "event,t_ns,src,dst,seq,t_gen_ns,bytes,kind,reason\n";

    for (uint32_t i = 0; i < n; ++i)
    {
        auto dev = DynamicCast<SimpleNetDevice>(devs.Get(i));
        auto em = CreateObject<RateErrorModel>();
        em->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
        em->SetRate(loss);
        dev->SetReceiveErrorModel(em);
        NS_ABORT_UNLESS(dev->TraceConnectWithoutContext("PhyRxDrop", MakeBoundCallback(&LogDrop, int(i), "rx_error")));
        NS_ABORT_UNLESS(dev->GetQueue()->TraceConnectWithoutContext("Drop", MakeBoundCallback(&LogDrop, -1, "queue")));
        NS_ABORT_UNLESS(qdiscs.Get(i)->TraceConnectWithoutContext("Drop", MakeCallback(&LogQdiscDrop)));

        auto s = Socket::CreateSocket(nodes.Get(i), UdpSocketFactory::GetTypeId());
        s->SetAllowBroadcast(true);
        s->Bind(InetSocketAddress(Ipv4Address::GetAny(), PORT));
        s->SetRecvCallback(MakeBoundCallback(&Receive, i));
        g_sockets.push_back(s);
        g_seq.push_back(0);

        if (bgKbps > 0)
        {
            OnOffHelper bg("ns3::UdpSocketFactory",
                           InetSocketAddress(Ipv4Address::GetBroadcast(), PORT + 1));
            bg.SetConstantRate(DataRate(uint64_t(bgKbps * 1e3)), bgPktBytes);
            auto apps = bg.Install(nodes.Get(i));
            apps.Start(MilliSeconds(1 + i)); // stagger so CBR sources are not phase-locked
        }
    }

    if (!g_coupled)
    {
        Time period = NanoSeconds(int64_t(1e9 / rateHz));
        Time stop = Seconds(duration);
        for (uint32_t i = 0; i < n; ++i)
        {
            Simulator::Schedule(NanoSeconds(int64_t(i * offsetMs * 1e6)), &Periodic, i, period, stop);
        }
        Simulator::Stop(stop + Seconds(5)); // drain in-flight packets
        Simulator::Run();
    }
    else
    {
        std::string line;
        while (std::getline(std::cin, line))
        {
            std::istringstream ss(line);
            std::string op;
            ss >> op;
            if (op == "TX")
            {
                uint32_t src;
                int64_t t;
                double st[5];
                ss >> src >> t >> st[0] >> st[1] >> st[2] >> st[3] >> st[4];
                NS_ABORT_MSG_IF(t < Simulator::Now().GetNanoSeconds(), "TX in the past");
                Simulator::Schedule(NanoSeconds(t) - Simulator::Now(),
                                    &SendState, src, st[0], st[1], st[2], st[3], st[4]);
            }
            else if (op == "RUN")
            {
                int64_t t;
                ss >> t;
                Simulator::Stop(NanoSeconds(t) - Simulator::Now());
                Simulator::Run();
                std::cout << "DONE " << Simulator::Now().GetNanoSeconds() << std::endl;
            }
            else if (op == "END")
            {
                break;
            }
        }
    }
    g_trace.close();
    Simulator::Destroy();
    return 0;
}
