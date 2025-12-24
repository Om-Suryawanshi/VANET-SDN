#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/ipv4-static-routing.h"
#include "ns3/ipv4-list-routing.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/config-store-module.h"

#include <queue>
#include <sstream>
#include <set>
#include <map>
#include <vector>

NS_LOG_COMPONENT_DEFINE("VanetSdnHybrid");

using namespace ns3;

namespace ns3 {

NodeContainer* g_allNodes = nullptr;
static uint32_t g_dataIfIndex = 1;
static uint32_t g_ctrlIfIndex = 2;

Ipv4Address GetNodeIpv4Address(Ptr<Node> node, uint32_t ifIndex)
{
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    if (!ipv4 || ifIndex >= ipv4->GetNInterfaces()) return Ipv4Address::GetZero();
    for (uint32_t j = 0; j < ipv4->GetNAddresses(ifIndex); ++j) {
        Ipv4Address addr = ipv4->GetAddress(ifIndex, j).GetLocal();
        if (addr != Ipv4Address::GetLoopback() && addr != Ipv4Address::GetZero()) {
            return addr;
        }
    }
    return Ipv4Address::GetZero();
}

Ptr<Node> FindNodeByIp(Ipv4Address ip)
{
    if (g_allNodes == nullptr) return nullptr;
    for (uint32_t i = 0; i < g_allNodes->GetN(); ++i) {
        if (GetNodeIpv4Address(g_allNodes->Get(i), g_dataIfIndex) == ip ||
            GetNodeIpv4Address(g_allNodes->Get(i), g_ctrlIfIndex) == ip) {
            return g_allNodes->Get(i);
        }
    }
    return nullptr;
}

Ipv4Address GetCtrlIpByNode(Ptr<Node> n) { return GetNodeIpv4Address(n, g_ctrlIfIndex); }
Ipv4Address GetDataIpByNode(Ptr<Node> n) { return GetNodeIpv4Address(n, g_dataIfIndex); }

Ptr<Node> FindNodeByCtrlIp(Ipv4Address ctrlIp) {
  if (!g_allNodes) return nullptr;
  for (uint32_t i = 0; i < g_allNodes->GetN(); ++i) {
    if (GetCtrlIpByNode(g_allNodes->Get(i)) == ctrlIp) return g_allNodes->Get(i);
  }
  return nullptr;
}

class VehicleSdnApp : public Application
{
public:
    static TypeId GetTypeId();
    VehicleSdnApp();
    void SetControllerIp(Ipv4Address ip, uint32_t ctrlIfIndex);

private:
    virtual void StartApplication();
    virtual void StopApplication();

    void SendBeacon();
    void ReceiveBeacon(Ptr<Socket> socket);
    void SendReport();
    void ReceiveRoute(Ptr<Socket> socket);

    Ptr<Socket> m_beaconTx;
    Ptr<Socket> m_beaconRx;
    Ptr<Socket> m_ctrlRx;

    Ipv4Address m_controllerIp;
    uint32_t m_ctrlIfIndex = 2;
    std::set<Ipv4Address> m_neighbors;
    EventId m_beaconEvent;
    EventId m_reportEvent;
};

NS_OBJECT_ENSURE_REGISTERED(VehicleSdnApp);

TypeId VehicleSdnApp::GetTypeId()
{
    static TypeId tid = TypeId("ns3::VehicleSdnApp")
        .SetParent<Application>()
        .AddConstructor<VehicleSdnApp>();
    return tid;
}

VehicleSdnApp::VehicleSdnApp() {}

void VehicleSdnApp::SetControllerIp(Ipv4Address ip, uint32_t ctrlIfIndex)
{
    m_controllerIp = ip;
    m_ctrlIfIndex = ctrlIfIndex;
}

void VehicleSdnApp::StartApplication()
{
    Ipv4Address myCtrlIp = GetNodeIpv4Address(GetNode(), m_ctrlIfIndex);

    m_beaconTx = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    m_beaconTx->SetAllowBroadcast(true);
    // bind to control interface so beacons use control-plane IP
    m_beaconTx->Bind(InetSocketAddress(myCtrlIp, 0));

    m_beaconRx = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    m_beaconRx->Bind(InetSocketAddress(Ipv4Address::GetAny(), 8888));
    m_beaconRx->SetRecvCallback(MakeCallback(&VehicleSdnApp::ReceiveBeacon, this));

    m_ctrlRx = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    m_ctrlRx->Bind(InetSocketAddress(myCtrlIp, 10000));
    m_ctrlRx->SetRecvCallback(MakeCallback(&VehicleSdnApp::ReceiveRoute, this));

    m_beaconEvent = Simulator::Schedule(Seconds(0.5), &VehicleSdnApp::SendBeacon, this);
    m_reportEvent = Simulator::Schedule(Seconds(1.0), &VehicleSdnApp::SendReport, this);
}

void VehicleSdnApp::StopApplication()
{
    Simulator::Cancel(m_beaconEvent);
    Simulator::Cancel(m_reportEvent);
    if (m_beaconTx) m_beaconTx->Close();
    if (m_beaconRx) m_beaconRx->Close();
    if (m_ctrlRx) m_ctrlRx->Close();
}

void VehicleSdnApp::SendBeacon()
{
    Ipv4Address myIp = GetNodeIpv4Address(GetNode(), m_ctrlIfIndex);
    if (myIp == Ipv4Address::GetZero()) return;

    std::ostringstream oss;
    myIp.Print(oss);
    std::string data = oss.str();

    Ptr<Packet> p = Create<Packet>((uint8_t*)data.c_str(), data.size());
    m_beaconTx->SendTo(p, 0, InetSocketAddress(Ipv4Address("255.255.255.255"), 8888));

    m_beaconEvent = Simulator::Schedule(Seconds(0.5), &VehicleSdnApp::SendBeacon, this);
}

void VehicleSdnApp::ReceiveBeacon(Ptr<Socket> socket)
{
    Address from;
    Ptr<Packet> pkt = socket->RecvFrom(from);
    Ipv4Address senderIp = InetSocketAddress::ConvertFrom(from).GetIpv4();

    Ipv4Address myIp = GetNodeIpv4Address(GetNode(), m_ctrlIfIndex);
    if (senderIp != myIp) {
        m_neighbors.insert(senderIp); // control-plane IPs expected
    }
}

void VehicleSdnApp::SendReport()
{
    Ipv4Address myIp = GetNodeIpv4Address(GetNode(), m_ctrlIfIndex);
    if (myIp == Ipv4Address::GetZero()) return;

    std::ostringstream ss;
    ss << myIp;
    for (const auto& n : m_neighbors) ss << " " << n;
    std::string data = ss.str();

    Ptr<Packet> pkt = Create<Packet>((uint8_t*)data.c_str(), data.size());
    Ptr<Socket> s = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    s->Bind(InetSocketAddress(myIp, 0));
    s->Connect(InetSocketAddress(m_controllerIp, 9999));
    s->Send(pkt);
    s->Close();

    NS_LOG_INFO("SendReport from " << myIp << " to " << m_controllerIp << " neighbors=" << m_neighbors.size());

    m_neighbors.clear();
    m_reportEvent = Simulator::Schedule(Seconds(1.0), &VehicleSdnApp::SendReport, this);
}

void VehicleSdnApp::ReceiveRoute(Ptr<Socket> socket)
{
    Address from;
    Ptr<Packet> pkt = socket->RecvFrom(from);

    uint32_t dataSize = pkt->GetSize();
    if (dataSize == 0) return;

    std::vector<uint8_t> buffer(dataSize + 1, 0);
    pkt->CopyData(buffer.data(), dataSize);
    std::stringstream ss((char*)buffer.data());

    Ptr<Ipv4> ipv4 = GetNode()->GetObject<Ipv4>();
    Ptr<Ipv4StaticRouting> staticRouting;
    Ptr<Ipv4RoutingProtocol> rp = ipv4->GetRoutingProtocol();

    if (DynamicCast<Ipv4ListRouting>(rp)) {
        Ptr<Ipv4ListRouting> lr = DynamicCast<Ipv4ListRouting>(rp);
        int16_t priority;
        for (uint32_t i = 0; i < lr->GetNRoutingProtocols(); i++) {
            Ptr<Ipv4RoutingProtocol> temp = lr->GetRoutingProtocol(i, priority);
            if (DynamicCast<Ipv4StaticRouting>(temp)) {
                staticRouting = DynamicCast<Ipv4StaticRouting>(temp);
                break;
            }
        }
    } else {
        staticRouting = DynamicCast<Ipv4StaticRouting>(rp);
    }

    if (!staticRouting) return;

    std::string dstStr, nhStr;
    while (ss >> dstStr >> nhStr) {
        if (dstStr.empty() || nhStr.empty()) continue;
        if (dstStr.find('.') == std::string::npos) continue;
        if (nhStr.find('.') == std::string::npos) continue;

        Ipv4Address dst(dstStr.c_str());
        Ipv4Address nh(nhStr.c_str());

        Ipv4Address myDataIp = GetNodeIpv4Address(GetNode(), g_dataIfIndex);
        if (dst == myDataIp) continue;

        staticRouting->AddHostRouteTo(dst, nh, g_dataIfIndex, 0);
    }
}

// --- Controller ---

class SdnControllerApp : public Application
{
public:
    static TypeId GetTypeId();
    SdnControllerApp();

    void SetCtrlIfIndex(uint32_t idx) { m_ctrlIfIndex = idx; }

private:
    virtual void StartApplication();
    virtual void StopApplication();

    void ReceiveReport(Ptr<Socket> socket);
    void RecomputeRoutes();
    void SendBeacon();

    Ptr<Socket> m_recvSocket;
    Ptr<Socket> m_beaconTx;
    std::map<uint32_t, std::set<uint32_t>> m_topology; // NodeId graph
    EventId m_routeEvent;
    EventId m_beaconEvent;
    uint32_t m_ctrlIfIndex = 2;
};

NS_OBJECT_ENSURE_REGISTERED(SdnControllerApp);

TypeId SdnControllerApp::GetTypeId()
{
    static TypeId tid = TypeId("ns3::SdnControllerApp")
        .SetParent<Application>()
        .AddConstructor<SdnControllerApp>();
    return tid;
}

SdnControllerApp::SdnControllerApp() {}

void SdnControllerApp::StartApplication()
{
    Ipv4Address myCtrlIp = GetNodeIpv4Address(GetNode(), m_ctrlIfIndex);
    m_recvSocket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    m_recvSocket->Bind(InetSocketAddress(myCtrlIp, 9999));
    m_recvSocket->SetRecvCallback(MakeCallback(&SdnControllerApp::ReceiveReport, this));

    m_beaconTx = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    m_beaconTx->SetAllowBroadcast(true);
    m_beaconTx->Bind(InetSocketAddress(myCtrlIp, 0));

    m_beaconEvent = Simulator::Schedule(Seconds(0.5), &SdnControllerApp::SendBeacon, this);
    m_routeEvent = Simulator::Schedule(Seconds(2.0), &SdnControllerApp::RecomputeRoutes, this);
}

void SdnControllerApp::StopApplication()
{
    Simulator::Cancel(m_routeEvent);
    Simulator::Cancel(m_beaconEvent);
    if (m_recvSocket) m_recvSocket->Close();
    if (m_beaconTx) m_beaconTx->Close();
}

void SdnControllerApp::SendBeacon()
{
    Ipv4Address myIp = GetNodeIpv4Address(GetNode(), m_ctrlIfIndex);
    std::ostringstream oss;
    myIp.Print(oss);
    std::string data = oss.str();

    Ptr<Packet> p = Create<Packet>((uint8_t*)data.c_str(), data.size());
    m_beaconTx->SendTo(p, 0, InetSocketAddress(Ipv4Address("255.255.255.255"), 8888));

    m_beaconEvent = Simulator::Schedule(Seconds(0.5), &SdnControllerApp::SendBeacon, this);
}

void SdnControllerApp::ReceiveReport(Ptr<Socket> socket)
{
    Address from;
    Ptr<Packet> pkt = socket->RecvFrom(from);

    uint32_t dataSize = pkt->GetSize();
    if (dataSize == 0) return;

    std::vector<uint8_t> buffer(dataSize + 1, 0);
    pkt->CopyData(buffer.data(), dataSize);

    std::stringstream ss((char*)buffer.data());
    std::string srcStr;

    ss >> srcStr;
    if (srcStr.empty() || srcStr.find('.') == std::string::npos) return;

    Ipv4Address srcIp(srcStr.c_str());
    Ptr<Node> srcNode = FindNodeByCtrlIp(srcIp);
    if (!srcNode) {
        NS_LOG_WARN("ReceiveReport: Source IP " << srcIp << " does not match any known control IP.");
        return;
    }

    uint32_t srcId = srcNode->GetId();
    m_topology[srcId].clear();

    std::string nbrStr;
    while (ss >> nbrStr) {
        if (!nbrStr.empty() && nbrStr.find('.') != std::string::npos) {
            Ipv4Address nbrIp(nbrStr.c_str());
            Ptr<Node> nbrNode = FindNodeByCtrlIp(nbrIp);
            if (nbrNode) {
                uint32_t nbrId = nbrNode->GetId();
                m_topology[srcId].insert(nbrId);
                m_topology[nbrId].insert(srcId);
            } else {
                NS_LOG_WARN("ReceiveReport: Unknown neighbor IP " << nbrIp);
            }
        }
    }

    NS_LOG_INFO("ReceiveReport from Node " << srcId << " (" << srcIp << ") deg=" << m_topology[srcId].size());
}

static std::map<uint32_t,uint32_t>
ComputeNextHop(const std::map<uint32_t,std::set<uint32_t>>& topo, uint32_t srcId)
{
    std::map<uint32_t, uint32_t> nextHop;
    std::queue<uint32_t> q;
    std::set<uint32_t> visited;

    q.push(srcId);
    visited.insert(srcId);

    while (!q.empty())
    {
        uint32_t u = q.front();
        q.pop();

        auto it = topo.find(u);
        if (it == topo.end()) continue;

        for (const auto& v : it->second)
        {
            if (visited.find(v) == visited.end())
            {
                visited.insert(v);
                q.push(v);
                nextHop[v] = (u == srcId) ? v : nextHop[u];
            }
        }
    }
    return nextHop;
}

void SdnControllerApp::RecomputeRoutes()
{
    NS_LOG_INFO("Recomputing routes. Topology has " << m_topology.size() << " nodes");

    if (!g_allNodes) return;

    for (uint32_t i = 0; i < g_allNodes->GetN(); ++i)
    {
        Ptr<Node> node = g_allNodes->Get(i);
        uint32_t nodeId = node->GetId();
        if (nodeId == 0) continue; // skip controller

        auto nh = ComputeNextHop(m_topology, nodeId);

        std::ostringstream msg;
        for (auto &p : nh) {
            uint32_t dstId = p.first;
            uint32_t nhId  = p.second;

            Ptr<Node> dstNode = g_allNodes->Get(dstId);
            Ptr<Node> nhNode  = g_allNodes->Get(nhId);

            Ipv4Address dstData = GetDataIpByNode(dstNode);
            Ipv4Address nhData  = GetDataIpByNode(nhNode);

            msg << dstData << " " << nhData << " ";
        }

        std::string sMsg = msg.str();
        if (sMsg.empty()) continue;

        Ptr<Packet> pkt = Create<Packet>((uint8_t*)sMsg.c_str(), sMsg.size());

        Ipv4Address myCtrlIp = GetNodeIpv4Address(GetNode(), m_ctrlIfIndex);
        Ipv4Address nodeCtrlIp = GetCtrlIpByNode(node);

        Ptr<Socket> s = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        s->Bind(InetSocketAddress(myCtrlIp, 0));
        s->SendTo(pkt, 0, InetSocketAddress(nodeCtrlIp, 10000));
        s->Close();
    }

    m_routeEvent = Simulator::Schedule(Seconds(1.0), &SdnControllerApp::RecomputeRoutes, this);
}

} // namespace ns3

int main(int argc, char *argv[])
{
    LogComponentEnable("VanetSdnHybrid", LOG_LEVEL_INFO);

    std::string mobilityTrace = "scratch/mobility_10ms.tcl";
    uint32_t seed = 1;
    uint32_t run = 1;
    double speedMs = 10.0;
    double simTime = 300.0;
    uint32_t nNodes = 51; // controller=0, vehicles=1..50

    CommandLine cmd;
    cmd.AddValue("mobilityTrace", "Ns2 mobility trace file", mobilityTrace);
    cmd.AddValue("seed", "RngSeed", seed);
    cmd.AddValue("run", "RngRun", run);
    cmd.AddValue("speedMs", "Nominal speed of the trace (for logging only)", speedMs);
    cmd.Parse(argc, argv);

    RngSeedManager::SetSeed(seed);
    RngSeedManager::SetRun(run);

    NodeContainer nodes;
    nodes.Create(nNodes);
    g_allNodes = &nodes;

    Ns2MobilityHelper ns2(mobilityTrace);
    ns2.Install();
    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        Ptr<MobilityModel> mm = nodes.Get(i)->GetObject<MobilityModel>();
        if (!mm) {
            Ptr<ConstantPositionMobilityModel> cpm = CreateObject<ConstantPositionMobilityModel>();
            cpm->SetPosition(Vector(0.0, 0.0, 0.0));
            nodes.Get(i)->AggregateObject(cpm);
        }
    }
    Ptr<MobilityModel> mm0 = nodes.Get(0)->GetObject<MobilityModel>();
    mm0->SetPosition(Vector(250.0, 250.0, 0.0));

    // Data plane WiFi (802.11p short range)
    WifiHelper wifiData;
    wifiData.SetStandard(WIFI_STANDARD_80211p);
    wifiData.SetRemoteStationManager("ns3::ConstantRateWifiManager",
        "DataMode", StringValue("OfdmRate6MbpsBW10MHz"),
        "ControlMode", StringValue("OfdmRate6MbpsBW10MHz"));

    YansWifiChannelHelper dataChannel = YansWifiChannelHelper::Default(); // Friis
    YansWifiPhyHelper dataPhy;
    dataPhy.SetChannel(dataChannel.Create());
    dataPhy.Set("TxPowerStart", DoubleValue(18.0));
    dataPhy.Set("TxPowerEnd", DoubleValue(18.0));

    WifiMacHelper dataMac;
    dataMac.SetType("ns3::AdhocWifiMac");

    NetDeviceContainer dataDevs = wifiData.Install(dataPhy, dataMac, nodes);

    // Control plane WiFi (long range)
    WifiHelper wifiCtrl;
    wifiCtrl.SetStandard(WIFI_STANDARD_80211p);
    wifiCtrl.SetRemoteStationManager("ns3::ConstantRateWifiManager",
        "DataMode", StringValue("OfdmRate6MbpsBW10MHz"),
        "ControlMode", StringValue("OfdmRate6MbpsBW10MHz"));

    YansWifiChannelHelper ctrlChannel;
    ctrlChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    ctrlChannel.AddPropagationLoss("ns3::FriisPropagationLossModel");
    ctrlChannel.AddPropagationLoss("ns3::RangePropagationLossModel",
                                   "MaxRange", DoubleValue(1200.0)); // ensure controller reach

    YansWifiPhyHelper ctrlPhy;
    ctrlPhy.SetChannel(ctrlChannel.Create());
    ctrlPhy.Set("TxPowerStart", DoubleValue(20.0));
    ctrlPhy.Set("TxPowerEnd", DoubleValue(20.0));

    WifiMacHelper ctrlMac;
    ctrlMac.SetType("ns3::AdhocWifiMac");

    NetDeviceContainer ctrlDevs = wifiCtrl.Install(ctrlPhy, ctrlMac, nodes);

    InternetStackHelper internet;
    internet.Install(nodes);

    Ipv4AddressHelper ipv4Data;
    ipv4Data.SetBase("10.1.0.0", "255.255.0.0");
    Ipv4InterfaceContainer dataIfs = ipv4Data.Assign(dataDevs);

    Ipv4AddressHelper ipv4Ctrl;
    ipv4Ctrl.SetBase("10.2.0.0", "255.255.0.0");
    Ipv4InterfaceContainer ctrlIfs = ipv4Ctrl.Assign(ctrlDevs);

    g_dataIfIndex = 1;
    g_ctrlIfIndex = 2;

    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        nodes.Get(i)->GetObject<Ipv4>()->SetAttribute("IpForward", BooleanValue(true));
    }

    Ipv4Address controllerCtrlIp = ctrlIfs.GetAddress(0);
    NS_LOG_INFO("Controller control IP: " << controllerCtrlIp);
    NS_LOG_INFO("Controller data IP: " << dataIfs.GetAddress(0));

    Ptr<SdnControllerApp> ctrl = CreateObject<SdnControllerApp>();
    ctrl->SetCtrlIfIndex(g_ctrlIfIndex);
    nodes.Get(0)->AddApplication(ctrl);
    ctrl->SetStartTime(Seconds(0.1));
    ctrl->SetStopTime(Seconds(simTime - 1));

    for (uint32_t i = 1; i < nodes.GetN(); ++i) {
        Ptr<VehicleSdnApp> app = CreateObject<VehicleSdnApp>();
        app->SetControllerIp(controllerCtrlIp, g_ctrlIfIndex);
        nodes.Get(i)->AddApplication(app);
        app->SetStartTime(Seconds(0.2 + i * 0.05));
        app->SetStopTime(Seconds(simTime - 1));
    }

    Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
    uint32_t serverId, clientId;
    do {
        serverId = uv->GetInteger(1, nNodes - 1);
        clientId = uv->GetInteger(1, nNodes - 1);
    } while (serverId == clientId);

    UdpEchoServerHelper server(9);
    ApplicationContainer serverApp = server.Install(nodes.Get(serverId));
    serverApp.Start(Seconds(5.0));
    serverApp.Stop(Seconds(simTime - 2));

    UdpEchoClientHelper client(dataIfs.GetAddress(serverId), 9);
    client.SetAttribute("Interval", TimeValue(Seconds(0.25)));   // 4 pkt/s
    client.SetAttribute("PacketSize", UintegerValue(1024));      // 1024 bytes
    client.SetAttribute("MaxPackets", UintegerValue(1000000000));
    ApplicationContainer clientApp = client.Install(nodes.Get(clientId));
    clientApp.Start(Seconds(10.0));
    clientApp.Stop(Seconds(simTime - 2));

    NS_LOG_INFO("Server node: " << serverId << " data IP: " << dataIfs.GetAddress(serverId));
    NS_LOG_INFO("Client node: " << clientId << " data IP: " << dataIfs.GetAddress(clientId));

    Simulator::Stop(Seconds(simTime));

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    AnimationInterface anim("sdn-animation.xml");
    anim.EnablePacketMetadata(true);

    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        anim.UpdateNodeDescription(nodes.Get(i)->GetId(), "Car");
        anim.UpdateNodeSize(nodes.Get(i)->GetId(), 3.0, 3.0);
        anim.UpdateNodeColor(nodes.Get(i)->GetId(), 180, 180, 180);
    }

    anim.UpdateNodeDescription(nodes.Get(0)->GetId(), "SDN Controller");
    anim.UpdateNodeColor(nodes.Get(0)->GetId(), 255, 0, 0);
    anim.UpdateNodeSize(nodes.Get(0)->GetId(), 5.0, 5.0);

    anim.UpdateNodeDescription(nodes.Get(serverId)->GetId(), "Server");
    anim.UpdateNodeColor(nodes.Get(serverId)->GetId(), 0, 200, 200);
    anim.UpdateNodeSize(nodes.Get(serverId)->GetId(), 4.0, 4.0);

    anim.UpdateNodeDescription(nodes.Get(clientId)->GetId(), "Client");
    anim.UpdateNodeColor(nodes.Get(clientId)->GetId(), 0, 120, 255);
    anim.UpdateNodeSize(nodes.Get(clientId)->GetId(), 4.0, 4.0);

    Simulator::Run();

    monitor->CheckForLostPackets();

    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

    uint64_t totalTxPackets = 0;
    uint64_t totalRxPackets = 0;

    std::cout << "\n========== Flow Statistics ==========\n";

    for (const auto& iter : stats) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(iter.first);

        if (t.destinationPort != 9 && t.sourcePort != 9) continue;

        std::cout << "Flow " << iter.first << " ("
                  << t.sourceAddress << ":" << t.sourcePort << " -> "
                  << t.destinationAddress << ":" << t.destinationPort << ")\n";
        std::cout << "  Tx Packets: " << iter.second.txPackets << "\n";
        std::cout << "  Rx Packets: " << iter.second.rxPackets << "\n";
        std::cout << "  Lost Packets: " << iter.second.lostPackets << "\n";

        if (iter.second.txPackets > 0) {
            double pdr = 100.0 * iter.second.rxPackets / iter.second.txPackets;
            std::cout << "  PDR: " << pdr << "%\n";
        }

        if (iter.second.rxPackets > 0) {
            double avgDelay = iter.second.delaySum.GetSeconds() / iter.second.rxPackets;
            std::cout << "  Avg Delay: " << avgDelay * 1000 << " ms\n";
        }

        totalTxPackets += iter.second.txPackets;
        totalRxPackets += iter.second.rxPackets;
    }

    std::cout << "\n========== Overall Statistics ==========\n";
    std::cout << "Total Tx Packets: " << totalTxPackets << "\n";
    std::cout << "Total Rx Packets: " << totalRxPackets << "\n";

    double overallPDR = (totalTxPackets > 0) ? (100.0 * totalRxPackets / totalTxPackets) : 0.0;
    std::cout << "Overall PDR: " << overallPDR << "%\n";
    std::cout << "RESULT"
              << ",seed=" << seed
              << ",speed=" << speedMs
              << ",tx=" << totalTxPackets
              << ",rx=" << totalRxPackets
              << ",pdr=" << overallPDR
              << ",server=" << serverId
              << ",client=" << clientId
              << std::endl;
    std::cout << "========================================\n";

    monitor->SerializeToXmlFile("sdn-results.xml", true, true);

    Simulator::Destroy();
    g_allNodes = nullptr;

    return 0;
}