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

#include <queue>
#include <sstream>
#include <set>
#include <map>
#include <vector>

NS_LOG_COMPONENT_DEFINE("VanetSdnHybrid");

using namespace ns3;

namespace ns3 {

NodeContainer* g_allNodes = nullptr;

Ipv4Address GetNodeIpv4Address(Ptr<Node> node)
{
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    for (uint32_t i = 0; i < ipv4->GetNInterfaces(); ++i) {
        for (uint32_t j = 0; j < ipv4->GetNAddresses(i); ++j) {
            Ipv4Address addr = ipv4->GetAddress(i, j).GetLocal();
            if (addr != Ipv4Address::GetLoopback() &&
                addr != Ipv4Address::GetZero()) {
                return addr;
            }
        }
    }
    return Ipv4Address::GetZero();
}

Ptr<Node> FindNodeByIp(Ipv4Address ip)
{
    if (g_allNodes == nullptr) return nullptr;
    for (uint32_t i = 0; i < g_allNodes->GetN(); ++i) {
        if (GetNodeIpv4Address(g_allNodes->Get(i)) == ip) {
            return g_allNodes->Get(i);
        }
    }
    return nullptr;
}

class VehicleSdnApp : public Application
{
public:
    static TypeId GetTypeId();
    VehicleSdnApp();
    void SetControllerIp(Ipv4Address ip);

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

void VehicleSdnApp::SetControllerIp(Ipv4Address ip)
{
    m_controllerIp = ip;
}

void VehicleSdnApp::StartApplication()
{
    m_beaconTx = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    m_beaconTx->SetAllowBroadcast(true);
    m_beaconTx->Bind();

    m_beaconRx = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    m_beaconRx->Bind(InetSocketAddress(Ipv4Address::GetAny(), 8888));
    m_beaconRx->SetRecvCallback(MakeCallback(&VehicleSdnApp::ReceiveBeacon, this));

    m_ctrlRx = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    m_ctrlRx->Bind(InetSocketAddress(Ipv4Address::GetAny(), 10000));
    m_ctrlRx->SetRecvCallback(MakeCallback(&VehicleSdnApp::ReceiveRoute, this));

    m_beaconEvent = Simulator::Schedule(Seconds(0.1), &VehicleSdnApp::SendBeacon, this);
    m_reportEvent = Simulator::Schedule(Seconds(0.5), &VehicleSdnApp::SendReport, this);
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
    Ipv4Address myIp = GetNodeIpv4Address(GetNode());
    std::ostringstream ss;
    ss << myIp;
    std::string data = ss.str();

    Ptr<Packet> p = Create<Packet>((uint8_t*)data.c_str(), data.size());
    m_beaconTx->SendTo(p, 0, InetSocketAddress(Ipv4Address("255.255.255.255"), 8888));

    m_beaconEvent = Simulator::Schedule(Seconds(0.25), &VehicleSdnApp::SendBeacon, this);
}

void VehicleSdnApp::ReceiveBeacon(Ptr<Socket> socket)
{
    Address from;
    Ptr<Packet> pkt = socket->RecvFrom(from);
    Ipv4Address senderIp = InetSocketAddress::ConvertFrom(from).GetIpv4();

    Ipv4Address myIp = GetNodeIpv4Address(GetNode());
    if (senderIp != myIp) {
        m_neighbors.insert(senderIp);
    }
}

void VehicleSdnApp::SendReport()
{
    Ipv4Address myIp = GetNodeIpv4Address(GetNode());

    std::ostringstream ss;
    ss << myIp;
    for (const auto& n : m_neighbors) {
        ss << " " << n;
    }

    std::string data = ss.str();
    Ptr<Packet> pkt = Create<Packet>((uint8_t*)data.c_str(), data.size());

    Ptr<Socket> s = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    s->Connect(InetSocketAddress(m_controllerIp, 9999));
    s->Send(pkt);
    s->Close();

    m_neighbors.clear();
    m_reportEvent = Simulator::Schedule(Seconds(0.5), &VehicleSdnApp::SendReport, this);
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

        Ipv4Address myIp = GetNodeIpv4Address(GetNode());
        if (dst == myIp) continue;

        staticRouting->AddHostRouteTo(dst, nh, 1, 0);
    }
}

class SdnControllerApp : public Application
{
public:
    static TypeId GetTypeId();
    SdnControllerApp();

private:
    virtual void StartApplication();
    virtual void StopApplication();

    void ReceiveReport(Ptr<Socket> socket);
    void RecomputeRoutes();
    void SendBeacon();

    Ptr<Socket> m_recvSocket;
    Ptr<Socket> m_beaconTx;
    std::map<Ipv4Address, std::set<Ipv4Address>> m_topology;
    EventId m_routeEvent;
    EventId m_beaconEvent;
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
    m_recvSocket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    m_recvSocket->Bind(InetSocketAddress(Ipv4Address::GetAny(), 9999));
    m_recvSocket->SetRecvCallback(MakeCallback(&SdnControllerApp::ReceiveReport, this));

    m_beaconTx = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    m_beaconTx->SetAllowBroadcast(true);
    m_beaconTx->Bind();

    m_beaconEvent = Simulator::Schedule(Seconds(0.1), &SdnControllerApp::SendBeacon, this);
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
    Ipv4Address myIp = GetNodeIpv4Address(GetNode());
    std::ostringstream ss;
    ss << myIp;
    std::string data = ss.str();

    Ptr<Packet> p = Create<Packet>((uint8_t*)data.c_str(), data.size());
    m_beaconTx->SendTo(p, 0, InetSocketAddress(Ipv4Address("255.255.255.255"), 8888));

    m_beaconEvent = Simulator::Schedule(Seconds(0.25), &SdnControllerApp::SendBeacon, this);
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

    Ipv4Address src(srcStr.c_str());
    m_topology[src].clear();

    std::string nbrStr;
    while (ss >> nbrStr) {
        if (!nbrStr.empty() && nbrStr.find('.') != std::string::npos) {
            Ipv4Address nbr(nbrStr.c_str());
            m_topology[src].insert(nbr);
            m_topology[nbr].insert(src);
        }
    }

    NS_LOG_INFO("Topology update from " << src << " with " << m_topology[src].size() << " neighbors");
}

static std::map<Ipv4Address, Ipv4Address>
ComputeNextHop(const std::map<Ipv4Address, std::set<Ipv4Address>>& topo, Ipv4Address src)
{
    std::map<Ipv4Address, Ipv4Address> nextHop;
    std::queue<Ipv4Address> q;
    std::set<Ipv4Address> visited;

    q.push(src);
    visited.insert(src);

    while (!q.empty())
    {
        Ipv4Address u = q.front();
        q.pop();

        auto it = topo.find(u);
        if (it == topo.end()) continue;

        for (const auto& v : it->second)
        {
            if (visited.find(v) == visited.end())
            {
                visited.insert(v);
                q.push(v);
                nextHop[v] = (u == src) ? v : nextHop[u];
            }
        }
    }
    return nextHop;
}

void SdnControllerApp::RecomputeRoutes()
{
    NS_LOG_INFO("Recomputing routes.  Topology has " << m_topology.size() << " nodes");

    std::set<Ipv4Address> allNodes;
    for (const auto& entry : m_topology) {
        allNodes.insert(entry.first);
        for (const auto& nbr : entry.second) {
            allNodes.insert(nbr);
        }
    }

    for (const auto& nodeIp : allNodes)
    {
        if (nodeIp == Ipv4Address::GetZero()) continue;

        Ipv4Address myIp = GetNodeIpv4Address(GetNode());
        if (nodeIp == myIp) continue;

        auto nh = ComputeNextHop(m_topology, nodeIp);

        std::ostringstream msg;
        for (const auto& p : nh) {
            msg << p.first << " " << p.second << " ";
        }

        std::string sMsg = msg.str();
        if (sMsg.empty()) continue;

        Ptr<Packet> pkt = Create<Packet>((uint8_t*)sMsg.c_str(), sMsg.size());
        Ptr<Socket> s = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        s->SendTo(pkt, 0, InetSocketAddress(nodeIp, 10000));
        s->Close();

        NS_LOG_INFO("Sent routes to " << nodeIp << ": " << sMsg);
    }

    m_routeEvent = Simulator::Schedule(Seconds(1.0), &SdnControllerApp::RecomputeRoutes, this);
}

} // namespace ns3

int main(int argc, char *argv[])
{
    LogComponentEnable("VanetSdnHybrid", LOG_LEVEL_INFO);

    uint32_t nNodes = 20;
    double simTime = 60.0;

    NodeContainer nodes;
    nodes.Create(nNodes);

    g_allNodes = &nodes;

    Config::SetDefault("ns3::RandomRectanglePositionAllocator::X",
        StringValue("ns3::UniformRandomVariable[Min=0.0|Max=500.0]"));
    Config::SetDefault("ns3::RandomRectanglePositionAllocator::Y",
        StringValue("ns3::UniformRandomVariable[Min=0.0|Max=500.0]"));

    MobilityHelper mobilityVehicles;
    mobilityVehicles.SetPositionAllocator("ns3::RandomRectanglePositionAllocator");
    mobilityVehicles.SetMobilityModel("ns3::RandomWaypointMobilityModel",
        "Speed", StringValue("ns3::UniformRandomVariable[Min=5|Max=15]"),
        "Pause", StringValue("ns3::ConstantRandomVariable[Constant=1]"),
        "PositionAllocator", StringValue("ns3::RandomRectanglePositionAllocator"));

    NodeContainer vehicleNodes;
    for (uint32_t i = 1; i < nNodes; ++i) {
        vehicleNodes.Add(nodes.Get(i));
    }
    mobilityVehicles.Install(vehicleNodes);

    MobilityHelper mobilityController;
    Ptr<ListPositionAllocator> posAlloc = CreateObject<ListPositionAllocator>();
    posAlloc->Add(Vector(250.0, 250.0, 0.0));
    mobilityController.SetPositionAllocator(posAlloc);
    mobilityController.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityController.Install(nodes.Get(0));

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211p);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
        "DataMode", StringValue("OfdmRate6MbpsBW10MHz"),
        "ControlMode", StringValue("OfdmRate6MbpsBW10MHz"));

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());
    phy.Set("TxPowerStart", DoubleValue(33.0));
    phy.Set("TxPowerEnd", DoubleValue(33.0));

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");

    NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

    InternetStackHelper internet;
    internet.Install(nodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.0.0", "255.255.0.0");
    Ipv4InterfaceContainer ifs = ipv4.Assign(devices);

    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        nodes.Get(i)->GetObject<Ipv4>()->SetAttribute("IpForward", BooleanValue(true));
    }

    Ipv4Address controllerIp = GetNodeIpv4Address(nodes.Get(0));
    NS_LOG_INFO("Controller IP: " << controllerIp);

    Ptr<SdnControllerApp> ctrl = CreateObject<SdnControllerApp>();
    nodes.Get(0)->AddApplication(ctrl);
    ctrl->SetStartTime(Seconds(0.1));
    ctrl->SetStopTime(Seconds(simTime - 1));

    for (uint32_t i = 1; i < nodes.GetN(); ++i) {
        Ptr<VehicleSdnApp> app = CreateObject<VehicleSdnApp>();
        app->SetControllerIp(controllerIp);
        nodes.Get(i)->AddApplication(app);
        app->SetStartTime(Seconds(0.2 + i * 0.05));
        app->SetStopTime(Seconds(simTime - 1));
    }

    UdpEchoServerHelper server(9);
    ApplicationContainer serverApp = server.Install(nodes.Get(1));
    serverApp.Start(Seconds(5.0));
    serverApp.Stop(Seconds(simTime - 2));

    UdpEchoClientHelper client(ifs.GetAddress(1), 9);
    client.SetAttribute("Interval", TimeValue(Seconds(0.5)));
    client.SetAttribute("PacketSize", UintegerValue(512));
    client.SetAttribute("MaxPackets", UintegerValue(100));

    ApplicationContainer clientApp = client.Install(nodes.Get(2));
    clientApp.Start(Seconds(10.0));
    clientApp.Stop(Seconds(simTime - 2));

    NS_LOG_INFO("Server on Node 1: " << ifs.GetAddress(1));
    NS_LOG_INFO("Client on Node 2: " << ifs.GetAddress(2));

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

    // Controller
    anim.UpdateNodeDescription(nodes.Get(0)->GetId(), "SDN Controller");
    anim.UpdateNodeColor(nodes.Get(0)->GetId(), 255, 0, 0);
    anim.UpdateNodeSize(nodes.Get(0)->GetId(), 5.0, 5.0);

    // Server node 1 
    anim.UpdateNodeDescription(nodes.Get(1)->GetId(), "Server");
    anim.UpdateNodeColor(nodes.Get(1)->GetId(), 0, 200, 200);
    anim.UpdateNodeSize(nodes.Get(1)->GetId(), 4.0, 4.0);

    // Client node 2
    anim.UpdateNodeDescription(nodes.Get(2)->GetId(), "Client");
    anim.UpdateNodeColor(nodes.Get(2)->GetId(), 0, 120, 255);
    anim.UpdateNodeSize(nodes.Get(2)->GetId(), 4.0, 4.0);

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

    if (totalTxPackets > 0) {
        double overallPDR = 100.0 * totalRxPackets / totalTxPackets;
        std::cout << "Overall PDR: " << overallPDR << "%\n";
    }
    std::cout << "========================================\n";

    monitor->SerializeToXmlFile("sdn-results.xml", true, true);

    Simulator::Destroy();
    g_allNodes = nullptr;

    return 0;
}
