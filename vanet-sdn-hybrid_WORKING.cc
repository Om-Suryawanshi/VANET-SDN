#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/ipv4-static-routing.h"
#include "ns3/ipv4-list-routing.h"

#include "ns3/netanim-module.h"
#include "ns3/flow-monitor-module.h"

#include <queue>
#include <sstream>
#include <set>
#include <map>

// Log Component
NS_LOG_COMPONENT_DEFINE("VanetSdnHybrid");

using namespace ns3;

namespace ns3 {

// --- HELPER FUNCTION MOVED INSIDE NAMESPACE ---
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

// --- VEHICLE APP DECLARATION ---
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
};

// --- CONTROLLER APP DECLARATION ---
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

  Ptr<Socket> m_recvSocket;
  std::map<Ipv4Address, std::set<Ipv4Address>> m_topology;
};

// --- VEHICLE APP IMPLEMENTATION ---
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

  m_beaconRx = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
  m_beaconRx->Bind(InetSocketAddress(Ipv4Address::GetAny(), 8888));
  m_beaconRx->SetRecvCallback(MakeCallback(&VehicleSdnApp::ReceiveBeacon, this));

  m_ctrlRx = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
  m_ctrlRx->Bind(InetSocketAddress(Ipv4Address::GetAny(), 10000));
  m_ctrlRx->SetRecvCallback(MakeCallback(&VehicleSdnApp::ReceiveRoute, this));

  Simulator::Schedule(Seconds(0.5), &VehicleSdnApp::SendBeacon, this);
  Simulator::Schedule(Seconds(1.0), &VehicleSdnApp::SendReport, this);
}

void VehicleSdnApp::StopApplication()
{
  if(m_beaconTx) m_beaconTx->Close();
  if(m_beaconRx) m_beaconRx->Close();
  if(m_ctrlRx) m_ctrlRx->Close();
}

void VehicleSdnApp::SendBeacon()
{
  Ptr<Packet> p = Create<Packet>(10);
  m_beaconTx->SendTo(p, 0, InetSocketAddress(Ipv4Address("255.255.255.255"), 8888));
  Simulator::Schedule(Seconds(0.5), &VehicleSdnApp::SendBeacon, this);
}

void VehicleSdnApp::ReceiveBeacon(Ptr<Socket> socket)
{
  Address from;
  socket->RecvFrom(from);
  m_neighbors.insert(InetSocketAddress::ConvertFrom(from).GetIpv4());
}

void VehicleSdnApp::SendReport()
{
  Ipv4Address myIp = GetNodeIpv4Address(GetNode());

  std::ostringstream ss;
  ss << myIp << " ";
  for (auto n : m_neighbors) ss << n << " ";

  Ptr<Packet> pkt = Create<Packet>((uint8_t*)ss.str().c_str(), ss.str().size());
  Ptr<Socket> s = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
  
  if (m_controllerIp != Ipv4Address::GetZero()) {
      s->SendTo(pkt, 0, InetSocketAddress(m_controllerIp, 9999));
  }
  
  m_neighbors.clear();
  Simulator::Schedule(Seconds(1.0), &VehicleSdnApp::SendReport, this);
}

void VehicleSdnApp::ReceiveRoute(Ptr<Socket> socket)
{
    Address from;
    Ptr<Packet> pkt = socket->RecvFrom(from);

    // FIX 2: Safer buffer handling
    uint32_t dataSize = pkt->GetSize();
    char buf[2048] = {0};
    if (dataSize >= 2048) dataSize = 2047; // prevent overflow
    pkt->CopyData((uint8_t*)buf, dataSize);
    buf[dataSize] = '\0'; // Ensure null-termination

    std::stringstream ss(buf);

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

    // Do NOT clear routes here, it breaks connectivity.
    // Just add new ones.

    std::string dstStr, nhStr;
    while (ss >> dstStr >> nhStr) {
        // FIX 2: Validate strings before creating Ipv4Address
        if (dstStr.empty() || nhStr.empty()) continue;
        if (dstStr.find('.') == std::string::npos) continue; 
        if (nhStr.find('.') == std::string::npos) continue;

        Ipv4Address dst(dstStr.c_str());
        Ipv4Address nh(nhStr.c_str());

        // Interface 1 is usually WiFi
        staticRouting->AddHostRouteTo(dst, nh, 1, 1);
    }
}

// --- CONTROLLER APP IMPLEMENTATION ---
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
  Simulator::Schedule(Seconds(1.0), &SdnControllerApp::RecomputeRoutes, this);
}

void SdnControllerApp::StopApplication()
{
  if (m_recvSocket) m_recvSocket->Close();
}

void SdnControllerApp::ReceiveReport(Ptr<Socket> socket)
{
  Address from;
  Ptr<Packet> pkt = socket->RecvFrom(from);

  char buf[2048] = {0};
  uint32_t dataSize = pkt->GetSize();
  if (dataSize >= 2048) dataSize = 2047;
  pkt->CopyData((uint8_t*)buf, dataSize);
  buf[dataSize] = '\0';
  
  std::stringstream ss(buf);
  Ipv4Address src;
  std::string srcStr;
  
  ss >> srcStr;
  if (srcStr.empty() || srcStr.find('.') == std::string::npos) return;
  src = Ipv4Address(srcStr.c_str());

  m_topology[src].clear();

  std::string nbrStr;
  while (ss >> nbrStr) {
      if (!nbrStr.empty() && nbrStr.find('.') != std::string::npos) {
          m_topology[src].insert(Ipv4Address(nbrStr.c_str()));
      }
  }
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
    Ipv4Address u = q.front(); q.pop();
    auto it = topo.find(u);
    if (it == topo.end()) continue;

    for (auto v : it->second)
    {
      if (!visited.count(v))
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
  for (auto& entry : m_topology)
  {
    Ipv4Address src = entry.first;
    if (src == Ipv4Address::GetZero()) continue;
    
    auto nh = ComputeNextHop(m_topology, src);

    std::ostringstream msg;
    for (auto& p : nh)
      msg << p.first << " " << p.second << " ";

    std::string sMsg = msg.str();
    if (sMsg.empty()) continue;

    Ptr<Packet> pkt = Create<Packet>((uint8_t*)sMsg.c_str(), sMsg.size());
    Ptr<Socket> s = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    s->SendTo(pkt, 0, InetSocketAddress(src, 10000));
  }
  Simulator::Schedule(Seconds(1.0), &SdnControllerApp::RecomputeRoutes, this);
}

} // End namespace ns3

int main(int argc, char *argv[])
{
  LogComponentEnable("VanetSdnHybrid", LOG_LEVEL_INFO);
  // LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
  // LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);

  uint32_t nNodes = 20;
  
  ns3::NodeContainer nodes;
  nodes.Create(nNodes);

  ns3::Config::SetDefault ("ns3::RandomRectanglePositionAllocator::X", 
    ns3::StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1000.0]"));
  ns3::Config::SetDefault ("ns3::RandomRectanglePositionAllocator::Y", 
    ns3::StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1000.0]"));

  ns3::MobilityHelper mobility;
  // mobility.SetMobilityModel("ns3::RandomWaypointMobilityModel",
  //   "Speed", ns3::StringValue("ns3::UniformRandomVariable[Min=5|Max=20]"),
  //   "Pause", ns3::StringValue("ns3::ConstantRandomVariable[Constant=0]"),
  //   "PositionAllocator", ns3::StringValue("ns3::RandomRectanglePositionAllocator"));

  mobility.SetPositionAllocator ("ns3::RandomRectanglePositionAllocator");

  // Set the Model for Movement (t>0), forcing it to use the same allocator type
  mobility.SetMobilityModel("ns3::RandomWaypointMobilityModel",
    "Speed", ns3::StringValue("ns3::UniformRandomVariable[Min=5|Max=20]"),
    "Pause", ns3::StringValue("ns3::ConstantRandomVariable[Constant=0]"),
    "PositionAllocator", ns3::StringValue("ns3::RandomRectanglePositionAllocator"));

  
  mobility.Install(nodes);

  ns3::WifiHelper wifi;
  wifi.SetStandard(ns3::WIFI_STANDARD_80211p);

  ns3::YansWifiChannelHelper channel = ns3::YansWifiChannelHelper::Default();
  ns3::YansWifiPhyHelper phy;
  phy.SetChannel(channel.Create());
  phy.Set("TxPowerStart", ns3::DoubleValue(20.0));
  phy.Set("TxPowerEnd", ns3::DoubleValue(20.0));

  ns3::WifiMacHelper mac;
  mac.SetType("ns3::AdhocWifiMac");

  ns3::NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

  ns3::InternetStackHelper internet;
  internet.Install(nodes);

  ns3::Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.1.0.0", "255.255.0.0");
  ns3::Ipv4InterfaceContainer ifs = ipv4.Assign(devices);
  
  ns3::Ipv4Address controllerIp = ns3::GetNodeIpv4Address(nodes.Get(0));

  // Install Controller
  ns3::Ptr<ns3::SdnControllerApp> ctrl = ns3::CreateObject<ns3::SdnControllerApp>();
  nodes.Get(0)->AddApplication(ctrl);
  ctrl->SetStartTime(ns3::Seconds(0.5));

  // Install Vehicle Apps
  for (uint32_t i = 1; i < nodes.GetN(); ++i) // Node 0 is controller
  {
    ns3::Ptr<ns3::VehicleSdnApp> app = ns3::CreateObject<ns3::VehicleSdnApp>();
    app->SetControllerIp(controllerIp);
    nodes.Get(i)->AddApplication(app);
    app->SetStartTime(ns3::Seconds(1.0 + i * 0.01));
  }

  // Traffic
  ns3::UdpEchoServerHelper server(9);
  ns3::ApplicationContainer serverApp = server.Install(nodes.Get(1));
  serverApp.Start(ns3::Seconds(1.0));

  ns3::UdpEchoClientHelper client(ifs.GetAddress(1), 9);
  client.SetAttribute("Interval", ns3::TimeValue(ns3::Seconds(0.25)));
  client.SetAttribute("PacketSize", ns3::UintegerValue(1024));
  client.SetAttribute("MaxPackets", ns3::UintegerValue(10000));
  
  ns3::ApplicationContainer clientApp = client.Install(nodes.Get(2));
  clientApp.Start(ns3::Seconds(3.0));
 
  ns3::Simulator::Stop(ns3::Seconds(60));

  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();

  AnimationInterface anim("sdn-animation.xml");
  // anim.SetMaxPktsPerTraceFile(50000);
  
  for (uint32_t i = 0; i < nodes.GetN(); ++i) {
    anim.UpdateNodeDescription(nodes.Get(i)->GetId(), "Car");
    anim.UpdateNodeSize(nodes.Get(i)->GetId(), 3.0, 3.0); // Make them visible
  }

  // Highlight the Controller (Node 0)
  anim.UpdateNodeDescription(nodes.Get(0)->GetId(), "SDN Controller");
  anim.UpdateNodeColor(nodes.Get(0)->GetId(), 255, 0, 0); // Red Color
  anim.UpdateNodeSize(nodes.Get(0)->GetId(), 5.0, 5.0);

  ns3::Simulator::Run();

  monitor->CheckForLostPackets();
  monitor->SerializeToXmlFile("sdn-results.xml", true, true);

  ns3::Simulator::Destroy();
  
  return 0;
}