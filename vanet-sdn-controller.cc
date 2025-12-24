/*
 * Reproduction of Figure 10 (SDN Line) 
 * Reference: "Towards Software-Defined VANET: Architecture and Services"
 * * Logic:
 * - 50 Vehicles (SUMO Trace) + 1 Static Controller (Node 50).
 * - Vehicles run "VehicleSdnApp": Send Neighbors -> Controller -> Receive Route.
 * - Controller runs "SdnControllerApp": Builds Graph -> Dijkstra -> Push Routes.
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h" 
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-list-routing-helper.h"

#include <vector>
#include <map>
#include <set>
#include <queue>
#include <string>
#include <sstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("VanetSdnController");

// Global helper to find nodes (since apps run deep in simulation)
NodeContainer* g_allNodes = nullptr;

// ============================================================================
//                                HELPERS
// ============================================================================

Ipv4Address GetNodeIpv4Address(Ptr<Node> node, uint32_t ifIndex)
{
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    if (!ipv4 || ifIndex >= ipv4->GetNInterfaces()) return Ipv4Address::GetZero();
    return ipv4->GetAddress(ifIndex, 0).GetLocal();
}

// Find a node pointer given its Control IP (10.2.x.x)
Ptr<Node> FindNodeByCtrlIp(Ipv4Address ctrlIp) {
  if (!g_allNodes) return nullptr;
  for (uint32_t i = 0; i < g_allNodes->GetN(); ++i) {
      // Vehicles have Ctrl at index 2, Controller at index 1
      // We check all interfaces to be safe
      Ptr<Ipv4> ipv4 = g_allNodes->Get(i)->GetObject<Ipv4>();
      for (uint32_t j = 0; j < ipv4->GetNInterfaces(); ++j) {
          if (ipv4->GetAddress(j, 0).GetLocal() == ctrlIp) return g_allNodes->Get(i);
      }
  }
  return nullptr;
}

// ============================================================================
//                              VEHICLE APP
// ============================================================================
class VehicleSdnApp : public Application
{
public:
    static TypeId GetTypeId();
    VehicleSdnApp();
    void Setup(Ipv4Address controllerIp, uint32_t dataIfIndex, uint32_t ctrlIfIndex);

private:
    virtual void StartApplication();
    virtual void StopApplication();

    void SendBeacon();
    void ReceiveBeacon(Ptr<Socket> socket);
    void SendReport();
    void ReceiveRoute(Ptr<Socket> socket);

    Ptr<Socket> m_beaconTx;
    Ptr<Socket> m_beaconRx; // Listens for Beacons (Data Plane)
    Ptr<Socket> m_ctrlRx;   // Listens for Routes (Control Plane)
    Ptr<Socket> m_reportSocket; // Sends reports

    Ipv4Address m_controllerIp;
    uint32_t m_dataIfIndex; // Interface for V2V data (Beacons)
    uint32_t m_ctrlIfIndex; // Interface for LTE/Control
    std::set<Ipv4Address> m_neighbors; // Neighbors detected via Beacons

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

VehicleSdnApp::VehicleSdnApp() : m_dataIfIndex(1), m_ctrlIfIndex(2) {}

void VehicleSdnApp::Setup(Ipv4Address controllerIp, uint32_t dataIfIndex, uint32_t ctrlIfIndex)
{
    m_controllerIp = controllerIp;
    m_dataIfIndex = dataIfIndex;
    m_ctrlIfIndex = ctrlIfIndex;
}

void VehicleSdnApp::StartApplication()
{
    // 1. Beacon TX (Broadcast on Data Interface)
    m_beaconTx = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    m_beaconTx->SetAllowBroadcast(true);
    // Bind to Data IP so packets go out correctly
    Ipv4Address myDataIp = GetNodeIpv4Address(GetNode(), m_dataIfIndex);
    m_beaconTx->Bind(InetSocketAddress(myDataIp, 0));

    // 2. Beacon RX (Listen on Port 8888)
    m_beaconRx = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    m_beaconRx->Bind(InetSocketAddress(Ipv4Address::GetAny(), 8888));
    m_beaconRx->SetRecvCallback(MakeCallback(&VehicleSdnApp::ReceiveBeacon, this));

    // 3. Control RX (Listen on Port 10000 for Routes)
    m_ctrlRx = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    Ipv4Address myCtrlIp = GetNodeIpv4Address(GetNode(), m_ctrlIfIndex);
    m_ctrlRx->Bind(InetSocketAddress(myCtrlIp, 10000));
    m_ctrlRx->SetRecvCallback(MakeCallback(&VehicleSdnApp::ReceiveRoute, this));

    // Schedule Events
    // Jitter start times to avoid collisions
    double jitter = (double)(GetNode()->GetId()) * 0.01; 
    m_beaconEvent = Simulator::Schedule(Seconds(0.5 + jitter), &VehicleSdnApp::SendBeacon, this);
    m_reportEvent = Simulator::Schedule(Seconds(1.0 + jitter), &VehicleSdnApp::SendReport, this);
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
    // Broadcast my Data IP so neighbors know who I am
    Ipv4Address myDataIp = GetNodeIpv4Address(GetNode(), m_dataIfIndex);
    std::ostringstream oss;
    oss << myDataIp; // Payload: "10.1.X.X"
    std::string data = oss.str();

    Ptr<Packet> p = Create<Packet>((uint8_t*)data.c_str(), data.size());
    // Send to Broadcast on Port 8888
    m_beaconTx->SendTo(p, 0, InetSocketAddress(Ipv4Address("255.255.255.255"), 8888));

    // Schedule next beacon (500ms interval per paper)
    m_beaconEvent = Simulator::Schedule(Seconds(0.5), &VehicleSdnApp::SendBeacon, this);
}

void VehicleSdnApp::ReceiveBeacon(Ptr<Socket> socket)
{
    Address from;
    Ptr<Packet> pkt = socket->RecvFrom(from);
    if (pkt->GetSize() == 0) return;

    std::vector<uint8_t> buffer(pkt->GetSize() + 1, 0);
    pkt->CopyData(buffer.data(), pkt->GetSize());
    std::string senderIpStr((char*)buffer.data());
    
    // Validate IP string
    if (senderIpStr.find(".") == std::string::npos) return;
    
    Ipv4Address neighborIp(senderIpStr.c_str());
    Ipv4Address myIp = GetNodeIpv4Address(GetNode(), m_dataIfIndex);

    if (neighborIp != myIp && neighborIp != Ipv4Address::GetZero()) {
        m_neighbors.insert(neighborIp);
    }
}

void VehicleSdnApp::SendReport()
{
    // Send Neighbor List to Controller (Control Plane)
    Ipv4Address myCtrlIp = GetNodeIpv4Address(GetNode(), m_ctrlIfIndex);
    
    // Format: "MyCtrlIP Nbr1DataIP Nbr2DataIP ..."
    std::ostringstream ss;
    ss << myCtrlIp; 
    for (const auto& n : m_neighbors) ss << " " << n;
    std::string data = ss.str();

    Ptr<Packet> pkt = Create<Packet>((uint8_t*)data.c_str(), data.size());
    
    // Create a temporary socket to send to controller
    Ptr<Socket> s = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    s->Bind(InetSocketAddress(myCtrlIp, 0)); 
    s->Connect(InetSocketAddress(m_controllerIp, 9999)); // Controller listens on 9999
    s->Send(pkt);
    s->Close();

    // Clear neighbors for next cycle (Soft State)
    m_neighbors.clear();
    
    // Schedule next report (1.0s interval)
    m_reportEvent = Simulator::Schedule(Seconds(1.0), &VehicleSdnApp::SendReport, this);
}

void VehicleSdnApp::ReceiveRoute(Ptr<Socket> socket)
{
    // Receive "Dst IP -> NextHop IP" instructions from Controller
    Address from;
    Ptr<Packet> pkt = socket->RecvFrom(from);
    if (pkt->GetSize() == 0) return;

    std::vector<uint8_t> buffer(pkt->GetSize() + 1, 0);
    pkt->CopyData(buffer.data(), pkt->GetSize());
    std::stringstream ss((char*)buffer.data());

    // Inject into Static Routing Table
    Ptr<Ipv4> ipv4 = GetNode()->GetObject<Ipv4>();
    Ptr<Ipv4RoutingProtocol> rp = ipv4->GetRoutingProtocol();
    Ptr<Ipv4ListRouting> lr = DynamicCast<Ipv4ListRouting>(rp);
    
    Ptr<Ipv4StaticRouting> staticRouting;
    int16_t priority;
    
    if (lr) {
        for (uint32_t i = 0; i < lr->GetNRoutingProtocols(); i++) {
            Ptr<Ipv4RoutingProtocol> temp = lr->GetRoutingProtocol(i, priority);
            if (DynamicCast<Ipv4StaticRouting>(temp)) {
                staticRouting = DynamicCast<Ipv4StaticRouting>(temp);
                break;
            }
        }
    }

    if (!staticRouting) return;

    std::string dstStr, nhStr;
    while (ss >> dstStr >> nhStr) {
        if (dstStr.find('.') == std::string::npos || nhStr.find('.') == std::string::npos) continue;
        
        Ipv4Address dst(dstStr.c_str());
        Ipv4Address nh(nhStr.c_str());
        
        // Add Route: To Dst via NextHop on Interface 1 (Data) with metric 0
        staticRouting->AddHostRouteTo(dst, nh, m_dataIfIndex, 0);
    }
}


// ============================================================================
//                            CONTROLLER APP
// ============================================================================

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
    
    // Helpers for Dijkstra
    std::map<uint32_t, uint32_t> ComputeNextHops(uint32_t srcId);

    Ptr<Socket> m_recvSocket; // Listens for reports
    EventId m_routeEvent;
    
    // Global Topology: NodeID -> Set of Neighbor NodeIDs
    std::map<uint32_t, std::set<uint32_t>> m_topology;
    uint32_t m_ctrlIfIndex;
};

NS_OBJECT_ENSURE_REGISTERED(SdnControllerApp);

TypeId SdnControllerApp::GetTypeId()
{
    static TypeId tid = TypeId("ns3::SdnControllerApp")
        .SetParent<Application>()
        .AddConstructor<SdnControllerApp>();
    return tid;
}

SdnControllerApp::SdnControllerApp() : m_ctrlIfIndex(1) {}

void SdnControllerApp::StartApplication()
{
    Ipv4Address myCtrlIp = GetNodeIpv4Address(GetNode(), m_ctrlIfIndex);
    m_recvSocket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    m_recvSocket->Bind(InetSocketAddress(myCtrlIp, 9999)); // Listen on 9999
    m_recvSocket->SetRecvCallback(MakeCallback(&SdnControllerApp::ReceiveReport, this));

    // Start Route Computation Loop
    m_routeEvent = Simulator::Schedule(Seconds(2.0), &SdnControllerApp::RecomputeRoutes, this);
}

void SdnControllerApp::StopApplication()
{
    Simulator::Cancel(m_routeEvent);
    if (m_recvSocket) m_recvSocket->Close();
}

void SdnControllerApp::ReceiveReport(Ptr<Socket> socket)
{
    // Parse "SrcCtrlIP Nbr1DataIP Nbr2DataIP..."
    Address from;
    Ptr<Packet> pkt = socket->RecvFrom(from);
    if (pkt->GetSize() == 0) return;

    std::vector<uint8_t> buffer(pkt->GetSize() + 1, 0);
    pkt->CopyData(buffer.data(), pkt->GetSize());
    std::stringstream ss((char*)buffer.data());

    std::string srcCtrlIpStr;
    ss >> srcCtrlIpStr;
    if (srcCtrlIpStr.empty()) return;

    // Identify Source Node
    Ipv4Address srcCtrlIp(srcCtrlIpStr.c_str());
    Ptr<Node> srcNode = FindNodeByCtrlIp(srcCtrlIp);
    if (!srcNode) return;
    
    uint32_t srcId = srcNode->GetId();
    m_topology[srcId].clear(); // Refresh topology for this node

    std::string nbrIpStr;
    while (ss >> nbrIpStr) {
        // Nbr IP reported is Data IP. We need to find the node ID.
        // Helper loop to find node by Data IP (usually 10.1.x.x)
        Ipv4Address nbrIp(nbrIpStr.c_str());
        Ptr<Node> nbrNode = nullptr;
        
        // Brute force find neighbor (optimized map would be better but this works for 50 nodes)
        if (g_allNodes) {
            for (uint32_t i=0; i<g_allNodes->GetN(); ++i) {
                 if (GetNodeIpv4Address(g_allNodes->Get(i), 1) == nbrIp) { // Interface 1 is Data
                     nbrNode = g_allNodes->Get(i);
                     break;
                 }
            }
        }
        
        if (nbrNode) {
            uint32_t nbrId = nbrNode->GetId();
            // Add Bi-directional link
            m_topology[srcId].insert(nbrId);
            m_topology[nbrId].insert(srcId);
        }
    }
}

// Simple BFS for unweighted shortest path (Hop Count)
std::map<uint32_t, uint32_t> SdnControllerApp::ComputeNextHops(uint32_t srcId)
{
    std::map<uint32_t, uint32_t> nextHop; // DestID -> NextHopID
    std::queue<uint32_t> q;
    std::set<uint32_t> visited;
    
    // Parent map to reconstruct path: Child -> Parent
    std::map<uint32_t, uint32_t> parent;

    q.push(srcId);
    visited.insert(srcId);
    parent[srcId] = srcId; // Root

    // BFS
    while(!q.empty()) {
        uint32_t u = q.front(); 
        q.pop();

        for(uint32_t v : m_topology[u]) {
            if(visited.find(v) == visited.end()) {
                visited.insert(v);
                parent[v] = u;
                q.push(v);
            }
        }
    }

    // Reconstruct NextHops for all reachable destinations
    for(uint32_t dst : visited) {
        if (dst == srcId) continue;
        
        // Backtrack from dst to src to find the immediate next hop
        uint32_t curr = dst;
        while(parent[curr] != srcId) {
            curr = parent[curr];
        }
        nextHop[dst] = curr;
    }
    return nextHop;
}

void SdnControllerApp::RecomputeRoutes()
{
    if (!g_allNodes) return;
    
    // For each Vehicle, calculate routing table and push it
    for (uint32_t i = 0; i < g_allNodes->GetN(); ++i) {
        Ptr<Node> node = g_allNodes->Get(i);
        uint32_t nodeId = node->GetId();
        
        // Skip Controller itself (Node 50)
        if (nodeId == 50) continue; 

        // Get Routes
        std::map<uint32_t, uint32_t> routes = ComputeNextHops(nodeId);
        
        if (routes.empty()) continue;

        // Build Message: "DstDataIP NextHopDataIP ..."
        std::ostringstream ss;
        for (auto const& [dstId, nhId] : routes) {
            // Only care if Dst is a Vehicle (0-49)
            if (dstId == 50) continue; 
            
            Ptr<Node> dstNode = g_allNodes->Get(dstId);
            Ptr<Node> nhNode = g_allNodes->Get(nhId);
            
            ss << GetNodeIpv4Address(dstNode, 1) << " "   // Dst Data IP
               << GetNodeIpv4Address(nhNode, 1) << " ";  // NextHop Data IP
        }

        std::string msg = ss.str();
        if (msg.empty()) continue;

        // Send to Vehicle's Control IP
        Ipv4Address vehicleCtrlIp = GetNodeIpv4Address(node, 2); // Interface 2 is Ctrl
        Ptr<Packet> p = Create<Packet>((uint8_t*)msg.c_str(), msg.size());
        
        // Send via socket
        Ptr<Socket> s = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        Ipv4Address myCtrlIp = GetNodeIpv4Address(GetNode(), m_ctrlIfIndex);
        s->Bind(InetSocketAddress(myCtrlIp, 0));
        s->SendTo(p, 0, InetSocketAddress(vehicleCtrlIp, 10000));
        s->Close();
    }
    
    // Schedule next computation
    m_routeEvent = Simulator::Schedule(Seconds(1.0), &SdnControllerApp::RecomputeRoutes, this);
}

// ============================================================================
//                                MAIN
// ============================================================================

int main (int argc, char *argv[])
{
  std::string protocol = "SDN";
  int speed = 10;
  int runId = 1;
  std::string traceFile = ""; 
  std::string outputDir = "."; 
  double simTime = 300.0;      
  uint32_t numVehicles = 50;
  bool enableNetAnim = false; 

  CommandLine cmd;
  cmd.AddValue ("speed", "Speed of nodes", speed);
  cmd.AddValue ("runId", "Run identifier", runId);
  cmd.AddValue ("traceFile", "Override path to trace file", traceFile);
  cmd.AddValue ("outputDir", "Directory to save results", outputDir);
  cmd.AddValue ("netanim", "Enable NetAnim", enableNetAnim);
  cmd.Parse (argc, argv);

  if (traceFile.empty ()) {
      traceFile = "scratch/mobility/mobility_" + std::to_string(speed) + ".tcl";
  }
  
  // Filenames
  std::stringstream ss;
  ss << outputDir << "/result_SDN_" << speed << "_" << runId << ".xml";
  std::string xmlFileName = ss.str();
  
  std::stringstream ssAnim;
  ssAnim << outputDir << "/netanim_SDN_" << speed << "_" << runId << ".xml";
  std::string animFileName = ssAnim.str();

  RngSeedManager::SetSeed (3 + runId); 
  RngSeedManager::SetRun (runId);

  // --- 1. Nodes ---
  // 50 Vehicles + 1 Controller
  NodeContainer vehicles;
  vehicles.Create(numVehicles);
  
  NodeContainer controller;
  controller.Create(1);
  
  NodeContainer allNodes = NodeContainer(vehicles, controller);
  g_allNodes = &allNodes; // Set global pointer

  // --- 2. Mobility ---
  // Vehicles: SUMO Trace
  Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);
  ns2.Install (vehicles.Begin(), vehicles.End());

  // Controller: Static at (500, 500)
  MobilityHelper mobilityCtrl;
  mobilityCtrl.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobilityCtrl.Install(controller);
  controller.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(500, 500, 0));

  // --- 3. WiFi (DATA Plane - Short Range) ---
  WifiHelper wifiData;
  wifiData.SetStandard (WIFI_STANDARD_80211a);
  
  YansWifiPhyHelper phyData;
  YansWifiChannelHelper chanData;
  chanData.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  chanData.AddPropagationLoss ("ns3::FriisPropagationLossModel");
  // Limit range strictly to ~250m for Data
  chanData.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(250.0));
  phyData.SetChannel (chanData.Create ());
  
  WifiMacHelper macData;
  macData.SetType ("ns3::AdhocWifiMac");
  
  // Only Vehicles have Data Interface
  NetDeviceContainer devicesData = wifiData.Install(phyData, macData, vehicles);

  // --- 4. WiFi (CONTROL Plane - Long Range "LTE") ---
  WifiHelper wifiCtrl;
  wifiCtrl.SetStandard (WIFI_STANDARD_80211a);
  
  YansWifiPhyHelper phyCtrl;
  YansWifiChannelHelper chanCtrl;
  chanCtrl.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  // Huge range to simulate LTE coverage over the whole map
  chanCtrl.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(2000.0));
  phyCtrl.SetChannel (chanCtrl.Create ());
  
  WifiMacHelper macCtrl;
  macCtrl.SetType ("ns3::AdhocWifiMac");
  
  // Vehicles AND Controller have Control Interface
  NetDeviceContainer devicesCtrlVehicles = wifiCtrl.Install(phyCtrl, macCtrl, vehicles);
  NetDeviceContainer devicesCtrlController = wifiCtrl.Install(phyCtrl, macCtrl, controller);

  // --- 5. Internet Stack ---
  InternetStackHelper stack;
  Ipv4ListRoutingHelper list;
  Ipv4StaticRoutingHelper staticRouting;
  
  // High priority Static Routing (SDN), Fallback not needed for this exp
  list.Add(staticRouting, 100); 
  
  stack.SetRoutingHelper(list);
  stack.Install(allNodes);

  // --- 6. IP Assignment ---
  // Data Subnet: 10.1.0.0/16
  Ipv4AddressHelper ipv4Data;
  ipv4Data.SetBase ("10.1.0.0", "255.255.0.0");
  Ipv4InterfaceContainer ifData = ipv4Data.Assign(devicesData);

  // Control Subnet: 10.2.0.0/16
  Ipv4AddressHelper ipv4Ctrl;
  ipv4Ctrl.SetBase ("10.2.0.0", "255.255.0.0");
  Ipv4InterfaceContainer ifCtrlVeh = ipv4Ctrl.Assign(devicesCtrlVehicles);
  Ipv4InterfaceContainer ifCtrlNode = ipv4Ctrl.Assign(devicesCtrlController);

  // --- 7. Install Applications ---
  
  // Controller App
  Ptr<SdnControllerApp> ctrlApp = CreateObject<SdnControllerApp>();
  ctrlApp->SetCtrlIfIndex(1); // Controller only has 1 interface (index 1, loopback is 0)
  controller.Get(0)->AddApplication(ctrlApp);
  ctrlApp->SetStartTime(Seconds(0.1));
  ctrlApp->SetStopTime(Seconds(simTime));
  
  // Vehicle Apps
  Ipv4Address controllerIp = ifCtrlNode.GetAddress(0);
  for (uint32_t i = 0; i < vehicles.GetN(); ++i) {
      Ptr<VehicleSdnApp> app = CreateObject<VehicleSdnApp>();
      // Interface 1 = Data, Interface 2 = Control (0 is loopback)
      app->Setup(controllerIp, 1, 2); 
      vehicles.Get(i)->AddApplication(app);
      app->SetStartTime(Seconds(0.5 + (i*0.01))); // Stagger start
      app->SetStopTime(Seconds(simTime));
  }

  // --- 8. Traffic (UDP Echo) ---
  uint16_t port = 9;
  
  // Server on Vehicle 0
  UdpEchoServerHelper server (port);
  ApplicationContainer serverApps = server.Install (vehicles.Get (0));
  serverApps.Start (Seconds (5.0)); // Wait for routing to stabilize
  serverApps.Stop (Seconds (simTime));

  // Client on Vehicle 49
  UdpEchoClientHelper client (ifData.GetAddress (0), port);
  client.SetAttribute ("MaxPackets", UintegerValue (100000));
  client.SetAttribute ("Interval", TimeValue (Seconds (0.25))); 
  client.SetAttribute ("PacketSize", UintegerValue (1024));

  ApplicationContainer clientApps = client.Install (vehicles.Get (numVehicles - 1));
  clientApps.Start (Seconds (6.0));
  clientApps.Stop (Seconds (simTime));

  // --- 9. Simulation & Output ---
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll ();
  
  AnimationInterface *anim = 0;
  if (enableNetAnim) {
      anim = new AnimationInterface(animFileName);
      anim->SetMaxPktsPerTraceFile(999999999999); 
      anim->EnablePacketMetadata(true);
      
      // Color Vehicles Blue
      for(uint32_t i=0; i<vehicles.GetN(); ++i) {
          anim->UpdateNodeColor(vehicles.Get(i), 0, 0, 255);
      }
      // Color Controller Red
      anim->UpdateNodeColor(controller.Get(0), 255, 0, 0);
      anim->UpdateNodeDescription(controller.Get(0), "SDN Controller");
      anim->UpdateNodeSize(controller.Get(0)->GetId(), 10.0, 10.0);
  }

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();

  monitor->CheckForLostPackets ();
  monitor->SerializeToXmlFile (xmlFileName, true, true);

  if (anim) delete anim;
  Simulator::Destroy ();
  return 0;
}
