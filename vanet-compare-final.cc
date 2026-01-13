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
#include "ns3/aodv-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/olsr-module.h"

#include <vector>
#include <map>
#include <set>
#include <queue>
#include <string>
#include <sstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("VanetCompareFinal");

NodeContainer* g_sdnNodes = nullptr;

// ============================================================================
//                          SDN HELPER FUNCTIONS
// ============================================================================

Ipv4Address GetNodeIpv4Address(Ptr<Node> node, uint32_t ifIndex) {
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    if (!ipv4 || ifIndex >= ipv4->GetNInterfaces()) return Ipv4Address::GetZero();
    return ipv4->GetAddress(ifIndex, 0).GetLocal();
}

Ptr<Node> FindNodeByCtrlIp(Ipv4Address ctrlIp) {
  if (!g_sdnNodes) return nullptr;
  for (uint32_t i = 0; i < g_sdnNodes->GetN(); ++i) {
      Ptr<Ipv4> ipv4 = g_sdnNodes->Get(i)->GetObject<Ipv4>();
      for (uint32_t j = 0; j < ipv4->GetNInterfaces(); ++j) {
          if (ipv4->GetAddress(j, 0).GetLocal() == ctrlIp) return g_sdnNodes->Get(i);
      }
  }
  return nullptr;
}

// ============================================================================
//                              SDN VEHICLE APP
// ============================================================================

class VehicleSdnApp : public Application {
public:
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("ns3::VehicleSdnApp").SetParent<Application>().AddConstructor<VehicleSdnApp>();
        return tid;
    }
    VehicleSdnApp() : m_dataIfIndex(1), m_ctrlIfIndex(2) {}
    void Setup(Ipv4Address controllerIp, uint32_t dataIfIndex, uint32_t ctrlIfIndex) {
        m_controllerIp = controllerIp; m_dataIfIndex = dataIfIndex; m_ctrlIfIndex = ctrlIfIndex;
    }
private:
    virtual void StartApplication() {
        // Beacon TX on control interface (persistent socket, no churn)
        m_beaconTx = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_beaconTx->SetAllowBroadcast(true);
        m_beaconTx->Bind(InetSocketAddress(GetNodeIpv4Address(GetNode(), m_ctrlIfIndex), 0));

        // Beacon RX on control interface (single persistent socket)
        m_beaconRx = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_beaconRx->Bind(InetSocketAddress(Ipv4Address::GetAny(), 8888));
        m_beaconRx->SetRecvCallback(MakeCallback(&VehicleSdnApp::ReceiveBeacon, this));

        // Control RX (Route Updates) - persistent
        m_ctrlRx = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_ctrlRx->Bind(InetSocketAddress(GetNodeIpv4Address(GetNode(), m_ctrlIfIndex), 10000));
        m_ctrlRx->SetRecvCallback(MakeCallback(&VehicleSdnApp::ReceiveRoute, this));

        double jitter = (double)(GetNode()->GetId()) * 0.002; 
        // Aggressive timing: fast discovery to match 15m/s mobility
        m_beaconEvent = Simulator::Schedule(Seconds(0.2 + jitter), &VehicleSdnApp::SendBeacon, this);
        m_reportEvent = Simulator::Schedule(Seconds(0.25 + jitter), &VehicleSdnApp::SendReport, this);
    }

    virtual void StopApplication() {
        Simulator::Cancel(m_beaconEvent); Simulator::Cancel(m_reportEvent);
        if(m_beaconTx) m_beaconTx->Close(); if(m_beaconRx) m_beaconRx->Close(); if(m_ctrlRx) m_ctrlRx->Close();
    }

    void SendBeacon() {
        // Reuse persistent control-intf beacon socket to avoid churn
        std::ostringstream oss; oss << GetNodeIpv4Address(GetNode(), m_ctrlIfIndex);
        std::string data = oss.str();
        Ptr<Packet> p = Create<Packet>((uint8_t*)data.c_str(), data.size());
        m_beaconTx->SendTo(p, 0, InetSocketAddress(Ipv4Address("255.255.255.255"), 8888));
        m_beaconEvent = Simulator::Schedule(Seconds(0.2), &VehicleSdnApp::SendBeacon, this);
    }

    void ReceiveBeacon(Ptr<Socket> socket) {
        Address from; Ptr<Packet> pkt = socket->RecvFrom(from);
        if (pkt->GetSize() == 0) return;
        std::vector<uint8_t> buffer(pkt->GetSize() + 1, 0); pkt->CopyData(buffer.data(), pkt->GetSize());
        std::string senderIpStr((char*)buffer.data());
        if (senderIpStr.find(".") == std::string::npos) return;
        Ipv4Address neighborCtrlIp(senderIpStr.c_str());
        // Store Neighbor Control IP
        if (neighborCtrlIp != GetNodeIpv4Address(GetNode(), m_ctrlIfIndex) && neighborCtrlIp != Ipv4Address::GetZero()) 
            m_neighbors.insert(neighborCtrlIp);
    }

    void SendReport() {
        // Msg: "MyCtrlIP Nbr1CtrlIP Nbr2CtrlIP..."
        std::ostringstream ss; ss << GetNodeIpv4Address(GetNode(), m_ctrlIfIndex); 
        for (const auto& n : m_neighbors) ss << " " << n;
        std::string data = ss.str();
        Ptr<Packet> pkt = Create<Packet>((uint8_t*)data.c_str(), data.size());
        Ptr<Socket> s = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        s->Bind(InetSocketAddress(GetNodeIpv4Address(GetNode(), m_ctrlIfIndex), 0)); 
        s->Connect(InetSocketAddress(m_controllerIp, 9999));
        s->Send(pkt); s->Close();
        m_neighbors.clear();
        m_reportEvent = Simulator::Schedule(Seconds(0.25), &VehicleSdnApp::SendReport, this);
    }

    // Purge old routes to fix the "Append" bug
    void PurgeSDNRoutes(Ptr<Ipv4StaticRouting> staticRouting) {
        for (uint32_t i = staticRouting->GetNRoutes(); i > 0; i--) {
            Ipv4RoutingTableEntry route = staticRouting->GetRoute(i-1);
            if (route.GetDestNetworkMask() == Ipv4Mask("255.255.255.255")) {
                if (route.GetDest().IsBroadcast()) continue; 
                if (route.GetDest().CombineMask(Ipv4Mask("255.255.0.0")) == Ipv4Address("10.1.0.0")) {
                    staticRouting->RemoveRoute(i-1);
                }
            }
        }
    }

    void ReceiveRoute(Ptr<Socket> socket) {
        Address from; Ptr<Packet> pkt = socket->RecvFrom(from);
        if (pkt->GetSize() == 0) return;
        std::vector<uint8_t> buffer(pkt->GetSize() + 1, 0); pkt->CopyData(buffer.data(), pkt->GetSize());
        std::stringstream ss((char*)buffer.data());
        
        Ptr<Ipv4StaticRouting> staticRouting;
        Ptr<Ipv4> ipv4 = GetNode()->GetObject<Ipv4>();
        Ptr<Ipv4ListRouting> lr = DynamicCast<Ipv4ListRouting>(ipv4->GetRoutingProtocol());
        if (lr) {
            int16_t priority;
            for (uint32_t i = 0; i < lr->GetNRoutingProtocols(); i++) {
                Ptr<Ipv4RoutingProtocol> temp = lr->GetRoutingProtocol(i, priority);
                if (DynamicCast<Ipv4StaticRouting>(temp)) { staticRouting = DynamicCast<Ipv4StaticRouting>(temp); break; }
            }
        }
        if (!staticRouting) return;

        // Incremental update: only remove/add if next-hop actually changed
        std::map<Ipv4Address, Ipv4Address> newRoutes; // dst -> nextHop
        std::string dstStr, nhStr;
        while (ss >> dstStr >> nhStr) {
            if (dstStr.find('.') == std::string::npos) continue;
            newRoutes[Ipv4Address(dstStr.c_str())] = Ipv4Address(nhStr.c_str());
        }

        // Remove routes that disappeared from the controller update
        for (auto it = m_cachedRoutes.begin(); it != m_cachedRoutes.end(); ) {
            if (newRoutes.find(it->first) == newRoutes.end()) {
                Ipv4Address dst = it->first;
                for (uint32_t i = staticRouting->GetNRoutes(); i > 0; i--) {
                    Ipv4RoutingTableEntry route = staticRouting->GetRoute(i-1);
                    if (route.GetDest() == dst &&
                        route.GetDestNetworkMask() == Ipv4Mask("255.255.255.255") &&
                        route.GetDest().CombineMask(Ipv4Mask("255.255.0.0")) == Ipv4Address("10.1.0.0")) {
                        staticRouting->RemoveRoute(i-1);
                        break;
                    }
                }
                it = m_cachedRoutes.erase(it);
            } else {
                ++it;
            }
        }

        // Compare with cached routes and only update if changed
        for (auto& [dst, nh] : newRoutes) {
            auto it = m_cachedRoutes.find(dst);
            if (it == m_cachedRoutes.end() || it->second != nh) {
                // Verify next-hop is still in range before installing route
                Ptr<Node> myNode = GetNode();
                Ptr<Node> nhNode = nullptr;
                
                // Check cache first for O(1) lookup
                auto cacheIt = m_ipToNodeCache.find(nh);
                if (cacheIt != m_ipToNodeCache.end()) {
                    nhNode = cacheIt->second;
                } else if (g_sdnNodes) {
                    // Build cache on-demand (first lookup only)
                    for (uint32_t i = 0; i < g_sdnNodes->GetN(); ++i) {
                        Ipv4Address dataIp = GetNodeIpv4Address(g_sdnNodes->Get(i), 1);
                        if (dataIp != Ipv4Address::GetZero()) {
                            m_ipToNodeCache[dataIp] = g_sdnNodes->Get(i);
                        }
                    }
                    cacheIt = m_ipToNodeCache.find(nh);
                    if (cacheIt != m_ipToNodeCache.end()) {
                        nhNode = cacheIt->second;
                    }
                }
                
                // Distance check: skip if next-hop is now out of range (230m strict threshold)
                if (nhNode) {
                    Ptr<MobilityModel> myMob = myNode->GetObject<MobilityModel>();
                    Ptr<MobilityModel> nhMob = nhNode->GetObject<MobilityModel>();
                    double dist = myMob->GetDistanceFrom(nhMob);
                    if (dist > 230.0) {
                        // Next-hop too far; skip this route to avoid blackhole
                        continue;
                    }
                }
                
                // Route is new or next-hop changed; remove old and add new
                for (uint32_t i = staticRouting->GetNRoutes(); i > 0; i--) {
                    Ipv4RoutingTableEntry route = staticRouting->GetRoute(i-1);
                    if (route.GetDest() == dst && 
                        route.GetDestNetworkMask() == Ipv4Mask("255.255.255.255") &&
                        route.GetDest().CombineMask(Ipv4Mask("255.255.0.0")) == Ipv4Address("10.1.0.0")) {
                        staticRouting->RemoveRoute(i-1);
                        break;
                    }
                }
                staticRouting->AddHostRouteTo(dst, nh, m_dataIfIndex, 0);
                m_cachedRoutes[dst] = nh;
            }
        }
    }

    Ptr<Socket> m_beaconTx, m_beaconRx, m_ctrlRx;
    Ipv4Address m_controllerIp;
    uint32_t m_dataIfIndex, m_ctrlIfIndex;
    std::set<Ipv4Address> m_neighbors;
    std::map<Ipv4Address, Ipv4Address> m_cachedRoutes; // dst -> nextHop cache
    std::map<Ipv4Address, Ptr<Node>> m_ipToNodeCache; // IP -> Node lookup cache
    EventId m_beaconEvent, m_reportEvent;
};

// --- CONTROLLER APP ---
class SdnControllerApp : public Application {
public:
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("ns3::SdnControllerApp").SetParent<Application>().AddConstructor<SdnControllerApp>();
        return tid;
    }
    SdnControllerApp() : m_ctrlIfIndex(1) {}
    void SetCtrlIfIndex(uint32_t idx) { m_ctrlIfIndex = idx; }

private:
    virtual void StartApplication() {
        m_recvSocket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_recvSocket->Bind(InetSocketAddress(GetNodeIpv4Address(GetNode(), m_ctrlIfIndex), 9999));
        m_recvSocket->SetRecvCallback(MakeCallback(&SdnControllerApp::ReceiveReport, this));
        m_routeEvent = Simulator::Schedule(Seconds(0.2), &SdnControllerApp::RecomputeRoutes, this); // Start early for initial topology discovery
    }
    virtual void StopApplication() { Simulator::Cancel(m_routeEvent); if(m_recvSocket) m_recvSocket->Close(); }

    void ReceiveReport(Ptr<Socket> socket) {
        Address from; Ptr<Packet> pkt = socket->RecvFrom(from);
        if (pkt->GetSize() == 0) return;
        std::vector<uint8_t> buffer(pkt->GetSize() + 1, 0); pkt->CopyData(buffer.data(), pkt->GetSize());
        std::stringstream ss((char*)buffer.data());
        
        std::string srcCtrlIpStr, nbrCtrlIpStr;
        ss >> srcCtrlIpStr; 
        if (srcCtrlIpStr.empty()) return;
        
        Ptr<Node> srcNode = FindNodeByCtrlIp(Ipv4Address(srcCtrlIpStr.c_str()));
        if (!srcNode) return;
        
        uint32_t srcId = srcNode->GetId();
        m_topology[srcId].clear();
        
        // --- CRITICAL FIX FROM SUS CODE: DISTANCE CHECK ---
        Ptr<MobilityModel> srcMob = srcNode->GetObject<MobilityModel>();

        while (ss >> nbrCtrlIpStr) {
            Ipv4Address nbrIp(nbrCtrlIpStr.c_str());
            Ptr<Node> nbrNode = FindNodeByCtrlIp(nbrIp);
            if (nbrNode) {
                // Verify Distance (The "God Mode" sanity check)
                Ptr<MobilityModel> nbrMob = nbrNode->GetObject<MobilityModel>();
                double dist = srcMob->GetDistanceFrom(nbrMob);
                
                // If within safe range (210m strict), accept the link
                if (dist <= 210.0) {
                    m_topology[srcId].insert(nbrNode->GetId());
                    m_topology[nbrNode->GetId()].insert(srcId);
                }
            }
        }
    }

    void RecomputeRoutes() {
        if (!g_sdnNodes) return;
        
        // Only recompute if topology *actually* changed (not just order)
        std::map<uint32_t, std::set<uint32_t>> sortedTopo = m_topology;
        if (m_lastTopology == sortedTopo) {
            m_routeEvent = Simulator::Schedule(Seconds(0.25), &SdnControllerApp::RecomputeRoutes, this);
            return;
        }
        m_lastTopology = sortedTopo;
        
        for (uint32_t i = 0; i < g_sdnNodes->GetN(); ++i) {
            Ptr<Node> node = g_sdnNodes->Get(i);
            uint32_t nodeId = node->GetId();
            if (nodeId == 50) continue; 

            // BFS Dijkstra
            std::map<uint32_t, uint32_t> nextHop, parent;
            std::queue<uint32_t> q; std::set<uint32_t> visited;
            q.push(nodeId); visited.insert(nodeId); parent[nodeId] = nodeId;
            
            while(!q.empty()) {
                uint32_t u = q.front(); q.pop();
                for(uint32_t v : m_topology[u]) {
                    if(visited.find(v) == visited.end()) {
                        visited.insert(v); parent[v] = u; q.push(v);
                    }
                }
            }
            
            for(uint32_t dst : visited) {
                if (dst == nodeId) continue;
                uint32_t curr = dst;
                while(parent[curr] != nodeId) curr = parent[curr];
                nextHop[dst] = curr;
            }

            if (nextHop.empty()) continue;

            std::ostringstream ss;
            for (auto const& [dstId, nhId] : nextHop) {
                if (dstId == 50) continue;
                // Convert Node IDs to DATA IPs for the route table
                ss << GetNodeIpv4Address(g_sdnNodes->Get(dstId), 1) << " " << GetNodeIpv4Address(g_sdnNodes->Get(nhId), 1) << " ";
            }
            std::string msg = ss.str();
            if (msg.empty()) continue;

            Simulator::Schedule(Seconds(i * 0.0005), &SdnControllerApp::SendRoutePacket, this, node, msg);
        }
        m_routeEvent = Simulator::Schedule(Seconds(0.25), &SdnControllerApp::RecomputeRoutes, this);
    }

    // New Helper Function to actually send the packet
    void SendRoutePacket(Ptr<Node> targetNode, std::string msg) {
        Ptr<Packet> p = Create<Packet>((uint8_t*)msg.c_str(), msg.size());
        Ptr<Socket> s = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        s->Bind(InetSocketAddress(GetNodeIpv4Address(GetNode(), m_ctrlIfIndex), 0));
        s->SendTo(p, 0, InetSocketAddress(GetNodeIpv4Address(targetNode, 2), 10000));
        s->Close();
    }

    Ptr<Socket> m_recvSocket;
    EventId m_routeEvent;
    std::map<uint32_t, std::set<uint32_t>> m_topology, m_lastTopology;
    uint32_t m_ctrlIfIndex;
};


// ============================================================================
//                                MAIN
// ============================================================================

int main (int argc, char *argv[])
{
  std::string protocol = "AODV"; 
  int speed = 10;
  int runId = 1;
  std::string traceFile = ""; 
  std::string outputDir = "."; 
  double simTime = 60.0;      // Change to 60 for quick res
  uint32_t numNodes = 50; 
  bool enableNetAnim = false; 

  CommandLine cmd;
  cmd.AddValue ("protocol", "Protocol (AODV, DSDV, OLSR, SDN)", protocol);
  cmd.AddValue ("speed", "Speed", speed);
  cmd.AddValue ("runId", "Run ID", runId);
  cmd.AddValue ("traceFile", "Trace file", traceFile);
  cmd.AddValue ("outputDir", "Output Directory", outputDir);
  cmd.AddValue ("netanim", "Enable NetAnim", enableNetAnim);
  cmd.Parse (argc, argv);

  if (traceFile.empty ()) {
      traceFile = "scratch/mobility/mobility_" + std::to_string(speed) + ".tcl";
  }
  
  std::stringstream ss;
  ss << outputDir << "/result_" << protocol << "_" << speed << "_" << runId << ".xml";
  std::string xmlFileName = ss.str();
  
  std::stringstream ssAnim;
  ssAnim << outputDir << "/netanim_" << protocol << "_" << speed << "_" << runId << ".xml";
  std::string animFileName = ssAnim.str();

  RngSeedManager::SetSeed (3 + runId); 
  RngSeedManager::SetRun (runId);

  // --------------------------------------------------------------------------
  //                        TRADITIONAL MODE
  // --------------------------------------------------------------------------
  if (protocol != "SDN") 
  {
      // ... (Same Traditional Mode Logic as before) ...
      NS_LOG_UNCOND("Running Traditional Mode: " << protocol);
      
      // Configure RTS/CTS BEFORE creating WiFi
      Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", UintegerValue(0));
      
      NodeContainer nodes; nodes.Create (numNodes);
      Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);
      ns2.Install (nodes.Begin(), nodes.End());

      WifiHelper wifi; wifi.SetStandard (WIFI_STANDARD_80211p);
      wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                             "DataMode", StringValue("OfdmRate12MbpsBW10MHz"),
                             "ControlMode", StringValue("OfdmRate6MbpsBW10MHz"));
      YansWifiPhyHelper wifiPhy; YansWifiChannelHelper wifiChannel;
      wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
      wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
      wifiChannel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(350.0));
      wifiPhy.SetChannel (wifiChannel.Create ());
      wifiPhy.Set ("TxPowerStart", DoubleValue(20.0));
      wifiPhy.Set ("TxPowerEnd", DoubleValue(20.0));
      wifiPhy.Set("ChannelWidth", UintegerValue(10));
      WifiMacHelper wifiMac; wifiMac.SetType ("ns3::AdhocWifiMac");
      NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, nodes);

      Ipv4ListRoutingHelper list; InternetStackHelper stack;
      if (protocol == "AODV") { AodvHelper aodv; list.Add (aodv, 100); }
      else if (protocol == "DSDV") { DsdvHelper dsdv; list.Add (dsdv, 100); }
      else if (protocol == "OLSR") { OlsrHelper olsr; list.Add (olsr, 100); }
      stack.SetRoutingHelper (list);
      stack.Install (nodes);

      Ipv4AddressHelper address; address.SetBase ("10.1.1.0", "255.255.255.0");
      Ipv4InterfaceContainer interfaces = address.Assign (devices);

      uint16_t port = 9;
      UdpEchoServerHelper server (port);
      ApplicationContainer serverApps = server.Install (nodes.Get (0));
      serverApps.Start (Seconds (5.0)); serverApps.Stop (Seconds (simTime));
      UdpEchoClientHelper client (interfaces.GetAddress (0), port);
      client.SetAttribute ("MaxPackets", UintegerValue (100000));
      client.SetAttribute ("Interval", TimeValue (Seconds (0.1))); 
      client.SetAttribute ("PacketSize", UintegerValue (1024));
      ApplicationContainer clientApps = client.Install (nodes.Get (numNodes - 1));
      clientApps.Start (Seconds (10.0)); clientApps.Stop (Seconds (simTime));

      FlowMonitorHelper flowmon; Ptr<FlowMonitor> monitor = flowmon.InstallAll ();
      Simulator::Stop (Seconds (simTime)); Simulator::Run ();
      monitor->CheckForLostPackets (); monitor->SerializeToXmlFile (xmlFileName, true, true);
      Simulator::Destroy ();
  }
  // --------------------------------------------------------------------------
  //                              SDN MODE (HYBRID)
  // --------------------------------------------------------------------------
  else 
  {
      NS_LOG_UNCOND("Running SDN Mode (Hybrid: Reporting + Distance Check)");

      NodeContainer vehicles; vehicles.Create(numNodes);
      NodeContainer controller; controller.Create(1);
      NodeContainer allNodes = NodeContainer(vehicles, controller);
      g_sdnNodes = &allNodes;

      Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);
      ns2.Install (vehicles.Begin(), vehicles.End());

      MobilityHelper mobilityCtrl;
      mobilityCtrl.SetMobilityModel("ns3::ConstantPositionMobilityModel");
      mobilityCtrl.Install(controller);
      controller.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(500, 500, 0));

    WifiHelper wifiData; wifiData.SetStandard (WIFI_STANDARD_80211p);
    // Higher rate for better throughput: 12 Mbps on 10 MHz channel
    wifiData.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                           "DataMode", StringValue("OfdmRate12MbpsBW10MHz"),
                           "ControlMode", StringValue("OfdmRate6MbpsBW10MHz"));
    Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", UintegerValue(0));
    YansWifiPhyHelper phyData; YansWifiChannelHelper chanData;
    chanData.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
    chanData.AddPropagationLoss ("ns3::FriisPropagationLossModel");
    chanData.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(350.0));
    phyData.SetChannel (chanData.Create ());
    phyData.Set("TxPowerStart", DoubleValue(20.0));
    phyData.Set("TxPowerEnd", DoubleValue(20.0));
    phyData.Set("ChannelWidth", UintegerValue(10));
    WifiMacHelper macData; macData.SetType ("ns3::AdhocWifiMac");
    NetDeviceContainer devicesData = wifiData.Install(phyData, macData, vehicles);

      WifiHelper wifiCtrl; wifiCtrl.SetStandard (WIFI_STANDARD_80211a);
      YansWifiPhyHelper phyCtrl; YansWifiChannelHelper chanCtrl;
      chanCtrl.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
      chanCtrl.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(2000.0));
      phyCtrl.SetChannel (chanCtrl.Create ());
      WifiMacHelper macCtrl; macCtrl.SetType ("ns3::AdhocWifiMac");
      NetDeviceContainer devCtrlVeh = wifiCtrl.Install(phyCtrl, macCtrl, vehicles);
      NetDeviceContainer devCtrlNode = wifiCtrl.Install(phyCtrl, macCtrl, controller);

      InternetStackHelper stack;
      Ipv4ListRoutingHelper list;
      Ipv4StaticRoutingHelper staticRouting;
      list.Add(staticRouting, 100); 
      stack.SetRoutingHelper(list);
      stack.Install(allNodes);

      // ENABLE IP FORWARDING
      for (uint32_t i = 0; i < vehicles.GetN(); ++i) {
          vehicles.Get(i)->GetObject<Ipv4>()->SetAttribute("IpForward", BooleanValue(true));
      }

      Ipv4AddressHelper ipv4Data; ipv4Data.SetBase ("10.1.0.0", "255.255.0.0");
      Ipv4InterfaceContainer ifData = ipv4Data.Assign(devicesData);

      Ipv4AddressHelper ipv4Ctrl; ipv4Ctrl.SetBase ("10.2.0.0", "255.255.0.0");
      Ipv4InterfaceContainer ifCtrlVeh = ipv4Ctrl.Assign(devCtrlVeh);
      Ipv4InterfaceContainer ifCtrlNode = ipv4Ctrl.Assign(devCtrlNode);

      Ptr<SdnControllerApp> ctrlApp = CreateObject<SdnControllerApp>();
      ctrlApp->SetCtrlIfIndex(1); 
      controller.Get(0)->AddApplication(ctrlApp);
      ctrlApp->SetStartTime(Seconds(0.1)); ctrlApp->SetStopTime(Seconds(simTime));
      
      Ipv4Address controllerIp = ifCtrlNode.GetAddress(0);
      for (uint32_t i = 0; i < vehicles.GetN(); ++i) {
          Ptr<VehicleSdnApp> app = CreateObject<VehicleSdnApp>();
          app->Setup(controllerIp, 1, 2); 
          vehicles.Get(i)->AddApplication(app);
          app->SetStartTime(Seconds(0.5 + (i*0.01))); app->SetStopTime(Seconds(simTime));
      }

      uint16_t port = 9;
      UdpEchoServerHelper server (port);
      ApplicationContainer serverApps = server.Install (vehicles.Get (0));
      serverApps.Start (Seconds (5.0)); serverApps.Stop (Seconds (simTime));

      UdpEchoClientHelper client (ifData.GetAddress (0), port);
      client.SetAttribute ("MaxPackets", UintegerValue (100000));
      client.SetAttribute ("Interval", TimeValue (Seconds (0.1))); 
      client.SetAttribute ("PacketSize", UintegerValue (1024));
      ApplicationContainer clientApps = client.Install (vehicles.Get (numNodes - 1));
      clientApps.Start (Seconds (10.0)); clientApps.Stop (Seconds (simTime));

      FlowMonitorHelper flowmon; Ptr<FlowMonitor> monitor = flowmon.InstallAll ();
      
      if (enableNetAnim) {
          AnimationInterface anim(animFileName);
          anim.SetMaxPktsPerTraceFile(999999999999); 
          anim.EnablePacketMetadata(false); 
          for(uint32_t i=0; i<vehicles.GetN(); ++i) anim.UpdateNodeColor(vehicles.Get(i), 0, 0, 255);
          anim.UpdateNodeColor(controller.Get(0), 255, 0, 0);
      }

      Simulator::Stop (Seconds (simTime)); Simulator::Run ();
      monitor->CheckForLostPackets (); monitor->SerializeToXmlFile (xmlFileName, true, true);
      Simulator::Destroy ();
  }

  return 0;
}