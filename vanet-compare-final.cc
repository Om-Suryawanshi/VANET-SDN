/*
 * Master Simulation Script for Figure 10 Replication
 * Protocol Support: SDN (Hybrid), AODV, OLSR, DSDV
 * Features: 
 * - SUMO Mobility Traces
 * - Hybrid SDN Application (Magic LTE Control Plane)
 * - Static ARP Injection (Fixes Layer 2 Drops)
 * - Smart Pair Selection (Ensures physical connectivity)
 */

 #include "ns3/core-module.h"
 #include "ns3/network-module.h"
 #include "ns3/mobility-module.h"
 #include "ns3/wifi-module.h"
 #include "ns3/internet-module.h"
 #include "ns3/applications-module.h"
 #include "ns3/ipv4-static-routing.h"
 #include "ns3/ipv4-list-routing.h"
 #include "ns3/flow-monitor-module.h"
 #include "ns3/aodv-module.h"
 #include "ns3/olsr-module.h"
 #include "ns3/dsdv-module.h"
 #include "ns3/arp-cache.h"
 #include "ns3/ipv4-l3-protocol.h"
 
 #include <queue>
 #include <sstream>
 #include <set>
 #include <map>
 
 using namespace ns3;
 
 NS_LOG_COMPONENT_DEFINE("VanetCompareFinal");
 
 namespace ns3 {
 
 // --- HELPER: Get IP Address ---
 Ipv4Address GetNodeIpv4Address(Ptr<Node> node) {
     Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
     for (uint32_t i = 0; i < ipv4->GetNInterfaces(); ++i) {
         for (uint32_t j = 0; j < ipv4->GetNAddresses(i); ++j) {
             Ipv4Address addr = ipv4->GetAddress(i, j).GetLocal();
             if (addr != Ipv4Address::GetLoopback() && addr != Ipv4Address::GetZero()) {
                 return addr;
             }
         }
     }
     return Ipv4Address::GetZero();
 }
 
 // Forward Declaration
 class VehicleSdnApp;
 
 // --- CONTROLLER APP ---
 class SdnControllerApp : public Application {
 public:
     static TypeId GetTypeId();
     SdnControllerApp();
     
     // "Magic" LTE Interface (Direct calls from Vehicles)
     void ReceiveReport(Ipv4Address nodeIp, std::set<Ipv4Address> neighbors);
     void RegisterVehicle(Ipv4Address ip, Ptr<VehicleSdnApp> app);
 
 private:
     virtual void StartApplication();
     virtual void StopApplication();
     void RecomputeRoutes();
 
     std::map<Ipv4Address, std::set<Ipv4Address>> m_topology;
     std::map<Ipv4Address, Ptr<VehicleSdnApp>> m_vehicleMap; // IP -> App Pointer
 };
 
 // --- VEHICLE APP ---
 class VehicleSdnApp : public Application {
 public:
     static TypeId GetTypeId();
     VehicleSdnApp();
     void SetController(Ptr<SdnControllerApp> controller);
     
     // "Magic" LTE Interface (Direct calls from Controller)
     void InstallRoute(Ipv4Address dst, Ipv4Address nextHop);
 
 private:
     virtual void StartApplication();
     virtual void StopApplication();
     void SendBeacon();
     void ReceiveBeacon(Ptr<Socket> socket);
     void SendReport(); // Calls Controller->ReceiveReport
 
     Ptr<Socket> m_beaconTx;
     Ptr<Socket> m_beaconRx;
     Ptr<SdnControllerApp> m_controller;
     std::set<Ipv4Address> m_neighbors;
 };
 
 // ============================================================================
 //  IMPLEMENTATION
 // ============================================================================
 
 // --- CONTROLLER ---
 NS_OBJECT_ENSURE_REGISTERED(SdnControllerApp);
 TypeId SdnControllerApp::GetTypeId() {
     static TypeId tid = TypeId("ns3::SdnControllerApp").SetParent<Application>().AddConstructor<SdnControllerApp>();
     return tid;
 }
 SdnControllerApp::SdnControllerApp() {}
 void SdnControllerApp::StartApplication() {
     Simulator::Schedule(Seconds(1.0), &SdnControllerApp::RecomputeRoutes, this);
     NS_LOG_UNCOND("SDN Controller Started (Magic LTE Mode)");
 }
 void SdnControllerApp::StopApplication() {}
 
 void SdnControllerApp::RegisterVehicle(Ipv4Address ip, Ptr<VehicleSdnApp> app) {
     m_vehicleMap[ip] = app;
 }
 
 void SdnControllerApp::ReceiveReport(Ipv4Address nodeIp, std::set<Ipv4Address> neighbors) {
     m_topology[nodeIp] = neighbors;
 }
 
 static std::map<Ipv4Address, Ipv4Address> ComputeNextHop(const std::map<Ipv4Address, std::set<Ipv4Address>>& topo, Ipv4Address src) {
     std::map<Ipv4Address, Ipv4Address> nextHop;
     std::queue<Ipv4Address> q;
     std::set<Ipv4Address> visited;
     q.push(src);
     visited.insert(src);
     while (!q.empty()) {
         Ipv4Address u = q.front(); q.pop();
         if (topo.find(u) == topo.end()) continue;
         for (auto v : topo.at(u)) {
             if (!visited.count(v)) {
                 visited.insert(v);
                 q.push(v);
                 nextHop[v] = (u == src) ? v : nextHop[u];
             }
         }
     }
     return nextHop;
 }
 
 void SdnControllerApp::RecomputeRoutes() {
     int routesCount = 0;
     for (auto& entry : m_topology) {
         Ipv4Address srcIp = entry.first;
         if (m_vehicleMap.find(srcIp) == m_vehicleMap.end()) continue;
 
         auto nhMap = ComputeNextHop(m_topology, srcIp);
         Ptr<VehicleSdnApp> vehicle = m_vehicleMap[srcIp];
 
         for (auto& p : nhMap) {
             vehicle->InstallRoute(p.first, p.second);
             routesCount++;
         }
     }
     Simulator::Schedule(Seconds(1.0), &SdnControllerApp::RecomputeRoutes, this);
 }
 
 // --- VEHICLE ---
 NS_OBJECT_ENSURE_REGISTERED(VehicleSdnApp);
 TypeId VehicleSdnApp::GetTypeId() {
     static TypeId tid = TypeId("ns3::VehicleSdnApp").SetParent<Application>().AddConstructor<VehicleSdnApp>();
     return tid;
 }
 VehicleSdnApp::VehicleSdnApp() {}
 void VehicleSdnApp::SetController(Ptr<SdnControllerApp> controller) { m_controller = controller; }
 
 void VehicleSdnApp::StartApplication() {
     m_beaconTx = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
     m_beaconTx->SetAllowBroadcast(true);
     m_beaconRx = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
     m_beaconRx->Bind(InetSocketAddress(Ipv4Address::GetAny(), 8888));
     m_beaconRx->SetRecvCallback(MakeCallback(&VehicleSdnApp::ReceiveBeacon, this));
 
     Simulator::Schedule(Seconds(0.5), &VehicleSdnApp::SendBeacon, this);
     Simulator::Schedule(Seconds(1.0), &VehicleSdnApp::SendReport, this);
 }
 void VehicleSdnApp::StopApplication() {
     if(m_beaconTx) m_beaconTx->Close();
     if(m_beaconRx) m_beaconRx->Close();
 }
 void VehicleSdnApp::SendBeacon() {
     Ptr<Packet> p = Create<Packet>(10);
     m_beaconTx->SendTo(p, 0, InetSocketAddress(Ipv4Address("255.255.255.255"), 8888));
     Simulator::Schedule(Seconds(0.5), &VehicleSdnApp::SendBeacon, this);
 }
 void VehicleSdnApp::ReceiveBeacon(Ptr<Socket> socket) {
     Address from;
     socket->RecvFrom(from);
     m_neighbors.insert(InetSocketAddress::ConvertFrom(from).GetIpv4());
 }
 void VehicleSdnApp::SendReport() {
     // "Magic" LTE: Direct function call
     if (m_controller) {
         Ptr<Ipv4> ipv4 = GetNode()->GetObject<Ipv4>();
         Ipv4Address myIp = ipv4->GetAddress(1, 0).GetLocal();
         m_controller->ReceiveReport(myIp, m_neighbors);
     }
     m_neighbors.clear();
     Simulator::Schedule(Seconds(1.0), &VehicleSdnApp::SendReport, this);
 }
 void VehicleSdnApp::InstallRoute(Ipv4Address dst, Ipv4Address nh) {
    Ptr<Ipv4> ipv4 = GetNode()->GetObject<Ipv4>();
    Ptr<Ipv4StaticRouting> staticRouting;
    Ptr<Ipv4RoutingProtocol> rp = ipv4->GetRoutingProtocol();
    
    // 1. Get Static Routing Protocol
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
    
    // 2. Install Static Route
    if (staticRouting) {
        staticRouting->AddHostRouteTo(dst, nh, 1, 1);
    }

    // 3. CRITICAL FIX: Pre-fill ARP Cache (Layer 2)
    // Fix: Cast to Ipv4L3Protocol to access GetInterface()
    Ptr<Ipv4L3Protocol> ipv4l3 = DynamicCast<Ipv4L3Protocol>(ipv4);
    if (!ipv4l3) return;

    int32_t interfaceIndex = 1; // WiFi Interface
    Ptr<ArpCache> arpCache = ipv4l3->GetInterface(interfaceIndex)->GetArpCache();
    
    if (arpCache) {
        Mac48Address nhMac;
        bool found = false;
        
        // Find MAC of Next Hop by scanning Global Node List
        for (uint32_t i = 0; i < NodeList::GetNNodes(); ++i) {
            Ptr<Node> candidate = NodeList::GetNode(i);
            Ptr<Ipv4> candIpv4 = candidate->GetObject<Ipv4>();
            // Check IP on Interface 1
            if (candIpv4->GetNInterfaces() > 1 && candIpv4->GetAddress(1,0).GetLocal() == nh) {
                Ptr<NetDevice> candDev = candIpv4->GetNetDevice(1); 
                nhMac = Mac48Address::ConvertFrom(candDev->GetAddress());
                found = true;
                break;
            }
        }
        
        if (found) {
            ArpCache::Entry *entry = arpCache->Add(nh);
            entry->MarkAlive(nhMac); 
            // Fix: Removed 'SetIsStatic'. MarkAlive is sufficient.
            // Timeout is handled globally in main().
        }
    }
}
 
 } // End namespace
 
 // ============================================================================
 //  MAIN
 // ============================================================================
 
 int main(int argc, char *argv[]) {
     std::string protocol = "SDN"; 
     uint32_t nNodes = 50;          
     double simTime = 300.0;        
     uint32_t run = 1;
     uint32_t speed = 20;           
 
     CommandLine cmd;
     cmd.AddValue("protocol", "Protocol", protocol);
     cmd.AddValue("nodes", "Nodes", nNodes);
     cmd.AddValue("speed", "Speed", speed);
     cmd.AddValue("run", "Run", run);
     cmd.Parse(argc, argv);
     RngSeedManager::SetSeed(112233);
     RngSeedManager::SetRun(run);
 
     Config::SetDefault ("ns3::RandomRectanglePositionAllocator::X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1000.0]"));
     Config::SetDefault ("ns3::RandomRectanglePositionAllocator::Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1000.0]"));
 
     // Prevent ARP entries from expiring (effectively making them static)
    Config::SetDefault("ns3::ArpCache::AliveTimeout", TimeValue(Seconds(10000.0)));
     NodeContainer controllerNode; controllerNode.Create(1);
     NodeContainer vehicleNodes;   vehicleNodes.Create(nNodes);
     NodeContainer allNodes;       allNodes.Add(controllerNode); allNodes.Add(vehicleNodes);
 
     MobilityHelper mobilityCtrl;
     mobilityCtrl.SetMobilityModel("ns3::ConstantPositionMobilityModel");
     mobilityCtrl.Install(controllerNode);
     controllerNode.Get(0)->GetObject<ConstantPositionMobilityModel>()->SetPosition(Vector(500, 500, 0));
 
     std::ostringstream traceFile;
     traceFile << "scratch/mobility_" << speed << "ms.tcl";
     Ns2MobilityHelper ns2 = Ns2MobilityHelper(traceFile.str());
     ns2.Install(vehicleNodes.Begin(), vehicleNodes.End());
 
     WifiHelper wifi;
     wifi.SetStandard(WIFI_STANDARD_80211p);
     wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue("OfdmRate6MbpsBW10MHz"));
     YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
     channel.AddPropagationLoss("ns3::FriisPropagationLossModel", "Frequency", DoubleValue(5.9e9));
     YansWifiPhyHelper phy;
     phy.SetChannel(channel.Create());
     phy.Set("TxPowerStart", DoubleValue(20.0)); 
     phy.Set("TxPowerEnd", DoubleValue(20.0));
     WifiMacHelper mac;
     mac.SetType("ns3::AdhocWifiMac");
     NetDeviceContainer devices = wifi.Install(phy, mac, allNodes);
 
     InternetStackHelper internet;
     if (protocol == "AODV") { internet.SetRoutingHelper(AodvHelper()); internet.Install(allNodes); }
     else if (protocol == "OLSR") { internet.SetRoutingHelper(OlsrHelper()); internet.Install(allNodes); }
     else if (protocol == "DSDV") { internet.SetRoutingHelper(DsdvHelper()); internet.Install(allNodes); }
     else if (protocol == "SDN") { internet.Install(allNodes); }
     else { NS_FATAL_ERROR("Unknown Protocol"); }
 
     Ipv4AddressHelper ipv4;
     ipv4.SetBase("10.1.0.0", "255.255.0.0");
     Ipv4InterfaceContainer ifs = ipv4.Assign(devices);
 
     // --- SDN SETUP ---
     if (protocol == "SDN") {
         Ptr<SdnControllerApp> ctrlApp = CreateObject<SdnControllerApp>();
         controllerNode.Get(0)->AddApplication(ctrlApp);
         ctrlApp->SetStartTime(Seconds(0.5));
 
         for (uint32_t i = 0; i < vehicleNodes.GetN(); ++i) {
             Ptr<VehicleSdnApp> vehApp = CreateObject<VehicleSdnApp>();
             vehApp->SetController(ctrlApp);
             vehicleNodes.Get(i)->AddApplication(vehApp);
             vehApp->SetStartTime(Seconds(1.0 + i * 0.01));
             
             // Register vehicle IP with controller
             // Vehicle i is at IP index i+1 (Index 0 is Controller)
             Ipv4Address vehIp = ifs.GetAddress(i + 1);
             ctrlApp->RegisterVehicle(vehIp, vehApp);
         }
     }
 
     // --- TRAFFIC (Smart Selection) ---
     Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable>();
     int dstIdx = -1, srcIdx = -1;
     bool foundPair = false;
     for (int attempts = 0; attempts < 100; ++attempts) {
         int s = rng->GetInteger(0, nNodes - 1);
         int d = rng->GetInteger(0, nNodes - 1);
         if (s == d) continue;
         Ptr<MobilityModel> m1 = vehicleNodes.Get(s)->GetObject<MobilityModel>();
         Ptr<MobilityModel> m2 = vehicleNodes.Get(d)->GetObject<MobilityModel>();
         double dist = m1->GetDistanceFrom(m2);
         // Smart Selection: Must be >250m (multihop) but <700m (reachable)
         if (dist > 250.0 && dist < 700.0) { srcIdx=s; dstIdx=d; foundPair=true; break; }
     }
     if (!foundPair) { srcIdx=0; dstIdx=1; }
     
     // Get Dest IP directly from Node Object (Avoids Index Math Bugs)
     Ptr<Node> destNode = vehicleNodes.Get(dstIdx);
     Ptr<Ipv4> ipv4Dest = destNode->GetObject<Ipv4>();
     Ipv4Address dstIp = ipv4Dest->GetAddress(1, 0).GetLocal();
     
     NS_LOG_UNCOND("TRAFFIC: SrcNode=" << vehicleNodes.Get(srcIdx)->GetId() << " DstNode=" << destNode->GetId() << " DstIP=" << dstIp);
 
     UdpEchoServerHelper server(9);
     server.Install(vehicleNodes.Get(dstIdx)).Start(Seconds(1.0));
     UdpEchoClientHelper client(dstIp, 9);
     client.SetAttribute("Interval", TimeValue(Seconds(0.25)));
     client.SetAttribute("PacketSize", UintegerValue(1024));
     client.SetAttribute("MaxPackets", UintegerValue(1000));
     client.Install(vehicleNodes.Get(srcIdx)).Start(Seconds(5.0));
 
     FlowMonitorHelper flowmon;
     Ptr<FlowMonitor> monitor = flowmon.InstallAll();
 
     Simulator::Stop(Seconds(simTime));
     Simulator::Run();
     
     std::ostringstream fileName;
     fileName << "results_" << protocol << "_" << speed << "_" << run << ".xml";
     monitor->CheckForLostPackets();
     monitor->SerializeToXmlFile(fileName.str(), true, true);
     Simulator::Destroy();
     return 0;
 }