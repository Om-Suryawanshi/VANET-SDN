/* * SDN Debug / Sanity Check Script
 * Purpose: Verify Mobility, WiFi Connectivity, and Route Injection in isolation.
 */

 #include "ns3/core-module.h"
 #include "ns3/network-module.h"
 #include "ns3/mobility-module.h"
 #include "ns3/wifi-module.h"
 #include "ns3/internet-module.h"
 #include "ns3/applications-module.h"
 #include "ns3/ipv4-static-routing.h"
 #include "ns3/ipv4-list-routing.h"
 
 #include <iostream>
 #include <vector>
 
 using namespace ns3;
 
 NS_LOG_COMPONENT_DEFINE("DebugSimulation");
 
 // --- GLOBAL VARIABLES ---
 NodeContainer vehicles;
 Ipv4InterfaceContainer vehIfs;
 double simTime = 10.0; 
 
 // --- DEBUG PRINTER (Runs every 1 second) ---
 void DebugState(int second) {
     if (second > simTime) return;
 
     std::cout << "\n[T=" << second << "s] DEBUG REPORT:" << std::endl;
     
     // 1. Check Positions
     Ptr<MobilityModel> m0 = vehicles.Get(0)->GetObject<MobilityModel>();
     Ptr<MobilityModel> m1 = vehicles.Get(1)->GetObject<MobilityModel>();
     Vector p0 = m0->GetPosition();
     Vector p1 = m1->GetPosition();
     double dist = m0->GetDistanceFrom(m1);
     
     std::cout << "  > Positions: Veh0(" << p0.x << "," << p0.y << ")  Veh1(" << p1.x << "," << p1.y << ")" << std::endl;
     std::cout << "  > Distance:  " << dist << "m" << std::endl;
 
     // 2. Check Routing Table (Veh0)
     Ptr<Ipv4> ipv4 = vehicles.Get(0)->GetObject<Ipv4>();
     Ptr<Ipv4RoutingProtocol> rp = ipv4->GetRoutingProtocol();
     Ptr<Ipv4StaticRouting> staticRouting;
     
     if (DynamicCast<Ipv4ListRouting>(rp)) {
         Ptr<Ipv4ListRouting> lr = DynamicCast<Ipv4ListRouting>(rp);
         int16_t priority;
         // Check first few protocols
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
 
     if (staticRouting) {
         if (staticRouting->GetNRoutes() == 0) {
             std::cout << "  > Routing:   Veh0 Table is EMPTY." << std::endl;
         } else {
             std::cout << "  > Routing:   Veh0 has " << staticRouting->GetNRoutes() << " routes." << std::endl;
             for (uint32_t i=0; i < staticRouting->GetNRoutes(); i++) {
                  Ipv4RoutingTableEntry route = staticRouting->GetRoute(i);
                  if (route.GetDest() != Ipv4Address::GetLoopback())
                     std::cout << "     - Rule: To " << route.GetDest() << " via " << route.GetGateway() << std::endl;
             }
         }
     } else {
         std::cout << "  > Routing:   ERROR (No Static Routing Protocol)" << std::endl;
     }
 
     Simulator::Schedule(Seconds(1.0), &DebugState, second + 1);
 }
 
 // --- MAIN ---
 int main(int argc, char *argv[]) {
     LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
     LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);
 
     // 1. Create 2 Nodes
     vehicles.Create(2); 
     
     // 2. Mobility: Place them 100m apart (Well within 250m range)
     MobilityHelper mobility;
     Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
     positionAlloc->Add(Vector(0.0, 0.0, 0.0));   // Veh 0
     positionAlloc->Add(Vector(100.0, 0.0, 0.0)); // Veh 1
     mobility.SetPositionAllocator(positionAlloc);
     mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
     mobility.Install(vehicles);
 
     // 3. WiFi: Standard 802.11p
     WifiHelper wifi;
     wifi.SetStandard(WIFI_STANDARD_80211p);
     wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager");
     YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
     channel.AddPropagationLoss("ns3::FriisPropagationLossModel", "Frequency", DoubleValue(5.9e9));
     YansWifiPhyHelper phy;
     phy.SetChannel(channel.Create());
     phy.Set("TxPowerStart", DoubleValue(20.0));
     phy.Set("TxPowerEnd", DoubleValue(20.0));
     WifiMacHelper mac;
     mac.SetType("ns3::AdhocWifiMac");
     NetDeviceContainer devs = wifi.Install(phy, mac, vehicles);
 
     // 4. Internet
     InternetStackHelper internet;
     internet.Install(vehicles);
     
     Ipv4AddressHelper ipv4;
     ipv4.SetBase("10.0.0.0", "255.255.255.0");
     vehIfs = ipv4.Assign(devs);
     
     // 5. MANUAL ROUTE INJECTION (Simulate what SDN does)
     // At T=2s, we force Veh0 to know that Veh1 is reachable via direct link
     Simulator::Schedule(Seconds(2.0), +[](Ptr<Node> n, Ipv4Address dst, Ipv4Address gw){
         Ptr<Ipv4> ipv4 = n->GetObject<Ipv4>();
         Ptr<Ipv4ListRouting> lr = DynamicCast<Ipv4ListRouting>(ipv4->GetRoutingProtocol());
         int16_t p;
         // Assuming Static is at index 0 or found via iteration (simplified for debug)
         Ptr<Ipv4StaticRouting> sr = DynamicCast<Ipv4StaticRouting>(lr->GetRoutingProtocol(0, p)); 
         
         // Add Route: Dest=Veh1, NextHop=Veh1, Interface=1, Metric=1
         sr->AddHostRouteTo(dst, gw, 1, 1);
         std::cout << "\n[!] CONTROLLER EVENT: Injected route into Veh0" << std::endl;
     }, vehicles.Get(0), vehIfs.GetAddress(1), vehIfs.GetAddress(1));
 
     // 6. Traffic: Veh0 -> Veh1
     UdpEchoServerHelper server(9);
     server.Install(vehicles.Get(1)).Start(Seconds(1.0));
     
     UdpEchoClientHelper client(vehIfs.GetAddress(1), 9);
     client.SetAttribute("Interval", TimeValue(Seconds(1.0)));
     client.SetAttribute("PacketSize", UintegerValue(1024));
     client.SetAttribute("MaxPackets", UintegerValue(10));
     client.Install(vehicles.Get(0)).Start(Seconds(3.0)); // Start AFTER route injection
 
     // 7. Run
     Simulator::Schedule(Seconds(0.1), &DebugState, 0);
     Simulator::Stop(Seconds(simTime));
     Simulator::Run();
     Simulator::Destroy();
     return 0;
 }