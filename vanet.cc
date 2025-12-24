/*
 * Reproduction of Figure 10 (Traditional Routing) with XML & NetAnim Output
 * Reference: "Towards Software-Defined VANET: Architecture and Services"
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h" 
#include "ns3/aodv-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/olsr-module.h"

#include <string>
#include <sstream>
#include <sys/stat.h> 

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("VanetSumoXml");

int main (int argc, char *argv[])
{
  // --- 1. Parameters ---
  std::string protocol = "AODV";
  int speed = 10;
  int runId = 1;
  std::string traceFile = ""; 
  std::string outputDir = "."; 
  double simTime = 300.0;      
  uint32_t numNodes = 50;
  bool enableNetAnim = false; 

  CommandLine cmd;
  cmd.AddValue ("protocol", "Routing protocol (AODV, DSDV, OLSR)", protocol);
  cmd.AddValue ("speed", "Speed of nodes", speed);
  cmd.AddValue ("runId", "Run identifier", runId);
  cmd.AddValue ("traceFile", "Override path to trace file", traceFile);
  cmd.AddValue ("outputDir", "Directory to save results", outputDir);
  cmd.AddValue ("netanim", "Enable NetAnim output (true/false)", enableNetAnim);
  cmd.Parse (argc, argv);

  // Auto-detect trace file in scratch/mobility if not provided
  if (traceFile.empty ()) {
      traceFile = "scratch/mobility/mobility_" + std::to_string(speed) + ".tcl";
  }

  // File Names
  std::stringstream ss;
  ss << outputDir << "/result_" << protocol << "_" << speed << "_" << runId << ".xml";
  std::string xmlFileName = ss.str();

  std::stringstream ssAnim;
  ssAnim << outputDir << "/netanim_" << protocol << "_" << speed << "_" << runId << ".xml";
  std::string animFileName = ssAnim.str();

  RngSeedManager::SetSeed (3 + runId); 
  RngSeedManager::SetRun (runId);

  // --- 2. Node Creation ---
  NodeContainer nodes;
  nodes.Create (numNodes);

  // --- 3. Mobility ---
  Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);
  ns2.Install (nodes.Begin(), nodes.End());

  // --- 4. Wifi Setup ---
  WifiHelper wifi;
  wifi.SetStandard (WIFI_STANDARD_80211a);

  YansWifiPhyHelper wifiPhy;
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
  wifiPhy.SetChannel (wifiChannel.Create ());
  
  wifiPhy.Set ("TxPowerStart", DoubleValue(16.0));
  wifiPhy.Set ("TxPowerEnd", DoubleValue(16.0));

  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac");

  NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, nodes);

  // --- 5. Routing ---
  Ipv4ListRoutingHelper list;
  InternetStackHelper stack;

  if (protocol == "AODV") {
      AodvHelper aodv; list.Add (aodv, 100);
  } else if (protocol == "DSDV") {
      DsdvHelper dsdv; list.Add (dsdv, 100);
  } else if (protocol == "OLSR") {
      OlsrHelper olsr; list.Add (olsr, 100);
  }

  stack.SetRoutingHelper (list);
  stack.Install (nodes);

  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interfaces = address.Assign (devices);

  // --- 6. Applications ---
  uint16_t port = 9;
  
  // Server Node (Index 0)
  UdpEchoServerHelper server (port);
  ApplicationContainer serverApps = server.Install (nodes.Get (0));
  serverApps.Start (Seconds (1.0));
  serverApps.Stop (Seconds (simTime));

  // Client Node (Last Index)
  UdpEchoClientHelper client (interfaces.GetAddress (0), port);
  client.SetAttribute ("MaxPackets", UintegerValue (100000));
  client.SetAttribute ("Interval", TimeValue (Seconds (0.25))); 
  client.SetAttribute ("PacketSize", UintegerValue (1024));

  ApplicationContainer clientApps = client.Install (nodes.Get (numNodes - 1));
  clientApps.Start (Seconds (2.0));
  clientApps.Stop (Seconds (simTime));

  // --- 7. Flow Monitor ---
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll ();

  // --- 8. NetAnim Setup ---
  AnimationInterface *anim = 0;
  if (enableNetAnim)
    {
      anim = new AnimationInterface (animFileName);
      anim->EnablePacketMetadata (true); 
      anim->SetMaxPktsPerTraceFile(999999999999);
      
      // 1. Set default color (BLUE) for all cars
      for (uint32_t i = 0; i < nodes.GetN(); ++i) {
          anim->UpdateNodeColor (nodes.Get (i), 0, 0, 255); // RGB: Blue
          std::string desc = "Car " + std::to_string(i);
          anim->UpdateNodeDescription(nodes.Get(i), desc); 
      }

      // 2. Highlight SERVER (Node 0) -> GREEN
      Ptr<Node> serverNode = nodes.Get(0);
      anim->UpdateNodeColor (serverNode, 0, 255, 0); 
      anim->UpdateNodeDescription(serverNode, "SERVER");
      // FIX: Use GetId() for UpdateNodeSize
      anim->UpdateNodeSize(serverNode->GetId(), 5.0, 5.0); 

      // 3. Highlight CLIENT (Last Node) -> RED
      Ptr<Node> clientNode = nodes.Get(numNodes - 1);
      anim->UpdateNodeColor (clientNode, 255, 0, 0); 
      anim->UpdateNodeDescription(clientNode, "CLIENT");
      // FIX: Use GetId() for UpdateNodeSize
      anim->UpdateNodeSize(clientNode->GetId(), 5.0, 5.0); 
    }

  // --- 9. Run ---
  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();

  // --- 10. Save Results ---
  monitor->CheckForLostPackets ();
  monitor->SerializeToXmlFile (xmlFileName, true, true);
  
  if (anim)
    {
      delete anim; // Important to flush XML file
    }

  Simulator::Destroy ();
  return 0;
}
