/*
 * PBRDV Implementation for NS-3
 * Based on: "Time Series Prediction QoS Routing in Software Defined VANET"
 * * Implements:
 * 1. Vehicle Mobility Reporting (Pos + Vel) 
 * 2. Controller Topology Prediction (T+0 to T+Horiz) [cite: 161]
 * 3. Pre-calculated Route Distribution [cite: 173]
 * 4. Local Route Application during Controller Failure 
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
#include "ns3/aodv-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/olsr-module.h"

#include <vector>
#include <map>
#include <set>
#include <queue>
#include <string>
#include <sstream>
#include <fstream>
#include <cmath>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("VanetPbrdvFinal");

NodeContainer* g_sdnNodes = nullptr;

// Global variables for failure simulation
double g_failureStartTime = -1.0;
double g_failureDuration = 0.0;

// ============================================================================
//                      PDR MONITORING & GRAPHING
// ============================================================================
uint32_t g_pdrTxCount = 0;
uint32_t g_pdrRxCount = 0;

void MonitorTxCallback(Ptr<const Packet> p) { g_pdrTxCount++; }
void MonitorRxCallback(Ptr<const Packet> p) { g_pdrRxCount++; }

void RecordPdrPerSecond(std::string filename) {
    static std::ofstream file;
    static double currentTime = 0;

    if (!file.is_open()) {
        file.open(filename.c_str(), std::ios::out);
        file << "Time,PDR,Tx,Rx" << std::endl;
    }

    double pdr = 0.0;
    if (g_pdrTxCount > 0) {
        pdr = (double)g_pdrRxCount / g_pdrTxCount * 100.0;
    }

    file << currentTime << "," << pdr << "," << g_pdrTxCount << "," << g_pdrRxCount << std::endl;

    g_pdrTxCount = 0;
    g_pdrRxCount = 0;
    currentTime += 1.0;

    Simulator::Schedule(Seconds(1.0), &RecordPdrPerSecond, filename);
}

// ============================================================================
//                      CONFIGURATION CONSTANTS
// ============================================================================
const double BEACON_INTERVAL = 0.5;           
const double REPORT_INTERVAL = 1.0;           // Slower report interval (PBRDV relies on prediction)
const double ROUTE_RECOMPUTE_INTERVAL = 1.0;  // Controller predicts every 1s
const double CONTROLLER_START_TIME = 0.5;     
const double JITTER_MULTIPLIER = 0.002;       

// PBRDV SPECIFIC CONSTANTS
const int PREDICTION_HORIZON = 25;            // Predict 10 seconds into the future [cite: 164]
const double PREDICTION_STEP_SIZE = 1.0;      // 1 second steps

const double VEHICLE_RANGE_THRESHOLD = 230.0; 
const double CONTROLLER_LINK_THRESHOLD = 210.0; 
const double WIFI_DATA_MAX_RANGE = 350.0;       
const double WIFI_CTRL_MAX_RANGE = 2000.0;      

const uint32_t DATA_IF_INDEX = 1;             
const uint32_t CTRL_IF_INDEX = 2;             
const uint32_t CTRL_NODE_IF_INDEX = 1;        

const uint16_t BEACON_PORT = 8888;            
const uint16_t CONTROLLER_REPORT_PORT = 9999; 
const uint16_t VEHICLE_ROUTE_PORT = 10000;    
const uint16_t DATA_APP_PORT = 9;             

const double SERVER_START_TIME = 5.0;         
const double CLIENT_START_TIME = 10.0;        
const double VEHICLE_APP_START_OFFSET = 0.5;  
const double VEHICLE_APP_STAGGER = 0.01;      
const double CONTROLLER_SEND_OFFSET = 0.0005; 
const uint32_t CONTROLLER_NODE_ID = 50;       

const char* DATA_NETWORK_PREFIX = "10.1.0.0";
const char* DATA_NETWORK_MASK = "255.255.0.0";
const char* CTRL_NETWORK_PREFIX = "10.2.0.0";
const char* CTRL_NETWORK_MASK = "255.255.0.0";

const uint32_t MAX_PACKETS = 100000;          
const double PACKET_INTERVAL = 0.1;           
const uint32_t PACKET_SIZE = 1024;            

// ============================================================================
//                          DATA STRUCTURES
// ============================================================================

struct MobilityData {
    Vector position;
    Vector velocity;
    double timestamp;
};

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
    VehicleSdnApp() : m_dataIfIndex(DATA_IF_INDEX), m_ctrlIfIndex(CTRL_IF_INDEX) {}
    
    void Setup(Ipv4Address controllerIp, uint32_t dataIfIndex, uint32_t ctrlIfIndex) {
        m_controllerIp = controllerIp; m_dataIfIndex = dataIfIndex; m_ctrlIfIndex = ctrlIfIndex;
    }

private:
    virtual void StartApplication() {
        // Beacon TX/RX setup
        m_beaconTx = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_beaconTx->SetAllowBroadcast(true);
        m_beaconTx->Bind(InetSocketAddress(GetNodeIpv4Address(GetNode(), m_ctrlIfIndex), 0));

        m_beaconRx = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_beaconRx->Bind(InetSocketAddress(Ipv4Address::GetAny(), BEACON_PORT));
        m_beaconRx->SetRecvCallback(MakeCallback(&VehicleSdnApp::ReceiveBeacon, this));

        // Control RX (Route Updates)
        m_ctrlRx = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_ctrlRx->Bind(InetSocketAddress(GetNodeIpv4Address(GetNode(), m_ctrlIfIndex), VEHICLE_ROUTE_PORT));
        m_ctrlRx->SetRecvCallback(MakeCallback(&VehicleSdnApp::ReceivePbrdvUpdate, this));

        double jitter = (double)(GetNode()->GetId()) * JITTER_MULTIPLIER; 
        
        m_beaconEvent = Simulator::Schedule(Seconds(BEACON_INTERVAL + jitter), &VehicleSdnApp::SendBeacon, this);
        m_reportEvent = Simulator::Schedule(Seconds(REPORT_INTERVAL + jitter), &VehicleSdnApp::SendReport, this);
        
        // PBRDV: Schedule the Local Prediction Check
        m_predictionApplyEvent = Simulator::Schedule(Seconds(1.0), &VehicleSdnApp::ApplyScheduledRoutes, this);
    }

    virtual void StopApplication() {
        Simulator::Cancel(m_beaconEvent); Simulator::Cancel(m_reportEvent); Simulator::Cancel(m_predictionApplyEvent);
        if(m_beaconTx) m_beaconTx->Close(); if(m_beaconRx) m_beaconRx->Close(); if(m_ctrlRx) m_ctrlRx->Close();
    }

    void SendBeacon() {
        std::ostringstream oss; oss << GetNodeIpv4Address(GetNode(), m_ctrlIfIndex);
        std::string data = oss.str();
        Ptr<Packet> p = Create<Packet>((uint8_t*)data.c_str(), data.size());
        m_beaconTx->SendTo(p, 0, InetSocketAddress(Ipv4Address("255.255.255.255"), BEACON_PORT));
        m_beaconEvent = Simulator::Schedule(Seconds(BEACON_INTERVAL), &VehicleSdnApp::SendBeacon, this);
    }

    void ReceiveBeacon(Ptr<Socket> socket) {
        Address from; Ptr<Packet> pkt = socket->RecvFrom(from);
        if (pkt->GetSize() == 0) return;
        std::vector<uint8_t> buffer(pkt->GetSize() + 1, 0); pkt->CopyData(buffer.data(), pkt->GetSize());
        std::string senderIpStr((char*)buffer.data());
        if (senderIpStr.find(".") == std::string::npos) return;
        Ipv4Address neighborCtrlIp(senderIpStr.c_str());
        if (neighborCtrlIp != GetNodeIpv4Address(GetNode(), m_ctrlIfIndex)) 
            m_neighbors.insert(neighborCtrlIp);
    }

    // [PBRDV] Modified to send Position and Velocity 
    void SendReport() {
        Ptr<MobilityModel> mob = GetNode()->GetObject<MobilityModel>();
        Vector pos = mob->GetPosition();
        Vector vel = mob->GetVelocity();

        std::ostringstream ss;
        // Format: MyCtrlIP PosX PosY VelX VelY Neighbor1 ...
        ss << GetNodeIpv4Address(GetNode(), m_ctrlIfIndex) << " "
           << pos.x << " " << pos.y << " " << vel.x << " " << vel.y;
        
        for (const auto& n : m_neighbors) ss << " " << n;

        std::string data = ss.str();
        Ptr<Packet> pkt = Create<Packet>((uint8_t*)data.c_str(), data.size());
        Ptr<Socket> s = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        s->Bind(InetSocketAddress(GetNodeIpv4Address(GetNode(), m_ctrlIfIndex), 0)); 
        s->Connect(InetSocketAddress(m_controllerIp, CONTROLLER_REPORT_PORT));
        s->Send(pkt); s->Close();
        m_neighbors.clear();
        m_reportEvent = Simulator::Schedule(Seconds(REPORT_INTERVAL), &VehicleSdnApp::SendReport, this);
    }

    // [PBRDV] Logic: Receive batch of future routes and store in buffer [cite: 173, 174]
    void ReceivePbrdvUpdate(Ptr<Socket> socket) {
        Address from; Ptr<Packet> pkt = socket->RecvFrom(from);
        if (pkt->GetSize() == 0) return;
        
        // Reset the "Last Contact" time so we know where T+0 is
        m_lastControllerUpdate = Simulator::Now();
        
        std::vector<uint8_t> buffer(pkt->GetSize() + 1, 0); pkt->CopyData(buffer.data(), pkt->GetSize());
        std::stringstream ss((char*)buffer.data());
        
        int timeOffset;
        std::string dstStr, nhStr;

        // Clear old predictions
        m_pbrdvBuffer.clear();

        // Parse: [TimeOffset DstIP NextHopIP] ...
        while (ss >> timeOffset >> dstStr >> nhStr) {
            if (dstStr.find('.') == std::string::npos) continue;
            Ipv4Address dst(dstStr.c_str());
            Ipv4Address nh(nhStr.c_str());
            m_pbrdvBuffer[timeOffset][dst] = nh;
        }
    }

    // [PBRDV] Logic: Apply routes for current time slice (T+delta) [cite: 180]
    void ApplyScheduledRoutes() {
        Time now = Simulator::Now();
        
        // How many seconds since the last controller update?
        // If controller is alive, delta is ~0 or ~1.
        // If controller DEAD, delta grows (2, 3, 4...).
        int deltaSeconds = std::round((now - m_lastControllerUpdate).GetSeconds());

        if (deltaSeconds > PREDICTION_HORIZON) {
             NS_LOG_UNCOND("[" << now.GetSeconds() << "s] Node " << GetNode()->GetId() << ": Prediction buffer expired! No routes.");
        } 
        else if (m_pbrdvBuffer.find(deltaSeconds) != m_pbrdvBuffer.end()) {
            // We have predictions for this time slice!
            Ptr<Ipv4StaticRouting> staticRouting = GetStaticRouting();
            if (staticRouting) {
                // Clear old SDN routes
                PurgeStaticRoutes(staticRouting);
                
                // Install Predicted Routes
                std::map<Ipv4Address, Ipv4Address> routes = m_pbrdvBuffer[deltaSeconds];
                // Only log occasionally to avoid clutter
                if (GetNode()->GetId() == 0) {
                     NS_LOG_UNCOND("[" << now.GetSeconds() << "s] PBRDV: Node 0 Applying routes for T+" << deltaSeconds << "s");
                }
                
                for (auto const& [dst, nh] : routes) {
                    staticRouting->AddHostRouteTo(dst, nh, m_dataIfIndex, 0);
                }
            }
        }

        m_predictionApplyEvent = Simulator::Schedule(Seconds(1.0), &VehicleSdnApp::ApplyScheduledRoutes, this);
    }

    Ptr<Ipv4StaticRouting> GetStaticRouting() {
        Ptr<Ipv4> ipv4 = GetNode()->GetObject<Ipv4>();
        Ptr<Ipv4ListRouting> lr = DynamicCast<Ipv4ListRouting>(ipv4->GetRoutingProtocol());
        if (lr) {
            int16_t priority;
            for (uint32_t i = 0; i < lr->GetNRoutingProtocols(); i++) {
                Ptr<Ipv4RoutingProtocol> temp = lr->GetRoutingProtocol(i, priority);
                if (DynamicCast<Ipv4StaticRouting>(temp)) { 
                    return DynamicCast<Ipv4StaticRouting>(temp); 
                }
            }
        }
        return nullptr;
    }

    void PurgeStaticRoutes(Ptr<Ipv4StaticRouting> staticRouting) {
        for (uint32_t i = staticRouting->GetNRoutes(); i > 0; i--) {
            Ipv4RoutingTableEntry route = staticRouting->GetRoute(i-1);
            if (route.GetDestNetworkMask() == Ipv4Mask("255.255.255.255")) {
                 if (route.GetDest().CombineMask(Ipv4Mask("255.255.0.0")) == Ipv4Address("10.1.0.0")) {
                    staticRouting->RemoveRoute(i-1);
                }
            }
        }
    }

    Ptr<Socket> m_beaconTx, m_beaconRx, m_ctrlRx;
    Ipv4Address m_controllerIp;
    uint32_t m_dataIfIndex, m_ctrlIfIndex;
    std::set<Ipv4Address> m_neighbors;
    
    // PBRDV Storage
    Time m_lastControllerUpdate;
    // Map<TimeOffset, Map<Dst, NextHop>>
    std::map<int, std::map<Ipv4Address, Ipv4Address>> m_pbrdvBuffer;
    
    EventId m_beaconEvent, m_reportEvent, m_predictionApplyEvent;
};

// ============================================================================
//                              CONTROLLER APP
// ============================================================================

class SdnControllerApp : public Application {
public:
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("ns3::SdnControllerApp").SetParent<Application>().AddConstructor<SdnControllerApp>();
        return tid;
    }
    SdnControllerApp() : m_ctrlIfIndex(CTRL_NODE_IF_INDEX) {}
    void SetCtrlIfIndex(uint32_t idx) { m_ctrlIfIndex = idx; }

private:
    virtual void StartApplication() {
        m_recvSocket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_recvSocket->Bind(InetSocketAddress(GetNodeIpv4Address(GetNode(), m_ctrlIfIndex), CONTROLLER_REPORT_PORT));
        m_recvSocket->SetRecvCallback(MakeCallback(&SdnControllerApp::ReceiveReport, this));
        m_routeEvent = Simulator::Schedule(Seconds(CONTROLLER_START_TIME), &SdnControllerApp::RecomputeRoutes, this);
    }
    virtual void StopApplication() { Simulator::Cancel(m_routeEvent); if(m_recvSocket) m_recvSocket->Close(); }

    // [PBRDV] Store Mobility Data 
    void ReceiveReport(Ptr<Socket> socket) {
        // Failure Simulation: If controller "Down", ignore packet
        if (g_failureStartTime >= 0) {
            double now = Simulator::Now().GetSeconds();
            if (now >= g_failureStartTime && now <= (g_failureStartTime + g_failureDuration)) return;
        }

        Address from; Ptr<Packet> pkt = socket->RecvFrom(from);
        if (pkt->GetSize() == 0) return;
        std::vector<uint8_t> buffer(pkt->GetSize() + 1, 0); pkt->CopyData(buffer.data(), pkt->GetSize());
        std::stringstream ss((char*)buffer.data());
        
        std::string srcCtrlIpStr;
        double x, y, vx, vy;
        
        ss >> srcCtrlIpStr >> x >> y >> vx >> vy;
        
        Ptr<Node> srcNode = FindNodeByCtrlIp(Ipv4Address(srcCtrlIpStr.c_str()));
        if (srcNode) {
            MobilityData md;
            md.position = Vector(x, y, 0);
            md.velocity = Vector(vx, vy, 0);
            md.timestamp = Simulator::Now().GetSeconds();
            m_nodeMobility[srcNode->GetId()] = md;
        }
    }

    // [PBRDV] Prediction Algorithm [cite: 161-170]
    void RecomputeRoutes() {
        // Failure Simulation: Stop computing if Down
        if (g_failureStartTime >= 0) {
            double now = Simulator::Now().GetSeconds();
            if (now >= g_failureStartTime && now <= (g_failureStartTime + g_failureDuration)) {
                 m_routeEvent = Simulator::Schedule(Seconds(ROUTE_RECOMPUTE_INTERVAL), &SdnControllerApp::RecomputeRoutes, this);
                 return;
            }
        }
        
        if (!g_sdnNodes || m_nodeMobility.empty()) {
             m_routeEvent = Simulator::Schedule(Seconds(ROUTE_RECOMPUTE_INTERVAL), &SdnControllerApp::RecomputeRoutes, this);
             return;
        }

        // Structure to hold message for each node: "0 Dst NH 1 Dst NH ..."
        std::map<uint32_t, std::stringstream> batchUpdates;

        // Loop through Future Time Steps [cite: 164]
        for (int t = 0; t <= PREDICTION_HORIZON; t++) {
            
            // 1. Predict Topology for T+t [cite: 166, 167]
            std::map<uint32_t, std::set<uint32_t>> futureTopo;
            
            for (auto const& [srcId, srcMob] : m_nodeMobility) {
                Vector p1 = srcMob.position;
                p1.x += srcMob.velocity.x * (t * PREDICTION_STEP_SIZE);
                p1.y += srcMob.velocity.y * (t * PREDICTION_STEP_SIZE);

                for (auto const& [dstId, dstMob] : m_nodeMobility) {
                    if (srcId == dstId) continue;
                    
                    Vector p2 = dstMob.position;
                    p2.x += dstMob.velocity.x * (t * PREDICTION_STEP_SIZE);
                    p2.y += dstMob.velocity.y * (t * PREDICTION_STEP_SIZE);

                    double dist = CalculateDistance(p1, p2);
                    if (dist <= CONTROLLER_LINK_THRESHOLD) {
                        futureTopo[srcId].insert(dstId);
                    }
                }
            }

            // 2. Calculate Routes (BFS/Dijkstra) for this future slice [cite: 173]
            for (uint32_t i = 0; i < g_sdnNodes->GetN(); ++i) {
                Ptr<Node> node = g_sdnNodes->Get(i);
                uint32_t nodeId = node->GetId();
                if (nodeId == CONTROLLER_NODE_ID) continue;

                // Simple BFS for Dijkstra
                std::map<uint32_t, uint32_t> nextHop, parent;
                std::queue<uint32_t> q; std::set<uint32_t> visited;
                q.push(nodeId); visited.insert(nodeId); parent[nodeId] = nodeId;
                
                while(!q.empty()) {
                    uint32_t u = q.front(); q.pop();
                    for(uint32_t v : futureTopo[u]) {
                        if(visited.find(v) == visited.end()) {
                            visited.insert(v); parent[v] = u; q.push(v);
                        }
                    }
                }
                
                // Reconstruct Paths
                for(uint32_t dst : visited) {
                    if (dst == nodeId) continue;
                    uint32_t curr = dst;
                    while(parent[curr] != nodeId) curr = parent[curr];
                    
                    // Add to Batch: "Time DstIP NextHopIP"
                    Ipv4Address dstIp = GetNodeIpv4Address(g_sdnNodes->Get(dst), 1);
                    Ipv4Address nhIp = GetNodeIpv4Address(g_sdnNodes->Get(curr), 1);
                    batchUpdates[nodeId] << t << " " << dstIp << " " << nhIp << " ";
                }
            }
        }

        // 3. Send Batch Updates 
        for (auto& [nodeId, ss] : batchUpdates) {
            std::string msg = ss.str();
            if (msg.empty()) continue;
            SendRoutePacket(g_sdnNodes->Get(nodeId), msg);
        }

        m_routeEvent = Simulator::Schedule(Seconds(ROUTE_RECOMPUTE_INTERVAL), &SdnControllerApp::RecomputeRoutes, this);
    }

    void SendRoutePacket(Ptr<Node> targetNode, std::string msg) {
        Ptr<Packet> p = Create<Packet>((uint8_t*)msg.c_str(), msg.size());
        Ptr<Socket> s = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        s->Bind(InetSocketAddress(GetNodeIpv4Address(GetNode(), m_ctrlIfIndex), 0));
        s->SendTo(p, 0, InetSocketAddress(GetNodeIpv4Address(targetNode, CTRL_IF_INDEX), VEHICLE_ROUTE_PORT));
        s->Close();
    }

    Ptr<Socket> m_recvSocket;
    EventId m_routeEvent;
    std::map<uint32_t, MobilityData> m_nodeMobility;
    uint32_t m_ctrlIfIndex;
};


// ============================================================================
//                                MAIN
// ============================================================================

int main (int argc, char *argv[])
{
    std::string protocol = "PBRDV"; 
    int speed = 10;
    int runId = 1;
    std::string traceFile = ""; 
    std::string outputDir = "."; 
    double simTime = 200.0;
    uint32_t numNodes = 50; 
    bool enableNetAnim = false;
    
    double failureStart = -1.0; 
    double failureDuration = 0.0;

    CommandLine cmd;
    cmd.AddValue ("protocol", "Protocol (only PBRDV supported in this file)", protocol);
    cmd.AddValue ("speed", "Speed", speed);
    cmd.AddValue ("runId", "Run ID", runId);
    cmd.AddValue ("traceFile", "Trace file", traceFile);
    cmd.AddValue ("outputDir", "Output Directory", outputDir);
    cmd.AddValue ("netanim", "Enable NetAnim", enableNetAnim);
    cmd.AddValue ("failureStart", "Controller failure start time (-1=disabled)", failureStart);
    cmd.AddValue ("failureDuration", "Controller failure duration", failureDuration);
    cmd.Parse (argc, argv);
    
    g_failureStartTime = failureStart;
    g_failureDuration = failureDuration;

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

    NS_LOG_UNCOND("Running PBRDV Mode (Prediction-Based Routing)");
    if (failureStart >= 0) {
        NS_LOG_UNCOND("⚠️  FAILURE SIMULATION: Start=" << failureStart << "s, Dur=" << failureDuration << "s");
        NS_LOG_UNCOND("   Nodes should use PREDICTED routes during this gap.");
    }

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

    // WiFi Setup
    WifiHelper wifiData; wifiData.SetStandard (WIFI_STANDARD_80211p);
    wifiData.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                        "DataMode", StringValue("OfdmRate12MbpsBW10MHz"),
                        "ControlMode", StringValue("OfdmRate6MbpsBW10MHz"));
    Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", UintegerValue(0));
    YansWifiPhyHelper phyData; YansWifiChannelHelper chanData;
    chanData.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
    chanData.AddPropagationLoss ("ns3::FriisPropagationLossModel");
    chanData.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(WIFI_DATA_MAX_RANGE));
    phyData.SetChannel (chanData.Create ());
    phyData.Set("TxPowerStart", DoubleValue(20.0));
    phyData.Set("TxPowerEnd", DoubleValue(20.0));
    phyData.Set("ChannelWidth", UintegerValue(10));
    WifiMacHelper macData; macData.SetType ("ns3::AdhocWifiMac");
    NetDeviceContainer devicesData = wifiData.Install(phyData, macData, vehicles);

    WifiHelper wifiCtrl; wifiCtrl.SetStandard (WIFI_STANDARD_80211a);
    YansWifiPhyHelper phyCtrl; YansWifiChannelHelper chanCtrl;
    chanCtrl.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
    chanCtrl.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(WIFI_CTRL_MAX_RANGE));
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

    for (uint32_t i = 0; i < vehicles.GetN(); ++i) {
        vehicles.Get(i)->GetObject<Ipv4>()->SetAttribute("IpForward", BooleanValue(true));
    }

    Ipv4AddressHelper ipv4Data; ipv4Data.SetBase (DATA_NETWORK_PREFIX, DATA_NETWORK_MASK);
    Ipv4InterfaceContainer ifData = ipv4Data.Assign(devicesData);

    Ipv4AddressHelper ipv4Ctrl; ipv4Ctrl.SetBase (CTRL_NETWORK_PREFIX, CTRL_NETWORK_MASK);
    Ipv4InterfaceContainer ifCtrlVeh = ipv4Ctrl.Assign(devCtrlVeh);
    Ipv4InterfaceContainer ifCtrlNode = ipv4Ctrl.Assign(devCtrlNode);

    Ptr<SdnControllerApp> ctrlApp = CreateObject<SdnControllerApp>();
    ctrlApp->SetCtrlIfIndex(CTRL_NODE_IF_INDEX); 
    controller.Get(0)->AddApplication(ctrlApp);
    ctrlApp->SetStartTime(Seconds(0.1)); ctrlApp->SetStopTime(Seconds(simTime));
    
    Ipv4Address controllerIp = ifCtrlNode.GetAddress(0);
    for (uint32_t i = 0; i < vehicles.GetN(); ++i) {
        Ptr<VehicleSdnApp> app = CreateObject<VehicleSdnApp>();
        app->Setup(controllerIp, DATA_IF_INDEX, CTRL_IF_INDEX); 
        vehicles.Get(i)->AddApplication(app);
        app->SetStartTime(Seconds(VEHICLE_APP_START_OFFSET + (i*VEHICLE_APP_STAGGER))); app->SetStopTime(Seconds(simTime));
    }

    uint16_t port = DATA_APP_PORT;
    UdpEchoServerHelper server (port);
    ApplicationContainer serverApps = server.Install (vehicles.Get (0));
    serverApps.Start (Seconds (SERVER_START_TIME)); serverApps.Stop (Seconds (simTime));

    UdpEchoClientHelper client (ifData.GetAddress (0), port);
    client.SetAttribute ("MaxPackets", UintegerValue (MAX_PACKETS));
    client.SetAttribute ("Interval", TimeValue (Seconds (PACKET_INTERVAL))); 
    client.SetAttribute ("PacketSize", UintegerValue (PACKET_SIZE));
    ApplicationContainer clientApps = client.Install (vehicles.Get (numNodes - 1));
    clientApps.Start (Seconds (CLIENT_START_TIME)); clientApps.Stop (Seconds (simTime));

    FlowMonitorHelper flowmon; Ptr<FlowMonitor> monitor = flowmon.InstallAll ();
    
    if (enableNetAnim) {
        AnimationInterface anim(animFileName);
        anim.SetMaxPktsPerTraceFile(999999999999); 
        anim.EnablePacketMetadata(false); 
        anim.UpdateNodeColor(controller.Get(0), 255, 0, 0);
    }

    Config::ConnectWithoutContext("/NodeList/*/ApplicationList/*/$ns3::UdpEchoClient/Tx", MakeCallback(&MonitorTxCallback));
    Config::ConnectWithoutContext("/NodeList/*/ApplicationList/*/$ns3::UdpEchoServer/Rx", MakeCallback(&MonitorRxCallback));

    std::string csvFileName = outputDir + "/pdr_graph_" + protocol + "_" + std::to_string(speed) + "_" + std::to_string(runId) + ".csv";
    Simulator::Schedule(Seconds(1.0), &RecordPdrPerSecond, csvFileName);

    Simulator::Stop (Seconds (simTime)); Simulator::Run ();
    monitor->CheckForLostPackets (); monitor->SerializeToXmlFile (xmlFileName, true, true);
    Simulator::Destroy ();

    return 0;
}