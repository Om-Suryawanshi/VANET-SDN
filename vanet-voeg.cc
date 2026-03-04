// vanet-voeg.cc
// VoEG (VANET-Oriented Evolving Graph) routing algorithm
// Based on "An Evolving Graph-Based Reliable Routing for VANETs"

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
#include <fstream>
#include <cmath>
#include <algorithm>
#include <limits>
#include <tuple>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("VanetVoeg");

NodeContainer* g_voegNodes = nullptr;

// Global failure simulation parameters
double g_failureStartTime = -1.0;
double g_failureDuration = 0.0;

// ============================================================================
//                      PDR MONITORING & GRAPHING
// ============================================================================

uint64_t g_pdrTxTotal = 0;      // Cumulative total Tx
uint64_t g_pdrRxTotal = 0;      // Cumulative total Rx
uint64_t g_pdrTxLast = 0;       // Last recorded Tx (for per-interval calculation)
uint64_t g_pdrRxLast = 0;       // Last recorded Rx (for per-interval calculation)
uint32_t g_cacheHits = 0;
uint32_t g_cacheMisses = 0;

void MonitorTxCallback(Ptr<const Packet> p) { g_pdrTxTotal++; }
void MonitorRxCallback(Ptr<const Packet> p) { g_pdrRxTotal++; }

void RecordPdrPerSecond(std::string filename) {
    static std::ofstream file;
    static double currentTime = 0;
    if (!file.is_open()) {
        file.open(filename.c_str(), std::ios::out);
        file << "Time,PDR,Tx,Rx,CumulativePDR" << std::endl;
    }
    
    uint64_t intervalTx = g_pdrTxTotal - g_pdrTxLast;
    uint64_t intervalRx = g_pdrRxTotal - g_pdrRxLast;
    
    double intervalPdr = 0.0;
    if (intervalTx > 0) {
        intervalPdr = std::min(100.0, (double)intervalRx / intervalTx * 100.0);
    } else if (intervalRx > 0) {
        intervalPdr = (g_pdrTxTotal > 0) ? std::min(100.0, (double)g_pdrRxTotal / g_pdrTxTotal * 100.0) : 0.0;
    }
    
    double cumulativePdr = 0.0;
    if (g_pdrTxTotal > 0) {
        cumulativePdr = std::min(100.0, (double)g_pdrRxTotal / g_pdrTxTotal * 100.0);
    }
    
    file << currentTime << "," 
         << intervalPdr << "," 
         << intervalTx << "," 
         << intervalRx << ","
         << cumulativePdr << std::endl;
    
    // Update last recorded values
    g_pdrTxLast = g_pdrTxTotal;
    g_pdrRxLast = g_pdrRxTotal;
    
    currentTime += 1.0;
    Simulator::Schedule(Seconds(1.0), &RecordPdrPerSecond, filename);
}


void RecordCacheStatus(std::string filename) {
    static std::ofstream file;
    static double currentTime = 0;
    if (!file.is_open()) {
        file.open(filename.c_str(), std::ios::out);
        file << "Time,CacheHits,CacheMisses" << std::endl;
    }
    file << currentTime << "," << g_cacheHits << "," << g_cacheMisses << std::endl;
    g_cacheHits = 0;
    g_cacheMisses = 0;
    currentTime += 1.0;
    Simulator::Schedule(Seconds(1.0), &RecordCacheStatus, filename);
}

// ============================================================================
//                      CONFIGURATION CONSTANTS
// ============================================================================
// === TIMING PARAMETERS (seconds) ===
const double REPORT_INTERVAL = 0.25;
const double ROUTE_RECOMPUTE_INTERVAL = 0.5; // Old 0.25
const double CONTROLLER_START_TIME = 0.2;
const double JITTER_MULTIPLIER = 0.002;
const double ROUTE_UPDATE_INTERVAL = 0.5;      // How often vehicles update routing table from journey

// === VOEG PARAMETERS ===
const double PREDICTION_HORIZON = 15.0;        // Time slots to predict ahead old 25
const double TIME_SLOT_DURATION = 0.5;         // Seconds per time slot
const double TRANSMISSION_RANGE = 250.0;       // Meters Old 250
const double TRANSMISSION_DELAY = 0.01;        // Seconds per hop (unused: placeholder)
const double MIN_LINK_RELIABILITY = 0.1;       // Threshold for valid links
const int    MSG_PREDICTION_SLOTS = 3;         // Future time slots included in each route message old 3
const int    PREDICTION_WINDOW_BACK  = 3;   // slots allowed behind current
const int    PREDICTION_WINDOW_FRONT = MSG_PREDICTION_SLOTS;
const int    SLOT_TOLERANCE = 2;
// Note: TIME_SLOT_DURATION is used as a divisor; must remain positive.

// === DISTANCE THRESHOLDS (meters) ===
const double WIFI_DATA_MAX_RANGE = 350.0;
const double WIFI_CTRL_MAX_RANGE = 15000.0;    // Entire map should be able to acess the controller

// === NETWORK INTERFACE INDICES ===
const uint32_t DATA_IF_INDEX = 1;
const uint32_t CTRL_IF_INDEX = 2;
const uint32_t CTRL_NODE_IF_INDEX = 1;

// === UDP PORTS ===
const uint16_t CONTROLLER_REPORT_PORT = 9999;
const uint16_t VEHICLE_ROUTE_PORT = 10000;
const uint16_t DATA_APP_PORT = 9;

// === APPLICATION TIMING ===
const double SERVER_START_TIME = 5.0;
const double CLIENT_START_TIME = 10.0;
const double VEHICLE_APP_START_OFFSET = 0.5;
const double VEHICLE_APP_STAGGER = 0.01;

// === SCHEDULING OFFSETS ===
const double CONTROLLER_SEND_OFFSET      = 0.0005;
const double PREDICTION_MESSAGE_OFFSET   = 0.0001; // Ensures P msg is sent after S msg per vehicle

// === SPECIAL NODE IDs ===
// const uint32_t CONTROLLER_NODE_ID = 50;
uint32_t g_controllerId = 0;

// === FAILURE RECOVERY ===
const double CONTROLLER_TIMEOUT = 2.0;
const double HEARTBEAT_CHECK_INTERVAL = 0.5;

// === IP CONFIGURATION ===
const char* DATA_NETWORK_PREFIX = "10.1.0.0";
const char* DATA_NETWORK_MASK   = "255.255.0.0";
const char* CTRL_NETWORK_PREFIX = "10.2.0.0";
const char* CTRL_NETWORK_MASK   = "255.255.0.0";

// === APPLICATION PARAMETERS ===
const uint32_t MAX_PACKETS    = 100000;
const double   PACKET_INTERVAL = 0.02; // Old 0.1
const uint32_t PACKET_SIZE    = 1024;


double g_totalPathLength = 0.0;
uint32_t g_totalPaths = 0;
uint32_t g_totalInstalledRoutes = 0;
uint32_t g_totalRouteSamples = 0;
double g_firstPredictionActivation = -1.0;


// === DEBUG FILE ===
std::ofstream g_routeDebugFile;


// ============================================================================
//                      VOEG DATA STRUCTURES
// ============================================================================

// Snapshot of the network topology at one predicted time slot
struct TimeSlotGraph {
    int timeSlot;
    std::map<uint32_t, std::set<uint32_t>>            adjacency;       // node -> neighbors
    std::map<std::pair<uint32_t,uint32_t>, double>    linkReliability; // (i,j) -> reliability
};

// Per-vehicle mobility state stored at the controller
struct VehicleState {
    uint32_t nodeId;
    Vector   position;
    Vector   velocity;
};

// ============================================================================
//                      LINK RELIABILITY CALCULATION
// ============================================================================

// Compute the fraction of time window [0, TIME_SLOT_DURATION] during which
// the link between vehicles i and j remains connected, given their current
// (predicted) positions and velocities, under a linear mobility model.
//
// The distance at time tau is:
//   d(tau)^2 = |dx + dvx*tau|^2 + |dy + dvy*tau|^2
//            = a*tau^2 + b*tau + c
// where a = |dv|^2, b = 2*(dr . dv), c = |dr|^2.
// The link is connected when d(tau) <= R, i.e. a*tau^2 + b*tau + (c - R^2) <= 0.
//
// Since c = |dr|^2 <= R^2 (precondition: vehicles are already in range),
// the parabola starts non-positive. The positive root t_break is where it
// rises above zero again (link breaks).  Returns reliability in [0, 1].
double ComputeLinkReliability(Vector pos_i, Vector vel_i,
                               Vector pos_j, Vector vel_j) {
    double dx  = pos_i.x - pos_j.x;
    double dy  = pos_i.y - pos_j.y;
    double dvx = vel_i.x - vel_j.x;
    double dvy = vel_i.y - vel_j.y;

    double R_sq  = TRANSMISSION_RANGE * TRANSMISSION_RANGE;
    double d_sq  = dx*dx + dy*dy;

    // Vehicles must currently be in range
    if (d_sq > R_sq) return 0.0;

    double a = dvx*dvx + dvy*dvy;
    double b = 2.0 * (dx*dvx + dy*dvy);
    double c = d_sq - R_sq; // <= 0

    // Zero relative velocity: distance is constant, link stays connected
    if (a < 1e-10) return 1.0;

    double disc = b*b - 4.0*a*c;
    // disc >= 0 always (c <= 0 => -4ac >= 0 => disc = b^2 - 4ac >= 0)
    if (disc < 0.0) return 1.0;

    // Positive root: when link breaks
    double t_break = (-b + std::sqrt(disc)) / (2.0 * a);
    if (t_break <= 0.0) return 0.0;

    double time_connected = std::min(t_break, TIME_SLOT_DURATION);
    return time_connected / TIME_SLOT_DURATION;
}

// ============================================================================
//                      HELPER FUNCTIONS
// ============================================================================

Ipv4Address GetNodeIpv4Address(Ptr<Node> node, uint32_t ifIndex) {
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    if (!ipv4 || ifIndex >= ipv4->GetNInterfaces()) return Ipv4Address::GetZero();
    return ipv4->GetAddress(ifIndex, 0).GetLocal();
}

// Old gpt says not working on highway for node 59
// Ptr<Node> FindNodeByCtrlIp(Ipv4Address ctrlIp) {
//     if (!g_voegNodes) return nullptr;
//     for (uint32_t i = 0; i < g_voegNodes->GetN(); ++i) {
//         Ptr<Ipv4> ipv4 = g_voegNodes->Get(i)->GetObject<Ipv4>();
//         for (uint32_t j = 0; j < ipv4->GetNInterfaces(); ++j) {
//             if (ipv4->GetAddress(j,0).GetLocal() == ctrlIp)
//                 return g_voegNodes->Get(i);
//         }
//     }
//     return nullptr;
// }

Ptr<Node> FindNodeByCtrlIp(Ipv4Address ctrlIp)
{
    // std::cout << "Total nodes in g_voegNodes: "
    //       << g_voegNodes->GetN() << std::endl;

    if (!g_voegNodes) return nullptr;

    for (uint32_t i = 0; i < g_voegNodes->GetN(); ++i)
    {
        Ptr<Node> node = g_voegNodes->Get(i);
        Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
        if (!ipv4) continue;

        for (uint32_t j = 0; j < ipv4->GetNInterfaces(); ++j)
        {
            for (uint32_t k = 0; k < ipv4->GetNAddresses(j); ++k)
            {
                Ipv4Address addr = ipv4->GetAddress(j, k).GetLocal();

                // Only match control subnet
                if (addr.CombineMask("255.255.255.0") ==
                    ctrlIp.CombineMask("255.255.255.0"))
                {
                    if (addr == ctrlIp)
                        return node;
                }
            }
        }
    }

    return nullptr;
}

// ============================================================================
//                      EG-DIJKSTRA ALGORITHM
// ============================================================================
// Modified Dijkstra over an evolving graph.
//
// State: (node_id, time_slot)
// Priority: accumulated reliability (maximised)
//
// From state (u, t), for each time slot ts >= t in the evolving graph,
// for each neighbor v of u at ts:
//   new_rel = rel_u * R_link(u,v,ts)
//   arrival_time = ts + 1
//   If new_rel > bestRel[v][arrival_time]: update and push to queue.
//
// Returns, for each reachable destination, the first-hop map:
//   journeys[dst][timeSlot] = next_hop_id
// where timeSlot is the absolute slot at which src takes that hop.

std::map<uint32_t, std::map<int,uint32_t>> RunEGDijkstra(
    uint32_t src,
    const std::vector<TimeSlotGraph>& eg,
    int startTimeSlot)
{
    using State = std::tuple<double, uint32_t, int>; // (reliability, node, time)

    std::priority_queue<State> pq;
    // bestRel[(node, time)] = best accumulated reliability reaching that state
    std::map<std::pair<uint32_t,int>, double> bestRel;
    // parent[(node, time)] = (prev_node, prev_time)
    std::map<std::pair<uint32_t,int>, std::pair<uint32_t,int>> parent;

    auto key = [](uint32_t n, int t) { return std::make_pair(n, t); };

    bestRel[key(src, startTimeSlot)] = 1.0;
    pq.push(State{1.0, src, startTimeSlot});

    int maxSlot = startTimeSlot + static_cast<int>(eg.size()) - 1;

    while (!pq.empty()) {
        auto [rel, u, t] = pq.top(); pq.pop();

        // Discard stale entries
        auto bIt = bestRel.find(key(u, t));
        if (bIt == bestRel.end() || rel < bIt->second - 1e-12) continue;

        // Explore edges at each future (or current) time slot
        for (int ts = t; ts <= maxSlot; ++ts) {
            int idx = ts - startTimeSlot;
            if (idx < 0 || idx >= static_cast<int>(eg.size())) continue;

            const TimeSlotGraph& g = eg[idx];
            auto adjIt = g.adjacency.find(u);
            if (adjIt == g.adjacency.end()) continue;

            for (uint32_t v : adjIt->second) {
                auto rIt = g.linkReliability.find({u, v});
                if (rIt == g.linkReliability.end()) continue;
                double linkRel = rIt->second;
                if (linkRel < MIN_LINK_RELIABILITY) continue;

                double newRel      = rel * linkRel;
                int    arrivalSlot = ts + 1;
                // States beyond maxSlot have no outgoing edges but are valid endpoints

                auto& best = bestRel[key(v, arrivalSlot)];
                if (newRel > best) {
                    best = newRel;
                    parent[key(v, arrivalSlot)] = {u, ts};
                    pq.push(State{newRel, v, arrivalSlot});
                }
            }
        }
    }

    // Reconstruct journeys: for each reachable dst, trace back to find the
    // first hop taken from src (and at which time slot).
    std::map<uint32_t, std::map<int,uint32_t>> journeys;

    // Collect reachable destinations
    std::set<uint32_t> reachable;
    for (auto& kv : bestRel) {
        if (kv.first.first != src) reachable.insert(kv.first.first);
    }

    for (uint32_t dst : reachable) {
        // Find the time slot where dst is reached with highest reliability
        int    bestArrival = -1;
        double maxRel      = -1.0;
        for (auto& kv : bestRel) {
            if (kv.first.first == dst && kv.second > maxRel) {
                maxRel      = kv.second;
                bestArrival = kv.first.second;
            }
        }
        if (bestArrival < 0 || maxRel < MIN_LINK_RELIABILITY) continue;

        // Walk the parent chain back to src; record first-hop from src
        std::map<int,uint32_t> journey;
        uint32_t curr     = dst;
        int      currTime = bestArrival;
        while (curr != src) {
            auto pIt = parent.find(key(curr, currTime));
            if (pIt == parent.end()) break;
            auto [prevNode, prevTime] = pIt->second;
            if (prevNode == src) {
                // src takes this hop at time prevTime
                journey[prevTime] = curr;
            }
            curr     = prevNode;
            currTime = prevTime;
        }
        if (!journey.empty()) journeys[dst] = journey;
    }

    return journeys;
}

// ============================================================================
//                      VEHICLE SDN APP (VOEG VERSION)
// ============================================================================

class VehicleSdnApp : public Application {
public:
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("ns3::VehicleVoegApp")
            .SetParent<Application>()
            .AddConstructor<VehicleSdnApp>();
        return tid;
    }
    VehicleSdnApp()
        : m_dataIfIndex(DATA_IF_INDEX), m_ctrlIfIndex(CTRL_IF_INDEX),
          m_usingSdnRouting(true), m_enablePredictionCache(true) {}

    void Setup(Ipv4Address controllerIp, uint32_t dataIfIndex,
               uint32_t ctrlIfIndex, bool enableCache = true) {
        m_controllerIp         = controllerIp;
        m_dataIfIndex          = dataIfIndex;
        m_ctrlIfIndex          = ctrlIfIndex;
        m_enablePredictionCache = enableCache;
        m_lastControllerContact = Simulator::Now();
    }

private:
    Ptr<Socket> m_reportSocket;

    virtual void StartApplication() {
        // Reset last-contact time to now so we don't immediately time out
        m_lastControllerContact = Simulator::Now();

        // Control RX: receive route updates from controller
        m_ctrlRx = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_ctrlRx->Bind(InetSocketAddress(
            GetNodeIpv4Address(GetNode(), m_ctrlIfIndex), VEHICLE_ROUTE_PORT));
        m_ctrlRx->SetRecvCallback(MakeCallback(&VehicleSdnApp::ReceiveRoute, this));

        double jitter = (double)(GetNode()->GetId()) * JITTER_MULTIPLIER;
        m_reportEvent = Simulator::Schedule(
            Seconds(REPORT_INTERVAL + jitter), &VehicleSdnApp::SendReport, this);
        m_heartbeatEvent = Simulator::Schedule(
            Seconds(HEARTBEAT_CHECK_INTERVAL), &VehicleSdnApp::CheckControllerConnectivity, this);
        m_routeUpdateEvent = Simulator::Schedule(
            Seconds(ROUTE_UPDATE_INTERVAL + jitter), &VehicleSdnApp::ApplyPredictionRoutes, this);

        // std::cout << "Node " << GetNode()->GetId()
        //   << " DATA IP: "
        //   << GetNodeIpv4Address(GetNode(), 1)
        //   << " CTRL IP: "
        //   << GetNodeIpv4Address(GetNode(), 2)
        //   << std::endl;

        // Temp 
        m_reportSocket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());

        Ipv4Address myCtrlIp = GetNodeIpv4Address(GetNode(), m_ctrlIfIndex);

        m_reportSocket->Bind(InetSocketAddress(myCtrlIp, 0));
        m_reportSocket->Connect(InetSocketAddress(m_controllerIp, CONTROLLER_REPORT_PORT));
    }

    virtual void StopApplication() {
        Simulator::Cancel(m_reportEvent);
        Simulator::Cancel(m_heartbeatEvent);
        Simulator::Cancel(m_routeUpdateEvent);
        if (m_ctrlRx) m_ctrlRx->Close();
    }

    // Send position + velocity report to controller
    void SendReport() {
        Ptr<MobilityModel> mob = GetNode()->GetObject<MobilityModel>();
        Vector pos = mob->GetPosition();
        Vector vel = mob->GetVelocity();
        Ipv4Address myCtrlIp = GetNodeIpv4Address(GetNode(), m_ctrlIfIndex);

        // if(GetNode()->GetId() == 59 || GetNode()->GetId() == 0)
        //     std::cout << "Node " << GetNode()->GetId()
        //         << "Ip :" << myCtrlIp 
        //         << " pos: " << pos.x << "," << pos.y
        //         << " vel: " << vel.x << "," << vel.y
        //         << std::endl;


        std::ostringstream ss;
        ss << myCtrlIp << " " << pos.x << " " << pos.y << " " << vel.x << " " << vel.y;
        std::string data = ss.str();

        Ptr<Packet> pkt = Create<Packet>((uint8_t*)data.c_str(), data.size());
        // Ptr<Socket> s = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        // s->Bind(InetSocketAddress(myCtrlIp, 0));
        // s->Connect(InetSocketAddress(m_controllerIp, CONTROLLER_REPORT_PORT));
        // s->Send(pkt);
        // s->Close();

        // Temp
        m_reportSocket->Send(pkt);

        m_reportEvent = Simulator::Schedule(
            Seconds(REPORT_INTERVAL), &VehicleSdnApp::SendReport, this);
    }

    // Receive route update from controller.
    // 'S' messages: "S DstDataIP NextHopDataIP ..."  (pairs, immediate SDN routes)
    // 'P' messages: "P TimeSlot DstDataIP NextHopDataIP ..."  (triples, prediction cache)
    void ReceiveRoute(Ptr<Socket> socket) {
        Address from;
        Ptr<Packet> pkt = socket->RecvFrom(from);
        if (pkt->GetSize() == 0) return;

        m_lastControllerContact = Simulator::Now();

        std::vector<uint8_t> buffer(pkt->GetSize() + 1, 0);
        pkt->CopyData(buffer.data(), pkt->GetSize());
        std::stringstream ss((char*)buffer.data());

        char msgType;
        if (!(ss >> msgType)) return;

        if (msgType == 'S') {
            // Immediate SDN route update
            if (!m_usingSdnRouting) {
                Simulator::Cancel(m_routeUpdateEvent);
                m_usingSdnRouting = true;
                NS_LOG_UNCOND("[" << Simulator::Now().GetSeconds() << "s] Node "
                              << GetNode()->GetId()
                              << ": Controller reconnected! Switching back to SDN routing");
            }
            std::string dstStr, nhStr;
            while (ss >> dstStr >> nhStr) {
                if (dstStr.find('.') == std::string::npos) continue;
                m_activeRoutes[Ipv4Address(dstStr.c_str())] = Ipv4Address(nhStr.c_str());
            }
            InstallActiveRoutes();
        } else if (msgType == 'P') {
            // Prediction cache update — store but do NOT install while controller is alive
            int timeSlot;
            std::string dstStr, nhStr;

            while (ss >> timeSlot >> dstStr >> nhStr){
                if (dstStr.find('.') == std::string::npos) continue;

                Ipv4Address dst(dstStr.c_str());
                Ipv4Address nh(nhStr.c_str());

                auto& slotMap = m_predictionRoutes[dst];

                slotMap[timeSlot] = nh;

                // ---- Sliding window pruning ----
                double now = Simulator::Now().GetSeconds();
                int currentSlot = static_cast<int>(now / TIME_SLOT_DURATION);

                int minSlot = currentSlot - PREDICTION_WINDOW_BACK;
                int maxSlot = currentSlot + PREDICTION_WINDOW_FRONT;

                for (auto it = slotMap.begin(); it != slotMap.end(); )
                {
                    if (it->first < minSlot || it->first > maxSlot)
                        it = slotMap.erase(it);
                    else
                        ++it;
                }
            }
        }
    }

    // Apply prediction routes to the routing table.
    // Only operates on m_predictionRoutes; only active when controller is down.
    void ApplyPredictionRoutes() {
        if (m_usingSdnRouting || !m_enablePredictionCache) {
            m_routeUpdateEvent = Simulator::Schedule(
                Seconds(ROUTE_UPDATE_INTERVAL), &VehicleSdnApp::ApplyPredictionRoutes, this);
            return;
        }

        double now         = Simulator::Now().GetSeconds();
        int    currentSlot = static_cast<int>(now / TIME_SLOT_DURATION);

        Ptr<Ipv4StaticRouting> staticRouting = GetStaticRouting();
        if (!staticRouting) {
            m_routeUpdateEvent = Simulator::Schedule(
                Seconds(ROUTE_UPDATE_INTERVAL), &VehicleSdnApp::ApplyPredictionRoutes, this);
            return;
        }

        for (auto& [dst, slotMap] : m_predictionRoutes)
        {
            Ipv4Address nh;
            bool found = false;
            auto it = slotMap.lower_bound(currentSlot - SLOT_TOLERANCE);
            while (it != slotMap.end() && it->first <= currentSlot + SLOT_TOLERANCE)
            {
                nh = it->second;
                g_cacheHits++;
                found = true;
                break;
            }

            if (!found)
            {
                auto past = slotMap.upper_bound(currentSlot);

                if (past == slotMap.begin())
                    continue;

                --past;
                nh = past->second;
                g_cacheMisses++;
            }

            // Remove old route
            for (uint32_t i = staticRouting->GetNRoutes(); i > 0; i--)
            {
                Ipv4RoutingTableEntry route = staticRouting->GetRoute(i-1);

                if (route.GetDest() == dst &&
                    route.GetDestNetworkMask() == Ipv4Mask("255.255.255.255") &&
                    route.GetDest().CombineMask(Ipv4Mask("255.255.0.0")) ==
                    Ipv4Address("10.1.0.0"))
                {
                    staticRouting->RemoveRoute(i-1);
                }
            }

            // Install predicted route
            staticRouting->AddHostRouteTo(dst, nh, m_dataIfIndex, 0);
            g_routeDebugFile
                << Simulator::Now().GetSeconds() << ","
                << GetNode()->GetId() << ","
                << dst << ","
                << nh << ",PREDICTION\n";
        }

        m_routeUpdateEvent = Simulator::Schedule(
            Seconds(ROUTE_UPDATE_INTERVAL), &VehicleSdnApp::ApplyPredictionRoutes, this);
    }

    void PurgeSDNRoutes(Ptr<Ipv4StaticRouting> staticRouting) {
        for (uint32_t i = staticRouting->GetNRoutes(); i > 0; i--) {
            Ipv4RoutingTableEntry route = staticRouting->GetRoute(i-1);
            if (route.GetDestNetworkMask() == Ipv4Mask("255.255.255.255") &&
                route.GetDest().CombineMask(Ipv4Mask("255.255.0.0")) == Ipv4Address("10.1.0.0")) {
                staticRouting->RemoveRoute(i-1);
            }
        }
    }

    Ptr<Ipv4StaticRouting> GetStaticRouting() {
        Ptr<Ipv4> ipv4 = GetNode()->GetObject<Ipv4>();
        Ptr<Ipv4ListRouting> lr = DynamicCast<Ipv4ListRouting>(ipv4->GetRoutingProtocol());
        if (!lr) return nullptr;
        int16_t priority;
        for (uint32_t i = 0; i < lr->GetNRoutingProtocols(); i++) {
            Ptr<Ipv4RoutingProtocol> temp = lr->GetRoutingProtocol(i, priority);
            if (DynamicCast<Ipv4StaticRouting>(temp))
                return DynamicCast<Ipv4StaticRouting>(temp);
        }
        return nullptr;
    }

    void InstallActiveRoutes() {
        Ptr<Ipv4StaticRouting> staticRouting = GetStaticRouting();
        if (!staticRouting) return;

        // ------------------------------------------------------------------
        // 1) Remove routes that no longer exist in m_activeRoutes
        // ------------------------------------------------------------------
        for (uint32_t i = staticRouting->GetNRoutes(); i > 0; i--) {
            Ipv4RoutingTableEntry route = staticRouting->GetRoute(i - 1);

            // Only manage host routes in data network (10.1.0.0/16)
            if (route.GetDestNetworkMask() == Ipv4Mask("255.255.255.255") &&
                route.GetDest().CombineMask(Ipv4Mask("255.255.0.0")) ==
                    Ipv4Address("10.1.0.0")) {

                Ipv4Address dst = route.GetDest();

                // If destination no longer present in new route set → remove it
                if (m_activeRoutes.find(dst) == m_activeRoutes.end()) {
                    staticRouting->RemoveRoute(i - 1);
                }
            }
        }

        // ------------------------------------------------------------------
        // 2) Add or update routes that changed
        // ------------------------------------------------------------------
        for (auto& [dst, newNh] : m_activeRoutes) {

            bool routeExists = false;
            bool needsUpdate = false;

            for (uint32_t i = 0; i < staticRouting->GetNRoutes(); i++) {
                Ipv4RoutingTableEntry route = staticRouting->GetRoute(i);

                if (route.GetDest() == dst &&
                    route.GetDestNetworkMask() == Ipv4Mask("255.255.255.255")) {

                    routeExists = true;

                    if (route.GetGateway() != newNh) {
                        // Next-hop changed → remove old route
                        staticRouting->RemoveRoute(i);
                        needsUpdate = true;
                    }
                    break;
                }
            }

            // Add new route if it didn’t exist or was removed
            if (!routeExists || needsUpdate) {
                staticRouting->AddHostRouteTo(dst, newNh, m_dataIfIndex, 0);
                g_routeDebugFile
                    << Simulator::Now().GetSeconds() << ","
                    << GetNode()->GetId() << ","
                    << dst << ","
                    << newNh << ",SDN\n";
            }
        }



        g_totalInstalledRoutes += m_activeRoutes.size();
        g_totalRouteSamples++;
    }

    void CheckControllerConnectivity() {
        double timeSince = (Simulator::Now() - m_lastControllerContact).GetSeconds();
        if (timeSince > CONTROLLER_TIMEOUT && m_usingSdnRouting) {
            NS_LOG_UNCOND("[" << Simulator::Now().GetSeconds() << "s] Node "
                          << GetNode()->GetId()
                          << ": Controller timeout (" << timeSince
                          << "s)! Switching to prediction routing");
            m_usingSdnRouting = false;
            if (g_firstPredictionActivation < 0)
            {
                g_firstPredictionActivation = Simulator::Now().GetSeconds();
            }
            Simulator::Cancel(m_routeUpdateEvent);
            if (m_enablePredictionCache && !m_predictionRoutes.empty()) {
                NS_LOG_UNCOND("  Using VoEG prediction cache for continued routing");
            } else {
                NS_LOG_UNCOND("  Controller down — no prediction routes available");
            }
            ApplyPredictionRoutes();
        }
        m_heartbeatEvent = Simulator::Schedule(
            Seconds(HEARTBEAT_CHECK_INTERVAL),
            &VehicleSdnApp::CheckControllerConnectivity, this);
    }

    Ptr<Socket> m_ctrlRx;
    Ipv4Address m_controllerIp;
    uint32_t    m_dataIfIndex, m_ctrlIfIndex;
    EventId     m_reportEvent, m_heartbeatEvent, m_routeUpdateEvent;
    bool        m_usingSdnRouting;
    bool        m_enablePredictionCache;
    Time        m_lastControllerContact;

    // Active SDN routes (installed while controller is reachable): dst -> next-hop
    std::map<Ipv4Address, Ipv4Address> m_activeRoutes;
    // Prediction cache (time-indexed, only installed when controller is down): dst -> (slot -> next-hop)
    std::map<Ipv4Address, std::map<int,Ipv4Address>> m_predictionRoutes;
};

// ============================================================================
//                      SDN CONTROLLER APP (VOEG VERSION)
// ============================================================================

class SdnControllerApp : public Application {
public:
    static TypeId GetTypeId() {
        static TypeId tid = TypeId("ns3::VoegControllerApp")
            .SetParent<Application>()
            .AddConstructor<SdnControllerApp>();
        return tid;
    }
    SdnControllerApp() : m_ctrlIfIndex(CTRL_NODE_IF_INDEX) {}
    void SetCtrlIfIndex(uint32_t idx) { m_ctrlIfIndex = idx; }
    void SetPredictionEnabled(bool enabled) {
        m_enablePrediction = enabled;
    }

private:
    bool m_enablePrediction = false;
    
    virtual void StartApplication() {
        m_recvSocket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_recvSocket->Bind(InetSocketAddress(
            GetNodeIpv4Address(GetNode(), m_ctrlIfIndex), CONTROLLER_REPORT_PORT));
        m_recvSocket->SetRecvCallback(
            MakeCallback(&SdnControllerApp::ReceiveReport, this));
        m_routeEvent = Simulator::Schedule(
            Seconds(CONTROLLER_START_TIME), &SdnControllerApp::RecomputeRoutes, this); // Seconds(CONTROLLER_START_TIME)
    }

    virtual void StopApplication() {
        Simulator::Cancel(m_routeEvent);
        if (m_recvSocket) m_recvSocket->Close();
    }

    // Receive mobility report: "CtrlIP x y vx vy"
    void ReceiveReport(Ptr<Socket> socket) {
        Address from;
        Ptr<Packet> pkt = socket->RecvFrom(from);

        // Simulate controller being offline
        if (g_failureStartTime >= 0) {
            double now = Simulator::Now().GetSeconds();
            if (now >= g_failureStartTime &&
                now <= (g_failureStartTime + g_failureDuration)) {
                return;
            }
        }

        if (pkt->GetSize() == 0) return;
        std::vector<uint8_t> buffer(pkt->GetSize() + 1, 0);
        pkt->CopyData(buffer.data(), pkt->GetSize());
        std::stringstream ss((char*)buffer.data());

        std::string ctrlIpStr;
        double x, y, vx, vy;
        if (!(ss >> ctrlIpStr >> x >> y >> vx >> vy)) return;

        Ptr<Node> srcNode = FindNodeByCtrlIp(Ipv4Address(ctrlIpStr.c_str()));
        if (!srcNode) return;

        uint32_t nodeId = srcNode->GetId();

        // if (nodeId == 59 || nodeId == 0)
        //     std::cout << "Report from node " << nodeId << std::endl;
        
        m_vehicleStates[nodeId] = { nodeId, Vector(x, y, 0), Vector(vx, vy, 0) };
    }

    // Build VoEG: one TimeSlotGraph per prediction slot.
    // For each slot s, predicted position = pos + vel * (s * TIME_SLOT_DURATION).
    void BuildEvolvingGraph(std::vector<TimeSlotGraph>& eg, int numSlots) {
        eg.clear();
        eg.resize(numSlots);

        double now      = Simulator::Now().GetSeconds();
        int    baseSlot = static_cast<int>(now / TIME_SLOT_DURATION);

        // Collect all node IDs once
        std::vector<uint32_t> ids;
        ids.reserve(m_vehicleStates.size());
        for (uint32_t i = 0; i < g_voegNodes->GetN(); ++i)
        {
            if (i == g_controllerId) continue;
            // Only include nodes that have reported at least once
            if (m_vehicleStates.find(i) != m_vehicleStates.end())
                ids.push_back(i);
        }

        for (int s = 0; s < numSlots; ++s) {
            TimeSlotGraph& g = eg[s];
            g.timeSlot = baseSlot + s;
            double tau = s * TIME_SLOT_DURATION;

            // Predicted positions at time slot s
            std::map<uint32_t, Vector> predPos;
            for (auto& [id, state] : m_vehicleStates) {
                predPos[id] = Vector(
                    state.position.x + state.velocity.x * tau,
                    state.position.y + state.velocity.y * tau,
                    0.0);
            }

            // Build edges: check each pair
            for (size_t i = 0; i < ids.size(); ++i) {
                for (size_t j = i + 1; j < ids.size(); ++j) {
                    uint32_t u = ids[i], v = ids[j];
                    Vector& pi = predPos[u];
                    Vector& pj = predPos[v];
                    double dx   = pi.x - pj.x;
                    double dy   = pi.y - pj.y;
                    double dist = std::sqrt(dx*dx + dy*dy);

                    if (dist <= TRANSMISSION_RANGE) {
                        // Reliability: probability link survives one time slot
                        double rel = ComputeLinkReliability(
                            pi, m_vehicleStates[u].velocity,
                            pj, m_vehicleStates[v].velocity);

                        if (rel >= MIN_LINK_RELIABILITY) {
                            g.adjacency[u].insert(v);
                            g.adjacency[v].insert(u);
                            g.linkReliability[{u,v}] = rel;
                            g.linkReliability[{v,u}] = rel;
                        }
                    }
                }
            }

            // Debug

            if (s == 0) {
                double totalDegree = 0.0;
                uint32_t nodeCount = ids.size();
                uint32_t isolated = 0;

                for (auto& id : ids) {
                    auto it = g.adjacency.find(id);
                    uint32_t deg = (it != g.adjacency.end()) ? it->second.size() : 0;

                    totalDegree += deg;
                    if (deg == 0) isolated++;
                }

                double avgDegree = (nodeCount > 0) ? totalDegree / nodeCount : 0.0;

                std::cout << "[" << Simulator::Now().GetSeconds()
                    << "s] Avg Degree: " << avgDegree
                    << " | Isolated Nodes: " << isolated << std::endl;
            }

        }
    }

    void RecomputeRoutes()
    {
        if (!g_voegNodes) return;
        double now = Simulator::Now().GetSeconds();
        static bool logged = false;
        
        if (g_failureStartTime >= 0 &&
            now >= g_failureStartTime &&
            now <= (g_failureStartTime + g_failureDuration))
        {
            // Controller is offline — skip route computation
            if (!logged)
            {
                logged = true;
                std::cout << "Controller faulure active" << std::endl;
            }
            m_routeEvent = Simulator::Schedule(
                Seconds(ROUTE_RECOMPUTE_INTERVAL),
                &SdnControllerApp::RecomputeRoutes,
                this);
            return;
        }

        if (!m_enablePrediction)
        {
        // --------------------------------------------------------------------
        // 1) Build snapshot graph (current slot only)
        // --------------------------------------------------------------------
        std::vector<TimeSlotGraph> currentGraph;
        BuildEvolvingGraph(currentGraph, 1);   // 1 slot = snapshot

            if (currentGraph.empty())
            {
                m_routeEvent = Simulator::Schedule(
                    Seconds(ROUTE_RECOMPUTE_INTERVAL),
                    &SdnControllerApp::RecomputeRoutes, this);
                return;
            }

            auto& adj = currentGraph[0].adjacency;
        // Debug Block for highway
        // uint32_t lastVehicle = g_voegNodes->GetN() - 2;  // controller is last node

        // if (adj.find(lastVehicle) != adj.end())
        // {
        //     std::cout << "[" << Simulator::Now().GetSeconds()
        //             << "s] Node " << lastVehicle << " neighbors: ";

        //     for (auto v : adj[lastVehicle])
        //         std::cout << v << " ";

        //     std::cout << std::endl;
        // }
        // else
        // {
        //     std::cout << "[" << Simulator::Now().GetSeconds()
        //             << "s] Node " << lastVehicle
        //             << " has NO adjacency entry\n";
        // }

        // --------------------------------------------------------------------
        // 2) For each vehicle compute BFS shortest paths
        // --------------------------------------------------------------------
            for (uint32_t srcId = 0; srcId < g_voegNodes->GetN(); ++srcId)
            {
                if (srcId == g_controllerId) continue;

                if (adj.find(srcId) == adj.end())
                    continue;

                std::map<uint32_t, uint32_t> parent;
                std::map<uint32_t, uint32_t> nextHop;
                std::set<uint32_t> visited;
                std::queue<uint32_t> q;

                q.push(srcId);
                visited.insert(srcId);
                parent[srcId] = srcId;

                while (!q.empty())
                {
                    uint32_t u = q.front();
                    q.pop();

                    for (uint32_t v : adj[u])
                    {
                        if (visited.find(v) == visited.end())
                        {
                            visited.insert(v);
                            parent[v] = u;
                            q.push(v);
                        }
                    }
                }

                // ----------------------------------------------------------------
                // 3) Build next-hop table
                // ----------------------------------------------------------------
                for (uint32_t dst : visited)
                {
                    if (dst == srcId) continue;
                    if (dst == g_controllerId) continue;

                    uint32_t hopCount = 0;
                    uint32_t curr = dst;

                    while (parent[curr] != srcId)
                    {
                        curr = parent[curr];
                        hopCount++;
                    }
                    hopCount++;
                    g_totalPathLength += hopCount;
                    g_totalPaths++;

                    nextHop[dst] = curr;
                }
                // if (srcId == g_voegNodes->GetN() - 2)  // last vehicle (since controller is last node)
                // {
                //     if (nextHop.find(0) == nextHop.end())
                //     {
                //         std::cout << "[" << Simulator::Now().GetSeconds()
                //                 << "s] NO PATH from "
                //                 << srcId << " to 0\n";
                //     }
                // }

                if (nextHop.empty())
                    continue;

            

            // ----------------------------------------------------------------
            // 4) Build route message
            // ----------------------------------------------------------------
            std::ostringstream ss;
            ss << "S ";

            for (auto& [dstId, nhId] : nextHop)
            {
                Ipv4Address dstIp =
                    GetNodeIpv4Address(g_voegNodes->Get(dstId), DATA_IF_INDEX);

                Ipv4Address nhIp =
                    GetNodeIpv4Address(g_voegNodes->Get(nhId), DATA_IF_INDEX);

                ss << dstIp << " " << nhIp << " ";
            }

            std::string msg = ss.str();
            if (ss.str().size() <= 2)
                continue;

            Ptr<Node> targetNode = g_voegNodes->Get(srcId);

            Simulator::Schedule(
                Seconds(srcId * CONTROLLER_SEND_OFFSET),
                &SdnControllerApp::SendRoutePacket,
                this,
                targetNode,
                msg);
        }
    }
    else
    {
              // ---------------------------------------
            // FULL VOEG: Multi-slot + EG-Dijkstra
            // ---------------------------------------
            int numSlots = static_cast<int>(PREDICTION_HORIZON / TIME_SLOT_DURATION);

            std::vector<TimeSlotGraph> eg;
            BuildEvolvingGraph(eg, numSlots);

            int baseSlot = static_cast<int>(
                Simulator::Now().GetSeconds() / TIME_SLOT_DURATION);

            for (uint32_t srcId = 0; srcId < g_voegNodes->GetN(); ++srcId)
            {
                if (srcId == g_controllerId) continue;

                auto journeys = RunEGDijkstra(srcId, eg, baseSlot);

                if (journeys.empty()) continue;

                std::ostringstream sMsg;
                std::ostringstream pMsg;

                sMsg << "S ";
                pMsg << "P ";

                for (auto& [dstId, slotMap] : journeys)
                {
                    if (slotMap.empty()) continue;

                    Ipv4Address dstIp =
                        GetNodeIpv4Address(g_voegNodes->Get(dstId), DATA_IF_INDEX);

                    // ---- S message = current-slot (or nearest future) hop ----
                    auto itS = slotMap.lower_bound(baseSlot);
                    if (itS == slotMap.end()) itS = slotMap.begin();
                    uint32_t firstHopId = itS->second;

                    Ipv4Address firstHopIp =
                        GetNodeIpv4Address(g_voegNodes->Get(firstHopId), DATA_IF_INDEX);

                    sMsg << dstIp << " " << firstHopIp << " ";

                    // ---- P message = all future hops ----
                    for (auto& [slot, nextHopId] : slotMap)
                    {
                        Ipv4Address nhIp =
                            GetNodeIpv4Address(g_voegNodes->Get(nextHopId), DATA_IF_INDEX);

                        pMsg << slot << " "
                            << dstIp << " "
                            << nhIp << " ";
                    }
                    g_totalPathLength += slotMap.size();
                    g_totalPaths++;
                }
                Ptr<Node> targetNode = g_voegNodes->Get(srcId);

                if (sMsg.str().size() > 2)
                {
                    Simulator::Schedule(
                        Seconds(srcId * CONTROLLER_SEND_OFFSET),
                        &SdnControllerApp::SendRoutePacket,
                        this,
                        targetNode,
                        sMsg.str());
                }

                if (pMsg.str().size() > 2)
                {
                    Simulator::Schedule(
                        Seconds(srcId * CONTROLLER_SEND_OFFSET + PREDICTION_MESSAGE_OFFSET),
                        &SdnControllerApp::SendRoutePacket,
                        this,
                        targetNode,
                        pMsg.str());
                }
            }
        }
        m_routeEvent = Simulator::Schedule(
            Seconds(ROUTE_RECOMPUTE_INTERVAL),
            &SdnControllerApp::RecomputeRoutes, this);
    }

    void SendRoutePacket(Ptr<Node> targetNode, std::string msg) {
        Ptr<Packet> p = Create<Packet>((uint8_t*)msg.c_str(), msg.size());
        Ptr<Socket> s = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        s->Bind(InetSocketAddress(
            GetNodeIpv4Address(GetNode(), m_ctrlIfIndex), 0));
        s->SendTo(p, 0, InetSocketAddress(
            GetNodeIpv4Address(targetNode, CTRL_IF_INDEX), VEHICLE_ROUTE_PORT));
        s->Close();
    }

    Ptr<Socket>                       m_recvSocket;
    EventId                           m_routeEvent;
    std::map<uint32_t, VehicleState>  m_vehicleStates;
    uint32_t                          m_ctrlIfIndex;
};

// ============================================================================
//                                MAIN
// ============================================================================

int main(int argc, char* argv[])
{
    std::string protocol              = "VOEG";
    std::string scenario              = "grid";
    int         speed                 = 10;
    int         runId                 = 1;
    std::string traceFile             = "";
    std::string outputDir             = ".";
    double      simTime               = 150.0;
    uint32_t    numNodes              = 40;
    bool        enableNetAnim         = false;
    bool        enableFailure         = false;
    bool        enablePredictionCache = false;
    double      failureStart          = -1.0;
    double      failureDuration       = 0.0;

    CommandLine cmd;
    cmd.AddValue("protocol",              "Protocol (VOEG)",                           protocol);
    cmd.AddValue("scenario",              "Simulation scenario (grid/highway/city)",   scenario);
    cmd.AddValue("speed",                 "Vehicle speed for mobility file selection", speed);
    cmd.AddValue("runId",                 "Simulation run ID",                         runId);
    cmd.AddValue("traceFile",             "Path to mobility trace file",               traceFile);
    cmd.AddValue("outputDir",             "Output directory for results",              outputDir);
    cmd.AddValue("netanim",               "Enable NetAnim output",                     enableNetAnim);
    cmd.AddValue("enableFailure",         "Enable controller failure simulation",      enableFailure);
    cmd.AddValue("enablePredictionCache", "Enable local route caching",                enablePredictionCache);
    cmd.AddValue("failureStart",          "Controller failure start time (-1=disabled)", failureStart);
    cmd.AddValue("failureDuration",       "Controller failure duration in seconds",    failureDuration);
    cmd.Parse(argc, argv);

    // If failure flag is set but no time specified, use defaults
    if (enableFailure && failureStart < 0) {
        failureStart    = 50.0;
        failureDuration = 20.0;
    }

    g_failureStartTime = failureStart;
    g_failureDuration  = failureDuration;

    if (traceFile.empty()) {
        traceFile = "scratch/mobility/" + scenario + "/mobility_" + std::to_string(speed) + ".tcl";
    }

    std::string xmlFileName        = outputDir + "/result_VOEG_"      + scenario + "_" + std::to_string(speed) + "_" + std::to_string(runId) + ".xml";
    std::string csvFileName        = outputDir + "/pdr_graph_VOEG_"   + scenario + "_" + std::to_string(speed) + "_" + std::to_string(runId) + ".csv";
    std::string cachFileName       = outputDir + "/cache_status_VOEG_"+ scenario + "_" + std::to_string(speed) + "_" + std::to_string(runId) + ".csv";
    std::string animFileName       = outputDir + "/netanim_VOEG_"     + scenario + "_" + std::to_string(speed) + "_" + std::to_string(runId) + ".xml";
    std::string routeDebugFileName = outputDir + "/route_debug_VOEG_" + scenario + "_" + std::to_string(speed) + "_" + std::to_string(runId) + ".csv";

    g_routeDebugFile.open(routeDebugFileName);
    g_routeDebugFile << "Time,Node,Destination,NextHop,Source\n";

    RngSeedManager::SetSeed(3 + runId);
    RngSeedManager::SetRun(runId);

    NS_LOG_UNCOND("Running VOEG Mode (VoEG - VANET-Oriented Evolving Graph Routing)");
    if (failureStart >= 0) {
        NS_LOG_UNCOND("  Controller failure: " << failureStart << "s - "
                      << (failureStart + failureDuration) << "s");
        NS_LOG_UNCOND("  Prediction cache:   "
                      << (enablePredictionCache ? "ENABLED" : "DISABLED"));
    } else {
        NS_LOG_UNCOND("  Normal operation - no controller failure simulation");
    }

    // --------------------------------------------------------------------------
    //  Node setup
    // --------------------------------------------------------------------------
    NodeContainer vehicles;  vehicles.Create(numNodes);
    NodeContainer controller; controller.Create(1);
    NodeContainer allNodes = NodeContainer(vehicles, controller);
    g_voegNodes = &allNodes;

    g_controllerId = controller.Get(0)->GetId();
    NS_LOG_UNCOND("Controller ID: " << g_controllerId);


    // Vehicle mobility from trace file
    Ns2MobilityHelper ns2(traceFile);
    ns2.Install(vehicles.Begin(), vehicles.End());

    // Controller at fixed position
    MobilityHelper mobilityCtrl;
    mobilityCtrl.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityCtrl.Install(controller);
    controller.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(500, 500, 0));

    // --------------------------------------------------------------------------
    //  WiFi — data channel (802.11p, 10 MHz, TRANSMISSION_RANGE-limited)
    // --------------------------------------------------------------------------
    WifiHelper wifiData; wifiData.SetStandard(WIFI_STANDARD_80211p);
    wifiData.SetRemoteStationManager("ns3::ConstantRateWifiManager",
        "DataMode",    StringValue("OfdmRate12MbpsBW10MHz"),
        "ControlMode", StringValue("OfdmRate6MbpsBW10MHz"));
    Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", UintegerValue(0));
    YansWifiPhyHelper  phyData;
    YansWifiChannelHelper chanData;
    chanData.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    chanData.AddPropagationLoss("ns3::FriisPropagationLossModel");
    chanData.AddPropagationLoss("ns3::RangePropagationLossModel",
        "MaxRange", DoubleValue(WIFI_DATA_MAX_RANGE));
    phyData.SetChannel(chanData.Create());
    phyData.Set("TxPowerStart", DoubleValue(20.0));
    phyData.Set("TxPowerEnd",   DoubleValue(20.0));
    phyData.Set("ChannelWidth", UintegerValue(10));
    WifiMacHelper macData; macData.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer devicesData = wifiData.Install(phyData, macData, vehicles);

    // --------------------------------------------------------------------------
    //  WiFi — control channel (802.11a, long range for vehicle↔controller)
    // --------------------------------------------------------------------------
    WifiHelper wifiCtrl; wifiCtrl.SetStandard(WIFI_STANDARD_80211a);
    YansWifiPhyHelper  phyCtrl;
    YansWifiChannelHelper chanCtrl;
    chanCtrl.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    chanCtrl.AddPropagationLoss("ns3::RangePropagationLossModel",
        "MaxRange", DoubleValue(WIFI_CTRL_MAX_RANGE));
    phyCtrl.SetChannel(chanCtrl.Create());
    WifiMacHelper macCtrl; macCtrl.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer devCtrlVeh  = wifiCtrl.Install(phyCtrl, macCtrl, vehicles);
    NetDeviceContainer devCtrlNode = wifiCtrl.Install(phyCtrl, macCtrl, controller);

    // --------------------------------------------------------------------------
    //  Routing: Static (SDN routes, priority 100)
    // --------------------------------------------------------------------------
    InternetStackHelper     stack;
    Ipv4ListRoutingHelper   list;
    Ipv4StaticRoutingHelper staticRouting;
    list.Add(staticRouting, 100);
    stack.SetRoutingHelper(list);
    stack.Install(allNodes);

    // Enable IP forwarding on vehicles
    for (uint32_t i = 0; i < vehicles.GetN(); ++i)
        vehicles.Get(i)->GetObject<Ipv4>()->SetAttribute("IpForward", BooleanValue(true));

    // --------------------------------------------------------------------------
    //  IP addressing
    // --------------------------------------------------------------------------
    Ipv4AddressHelper ipv4Data; ipv4Data.SetBase(DATA_NETWORK_PREFIX, DATA_NETWORK_MASK);
    Ipv4InterfaceContainer ifData = ipv4Data.Assign(devicesData);

    Ipv4AddressHelper ipv4Ctrl; ipv4Ctrl.SetBase(CTRL_NETWORK_PREFIX, CTRL_NETWORK_MASK);
    ipv4Ctrl.Assign(devCtrlVeh);   // assign addresses to vehicle control interfaces
    Ipv4InterfaceContainer ifCtrlNode = ipv4Ctrl.Assign(devCtrlNode);

    // --------------------------------------------------------------------------
    //  Applications
    // --------------------------------------------------------------------------
    // Controller app
    Ptr<SdnControllerApp> ctrlApp = CreateObject<SdnControllerApp>();
    ctrlApp->SetCtrlIfIndex(CTRL_NODE_IF_INDEX);
    ctrlApp->SetPredictionEnabled(enablePredictionCache);
    controller.Get(0)->AddApplication(ctrlApp);
    ctrlApp->SetStartTime(Seconds(0.1));
    ctrlApp->SetStopTime(Seconds(simTime));


    // Vehicle apps
    Ipv4Address controllerIp = ifCtrlNode.GetAddress(0);
    for (uint32_t i = 0; i < vehicles.GetN(); ++i) {
        Ptr<VehicleSdnApp> app = CreateObject<VehicleSdnApp>();
        app->Setup(controllerIp, DATA_IF_INDEX, CTRL_IF_INDEX, enablePredictionCache);
        vehicles.Get(i)->AddApplication(app);
        app->SetStartTime(Seconds(VEHICLE_APP_START_OFFSET + i * VEHICLE_APP_STAGGER));
        app->SetStopTime(Seconds(simTime));
    }

    // Data application: echo client (vehicle N-1) → echo server (vehicle 0)
    UdpEchoServerHelper server(DATA_APP_PORT);
    ApplicationContainer serverApps = server.Install(vehicles.Get(0));
    serverApps.Start(Seconds(SERVER_START_TIME));
    serverApps.Stop(Seconds(simTime));

    UdpEchoClientHelper client(ifData.GetAddress(0), DATA_APP_PORT);
    client.SetAttribute("MaxPackets", UintegerValue(MAX_PACKETS));
    client.SetAttribute("Interval",   TimeValue(Seconds(PACKET_INTERVAL)));
    client.SetAttribute("PacketSize", UintegerValue(PACKET_SIZE));
    ApplicationContainer clientApps = client.Install(vehicles.Get(numNodes - 1));
    clientApps.Start(Seconds(CLIENT_START_TIME));
    clientApps.Stop(Seconds(simTime));

    // --------------------------------------------------------------------------
    //  FlowMonitor
    // --------------------------------------------------------------------------
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    // --------------------------------------------------------------------------
    //  NetAnim (optional)
    // --------------------------------------------------------------------------
    if (enableNetAnim) {
        AnimationInterface anim(animFileName);
        anim.SetMaxPktsPerTraceFile(std::numeric_limits<uint64_t>::max());
        anim.EnablePacketMetadata(false);
        for (uint32_t i = 0; i < vehicles.GetN(); ++i)
            anim.UpdateNodeColor(vehicles.Get(i), 0, 128, 255);
        anim.UpdateNodeColor(controller.Get(0), 255, 0, 0);
    }

    // --------------------------------------------------------------------------
    //  PDR monitoring callbacks & CSV scheduling
    // --------------------------------------------------------------------------
    Config::ConnectWithoutContext(
        "/NodeList/*/ApplicationList/*/$ns3::UdpEchoClient/Tx",
        MakeCallback(&MonitorTxCallback));
    Config::ConnectWithoutContext(
        "/NodeList/*/ApplicationList/*/$ns3::UdpEchoServer/Rx",
        MakeCallback(&MonitorRxCallback));

    Simulator::Schedule(Seconds(1.0), &RecordPdrPerSecond, csvFileName);
    Simulator::Schedule(Seconds(1.0), &RecordCacheStatus,  cachFileName);

    // --------------------------------------------------------------------------
    //  Run simulation
    // --------------------------------------------------------------------------
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    monitor->CheckForLostPackets();
    monitor->SerializeToXmlFile(xmlFileName, true, true);
    
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();
    
    // Updated Output: PDR, Throughput, and Delay
    std::ofstream finalStats(outputDir + "/final_stats_VOEG_" + scenario + "_" + std::to_string(speed) + "_" + std::to_string(runId) + ".txt");
    
    for (auto const& [flowId, flowStats] : stats) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flowId);
        
        // Only count data flows (port 9)
        if (t.destinationPort == DATA_APP_PORT) {
            
            // 1. Packet Delivery Ratio (PDR)
            double pdr = (flowStats.txPackets > 0) ? 
                         std::min(100.0, 100.0 * flowStats.rxPackets / flowStats.txPackets) : 0.0;
            
            // 2. Throughput (in Kbps)
            double throughput = 0.0;
            double duration = (flowStats.timeLastRxPacket - flowStats.timeFirstTxPacket).GetSeconds();
            if (duration > 0 && flowStats.rxPackets > 0) {
                // rxBytes * 8 bits / (duration in seconds * 1000) = Kbps
                throughput = (flowStats.rxBytes * 8.0) / (duration * 1000.0);
            }
            // 3. Average End-to-End Delay (in ms)
            double delay = 0.0;
            if (flowStats.rxPackets > 0) {
                delay = (flowStats.delaySum.GetSeconds() / flowStats.rxPackets) * 1000.0;
            }
            finalStats << "Flow ID     : " << flowId << "\n"
                       << "Connection  : " << t.sourceAddress << " -> " << t.destinationAddress << "\n"
                       << "Tx Packets  : " << flowStats.txPackets << "\n"
                       << "Rx Packets  : " << flowStats.rxPackets << "\n"
                       << "PDR         : " << pdr << " %\n"
                       << "Throughput  : " << throughput << " kbps\n"
                       << "Avg Delay   : " << delay << " ms\n"
                       << "------------------------------------------------\n";
            finalStats << "\n===== VOEG ADDITIONAL METRICS =====\n";

            // ---- Average Path Length ----
            double avgPathLen = (g_totalPaths > 0) ?
                g_totalPathLength / g_totalPaths : 0.0;

            finalStats << "Average Path Length (hops): "
                    << avgPathLen << "\n";

            // ---- Avg SDN Routes Per Node ----
            double avgRoutes = (g_totalRouteSamples > 0) ?
                (double)g_totalInstalledRoutes / g_totalRouteSamples : 0.0;

            finalStats << "Average SDN Routes Installed per Node: "
                    << avgRoutes << "\n";

            // ---- Failure → Prediction Delay ----
            if (g_failureStartTime >= 0 && g_firstPredictionActivation > 0)
            {
                double activationDelay = 0;
                if (g_firstPredictionActivation >= g_failureStartTime)
                {
                    activationDelay = g_firstPredictionActivation - g_failureStartTime;
                }

                finalStats << "Failure to Prediction Activation Delay: "
                        << activationDelay << " s\n";
            }
            else
            {
                finalStats << "Failure to Prediction Activation Delay: N/A\n";
            }
        }
    }
    finalStats.close();
    Simulator::Destroy();
    return 0;
}
