// vanet-voeg.cc
// VANET-Oriented Evolving Graph (VoEG) Reliable Routing for NS-3.37
// Based on: "An Evolving Graph-Based Reliable Routing Scheme for VANETs"
//
// Usage:
//   ./ns3 run "scratch/vanet-voeg --protocol=VoEG --speed=15 --runId=1"
//   ./ns3 run "scratch/vanet-voeg --speed=15 --failureStart=60 --failureDuration=20"
//   ./ns3 run "scratch/vanet-voeg --speed=15 --enablePredictionCache=false"

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/olsr-module.h"

#include <vector>
#include <map>
#include <set>
#include <queue>
#include <string>
#include <sstream>
#include <fstream>
#include <cmath>
#include <limits>
#include <algorithm>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("VanetVoEG");

// ============================================================================
//                         GLOBAL STATE
// ============================================================================

NodeContainer* g_voegNodes = nullptr;

double g_failureStartTime = -1.0;
double g_failureDuration  = 0.0;

// PDR counters (reset each second)
uint32_t g_pdrTxCount = 0;
uint32_t g_pdrRxCount = 0;

void MonitorTxCallback(Ptr<const Packet> /*p*/) { g_pdrTxCount++; }
void MonitorRxCallback(Ptr<const Packet> /*p*/) { g_pdrRxCount++; }

void RecordPdrPerSecond(std::string filename)
{
    static std::ofstream file;
    static double currentTime = 0;

    if (!file.is_open()) {
        file.open(filename.c_str(), std::ios::out);
        file << "Time,PDR,Tx,Rx\n";
    }

    double pdr = (g_pdrTxCount > 0)
                     ? (double)g_pdrRxCount / g_pdrTxCount * 100.0
                     : 0.0;
    file << currentTime << "," << pdr << "," << g_pdrTxCount << "," << g_pdrRxCount << "\n";

    g_pdrTxCount = 0;
    g_pdrRxCount = 0;
    currentTime += 1.0;

    Simulator::Schedule(Seconds(1.0), &RecordPdrPerSecond, filename);
}

// ============================================================================
//                      CONFIGURATION CONSTANTS
// ============================================================================

// Evolving-Graph parameters
const int    PREDICTION_HORIZON     = 25;    // Predict 25 seconds into future
const double TIME_SLOT_DURATION     = 1.0;   // 1-second time slots
const double LINK_LATENCY           = 0.01;  // 10 ms per-hop transmission delay

// Reliability thresholds
const double MIN_LINK_RELIABILITY   = 0.5;   // Ignore links below 50% reliability
const double TRANSMISSION_RANGE     = 250.0; // WiFi transmission range (m)

// SDN timing (seconds)
const double REPORT_INTERVAL        = 1.0;
const double ROUTE_RECOMPUTE_INTERVAL = 2.0;
const double CONTROLLER_START_TIME  = 1.0;
const double JITTER_MULTIPLIER      = 0.002; // Per-node jitter to avoid sync

// WiFi range limits
const double WIFI_DATA_MAX_RANGE    = 350.0;
const double WIFI_CTRL_MAX_RANGE    = 2000.0;

// Network interface indices
const uint32_t DATA_IF_INDEX        = 1;
const uint32_t CTRL_IF_INDEX        = 2;
const uint32_t CTRL_NODE_IF_INDEX   = 1;

// UDP ports
const uint16_t MOBILITY_REPORT_PORT = 9999;
const uint16_t VEHICLE_ROUTE_PORT   = 10000;
const uint16_t DATA_APP_PORT        = 9;

// Application timing
const double SERVER_START_TIME      = 5.0;
const double CLIENT_START_TIME      = 10.0;
const double VEHICLE_APP_START_OFFSET = 0.5;
const double VEHICLE_APP_STAGGER    = 0.01;

// Route distribution stagger per node (avoids burst)
const double CONTROLLER_SEND_OFFSET = 0.001;

// Special node
const uint32_t CONTROLLER_NODE_ID   = 50;

// Failure-recovery timing
const double CONTROLLER_TIMEOUT         = 4.0;
const double HEARTBEAT_CHECK_INTERVAL   = 0.5;

// IP configuration
const char* DATA_NETWORK_PREFIX = "10.1.0.0";
const char* DATA_NETWORK_MASK   = "255.255.0.0";
const char* CTRL_NETWORK_PREFIX = "10.2.0.0";
const char* CTRL_NETWORK_MASK   = "255.255.0.0";

// Traffic generation
const uint32_t MAX_PACKETS     = 100000;
const double   PACKET_INTERVAL = 0.1;
const uint32_t PACKET_SIZE     = 1024;

// ============================================================================
//                        VOEG DATA STRUCTURES
// ============================================================================

struct MobilityData {
    Vector position;
    Vector velocity;
    double timestamp;
};

// One time-indexed snapshot of the network graph
struct TimeSlot {
    double start_time;
    double end_time;
    std::map<uint32_t, std::set<uint32_t>> adjacency;
    std::map<std::pair<uint32_t, uint32_t>, double> link_reliability; // (min,max) -> R
};

struct EvolvingGraph {
    std::vector<TimeSlot> time_slots;
    uint32_t num_nodes;
    double   slot_duration;
    int      prediction_horizon;
};

struct RouteInfo {
    uint32_t next_hop;
    double   reliability;
};

// ============================================================================
//              LINK RELIABILITY MODEL (Vehicle Kinematics)
// ============================================================================
//
// Two vehicles i,j with relative displacement Δr and relative velocity Δv.
// Squared distance at time τ: d²(τ) = |Δr + Δv·τ|² = A·τ² + B·τ + C
//   A = |Δv|²,  B = 2·(Δr·Δv),  C = |Δr|²
//
// Link is valid while d(τ) ≤ R, i.e. while d²(τ) ≤ R².
// Reliability = fraction of [0, required_duration] for which link holds.

double CalculateLinkReliability(Vector pos_i, Vector vel_i,
                                Vector pos_j, Vector vel_j,
                                double transmission_range,
                                double required_duration)
{
    double dx  = pos_i.x - pos_j.x;
    double dy  = pos_i.y - pos_j.y;
    double dvx = vel_i.x - vel_j.x;
    double dvy = vel_i.y - vel_j.y;

    double d_sq = dx * dx + dy * dy;
    double R_sq = transmission_range * transmission_range;

    // Link is currently out of range
    if (d_sq > R_sq) return 0.0;

    double A = dvx * dvx + dvy * dvy;
    double B = 2.0 * (dx * dvx + dy * dvy);
    double C = d_sq - R_sq; // C ≤ 0 (currently in range)

    // No relative motion → link persists indefinitely
    if (A < 1e-10) return 1.0;

    // Solve A·τ² + B·τ + C = 0 for the exit time (positive root)
    double disc = B * B - 4.0 * A * C;
    if (disc < 0.0) return 1.0; // No real root; should not happen since C≤0

    double sqrt_disc  = std::sqrt(disc);
    double t_exit = (-B + sqrt_disc) / (2.0 * A); // Larger root = link-break time

    if (t_exit <= 0.0) return 0.0; // Link already broken

    // Clamp: if link lasts at least required_duration → R = 1.0
    if (t_exit >= required_duration) return 1.0;

    return t_exit / required_duration;
}

// ============================================================================
//       PER-SLOT RELIABILITY-WEIGHTED DIJKSTRA (Most Reliable Journey)
// ============================================================================
//
// Edge weight = −log(reliability) so that product → sum and we can use a
// standard min-heap Dijkstra.  Priority queue entry: (neg_log_rel, node).

std::map<uint32_t, RouteInfo> ComputeRoutesForSlot(uint32_t source,
                                                    const TimeSlot& slot)
{
    // best accumulated neg-log-reliability to reach each node
    std::map<uint32_t, double>   bestNLR;
    std::map<uint32_t, uint32_t> parent;

    using Entry = std::pair<double, uint32_t>; // (nlr, node)
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;

    bestNLR[source] = 0.0;
    parent[source]  = source;
    pq.push({0.0, source});

    while (!pq.empty()) {
        auto [nlr, u] = pq.top();
        pq.pop();

        auto bit = bestNLR.find(u);
        if (bit != bestNLR.end() && bit->second < nlr - 1e-10) continue;

        auto adjIt = slot.adjacency.find(u);
        if (adjIt == slot.adjacency.end()) continue;

        for (uint32_t v : adjIt->second) {
            auto key   = std::make_pair(std::min(u, v), std::max(u, v));
            auto relIt = slot.link_reliability.find(key);
            if (relIt == slot.link_reliability.end()) continue;

            double link_rel = relIt->second;
            if (link_rel < MIN_LINK_RELIABILITY) continue;

            double new_nlr = nlr + (-std::log(link_rel));

            auto vbit = bestNLR.find(v);
            if (vbit == bestNLR.end() || new_nlr < vbit->second - 1e-10) {
                bestNLR[v] = new_nlr;
                parent[v]  = u;
                pq.push({new_nlr, v});
            }
        }
    }

    // Build result: trace parent chain to find first hop from source
    std::map<uint32_t, RouteInfo> result;
    for (const auto& [dst, nlr] : bestNLR) {
        if (dst == source) continue;

        // Trace parent chain back to find the first hop from source.
        // Guard with a step limit to avoid any cycle in the unlikely case of
        // a malformed parent map.
        uint32_t curr = dst;
        uint32_t steps = 0;
        uint32_t maxSteps = (uint32_t)slot.adjacency.size() + 1;
        while (parent.count(curr) && parent.at(curr) != source && steps < maxSteps) {
            curr = parent.at(curr);
            ++steps;
        }
        if (steps == maxSteps) {
            NS_LOG_WARN("ComputeRoutesForSlot: parent-chain traversal limit reached for dst="
                        << dst << " src=" << source << "; skipping route");
            continue;
        }

        RouteInfo ri;
        ri.next_hop   = curr;
        ri.reliability = std::exp(-nlr);
        result[dst]   = ri;
    }
    return result;
}

// ============================================================================
//                          HELPER FUNCTIONS
// ============================================================================

Ipv4Address GetNodeIpv4Address(Ptr<Node> node, uint32_t ifIndex)
{
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    if (!ipv4 || ifIndex >= ipv4->GetNInterfaces())
        return Ipv4Address::GetZero();
    return ipv4->GetAddress(ifIndex, 0).GetLocal();
}

Ptr<Node> FindNodeByCtrlIp(Ipv4Address ctrlIp)
{
    if (!g_voegNodes) return nullptr;
    for (uint32_t i = 0; i < g_voegNodes->GetN(); ++i) {
        Ptr<Ipv4> ipv4 = g_voegNodes->Get(i)->GetObject<Ipv4>();
        for (uint32_t j = 0; j < ipv4->GetNInterfaces(); ++j) {
            if (ipv4->GetAddress(j, 0).GetLocal() == ctrlIp)
                return g_voegNodes->Get(i);
        }
    }
    return nullptr;
}

// ============================================================================
//                      VOEG VEHICLE APPLICATION
// ============================================================================
// Responsibilities:
//   • Sends mobility reports (position + velocity) to the controller every second.
//   • Receives pre-computed route tables from controller, indexed by time offset.
//   • Applies the route table for the current second.
//   • During controller failure, keeps using the last cached pre-distributed routes.

class VoegVehicleApp : public Application
{
public:
    static TypeId GetTypeId()
    {
        static TypeId tid =
            TypeId("ns3::VoegVehicleApp")
                .SetParent<Application>()
                .AddConstructor<VoegVehicleApp>();
        return tid;
    }

    VoegVehicleApp()
        : m_dataIfIndex(DATA_IF_INDEX),
          m_ctrlIfIndex(CTRL_IF_INDEX),
          m_usingSdnRouting(true)
    {
    }

    void Setup(Ipv4Address controllerIp, uint32_t dataIfIndex, uint32_t ctrlIfIndex)
    {
        m_controllerIp  = controllerIp;
        m_dataIfIndex   = dataIfIndex;
        m_ctrlIfIndex   = ctrlIfIndex;
        m_lastControllerContact = Simulator::Now();
    }

private:
    void StartApplication() override
    {
        // Socket for sending mobility reports to controller
        m_reportTx = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_reportTx->Bind(
            InetSocketAddress(GetNodeIpv4Address(GetNode(), m_ctrlIfIndex), 0));

        // Socket for receiving route updates from controller
        m_routeRx = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_routeRx->Bind(
            InetSocketAddress(GetNodeIpv4Address(GetNode(), m_ctrlIfIndex),
                              VEHICLE_ROUTE_PORT));
        m_routeRx->SetRecvCallback(MakeCallback(&VoegVehicleApp::ReceiveRoute, this));

        double jitter = (double)GetNode()->GetId() * JITTER_MULTIPLIER;
        m_reportEvent = Simulator::Schedule(
            Seconds(REPORT_INTERVAL + jitter), &VoegVehicleApp::SendMobilityReport, this);
        m_applyEvent = Simulator::Schedule(
            Seconds(1.0 + jitter), &VoegVehicleApp::ApplyScheduledRoutes, this);
        m_heartbeatEvent = Simulator::Schedule(
            Seconds(HEARTBEAT_CHECK_INTERVAL),
            &VoegVehicleApp::CheckControllerConnectivity, this);
    }

    void StopApplication() override
    {
        Simulator::Cancel(m_reportEvent);
        Simulator::Cancel(m_applyEvent);
        Simulator::Cancel(m_heartbeatEvent);
        if (m_reportTx) m_reportTx->Close();
        if (m_routeRx)  m_routeRx->Close();
    }

    // --- Send current position and velocity to controller ---
    void SendMobilityReport()
    {
        Ptr<MobilityModel> mob = GetNode()->GetObject<MobilityModel>();
        Vector pos = mob->GetPosition();
        Vector vel = mob->GetVelocity();

        std::ostringstream oss;
        oss << GetNodeIpv4Address(GetNode(), m_ctrlIfIndex)
            << " " << pos.x << " " << pos.y
            << " " << vel.x << " " << vel.y;

        std::string data = oss.str();
        Ptr<Packet> p = Create<Packet>((uint8_t*)data.c_str(), data.size());
        m_reportTx->SendTo(
            p, 0, InetSocketAddress(m_controllerIp, MOBILITY_REPORT_PORT));

        m_reportEvent = Simulator::Schedule(
            Seconds(REPORT_INTERVAL), &VoegVehicleApp::SendMobilityReport, this);
    }

    // --- Parse incoming route update and store in cache indexed by time offset ---
    // Message format: "<timeOffset> <dstIP> <nextHopIP> <reliability> ..."
    void ReceiveRoute(Ptr<Socket> socket)
    {
        Address from;
        Ptr<Packet> pkt = socket->RecvFrom(from);
        if (pkt->GetSize() == 0) return;

        m_lastControllerContact = Simulator::Now();

        if (!m_usingSdnRouting) {
            NS_LOG_UNCOND("[" << Simulator::Now().GetSeconds() << "s] Node "
                          << GetNode()->GetId()
                          << ": Controller reconnected – resuming SDN routing");
            m_usingSdnRouting = true;
        }

        std::vector<uint8_t> buffer(pkt->GetSize() + 1, 0);
        pkt->CopyData(buffer.data(), pkt->GetSize());
        std::stringstream ss((char*)buffer.data());

        int        timeOffset;
        std::string dstStr, nhStr;
        double      reliability;

        while (ss >> timeOffset >> dstStr >> nhStr >> reliability) {
            if (dstStr.find('.') == std::string::npos) break;
            Ipv4Address dst(dstStr.c_str());
            Ipv4Address nh(nhStr.c_str());
            m_routeCache[timeOffset][dst] = std::make_pair(nh, reliability);
        }
    }

    // --- Apply pre-computed routes for the current simulation second ---
    void ApplyScheduledRoutes()
    {
        int currentOffset = (int)Simulator::Now().GetSeconds();

        auto it = m_routeCache.find(currentOffset);
        if (it != m_routeCache.end()) {
            InstallRoutes(it->second);
        }

        // Evict stale entries to bound memory usage
        for (auto cit = m_routeCache.begin(); cit != m_routeCache.end(); ) {
            if (cit->first < currentOffset - 2)
                cit = m_routeCache.erase(cit);
            else
                ++cit;
        }

        m_applyEvent = Simulator::Schedule(
            Seconds(1.0), &VoegVehicleApp::ApplyScheduledRoutes, this);
    }

    // --- Install a route map into the static routing table ---
    void InstallRoutes(const std::map<Ipv4Address,
                                      std::pair<Ipv4Address, double>>& routes)
    {
        Ptr<Ipv4> ipv4 = GetNode()->GetObject<Ipv4>();
        Ptr<Ipv4ListRouting> lr =
            DynamicCast<Ipv4ListRouting>(ipv4->GetRoutingProtocol());
        if (!lr) return;

        Ptr<Ipv4StaticRouting> staticRouting;
        for (uint32_t i = 0; i < lr->GetNRoutingProtocols(); i++) {
            int16_t priority;
            Ptr<Ipv4RoutingProtocol> proto = lr->GetRoutingProtocol(i, priority);
            if (DynamicCast<Ipv4StaticRouting>(proto)) {
                staticRouting = DynamicCast<Ipv4StaticRouting>(proto);
                break;
            }
        }
        if (!staticRouting) return;

        for (const auto& [dst, nhRel] : routes) {
            Ipv4Address nh = nhRel.first;

            // Remove existing host route to this destination.
            // Iterate in reverse because RemoveRoute shifts subsequent indices.
            for (uint32_t i = staticRouting->GetNRoutes(); i > 0; i--) {
                Ipv4RoutingTableEntry route = staticRouting->GetRoute(i - 1);
                if (route.GetDest() == dst &&
                    route.GetDestNetworkMask() == Ipv4Mask("255.255.255.255")) {
                    staticRouting->RemoveRoute(i - 1);
                    break;
                }
            }

            staticRouting->AddHostRouteTo(dst, nh, m_dataIfIndex, 0);
            m_cachedRoutes[dst] = nh;
        }
    }

    // --- Monitor controller heartbeat; switch to cached-route fallback if silent ---
    void CheckControllerConnectivity()
    {
        Time   now     = Simulator::Now();
        double elapsed = (now - m_lastControllerContact).GetSeconds();

        if (elapsed > CONTROLLER_TIMEOUT && m_usingSdnRouting) {
            NS_LOG_UNCOND("[" << now.GetSeconds() << "s] Node "
                          << GetNode()->GetId()
                          << ": Controller timeout – using pre-distributed prediction cache");
            m_usingSdnRouting = false;
            // Intentionally keep m_cachedRoutes: pre-distributed routes remain active.
        }

        m_heartbeatEvent = Simulator::Schedule(
            Seconds(HEARTBEAT_CHECK_INTERVAL),
            &VoegVehicleApp::CheckControllerConnectivity, this);
    }

    // Member variables
    Ptr<Socket>    m_reportTx;
    Ptr<Socket>    m_routeRx;
    Ipv4Address    m_controllerIp;
    uint32_t       m_dataIfIndex;
    uint32_t       m_ctrlIfIndex;
    bool           m_usingSdnRouting;
    Time           m_lastControllerContact;
    EventId        m_reportEvent;
    EventId        m_applyEvent;
    EventId        m_heartbeatEvent;
    std::map<Ipv4Address, Ipv4Address> m_cachedRoutes; // dst → current next-hop
    // Pre-distributed routes: timeOffset (seconds) → dst → (next-hop, reliability)
    std::map<int, std::map<Ipv4Address, std::pair<Ipv4Address, double>>> m_routeCache;
};

// ============================================================================
//                      VOEG CONTROLLER APPLICATION
// ============================================================================
// Responsibilities:
//   1. Receive mobility reports from vehicles.
//   2. Build Evolving Graph for PREDICTION_HORIZON future time slots.
//   3. Run reliability-weighted Dijkstra on each time slot.
//   4. Distribute pre-computed route tables (with future time offsets) to vehicles.

class VoegControllerApp : public Application
{
public:
    static TypeId GetTypeId()
    {
        static TypeId tid =
            TypeId("ns3::VoegControllerApp")
                .SetParent<Application>()
                .AddConstructor<VoegControllerApp>();
        return tid;
    }

    VoegControllerApp()
        : m_ctrlIfIndex(CTRL_NODE_IF_INDEX),
          m_enablePredictionCache(true)
    {
    }

    void SetCtrlIfIndex(uint32_t idx)          { m_ctrlIfIndex = idx; }
    void SetEnablePredictionCache(bool enable)  { m_enablePredictionCache = enable; }
    void SetReliabilityLogFile(const std::string& path)
    {
        m_reliabilityLog.open(path.c_str(), std::ios::out);
        if (m_reliabilityLog.is_open())
            m_reliabilityLog << "Time,SourceNode,DestNode,Reliability\n";
    }

private:
    void StartApplication() override
    {
        m_recvSocket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_recvSocket->Bind(
            InetSocketAddress(GetNodeIpv4Address(GetNode(), m_ctrlIfIndex),
                              MOBILITY_REPORT_PORT));
        m_recvSocket->SetRecvCallback(
            MakeCallback(&VoegControllerApp::ReceiveMobilityReport, this));

        m_routeEvent = Simulator::Schedule(
            Seconds(CONTROLLER_START_TIME),
            &VoegControllerApp::BuildAndDistribute, this);
    }

    void StopApplication() override
    {
        Simulator::Cancel(m_routeEvent);
        if (m_recvSocket)          m_recvSocket->Close();
        if (m_reliabilityLog.is_open()) m_reliabilityLog.close();
    }

    // --- Receive and store mobility report from a vehicle ---
    // Message format: "<ctrlIP> <posX> <posY> <velX> <velY>"
    void ReceiveMobilityReport(Ptr<Socket> socket)
    {
        // Drop if controller is in simulated failure mode
        if (g_failureStartTime >= 0.0) {
            double now = Simulator::Now().GetSeconds();
            if (now >= g_failureStartTime &&
                now <= g_failureStartTime + g_failureDuration) {
                Address from;
                socket->RecvFrom(from); // consume and discard
                return;
            }
        }

        Address from;
        Ptr<Packet> pkt = socket->RecvFrom(from);
        if (pkt->GetSize() == 0) return;

        std::vector<uint8_t> buf(pkt->GetSize() + 1, 0);
        pkt->CopyData(buf.data(), pkt->GetSize());
        std::stringstream ss((char*)buf.data());

        std::string ctrlIpStr;
        double px, py, vx, vy;
        ss >> ctrlIpStr >> px >> py >> vx >> vy;
        if (ctrlIpStr.empty() || ctrlIpStr.find('.') == std::string::npos) return;

        Ptr<Node> node = FindNodeByCtrlIp(Ipv4Address(ctrlIpStr.c_str()));
        if (!node) return;

        MobilityData& md = m_nodeMobility[node->GetId()];
        md.position  = Vector(px, py, 0.0);
        md.velocity  = Vector(vx, vy, 0.0);
        md.timestamp = Simulator::Now().GetSeconds();
    }

    // --- Build Evolving Graph for the next 'horizon' time slots ---
    void BuildEvolvingGraph(double now)
    {
        m_evolvingGraph.time_slots.clear();
        m_evolvingGraph.slot_duration       = TIME_SLOT_DURATION;
        m_evolvingGraph.prediction_horizon  =
            m_enablePredictionCache ? PREDICTION_HORIZON : 1;
        m_evolvingGraph.num_nodes =
            g_voegNodes ? g_voegNodes->GetN() : 0;

        if (!g_voegNodes) return;

        int    horizon = m_evolvingGraph.prediction_horizon;
        uint32_t n     = g_voegNodes->GetN();

        for (int k = 0; k < horizon; k++) {
            TimeSlot slot;
            slot.start_time = now + k * TIME_SLOT_DURATION;
            slot.end_time   = now + (k + 1) * TIME_SLOT_DURATION;

            double dt = k * TIME_SLOT_DURATION; // lookahead offset

            for (uint32_t a = 0; a < n; a++) {
                uint32_t idA = g_voegNodes->Get(a)->GetId();
                if (idA == CONTROLLER_NODE_ID) continue;

                for (uint32_t b = a + 1; b < n; b++) {
                    uint32_t idB = g_voegNodes->Get(b)->GetId();
                    if (idB == CONTROLLER_NODE_ID) continue;

                    auto itA = m_nodeMobility.find(idA);
                    auto itB = m_nodeMobility.find(idB);
                    if (itA == m_nodeMobility.end() ||
                        itB == m_nodeMobility.end()) continue;

                    // Predict positions at time slot k using linear kinematics
                    Vector predA(itA->second.position.x + itA->second.velocity.x * dt,
                                 itA->second.position.y + itA->second.velocity.y * dt,
                                 0.0);
                    Vector predB(itB->second.position.x + itB->second.velocity.x * dt,
                                 itB->second.position.y + itB->second.velocity.y * dt,
                                 0.0);

                    double rel = CalculateLinkReliability(
                        predA, itA->second.velocity,
                        predB, itB->second.velocity,
                        TRANSMISSION_RANGE,
                        TIME_SLOT_DURATION);

                    if (rel >= MIN_LINK_RELIABILITY) {
                        slot.adjacency[idA].insert(idB);
                        slot.adjacency[idB].insert(idA);
                        slot.link_reliability[{idA, idB}] = rel;
                    }
                }
            }
            m_evolvingGraph.time_slots.push_back(slot);
        }
    }

    // --- Compute routes for all sources across all time slots ---
    void ComputeAllMRJ(double now)
    {
        if (!g_voegNodes) return;
        m_allRoutes.clear();

        int horizon = (int)m_evolvingGraph.time_slots.size();
        uint32_t n  = g_voegNodes->GetN();

        for (uint32_t i = 0; i < n; i++) {
            uint32_t srcId = g_voegNodes->Get(i)->GetId();
            if (srcId == CONTROLLER_NODE_ID) continue;

            for (int k = 0; k < horizon; k++) {
                int timeOffset = (int)(now + k * TIME_SLOT_DURATION);
                m_allRoutes[srcId][timeOffset] =
                    ComputeRoutesForSlot(srcId, m_evolvingGraph.time_slots[k]);
            }
        }
    }

    // --- Send pre-computed routes (with time offsets) to each vehicle ---
    void DistributeRoutes(double now)
    {
        if (!g_voegNodes) return;

        uint32_t n = g_voegNodes->GetN();
        for (uint32_t i = 0; i < n; i++) {
            Ptr<Node> node   = g_voegNodes->Get(i);
            uint32_t  nodeId = node->GetId();
            if (nodeId == CONTROLLER_NODE_ID) continue;

            auto srcIt = m_allRoutes.find(nodeId);
            if (srcIt == m_allRoutes.end()) continue;

            std::ostringstream ss;

            for (const auto& [timeOffset, routes] : srcIt->second) {
                for (const auto& [dstId, ri] : routes) {
                    if (dstId == CONTROLLER_NODE_ID) continue;
                    if (dstId >= n) continue;

                    if (ri.next_hop >= n || ri.next_hop == CONTROLLER_NODE_ID) continue;

                    Ipv4Address dstIp =
                        GetNodeIpv4Address(g_voegNodes->Get(dstId), DATA_IF_INDEX);
                    Ipv4Address nhIp =
                        GetNodeIpv4Address(g_voegNodes->Get(ri.next_hop), DATA_IF_INDEX);

                    if (dstIp == Ipv4Address::GetZero() ||
                        nhIp  == Ipv4Address::GetZero()) continue;

                    ss << timeOffset << " " << dstIp << " " << nhIp
                       << " " << ri.reliability << " ";

                    // Log reliability metric
                    if (m_reliabilityLog.is_open()) {
                        m_reliabilityLog
                            << now << "," << nodeId << "," << dstId
                            << "," << ri.reliability << "\n";
                    }
                }
            }

            std::string msg = ss.str();
            if (msg.empty()) continue;

            Simulator::Schedule(
                Seconds(i * CONTROLLER_SEND_OFFSET),
                &VoegControllerApp::SendRoutePacket, this, node, msg);
        }
    }

    // --- Main periodic cycle: build EG → compute MRJ → distribute ---
    void BuildAndDistribute()
    {
        // Respect simulated controller failure
        if (g_failureStartTime >= 0.0) {
            double now = Simulator::Now().GetSeconds();
            double failEnd = g_failureStartTime + g_failureDuration;
            if (now >= g_failureStartTime && now <= failEnd) {
                NS_LOG_UNCOND("[" << now << "s] VoEG CONTROLLER FAILURE MODE ("
                              << (failEnd - now) << "s remaining)");
                // Intentionally preserve m_nodeMobility so the controller can
                // resume computing routes immediately when it recovers.
                m_routeEvent = Simulator::Schedule(
                    Seconds(ROUTE_RECOMPUTE_INTERVAL),
                    &VoegControllerApp::BuildAndDistribute, this);
                return;
            }
        }

        double now = Simulator::Now().GetSeconds();
        BuildEvolvingGraph(now);
        ComputeAllMRJ(now);
        DistributeRoutes(now);

        m_routeEvent = Simulator::Schedule(
            Seconds(ROUTE_RECOMPUTE_INTERVAL),
            &VoegControllerApp::BuildAndDistribute, this);
    }

    // --- Unicast a route-update message to a specific vehicle ---
    void SendRoutePacket(Ptr<Node> targetNode, std::string msg)
    {
        Ptr<Packet> p = Create<Packet>((uint8_t*)msg.c_str(), msg.size());
        Ptr<Socket> s = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        s->Bind(InetSocketAddress(GetNodeIpv4Address(GetNode(), m_ctrlIfIndex), 0));
        s->SendTo(p, 0,
                  InetSocketAddress(
                      GetNodeIpv4Address(targetNode, CTRL_IF_INDEX),
                      VEHICLE_ROUTE_PORT));
        s->Close();
    }

    // Member variables
    Ptr<Socket>    m_recvSocket;
    EventId        m_routeEvent;
    uint32_t       m_ctrlIfIndex;
    bool           m_enablePredictionCache;
    std::ofstream  m_reliabilityLog;

    std::map<uint32_t, MobilityData>  m_nodeMobility;
    EvolvingGraph                     m_evolvingGraph;
    // allRoutes[sourceId][timeOffset] → {dstId → RouteInfo}
    std::map<uint32_t,
             std::map<int, std::map<uint32_t, RouteInfo>>> m_allRoutes;
};

// ============================================================================
//                               MAIN
// ============================================================================

int main(int argc, char* argv[])
{
    std::string protocol           = "VoEG";
    int         speed              = 15;
    int         runId              = 1;
    std::string traceFile          = "";
    std::string outputDir          = ".";
    double      simTime            = 200.0;
    uint32_t    numNodes           = 50;
    bool        enableNetAnim      = false;
    bool        enablePredictionCache = true;
    double      failureStart       = -1.0;
    double      failureDuration    = 0.0;

    CommandLine cmd;
    cmd.AddValue("protocol",              "Protocol name (used in output file names)", protocol);
    cmd.AddValue("speed",                 "Vehicle speed (selects mobility trace file)", speed);
    cmd.AddValue("runId",                 "Run ID for repeated experiments", runId);
    cmd.AddValue("traceFile",             "Explicit mobility trace file path", traceFile);
    cmd.AddValue("outputDir",             "Output directory for result files", outputDir);
    cmd.AddValue("netanim",               "Enable NetAnim trace (not used in VoEG)", enableNetAnim);
    cmd.AddValue("enablePredictionCache", "Pre-distribute future routes to vehicles", enablePredictionCache);
    cmd.AddValue("failureStart",          "Controller failure start time in seconds (-1=off)", failureStart);
    cmd.AddValue("failureDuration",       "Duration of controller failure in seconds", failureDuration);
    cmd.Parse(argc, argv);

    g_failureStartTime = failureStart;
    g_failureDuration  = failureDuration;

    if (traceFile.empty())
        traceFile = "scratch/mobility/mobility_" + std::to_string(speed) + ".tcl";

    // Output file paths
    std::string tag        = "_" + std::to_string(speed) + "_" + std::to_string(runId);
    std::string xmlFile    = outputDir + "/result_VoEG"       + tag + ".xml";
    std::string csvFile    = outputDir + "/pdr_graph_VoEG"    + tag + ".csv";
    std::string relLogFile = outputDir + "/reliability_log_VoEG" + tag + ".csv";

    RngSeedManager::SetSeed(3 + runId);
    RngSeedManager::SetRun(runId);

    NS_LOG_UNCOND("=== VoEG Reliable Routing Simulation ===");
    NS_LOG_UNCOND("Speed: " << speed << " m/s | RunId: " << runId);
    if (failureStart >= 0) {
        NS_LOG_UNCOND("Controller failure: " << failureStart << "s – "
                      << (failureStart + failureDuration) << "s");
    }
    if (!enablePredictionCache) {
        NS_LOG_UNCOND("Prediction cache DISABLED (comparison mode)");
    }
    if (enableNetAnim) {
        NS_LOG_UNCOND("Note: NetAnim output is not produced in VoEG mode");
    }

    // -----------------------------------------------------------------------
    //  Node containers
    // -----------------------------------------------------------------------
    NodeContainer vehicles;   vehicles.Create(numNodes);
    NodeContainer controller; controller.Create(1);
    NodeContainer allNodes = NodeContainer(vehicles, controller);
    g_voegNodes = &allNodes;

    // -----------------------------------------------------------------------
    //  Mobility
    // -----------------------------------------------------------------------
    Ns2MobilityHelper ns2(traceFile);
    ns2.Install(vehicles.Begin(), vehicles.End());

    MobilityHelper mobilityCtrl;
    mobilityCtrl.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityCtrl.Install(controller);
    controller.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(500, 500, 0));

    // -----------------------------------------------------------------------
    //  Data WiFi channel: 802.11p (DSRC / WAVE)
    // -----------------------------------------------------------------------
    Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold",
                       UintegerValue(0));
    WifiHelper wifiData;
    wifiData.SetStandard(WIFI_STANDARD_80211p);
    wifiData.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                     "DataMode",    StringValue("OfdmRate12MbpsBW10MHz"),
                                     "ControlMode", StringValue("OfdmRate6MbpsBW10MHz"));
    YansWifiPhyHelper     phyData;
    YansWifiChannelHelper chanData;
    chanData.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    chanData.AddPropagationLoss("ns3::FriisPropagationLossModel");
    chanData.AddPropagationLoss("ns3::RangePropagationLossModel",
                                "MaxRange", DoubleValue(WIFI_DATA_MAX_RANGE));
    phyData.SetChannel(chanData.Create());
    phyData.Set("TxPowerStart", DoubleValue(20.0));
    phyData.Set("TxPowerEnd",   DoubleValue(20.0));
    phyData.Set("ChannelWidth", UintegerValue(10));
    WifiMacHelper macData;
    macData.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer devicesData = wifiData.Install(phyData, macData, vehicles);

    // -----------------------------------------------------------------------
    //  Control WiFi channel: 802.11a (long-range control plane)
    // -----------------------------------------------------------------------
    WifiHelper wifiCtrl;
    wifiCtrl.SetStandard(WIFI_STANDARD_80211a);
    YansWifiPhyHelper     phyCtrl;
    YansWifiChannelHelper chanCtrl;
    chanCtrl.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    chanCtrl.AddPropagationLoss("ns3::RangePropagationLossModel",
                                "MaxRange", DoubleValue(WIFI_CTRL_MAX_RANGE));
    phyCtrl.SetChannel(chanCtrl.Create());
    WifiMacHelper macCtrl;
    macCtrl.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer devCtrlVeh  = wifiCtrl.Install(phyCtrl, macCtrl, vehicles);
    NetDeviceContainer devCtrlNode = wifiCtrl.Install(phyCtrl, macCtrl, controller);

    // -----------------------------------------------------------------------
    //  Internet stack: Static routing (SDN, high priority) + OLSR (fallback)
    // -----------------------------------------------------------------------
    InternetStackHelper      stack;
    Ipv4ListRoutingHelper    list;
    Ipv4StaticRoutingHelper  staticRoutingHelper;
    OlsrHelper               olsr;
    list.Add(staticRoutingHelper, 100); // SDN routes take precedence
    list.Add(olsr,                 10); // OLSR fills gaps during controller failure
    stack.SetRoutingHelper(list);
    stack.Install(allNodes);

    // Enable IP forwarding on vehicle nodes
    for (uint32_t i = 0; i < vehicles.GetN(); ++i)
        vehicles.Get(i)->GetObject<Ipv4>()->SetAttribute("IpForward",
                                                          BooleanValue(true));

    // -----------------------------------------------------------------------
    //  IP address assignment
    // -----------------------------------------------------------------------
    Ipv4AddressHelper ipv4Data;
    ipv4Data.SetBase(DATA_NETWORK_PREFIX, DATA_NETWORK_MASK);
    Ipv4InterfaceContainer ifData = ipv4Data.Assign(devicesData);

    Ipv4AddressHelper ipv4Ctrl;
    ipv4Ctrl.SetBase(CTRL_NETWORK_PREFIX, CTRL_NETWORK_MASK);
    ipv4Ctrl.Assign(devCtrlVeh);                              // assign 10.2.x.x to vehicles
    Ipv4InterfaceContainer ifCtrlNode = ipv4Ctrl.Assign(devCtrlNode);

    // -----------------------------------------------------------------------
    //  Controller application
    // -----------------------------------------------------------------------
    Ptr<VoegControllerApp> ctrlApp = CreateObject<VoegControllerApp>();
    ctrlApp->SetCtrlIfIndex(CTRL_NODE_IF_INDEX);
    ctrlApp->SetEnablePredictionCache(enablePredictionCache);
    ctrlApp->SetReliabilityLogFile(relLogFile);
    controller.Get(0)->AddApplication(ctrlApp);
    ctrlApp->SetStartTime(Seconds(0.1));
    ctrlApp->SetStopTime(Seconds(simTime));

    // -----------------------------------------------------------------------
    //  Vehicle applications
    // -----------------------------------------------------------------------
    Ipv4Address controllerCtrlIp = ifCtrlNode.GetAddress(0);
    for (uint32_t i = 0; i < vehicles.GetN(); ++i) {
        Ptr<VoegVehicleApp> app = CreateObject<VoegVehicleApp>();
        app->Setup(controllerCtrlIp, DATA_IF_INDEX, CTRL_IF_INDEX);
        vehicles.Get(i)->AddApplication(app);
        app->SetStartTime(Seconds(VEHICLE_APP_START_OFFSET + i * VEHICLE_APP_STAGGER));
        app->SetStopTime(Seconds(simTime));
    }

    // -----------------------------------------------------------------------
    //  UDP Echo traffic (vehicle 49 → vehicle 0)
    // -----------------------------------------------------------------------
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

    // -----------------------------------------------------------------------
    //  PDR monitoring
    // -----------------------------------------------------------------------
    Config::ConnectWithoutContext(
        "/NodeList/*/ApplicationList/*/$ns3::UdpEchoClient/Tx",
        MakeCallback(&MonitorTxCallback));
    Config::ConnectWithoutContext(
        "/NodeList/*/ApplicationList/*/$ns3::UdpEchoServer/Rx",
        MakeCallback(&MonitorRxCallback));
    Simulator::Schedule(Seconds(1.0), &RecordPdrPerSecond, csvFile);

    // -----------------------------------------------------------------------
    //  Run simulation and collect FlowMonitor results
    // -----------------------------------------------------------------------
    FlowMonitorHelper  flowmon;
    Ptr<FlowMonitor>   monitor = flowmon.InstallAll();

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    monitor->CheckForLostPackets();
    monitor->SerializeToXmlFile(xmlFile, true, true);
    Simulator::Destroy();

    return 0;
}
