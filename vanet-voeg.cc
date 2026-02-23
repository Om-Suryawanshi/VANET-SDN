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
#include "ns3/olsr-module.h"

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
uint32_t g_pdrTxCount = 0;
uint32_t g_pdrRxCount = 0;
uint32_t g_cacheHits = 0;
uint32_t g_cacheMisses = 0;

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
    if (g_pdrTxCount > 0) pdr = (double)g_pdrRxCount / g_pdrTxCount * 100.0;
    file << currentTime << "," << pdr << "," << g_pdrTxCount << "," << g_pdrRxCount << std::endl;
    g_pdrTxCount = 0;
    g_pdrRxCount = 0;
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
const double ROUTE_RECOMPUTE_INTERVAL = 0.25;
const double CONTROLLER_START_TIME = 0.2;
const double JITTER_MULTIPLIER = 0.002;
const double ROUTE_UPDATE_INTERVAL = 0.5;      // How often vehicles update routing table from journey

// === VOEG PARAMETERS ===
const double PREDICTION_HORIZON = 25.0;        // Time slots to predict ahead
const double TIME_SLOT_DURATION = 1.0;         // Seconds per time slot
const double TRANSMISSION_RANGE = 250.0;       // Meters
const double TRANSMISSION_DELAY = 0.01;        // Seconds per hop (unused: placeholder)
const double MIN_LINK_RELIABILITY = 0.1;       // Threshold for valid links
const int    MSG_PREDICTION_SLOTS = 3;         // Future time slots included in each route message
// Note: TIME_SLOT_DURATION is used as a divisor; must remain positive.

// === DISTANCE THRESHOLDS (meters) ===
const double WIFI_DATA_MAX_RANGE = 350.0;
const double WIFI_CTRL_MAX_RANGE = 2000.0;

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
const double CONTROLLER_SEND_OFFSET = 0.0005;

// === SPECIAL NODE IDs ===
const uint32_t CONTROLLER_NODE_ID = 50;

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
const double   PACKET_INTERVAL = 0.1;
const uint32_t PACKET_SIZE    = 1024;

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

Ptr<Node> FindNodeByCtrlIp(Ipv4Address ctrlIp) {
    if (!g_voegNodes) return nullptr;
    for (uint32_t i = 0; i < g_voegNodes->GetN(); ++i) {
        Ptr<Ipv4> ipv4 = g_voegNodes->Get(i)->GetObject<Ipv4>();
        for (uint32_t j = 0; j < ipv4->GetNInterfaces(); ++j) {
            if (ipv4->GetAddress(j,0).GetLocal() == ctrlIp)
                return g_voegNodes->Get(i);
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
            Seconds(ROUTE_UPDATE_INTERVAL + jitter), &VehicleSdnApp::UpdateRoutingFromJourney, this);
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

        std::ostringstream ss;
        ss << myCtrlIp << " " << pos.x << " " << pos.y << " " << vel.x << " " << vel.y;
        std::string data = ss.str();

        Ptr<Packet> pkt = Create<Packet>((uint8_t*)data.c_str(), data.size());
        Ptr<Socket> s = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        s->Bind(InetSocketAddress(myCtrlIp, 0));
        s->Connect(InetSocketAddress(m_controllerIp, CONTROLLER_REPORT_PORT));
        s->Send(pkt);
        s->Close();

        m_reportEvent = Simulator::Schedule(
            Seconds(REPORT_INTERVAL), &VehicleSdnApp::SendReport, this);
    }

    // Receive time-indexed route update from controller.
    // Message format: "TimeSlot DstDataIP NextHopDataIP ..."  (triples)
    void ReceiveRoute(Ptr<Socket> socket) {
        Address from;
        Ptr<Packet> pkt = socket->RecvFrom(from);
        if (pkt->GetSize() == 0) return;

        m_lastControllerContact = Simulator::Now();

        if (!m_usingSdnRouting) {
            NS_LOG_UNCOND("[" << Simulator::Now().GetSeconds() << "s] Node "
                          << GetNode()->GetId()
                          << ": Controller reconnected! Switching back to VOEG routing");
            m_usingSdnRouting = true;
        }

        std::vector<uint8_t> buffer(pkt->GetSize() + 1, 0);
        pkt->CopyData(buffer.data(), pkt->GetSize());
        std::stringstream ss((char*)buffer.data());

        int        timeSlot;
        std::string dstStr, nhStr;
        while (ss >> timeSlot >> dstStr >> nhStr) {
            if (dstStr.find('.') == std::string::npos) continue;
            Ipv4Address dst(dstStr.c_str());
            Ipv4Address nh(nhStr.c_str());
            m_journey[dst][timeSlot] = nh;
        }

        // Apply current-slot routes immediately
        if (m_usingSdnRouting) UpdateRoutingFromJourney();
    }

    // Apply the journey entry for the current time slot to the static routing table.
    void UpdateRoutingFromJourney() {
        if (!m_usingSdnRouting) {
            m_routeUpdateEvent = Simulator::Schedule(
                Seconds(ROUTE_UPDATE_INTERVAL), &VehicleSdnApp::UpdateRoutingFromJourney, this);
            return;
        }

        double now         = Simulator::Now().GetSeconds();
        int    currentSlot = static_cast<int>(now / TIME_SLOT_DURATION);

        Ptr<Ipv4StaticRouting> staticRouting;
        Ptr<Ipv4> ipv4 = GetNode()->GetObject<Ipv4>();
        Ptr<Ipv4ListRouting> lr = DynamicCast<Ipv4ListRouting>(ipv4->GetRoutingProtocol());
        if (lr) {
            int16_t priority;
            for (uint32_t i = 0; i < lr->GetNRoutingProtocols(); i++) {
                Ptr<Ipv4RoutingProtocol> temp = lr->GetRoutingProtocol(i, priority);
                if (DynamicCast<Ipv4StaticRouting>(temp)) {
                    staticRouting = DynamicCast<Ipv4StaticRouting>(temp);
                    break;
                }
            }
        }
        if (!staticRouting) {
            m_routeUpdateEvent = Simulator::Schedule(
                Seconds(ROUTE_UPDATE_INTERVAL), &VehicleSdnApp::UpdateRoutingFromJourney, this);
            return;
        }

        for (auto& [dst, slotMap] : m_journey) {
            // Find the closest slot entry at or before currentSlot.
            // upper_bound returns iterator to first entry with key > currentSlot.
            auto it = slotMap.upper_bound(currentSlot);
            // If all stored slots are strictly in the future, nothing to apply yet.
            if (it == slotMap.begin()) continue;
            --it; // it->first <= currentSlot

            Ipv4Address nh = it->second;

            // Track cache usage: entry from a previous slot = cache hit
            if (it->first < currentSlot) {
                g_cacheHits++;
            } else {
                g_cacheMisses++;
            }

            // Remove old route to this destination and install updated one
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

        m_routeUpdateEvent = Simulator::Schedule(
            Seconds(ROUTE_UPDATE_INTERVAL), &VehicleSdnApp::UpdateRoutingFromJourney, this);
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

    void CheckControllerConnectivity() {
        double timeSince = (Simulator::Now() - m_lastControllerContact).GetSeconds();
        if (timeSince > CONTROLLER_TIMEOUT && m_usingSdnRouting) {
            NS_LOG_UNCOND("[" << Simulator::Now().GetSeconds() << "s] Node "
                          << GetNode()->GetId()
                          << ": Controller timeout (" << timeSince
                          << "s)! ");
            if (m_enablePredictionCache && !m_journey.empty()) {
                // Use pre-computed VoEG predictions; keep routes in place
                NS_LOG_UNCOND("  Using VoEG prediction cache for continued routing");
            } else {
                SwitchToFallbackRouting();
            }
        }
        m_heartbeatEvent = Simulator::Schedule(
            Seconds(HEARTBEAT_CHECK_INTERVAL),
            &VehicleSdnApp::CheckControllerConnectivity, this);
    }

    void SwitchToFallbackRouting() {
        if (!m_usingSdnRouting) return;
        Ptr<Ipv4> ipv4 = GetNode()->GetObject<Ipv4>();
        Ptr<Ipv4ListRouting> listRouting = DynamicCast<Ipv4ListRouting>(ipv4->GetRoutingProtocol());
        if (!listRouting) return;
        for (uint32_t i = 0; i < listRouting->GetNRoutingProtocols(); ++i) {
            int16_t priority;
            Ptr<Ipv4RoutingProtocol> proto = listRouting->GetRoutingProtocol(i, priority);
            if (DynamicCast<Ipv4StaticRouting>(proto)) {
                PurgeSDNRoutes(DynamicCast<Ipv4StaticRouting>(proto));
                break;
            }
        }
        m_cachedRoutes.clear();
        m_usingSdnRouting = false;
    }

    Ptr<Socket> m_ctrlRx;
    Ipv4Address m_controllerIp;
    uint32_t    m_dataIfIndex, m_ctrlIfIndex;
    EventId     m_reportEvent, m_heartbeatEvent, m_routeUpdateEvent;
    bool        m_usingSdnRouting;
    bool        m_enablePredictionCache;
    Time        m_lastControllerContact;

    // Journey table: dst -> (time_slot -> next_hop)
    std::map<Ipv4Address, std::map<int,Ipv4Address>> m_journey;
    // Last-applied routes (cache)
    std::map<Ipv4Address, Ipv4Address> m_cachedRoutes;
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

private:
    virtual void StartApplication() {
        m_recvSocket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_recvSocket->Bind(InetSocketAddress(
            GetNodeIpv4Address(GetNode(), m_ctrlIfIndex), CONTROLLER_REPORT_PORT));
        m_recvSocket->SetRecvCallback(
            MakeCallback(&SdnControllerApp::ReceiveReport, this));
        m_routeEvent = Simulator::Schedule(
            Seconds(CONTROLLER_START_TIME), &SdnControllerApp::RecomputeRoutes, this);
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
        for (auto& [id, _] : m_vehicleStates) ids.push_back(id);

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
        }
    }

    void RecomputeRoutes() {
        // Simulate controller being offline
        if (g_failureStartTime >= 0) {
            double now        = Simulator::Now().GetSeconds();
            double failureEnd = g_failureStartTime + g_failureDuration;
            if (now >= g_failureStartTime && now <= failureEnd) {
                NS_LOG_UNCOND("[" << now
                              << "s] VOEG CONTROLLER FAILURE MODE - Not sending routes ("
                              << (failureEnd - now) << "s remaining)");
                // Clear vehicle states intentionally: simulates the controller
                // "rebooting" and losing all topology knowledge.  Routes will
                // resume after vehicles resend their next mobility reports
                // (within REPORT_INTERVAL seconds of controller recovery).
                m_vehicleStates.clear();
                m_routeEvent = Simulator::Schedule(
                    Seconds(ROUTE_RECOMPUTE_INTERVAL),
                    &SdnControllerApp::RecomputeRoutes, this);
                return;
            }
        }

        if (!g_voegNodes || m_vehicleStates.empty()) {
            m_routeEvent = Simulator::Schedule(
                Seconds(ROUTE_RECOMPUTE_INTERVAL),
                &SdnControllerApp::RecomputeRoutes, this);
            return;
        }

        // Build the VoEG for the prediction horizon
        std::vector<TimeSlotGraph> evolvingGraph;
        BuildEvolvingGraph(evolvingGraph, static_cast<int>(PREDICTION_HORIZON));

        double now         = Simulator::Now().GetSeconds();
        int    baseSlot    = static_cast<int>(now / TIME_SLOT_DURATION);

        // For each source vehicle, run EG-Dijkstra and dispatch route packets
        for (uint32_t i = 0; i < g_voegNodes->GetN(); ++i) {
            Ptr<Node> srcNode = g_voegNodes->Get(i);
            uint32_t  srcId   = srcNode->GetId();
            if (srcId == CONTROLLER_NODE_ID) continue;
            if (m_vehicleStates.find(srcId) == m_vehicleStates.end()) continue;

            // Run EG-Dijkstra once from baseSlot.  The algorithm already explores
            // all future time slots, so a single call naturally produces journey
            // entries at multiple time slots (covering the prediction window).
            std::ostringstream ss;

            auto journeys = RunEGDijkstra(srcId, evolvingGraph, baseSlot);

            for (auto& [dstId, slotMap] : journeys) {
                if (dstId == CONTROLLER_NODE_ID) continue;
                if (dstId >= g_voegNodes->GetN()) continue;

                Ipv4Address dstDataIp =
                    GetNodeIpv4Address(g_voegNodes->Get(dstId), 1);
                if (dstDataIp == Ipv4Address::GetZero()) continue;

                // Send at most MSG_PREDICTION_SLOTS entries per destination
                int sent = 0;
                for (auto& [slot, nhId] : slotMap) {
                    if (sent >= MSG_PREDICTION_SLOTS) break;
                    if (nhId >= g_voegNodes->GetN()) continue;
                    Ipv4Address nhDataIp =
                        GetNodeIpv4Address(g_voegNodes->Get(nhId), 1);
                    if (nhDataIp == Ipv4Address::GetZero()) continue;
                    ss << slot << " " << dstDataIp << " " << nhDataIp << " ";
                    ++sent;
                }
            }

            std::string msg = ss.str();
            if (msg.empty()) continue;

            Simulator::Schedule(
                Seconds(i * CONTROLLER_SEND_OFFSET),
                &SdnControllerApp::SendRoutePacket, this, srcNode, msg);
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
    int         speed                 = 10;
    int         runId                 = 1;
    std::string traceFile             = "";
    std::string outputDir             = ".";
    double      simTime               = 200.0;
    uint32_t    numNodes              = 50;
    bool        enableNetAnim         = false;
    bool        enableFailure         = false;
    bool        enablePredictionCache = true;
    double      failureStart          = -1.0;
    double      failureDuration       = 0.0;

    CommandLine cmd;
    cmd.AddValue("protocol",              "Protocol (VOEG)",                           protocol);
    cmd.AddValue("speed",                 "Vehicle speed for mobility file selection", speed);
    cmd.AddValue("runId",                 "Simulation run ID",                         runId);
    cmd.AddValue("traceFile",             "Path to mobility trace file",               traceFile);
    cmd.AddValue("outputDir",             "Output directory for results",              outputDir);
    cmd.AddValue("netanim",               "Enable NetAnim output",                     enableNetAnim);
    cmd.AddValue("enableFailure",         "Enable controller failure simulation",      enableFailure);
    cmd.AddValue("enablePredictionCache", "Enable local route caching",               enablePredictionCache);
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
        traceFile = "scratch/mobility/mobility_" + std::to_string(speed) + ".tcl";
    }

    std::string xmlFileName  = outputDir + "/result_VOEG_"       + std::to_string(speed) + "_" + std::to_string(runId) + ".xml";
    std::string csvFileName  = outputDir + "/pdr_graph_VOEG_"    + std::to_string(speed) + "_" + std::to_string(runId) + ".csv";
    std::string cachFileName = outputDir + "/cache_status_VOEG_" + std::to_string(speed) + "_" + std::to_string(runId) + ".csv";
    std::string animFileName = outputDir + "/netanim_VOEG_"       + std::to_string(speed) + "_" + std::to_string(runId) + ".xml";

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
    //  Routing: Static (SDN routes, priority 100) + OLSR fallback (priority 10)
    // --------------------------------------------------------------------------
    InternetStackHelper     stack;
    Ipv4ListRoutingHelper   list;
    Ipv4StaticRoutingHelper staticRouting;
    list.Add(staticRouting, 100);
    OlsrHelper olsr;
    list.Add(olsr, 10);
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
    Simulator::Destroy();

    return 0;
}
