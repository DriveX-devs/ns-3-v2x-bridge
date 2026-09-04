/*
 * v2x-bridge.cc
 *
 * Real-time co-simulation bridge between an external driving simulator (CARLA/OpenCDA, 
 * or a Python application similar to the included test harness ) and a simulated 5G NR
 * network built with 5G-LENA (nr module).
 *
 * Architecture
 * ------------
 * external appl. --> real UDP (JSON) -->  v2x-bridge  --> simulated 5G NR network
 *
 * The external application is expected to send control datagrams over a real OS
 * UDP socket to control the simulation. These datagrams contains "commands" for ns-3.
 * Each UDP control datagram must carry the current scene (position of road users) and,
 * optionally, a descriptor of one packet to transmit. This program then
 * generates that packet inside the simulation, between simulated NR nodes,
 * over the Uu air interface (UE <-> gNB <-> EPC). It measures the one-way
 * application-layer latency, logs every send and every receive/failure to two
 * CSV files, and, when requested, reports the outcome back to the external
 * application with a real UDP JSON reply.
 *
 * The real control packets are commands, and not directly traffic which tx
 * should be simulated: hence, no ns-3 TapBridge/FdNetDevice is involved.
 *
 * Time synchronization (two modes)
 * --------------------------------
 * 1 (default) - external clock (OpenCDA/CARLA/external appl. co-simulation):
 * the external simulator is the clock master; ns-3 never runs ahead of it.
 *
 * How it works in practice: ExternalSyncLoop() is scheduled on the ns-3
 * event queue like any other event, but its callback does not return until a
 * command shows up, i.e., it sits in a poll() on the real UDP socket. ns-3 is
 * single-threaded and runs one event at a time, so the whole simulator waits
 * inside that call: wall-clock time passes, simulated time does not. This is
 * the "gate", and there is always exactly one of them pending.
 *
 * Once commands arrive, the gate does three things and returns: (1) it maps each
 * external timestamp onto the simulation clock (see below) and schedules
 * every command at its own simulated time; (2) it schedules the next gate event
 * `drainMs` after the newest of the commands; and (3) by returning it lets ns-3,
 * resume, which jumps the clock forward to the first of those events.
 *
 * That `drainMs` margin is what leaves a slice of simulated time free to run
 * before the next gate freezes the clock again. Packets need it: sending a
 * packet does not deliver it, it only schedules its arrival a few
 * milliseconds later, and a frozen clock never reaches that arrival.
 * Example with CARLA ticking every 50 ms and drainMs=20: a command for
 * t_sim=0.500 s sends a packet whose arrival is ~7 ms away -> the next gate is
 * placed at 0.520 s, so the simulation runs that far, the packet is delivered
 * at ~0.507 s and its reply can reach OpenCDA inside the same CARLA tick. With
 * drainMs=0 the next gate would sit at 0.500 s, no time would pass at all,
 * and that reply would only come out on the following tick, i.e., 50 ms late.
 * Important: `drainMs` must be kept below the external tick period (e.g., CARLA
 * update peridocity).
 *
 * The mapping itself works as follows: external timestamps and simulation time run at the same
 * rate but start from different origins, so the bridge locks them together on
 * the first command it ever receives: that timestamp becomes the origin and
 * is mapped to `warmupMs` of simulation time (500 ms by default, which the UE
 * pool needs to complete its RRC attach before any packet flies). Every later
 * command is placed at
 *
 *     t_sim = warmupMs + (t_ext - t_ext_first)
 *
 * e.g., with warmupMs=500 and a first command at t_ext=6.735 s: that command
 * runs at t_sim=0.5 s, one at t_ext=6.785 s runs at t_sim=0.55 s, and one at
 * t_ext=10.735 s runs at t_sim=4.5 s. Nothing is scaled or dropped: a command
 * whose timestamp is already behind the simulation clock (out-of-order, or
 * inside an already-simulated drain window) simply runs at the current time.
 *
 * Wall-clock speed is irrelevant in this mode: a paused, slower- or
 * faster-than-real-time CARLA only makes the gate block longer.
 *
 * 2 - real time (--realtime): the program runs under
 * ns3::RealtimeSimulatorImpl so that simulation time tracks wall-clock time,
 * and a recurring 1 ms poll event drains the real socket. Can be used when the
 * external application has no usable time reference of its own.
 *
 * Modelled network (can be modified by changing the code below)
 * -------------------------------------------------------------
 *  - One gNB at a position provided by the user; band n78 (3.5 GHz), 60 MHz,
 *    numerology 1, TDD with the following DL/UL-symmetric pattern:
 *    "DL|S|UL|UL|DL|DL|S|UL|UL|DL|".
 *  - A pool of `maxUes` UEs created and attached at t=0 (5G-LENA does
 *    not support attaching UEs after Simulator::Run in this release). Idle
 *    UEs are placed close to the gNB; the first time an origin_ID appears it
 *    permanently claims a pool slot and is teleported to its reported
 *    position. Subsequent scene updates just move it.
 *  - The Base Station acts as an application endpoint (with reserved origin_ID "BS")
 *    and it is connected to an edge/MEC server: the EPC remote host is connected to the
 *    PGW through a zero-delay 100 Gb/s point-to-point link. This is the
 *    idiomatic 5G-LENA equivalent of "an application running at the gNB
 *    site"; the gNB node itself is a radio node, not an IP application host.
 *  - IPv4 only. UEs get 7.0.0.0/8 addresses from the EPC helper (which also
 *    installs their default route); the remote host is wired by
 *    NrEpcHelper::SetupRemoteHost.
 *  - All simulated traffic uses UDP port `simAppPort` as both source and
 *    destination port (each node single simulated socket is bound to it), so one
 *    dedicated EPS bearer TFT on that port classifies every direction,
 *    including the UE->PGW->UE hairpin used for vehicle-to-vehicle messages.
 *
 * Packet identity and application payload
 * ---------------------------------------
 * The first 8 bytes of every simulated payload carry the packet_id
 * (big-endian uint64), followed by a 4-byte big-endian payload length and the
 * optional application payload (the actual bytes of the message being
 * simulated: a UPER-encoded ETSI CAM, a JSON detection list, a warning JSON, ...),
 * supplied by the external application as base64 in the command's
 * "packet.payload" field. The payload therefore traverses the
 * simulated NR network inside the packet; at the destination node it is
 * extracted and handed back to the external application inside the delivery
 * reply ("payload" field, base64 again). Everything else about the packet
 * (send time, type, requested reply, real reply address) is kept in the host in
 * a pending map. Delivery is detected by the receiver socket callback; when a
 * per-packet timeout (--timeoutMs) expires before the packet is received, it is
 * logged as a failure/loss. The id/length/payload travel in the payload rather than a
 * PacketTag so that they survive RLC segmentation/reassembly and GTP-U
 * tunnelling unconditionally.
 *
 * Injected packet loss (--injected-loss-perc)
 * -------------------------------------------
 * On top of whatever the NR model itself loses, a configurable percentage of
 * the packets can be discarded on purpose, to study how the external
 * application behaves on an unreliable link even when few UEs are actually simulated. 
 * The drop is applied at the destination application socket: the packet is really generated,
 * scheduled and transmitted over the simulated air interface (so radio load, latency
 * distribution and every other measurement stay exactly what they would be
 * without the injected loss), and is then discarded instead of being handed back to
 * the external application. Such a packet is logged and flagged as "lost"
 * (i.e. as a non-delivery, with no latency) as soon as it would have been
 * delivered, so the external application learns about the loss immediately
 * rather than after `timeoutMs`. Losses are drawn from a dedicated
 * UniformRandomVariable created before any model object, so the sequence of
 * draws is fixed by --RngRun/--RngSeed alone and is not shifted by how many
 * random variables the rest of the scenario happens to instantiate (which
 * varies with maxUes, scenario, ...).
 *
 * JSON protocol and CSV formats are documented in detail in DOCS.md.
 */

#include "json.hpp"

#include "ns3/antenna-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"

#include <arpa/inet.h>
#include <cmath>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <netinet/in.h>
#include <optional>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <unordered_map>

using namespace ns3;
using json = nlohmann::json;

NS_LOG_COMPONENT_DEFINE("V2XBridge");

// Reserved origin_ID identifying the Base Station/MEC endpoint
static const std::string BS_ID = "BS";

// One pre-created UE of the pool: a slot is claimed permanently by the first
// origin_ID that needs it; node, IP and identity persist for the whole run
struct UeSlot
{
    Ptr<Node> node;
    Ptr<NetDevice> dev;
    Ipv4Address ip;
    Ptr<Socket> sock;     // app socket, bound to g_simAppPort
    bool active = false;  // flag: "is it claimed by an origin_ID?"
    std::string originId; // owner (once active)
    double velocity = 0;  // last reported speed (m/s)
    double heading = 0;   // last reported heading (rad)
};

// Structure to store information about still pending/in-flight simulated packets
struct PendingPacket
{
    Time txTime;
    std::string type;
    std::string senderId;
    std::string receiverId;
    bool replyRequested = false;
    sockaddr_in replyAddr{}; // real source of the triggering command (from the real socket)
    EventId timeoutEv;
};

static std::vector<UeSlot> g_slots;
static std::map<std::string, uint32_t> g_originToSlot;
static std::unordered_map<uint64_t, PendingPacket> g_pending;

static Ptr<Node> g_remoteHost;    // "BS"/MEC application endpoint
static Ipv4Address g_remoteHostIp;
static Ptr<Socket> g_bsSock;      // Simulated socket on the remote host
static Ptr<IdealBeamformingHelper> g_beamHelper;

static int g_ctlFd = -1;          // Real OS UDP control socket descriptor
static uint16_t g_simAppPort = 6000;
static uint32_t g_timeoutMs = 500;
static bool g_verbose = false;

// Variables to manage the injected packet loss, i.e., the percentage of the packets that reach
// their destination node and are nevertheless discarded there for testing unreliable networks
static double g_injectedLossPerc = 0.0;      // 0 = no injected loss (default)
static Ptr<UniformRandomVariable> g_lossRng; // dedicated random variable stream, drawn in [0,100)
static uint64_t g_lossDraws = 0;             // packets that reached their destination
static uint64_t g_lossDrops = 0;             // packets that were dropped on purpose

// Time-synchronization mode
static bool g_realtime = false;   // true for wall-clock pacing instead of external clock
static Time g_warmup;             // warmup time (see above)
static Time g_drain;              // drain time (see above)
static bool g_extEpochSet = false;
static double g_extEpoch0 = 0;    // external timestamp mapped to g_warmup [s]

static std::ofstream g_csvSend;
static std::ofstream g_csvRecv;

static volatile sig_atomic_t g_sigStop = 0;

static void
SignalHandler(int)
{
    // A signal can interrupt the program anywhere, so the handler only raises
    // a flag; the gate's poll loop then checks the flag and stops the simulation
    g_sigStop = 1;
}

/// Simulation time in milliseconds with microsecond resolution (>=3 decimals).
static double
NowMs()
{
    return Simulator::Now().GetNanoSeconds() / 1e6;
}

// Base64 (RFC 4648) encode/decode for the application payload carried inside
// the simulated packets ("packet.payload" in commands, "payload" in replies).
static const char B64_ALPHABET[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static std::string
Base64Encode(const uint8_t* data, size_t len)
{
    std::string out;
    out.reserve(((len + 2) / 3) * 4);
    for (size_t i = 0; i < len; i += 3)
    {
        uint32_t v = data[i] << 16;
        if (i + 1 < len)
        {
            v |= data[i + 1] << 8;
        }
        if (i + 2 < len)
        {
            v |= data[i + 2];
        }
        out.push_back(B64_ALPHABET[(v >> 18) & 0x3f]);
        out.push_back(B64_ALPHABET[(v >> 12) & 0x3f]);
        out.push_back(i + 1 < len ? B64_ALPHABET[(v >> 6) & 0x3f] : '=');
        out.push_back(i + 2 < len ? B64_ALPHABET[v & 0x3f] : '=');
    }
    return out;
}

static bool
Base64Decode(const std::string& in, std::vector<uint8_t>& out)
{
    static int8_t rev[256];
    static bool init = false;
    if (!init)
    {
        std::fill(rev, rev + 256, int8_t(-1));
        for (int i = 0; i < 64; i++)
        {
            rev[uint8_t(B64_ALPHABET[i])] = int8_t(i);
        }
        init = true;
    }
    out.clear();
    uint32_t v = 0;
    int bits = 0;
    for (char c : in)
    {
        if (c == '=' || c == '\n' || c == '\r')
        {
            continue;
        }
        int8_t d = rev[uint8_t(c)];
        if (d < 0)
        {
            return false;
        }
        v = (v << 6) | d;
        bits += 6;
        if (bits >= 8)
        {
            bits -= 8;
            out.push_back((v >> bits) & 0xff);
        }
    }
    return true;
}

static void
SendRealReply(const sockaddr_in& to, const json& j)
{
    std::string s = j.dump();
    ssize_t n = sendto(g_ctlFd, s.data(), s.size(), 0,
                       reinterpret_cast<const sockaddr*>(&to), sizeof(to));
    if (n < 0)
    {
        std::cerr << "WARNING: v2x-bridge: sendto(reply) failed: " << std::strerror(errno) << std::endl;
    }
}

// Reply for a completed/failed packet. latencyMs < 0 means "no latency"
// (null). payloadB64, when non-empty, is the application payload extracted
// from the simulated packet at the destination node (if the packet was successfully
// delivered)
static void
SendPacketReply(const PendingPacket& p, uint64_t id, const std::string& status, double latencyMs,
                const std::string& errorMsg = "", const std::string& payloadB64 = "")
{
    if (!p.replyRequested)
    {
        return;
    }
    json j = {{"msg_type", "reply"},
              {"packet_id", id},
              {"type", p.type},
              {"status", status},
              {"latency_ms", latencyMs < 0 ? json(nullptr) : json(latencyMs)},
              {"sim_time_ms", NowMs()}};
    if (!errorMsg.empty())
    {
        j["error_msg"] = errorMsg;
    }
    if (!payloadB64.empty())
    {
        j["payload"] = payloadB64;
    }
    SendRealReply(p.replyAddr, j);
}

static void
LogSendRow(uint64_t id, const std::string& receiver, const std::string& sender, uint32_t size,
           const std::string& type)
{
    g_csvSend << id << ',' << receiver << ',' << sender << ',' << size << ',' << std::fixed
              << std::setprecision(3) << NowMs() << ',' << type << '\n';
    g_csvSend.flush();
}

// A latencyMs < 0 is set to point to an empty latency field (i.e.,for failed/error).
static void
LogRecvRow(uint64_t id, double latencyMs, const std::string& type, const std::string& status)
{
    g_csvRecv << id << ',' << std::fixed << std::setprecision(3) << NowMs() << ',';
    if (latencyMs >= 0)
    {
        g_csvRecv << std::fixed << std::setprecision(3) << latencyMs;
    }
    g_csvRecv << ',' << type << ',' << status << '\n';
    g_csvRecv.flush();
}

static void
OnSimPacketReceived(Ptr<Socket> socket)
{
    Ptr<Packet> pkt;
    Address from;
    while ((pkt = socket->RecvFrom(from)))
    {
        uint32_t sz = pkt->GetSize();
        if (sz < 8)
        {
            std::cerr << "WARNING: v2x-bridge: simulated packet shorter than 8 bytes, ignoring" << std::endl;
            continue;
        }
        std::vector<uint8_t> data(sz);
        pkt->CopyData(data.data(), sz);
        uint64_t id = 0;
        for (int i = 0; i < 8; i++)
        {
            id = (id << 8) | data[i];
        }
        // Layout of every simulated packet:
        //   [0..7]   packet id           (big-endian uint64, read above)
        //   [8..11]  payload length L    (big-endian uint32, 0 = no payload)
        //   [12..]   the L payload bytes
        // Packets shorter than 12 B carry the id alone, with no length field:
        // they are accepted and simply treated as payload-less
        std::string payloadB64;
        if (sz >= 12)
        {
            uint32_t plen = (uint32_t(data[8]) << 24) | (uint32_t(data[9]) << 16) |
                            (uint32_t(data[10]) << 8) | uint32_t(data[11]);
            if (plen > 0 && 12 + plen <= sz)
            {
                payloadB64 = Base64Encode(data.data() + 12, plen);
            }
            else if (plen > 0)
            {
                std::cerr << "WARNING: v2x-bridge: sim packet id " << id << " declares a " << plen
                          << " B payload but is only " << sz << " B, payload dropped"
                          << std::endl;
            }
        }

        auto it = g_pending.find(id);
        if (it == g_pending.end())
        {
            // Late arrival after its timeout already fired (already logged as
            // failed), or a stray duplicate. Keep one CSV row per attempt.
            std::cerr << "INFO: v2x-bridge: late/unknown sim packet id " << id << ", dropped"
                      << std::endl;
            continue;
        }

        PendingPacket& p = it->second;
        double latencyMs = (Simulator::Now() - p.txTime).GetNanoSeconds() / 1e6;

        // Injected loss: the packet did traverse the simulated network, but is
        // discarded here instead of being delivered to the application
        // This enables the simulation of unreliable networks even when few UEs are actually simulated

        // The outcome is managed now (not at the timeout) so that the external application/simulator is
        // not stalled waiting for a reply that only a failure could produce
        if (g_injectedLossPerc > 0.0)
        {
            g_lossDraws++;
            if (g_lossRng->GetValue() < g_injectedLossPerc)
            {
                g_lossDrops++;
                LogRecvRow(id, -1, p.type, "lost");
                if (g_verbose)
                {
                    std::cout << "INFO: [" << std::fixed << std::setprecision(3) << NowMs()
                              << " ms] lost id=" << id << " " << p.senderId << " -> "
                              << p.receiverId << " (injected loss, would have been "
                              << latencyMs << " ms)" << std::endl;
                }
                p.timeoutEv.Cancel();
                SendPacketReply(p, id, "lost", -1, "discarded by injected packet loss");
                g_pending.erase(it);
                continue;
            }
        }

        LogRecvRow(id, latencyMs, p.type, "delivered");
        if (g_verbose)
        {
            std::cout << "INFO: [" << std::fixed << std::setprecision(3) << NowMs() << " ms] delivered id="
                      << id << " " << p.senderId << " -> " << p.receiverId << " latency="
                      << latencyMs << " ms" << std::endl;
        }
        p.timeoutEv.Cancel();
        SendPacketReply(p, id, "delivered", latencyMs, "", payloadB64);
        g_pending.erase(it);
    }
}

static void
OnPacketTimeout(uint64_t id)
{
    auto it = g_pending.find(id);
    if (it == g_pending.end())
    {
        return;
    }
    PendingPacket& p = it->second;
    LogRecvRow(id, -1, p.type, "timeout");
    std::cerr << "INFO: v2x-bridge: packet id " << id << " (" << p.senderId << " -> " << p.receiverId
              << ") not delivered within " << g_timeoutMs << " ms" << std::endl;
    SendPacketReply(p, id, "timeout", -1);
    g_pending.erase(it);
}

// This function handles one element of a command's "entities" array, i.e., the reported
// state of a single vehicle or VRU (origin_ID + position). The first time an
// origin_ID is seen it permanently claims a free UE from the pool (node + IP, kept for
// the whole run); every later report just moves that UE to the new position.
// Velocity and heading are stored for logging only and they do not drive ns-3
// mobility, as opposed to what the external simulator likely does.
// Returns false when a new origin_ID cannot be admitted because the pool is
// full. The caller uses the return value to decide whether to refresh the
// beamforming vectors, so it reads as "something in the scene changed".
static bool
UpdateEntity(const json& e)
{
    std::string id = e.at("origin_ID").get<std::string>();
    if (id == BS_ID)
    {
        std::cerr << "ERROR: v2x-bridge: entity update for reserved ID \"" << BS_ID
                  << "\" ignored (gNB position is fixed by CLI)" << std::endl;
        return true;
    }

    uint32_t slotIdx;
    auto it = g_originToSlot.find(id);
    if (it != g_originToSlot.end())
    {
        slotIdx = it->second;
    }
    else
    {
        // First appearance: claim the next free pool slot (persistently)
        bool found = false;
        for (uint32_t i = 0; i < g_slots.size(); i++)
        {
            if (!g_slots[i].active)
            {
                slotIdx = i;
                found = true;
                break;
            }
        }
        if (!found)
        {
            std::cerr << "ERROR: v2x-bridge: UE pool exhausted, cannot map origin_ID \"" << id
                      << "\" (raise --maxUes)" << std::endl;
            return false;
        }
        g_slots[slotIdx].active = true;
        g_slots[slotIdx].originId = id;
        g_originToSlot[id] = slotIdx;
        std::cout << "INFO: v2x-bridge: origin_ID \"" << id << "\" -> UE slot " << slotIdx << " (IP "
                  << g_slots[slotIdx].ip << ")" << std::endl;
    }

    const json& pos = e.at("Position");
    Vector v(pos.at("x_m").get<double>(), pos.at("y_m").get<double>(),
             pos.at("z_m").get<double>());
    g_slots[slotIdx].node->GetObject<MobilityModel>()->SetPosition(v);
    g_slots[slotIdx].velocity = e.value("Velocity", 0.0);
    g_slots[slotIdx].heading = e.value("Heading", 0.0);
    if (g_verbose)
    {
        std::cout << "[" << std::fixed << std::setprecision(3) << NowMs() << " ms] " << id
                  << " at (" << v.x << ", " << v.y << ", " << v.z << ")" << std::endl;
    }
    return true;
}

// Resolve an endpoint origin_ID to its simulated socket (sender side) and IP address
// (receiver side). Returns false if unknown/inactive.
static bool
ResolveEndpoint(const std::string& id, Ptr<Socket>& sock, Ipv4Address& ip)
{
    if (id == BS_ID)
    {
        sock = g_bsSock;
        ip = g_remoteHostIp;
        return true;
    }
    auto it = g_originToSlot.find(id);
    if (it == g_originToSlot.end())
    {
        return false;
    }
    sock = g_slots[it->second].sock;
    ip = g_slots[it->second].ip;
    return true;
}

// Main function to handle packet commands (see DOCS.md)
static void
HandlePacketCommand(const json& pk, const sockaddr_in& src)
{
    uint64_t id = pk.at("packet_id").get<uint64_t>();
    std::string sender = pk.at("sender").get<std::string>();
    std::string receiver = pk.at("receiver").get<std::string>();
    uint32_t size = pk.at("size_bytes").get<uint32_t>();
    std::string type = pk.at("type").get<std::string>();
    bool wantReply = pk.value("request_reply", false);
    std::vector<uint8_t> payload;
    bool badPayload = false;
    if (pk.contains("payload"))
    {
        badPayload = !Base64Decode(pk["payload"].get<std::string>(), payload);
    }

    PendingPacket p;
    p.txTime = Simulator::Now();
    p.type = type;
    p.senderId = sender;
    p.receiverId = receiver;
    p.replyRequested = wantReply;
    p.replyAddr = src;

    // Errors are logged to both CSVs
    auto fail = [&](const std::string& status, const std::string& why) {
        LogSendRow(id, receiver, sender, size, type);
        LogRecvRow(id, -1, type, status);
        std::cerr << "WARNING: v2x-bridge: packet id " << id << " rejected: " << why << std::endl;
        SendPacketReply(p, id, status, -1, why);
    };

    if (type != "detection" && type != "warning")
    {
        fail("error", "unknown packet type \"" + type + "\"");
        return;
    }
    if (badPayload)
    {
        fail("error", "invalid base64 in \"payload\"");
        return;
    }
    if (12 + payload.size() > 60000u)
    {
        fail("error", "payload larger than the 60000 B simulated datagram limit");
        return;
    }
    if (sender == receiver)
    {
        fail("error", "sender == receiver");
        return;
    }
    if (g_pending.count(id) > 0)
    {
        // Do not touch the original in-flight packet.
        LogSendRow(id, receiver, sender, size, type);
        LogRecvRow(id, -1, type, "error_duplicate");
        std::cerr << "WARNING: v2x-bridge: duplicate in-flight packet_id " << id << std::endl;
        SendPacketReply(p, id, "error_duplicate", -1, "packet_id already in flight");
        return;
    }

    Ptr<Socket> srcSock;
    Ipv4Address srcIp;
    Ptr<Socket> dstSock;
    Ipv4Address dstIp;
    if (!ResolveEndpoint(sender, srcSock, srcIp))
    {
        fail("error", "unknown sender \"" + sender + "\"");
        return;
    }
    if (!ResolveEndpoint(receiver, dstSock, dstIp))
    {
        fail("error", "unknown receiver \"" + receiver + "\"");
        return;
    }

    // The simulated payload needs 12 bytes for the embedded packet_id +
    // application-payload length, plus the application payload itself (if
    // any); size_bytes can pad it further (never truncate it). Datagrams
    // above ~65 kB do not fit a single UDP packet
    uint32_t effSize = std::max(size, 12u + uint32_t(payload.size()));
    effSize = std::min(effSize, 60000u);

    std::vector<uint8_t> buf(effSize, 0);
    for (int i = 0; i < 8; i++)
    {
        buf[i] = (id >> (8 * (7 - i))) & 0xff;
    }
    uint32_t plen = payload.size();
    buf[8] = (plen >> 24) & 0xff;
    buf[9] = (plen >> 16) & 0xff;
    buf[10] = (plen >> 8) & 0xff;
    buf[11] = plen & 0xff;
    std::copy(payload.begin(), payload.end(), buf.begin() + 12);
    Ptr<Packet> pkt = Create<Packet>(buf.data(), effSize);

    // Log the tx, then transmit over the simulated NR network
    LogSendRow(id, receiver, sender, size, type);
    int sent = srcSock->SendTo(pkt, 0, InetSocketAddress(dstIp, g_simAppPort));
    if (sent < 0)
    {
        LogRecvRow(id, -1, type, "error");
        SendPacketReply(p, id, "error", -1, "ns-3 SendTo failed");
        return;
    }

    p.timeoutEv = Simulator::Schedule(MilliSeconds(g_timeoutMs), &OnPacketTimeout, id);
    g_pending[id] = p;
    if (g_verbose)
    {
        std::cout << "INFO: [" << std::fixed << std::setprecision(3) << NowMs() << " ms] sent id=" << id
                  << " " << sender << " -> " << receiver << " (" << effSize << " B, " << type
                  << ")" << std::endl;
    }
}

/// Scene update + optional packet send, shared by both time modes. Runs at
/// the simulation time the command is meant for.
static void
ProcessControlCommand(const json& j, const sockaddr_in& src)
{
    // 1) Scene update: place/update every reported entity. Entities are
    //    processed before the packet command so that a brand-new origin_ID can
    //    appear and transmit within the same control datagram.
    bool moved = false;
    if (j.contains("entities"))
    {
        for (const auto& e : j["entities"])
        {
            moved = UpdateEntity(e) || moved;
        }
    }
    if (moved)
    {
        // Recompute the (ideal) beamforming vectors right away: beams are
        // otherwise refreshed only every BeamformingPeriodicity (100 ms), and
        // a packet sent immediately after a large position jump would go out
        // on a beam still pointing at the old position. The resulting HARQ
        // failures are fatal for the 23 dBm uplink under RLC UM (no
        // retransmission), so it is important to refresh before any tx
        g_beamHelper->Run();
    }

    // 2) Optional packet command
    if (j.contains("packet"))
    {
        HandlePacketCommand(j["packet"], src);
    }
}

// This function returns True if a shutdown command was received (see DOCS.md)
static bool
IsShutdown(const json& j, const sockaddr_in& src)
{
    if (j.value("msg_type", "control") != "shutdown")
    {
        return false;
    }
    std::cout << "v2x-bridge: shutdown command received" << std::endl;
    json ack = {{"msg_type", "shutdown_ack"}, {"sim_time_ms", NowMs()}};
    SendRealReply(src, ack);
    Simulator::Stop();
    return true;
}

// ---------------------------------------------------------------------------
// Real-time mode: recurring non-blocking poll
// ---------------------------------------------------------------------------

// Recurring 1 ms event: drains the real control socket, reacts to signals,
// and keeps the realtime simulator alive
static void
PollControlSocket()
{
    if (g_sigStop)
    {
        std::cout << "INFO: v2x-bridge: signal received, stopping" << std::endl;
        Simulator::Stop();
        return;
    }

    uint8_t buf[65536];
    sockaddr_in src{};
    socklen_t srcLen = sizeof(src);
    ssize_t n;
    while ((n = recvfrom(g_ctlFd, buf, sizeof(buf), MSG_DONTWAIT,
                         reinterpret_cast<sockaddr*>(&src), &srcLen)) > 0)
    {
        try
        {
            json j = json::parse(std::string(reinterpret_cast<char*>(buf), n));
            if (IsShutdown(j, src))
            {
                return;
            }
            ProcessControlCommand(j, src);
        }
        catch (const std::exception& e)
        {
            std::cerr << "ERROR: v2x-bridge: bad control datagram dropped: " << e.what() << std::endl;
        }
        srcLen = sizeof(src);
    }

    Simulator::Schedule(MilliSeconds(1), &PollControlSocket);
}

// ---------------------------------------------------------------------------------------------------
// External-clock mode (default): the sim clock follows the external application (e.g., OpenCDA/CARLA)
// ---------------------------------------------------------------------------------------------------

// This function extracts the external time a command refers to: the top-level "timestamp" if
// present, otherwise the newest "timestamp" among its entities. Returns an
// empty optional (std::nullopt) when the datagram carries no timestamp at all;
// this is not the same as time t=0, so that the caller can tell the two apart and run such a command
// at the current simulation time instead
static std::optional<double>
ExtractExtTimestamp(const json& j)
{
    if (j.contains("timestamp"))
    {
        return j["timestamp"].get<double>();
    }
    std::optional<double> ts;
    if (j.contains("entities"))
    {
        for (const auto& e : j["entities"])
        {
            if (e.contains("timestamp"))
            {
                double t = e["timestamp"].get<double>();
                ts = ts.has_value() ? std::max(*ts, t) : t;
            }
        }
    }
    return ts;
}

// This function converts a command's external timestamp into the simulation time at which the
// command must run. The very first timestamp ever received defines the origin:
// it is mapped to g_warmup, and every later one follows:
//     t_sim = g_warmup + (t_ext - t_ext_first)
// so both clocks tick at the same rate and only their origins differ.
// Two cases run at the current simulation time instead:
//   - no timestamp at all (empty optional): warned about once, because without
//     timestamps the simulation clock can never advance in this mode;
//   - a timestamp already behind the clock: either an out-of-order command, or
//     one landing inside the drain window that was just simulated. The latter is
//     normal, so the warning is printed only when the command is more than
//     drainMs late.
static Time
MapExtToSim(const std::optional<double>& extTs)
{
    if (!extTs.has_value())
    {
        static bool warned = false;
        if (!warned)
        {
            std::cerr << "WARNING: v2x-bridge: command without any timestamp in external-clock mode; "
                      << "executing at the current simulation time. Without timestamps the "
                      << "simulation clock cannot advance (use --realtime otherwise)."
                      << std::endl;
            warned = true;
        }
        return Simulator::Now();
    }
    if (!g_extEpochSet)
    {
        g_extEpoch0 = *extTs;
        g_extEpochSet = true;
        std::cout << "INFO: v2x-bridge: external clock locked, t_ext=" << std::fixed
                  << std::setprecision(3) << *extTs << " s <-> t_sim=" << g_warmup.GetSeconds()
                  << " s" << std::endl;
    }
    Time target = g_warmup + Seconds(*extTs - g_extEpoch0);
    if (target < Simulator::Now())
    {
        if (Simulator::Now() - target > g_drain)
        {
            std::cerr << "WARNING: v2x-bridge: external timestamp " << *extTs << " s is "
                      << (Simulator::Now() - target).GetMilliSeconds()
                      << " ms behind the simulation clock; executing now" << std::endl;
        }
        target = Simulator::Now();
    }
    return target;
}

// ns-3 event wrapper around one control command, scheduled by the gate at the
// simulation time MapExtToSim() returned for it
// Any exception raised while processing is caught here, so one malformed datagram
// is dropped instead of tearing down the simulation
static void
ExecuteControlEvent(json j, sockaddr_in src)
{
    try
    {
        ProcessControlCommand(j, src);
    }
    catch (const std::exception& e)
    {
        std::cerr << "v2x-bridge: bad control datagram dropped: " << e.what() << std::endl;
    }
}

// This function collects every datagram currently queued on the control (real UDP) socket
static void
DrainCtlSocket(std::vector<std::pair<std::string, sockaddr_in>>& batch)
{
    uint8_t buf[65536];
    sockaddr_in src{};
    socklen_t srcLen = sizeof(src);
    ssize_t n;
    while ((n = recvfrom(g_ctlFd, buf, sizeof(buf), MSG_DONTWAIT,
                         reinterpret_cast<sockaddr*>(&src), &srcLen)) > 0)
    {
        batch.emplace_back(std::string(reinterpret_cast<char*>(buf), n), src);
        srcLen = sizeof(src);
    }
}

// This function implements the gate of the external-clock mode. While this event blocks
// on the real socket, simulation time is frozen; each received command batch schedules
// its execution at the mapped sim time and the next gate right after it, so
// the event loop advances exactly as far as the external clock dictates
static void
ExternalSyncLoop()
{
    std::vector<std::pair<std::string, sockaddr_in>> batch;
    DrainCtlSocket(batch);
    while (batch.empty())
    {
        pollfd pfd{g_ctlFd, POLLIN, 0};
        poll(&pfd, 1, 100); // wake up regularly so Ctrl-C works while idle
        if (g_sigStop)
        {
            std::cout << "INFO: v2x-bridge: signal received, stopping" << std::endl;
            Simulator::Stop();
            return;
        }
        DrainCtlSocket(batch);
    }

    Time maxTarget = Simulator::Now();
    bool anyCommand = false;
    for (const auto& [data, src] : batch)
    {
        try
        {
            json j = json::parse(data);
            if (IsShutdown(j, src))
            {
                return;
            }
            Time target = MapExtToSim(ExtractExtTimestamp(j));
            Simulator::Schedule(target - Simulator::Now(), &ExecuteControlEvent, j, src);
            maxTarget = std::max(maxTarget, target);
            anyCommand = true;
        }
        catch (const std::exception& e)
        {
            std::cerr << "v2x-bridge: bad control datagram dropped: " << e.what() << std::endl;
        }
    }

    // Re-arm the gate just past the newest command (+ drain, so deliveries and
    // replies for this batch resolve now rather than on the next tick)
    Time next = anyCommand ? maxTarget + g_drain : Simulator::Now();
    Simulator::Schedule(next - Simulator::Now(), &ExternalSyncLoop);
}

int
main(int argc, char* argv[])
{
    // Default parameter values
    double gnbX = 0.0;
    double gnbY = 0.0;
    double gnbZ = 10.0;
    uint32_t maxUes = 10;
    uint16_t listenPort = 5555;
    std::string csvSend = "v2x-send.csv";
    std::string csvRecv = "v2x-recv.csv";
    double gnbTxPowerDbm = 40.0;
    double ueTxPowerDbm = 23.0;
    uint32_t gnbNumRows = 4;
    uint32_t gnbNumColumns = 8;
    uint32_t ueNumRows = 1;
    uint32_t ueNumColumns = 2;
    double gnbDowntiltDeg = 6.0;
    std::string scenario = "UMi";
    uint32_t channelUpdateMs = 0;
    bool shadowing = false;
    uint32_t warmupMs = 500;
    uint32_t drainMs = 20;

    // Command line options
    CommandLine cmd(__FILE__);
    cmd.AddValue("gnbX", "Base Station x position [m]", gnbX);
    cmd.AddValue("gnbY", "Base Station y position [m]", gnbY);
    cmd.AddValue("gnbZ", "Base Station z position (antenna height) [m]", gnbZ);
    cmd.AddValue("maxUes", "Maximum number of vehicles+VRUs (UE pool size)", maxUes);
    cmd.AddValue("listenPort", "Real OS UDP port for JSON control messages", listenPort);
    cmd.AddValue("simAppPort", "UDP port used by the simulated applications", g_simAppPort);
    cmd.AddValue("timeoutMs", "Per-packet delivery timeout before logging a failure [ms]",
                 g_timeoutMs);
    cmd.AddValue("csvSend", "Output CSV path for the send log", csvSend);
    cmd.AddValue("csvRecv", "Output CSV path for the receive/failure log", csvRecv);
    cmd.AddValue("gnbTxPowerDbm",
                 "gNB Tx power [dBm]; 40 dBm = 10 W over 60 MHz, a typical urban "
                 "macro/small-macro value (3GPP TR 38.802 ranges 33-44 dBm)",
                 gnbTxPowerDbm);
    cmd.AddValue("ueTxPowerDbm", "UE Tx power [dBm]; 23 dBm = power class 3 (TS 38.101-1)",
                 ueTxPowerDbm);
    cmd.AddValue("gnbNumRows", "gNB antenna array rows", gnbNumRows);
    cmd.AddValue("gnbNumColumns", "gNB antenna array columns", gnbNumColumns);
    cmd.AddValue("ueNumRows", "UE antenna array rows", ueNumRows);
    cmd.AddValue("ueNumColumns", "UE antenna array columns", ueNumColumns);
    cmd.AddValue("gnbDowntiltDeg", "gNB electrical downtilt [deg]", gnbDowntiltDeg);
    cmd.AddValue("scenario", "3GPP channel scenario: UMi or UMa", scenario);
    cmd.AddValue("channelUpdateMs",
                 "If >0, fading channel update period [ms] (larger = lighter CPU load "
                 "for real-time operation; 0 = model default)",
                 channelUpdateMs);
    cmd.AddValue("shadowing", "Enable log-normal shadowing", shadowing);
    cmd.AddValue("injected-loss-perc",
                 "Percentage [0-100] of the packets that reach their destination node "
                 "and are dropped there on purpose, on top of the losses of the NR "
                 "model itself (disabled with 0). Such packets are logged and replied as "
                 "\"lost\" with no latency; to change the loss pattern, --RngRun can be used.",
                 g_injectedLossPerc);
    cmd.AddValue("verbose", "Print per-packet progress to stdout", g_verbose);
    cmd.AddValue("realtime",
                 "Pace the simulation with the wall clock (ns3::RealtimeSimulatorImpl) "
                 "instead of following the external simulator's clock (default: off — "
                 "the sim clock follows the 'timestamp' field of incoming commands, "
                 "as required for OpenCDA/CARLA lockstep integration)",
                 g_realtime);
    cmd.AddValue("warmupMs",
                 "External-clock mode: simulation time mapped to the first received "
                 "timestamp [ms] (lets the RRC attach of the UE pool complete first)",
                 warmupMs);
    cmd.AddValue("drainMs",
                 "External-clock mode: extra simulation time processed after each "
                 "command batch so in-flight packets resolve (and replies are sent) "
                 "immediately; keep it below the external tick period [ms]",
                 drainMs);
    cmd.Parse(argc, argv);
    g_warmup = MilliSeconds(warmupMs);
    g_drain = MilliSeconds(drainMs);
    
    // Injected packet loss management (if activated)
    if (g_injectedLossPerc < 0.0 || g_injectedLossPerc > 100.0)
    {
        std::cerr << "FATAL ERROR: v2x-bridge: --injected-loss-perc must be a percentage in [0, 100], got "
                  << g_injectedLossPerc << std::endl;
        return 1;
    }
    g_lossRng = CreateObject<UniformRandomVariable>();
    g_lossRng->SetAttribute("Min", DoubleValue(0.0));
    g_lossRng->SetAttribute("Max", DoubleValue(100.0));

    if (g_realtime)
    {
        GlobalValue::Bind("SimulatorImplementationType",StringValue("ns3::RealtimeSimulatorImpl"));
    }

    if (channelUpdateMs > 0)
    {
        Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                           TimeValue(MilliSeconds(channelUpdateMs)));
    }
    // We set a large RLC UM buffer so that bursts are never dropped at the RLC queue
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    // Create the ns-3 node containers for the gNB and UEs
    NodeContainer gnbNodes;
    gnbNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(maxUes);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(gnbNodes);
    mobility.Install(ueNodes);
    gnbNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(gnbX, gnbY, gnbZ));
    // Idle UEs are parked in a small range a few meters from the gNB
    // (so that the mandatory t=0 attach succeeds)
    // they are then teleported to their real position the first time their origin_ID appears
    for (uint32_t i = 0; i < maxUes; i++)
    {
        ueNodes.Get(i)->GetObject<MobilityModel>()->SetPosition(
            Vector(gnbX + 5.0 + 0.5 * i, gnbY + 5.0, 1.5));
    }

    // We do here the NR/EPC setup
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> beamHelper = CreateObject<IdealBeamformingHelper>();
    g_beamHelper = beamHelper;
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(beamHelper);
    nrHelper->SetEpcHelper(epcHelper);

    // Band n78: 3.5 GHz centre, 60 MHz
    // The band holds a single component carrier (CC), itself a single bandwidth part
    // (BWP - no carrier aggregation, no BWP split), so the whole 60 MHz is served by
    // one single PHY/MAC pair
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(3.5e9, 60e6, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories(scenario, "Default", "ThreeGpp");
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(shadowing));
    channelHelper->AssignChannelsToBands({band});
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    beamHelper->SetAttribute("BeamformingMethod",
                             TypeIdValue(DirectPathBeamforming::GetTypeId()));
    epcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));

    // Urban antenna setup: planar gNB array with the 3GPP element pattern and
    // electrical downtilt; small quasi-omni UE array (considering a vehicle rooftop configuration)
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(gnbNumRows));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(gnbNumColumns));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<ThreeGppAntennaModel>()));
    nrHelper->SetGnbAntennaAttribute("DowntiltAngle",
                                     DoubleValue(gnbDowntiltDeg * M_PI / 180.0));
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(ueNumRows));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(ueNumColumns));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));

    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

    // Per-BWP attributes (numerology 1, TDD symmetric pattern, Tx powers).
    // The UEs inherit numerology and pattern from the gNB at attach time.
    NrHelper::GetGnbPhy(gnbDevs.Get(0), 0)->SetAttribute("Numerology", UintegerValue(1));
    NrHelper::GetGnbPhy(gnbDevs.Get(0), 0)->SetAttribute("TxPower", DoubleValue(gnbTxPowerDbm));
    NrHelper::GetGnbPhy(gnbDevs.Get(0), 0)
        ->SetAttribute("Pattern", StringValue("DL|S|UL|UL|DL|DL|S|UL|UL|DL|"));
    for (uint32_t i = 0; i < ueDevs.GetN(); i++)
    {
        NrHelper::GetUePhy(ueDevs.Get(i), 0)->SetAttribute("TxPower", DoubleValue(ueTxPowerDbm));
    }

    // EPC: remote host = the "BS"/MEC application endpoint, behind a
    // zero-delay 100 Gb/s link to the PGW (this approximates a high capacity fiber link)
    // This link can be configured differently by changing the values inside SetupRemoteHost(),
    // depending on the simulation needs
    auto [remoteHost, remoteHostIpv4] = epcHelper->SetupRemoteHost("100Gb/s", 2500, Seconds(0));
    g_remoteHost = remoteHost;
    // The address returned by SetupRemoteHost is currently (with 5G LENA v4.1.1) the
    // PGW's side of the PGW-remote host link, not the remote host's address; using
    // it direcly as a destination makes the PGW answer with ICMP port-unreachable
    g_remoteHostIp = remoteHost->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
    (void)remoteHostIpv4;

    InternetStackHelper internet;
    internet.Install(ueNodes);
    // AssignUeIpv4Address() also installs each UE's default route to the EPC (at least in 5G LENA v4.1.1)
    Ipv4InterfaceContainer ueIfaces = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));

    // Attach the whole pool now: 5G-LENA currently (as of v4.1.1) does not support post-run attach
    nrHelper->AttachToClosestGnb(ueDevs, gnbDevs);

    // Configure one dedicated low-latency bearer per UE with a TFT (Traffic Flow Template) on
    // the application port, so both UL and DL flows, including UE->PGW->UE traffic, are classified
    // onto it (all simulated sockets use g_simAppPort as source and destination)
    // TFT (4G terminology) is mentioned here because 5G LENA runs NR over an EPC core
    Ptr<NrEpcTft> tft = Create<NrEpcTft>();
    NrEpcTft::PacketFilter pf;
    pf.localPortStart = g_simAppPort;
    pf.localPortEnd = g_simAppPort;
    pf.remotePortStart = g_simAppPort;
    pf.remotePortEnd = g_simAppPort;
    tft->Add(pf);
    nrHelper->ActivateDedicatedEpsBearer(ueDevs, NrEpsBearer(NrEpsBearer::NGBR_LOW_LAT_EMBB),
                                         tft);

    // Simulated application sockets
    // One UDP socket per endpoint, bound to the app port; the same socket is
    // used both to receive and to send
    for (uint32_t i = 0; i < maxUes; i++)
    {
        UeSlot slot;
        slot.node = ueNodes.Get(i);
        slot.dev = ueDevs.Get(i);
        slot.ip = ueIfaces.GetAddress(i);
        slot.sock = Socket::CreateSocket(slot.node, UdpSocketFactory::GetTypeId());
        slot.sock->Bind(InetSocketAddress(Ipv4Address::GetAny(), g_simAppPort));
        slot.sock->SetRecvCallback(MakeCallback(&OnSimPacketReceived));
        g_slots.push_back(slot);
    }
    g_bsSock = Socket::CreateSocket(g_remoteHost, UdpSocketFactory::GetTypeId());
    g_bsSock->Bind(InetSocketAddress(Ipv4Address::GetAny(), g_simAppPort));
    g_bsSock->SetRecvCallback(MakeCallback(&OnSimPacketReceived));

    // Open the CSV files for logging
    g_csvSend.open(csvSend, std::ofstream::out | std::ofstream::trunc);
    g_csvRecv.open(csvRecv, std::ofstream::out | std::ofstream::trunc);
    if (!g_csvSend.is_open() || !g_csvRecv.is_open())
    {
        std::cerr << "FATAL ERROR: v2x-bridge: cannot open CSV output files (" << csvSend << ", " << csvRecv
                  << ")" << std::endl;
        return 1;
    }
    g_csvSend << "packet_id,receiver,sender,size_bytes,tx_sim_time_ms,type\n";
    g_csvSend.flush();
    g_csvRecv << "packet_id,rx_sim_time_ms,latency_ms,type,status\n";
    g_csvRecv.flush();

    // Real socket for receiving control UDP datagrams from the external application (e.g., CARLA/OpenCDA)
    g_ctlFd = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_ctlFd < 0)
    {
        std::cerr << "FATAL ERROR: v2x-bridge: socket() failed: " << std::strerror(errno) << std::endl;
        return 1;
    }
    fcntl(g_ctlFd, F_SETFL, fcntl(g_ctlFd, F_GETFL, 0) | O_NONBLOCK);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(listenPort);
    if (bind(g_ctlFd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
    {
        std::cerr << "FATAL ERROR: v2x-bridge: cannot bind UDP port " << listenPort << ": "
                  << std::strerror(errno) << std::endl;
        return 1;
    }

    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    std::cout << "INFO: v2x-bridge: gNB at (" << gnbX << ", " << gnbY << ", " << gnbZ << ") m, "
              << maxUes << " UE slots, band n78 3.5 GHz / 60 MHz / numerology 1, TDD "
              << "DL|S|UL|UL|DL|DL|S|UL|UL|DL|, scenario " << scenario << std::endl;
    std::cout << "v2x-bridge: BS/MEC endpoint IP " << g_remoteHostIp << ", UE IPs "
              << g_slots.front().ip << " .. " << g_slots.back().ip << std::endl;
    if (g_injectedLossPerc > 0.0)
    {
        std::cout << "INFO: v2x-bridge: injected packet loss " << std::fixed << std::setprecision(2)
                  << g_injectedLossPerc << " % (RngRun " << RngSeedManager::GetRun()
                  << "), applied at the destination node on top of the NR model losses"
                  << std::endl;
    }
    std::cout << "INFO: v2x-bridge: listening for JSON control messages on UDP 0.0.0.0:" << listenPort
              << (g_realtime
                      ? " (real-time / wall-clock pacing)"
                      : " (external-clock mode: sim time follows the commands' 'timestamp')")
              << ". Ctrl-C or {\"msg_type\":\"shutdown\"} to stop." << std::endl;

    if (g_realtime)
    {
        // First poll
        // Then, each poll re-schedules itself, keeping the simulator alive
        Simulator::Schedule(MilliSeconds(1), &PollControlSocket);
    }
    else
    {
        // First gate event after the warm-up window (RRC attach of the pool completes
        // during it)
        // Each gate event re-arms itself after the command batch it admits
        Simulator::Schedule(g_warmup, &ExternalSyncLoop);
    }

    Simulator::Run();

    // Code below this line will be executed after the V2X bridge simulation harness terminates
    if (g_injectedLossPerc > 0.0)
    {
        std::cout << "INFO: v2x-bridge: injected loss dropped " << g_lossDrops << " of the "
                  << g_lossDraws << " packet(s) that reached their destination ("
                  << std::fixed << std::setprecision(2)
                  << (g_lossDraws ? 100.0 * double(g_lossDrops) / double(g_lossDraws) : 0.0)
                  << " %, target " << g_injectedLossPerc << " %)" << std::endl;
    }

    // Clean shutdown: flush/close everything before returning
    g_csvSend.close();
    g_csvRecv.close();
    close(g_ctlFd);
    Simulator::Destroy();
    std::cout << "INFO: v2x-bridge: done." << std::endl;
    return 0;
}
