// sim.c - RS-485 multidrop + routed overlay simulation (adjacency-only config)
//
// Build: gcc -std=c11 -Wall -Wextra -O2 -pthread sim.c -o sim
// Run:   ./sim
//
// Properties:
//  - L2 (RS-485 bus): shared medium broadcast. Every node on a bus receives every frame,
//    but ACCEPTS it only if unit_id (Modbus address) matches its position.
//  - L3 (overlay): src/dst/txid/type/ttl/from/target_bus + payload.
//  - Only config input is the adjacency list (neighbor + local out_port).
//  - Physical buses (BUS1..BUSn) are DERIVED from adjacency by union-find over interfaces (pos,port).
//  - Routing uses: from + MUST-DECREASE-METRIC rule.
//      metric = dist_to_bus[pos][target_bus] (computed once by BFS).
//      forward only to a neighbor with strictly smaller metric.
//
// Notes:
//  - BUS numbering is derived (BUS1/BUS2/...) and may not match your drawing labels.
//  - This is a simulation; bus �hub� broadcasts frames, nodes filter by unit_id.

//#include "global.h"
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <cstring>
#include <array>
#define _Static_assert(cond, msg) static_assert((cond), msg)

extern "C" {
#include "network.h"
#include "layer2.h"
#include "layer1.h"
}

#if 0
enum { PORT_NONE = 0, PORT1 = 1, PORT2 = 2 };
enum { MSG_REQ = 0, MSG_RSP = 1 };

enum { MAX_POS = 8 };          // positions 1..7 usable
enum { MAX_NEI = 8 };
enum { MAX_BUSES = 16 };       // derived buses (components)
enum { MAX_CANDS = 8 };
enum { PAYLOAD_MAX = 64 };

enum { BUS_NONE = 0 };         // bus ids are 1..g_bus_count

static pthread_mutex_t g_print_lock = PTHREAD_MUTEX_INITIALIZER;
static atomic_bool g_running = true;

static void logf_pos(uint8_t pos, const char* fmt, ...)
{
    pthread_mutex_lock(&g_print_lock);
    va_list ap;
    va_start(ap, fmt);
    fprintf(stdout, "[pos%u] ", (unsigned)pos);
    vfprintf(stdout, fmt, ap);
    fprintf(stdout, "\n");
    va_end(ap);
    pthread_mutex_unlock(&g_print_lock);
}

static void logf_bus(const char* name, const char* fmt, ...)
{
    pthread_mutex_lock(&g_print_lock);
    va_list ap;
    va_start(ap, fmt);
    fprintf(stdout, "[%s] ", name);
    vfprintf(stdout, fmt, ap);
    fprintf(stdout, "\n");
    va_end(ap);
    pthread_mutex_unlock(&g_print_lock);
}

static size_t my_strnlen(const char* s, size_t maxlen)
{
    size_t n = 0U;
    if (s == NULL) return 0U;
    while ((n < maxlen) && (s[n] != '\0')) { ++n; }
    return n;
}

#pragma pack(push, 1)
typedef struct
{
    uint8_t  src;        // original sender position (end-to-end)
    uint8_t  dst;        // final destination position (end-to-end)
    uint16_t txid;       // transaction id
    uint8_t  type;       // MSG_REQ / MSG_RSP
    uint8_t  ttl;        // hop limit
    uint8_t  from;       // previous hop (changes per hop)
    uint8_t  target_bus; // derived bus id chosen for dst
    uint8_t  len;
    char     payload[PAYLOAD_MAX];
} NetL3Pdu;

typedef struct
{
    uint8_t  unit_id;    // Modbus address on this bus (L2 destination) == next hop or final dst
    uint8_t  func;       // pretend modbus function code (0x41 = NET)
    NetL3Pdu pdu;        // routed payload
    uint16_t crc16;      // unused in sim
} Rs485L2Frame;
#pragma pack(pop)

typedef struct
{
    uint8_t nb;          // neighbor position
    uint8_t out_port;    // local port used to reach nb (PORT1/PORT2)
} Adj;

typedef struct
{
    uint8_t nextHop;
    uint8_t outPort;
} RouteCand;

typedef struct
{
    RouteCand cands[MAX_CANDS];
    uint8_t   count;
} RouteEntry;

typedef struct
{
    uint8_t pos;

    int fd_port1;
    int fd_port2;

    Adj adj[MAX_NEI];
    uint8_t adj_count;

    RouteEntry toBus[MAX_BUSES];   // toBus[busId] => candidates (already decreasing metric)

    uint16_t txid_next;

    bool is_master;
} Node;

typedef struct
{
    const char* name;
    int bus_fds[32];
    uint8_t count;
} Bus;

/* ---------- Global derived topology ---------- */
static uint8_t g_bus_count = 0U;

// bus_of_iface[pos][port] = busId (1..g_bus_count) or 0 if port not present
static uint8_t g_bus_of_iface[MAX_POS][3];

// pos_on_bus[pos][busId] = true if any iface of pos belongs to that bus
static bool g_pos_on_bus[MAX_POS][MAX_BUSES];

// dist_to_bus[pos][busId] = min hop count from pos to reach ANY node on that bus; 0 if already on bus; 0xFF if unreachable
static uint8_t g_dist_to_bus[MAX_POS][MAX_BUSES];

/* -------------------- Adjacency (ONLY CONFIG INPUT) -------------------- */
// Update this to change wiring/topology.
// Must be reciprocal for interface derivation to work deterministically.
static void add_adj(Node* n, uint8_t nb, uint8_t out_port)
{
    if (n->adj_count < MAX_NEI)
    {
        n->adj[n->adj_count].nb = nb;
        n->adj[n->adj_count].out_port = out_port;
        n->adj_count++;
    }
}

static void build_adjacency(Node nodes[MAX_POS])
{
    // Clear
    for (uint8_t p = 0U; p < MAX_POS; ++p) { nodes[p].adj_count = 0U; }

    // Topology matching your last diagram:
    // - Segment A: (7,P1) <-> (1,P1) <-> (3,P1) <-> (7,P1)
    // - Segment B: (3,P2) <-> (2,P1) <-> (5,P1) <-> (3,P2)  (pos3 uses P2 to reach 2/5)
    // - Segment C: (1,P2) <-> (2,P2)
    //
    // pos7
    add_adj(&nodes[7], 1, PORT1);
    add_adj(&nodes[7], 3, PORT1);

    // pos1
    add_adj(&nodes[1], 7, PORT1);
    add_adj(&nodes[1], 3, PORT1);
    add_adj(&nodes[1], 2, PORT2);

    // pos3
    add_adj(&nodes[3], 7, PORT1);
    add_adj(&nodes[3], 1, PORT1);
    add_adj(&nodes[3], 2, PORT2);
    add_adj(&nodes[3], 5, PORT2);

    // pos2
    add_adj(&nodes[2], 1, PORT2);
    add_adj(&nodes[2], 3, PORT1);
    add_adj(&nodes[2], 5, PORT1);

    // pos5
    add_adj(&nodes[5], 3, PORT1);
    add_adj(&nodes[5], 2, PORT1);
}

/* -------------------- DSU for deriving buses from adjacency -------------------- */

enum { IFACE_MAX = MAX_POS * 2 }; // index = pos*2 + (port-1), pos in [0..7], port in {1,2}

static uint8_t dsu_parent[IFACE_MAX];
static uint8_t dsu_rankv[IFACE_MAX];

static uint8_t iface_index(uint8_t pos, uint8_t port)
{
    // port is 1 or 2
    return (uint8_t)(pos * 2U + (uint8_t)(port - 1U));
}

static void dsu_init(void)
{
    for (uint8_t i = 0U; i < IFACE_MAX; ++i)
    {
        dsu_parent[i] = i;
        dsu_rankv[i] = 0U;
    }
}

static uint8_t dsu_find(uint8_t x)
{
    while (dsu_parent[x] != x)
    {
        dsu_parent[x] = dsu_parent[dsu_parent[x]];
        x = dsu_parent[x];
    }
    return x;
}

static void dsu_union(uint8_t a, uint8_t b)
{
    uint8_t ra = dsu_find(a);
    uint8_t rb = dsu_find(b);
    if (ra == rb) return;

    if (dsu_rankv[ra] < dsu_rankv[rb])
    {
        dsu_parent[ra] = rb;
    }
    else if (dsu_rankv[ra] > dsu_rankv[rb])
    {
        dsu_parent[rb] = ra;
    }
    else
    {
        dsu_parent[rb] = ra;
        dsu_rankv[ra] = (uint8_t)(dsu_rankv[ra] + 1U);
    }
}

static bool find_reciprocal_port(const Node nodes[MAX_POS], uint8_t a, uint8_t b, uint8_t* out_port_b_to_a)
{
    // Find in node b: an adjacency entry pointing back to a; return its out_port.
    const Node* nb = &nodes[b];
    for (uint8_t k = 0U; k < nb->adj_count; ++k)
    {
        if (nb->adj[k].nb == a)
        {
            *out_port_b_to_a = nb->adj[k].out_port;
            return true;
        }
    }
    return false;
}

static const char* bus_tok(uint8_t busId)
{
    // For initializer printing, show BUS1/BUS2...
    static char buf[16];
    if (busId == BUS_NONE) return "BUS_NONE";
    (void)snprintf(buf, sizeof(buf), "BUS%u", (unsigned)busId);
    return buf;
}

static const char* port_tok(uint8_t port)
{
    if (port == PORT1) return "PORT1";
    if (port == PORT2) return "PORT2";
    return "PORT_NONE";
}

static void clear_globals(void)
{
    memset(g_bus_of_iface, 0, sizeof(g_bus_of_iface));
    memset(g_pos_on_bus, 0, sizeof(g_pos_on_bus));
    memset(g_dist_to_bus, 0xFF, sizeof(g_dist_to_bus));
    g_bus_count = 0U;
}

static void derive_buses_from_adjacency(const Node nodes[MAX_POS])
{
    clear_globals();
    dsu_init();

    // Union interfaces according to adjacency (requires reciprocal port lookup).
    for (uint8_t a = 1U; a < MAX_POS; ++a)
    {
        const Node* na = &nodes[a];
        for (uint8_t k = 0U; k < na->adj_count; ++k)
        {
            const uint8_t b = na->adj[k].nb;
            const uint8_t pa = na->adj[k].out_port;

            if ((pa != PORT1) && (pa != PORT2)) continue;
            if (b == 0U || b >= MAX_POS) continue;

            uint8_t pb = 0U;
            if (!find_reciprocal_port(nodes, a, b, &pb))
            {
                // If not reciprocal, we skip union (topology ambiguous).
                continue;
            }
            if ((pb != PORT1) && (pb != PORT2)) continue;

            dsu_union(iface_index(a, pa), iface_index(b, pb));
        }
    }

    // Assign sequential bus IDs to DSU roots for all interfaces that appear in adjacency.
    // We treat an interface as "present" if it was used as an out_port in any adjacency entry.
    bool iface_present[MAX_POS][3];
    memset(iface_present, 0, sizeof(iface_present));

    for (uint8_t a = 1U; a < MAX_POS; ++a)
    {
        const Node* na = &nodes[a];
        for (uint8_t k = 0U; k < na->adj_count; ++k)
        {
            const uint8_t pa = na->adj[k].out_port;
            if ((pa == PORT1) || (pa == PORT2))
            {
                iface_present[a][pa] = true;
            }

            // also mark reciprocal iface as present if found
            const uint8_t b = na->adj[k].nb;
            uint8_t pb = 0U;
            if (b > 0U && b < MAX_POS && find_reciprocal_port(nodes, a, b, &pb))
            {
                if ((pb == PORT1) || (pb == PORT2))
                {
                    iface_present[b][pb] = true;
                }
            }
        }
    }

    uint8_t root_to_bus[IFACE_MAX];
    for (uint8_t i = 0U; i < IFACE_MAX; ++i) { root_to_bus[i] = 0U; }

    for (uint8_t pos = 1U; pos < MAX_POS; ++pos)
    {
        for (uint8_t port = PORT1; port <= PORT2; ++port)
        {
            if (!iface_present[pos][port]) continue;

            const uint8_t idx = iface_index(pos, port);
            const uint8_t root = dsu_find(idx);

            uint8_t busId = root_to_bus[root];
            if (busId == 0U)
            {
                if (g_bus_count + 1U >= MAX_BUSES)
                {
                    // clamp
                    continue;
                }
                g_bus_count = (uint8_t)(g_bus_count + 1U);
                busId = g_bus_count;
                root_to_bus[root] = busId;
            }

            g_bus_of_iface[pos][port] = busId;
            g_pos_on_bus[pos][busId] = true;
        }
    }
}

/* -------------------- Distances to each bus (metric) -------------------- */

static void bfs_dist_to_bus(const Node nodes[MAX_POS], uint8_t busId, uint8_t out_dist[MAX_POS])
{
    const uint8_t INF = 0xFF;
    for (uint8_t i = 0U; i < MAX_POS; ++i) out_dist[i] = INF;

    uint8_t q[MAX_POS];
    uint8_t qh = 0U, qt = 0U;

    // multi-source BFS: all positions that are on this bus start at dist 0
    for (uint8_t pos = 1U; pos < MAX_POS; ++pos)
    {
        if (g_pos_on_bus[pos][busId])
        {
            out_dist[pos] = 0U;
            q[qt++] = pos;
        }
    }

    while (qh != qt)
    {
        const uint8_t u = q[qh++];
        const Node* nu = &nodes[u];

        for (uint8_t k = 0U; k < nu->adj_count; ++k)
        {
            const uint8_t v = nu->adj[k].nb;
            if (v == 0U || v >= MAX_POS) continue;

            if (out_dist[v] == INF)
            {
                out_dist[v] = (uint8_t)(out_dist[u] + 1U);
                q[qt++] = v;
            }
        }
    }
}

static void build_all_dist_to_bus(const Node nodes[MAX_POS])
{
    // g_dist_to_bus[pos][busId]
    for (uint8_t busId = 1U; busId <= g_bus_count; ++busId)
    {
        uint8_t dist[MAX_POS];
        bfs_dist_to_bus(nodes, busId, dist);

        for (uint8_t pos = 0U; pos < MAX_POS; ++pos)
        {
            g_dist_to_bus[pos][busId] = dist[pos];
        }
    }
}

/* -------------------- Routing table toBus[bus] (built once) -------------------- */

typedef struct
{
    uint8_t nextHop;
    uint8_t outPort;
    uint8_t cost; // for printing/debug only
} PrintCand;

static void sort_print_cands(PrintCand* c, uint8_t count)
{
    // bubble sort: cost asc, then nextHop asc
    for (uint8_t i = 0U; i < count; ++i)
    {
        for (uint8_t j = (uint8_t)(i + 1U); j < count; ++j)
        {
            const bool swap =
                (c[j].cost < c[i].cost) ||
                ((c[j].cost == c[i].cost) && (c[j].nextHop < c[i].nextHop));
            if (swap)
            {
                const PrintCand tmp = c[i];
                c[i] = c[j];
                c[j] = tmp;
            }
        }
    }
}

static void build_toBus_once(Node nodes[MAX_POS])
{
    // For each node s and each busId:
    //  - if already on busId: count = 0
    //  - else: include ONLY neighbors that strictly decrease the metric (must-decrease rule)
    //          and sort by metric.

    for (uint8_t s = 1U; s < MAX_POS; ++s)
    {
        for (uint8_t busId = 1U; busId <= g_bus_count; ++busId)
        {
            nodes[s].toBus[busId].count = 0U;

            if (g_pos_on_bus[s][busId])
            {
                // already on this bus; no routing needed
                continue;
            }

            const uint8_t my = g_dist_to_bus[s][busId];
            if (my == 0xFF) continue; // unreachable

            const Node* ns = &nodes[s];
            PrintCand tmp[MAX_CANDS];
            uint8_t tmpCount = 0U;

            for (uint8_t k = 0U; k < ns->adj_count; ++k)
            {
                const uint8_t nh = ns->adj[k].nb;
                const uint8_t outp = ns->adj[k].out_port;
                if (nh == 0U || nh >= MAX_POS) continue;
                if ((outp != PORT1) && (outp != PORT2)) continue;

                const uint8_t nd = g_dist_to_bus[nh][busId];
                // MUST-DECREASE
                if (nd < my)
                {
                    if (tmpCount < MAX_CANDS)
                    {
                        tmp[tmpCount].nextHop = nh;
                        tmp[tmpCount].outPort = outp;
                        tmp[tmpCount].cost = (uint8_t)(1U + nd);
                        tmpCount++;
                    }
                }
            }

            sort_print_cands(tmp, tmpCount);

            RouteEntry* re = &nodes[s].toBus[busId];
            re->count = tmpCount;
            for (uint8_t i = 0U; i < tmpCount; ++i)
            {
                re->cands[i].nextHop = tmp[i].nextHop;
                re->cands[i].outPort = tmp[i].outPort;
            }
        }
    }
}

static void dump_toBus_table_as_initializers(const Node nodes[MAX_POS], const uint8_t used_pos[], size_t used_cnt)
{
    fprintf(stdout, "\n=== STATIC ROUTING TABLE (toBus initializers) ===\n\n");

    for (size_t ui = 0U; ui < used_cnt; ++ui)
    {
        const uint8_t p = used_pos[ui];
        fprintf(stdout, "/* ===================== pos%u ===================== */\n", (unsigned)p);

        for (uint8_t busId = 1U; busId <= g_bus_count; ++busId)
        {
            const RouteEntry* re = &nodes[p].toBus[busId];

            fprintf(stdout, "toBus[%s] = (RouteEntry){ .count = %u, .cands = { ",
                bus_tok(busId), (unsigned)re->count);

            if (re->count == 0U)
            {
                fprintf(stdout, "} };\n\n");
                continue;
            }

            fprintf(stdout, "\n");
            for (uint8_t i = 0U; i < re->count; ++i)
            {
                const uint8_t nh = re->cands[i].nextHop;
                const uint8_t outp = re->cands[i].outPort;
                const uint8_t cost = (g_dist_to_bus[nh][busId] == 0xFF) ? 0xFF : (uint8_t)(1U + g_dist_to_bus[nh][busId]);

                fprintf(stdout, "  { .nextHop = %u, .outPort = %s, .cost = %u }%s\n",
                    (unsigned)nh, port_tok(outp), (unsigned)cost,
                    (i + 1U == re->count) ? "" : ",");
            }
            fprintf(stdout, "}};\n\n");
        }

        fprintf(stdout, "\n");
    }
}

/* -------------------- Choose target bus for dst (derived) -------------------- */

static uint8_t choose_target_bus(uint8_t src, uint8_t dst)
{
    // Candidate buses = buses that dst is on; choose bus with minimal dist_to_bus[src][bus]
    uint8_t bestBus = BUS_NONE;
    uint8_t bestDist = 0xFF;

    if (dst == 0U || dst >= MAX_POS) return BUS_NONE;

    for (uint8_t busId = 1U; busId <= g_bus_count; ++busId)
    {
        if (!g_pos_on_bus[dst][busId]) continue;

        const uint8_t d = g_dist_to_bus[src][busId];
        if (d == 0xFF) continue;

        if ((bestBus == BUS_NONE) || (d < bestDist) || ((d == bestDist) && (busId < bestBus)))
        {
            bestBus = busId;
            bestDist = d;
        }
    }

    return bestBus;
}

static uint8_t local_port_for_bus(uint8_t pos, uint8_t busId)
{
    // choose which local port is attached to busId (if any)
    if (g_bus_of_iface[pos][PORT1] == busId) return PORT1;
    if (g_bus_of_iface[pos][PORT2] == busId) return PORT2;
    return PORT_NONE;
}

/* -------------------- �must decrease metric� next hop pick -------------------- */

static bool pick_next_hop_decreasing(const Node* n, uint8_t target_bus, uint8_t came_from,
    uint8_t* out_nextHop, uint8_t* out_outPort)
{
    if (target_bus == BUS_NONE || target_bus > g_bus_count) return false;

    const uint8_t my = g_dist_to_bus[n->pos][target_bus];
    if (my == 0xFF) return false;
    if (my == 0U) return false; // already on bus, should direct-deliver instead

    const RouteEntry* re = &n->toBus[target_bus];
    if (re->count == 0U) return false;

    // First pass: avoid bounce-back
    for (uint8_t i = 0U; i < re->count; ++i)
    {
        const uint8_t nh = re->cands[i].nextHop;
        if (nh == came_from) continue;

        const uint8_t nd = g_dist_to_bus[nh][target_bus];
        if (nd < my)
        {
            *out_nextHop = nh;
            *out_outPort = re->cands[i].outPort;
            return true;
        }
    }

    // Second pass: allow came_from if it still decreases (should be rare)
    for (uint8_t i = 0U; i < re->count; ++i)
    {
        const uint8_t nh = re->cands[i].nextHop;
        const uint8_t nd = g_dist_to_bus[nh][target_bus];
        if (nd < my)
        {
            *out_nextHop = nh;
            *out_outPort = re->cands[i].outPort;
            return true;
        }
    }

    return false;
}

/* -------------------- Bus hub (broadcast) -------------------- */

static void* bus_thread(void* arg)
{
    Bus* b = (Bus*)arg;

    logf_bus(b->name, "started with %u ports", (unsigned)b->count);

    while (atomic_load(&g_running))
    {
        fd_set rfds;
        FD_ZERO(&rfds);

        int maxfd = -1;
        for (uint8_t i = 0U; i < b->count; ++i)
        {
            FD_SET(b->bus_fds[i], &rfds);
            if (b->bus_fds[i] > maxfd) maxfd = b->bus_fds[i];
        }

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 200000; // 200ms

        const int rc = select(maxfd + 1, &rfds, NULL, NULL, &tv);
        if (rc <= 0) continue;

        for (uint8_t i = 0U; i < b->count; ++i)
        {
            const int fd = b->bus_fds[i];
            if (!FD_ISSET(fd, &rfds)) continue;

            Rs485L2Frame fr;
            const ssize_t r = recv(fd, &fr, (ssize_t)sizeof(fr), 0);
            if (r != (ssize_t)sizeof(fr)) continue;

            // broadcast to everyone else on the bus
            for (uint8_t j = 0U; j < b->count; ++j)
            {
                if (j == i) continue;
                (void)send(b->bus_fds[j], &fr, (size_t)sizeof(fr), 0);
            }
        }
    }

    logf_bus(b->name, "stopping");
    return NULL;
}

static bool attach_port_to_bus(Bus* bus, Node* node, uint8_t port)
{
    int sp[2];
    if (socketpair(AF_UNIX, SOCK_DGRAM, 0, sp) != 0)
    {
        return false;
    }

    const int node_fd = sp[0];
    const int bus_fd = sp[1];

    if (bus->count >= 32)
    {
        close(node_fd);
        close(bus_fd);
        return false;
    }

    bus->bus_fds[bus->count++] = bus_fd;

    if (port == PORT1) node->fd_port1 = node_fd;
    else if (port == PORT2) node->fd_port2 = node_fd;
    else
    {
        close(node_fd);
        close(bus_fd);
        return false;
    }

    return true;
}

/* -------------------- Node send/recv -------------------- */

static bool node_send_on_port(const Node* n, uint8_t outPort, const Rs485L2Frame* fr)
{
    int fd = -1;
    if (outPort == PORT1) fd = n->fd_port1;
    else if (outPort == PORT2) fd = n->fd_port2;
    if (fd < 0) return false;

    const ssize_t w = send(fd, fr, (size_t)sizeof(*fr), 0);
    return (w == (ssize_t)sizeof(*fr));
}

static bool send_direct_on_target_bus(Node* n, uint8_t target_bus, uint8_t dst, const NetL3Pdu* pdu)
{
    const uint8_t outp = local_port_for_bus(n->pos, target_bus);
    if (outp == PORT_NONE) return false;

    Rs485L2Frame fr;
    memset(&fr, 0, sizeof(fr));
    fr.unit_id = dst; // final dst on this bus
    fr.func = 0x41;
    fr.pdu = *pdu;
    fr.crc16 = 0;

    return node_send_on_port(n, outp, &fr);
}

static bool send_toward_target_bus(Node* n, uint8_t target_bus, uint8_t came_from, const NetL3Pdu* pdu, uint8_t* out_l2_unit)
{
    uint8_t nh = 0U, outp = 0U;
    if (!pick_next_hop_decreasing(n, target_bus, came_from, &nh, &outp)) return false;

    Rs485L2Frame fr;
    memset(&fr, 0, sizeof(fr));
    fr.unit_id = nh; // next hop address on this bus
    fr.func = 0x41;
    fr.pdu = *pdu;
    fr.crc16 = 0;

    if (out_l2_unit != NULL) { *out_l2_unit = nh; }
    return node_send_on_port(n, outp, &fr);
}

static void node_send_req(Node* n, uint8_t dst, const char* text)
{
    NetL3Pdu p;
    memset(&p, 0, sizeof(p));
    p.src = n->pos;
    p.dst = dst;
    p.txid = n->txid_next++;
    p.type = MSG_REQ;
    p.ttl = 20;
    p.from = n->pos;

    const size_t L = my_strnlen(text, PAYLOAD_MAX - 1U);
    p.len = (uint8_t)L;
    memcpy(p.payload, text, L);
    p.payload[L] = '\0';

    const uint8_t target_bus = choose_target_bus(n->pos, dst);
    if (target_bus == BUS_NONE)
    {
        logf_pos(n->pos, "REQ txid=%u dst=pos%u: NO ROUTE", (unsigned)p.txid, (unsigned)dst);
        return;
    }
    p.target_bus = target_bus;

    bool ok = false;
    uint8_t l2_unit = 0U;

    if (g_pos_on_bus[n->pos][target_bus])
    {
        ok = send_direct_on_target_bus(n, target_bus, dst, &p);
        l2_unit = dst;
    }
    else
    {
        ok = send_toward_target_bus(n, target_bus, 0U, &p, &l2_unit);
    }

    logf_pos(n->pos, "SEND REQ txid=%u dst=pos%u target=%s L2.unit_id=pos%u '%s' %s",
        (unsigned)p.txid, (unsigned)dst, bus_tok(target_bus), (unsigned)l2_unit, p.payload,
        ok ? "OK" : "FAIL");
}

static void node_send_rsp(Node* n, uint8_t dst, uint16_t txid, const char* text)
{
    NetL3Pdu p;
    memset(&p, 0, sizeof(p));
    p.src = n->pos;
    p.dst = dst;
    p.txid = txid;
    p.type = MSG_RSP;
    p.ttl = 20;
    p.from = n->pos;

    const size_t L = my_strnlen(text, PAYLOAD_MAX - 1U);
    p.len = (uint8_t)L;
    memcpy(p.payload, text, L);
    p.payload[L] = '\0';

    const uint8_t target_bus = choose_target_bus(n->pos, dst);
    if (target_bus == BUS_NONE)
    {
        logf_pos(n->pos, "RSP txid=%u dst=pos%u: NO ROUTE", (unsigned)txid, (unsigned)dst);
        return;
    }
    p.target_bus = target_bus;

    bool ok = false;
    uint8_t l2_unit = 0U;

    if (g_pos_on_bus[n->pos][target_bus])
    {
        ok = send_direct_on_target_bus(n, target_bus, dst, &p);
        l2_unit = dst;
    }
    else
    {
        ok = send_toward_target_bus(n, target_bus, 0U, &p, &l2_unit);
    }

    logf_pos(n->pos, "SEND RSP txid=%u dst=pos%u target=%s L2.unit_id=pos%u '%s' %s",
        (unsigned)p.txid, (unsigned)dst, bus_tok(target_bus), (unsigned)l2_unit, p.payload,
        ok ? "OK" : "FAIL");
}

static void handle_l2_frame(Node* n, const Rs485L2Frame* fr)
{
    // L2 filtering: accept only if modbus address matches me
    if (fr->unit_id != n->pos) return;

    NetL3Pdu p = fr->pdu;

    if (p.ttl == 0U)
    {
        logf_pos(n->pos, "DROP ttl=0 (dst=pos%u txid=%u)", (unsigned)p.dst, (unsigned)p.txid);
        return;
    }

    if (p.dst == n->pos)
    {
        if (p.type == MSG_REQ)
        {
            logf_pos(n->pos, "RECV REQ txid=%u from=pos%u origsrc=pos%u target=%s '%s'",
                (unsigned)p.txid, (unsigned)p.from, (unsigned)p.src, bus_tok(p.target_bus), p.payload);

            char rsp[PAYLOAD_MAX];
            (void)snprintf(rsp, sizeof(rsp), "ACK from pos%u", (unsigned)n->pos);
            node_send_rsp(n, p.src, p.txid, rsp);
        }
        else
        {
            logf_pos(n->pos, "RECV RSP txid=%u from=pos%u '%s'",
                (unsigned)p.txid, (unsigned)p.from, p.payload);
        }
        return;
    }

    // Forward (from + must-decrease-metric)
    const uint8_t came_from = p.from;

    // keep same target_bus (chosen by original sender)
    const uint8_t target_bus = p.target_bus;
    if ((target_bus == BUS_NONE) || (target_bus > g_bus_count))
    {
        logf_pos(n->pos, "DROP: invalid target_bus");
        return;
    }

    p.ttl = (uint8_t)(p.ttl - 1U);
    p.from = n->pos;

    bool ok = false;
    uint8_t l2_unit = 0U;

    if (g_pos_on_bus[n->pos][target_bus])
    {
        ok = send_direct_on_target_bus(n, target_bus, p.dst, &p);
        l2_unit = p.dst;
    }
    else
    {
        ok = send_toward_target_bus(n, target_bus, came_from, &p, &l2_unit);
    }

    logf_pos(n->pos, "FWD %s txid=%u toward %s L2.unit_id=pos%u %s",
        (p.type == MSG_REQ) ? "REQ" : "RSP",
        (unsigned)p.txid, bus_tok(target_bus), (unsigned)l2_unit,
        ok ? "OK" : "FAIL");
}

typedef struct
{
    Node* self;
} NodeThreadArg;

static void* node_thread(void* arg)
{
    NodeThreadArg* a = (NodeThreadArg*)arg;
    Node* n = a->self;

    // small startup delay
    struct timespec ts = { .tv_sec = 0, .tv_nsec = 200 * 1000 * 1000 };
    nanosleep(&ts, NULL);

    // simple tests
    if (n->is_master && n->pos == 7)
    {
        node_send_req(n, 5, "HELLO 7->5");
    }
    if (n->is_master && n->pos == 2)
    {
        node_send_req(n, 1, "HELLO 2->1");
    }

    while (atomic_load(&g_running))
    {
        fd_set rfds;
        FD_ZERO(&rfds);

        int maxfd = -1;
        if (n->fd_port1 >= 0) { FD_SET(n->fd_port1, &rfds); if (n->fd_port1 > maxfd) maxfd = n->fd_port1; }
        if (n->fd_port2 >= 0) { FD_SET(n->fd_port2, &rfds); if (n->fd_port2 > maxfd) maxfd = n->fd_port2; }

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 200000;

        const int rc = select(maxfd + 1, &rfds, NULL, NULL, &tv);
        if (rc <= 0) continue;

        if ((n->fd_port1 >= 0) && FD_ISSET(n->fd_port1, &rfds))
        {
            Rs485L2Frame fr;
            const ssize_t r = recv(n->fd_port1, &fr, (ssize_t)sizeof(fr), 0);
            if (r == (ssize_t)sizeof(fr)) { handle_l2_frame(n, &fr); }
        }

        if ((n->fd_port2 >= 0) && FD_ISSET(n->fd_port2, &rfds))
        {
            Rs485L2Frame fr;
            const ssize_t r = recv(n->fd_port2, &fr, (ssize_t)sizeof(fr), 0);
            if (r == (ssize_t)sizeof(fr)) { handle_l2_frame(n, &fr); }
        }
    }

    logf_pos(n->pos, "stopping");
    return NULL;
}
#endif

//static UART_Type* UART;

#if 0
int main(void)
{
    Node nodes[MAX_POS];
    memset(nodes, 0, sizeof(nodes));

    const uint8_t used_pos[] = { 1, 2, 3, 5, 7 };

    // init used nodes
    for (size_t i = 0U; i < sizeof(used_pos) / sizeof(used_pos[0]); ++i)
    {
        const uint8_t p = used_pos[i];
        nodes[p].pos = p;
        nodes[p].fd_port1 = -1;
        nodes[p].fd_port2 = -1;
        nodes[p].txid_next = 100;
    }

    // Build adjacency (ONLY input)
    build_adjacency(nodes);

    // Derive bus segments from adjacency, then metric distances, then routing
    derive_buses_from_adjacency(nodes);
    build_all_dist_to_bus(nodes);
    build_toBus_once(nodes);

    // Print routing table initializers
    dump_toBus_table_as_initializers(nodes, used_pos, sizeof(used_pos) / sizeof(used_pos[0]));
    fprintf(stdout, "=== END ROUTING TABLE ===\n\n");

    // Create derived buses and attach every present interface (pos,port) to its derived bus
    Bus buses[MAX_BUSES];
    memset(buses, 0, sizeof(buses));

    static char bus_names[MAX_BUSES][16];
    for (uint8_t busId = 1U; busId <= g_bus_count; ++busId)
    {
        (void)snprintf(bus_names[busId], sizeof(bus_names[busId]), "BUS%u", (unsigned)busId);
        buses[busId].name = bus_names[busId];
        buses[busId].count = 0U;
    }

    for (size_t i = 0U; i < sizeof(used_pos) / sizeof(used_pos[0]); ++i)
    {
        const uint8_t p = used_pos[i];

        for (uint8_t port = PORT1; port <= PORT2; ++port)
        {
            const uint8_t busId = g_bus_of_iface[p][port];
            if (busId == 0U) continue;

            (void)attach_port_to_bus(&buses[busId], &nodes[p], port);
        }
    }

    // Start bus threads
    pthread_t t_bus[MAX_BUSES];
    memset(t_bus, 0, sizeof(t_bus));

    for (uint8_t busId = 1U; busId <= g_bus_count; ++busId)
    {
        pthread_create(&t_bus[busId], NULL, bus_thread, &buses[busId]);
    }

    // Hardcode some �masters� just to generate traffic
    nodes[7].is_master = true;
    nodes[2].is_master = true;

    // Start node threads
    pthread_t t_nodes[MAX_POS];
    memset(t_nodes, 0, sizeof(t_nodes));

    NodeThreadArg args[MAX_POS];
    memset(args, 0, sizeof(args));

    for (size_t i = 0U; i < sizeof(used_pos) / sizeof(used_pos[0]); ++i)
    {
        const uint8_t pos = used_pos[i];
        args[pos].self = &nodes[pos];
        pthread_create(&t_nodes[pos], NULL, node_thread, &args[pos]);
    }

    // Run briefly
    sleep(3);
    atomic_store(&g_running, false);

    // Join nodes
    for (size_t i = 0U; i < sizeof(used_pos) / sizeof(used_pos[0]); ++i)
    {
        const uint8_t pos = used_pos[i];
        pthread_join(t_nodes[pos], NULL);
    }

    // Join buses
    for (uint8_t busId = 1U; busId <= g_bus_count; ++busId)
    {
        pthread_join(t_bus[busId], NULL);
    }

    // Close fds best-effort
    for (uint8_t p = 1U; p < MAX_POS; ++p)
    {
        if (nodes[p].fd_port1 >= 0) close(nodes[p].fd_port1);
        if (nodes[p].fd_port2 >= 0) close(nodes[p].fd_port2);
    }
    for (uint8_t busId = 1U; busId <= g_bus_count; ++busId)
    {
        for (uint8_t i = 0U; i < buses[busId].count; ++i) close(buses[busId].bus_fds[i]);
    }

    return 0;
}
#endif

#if 0 



#define MAX_FRAME_WINDOW 3

static const uint8_t MAX_PRIORITY = 3;

typedef struct {
    uint16_t snd_una; // frame unacked
    uint16_t snd_nxt; // next frame to send

    // free masks   

    page_id_t head_page;
    page_id_t tail_page;
    //uint8_t  head_off;       /* 0..UNIT-1 */
    uint8_t  tail_used;      /* 0..UNIT   */
} prio_stream_t;



static stream_t streams[MAX_POS];


static const uint8_t MAX_POS = 5;

//typedef stream_t L3_streams[MAX_PRIORITY][MAX_POS];



static const uint8_t MAX_PORT = 2;
static const uint8_t MAX_QUEUE = 8;

typedef struct {
    uint8_t addr;
    uint8_t type;
} L2_hdr; // 2 bytes

typedef struct {
    uint16_t src;
    uint16_t dst;
    uint8_t prio;
    uint8_t ttl; // ttl 
} L3_hdr; // 6 bytes

typedef struct {
    L3_hdr hdr;
} L3Packet;

typedef struct {
    uint16_t frame_id;
    uint8_t rsv;
} L4_hdr; // 3 bytes

typedef struct {
    L2_hdr l2hdr;
    L3_hdr l3hdr;
    L4_hdr l4hdr;
} packet_hdr; // 11 bytes

typedef struct {
    uint8_t retry;
    uint8_t time;
} L2_packet_desc;

//typedef L2_packet L2_Queue[MAX_PORT][MAX_PRIORITY][MAX_QUEUE];
static uint8_t l2QueueHd[MAX_PORT][MAX_PRIORITY] = { 0 };
static uint8_t l2QueueTail[MAX_PORT][MAX_PRIORITY] = { 0 };

//static L2_packet l2TxQueue[MAX_PORT][MAX_PRIORITY][MAX_QUEUE];

static uint8_t l2_queue_is_full(uint8_t port, uint8_t prio)
{
    const uint8_t tail = l2QueueTail[port][prio];
    const uint8_t next = (uint8_t)((tail + 1U) % (uint8_t)MAX_QUEUE);
    return (next == l2QueueHd[port][prio]) ? 1U : 0U;
}

void initStreams() {
    for (int pos = 0; pos < MAX_POS; pos++) {
        stream_t* s = &streams[pos];
        for (int prio = 0; prio < prio; prio++) {
            prio_stream_t* p = &s->prio_stream[prio];
            p->head_page = INVALID_PAGE;
            p->tail_page = INVALID_PAGE;
        }
    }
}

static uint16_t pos_addr_table[MAX_POS] = {
     0,
     0x101, // 1.1
     0x102, // 1.2
     0x203, // 2.3
     0x303  // 3.3
};

#define MAX_SUBNET 255
static uint16_t route_table[MAX_SUBNET] = { 0 };
static uint16_t port_ip[MAX_PORT] = { 0x102, 0x202 };
static uint16_t currPos = 1;

#define L2_MSG_TYPE_PDU 0x40

typedef struct  {
    // ptrs to buffer
    page_id_t head_page;
    // page_id_t tail_page;
     //uint8_t  head_off;       /* 0..UNIT-1 */
    uint8_t  tail_used;      /* 0..UNIT   */

    uint8_t rx_index;
    packet_hdr hdr;

    // retry timeout
    uint8_t time;
    uint8_t retry;
} l1_desc;

static l1_desc l1tx[MAX_PORT] = { 0 };

// UART consts
#define UART_FIFO_SIZE 128
#define UART_C2_TIE_MASK                         (0x80U)




typedef enum {
    L2_ADDR = 0,
    L2_TYPE,
    L3_SRC_LOW,
    L3_SRC_HIGH,
    L3_DST_LOW,
    L3_DST_HIGH
};

void UART_PORT1_isr() {
    uint8_t data;
    uint8_t count;

    if (UART->S1 & UART_S1_RDRF_MASK) {
        if (l1tx[PORT1].head_page != INVALID_PAGE) {
            count = UART->RCFIFO;
            for (int i = 0; i < count; i++) {
                data = UART->D;
                // Compare with expected header byte
                if (l1tx[PORT1].rx_index < sizeof(packet_hdr)) {
                    if (data != ((uint8_t*)&l1tx[PORT1].hdr)[l1tx[PORT1].rx_index]) {
                        // Collision detected: abort transfer
                        l1tx[PORT1] = { .head_page = INVALID_PAGE, 0 };
                        return;
                    }
                }
                l1tx[PORT1].rx_index++;
            }
        }
    }
}

void UART_PORT2_isr() {

}

static inline uint8_t UART_ReadByte()
{
    return 0;
}


volatile L2_packet_desc* currentPacket[MAX_PORT] = { NULL, NULL };

static inline void UART_WriteByte(uint8_t data) {

}

void l2Send() {
    for (int port = 0; port < MAX_PORT; port++) {
        if (currentPacket[port] != NULL) {
            continue;
        }
        for (int prio = 0; prio < prio; prio++) {

        }
    }
}



void l3Send(stream_t* steam) {

}

static uint16_t ceil_pages(uint16_t len)
{
    return (uint16_t)((len + (UNIT - 1U)) / UNIT);
}

static uint32_t page_off(page_id_t p) { return ((uint32_t)p * (uint32_t)UNIT); }

bool appSend(const uint8_t* data, uint8_t len, uint16_t pos, uint8_t priority)
{
    if ((data == NULL) || (len == 0U)) { return false; }
    if ((pos >= (uint16_t)MAX_POS) || (priority >= (uint8_t)MAX_PRIORITY)) { return false; }

    stream_t* sp = &streams[pos];
    prio_stream_t* s = &sp->prio_stream[priority];

    /* --- Deterministic capacity check (optional but recommended) ---
     * Worst-case extra pages needed if tail has some free space:
     * bytes can fit into tail slack first, then pages.
     */
    uint16_t tail_free = 0U;
    if (s->tail_page != INVALID_PAGE)
    {
        tail_free = (uint16_t)UNIT - (uint16_t)s->tail_used;  /* 0..UNIT */
    }

    const uint16_t bytes_after_tail = (len > tail_free) ? (uint16_t)(len - tail_free) : 0U;
    const uint16_t need_pages = (bytes_after_tail == 0U) ? 0U : ceil_pages(bytes_after_tail);

    if ((uint16_t)g_free_count < need_pages)
    {
        return false; /* deterministic fail */
    }

    /* --- Append bytes into stream --- */
    uint16_t in = 0U;

    /* Ensure a tail page exists if stream empty */
    if (s->tail_page == INVALID_PAGE)
    {
        page_id_t p = page_alloc();
        if (p == INVALID_PAGE) { return false; } /* should not happen after check */
        s->head_page = p;
        s->tail_page = p;
        // s->head_off = 0U;
        s->tail_used = 0U;
    }

    while (in < (uint16_t)len)
    {
        /* If current tail page is full, allocate a new one */
        if ((uint16_t)s->tail_used >= (uint16_t)UNIT)
        {
            page_id_t p = page_alloc();
            if (p == INVALID_PAGE)
            {
                /* We already checked capacity, so this should not happen,
                   but if it does, return false deterministically without corrupting chain.
                   (No rollback needed because nothing allocated in this iteration if fail here.) */
                return false;
            }
            g_next[s->tail_page] = p;
            s->tail_page = p;
            s->tail_used = 0U;
        }

        /* Copy into tail page */
        const uint16_t space = (uint16_t)UNIT - (uint16_t)s->tail_used;
        uint16_t take = (uint16_t)len - in;
        if (take > space) { take = space; }

        {
            const uint32_t base = page_off(s->tail_page) + (uint32_t)s->tail_used;
            for (uint16_t i = 0U; i < take; ++i)
            {
                g_tx_pool[base + (uint32_t)i] = data[in + i];
            }
        }

        s->tail_used = (uint8_t)((uint16_t)s->tail_used + take);
        //   s->queued_bytes = (uint16_t)(s->queued_bytes + take);
        in = (uint16_t)(in + take);
    }

    return true;
}
#endif

struct Case {
    int pos;
    uint16_t portAddr[MAX_PORT];
    uint8_t devCnt[MAX_PORT];
    //uin
};

class MultiHop : public ::testing::TestWithParam<Case> {
protected:
    static void SetUpTestSuite() {
        ::topology[1] = NodeCfg{ {1,3} };
        ::topology[2] = NodeCfg{ {2,3} };
        ::topology[3] = NodeCfg{ {1,2} };
        ::topology[5] = NodeCfg{ {2,0} };
        ::topology[7] = NodeCfg{ {1,0} };
        for (int i = 0; i < MAX_PORT; ++i) {
            uart_ptrs[i] = &uart_objs[i];
        }
      
    }

    void SetUp() override {
        // runs before each TEST_F(MyFixture, ...)
        myPos = GetParam().pos;
        

        netInit(uart_ptrs);
    }

    void TearDown() override {
        // runs after each TEST_F(MyFixture, ...)
        //value = 0;
    }

    static UART_Type  uart_objs[MAX_PORT];   // objects
public:
    static UART_Type* uart_ptrs[MAX_PORT];   // pointers passed to netInit
};

UART_Type MultiHop::uart_objs[MAX_PORT];
UART_Type* MultiHop::uart_ptrs[MAX_PORT];


struct MockUart {
    MOCK_METHOD(void, l1UARTWriteNonBlocking, (UART_Type* UART, const uint8_t* data, size_t length), ());
    MOCK_METHOD(bool, l1UARTCmpNonBlocking, (UART_Type* UART, const uint8_t* data, size_t length), ());
    MOCK_METHOD(void, l1UARTReadNonBlocking, (UART_Type* UART, uint8_t* data, size_t length), ());
};

static MockUart* g_mock = nullptr;

// 3) The linker will redirect calls to hw_read() to __wrap_hw_read()
extern "C" void l1UARTWriteNonBlocking(UART_Type* UART, const uint8_t* data, size_t length)
{
    ASSERT_NE(g_mock, nullptr);
    g_mock->l1UARTWriteNonBlocking(UART, data, length);

    // echo 
    for (int port = 0; port < MAX_PORT; port++) {
        if (UART != MultiHop::uart_ptrs[port]) {
            continue;
        }
        UART->S1 |= UART_S1_RDRF_MASK;
        UART->RCFIFO = length;
        l1TransferHandleIRQ(UART, port);
        break;
    }
}


extern "C" void l1UARTReadNonBlocking(UART_Type* UART, uint8_t* data, size_t length)
{
    ASSERT_NE(g_mock, nullptr);
    g_mock->l1UARTReadNonBlocking(UART, data, length);
}

extern "C" bool l1UARTCmpNonBlocking(UART_Type* UART, const uint8_t* data, size_t length)
{
    EXPECT_NE(g_mock, nullptr);
    return  g_mock->l1UARTCmpNonBlocking(UART, data, length);
}

TEST_P(MultiHop, addr) {
    for (int port = 0; port < MAX_PORT; port++) {
        EXPECT_EQ(port_addr[port], GetParam().portAddr[port]);
    }
}

TEST_P(MultiHop, mstPassFail) {

    MockUart mock;
    g_mock = &mock;

    // pass time for 
    bool portsTested[MAX_PORT];
    uint8_t nxtMst[MAX_PORT];
    memset(portsTested, 0, sizeof portsTested);

    bool mstToken[MAX_PORT];
    memset(mstToken, 0, sizeof mstToken);

    uint16_t time[MAX_PORT];
    std::fill(std::begin(time), std::end(time), LINE_SILENT);
    uint8_t tick = LINE_SILENT / 2;
    netTick(LINE_SILENT);

    while (true) {
        bool allPortsTested = true;
        for (int port = 0; port < MAX_PORT; port++) {
            time[port] += tick;
            uint8_t l2Addr = GetParam().portAddr[port] & 0x00FF;
            if (l2Addr == 0 /* ||
                portsTested[port]*/) {
                continue;
            }

            uint16_t mstSelTime = (l2Addr * LINE_SILENT);
            bool expectCall = false;

            // advance by LINE_SILENT
            if (time[port] > mstSelTime) {
                    // compute next MST
               nxtMst[port] = (l2Addr + 1) > GetParam().devCnt[port] ? 1 : l2Addr + 1;
               mstToken[port] = true;
               expectCall = true;
            }
            else if (mstToken[port] && time[port] > (LINE_SILENT/2)) {
                nxtMst[port] = (nxtMst[port] + 1) > GetParam().devCnt[port] ? 1 : nxtMst[port] + 1;

                if (nxtMst[port] == l2Addr) // rollover test done for port
                {
                    portsTested[port] = true;
                    nxtMst[port] = (nxtMst[port] + 1) > GetParam().devCnt[port] ? 1 : nxtMst[port] + 1;
                }

                expectCall = true;
            }

            if (expectCall) {
                time[port] = 0;
                std::array<uint8_t, 3> expected{ { nxtMst[port], L2_PKT_TYPE_MST, 0xFF } };
                const uint8_t expectedSize = (sizeof(L2Hdr) + sizeof(L2Pkt::crc));

                EXPECT_CALL(mock, l1UARTWriteNonBlocking(uart_ptrs[port], testing::NotNull(), expectedSize))
                    .Times(1)
                    .WillOnce(testing::Invoke([expected](UART_Type* UART, const uint8_t* data, size_t len) {
                    EXPECT_EQ(0, std::memcmp(data, expected.data(), expected.size()));
                        }))
                    .RetiresOnSaturation();

                // echo
                EXPECT_CALL(mock, l1UARTCmpNonBlocking(uart_ptrs[port], testing::NotNull(), expectedSize))
                    .Times(1)
                    .WillOnce(testing::Invoke([expected](UART_Type* UART, const uint8_t* data, size_t len) {
                    EXPECT_EQ(0, std::memcmp(data, expected.data(), expected.size()));
                    return true;
                        }))
                    .RetiresOnSaturation();
            }

            if (!portsTested[port]) {
                allPortsTested = false;
            }
        }
       
        netTick(tick);
        

        if (allPortsTested) {
            break;
        }
    } 
}

TEST_P(MultiHop, mstPassMsg) {
    MockUart mock;
    g_mock = &mock;
    uint8_t nxtMst[MAX_PORT];

    for (int port = 0; port < MAX_PORT; port++) {
        uint8_t l2Addr = GetParam().portAddr[port] & 0x00FF;
        if (l2Addr == 0 /* ||
               portsTested[port]*/) {
            continue;
        }

        std::array<uint8_t, 3> pkt{ { l2Addr, L2_PKT_TYPE_MST, 0xFF } };

        uart_ptrs[port]->S1 |= UART_S1_RDRF_MASK;
        uart_ptrs[port]->RCFIFO = pkt.size();

        // call to copy header
        EXPECT_CALL(mock, l1UARTReadNonBlocking(uart_ptrs[port], testing::NotNull(), sizeof(L2Hdr)))
            .Times(1)
            .WillOnce(testing::Invoke([pkt](UART_Type* UART, uint8_t* data, size_t len) {
            std::memcpy(data, pkt.data(), len);
                }))
            .RetiresOnSaturation();

        EXPECT_CALL(mock, l1UARTReadNonBlocking(uart_ptrs[port], testing::NotNull(), sizeof(L2Pkt::crc)))
            .Times(1)
            .WillOnce(testing::Invoke([pkt](UART_Type* UART, uint8_t* data, size_t len) {
            memcpy(data, pkt.data() + sizeof(L2Hdr), len);
                }))
            .RetiresOnSaturation();

        l1TransferHandleIRQ(uart_ptrs[port], port);

        uart_ptrs[port]->S1 &= ~UART_S1_RDRF_MASK;
        uart_ptrs[port]->RCFIFO = 0;

        // since there is no message should pass token immediatly
        nxtMst[port] = (l2Addr + 1) > GetParam().devCnt[port] ? 1 : l2Addr + 1;
        std::array<uint8_t, 3> expected{ { nxtMst[port], L2_PKT_TYPE_MST, 0xFF } };
        const uint8_t expectedSize = (sizeof(L2Hdr) + sizeof(L2Pkt::crc));

        EXPECT_CALL(mock, l1UARTWriteNonBlocking(uart_ptrs[port], testing::NotNull(), expectedSize))
            .Times(1)
            .WillOnce(testing::Invoke([expected](UART_Type* UART, const uint8_t* data, size_t len) {
            EXPECT_EQ(0, std::memcmp(data, expected.data(), expected.size()));
                }))
            .RetiresOnSaturation();

        // echo
        EXPECT_CALL(mock, l1UARTCmpNonBlocking(uart_ptrs[port], testing::NotNull(), expectedSize))
            .Times(1)
            .WillOnce(testing::Invoke([expected](UART_Type* UART, const uint8_t* data, size_t len) {
            EXPECT_EQ(0, std::memcmp(data, expected.data(), expected.size()));
            return true;
                }))
            .RetiresOnSaturation();
    }
    netTick(INTER_FRAME_SILENCE+1);

    // TODO Fails
}


//TEST_P(MultiHop, l2test) {
//
//}

INSTANTIATE_TEST_SUITE_P(
    Runs, MultiHop,
    ::testing::Values(
      //     pos port1   port2
      Case{ 1, {0x101, 0x301}, {3, 2} },
      Case{ 2, {0x201, 0x302}, {3, 2} },
      Case{ 3, {0x102, 0x202}, {3, 3} },
      Case{ 5, {0x203, 0x000}, {3, 0} },
      Case{ 7, {0x103, 0x000}, {3, 0} }
    )
);

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}




#if 0
typedef struct {
    uint8_t subnet[MAX_PORT];   // 0 if unused
} NodeCfg;

static NodeCfg topology[MAX_POS] = {
    {},
    {1, 3},
    {2, 3},
    {1, 2},
    {},
    {2},
    {1}
}; // topology from config

static uint16_t pos_addr_table[MAX_POS];

int main(void)
{
    // set port addr
    uint8_t rs485Addr[MAX_PORT] = {1, 1};

    for (int pos = 0; pos < MAX_POS; pos++) {
        if (pos == myPos) {
            
        }
        else {
           // one subnet directly reachable
            for (uint8_t port = 0; port < MAX_PORT; port++)
            if ((topology[pos].subnet[0] == topology[myPos].subnet[0]) || 
                (topology[pos].subnet[1] == topology[myPos].subnet[0])) {
                rs485Addr[0]++;
            }
        }
    }
}
#endif