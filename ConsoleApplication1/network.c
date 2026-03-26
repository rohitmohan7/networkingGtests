#include "network.h"
#include "layer2.h"
#include "layer3.h"
#include "layer1.h"
#include "layer4.h"
#include "allocator.h"
#include "pmm.h"

/* ----------------------------- configuration ---------------------------- */
// TODO move to cfg
PosType_t myPos = MAX_POS;
NodeCfg_t topology[MAX_POS];


#define INVALID_SUBNET   0u
#define INVALID_GATEWAY  0u
#define INVALID_POS MAX_POS
#define DIRECT_GATEWAY   0u

#define INF_HOPS            0xFFu
#define INF_COST            0xFFFFFFFFu

/*
 * Number of global fixed-point passes.
 * 6..10 is usually a good range on small embedded topologies.
 */
#define MAX_GLOBAL_PASSES   8u

/* ---------------------------- route scratch ----------------------------- */
/*
 * Compact scratch for one source position.
 * Reachable iff hops[subnet] != INF_HOPS.
 */
typedef struct
{
    uint8_t hops[MAX_SUBNET];
    uint8_t prevSubnet[MAX_SUBNET];
    PosType_t prevPeerPos[MAX_SUBNET];
    uint16_t firstGw[MAX_SUBNET];
    uint32_t cost[MAX_SUBNET];
} RouteScratch;

/* ------------------------------ RAM USAGE GATING ------------------------------- */

#define RAM_BUDGET_BYTES (20u * 1024u)

/* setPortAddr() locals */
#define SETPORT_LOCAL_BYTES \
    (sizeof(uint16_t[MAX_SUBNET]) + sizeof(uint16_t[MAX_POS][MAX_PORT]))

/* solve_global_bus_load() locals */
#define SOLVE_LOCAL_BYTES           \
    (sizeof(uint16_t[MAX_SUBNET]) + \
     sizeof(uint16_t[MAX_SUBNET]) + \
     sizeof(RouteScratch) +         \
     sizeof(PosType_t[MAX_POS]))

/* Worst nested stack peak: setPortAddr() -> solve_global_bus_load() */
#define PEAK_STACK_BYTES \
    (SETPORT_LOCAL_BYTES + SOLVE_LOCAL_BYTES)

/* Persistent RAM used by shown globals */
#define PERSISTENT_RAM_BYTES             \
    (sizeof(myPos) +                     \
     sizeof(topology) +                  \
     sizeof(l3RouteTable) +              \
     sizeof(l3RouteHops) +               \
     sizeof(l3AddrTblPrio) +             \
     sizeof(l3BcastInSubnetForSrcPort) + \
     sizeof(maxL2Addr))

/* Approx total RAM consumed while setPortAddr() is active */
#define TOTAL_RAM_EST_BYTES \
    (PEAK_STACK_BYTES + PERSISTENT_RAM_BYTES)

_Static_assert(PEAK_STACK_BYTES <= RAM_BUDGET_BYTES,
               "Routing stack usage exceeds 20 KB");

_Static_assert(TOTAL_RAM_EST_BYTES <= RAM_BUDGET_BYTES,
               "Routing total RAM estimate exceeds 20 KB");

/* ------------------------------ utilities ------------------------------- */

static bool better_route(uint8_t newHops,
                         uint32_t newCost,
                         uint8_t oldHops,
                         uint32_t oldCost)
{
    if (newHops < oldHops) {
        return true;
    }
    if ((newHops == oldHops) && (newCost < oldCost)) {
        return true;
    }
    return false;
}

static bool node_is_used(int pos)
{
    int port;

    for (port = 0; port < MAX_PORT; port++) {
        if (topology[pos].subnet[port] != INVALID_SUBNET) {
            return true;
        }
    }
    return false;
}

static bool node_has_subnet(int pos, uint8_t subnet)
{
    int port;

    if (subnet == INVALID_SUBNET) {
        return false;
    }

    for (port = 0; port < MAX_PORT; port++) {
        if (topology[pos].subnet[port] == subnet) {
            return true;
        }
    }
    return false;
}

static PosType_t build_used_pos_list(PosType_t usedPos[MAX_POS])
{
    PosType_t count = 0;
    int pos;

    for (pos = 1; pos < MAX_POS; pos++)
    {
        if (node_is_used(pos))
        {
            usedPos[count++] = (PosType_t)pos;
        }
    }

    return count;
}

/* -------------------------- L3 address assign --------------------------- */

static void assign_l3_addresses(uint16_t l3AddrTable[MAX_POS][MAX_PORT])
{
    uint8_t hostId[MAX_SUBNET];
    int pos;
    int port;

    memset(hostId, 0, sizeof(hostId));
    memset(l3AddrTable, 0, sizeof(uint16_t) * MAX_POS * MAX_PORT);
    memset(maxL2Addr, 0, sizeof(maxL2Addr));

    for (pos = 1; pos < MAX_POS; pos++) {
        for (port = 0; port < MAX_PORT; port++) {
            uint8_t subnet = topology[pos].subnet[port];

            if (subnet == INVALID_SUBNET) {
                continue;
            }

            hostId[subnet]++;
            l3AddrTable[pos][port] =
                (uint16_t)(((uint16_t)subnet << 8) | hostId[subnet]);
        }
    }

    for (port = 0; port < MAX_PORT; port++) {
        uint8_t subnet = topology[myPos].subnet[port];
        
//#ifndef IS_DEBUG // allow debug port access
        if (hostId[subnet] <= 1) { // if we have only one device in bus disable the bus
            l3AddrTable[myPos][port] = 0;
        }
        else {
//#endif
            maxL2Addr[port] = hostId[subnet];
        }
    }
}

/* ------------------------ shortest-path solving ------------------------- */

/*
 * Compute best routes from srcPos to every subnet using current busLoad.
 *
 * Hard rule:
 *   1) minimum hops
 * Tie break:
 *   2) minimum total traversed-subnet load
 */
static void compute_routes_for_pos(int srcPos,
                                   const uint16_t l3AddrTable[MAX_POS][MAX_PORT],
                                   const uint16_t busLoad[MAX_SUBNET],
                                   const PosType_t usedPos[MAX_POS],
                                   PosType_t usedCount,
                                   RouteScratch *rs)
{
    int s;
    uint8_t passIdx;

    for (s = 0; s < MAX_SUBNET; s++) {
        rs->hops[s] = INF_HOPS;
        rs->prevSubnet[s] = INVALID_SUBNET;
        rs->prevPeerPos[s] = INVALID_POS;
        rs->firstGw[s] = INVALID_GATEWAY;
        rs->cost[s] = INF_COST;
    }

    /* Seed directly attached subnets */
    {
        int port;

        for (port = 0; port < MAX_PORT; port++) {
            uint8_t subnet = topology[srcPos].subnet[port];

            if (subnet == INVALID_SUBNET) {
                continue;
            }

            rs->hops[subnet] = 0u;
            rs->prevSubnet[subnet] = INVALID_SUBNET;
            rs->prevPeerPos[subnet] = INVALID_POS;
            rs->firstGw[subnet] = DIRECT_GATEWAY;
            rs->cost[subnet] = (uint32_t)busLoad[subnet];
        }
    }

    /*
     * Bellman-Ford style relaxation over the implicit subnet graph.
     * With MAX_PORT == 2 this is quite cheap.
     */
    for (passIdx = 0; passIdx < (uint8_t)(MAX_SUBNET - 1u); passIdx++) {
        bool changed = false;
        PosType_t ui;

        for (ui = 0; ui < usedCount; ui++) {
            int peerPos = usedPos[ui];
            int inPort;
            int outPort;

            if (peerPos == srcPos) {
                continue;
            }

            for (inPort = 0; inPort < MAX_PORT; inPort++) {
                uint8_t inSubnet = topology[peerPos].subnet[inPort];

                if (inSubnet == INVALID_SUBNET) {
                    continue;
                }

                if (rs->hops[inSubnet] == INF_HOPS) {
                    continue;
                }

                {
                    uint16_t gwAddrOnInSubnet = l3AddrTable[peerPos][inPort];
                    uint8_t newHops = (uint8_t)(rs->hops[inSubnet] + 1u);
                    uint16_t newFirstGw =
                        (rs->hops[inSubnet] == 0u) ? gwAddrOnInSubnet
                                                   : rs->firstGw[inSubnet];

                    for (outPort = 0; outPort < MAX_PORT; outPort++) {
                        uint8_t outSubnet = topology[peerPos].subnet[outPort];

                        if ((outSubnet == INVALID_SUBNET) ||
                            (outSubnet == inSubnet)) {
                            continue;
                        }

                        {
                            uint32_t newCost =
                                rs->cost[inSubnet] + (uint32_t)busLoad[outSubnet];

                            if ((rs->hops[outSubnet] == INF_HOPS) ||
                                better_route(newHops,
                                             newCost,
                                             rs->hops[outSubnet],
                                             rs->cost[outSubnet])) {
                                rs->hops[outSubnet] = newHops;
                                rs->prevSubnet[outSubnet] = inSubnet;
                                rs->prevPeerPos[outSubnet] = peerPos;
                                rs->firstGw[outSubnet] = newFirstGw;
                                rs->cost[outSubnet] = newCost;
                                changed = true;
                            }
                        }
                    }
                }
            }
        }

        if (!changed) {
            break;
        }
    }
}

/*
 * For one destination node, choose the best destination subnet reachable
 * from the already-computed route snapshot.
 */
static bool choose_best_dst_subnet_for_node(int dstPos,
                                            const RouteScratch *rs,
                                            uint8_t *bestDstSubnet)
{
    bool found = false;
    uint8_t chosenSubnet = INVALID_SUBNET;
    uint8_t chosenHops = INF_HOPS;
    uint32_t chosenCost = INF_COST;
    int port;

    for (port = 0; port < MAX_PORT; port++) {
        uint8_t subnet = topology[dstPos].subnet[port];

        if (subnet == INVALID_SUBNET) {
            continue;
        }

        if (rs->hops[subnet] == INF_HOPS) {
            continue;
        }

        if (!found ||
            better_route(rs->hops[subnet],
                         rs->cost[subnet],
                         chosenHops,
                         chosenCost) ||
            ((rs->hops[subnet] == chosenHops) &&
             (rs->cost[subnet] == chosenCost) &&
             (subnet < chosenSubnet))) {
            found = true;
            chosenSubnet = subnet;
            chosenHops = rs->hops[subnet];
            chosenCost = rs->cost[subnet];
        }
    }

    if (!found) {
        return false;
    }

    *bestDstSubnet = chosenSubnet;
    return true;
}

/*
 * Add one unit of traffic on the chosen path:
 *   dstSubnet -> prevSubnet -> ... -> local source subnet
 */
static void increment_path_load(uint8_t dstSubnet,
                                const uint8_t prevSubnet[MAX_SUBNET],
                                uint16_t busLoad[MAX_SUBNET])
{
    uint8_t subnet = dstSubnet;
    uint16_t guard = 0u;

    while ((subnet != INVALID_SUBNET) && (guard < MAX_SUBNET)) {
        busLoad[subnet]++;

        if (prevSubnet[subnet] == INVALID_SUBNET) {
            break;
        }

        subnet = prevSubnet[subnet];
        guard++;
    }
}

/* ------------------------ global load estimation ------------------------ */
/*
 * This is the RAM-light global version:
 *
 *   old busLoad  -> compute every source route against same snapshot
 *                -> build nextBusLoad from scratch
 *
 * So routing decisions are globally coupled across all sources,
 * but RAM stays tiny.
 *
 * This is not the exhaustive NP-hard exact optimum. That exact version will
 * not fit the stated K22F RAM budget at MAX_POS=100.
 */
static void solve_global_bus_load(const uint16_t l3AddrTable[MAX_POS][MAX_PORT],
    uint16_t finalBusLoad[MAX_SUBNET])
{
    uint16_t busLoad[MAX_SUBNET];
    uint16_t nextBusLoad[MAX_SUBNET];
    RouteScratch rs;
    PosType_t usedPos[MAX_POS];
    PosType_t usedCount;
    uint8_t passIdx;

    usedCount = build_used_pos_list(usedPos);

    memset(busLoad, 0, sizeof(busLoad));

    for (passIdx = 0; passIdx < MAX_GLOBAL_PASSES; passIdx++) {
        PosType_t ui;

        memset(nextBusLoad, 0, sizeof(nextBusLoad));

        for (ui = 0; ui < usedCount; ui++) {
            int srcPos = usedPos[ui];
            PosType_t uj;

            compute_routes_for_pos(srcPos, l3AddrTable, busLoad, usedPos, usedCount, &rs);

            for (uj = 0; uj < usedCount; uj++) {
                int dstPos = usedPos[uj];
                uint8_t bestDstSubnet;

                if (dstPos == srcPos) {
                    continue;
                }

                if (!choose_best_dst_subnet_for_node(dstPos, &rs, &bestDstSubnet)) {
                    continue;
                }

                increment_path_load(bestDstSubnet, rs.prevSubnet, nextBusLoad);
            }
        }

        if (memcmp(busLoad, nextBusLoad, sizeof(busLoad)) == 0) {
            memcpy(finalBusLoad, nextBusLoad, sizeof(nextBusLoad));
            return;
        }

        memcpy(busLoad, nextBusLoad, sizeof(busLoad));
    }

    memcpy(finalBusLoad, busLoad, sizeof(busLoad));
}

/* ------------------------- final myPos route row ------------------------ */

static void build_route_row_for_my_pos(const uint16_t l3AddrTable[MAX_POS][MAX_PORT],
                                       const uint16_t busLoad[MAX_SUBNET])
{
    RouteScratch rs;
    PosType_t usedPos[MAX_POS];
    PosType_t usedCount;
    int subnet;

    usedCount = build_used_pos_list(usedPos);

    for (subnet = 0; subnet < MAX_SUBNET; subnet++)
    {
        l3RouteTable[subnet] = INVALID_GATEWAY;
        l3RouteHops[subnet] = INF_HOPS;
    }

    if ((myPos == 0u) || (myPos >= MAX_POS) || !node_is_used((int)myPos))
    {
        return;
    }

    compute_routes_for_pos((int)myPos, l3AddrTable, busLoad, usedPos, usedCount, &rs);

    for (subnet = 1; subnet < MAX_SUBNET; subnet++)
    {
        if (rs.hops[subnet] == INF_HOPS)
        {
            l3RouteTable[subnet] = INVALID_GATEWAY;
            l3RouteHops[subnet] = INF_HOPS;
        }
        else
        {
            l3RouteTable[subnet] = rs.firstGw[subnet];
            l3RouteHops[subnet] = (uint16_t)rs.hops[subnet];
            if (l3MaxHops < l3RouteHops[subnet]) {
                l3MaxHops = l3RouteHops[subnet];
            }
        }
    }
}

static bool better_dest_addr(uint8_t newHops,
    uint32_t newCost,
    uint16_t newAddr,
    uint8_t oldHops,
    uint32_t oldCost,
    uint16_t oldAddr)
{
    if (newHops < oldHops) {
        return true;
    }
    if (newHops > oldHops) {
        return false;
    }

    if (newCost < oldCost) {
        return true;
    }
    if (newCost > oldCost) {
        return false;
    }

    return (newAddr < oldAddr);
}

static void build_broadcast_forward_info_for_my_pos(
    const uint16_t l3AddrTable[MAX_POS][MAX_PORT],
    const uint16_t busLoad[MAX_SUBNET])
{
    RouteScratch rs;
    PosType_t usedPos[MAX_POS];
    PosType_t usedCount;
    PosType_t ui;
    int outPort;

    usedCount = build_used_pos_list(usedPos);

    memset(l3BcastInSubnetForSrcPort, 0, sizeof(l3BcastInSubnetForSrcPort));

    if ((myPos == 0u) || (myPos >= MAX_POS) || !node_is_used((int)myPos))
    {
        return;
    }

    for (ui = 0; ui < usedCount; ui++)
    {
        int srcPos = usedPos[ui];

        if (srcPos == (int)myPos)
        {
            continue; /* local-origin broadcast handled separately */
        }

        compute_routes_for_pos(srcPos, l3AddrTable, busLoad, usedPos, usedCount, &rs);

        for (outPort = 0; outPort < MAX_PORT; outPort++)
        {
            uint8_t outSubnet = topology[myPos].subnet[outPort];

            if (outSubnet == INVALID_SUBNET)
            {
                continue;
            }

            if (rs.hops[outSubnet] == INF_HOPS)
            {
                continue;
            }

            /*
             * myPos is the designated forwarder into outSubnet only if the
             * chosen shortest-path tree enters outSubnet through myPos.
             */
            if ((rs.prevPeerPos[outSubnet] == myPos) &&
                (rs.prevSubnet[outSubnet] != INVALID_SUBNET))
            {
                l3BcastInSubnetForSrcPort[srcPos][outPort] =
                    rs.prevSubnet[outSubnet];
            }
        }
    }
}

static void build_ordered_dst_addr_for_my_pos(
    const uint16_t l3AddrTable[MAX_POS][MAX_PORT],
    const uint16_t busLoad[MAX_SUBNET])
{
    RouteScratch rs;
    PosType_t usedPos[MAX_POS];
    PosType_t usedCount;
    int dstPos;

    usedCount = build_used_pos_list(usedPos);

    memset(l3AddrTblPrio, 0, sizeof(l3AddrTblPrio));

    if ((myPos == 0u) || (myPos >= MAX_POS) || !node_is_used((int)myPos)) {
        return;
    }

    compute_routes_for_pos((int)myPos, l3AddrTable, busLoad, usedPos, usedCount, &rs);

    for (dstPos = 1; dstPos < MAX_POS; dstPos++) {
        uint16_t addr[MAX_PORT];
        uint8_t hops[MAX_PORT];
        uint32_t cost[MAX_PORT];
        uint8_t count = 0;
        int port;
        int i;
        int j;

        if (!node_is_used(dstPos)) {
            continue;
        }

        /* For myPos itself, keep current port order exactly as in topology */
        if (dstPos == (int)myPos) {
            for (port = 0; port < MAX_PORT; port++) {
                uint8_t subnet = topology[dstPos].subnet[port];

                if (subnet == INVALID_SUBNET) {
                    l3AddrTblPrio[dstPos][port] = 0u;
                }
                else {
                    l3AddrTblPrio[dstPos][port] = l3AddrTable[dstPos][port];
                }
            }
            continue;
        }

        for (port = 0; port < MAX_PORT; port++) {
            uint8_t subnet = topology[dstPos].subnet[port];

            if (subnet == INVALID_SUBNET) {
                continue;
            }

            if (rs.hops[subnet] == INF_HOPS) {
                continue;
            }

            addr[count] = l3AddrTable[dstPos][port];
            hops[count] = rs.hops[subnet];
            cost[count] = rs.cost[subnet];
            count++;
        }

        /* insertion sort; MAX_PORT is tiny */
        for (i = 1; i < count; i++) {
            uint16_t a = addr[i];
            uint8_t h = hops[i];
            uint32_t c = cost[i];

            j = i - 1;
            while (j >= 0 &&
                better_dest_addr(h, c, a, hops[j], cost[j], addr[j])) {
                addr[j + 1] = addr[j];
                hops[j + 1] = hops[j];
                cost[j + 1] = cost[j];
                j--;
            }

            addr[j + 1] = a;
            hops[j + 1] = h;
            cost[j + 1] = c;
        }

        for (i = 0; i < count; i++) {
            l3AddrTblPrio[dstPos][i] = addr[i];
        }
    }
}

static void setPortAddr(void)
{
    uint16_t finalBusLoad[MAX_SUBNET];
    uint16_t l3AddrTable[MAX_POS][MAX_PORT];

    assign_l3_addresses(l3AddrTable);
    solve_global_bus_load(l3AddrTable, finalBusLoad);
    build_route_row_for_my_pos(l3AddrTable, finalBusLoad);
    build_ordered_dst_addr_for_my_pos(l3AddrTable, finalBusLoad);
    build_broadcast_forward_info_for_my_pos(l3AddrTable, finalBusLoad);
    return;
}

void netInit(UART_Type* UART[MAX_PORT]) {
    pages_init();

    if (g_pmm.config) // config is present
    {
        // copy once from config to avoid repeated access (TODO access from flex ram ? )
        myPos = g_pmm.config->pos;
        memcpy(topology, g_pmm.config->topology, sizeof(topology));
        
        // disable subnets with no UART
        for (int port = 0; port < MAX_PORT; port++) {
            if (!UART[port])
            {
                topology[myPos].subnet[port] = INVALID_SUBNET;
            }
        }
        
        setPortAddr();
    }
    
    l1Init(UART);
	l2Init();
	l3Init();
    l4Init();
}

void netTick() {
#if 0
	for (int port = 0; port < MAX_PORT; port) {
		if (mst_token[port]) {
			scheduler();
			break;
		}
	}
#endif
	//l2Tick();
   // l4Tick(ms);
}
