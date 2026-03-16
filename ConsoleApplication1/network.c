#include "network.h"
#include "layer2.h"
#include "layer3.h"
#include "layer1.h"
#include "layer4.h"
#include "allocator.h"

/* ----------------------------- configuration ---------------------------- */
// TODO move to cfg
uint16_t myPos;
NodeCfg topology[MAX_POS];


#define INVALID_SUBNET   0u
#define INVALID_GATEWAY  0u
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
    uint8_t  hops[MAX_SUBNET];
    uint8_t  prevSubnet[MAX_SUBNET];
    uint16_t firstGw[MAX_SUBNET];
    uint32_t cost[MAX_SUBNET];
} RouteScratch;

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

static uint8_t build_used_pos_list(uint8_t usedPos[MAX_POS])
{
    uint8_t count = 0;
    int pos;

    for (pos = 1; pos < MAX_POS; pos++) {
        if (node_is_used(pos)) {
            usedPos[count++] = (uint8_t)pos;
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
        if (hostId[subnet] <= 1) { // if we have only one device in bus disable the bus
            l3AddrTable[myPos][port] = 0;
        }
        else {
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
    const uint8_t usedPos[MAX_POS],
    uint8_t usedCount,
    RouteScratch* rs)
{
    int s;
    uint8_t passIdx;

    for (s = 0; s < MAX_SUBNET; s++) {
        rs->hops[s] = INF_HOPS;
        rs->prevSubnet[s] = INVALID_SUBNET;
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
        uint8_t ui;

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
    uint8_t usedPos[MAX_POS];
    uint8_t usedCount;
    uint8_t passIdx;

    usedCount = build_used_pos_list(usedPos);

    memset(busLoad, 0, sizeof(busLoad));

    for (passIdx = 0; passIdx < MAX_GLOBAL_PASSES; passIdx++) {
        uint8_t ui;

        memset(nextBusLoad, 0, sizeof(nextBusLoad));

        for (ui = 0; ui < usedCount; ui++) {
            int srcPos = usedPos[ui];
            uint8_t uj;

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
    uint8_t usedPos[MAX_POS];
    uint8_t usedCount;
    int subnet;

    usedCount = build_used_pos_list(usedPos);

    for (subnet = 0; subnet < MAX_SUBNET; subnet++) {
        l3RouteTable[subnet] = INVALID_GATEWAY;
    }

    if ((myPos == 0u) || (myPos >= MAX_POS) || !node_is_used((int)myPos)) {
        return;
    }

    compute_routes_for_pos((int)myPos, l3AddrTable, busLoad, usedPos, usedCount, &rs);

    for (subnet = 1; subnet < MAX_SUBNET; subnet++) {
        if (rs.hops[subnet] == INF_HOPS) {
            l3RouteTable[subnet] = INVALID_GATEWAY;
        }
        else {
            l3RouteTable[subnet] = rs.firstGw[subnet];
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

static void build_ordered_dst_addr_for_my_pos(
    const uint16_t l3AddrTable[MAX_POS][MAX_PORT],
    const uint16_t busLoad[MAX_SUBNET])
{
    RouteScratch rs;
    uint8_t usedPos[MAX_POS];
    uint8_t usedCount;
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

void setPortAddr(void)
{
    uint16_t finalBusLoad[MAX_SUBNET];
    uint16_t l3AddrTable[MAX_POS][MAX_PORT];

    assign_l3_addresses(l3AddrTable);
    solve_global_bus_load(l3AddrTable, finalBusLoad);
    build_route_row_for_my_pos(l3AddrTable, finalBusLoad);
    build_ordered_dst_addr_for_my_pos(l3AddrTable, finalBusLoad);
    return;
}

#if 0
void setPortAddr() {
    // set port addr
    uint8_t l2Addr[MAX_SUBNET];
    memset(l2Addr, 0, (MAX_POS * MAX_PORT));

    uint8_t busLoad[MAX_SUBNET];
    memset(busLoad, 0, MAX_SUBNET);

    memset(l3RouteTableTest, 0, (MAX_POS*MAX_SUBNET));

    // TODO pos to be 0 indexed in topology ?
    for (int pos = 1; pos < MAX_POS; pos++) {

#if 0
        uint8_t subnetHop[MAX_SUBNET];
        memset(subnetHop, 0xFF, MAX_SUBNET);

        uint16_t subnetGateways[MAX_SUBNET][MAX_POS];
        uint8_t subnetGatewayCnt[MAX_SUBNET];
        memset(subnetGatewayCnt, 0x0, MAX_SUBNET);
#endif
        // first compute addresses 
        for (int peerPos = 1; peerPos < MAX_POS; peerPos++) {
            if (pos == peerPos) {
                for (int port = 0; port < MAX_PORT; port++) {
                    uint8_t subnet = topology[pos].subnet[port];
                    if (!subnet) {
                        continue;
                    }

                    l2Addr[subnet]++;
                    l3AddrTableTest[pos][port] = (subnet << 8) | l2Addr[subnet]; // give addr
                }
                continue;
            }
#if 0
            /* resolve local reachable */
            uint8_t unReachableSubnetCnt = 0;
            uint8_t unReachableSubnet[MAX_SUBNET];

            //uint8_t reachableSubnetCnt = 0;
            //uint8_t reachableSubnet[MAX_SUBNET];

            uint8_t distantSubnetCnt = 0;
            uint8_t distantSubnet[MAX_SUBNET];

            uint8_t localGatewayCnt = 0;
            uint16_t localGateway[MAX_SUBNET];
#endif
            uint8_t localGatewayCnt = 0;
            uint8_t prefTxIdx;
            uint8_t prefTxtxLoad;

            for (int peerPort = 0; peerPort < MAX_PORT; peerPort++) {
                uint8_t peerSubnet = topology[peerPos].subnet[peerPort];

                if (!peerSubnet) {
                    continue;
                }

                l2Addr[peerSubnet]++;
                l3AddrTableTest[peerPos][peerPort] = (peerSubnet << 8) | l2Addr[peerSubnet]; // give addr

                bool isDistantSubnet = true;

                for (int port = 0; port < MAX_PORT; port++) {
                    uint8_t mySubnet = topology[pos].subnet[port];
                    if (!mySubnet) {
                        continue;
                    }

                    if (mySubnet == peerSubnet) {
                        isDistantSubnet = false;
                        break;
                    }
                }

                if (isDistantSubnet) {
                    // set as possible gateway for this distant subnet
#if 0
                    distantSubnet[distantSubnetCnt++] = peerSubnet;
#endif
                }
                else {
                    if (!localGatewayCnt) {
                        prefTxIdx = peerPort;
                        prefTxtxLoad = busLoad[peerSubnet];
                    }
                    else if (busLoad[peerSubnet] < prefTxtxLoad) { // chose lowest loaded subnet
                        prefTxIdx = peerPort;
                        prefTxtxLoad = busLoad[peerSubnet];
                    }
#if 0
                    subnetHop[peerSubnet] = 0;
                    localGateway[localGatewayCnt++] = l3AddrTableTest[peerPos][peerPort];
#endif
                    localGatewayCnt++;
                }
            }

            if (localGatewayCnt > 0) { // single hop 
                // increase the load for desired tx for the pos
                busLoad[topology[peerPos].subnet[prefTxIdx]]++;
#if 0
                for (uint8_t distantSubnetIdx = 0; distantSubnetIdx < distantSubnetCnt; distantSubnetIdx++) { // for distant subnets all possible local gateway
                    const uint8_t currDistantSubnet = distantSubnet[distantSubnetIdx];
                    subnetHop[currDistantSubnet] = 1;

                    for (uint8_t localGatewayIdx = 0; localGatewayIdx < localGatewayCnt; localGatewayIdx++) {
                        const uint16_t currlocalGateway = localGateway[localGatewayIdx];
                        const uint8_t currSubnetGatewayCnt = subnetGatewayCnt[currDistantSubnet];

                        subnetGateways[currDistantSubnet][currSubnetGatewayCnt] = currlocalGateway;
                        subnetGatewayCnt[currDistantSubnet]++;
                    }
                }
#endif
            }
            else {

#if 0

                for (uint8_t distantSubnetIdx = 0; distantSubnetIdx < distantSubnetCnt; distantSubnetIdx++) {
                    const uint8_t currDistantSubnet = distantSubnet[distantSubnetIdx];

                    if (subnetHop[currDistantSubnet] == 1) { // we dont want to get a gateway from another distant subnet that will add hops
                        continue;
                    }

                     for (uint8_t distantSubnetIdx2 = 0; distantSubnetIdx2 < distantSubnetCnt; distantSubnetIdx2++) {
                         if (distantSubnetIdx == distantSubnetIdx2) {
                             continue;
                         }
                         const uint8_t currDistantSubnet2 = distantSubnet[distantSubnetIdx2];
                         // if another distant subnet in pos has a local gateway to it, pass its distinct gateways
                         if (subnetGatewayCnt[currDistantSubnet2]) {

                             uint16_t selectedGateways[MAX_SUBNET];
                             for (uint8_t subnetGatewayIdx2 = 0; subnetGatewayIdx2 < subnetGatewayCnt[currDistantSubnet2]; subnetGatewayIdx2++) { // local gateways of idx 2
                                 bool distinctGateway = true;
                                 for (uint8_t subnetGatewayIdx = 0; subnetGatewayIdx < subnetGatewayCnt[currDistantSubnet]; subnetGatewayIdx++) { // local gateways of idx 1
                                     if (subnetGateways[currDistantSubnet][subnetGatewayIdx] ==
                                         subnetGateways[currDistantSubnet2][subnetGatewayIdx2]) {
                                         distinctGateway = false;
                                         break;
                                     }
                                 }
                                 
                                 if (distinctGateway) {
                                     uint8_t newHop = subnetHop[currDistantSubnet2] + 1; // this is new hop if we add this local gateway from a distant subnet

                                     if (subnetHop[currDistantSubnet] > newHop) { // clear old gateways since we have found a lower hop gateway
                                         subnetGatewayCnt[currDistantSubnet] = 0;
                                         subnetGateways[currDistantSubnet][0] = subnetGateways[currDistantSubnet2][subnetGatewayIdx2];
                                         subnetHop[currDistantSubnet] = newHop;
                                     }
                                     else if (subnetHop[currDistantSubnet] == newHop) { // samehop gateway subnet append to subnets
                                         subnetGateways[currDistantSubnet][subnetGatewayCnt[currDistantSubnet]++] = subnetGateways[currDistantSubnet2][subnetGatewayIdx2];
                                     }
                                 }
                             }
                         }
                     }
                }
#endif
            }
            // try to resolve unreachable subnets with gateways

        }

#if 0
        if (pos == myPos) { // end route computation
            break;
        }
#endif
    }

    // route table computation
    for (int pos = 1; pos < MAX_POS; pos++) {

        uint8_t subnetHop[MAX_SUBNET];
        memset(subnetHop, 0xFF, MAX_SUBNET);

        uint16_t subnetGateways[MAX_SUBNET][MAX_POS];
        uint8_t subnetGatewayCnt[MAX_SUBNET];
        memset(subnetGatewayCnt, 0x0, MAX_SUBNET);

        // first compute addresses 
        for (int peerPos = 1; peerPos < MAX_POS; peerPos++) {
            if (pos == peerPos) {
                continue;
            }

            /* resolve local reachable */
            uint8_t unReachableSubnetCnt = 0;
            uint8_t unReachableSubnet[MAX_SUBNET];

            //uint8_t reachableSubnetCnt = 0;
            //uint8_t reachableSubnet[MAX_SUBNET];

            uint8_t distantSubnetCnt = 0;
            uint8_t distantSubnet[MAX_SUBNET];

            uint8_t localGatewayCnt = 0;
            uint16_t localGateway[MAX_SUBNET];


            for (int peerPort = 0; peerPort < MAX_PORT; peerPort++) {
                uint8_t peerSubnet = topology[peerPos].subnet[peerPort];

                if (!peerSubnet) {
                    continue;
                }

                bool isDistantSubnet = true;

                for (int port = 0; port < MAX_PORT; port++) {
                    uint8_t mySubnet = topology[pos].subnet[port];
                    if (!mySubnet) {
                        continue;
                    }

                    if (mySubnet == peerSubnet) {
                        isDistantSubnet = false;
                        break;
                    }
                }

                if (isDistantSubnet) {
                    // set as possible gateway for this distant subnet
                    distantSubnet[distantSubnetCnt++] = peerSubnet;
                }
                else {

                    subnetHop[peerSubnet] = 0;
                    localGateway[localGatewayCnt++] = l3AddrTableTest[peerPos][peerPort];
                }
            }

            if (localGatewayCnt > 0) { // single hop 

                for (uint8_t distantSubnetIdx = 0; distantSubnetIdx < distantSubnetCnt; distantSubnetIdx++) { // for distant subnets all possible local gateway
                    const uint8_t currDistantSubnet = distantSubnet[distantSubnetIdx];
                    subnetHop[currDistantSubnet] = 1;

                    for (uint8_t localGatewayIdx = 0; localGatewayIdx < localGatewayCnt; localGatewayIdx++) {
                        const uint16_t currlocalGateway = localGateway[localGatewayIdx];
                        const uint8_t currSubnetGatewayCnt = subnetGatewayCnt[currDistantSubnet];

                        subnetGateways[currDistantSubnet][currSubnetGatewayCnt] = currlocalGateway;
                        subnetGatewayCnt[currDistantSubnet]++;
                    }
                }
            }
            else {

#if 0
                if (distantSubnetCnt == 1) { // we can only reach this pos on this port address
                    busLoad[distantSubnet[0]]++; // tx from pos to peer pos on distant subnet
                    continue;
                }
#endif

                for (uint8_t distantSubnetIdx = 0; distantSubnetIdx < distantSubnetCnt; distantSubnetIdx++) {
                    const uint8_t currDistantSubnet = distantSubnet[distantSubnetIdx];

                    if (subnetHop[currDistantSubnet] == 1) { // we dont want to get a gateway from another distant subnet that will add hops
                        continue;
                    }

                    for (uint8_t distantSubnetIdx2 = 0; distantSubnetIdx2 < distantSubnetCnt; distantSubnetIdx2++) {
                        if (distantSubnetIdx == distantSubnetIdx2) {
                            continue;
                        }
                        const uint8_t currDistantSubnet2 = distantSubnet[distantSubnetIdx2];
                        // if another distant subnet in pos has a local gateway to it, pass its distinct gateways
                        if (subnetGatewayCnt[currDistantSubnet2]) {

                            uint16_t selectedGateways[MAX_SUBNET];
                            for (uint8_t subnetGatewayIdx2 = 0; subnetGatewayIdx2 < subnetGatewayCnt[currDistantSubnet2]; subnetGatewayIdx2++) { // local gateways of idx 2
                                bool distinctGateway = true;
                                for (uint8_t subnetGatewayIdx = 0; subnetGatewayIdx < subnetGatewayCnt[currDistantSubnet]; subnetGatewayIdx++) { // local gateways of idx 1
                                    if (subnetGateways[currDistantSubnet][subnetGatewayIdx] ==
                                        subnetGateways[currDistantSubnet2][subnetGatewayIdx2]) {
                                        distinctGateway = false;
                                        break;
                                    }
                                }

                                if (distinctGateway) {
                                    uint8_t newHop = subnetHop[currDistantSubnet2] + 1; // this is new hop if we add this local gateway from a distant subnet

                                    if (subnetHop[currDistantSubnet] > newHop) { // clear old gateways since we have found a lower hop gateway
                                        subnetGatewayCnt[currDistantSubnet] = 0;
                                        subnetGateways[currDistantSubnet][0] = subnetGateways[currDistantSubnet2][subnetGatewayIdx2];
                                        subnetHop[currDistantSubnet] = newHop;
                                    }
                                    else if (subnetHop[currDistantSubnet] == newHop) { // samehop gateway subnet append to subnets
                                        subnetGateways[currDistantSubnet][subnetGatewayCnt[currDistantSubnet]++] = subnetGateways[currDistantSubnet2][subnetGatewayIdx2];
                                    }
                                }
                            }
                        }
                    }
                }
            }
            // try to resolve unreachable subnets with gateways
        }

        for (int peerPos = 1; peerPos < MAX_POS; peerPos++) {
            bool remoteDev = false;

            uint8_t remoteBusLoad = 0xFF;
            uint16_t selectedGateway = 0x0;

            for (int peerPort = 0; peerPort < MAX_PORT; peerPort++) {
                uint8_t peerSubnet = topology[peerPos].subnet[peerPort];

                for (int port = 0; port < MAX_PORT; port++) {
                    if (peerSubnet == topology[pos].subnet[port]) {
                        remoteDev = false;
                        break;
                    }
                }

                if (!remoteDev) {
                    break;
                }

                //select lowest loaded gateway traverse every path
                for (uint8_t subnetGatewayIdx = 0; subnetGatewayIdx < subnetGatewayCnt[peerSubnet]; subnetGatewayIdx++) {
                    uint8_t currsubnetGateway = subnetGateways[subnetGatewayIdx];
                    uint8_t subnetLoad = busLoad[subnetGateways[peerSubnet][subnetGatewayIdx]];

                    if (remoteBusLoad <= subnetLoad) {
                        continue;
                    }

                    while (subnetHop[currsubnetGateway] < 1) { // single hop gateways already resolved and selected for

                        for (uint8_t subnetGatewayIdx2 = 0; subnetGatewayIdx2 < subnetGatewayCnt[currsubnetGateway]; subnetGatewayIdx2++) {
                            uint8_t currsubnetGateway2 = subnetGateways[subnetGatewayIdx2];
                            uint8_t subnetLoad2 = busLoad[subnetGateways[currsubnetGateway][subnetGatewayIdx2]];

                        }
                    }

                   
                    if (remoteBusLoad > subnetLoad) {
                        // compute total load to reach device from this subnet

                    }
                }
            }
        }

        if (pos == myPos) { // end route computation
            break;
        }
    }

#if 0
    memset(l3AddrTableTest, 0, (MAX_POS * MAX_PORT));
    

    uint8_t reachableSubnetCnt = MAX_PORT;
    uint8_t reachableSubnet[MAX_SUBNET];
    for (uint8_t port = 0; port < MAX_PORT; port++) {
        const uint8_t subnet = topology[myPos].subnet[port];
        subnetHop[subnet] = 0;
        reachableSubnet[port] = subnet;
    }

    uint8_t unReachableSubnetCnt = 0;
    uint8_t unReachableSubnet[MAX_SUBNET];

    for (int pos = 0; pos < MAX_POS; pos++) {
        for (uint8_t port = 0; port < MAX_PORT; port++) {
            const uint8_t subnet = topology[pos].subnet[port];
            l3AddrTableTest[pos][port] = (subnet << 8) | l2Addr[subnet][port];

            if (pos != myPos) {

                uint8_t subnetIdx = 0;
                uint8_t load = busLoad[reachableSubnet[subnetIdx++]];
                for (; subnetIdx < reachableSubnetCnt; subnetIdx) {
                    if (load < )
                }

                bool unreachable = true;
                for (uint8_t myPort = 0; myPort < MAX_PORT; myPort++) {
                    if (subnet == topology[myPos].subnet[myPort]) {
                        unreachable = false;
                    }
                }

                if (unreachable) { // localy unreachable
                    // check existing reachable

                }
                if (unreachable) {
                    // select best route to reach subnet
                    uint8_t subnetIdx = 0;
                    uint8_t load = busLoad[reachableSubnet[subnetIdx]];
                    for (; subnetIdx < reachableSubnetCnt; subnetIdx) {

                    }
                }
            }
        }

        if (pos == myPos) {
            for (uint8_t port = 0; port < MAX_PORT; port++) {
                const uint8_t subnet = topology[pos].subnet[port];
                port_addr[port] = (subnet << 8) | l2Addr[subnet][port];
                l2Addr[subnet][port]++;
            }
        }
        else {
            bool unreachable = true;
            for (int peerPort = 0; peerPort < MAX_PORT; peerPort++) {
                const uint8_t subnet = topology[pos].subnet[peerPort];
                l3AddrTableTest[pos][peerPort] = (topology[pos].subnet[peerPort] << 8) | l2Addr[port];
                for (uint8_t port = 0; port < MAX_PORT; port++) {
                    if (topology[pos].subnet[peerPort] == topology[myPos].subnet[port]) {
                        unreachable = false;
                    }
                }
            }
        }
    }
#endif
}
#endif

#if 0

void setPortAddr() {
    // set port addr
    uint8_t l2Addr[MAX_SUBNET][MAX_PORT];
    memset(l2Addr, 1, (MAX_POS * MAX_PORT));

    uint8_t busLoad[MAX_SUBNET];
    memset(busLoad, 0, MAX_SUBNET);

    memset(l3AddrTableTest, 0, (MAX_POS * MAX_PORT));
    uint8_t subnetHop[MAX_SUBNET];
    memset(subnetHop, 0xFF, MAX_SUBNET);

    uint8_t reachableSubnetCnt = MAX_PORT;
    uint8_t reachableSubnet[MAX_SUBNET];
    for (uint8_t port = 0; port < MAX_PORT; port++) {
        const uint8_t subnet = topology[myPos].subnet[port];
        subnetHop[subnet] = 0;
        reachableSubnet[port] = subnet;
    }

    uint8_t unReachableSubnetCnt = 0;
    uint8_t unReachableSubnet[MAX_SUBNET];

    for (int pos = 0; pos < MAX_POS; pos++) {
        for (uint8_t port = 0; port < MAX_PORT; port++) {
            const uint8_t subnet = topology[pos].subnet[port];
            l3AddrTableTest[pos][port] = (subnet << 8) | l2Addr[subnet][port];

            if (pos != myPos) {

                uint8_t subnetIdx = 0;
                uint8_t load = busLoad[reachableSubnet[subnetIdx++]];
                for (; subnetIdx < reachableSubnetCnt; subnetIdx) {
                    if (load < )
                }

                bool unreachable = true;
                for (uint8_t myPort = 0; myPort < MAX_PORT; myPort++) {
                    if (subnet == topology[myPos].subnet[myPort]) {
                        unreachable = false;
                    }
                }

                if (unreachable) { // localy unreachable
                    // check existing reachable
                    
                }
                if (unreachable) {
                    // select best route to reach subnet
                    uint8_t subnetIdx = 0;
                    uint8_t load = busLoad[reachableSubnet[subnetIdx]];
                    for (; subnetIdx < reachableSubnetCnt; subnetIdx) {

                    }
                }
            }
        }

        if (pos == myPos) {
            for (uint8_t port = 0; port < MAX_PORT; port++) {
                const uint8_t subnet = topology[pos].subnet[port];
                port_addr[port] = (subnet << 8) | l2Addr[subnet][port];
                l2Addr[subnet][port]++;
            }
        }
        else {
            bool unreachable = true;
            for (int peerPort = 0; peerPort < MAX_PORT; peerPort++) {
                const uint8_t subnet = topology[pos].subnet[peerPort];
                l3AddrTableTest[pos][peerPort] = (topology[pos].subnet[peerPort] << 8) | l2Addr[port];
                for (uint8_t port = 0; port < MAX_PORT; port++) {
                    if (topology[pos].subnet[peerPort] == topology[myPos].subnet[port]) {
                        unreachable = false;
                    }
                }
            }
        }
    }
}


void setPortAddr() {
    // set port addr
    uint8_t l2Addr[MAX_PORT] = { 
        (topology[myPos].subnet[0] == 0) ? 0 : 1, 
        (topology[myPos].subnet[1] == 0) ? 0 : 1 
    };

    uint8_t busLoad[MAX_SUBNET];
    memset(busLoad, 0, MAX_SUBNET);

    memset(l3AddrTableTest, 0, (MAX_POS * MAX_PORT));
    uint8_t subnetHop[MAX_SUBNET];
    memset(subnetHop, 0xFF, MAX_SUBNET);

    for (uint8_t port = 0; port < MAX_PORT; port++) {
        subnetHop[topology[myPos].subnet[port]] = 0;
    }

    for (int pos = 0; pos < MAX_POS; pos++) {
        if (pos == myPos) {
            for (uint8_t port = 0; port < MAX_PORT; port++) {
                port_addr[port] = (topology[pos].subnet[port] << 8) | l2Addr[port];
            }
        }
        else {
            bool unreachable = true;
            for (int peerPort = 0; peerPort < MAX_PORT; peerPort++) {
                for (uint8_t port = 0; port < MAX_PORT; port++) {
                    if (topology[pos].subnet[peerPort] == topology[myPos].subnet[port]) {
                        unreachable = false;
                    }
                }
            }

            // one subnet directly reachable
            for (uint8_t port = 0; port < MAX_PORT; port++) {
                if (topology[myPos].subnet[port] == 0) {
                    continue;
                }

                for (int peerPort = 0; peerPort < MAX_PORT; peerPort++) {
                    l3AddrTableTest[pos][peerPort] = (topology[pos].subnet[port] << 8) | l2Addr[port];
                    if (topology[pos].subnet[peerPort] == topology[myPos].subnet[port]) {
                        // for now set addr directly (TODO loadbalance equal hops + routing table gateway)
                        //streams[pos].gateway = streams[pos].dst = (topology[pos].subnet[port] << 8) | l2Addr[port];
                        l3RouteTable[pos] = 
                        l3AddrTable[pos] = (topology[pos].subnet[port] << 8) | l2Addr[port];

                        l2Addr[port]++;

                        l3RouteTableTest[pos][peerPort] = l3AddrTableTest[pos][peerPort];
                    }
                }
            }
        }
    }

    for (uint8_t port = 0; port < MAX_PORT; port++) { // max addr for MST rollover
        maxL2Addr[port] = l2Addr[port];
        if (maxL2Addr[port] <= 1) { // deactivate port if only single device
            port_addr[port] = 0;
        }
    }
}
#endif
void netInit(UART_Type* UART[MAX_PORT]) {
    pages_init();
    setPortAddr();
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
