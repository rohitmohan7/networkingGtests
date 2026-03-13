#include "network.h"
#include "layer2.h"
#include "layer3.h"
#include "layer1.h"
#include "layer4.h"
#include "allocator.h"

uint16_t port_addr[MAX_PORT]; // L3 & L2

// TODO move to cfg
uint16_t myPos;
NodeCfg topology[MAX_POS];

#define INVALID_SUBNET              0xFFu
#define INVALID_GATEWAY             0xFFFFu

#define MAX_FLOWS                   (MAX_POS * MAX_POS)
#define MAX_PATH_SUBNETS            (MAX_SUBNET)
#define MAX_CANDIDATES_PER_FLOW     32   /* increase if needed */

/* ------------------------------- outputs -------------------------------- */

uint16_t l3AddrTableTest[MAX_POS][MAX_PORT];

/*
 * Route to destination subnet:
 *   0       -> directly connected
 *   0xFFFF  -> unreachable
 *   else    -> first-hop gateway L3 address
 */
uint16_t l3RouteTableTest[MAX_POS][MAX_SUBNET];

/*
 * Chosen global one-way flow decision for srcPos -> dstPos:
 *   nodeFirstGatewayTable[src][dst] = 0       if direct
 *   nodeFirstGatewayTable[src][dst] = 0xFFFF  if unreachable
 *   otherwise first-hop gateway address
 *
 * nodeChosenDstSubnetTable[src][dst] tells which destination subnet
 * was selected for that destination node in the global optimum.
 */
uint16_t nodeFirstGatewayTable[MAX_POS][MAX_POS];
uint8_t  nodeChosenDstSubnetTable[MAX_POS][MAX_POS];

/* Final globally optimized per-subnet one-way load */
uint16_t optimizedBusLoad[MAX_SUBNET];

/* ---------------------- route building helper state ---------------------- */

typedef struct
{
    uint8_t  reachable;
    uint8_t  hops;
    uint32_t load;
    uint16_t firstGw;
    uint8_t  prevSubnet;
    int16_t  prevPeerPos;
} RouteInfo;

static bool better_route(uint8_t newHops,
    uint32_t newLoad,
    uint8_t oldHops,
    uint32_t oldLoad)
{
    if (newHops < oldHops) {
        return true;
    }
    if ((newHops == oldHops) && (newLoad < oldLoad)) {
        return true;
    }
    return false;
}

/* ------------------------- candidate path structs ------------------------ */

typedef struct
{
    uint8_t  subnetCount;                     /* number of traversed subnets */
    uint8_t  subnets[MAX_PATH_SUBNETS];       /* ordered list of traversed subnets */
    uint8_t  hops;                            /* gateway hops */
    uint16_t firstGw;                         /* first-hop gateway address, 0 if direct */
    uint8_t  dstSubnet;                       /* destination subnet used for dstPos */
} PathCandidate;

typedef struct
{
    uint8_t srcPos;
    uint8_t dstPos;
    uint8_t candidateCount;
    uint8_t overflowed;                       /* exactness lost if 1 */
    PathCandidate candidate[MAX_CANDIDATES_PER_FLOW];
} Flow;

/* ------------------------------- utilities ------------------------------- */

static bool node_is_used(int pos)
{
    for (int port = 0; port < MAX_PORT; port++) {
        if (topology[pos].subnet[port] != 0) {
            return true;
        }
    }
    return false;
}

static bool node_has_subnet(int pos, uint8_t subnet)
{
    if (subnet == 0) {
        return false;
    }

    for (int port = 0; port < MAX_PORT; port++) {
        if (topology[pos].subnet[port] == subnet) {
            return true;
        }
    }
    return false;
}

static bool is_subnet_in_node_multiple_times_safe(int pos, uint8_t subnet, int* firstPort)
{
    for (int port = 0; port < MAX_PORT; port++) {
        if (topology[pos].subnet[port] == subnet) {
            *firstPort = port;
            return true;
        }
    }
    return false;
}

static uint16_t recompute_max_load(const uint16_t load[MAX_SUBNET])
{
    uint16_t maxLoad = 0;

    for (int s = 1; s < MAX_SUBNET; s++) {
        if (load[s] > maxLoad) {
            maxLoad = load[s];
        }
    }

    return maxLoad;
}

static bool candidate_equals(const PathCandidate* a, const PathCandidate* b)
{
    if (a->subnetCount != b->subnetCount) {
        return false;
    }
    if (a->hops != b->hops) {
        return false;
    }
    if (a->firstGw != b->firstGw) {
        return false;
    }
    if (a->dstSubnet != b->dstSubnet) {
        return false;
    }

    for (uint8_t i = 0; i < a->subnetCount; i++) {
        if (a->subnets[i] != b->subnets[i]) {
            return false;
        }
    }

    return true;
}

/* -------------------------- L3 address assignment ------------------------ */

static void assign_l3_addresses(void)
{
    uint8_t l2Addr[MAX_SUBNET];

    memset(l2Addr, 0, sizeof(l2Addr));
    memset(l3AddrTableTest, 0, sizeof(l3AddrTableTest));

    for (int pos = 1; pos < MAX_POS; pos++) {
        for (int port = 0; port < MAX_PORT; port++) {
            uint8_t subnet = topology[pos].subnet[port];
            if (subnet == 0) {
                continue;
            }

            l2Addr[subnet]++;
            l3AddrTableTest[pos][port] = ((uint16_t)subnet << 8) | l2Addr[subnet];
        }
    }
}

/* ----------------------- shortest-path route solving --------------------- */

/*
 * Compute best routes from one source node to all subnets using a fixed
 * busLoad snapshot. This is used later to build l3RouteTableTest after the
 * global optimizer has chosen final loads.
 */
static void compute_routes_for_pos(int srcPos,
    const uint16_t busLoad[MAX_SUBNET],
    RouteInfo route[MAX_SUBNET])
{
    for (int s = 0; s < MAX_SUBNET; s++) {
        route[s].reachable = 0;
        route[s].hops = 0xFFu;
        route[s].load = 0xFFFFFFFFu;
        route[s].firstGw = INVALID_GATEWAY;
        route[s].prevSubnet = INVALID_SUBNET;
        route[s].prevPeerPos = -1;
    }

    for (int port = 0; port < MAX_PORT; port++) {
        uint8_t subnet = topology[srcPos].subnet[port];
        if (subnet == 0) {
            continue;
        }

        route[subnet].reachable = 1;
        route[subnet].hops = 0;
        route[subnet].load = busLoad[subnet];
        route[subnet].firstGw = 0;
        route[subnet].prevSubnet = INVALID_SUBNET;
        route[subnet].prevPeerPos = -1;
    }

    for (int pass = 0; pass < MAX_SUBNET - 1; pass++) {
        bool changed = false;

        for (int peerPos = 1; peerPos < MAX_POS; peerPos++) {
            if (peerPos == srcPos) {
                continue;
            }

            for (int inPort = 0; inPort < MAX_PORT; inPort++) {
                uint8_t inSubnet = topology[peerPos].subnet[inPort];
                if (inSubnet == 0) {
                    continue;
                }

                if (!route[inSubnet].reachable) {
                    continue;
                }

                uint16_t gwAddrOnInSubnet = l3AddrTableTest[peerPos][inPort];
                uint8_t newHops = (uint8_t)(route[inSubnet].hops + 1);
                uint16_t newFirstGw = (route[inSubnet].hops == 0)
                    ? gwAddrOnInSubnet
                    : route[inSubnet].firstGw;

                for (int outPort = 0; outPort < MAX_PORT; outPort++) {
                    uint8_t outSubnet = topology[peerPos].subnet[outPort];
                    if ((outSubnet == 0) || (outSubnet == inSubnet)) {
                        continue;
                    }

                    uint32_t newLoad = route[inSubnet].load + busLoad[outSubnet];

                    if (!route[outSubnet].reachable ||
                        better_route(newHops, newLoad,
                            route[outSubnet].hops, route[outSubnet].load)) {
                        route[outSubnet].reachable = 1;
                        route[outSubnet].hops = newHops;
                        route[outSubnet].load = newLoad;
                        route[outSubnet].firstGw = newFirstGw;
                        route[outSubnet].prevSubnet = inSubnet;
                        route[outSubnet].prevPeerPos = (int16_t)peerPos;
                        changed = true;
                    }
                }
            }
        }

        if (!changed) {
            break;
        }
    }
}

/* ------------------ candidate generation for each flow ------------------- */

static void bfs_shortest_hops_from_src(int srcPos, uint8_t dist[MAX_SUBNET])
{
    uint8_t queue[MAX_SUBNET];
    int qHead = 0;
    int qTail = 0;

    for (int s = 0; s < MAX_SUBNET; s++) {
        dist[s] = 0xFFu;
    }

    for (int port = 0; port < MAX_PORT; port++) {
        uint8_t subnet = topology[srcPos].subnet[port];
        if (subnet == 0) {
            continue;
        }

        if (dist[subnet] == 0xFFu) {
            dist[subnet] = 0;
            queue[qTail++] = subnet;
        }
    }

    while (qHead < qTail) {
        uint8_t inSubnet = queue[qHead++];

        for (int peerPos = 1; peerPos < MAX_POS; peerPos++) {
            if (peerPos == srcPos) {
                continue;
            }

            for (int inPort = 0; inPort < MAX_PORT; inPort++) {
                if (topology[peerPos].subnet[inPort] != inSubnet) {
                    continue;
                }

                for (int outPort = 0; outPort < MAX_PORT; outPort++) {
                    uint8_t outSubnet = topology[peerPos].subnet[outPort];
                    if ((outSubnet == 0) || (outSubnet == inSubnet)) {
                        continue;
                    }

                    if (dist[outSubnet] == 0xFFu) {
                        dist[outSubnet] = (uint8_t)(dist[inSubnet] + 1);
                        queue[qTail++] = outSubnet;
                    }
                }
            }
        }
    }
}

static void record_candidate(Flow* flow,
    const uint8_t pathSubnets[MAX_PATH_SUBNETS],
    uint8_t pathLen,
    uint8_t hops,
    uint16_t firstGw,
    uint8_t dstSubnet)
{
    PathCandidate temp;
    temp.subnetCount = pathLen;
    temp.hops = hops;
    temp.firstGw = firstGw;
    temp.dstSubnet = dstSubnet;

    for (uint8_t i = 0; i < pathLen; i++) {
        temp.subnets[i] = pathSubnets[i];
    }

    for (uint8_t i = 0; i < flow->candidateCount; i++) {
        if (candidate_equals(&temp, &flow->candidate[i])) {
            return;
        }
    }

    if (flow->candidateCount >= MAX_CANDIDATES_PER_FLOW) {
        flow->overflowed = 1;
        return;
    }

    flow->candidate[flow->candidateCount++] = temp;
}

static void dfs_collect_shortest_candidates(int srcPos,
    int dstPos,
    uint8_t currentSubnet,
    uint8_t minDstHops,
    const uint8_t dist[MAX_SUBNET],
    uint8_t pathSubnets[MAX_PATH_SUBNETS],
    uint8_t pathLen,
    uint16_t firstGw,
    bool firstGwSet,
    Flow* flow)
{
    if (pathLen == 0 || pathLen > MAX_PATH_SUBNETS) {
        return;
    }

    if (node_has_subnet(dstPos, currentSubnet) && (dist[currentSubnet] == minDstHops)) {
        record_candidate(flow,
            pathSubnets,
            pathLen,
            minDstHops,
            firstGwSet ? firstGw : 0,
            currentSubnet);
        return;
    }

    if (dist[currentSubnet] >= minDstHops) {
        return;
    }

    for (int peerPos = 1; peerPos < MAX_POS; peerPos++) {
        if (peerPos == srcPos) {
            continue;
        }

        for (int inPort = 0; inPort < MAX_PORT; inPort++) {
            if (topology[peerPos].subnet[inPort] != currentSubnet) {
                continue;
            }

            uint16_t gwAddr = l3AddrTableTest[peerPos][inPort];

            for (int outPort = 0; outPort < MAX_PORT; outPort++) {
                uint8_t outSubnet = topology[peerPos].subnet[outPort];
                if ((outSubnet == 0) || (outSubnet == currentSubnet)) {
                    continue;
                }

                if (dist[outSubnet] != (uint8_t)(dist[currentSubnet] + 1)) {
                    continue;
                }

                if (pathLen >= MAX_PATH_SUBNETS) {
                    continue;
                }

                pathSubnets[pathLen] = outSubnet;

                dfs_collect_shortest_candidates(srcPos,
                    dstPos,
                    outSubnet,
                    minDstHops,
                    dist,
                    pathSubnets,
                    (uint8_t)(pathLen + 1),
                    firstGwSet ? firstGw : gwAddr,
                    true,
                    flow);
            }
        }
    }
}

static void generate_flow_candidates(int srcPos, int dstPos, Flow* flow)
{
    uint8_t dist[MAX_SUBNET];
    uint8_t minDstHops = 0xFFu;
    uint8_t pathSubnets[MAX_PATH_SUBNETS];

    memset(flow, 0, sizeof(*flow));
    flow->srcPos = (uint8_t)srcPos;
    flow->dstPos = (uint8_t)dstPos;

    if (!node_is_used(srcPos) || !node_is_used(dstPos) || (srcPos == dstPos)) {
        return;
    }

    bfs_shortest_hops_from_src(srcPos, dist);

    for (int port = 0; port < MAX_PORT; port++) {
        uint8_t dstSubnet = topology[dstPos].subnet[port];
        if (dstSubnet == 0) {
            continue;
        }

        if (dist[dstSubnet] < minDstHops) {
            minDstHops = dist[dstSubnet];
        }
    }

    if (minDstHops == 0xFFu) {
        return; /* unreachable */
    }

    for (int port = 0; port < MAX_PORT; port++) {
        uint8_t srcSubnet = topology[srcPos].subnet[port];
        if (srcSubnet == 0) {
            continue;
        }

        if (dist[srcSubnet] != 0) {
            continue;
        }

        pathSubnets[0] = srcSubnet;

        if ((minDstHops == 0) && node_has_subnet(dstPos, srcSubnet)) {
            record_candidate(flow, pathSubnets, 1, 0, 0, srcSubnet);
            continue;
        }

        dfs_collect_shortest_candidates(srcPos,
            dstPos,
            srcSubnet,
            minDstHops,
            dist,
            pathSubnets,
            1,
            0,
            false,
            flow);
    }
}

/* ------------------------ exact global optimization ---------------------- */

typedef struct
{
    Flow flows[MAX_FLOWS];
    int flowCount;

    int reachableFlowIndex[MAX_FLOWS];
    int reachableFlowCount;

    int currentChoice[MAX_FLOWS];
    int bestChoice[MAX_FLOWS];

    uint16_t currentLoad[MAX_SUBNET];
    uint16_t bestLoad[MAX_SUBNET];

    uint64_t currentSumSquares;
    uint16_t bestMaxLoad;
    uint64_t bestSumSquares;

    bool found;
} SearchState;

static void sort_reachable_flows_for_search(SearchState* st)
{
    for (int i = 0; i < st->reachableFlowCount; i++) {
        for (int j = i + 1; j < st->reachableFlowCount; j++) {
            Flow* fi = &st->flows[st->reachableFlowIndex[i]];
            Flow* fj = &st->flows[st->reachableFlowIndex[j]];

            uint8_t fiCand = fi->candidateCount;
            uint8_t fjCand = fj->candidateCount;
            uint8_t fiHops = (fiCand > 0) ? fi->candidate[0].hops : 0;
            uint8_t fjHops = (fjCand > 0) ? fj->candidate[0].hops : 0;

            bool swap = false;

            if (fiCand > fjCand) {
                swap = true;
            }
            else if ((fiCand == fjCand) && (fiHops < fjHops)) {
                swap = true;
            }

            if (swap) {
                int tmp = st->reachableFlowIndex[i];
                st->reachableFlowIndex[i] = st->reachableFlowIndex[j];
                st->reachableFlowIndex[j] = tmp;
            }
        }
    }
}

static void apply_candidate_load(const PathCandidate* cand,
    uint16_t load[MAX_SUBNET],
    uint64_t* sumSquares)
{
    for (uint8_t i = 0; i < cand->subnetCount; i++) {
        uint8_t s = cand->subnets[i];
        uint16_t old = load[s];
        load[s] = (uint16_t)(old + 1);
        *sumSquares += (uint64_t)(2u * old + 1u);
    }
}

static void undo_candidate_load(const PathCandidate* cand,
    uint16_t load[MAX_SUBNET],
    uint64_t* sumSquares)
{
    for (uint8_t i = 0; i < cand->subnetCount; i++) {
        uint8_t s = cand->subnets[i];
        load[s] = (uint16_t)(load[s] - 1);
        *sumSquares -= (uint64_t)(2u * load[s] + 1u);
    }
}

static void search_exact_global(SearchState* st, int depth, uint16_t currentMaxLoad)
{
    if (st->found) {
        if (currentMaxLoad > st->bestMaxLoad) {
            return;
        }
        if ((currentMaxLoad == st->bestMaxLoad) &&
            (st->currentSumSquares >= st->bestSumSquares)) {
            return;
        }
    }

    if (depth >= st->reachableFlowCount) {
        st->found = true;
        st->bestMaxLoad = currentMaxLoad;
        st->bestSumSquares = st->currentSumSquares;
        memcpy(st->bestChoice, st->currentChoice, sizeof(st->bestChoice));
        memcpy(st->bestLoad, st->currentLoad, sizeof(st->bestLoad));
        return;
    }

    int flowIdx = st->reachableFlowIndex[depth];
    Flow* flow = &st->flows[flowIdx];

    for (int candIdx = 0; candIdx < flow->candidateCount; candIdx++) {
        const PathCandidate* cand = &flow->candidate[candIdx];

        apply_candidate_load(cand, st->currentLoad, &st->currentSumSquares);
        st->currentChoice[flowIdx] = candIdx;

        uint16_t newMaxLoad = recompute_max_load(st->currentLoad);

        search_exact_global(st, depth + 1, newMaxLoad);

        undo_candidate_load(cand, st->currentLoad, &st->currentSumSquares);
        st->currentChoice[flowIdx] = -1;
    }
}

static bool optimize_all_flows_exact(void)
{
    SearchState st;
    memset(&st, 0, sizeof(st));

    for (int i = 0; i < MAX_FLOWS; i++) {
        st.currentChoice[i] = -1;
        st.bestChoice[i] = -1;
    }

    /* Build all ordered flows srcPos -> dstPos */
    for (int srcPos = 1; srcPos < MAX_POS; srcPos++) {
        if (!node_is_used(srcPos)) {
            continue;
        }

        for (int dstPos = 1; dstPos < MAX_POS; dstPos++) {
            if (!node_is_used(dstPos) || (srcPos == dstPos)) {
                continue;
            }

            if (st.flowCount >= MAX_FLOWS) {
                return false;
            }

            generate_flow_candidates(srcPos, dstPos, &st.flows[st.flowCount]);

            if (st.flows[st.flowCount].overflowed) {
                return false; /* increase MAX_CANDIDATES_PER_FLOW */
            }

            if (st.flows[st.flowCount].candidateCount > 0) {
                st.reachableFlowIndex[st.reachableFlowCount++] = st.flowCount;
            }

            st.flowCount++;
        }
    }

    sort_reachable_flows_for_search(&st);

    memset(st.currentLoad, 0, sizeof(st.currentLoad));
    memset(st.bestLoad, 0, sizeof(st.bestLoad));
    st.currentSumSquares = 0;
    st.bestMaxLoad = 0xFFFFu;
    st.bestSumSquares = 0xFFFFFFFFFFFFFFFFull;
    st.found = false;

    search_exact_global(&st, 0, 0);

    if (!st.found) {
        /*
         * This can still happen if there are simply no reachable flows.
         * We treat that as success with zero load.
         */
        memset(optimizedBusLoad, 0, sizeof(optimizedBusLoad));
    }
    else {
        memcpy(optimizedBusLoad, st.bestLoad, sizeof(optimizedBusLoad));
    }

    /* Fill node-level chosen route tables */
    for (int srcPos = 0; srcPos < MAX_POS; srcPos++) {
        for (int dstPos = 0; dstPos < MAX_POS; dstPos++) {
            nodeFirstGatewayTable[srcPos][dstPos] = INVALID_GATEWAY;
            nodeChosenDstSubnetTable[srcPos][dstPos] = INVALID_SUBNET;
        }
    }

    for (int i = 0; i < st.flowCount; i++) {
        Flow* flow = &st.flows[i];
        int srcPos = flow->srcPos;
        int dstPos = flow->dstPos;

        if (flow->candidateCount == 0) {
            nodeFirstGatewayTable[srcPos][dstPos] = INVALID_GATEWAY;
            nodeChosenDstSubnetTable[srcPos][dstPos] = INVALID_SUBNET;
            continue;
        }

        if (!st.found || st.bestChoice[i] < 0) {
            nodeFirstGatewayTable[srcPos][dstPos] = INVALID_GATEWAY;
            nodeChosenDstSubnetTable[srcPos][dstPos] = INVALID_SUBNET;
            continue;
        }

        const PathCandidate* bestCand = &flow->candidate[st.bestChoice[i]];
        nodeFirstGatewayTable[srcPos][dstPos] = bestCand->firstGw;
        nodeChosenDstSubnetTable[srcPos][dstPos] = bestCand->dstSubnet;
    }

    return true;
}

/* ------------------------ final subnet route table ----------------------- */

static void build_final_route_table_from_optimized_load(void)
{
    RouteInfo route[MAX_SUBNET];

    memset(l3RouteTableTest, 0xFF, sizeof(l3RouteTableTest));

    for (int srcPos = 1; srcPos < MAX_POS; srcPos++) {
        if (!node_is_used(srcPos)) {
            continue;
        }

        compute_routes_for_pos(srcPos, optimizedBusLoad, route);

        for (int dstSubnet = 1; dstSubnet < MAX_SUBNET; dstSubnet++) {
            if (!route[dstSubnet].reachable) {
                l3RouteTableTest[srcPos][dstSubnet] = INVALID_GATEWAY;
            }
            else {
                l3RouteTableTest[srcPos][dstSubnet] = route[dstSubnet].firstGw;
            }
        }
    }
}

/* ------------------------------ entry point ------------------------------ */

/*
 * Returns true if an exact global optimum was computed successfully.
 * Returns false if MAX_CANDIDATES_PER_FLOW was too small for exactness.
 */
bool setPortAddr(void)
{
    assign_l3_addresses();

    if (!optimize_all_flows_exact()) {
        memset(optimizedBusLoad, 0, sizeof(optimizedBusLoad));
        memset(nodeFirstGatewayTable, 0xFF, sizeof(nodeFirstGatewayTable));
        memset(nodeChosenDstSubnetTable, 0xFF, sizeof(nodeChosenDstSubnetTable));
        memset(l3RouteTableTest, 0xFF, sizeof(l3RouteTableTest));
        return false;
    }

    build_final_route_table_from_optimized_load();
    return true;
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
