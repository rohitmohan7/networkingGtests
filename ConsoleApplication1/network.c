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

uint16_t l3AddrTableTest[MAX_POS][MAX_PORT];
uint16_t l3RouteTableTest[MAX_POS][MAX_SUBNET];
//void scheduler();

typedef struct
{
    uint8_t reachable;
    uint8_t hops;
    uint16_t load;
    uint16_t firstGw;
    uint8_t prevSubnet;
    int8_t prevPeerPos;
} RouteInfo;

static bool better_route(uint8_t newHops, uint16_t newLoad,
    uint8_t oldHops, uint16_t oldLoad)
{
    if (newHops < oldHops) {
        return true;
    }
    if (newHops == oldHops && newLoad < oldLoad) {
        return true;
    }
    return false;
}

void compute_routes_for_pos(int srcPos,
    /*NodeCfg topology[MAX_POS],
   /* uint16_t l3AddrTableTest[MAX_POS][MAX_PORT],
    uint16_t l3RouteTableTest[MAX_POS][MAX_SUBNET],*/
    uint8_t busLoad[MAX_SUBNET])
{
    RouteInfo route[MAX_SUBNET];

    for (int s = 0; s < MAX_SUBNET; s++) {
        route[s].reachable = 0;
        route[s].hops = 0xFF;
        route[s].load = 0xFFFF;
        route[s].firstGw = 0;
        route[s].prevSubnet = 0xFF;
        route[s].prevPeerPos = -1;
    }

    // Seed: source node's directly attached subnets
    for (int port = 0; port < MAX_PORT; port++) {
        uint8_t s = topology[srcPos].subnet[port];
        if (!s) {
            continue;
        }

        route[s].reachable = 1;
        route[s].hops = 0;
        route[s].load = 0;
        route[s].firstGw = 0;
        route[s].prevSubnet = 0xFF;
        route[s].prevPeerPos = -1;
    }

    // Relax routes repeatedly, Bellman-Ford style
    for (int pass = 0; pass < MAX_SUBNET - 1; pass++) {
        bool changed = false;

        for (int peerPos = 1; peerPos < MAX_POS; peerPos++) {
            if (peerPos == srcPos) {
                continue;
            }

            // Try every "input subnet" on this peer
            for (int inPort = 0; inPort < MAX_PORT; inPort++) {
                uint8_t inSubnet = topology[peerPos].subnet[inPort];
                if (!inSubnet) {
                    continue;
                }

                if (!route[inSubnet].reachable) {
                    continue;
                }

                uint16_t gwAddrOnInSubnet = l3AddrTableTest[peerPos][inPort];
                uint8_t newHopsBase = (uint8_t)(route[inSubnet].hops + 1);
                uint16_t newLoadBase = (uint16_t)(route[inSubnet].load + busLoad[inSubnet]);
                uint16_t newFirstGw = (route[inSubnet].hops == 0)
                    ? gwAddrOnInSubnet
                    : route[inSubnet].firstGw;

                // This peer can bridge from inSubnet to every other subnet it owns
                for (int outPort = 0; outPort < MAX_PORT; outPort++) {
                    uint8_t outSubnet = topology[peerPos].subnet[outPort];
                    if (!outSubnet || outSubnet == inSubnet) {
                        continue;
                    }

                    if (!route[outSubnet].reachable ||
                        better_route(newHopsBase, newLoadBase,
                            route[outSubnet].hops, route[outSubnet].load)) {

                        route[outSubnet].reachable = 1;
                        route[outSubnet].hops = newHopsBase;
                        route[outSubnet].load = newLoadBase;
                        route[outSubnet].firstGw = newFirstGw;
                        route[outSubnet].prevSubnet = inSubnet;
                        route[outSubnet].prevPeerPos = (int8_t)peerPos;
                        changed = true;
                    }
                }
            }
        }

        if (!changed) {
            break;
        }
    }

    // Write route table: one next-hop gateway per destination subnet
    for (int dstSubnet = 0; dstSubnet < MAX_SUBNET; dstSubnet++) {
        l3RouteTableTest[srcPos][dstSubnet] = route[dstSubnet].firstGw;
    }
}

void setPortAddr() {
    // set port addr
    uint8_t l2Addr[MAX_SUBNET];
    uint8_t busLoad[MAX_SUBNET];

    memset(l2Addr, 0, sizeof(l2Addr));
    memset(busLoad, 0, sizeof(busLoad));
    memset(l3RouteTableTest, 0, sizeof(l3RouteTableTest));
    memset(l3AddrTableTest, 0, sizeof(l3AddrTableTest));

    // assign all L3 port addresses once
    for (int pos = 1; pos < MAX_POS; pos++) {
        for (int port = 0; port < MAX_PORT; port++) {
            uint8_t subnet = topology[pos].subnet[port];
            if (!subnet) {
                continue;
            }

            l2Addr[subnet]++;
            l3AddrTableTest[pos][port] = ((uint16_t)subnet << 8) | l2Addr[subnet];
        }
    }

    /* Estimate bus load for direct/shared-subnet choices */
    for (int pos = 1; pos < MAX_POS; pos++) {
        for (int peerPos = 1; peerPos < MAX_POS; peerPos++) {
            if (pos == peerPos) {
                continue;
            }

            uint8_t localGatewayCnt = 0;
            uint8_t prefTxIdx = 0;
            uint8_t prefTxLoad = 0xFF;

            for (int peerPort = 0; peerPort < MAX_PORT; peerPort++) {
                uint8_t peerSubnet = topology[peerPos].subnet[peerPort];
                if (!peerSubnet) {
                    continue;
                }

                bool sharedSubnet = false;
                for (int port = 0; port < MAX_PORT; port++) {
                    if (topology[pos].subnet[port] == peerSubnet) {
                        sharedSubnet = true;
                        break;
                    }
                }

                if (!sharedSubnet) {
                    continue;
                }

                if (localGatewayCnt == 0 || busLoad[peerSubnet] < prefTxLoad) {
                    prefTxIdx = peerPort;
                    prefTxLoad = busLoad[peerSubnet];
                }
                localGatewayCnt++;
            }

            if (localGatewayCnt > 0) {
                uint8_t chosenSubnet = topology[peerPos].subnet[prefTxIdx];
                busLoad[chosenSubnet]++;
            }
        }
    }

    for (int pos = 1; pos < MAX_POS; pos++) {
        compute_routes_for_pos(pos, busLoad);
    }
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
