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
//void scheduler();

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
            // one subnet directly reachable
            for (uint8_t port = 0; port < MAX_PORT; port++) {
                if (l2Addr[port] == 0) {
                    continue;
                }

                bool reachableOnBothSubnets = false;
                for (int peerPort = 0; peerPort < MAX_PORT; peerPort++) {
                    if (topology[pos].subnet[peerPort] == topology[myPos].subnet[port]) {
                        // for now set addr directly (TODO loadbalance equal hops + routing table gateway)
                        //streams[pos].gateway = streams[pos].dst = (topology[pos].subnet[port] << 8) | l2Addr[port];
                        l3RouteTable[pos] = 
                        l3AddrTable[pos] = (topology[pos].subnet[port] << 8) | l2Addr[port];

                        l2Addr[port]++;
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
