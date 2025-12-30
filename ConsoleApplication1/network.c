#include "network.h"
#include "layer2.h"
#include "layer3.h"

uint16_t port_addr[MAX_PORT]; // L3 & L2

//void scheduler();

void setPortAddr() {
    // set port addr
    uint8_t l2Addr[MAX_PORT] = { 
        (topology[myPos].subnet[0] == 0) ? 0 : 1, 
        (topology[myPos].subnet[1] == 0) ? 0 : 1 
    };

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

                if ((topology[pos].subnet[0] == topology[myPos].subnet[port]) ||
                    (topology[pos].subnet[1] == topology[myPos].subnet[port])) {
                    l2Addr[port]++;
                }
            }
        }
    }

    for (uint8_t port = 0; port < MAX_PORT; port++) { // max addr for MST rollover
        maxL2Addr[port] = l2Addr[port];
    }
}

void netInit() {
	l2Init();
	l3Init();
    setPortAddr();
}

void netTick(uint8_t ms) {
#if 0
	for (int port = 0; port < MAX_PORT; port) {
		if (mst_token[port]) {
			scheduler();
			break;
		}
	}
#endif
	l2Tick(ms);
}