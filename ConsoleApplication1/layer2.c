#include "layer2.h"
#include "layer3.h"
#include "network.h"

#define LINE_SILENT 100

bool mst_token[MAX_PORT];
uint16_t l2TmLstRx[MAX_PORT];
L2PktDesc l2PktDesc[MAX_PORT];
uint8_t maxL2Addr[MAX_PORT];

void l2Init() {
	memset(mst_token, 0, sizeof mst_token);
	memset(l2TmLstRx, 0, sizeof l2TmLstRx);
	memset(l2PktDesc, 0xFF, sizeof l2PktDesc);
	memset(maxL2Addr, 0xFF, sizeof maxL2Addr);
}

void l2TmLstRxRst(uint8_t port) {
	l2TmLstRx[port] = 0;
}

/*void l2Send(L2Packet) {

}*/

uint8_t l2GetTxPkt(uint8_t port, uint8_t * ptr, uint8_t len, uint8_t idx) {

}

void l2SendMst(uint8_t port) {
	l2PktDesc[port].l2Pkt.hdr.addr = (port_addr[port] + 1) % maxL2Addr[port];  // Best way to find next table in line ? 
	l2PktDesc[port].l2Pkt.hdr.type = L2_PKT_TYPE_MST;
	l2PktDesc[port].l2Pkt.msgDesc.mst = l2PktDesc[port].l2Pkt.hdr.addr; // so we can select next MST

	//l2PktDesc->l2Pkt.crc =
	l1StartTx(port);
}

void l2Tick(uint8_t ms) { // ms is milliseconds since last tick 
	for (int port = 0; port < MAX_PORT; port++) {
		l2TmLstRx[port]+= ms;
		if (l2TmLstRx[port] > (myPos * LINE_SILENT)) {
			mst_token[port] = true;
		}
		/*else if (&& lineSlntTmr[port] > (LINE_SILENT / 2)) { // MST PASS

		}*/
		if (mst_token[port] 
			&& l2PktDesc[port].l2Pkt.type == L2_PKT_TYPE_INVALID) { // pas MST token immediatly no packet to send
			l2SendMst(port);
		}
	}
}

#if 0
void l2Send(L3Packet packet, uint8_t addr) {

}
#endif