#include "layer2.h"
#include "layer3.h"
#include "network.h"

bool mst_token[MAX_PORT];
uint16_t l2TmLstRx[MAX_PORT];
L2TxPktDesc l2PktDesc[MAX_PORT];
uint8_t maxL2Addr[MAX_PORT];

void l2Init() {
	memset(mst_token, 0, sizeof mst_token);
	memset(l2TmLstRx, 0, sizeof l2TmLstRx);
	memset(l2PktDesc, 0xFF, sizeof l2PktDesc);
	memset(maxL2Addr, 0xFF, sizeof maxL2Addr);
}

void l2TxCmplt(uint8_t port) {
	// start timer 

	if (l2PktDesc[port].l2Pkt.hdr.type == L2_PKT_TYPE_ACK ||
		l2PktDesc[port].l2Pkt.hdr.type == L2_PKT_TYPE_NAK) {
		// if its an ACK/NAK response remove msg
		l2PktDesc[port].time = 0xFF;
		l2PktDesc[port].l2Pkt.hdr.type = L2_PKT_TYPE_INVALID;
	}
	else {
		l2PktDesc[port].time = 0x0;
	}

#if 0
	switch (l2PktDesc[port].l2Pkt.hdr.type) {
	case L2_PKT_TYPE_MST:
		break;
	}
#endif
}

void l2AbortTx(uint8_t port) {
	l2PktDesc[port].time = 0xFF; // reprime message for tx
	mst_token[port] = false;
	if (l2PktDesc[port].l2Pkt.hdr.type == L2_PKT_TYPE_MST) {
		l2PktDesc[port].l2Pkt.hdr.type = L2_PKT_TYPE_INVALID;
	}
}

/*void l2Send(L2Packet) {

}*/

bool l2GetTxPkt(uint8_t port, uint8_t ** ptr, uint8_t * len, uint8_t idx) {

	if (idx < sizeof(L2Hdr)) { // give header
		*ptr = (uint8_t *)&l2PktDesc[port].l2Pkt.hdr;
		*len = sizeof(L2Hdr);
	}

	switch (l2PktDesc[port].l2Pkt.hdr.type) {
	case L2_PKT_TYPE_MST:
		// next is CRC
		*len += sizeof(l2PktDesc[port].l2Pkt.crc);
		return true;
	}

	return false;
}

uint8_t l2GetRxPkt(uint8_t port, uint8_t* ptr, uint8_t len, uint8_t idx) {
	/* */
	if (l2TmLstRx[port] > 1.5 && l2TmLstRx[port] < 3.5) { // abort Rx

	}


	l2TmLstRx[port] = 0;
}

uint8_t getNxtMst(uint8_t port, uint8_t addr) {
	uint8_t max = maxL2Addr[port];     // valid range: 1..max
	uint8_t r = port_addr[port];

	uint8_t next = (uint8_t)(addr + 1u);
	if (next == 0u || next > max) next = 1u;   // wrap, and also handles uint8_t overflow

	if (next == r) {                            // skip reserved
		next++;
		if (next == 0u || next > max) next = 1u;
	}
	return next;
}

void l2SendMst(uint8_t port, uint8_t addr) {
	l2PktDesc[port].l2Pkt.hdr.addr = addr;  // Best way to find next table in line ? 
	l2PktDesc[port].l2Pkt.hdr.type = L2_PKT_TYPE_MST;
	l2PktDesc[port].l2Pkt.msg.mst.nextMst = getNxtMst(port, addr); // so we can select next MST
	l2PktDesc[port].time = 0xFF;

	//l2PktDesc->l2Pkt.crc =
	l1StartTx(port);
}

void l2Tick(uint8_t ms) { // ms is milliseconds since last tick 
	for (int port = 0; port < MAX_PORT; port++) {
		if (maxL2Addr[port] <= 1) { // only single device in port consider dead
			continue;
		}

		if (l2PktDesc[port].time != 0xFF) { // prevent rollover
			uint8_t t = l2PktDesc[port].time;
			l2PktDesc[port].time = (ms > (0xFFu - t)) ? 0xFF : (uint8_t)(t + ms);

			if (l2PktDesc[port].l2Pkt.hdr.type == L2_PKT_TYPE_MST &&
				l2PktDesc[port].time > (LINE_SILENT / 2)) { // retry with next mst
				l2SendMst(port, l2PktDesc[port].l2Pkt.msg.mst.nextMst);
			}
			l2TmLstRx[port] = 0; // start Rx timer after Tx has been acked

		} else {
			l2TmLstRx[port] += ms;
		}// increment tx timer

		if (l2TmLstRx[port] > (((uint8_t)port_addr[port]) * LINE_SILENT)) {
			mst_token[port] = true;
		}

		if (mst_token[port]) {
			if (l2PktDesc[port].l2Pkt.hdr.type == L2_PKT_TYPE_INVALID) {// pas MST token immediatly no packet to send
				l2SendMst(port, getNxtMst(port, port_addr[port]));
			}
			else {
			
			}// message in tx buff start tx 
		}
	}
}

#if 0
void l2Send(L3Packet packet, uint8_t addr) {

}
#endif