#include "layer2.h"
#include "layer3.h"
#include "network.h"

bool mst_token[MAX_PORT];
uint16_t l2TmLstRx[MAX_PORT];
L2TxPktDesc l2TxPktDesc[MAX_PORT];
L2RxPktDesc l2RxPktDesc[MAX_PORT];
uint8_t maxL2Addr[MAX_PORT];

#define L2_RETRY_THRESHOLD 3

void l2Init() {
	memset(mst_token, 0, sizeof mst_token);
	memset(l2TmLstRx, 0, sizeof l2TmLstRx);
	memset(l2TxPktDesc, 0xFF, sizeof l2TxPktDesc);
	memset(maxL2Addr, 0xFF, sizeof maxL2Addr);
	memset(l2RxPktDesc, 0xFF, sizeof l2RxPktDesc);
}

void l2TxCmplt(uint8_t port) {
	// start timer 

	if (l2TxPktDesc[port].l2TxPkt.hdr.type == L2_PKT_TYPE_ACK ||
		l2TxPktDesc[port].l2TxPkt.hdr.type == L2_PKT_TYPE_NAK) {
		// if its an ACK/NAK response remove msg
		l2TxPktDesc[port].time = 0xFF;
		l2TxPktDesc[port].l2TxPkt.hdr.type = L2_PKT_TYPE_INVALID;
	}
	else {
		l2TxPktDesc[port].time = 0x0;
	}

#if 0
	switch (l2TxPktDesc[port].l2TxPkt.hdr.type) {
	case L2_PKT_TYPE_MST:
		break;
	}
#endif
}

void l2AbortTx(uint8_t port) {
	l2TxPktDesc[port].time = 0xFF; // reprime message for tx
	mst_token[port] = false;
	if (l2TxPktDesc[port].l2TxPkt.hdr.type == L2_PKT_TYPE_MST) {
		l2TxPktDesc[port].l2TxPkt.hdr.type = L2_PKT_TYPE_INVALID;
	}
}

/*void l2Send(L2Packet) {

}*/


bool l2GetTxPkt(uint8_t port, uint8_t ** ptr, uint8_t * len, uint8_t idx) {

	if (idx < sizeof(L2Hdr)) { // give header
		*ptr = ((uint8_t *)&l2TxPktDesc[port].l2TxPkt.hdr) + idx;
		*len = sizeof(L2Hdr) - idx;
	}

	switch (l2TxPktDesc[port].l2TxPkt.hdr.type) {
	case L2_PKT_TYPE_MST:
		// next is CRC
		*len += sizeof(l2TxPktDesc[port].l2TxPkt.crc);
		return true;
	}

	return false;
}

void l2SendNak(uint8_t port, uint8_t rsn) {
	l2TxPktDesc[port].l2TxPkt.hdr.addr = (uint8_t) port_addr[port];  // Best way to find next table in line ? 
	l2TxPktDesc[port].l2TxPkt.hdr.type = L2_PKT_TYPE_NAK;
	l2TxPktDesc[port].l2TxPkt.msg.nak.reason = rsn; // so we can select next MST
	l2TxPktDesc[port].time = 0xFF;

	//l2TxPktDesc->l2TxPkt.crc =
	//l1StartTx(port);
}


uint8_t l2GetRxPkt(uint8_t port, uint8_t** ptr, uint8_t rxLen, uint8_t idx) {
	uint8_t len = 0;
	/* */
	if ((l2TmLstRx[port] > INTER_CHAR_SILENCE) && 
		(l2TmLstRx[port] < INTER_FRAME_SILENCE)) { // abort Rx Todo
		l2RxPktDesc[port].abort = true;
		l2SendNak(port, L2_NAK_RSN_CHR_TMEOUT);
	}

	l2TmLstRx[port] = 0;

	if (l2RxPktDesc[port].abort) {
		return len;
	}

	// validate message early
	if (mst_token[port]) {
		if (l2TxPktDesc[port].l2TxPkt.hdr.type == L2_PKT_TYPE_INVALID) { // if there is no active Tx packet abort RX and give up MST token or RX size is greater than NAK/ACK fail early
			mst_token[port] = false;
			l2RxPktDesc[port].abort = true;
			return len;
		}

		if (idx + rxLen > (sizeof(L2Hdr) + sizeof(l2RxPktDesc[port].l2RxPkt.msg.nak))) {
			l2AbortTx(port);
			l2RxPktDesc[port].abort = true;
			return len;
		}
	}
	else {
		if (idx >= sizeof(l2RxPktDesc[port].l2RxPkt.hdr.addr)) {
			if (l2RxPktDesc[port].l2RxPkt.hdr.addr != ((uint8_t)port_addr[port])) {
				l2RxPktDesc[port].abort = true;
				return len; // abort rx early pkt not for this dev let mst timeout retry since cannot distinguish if its due to a if due to a tx error
			}
		}
	}

	if (idx < sizeof(L2Hdr)) { // give header
		*ptr = ((uint8_t*)&l2RxPktDesc[port].l2RxPkt.hdr) + idx;
		len = sizeof(L2Hdr) - idx;
		return len; // let Hdr finish first
	}

	// at this point confirmed message is for this device
	switch (l2RxPktDesc[port].l2RxPkt.hdr.type) {
	case L2_PKT_TYPE_MST:

		if (idx + rxLen > (sizeof(L2Hdr) + sizeof(l2RxPktDesc[port].l2RxPkt.crc))) {
			l2SendNak(port, L2_NAK_RSN_INV_LEN); // queue NAK
			l2RxPktDesc[port].abort = true;
			return len;
		}

		// next is CRC
		*ptr = ((uint8_t*)&l2RxPktDesc[port].l2RxPkt.msg.mst.mstCrc);
		len = 1;
		return len;
	default:
		l2SendNak(port, L2_NAK_RSN_INV_TYPE);
		l2RxPktDesc[port].abort = true;
		return len;
	}
	
	//return len;
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
	l2TxPktDesc[port].l2TxPkt.hdr.addr = addr;  // Best way to find next table in line ? 
	l2TxPktDesc[port].l2TxPkt.hdr.type = L2_PKT_TYPE_MST;
	l2TxPktDesc[port].l2TxPkt.msg.mst.nextMst = getNxtMst(port, addr); // so we can select next MST
	l2TxPktDesc[port].time = 0xFF;

	//l2TxPktDesc->l2TxPkt.crc =
	l1StartTx(port);
}

void l2TxRetry(uint8_t port) {
	l2TxPktDesc[port].time = 0xFF;
	l2TxPktDesc[port].retry++;
	if (l2TxPktDesc[port].retry > L2_RETRY_THRESHOLD) { // drop
		// should we pass MST token ?
		l2TxPktDesc[port].l2TxPkt.hdr.type = L2_PKT_TYPE_INVALID;
	}
}

void l2CmtRx(port) {
	if (l2RxPktDesc[port].abort) {
		l2RxPktDesc[port].abort = false;
		return;
	}

	uint8_t rxType = l2RxPktDesc[port].l2RxPkt.hdr.type;
	l2RxPktDesc[port].l2RxPkt.hdr.type = L2_PKT_TYPE_INVALID; // invalidate msg for future

	if (rxType == L2_PKT_TYPE_INVALID) {
		return;
	}

	bool crcValid = l2RxPktDesc[port].l2RxPkt.crc != 0xFF;
	// validate CRC TODO
	if (mst_token[port]) {
		if (!crcValid) {
			l2TxRetry(port);
			return;
		}

		// collision conditions
		bool notAckNak =
			(rxType != L2_PKT_TYPE_ACK) && (rxType != L2_PKT_TYPE_NAK);

		bool addrErr = l2RxPktDesc[port].l2RxPkt.hdr.addr != l2TxPktDesc[port].l2TxPkt.hdr.addr;

		if (notAckNak || addrErr) {
			l2AbortTx(port);
			return;
		}
	}
	else {
		l2SendNak(port, L2_NAK_RSN_INV_CRC);
		return;
	}

	switch (rxType) {
	case L2_PKT_TYPE_MST:
		mst_token[port] = true;
		break;
	case L2_PKT_TYPE_ACK: // not for slave
		l2TxPktDesc[port].time = 0xFF; // reset msg timer
		// TODO L3 ACK
		l2TxPktDesc[port].l2TxPkt.hdr.type = L2_PKT_TYPE_INVALID;
		break;
	case L2_PKT_TYPE_NAK: // not applicable for slave
		l2TxRetry(port);
		break;
	default:
		break;
	}
}

void l2Tick(uint8_t ms) { // ms is milliseconds since last tick 
	for (int port = 0; port < MAX_PORT; port++) {
		if (maxL2Addr[port] <= 1) { // only single device in port consider dead
			continue;
		}

		if (mst_token[port] && l2TxPktDesc[port].time != 0xFF) { // prevent rollover
			uint8_t t = l2TxPktDesc[port].time;
			l2TxPktDesc[port].time = (ms > (0xFFu - t)) ? 0xFF : (uint8_t)(t + ms);

			if (l2TxPktDesc[port].l2TxPkt.hdr.type == L2_PKT_TYPE_MST &&
				l2TxPktDesc[port].time > (LINE_SILENT / 2)) { // retry with next mst
				l2SendMst(port, l2TxPktDesc[port].l2TxPkt.msg.mst.nextMst);
			}
			l2TmLstRx[port] = 0; // start Rx timer after Tx has been acked

		} else {
			l2TmLstRx[port] += ms;
		}// increment tx timer

		if (l2TmLstRx[port] > (((uint8_t)port_addr[port]) * LINE_SILENT)) {
			mst_token[port] = true;
		}

		if (l2TmLstRx[port] > INTER_FRAME_SILENCE) {
			l2CmtRx(port);
			if (mst_token[port]) {
				// here request packet from l3

				if (l2TxPktDesc[port].l2TxPkt.hdr.type == L2_PKT_TYPE_INVALID) {// pas MST token immediatly no packet to send
					l2SendMst(port, getNxtMst(port, port_addr[port]));
				}
			}

			if (l2TxPktDesc[port].l2TxPkt.hdr.type != L2_PKT_TYPE_INVALID &&
				l2TxPktDesc[port].time == 0xFF) {
				l1StartTx(port);
			}
		}
	}
}

#if 0
void l2Send(L3Packet packet, uint8_t addr) {

}
#endif