#include "network.h"
#include "layer2.h"

bool mst_token[MAX_PORT];
uint16_t l2TmLstRx[MAX_PORT];
L2TxPktDesc l2TxPktDesc[MAX_PORT];
L2RxPktDesc l2RxPktDesc[MAX_PORT];
uint8_t maxL2Addr[MAX_PORT];
static bool txActive[MAX_PORT];

#define L2_RETRY_THRESHOLD 3

void l2Init() {
	memset(mst_token, 0, sizeof mst_token);
	memset(l2TmLstRx, 0, sizeof l2TmLstRx);
	memset(l2TxPktDesc, 0xFF, sizeof l2TxPktDesc);
	memset(maxL2Addr, 0xFF, sizeof maxL2Addr);
	memset(l2RxPktDesc, 0xFF, sizeof l2RxPktDesc);
	memset(txActive, 0x0, sizeof txActive);
	for (int port = 0; port < MAX_PORT; port++) {
		l2RxPktDesc[port].abort = false;
	}
}

void l2TxCmplt(uint8_t port) {
	txActive[port] = false;
	l3TxCmplt(&l2TxPktDesc[port].l2TxPkt.msg.pdu);
}

void l2AbortTx(uint8_t port) {
	mst_token[port] = false;
	txActive[port] = false;
}

/*void l2Send(L2Packet) {

}*/
#define XFER_DIR 2 // TX RX

bool l2GetTxPkt(uint8_t port, uint8_t ** ptr, uint8_t * len, uint16_t idx, uint8_t txRxFifoLen, uint8_t xfer) {

	static uint8_t tx_pdu_head[XFER_DIR][MAX_PORT];
	static uint8_t tx_pdu_hd_off[XFER_DIR][MAX_PORT];

	if (idx < sizeof(L2Hdr)) { // give header
		*ptr = ((uint8_t *)&l2TxPktDesc[port].l2TxPkt.hdr) + idx;
		*len = sizeof(L2Hdr) - idx;
	}

	switch (l2TxPktDesc[port].l2TxPkt.hdr.type) {
	case L2_PKT_TYPE_MST:
		// next is CRC
		*len += sizeof(l2TxPktDesc[port].l2TxPkt.crc);
		return true;
	case L2_PKT_TYPE_PDU:
		L3Pkt* l3Pkt = &l2TxPktDesc[port].l2TxPkt.msg.pdu;
		if (idx < sizeof(PduHdr)) {
			*len = sizeof(PduHdr) - idx;

			tx_pdu_head[xfer][port] = getL3PktHd(l3Pkt, &tx_pdu_hd_off[xfer][port]);
			//tx_pdu_head[xfer][port] = ps->head_page;
			//tx_pdu_hd_off[xfer][port] = ps->head_off;
			return false;
		}
		else {

			if (tx_pdu_head[xfer][port] == INVALID_PAGE) { // tx complete give crc
				*ptr = &l2TxPktDesc[port].l2TxPkt.crc;
				*len = sizeof(l2TxPktDesc[port].l2TxPkt.crc);
				return true;
			}

			getL3PktFrag(&l2TxPktDesc[port].l2TxPkt.msg.pdu, 
						  ptr, len, 
						 &tx_pdu_head[xfer][port],
						 &tx_pdu_hd_off[xfer][port],
						 txRxFifoLen);
#if 0
			// msg offset
			if (tx_pdu_head[xfer][port] == INVALID_PAGE) {
				*ptr = &l2TxPktDesc[port].l2TxPkt.crc;
				*len = sizeof(l2TxPktDesc[port].l2TxPkt.crc);
				return true;
			}

			const uint16_t base = pageOff(tx_pdu_head[xfer][port]) + (uint16_t)tx_pdu_hd_off[xfer][port];
			// start with first page
			*ptr = &g_pool[base];
			*len = (UNIT - tx_pdu_hd_off[xfer][port]);

			if (tx_pdu_head[xfer][port] == l4pkt->tail_page) {

				if (tx_pdu_hd_off[xfer][port] < l4pkt->tail_used) {
					*len -= (UNIT - l4pkt->tail_used);

					//if (txRxFifoLen >= *len) { // all data will be sent in this single Tx
					//	return true;
					//}
					tx_pdu_hd_off[xfer][port] += txRxFifoLen;
				}
				else { //  this is CRC
					*ptr = &l2TxPktDesc[port].l2TxPkt.crc;
					*len = sizeof(l2TxPktDesc[port].l2TxPkt.crc);
					return true;
				}
				return false;

			} else if (*len <= txRxFifoLen) { // we are at the end of current page
				tx_pdu_head[xfer][port] = g_next[tx_pdu_head[xfer][port]];
				tx_pdu_hd_off[xfer][port] = 0;
				return false;
			}
			else {
				tx_pdu_hd_off[xfer][port] += txRxFifoLen;
			}
#endif
		}
	}

	return false;
}


uint8_t l2GetRxPkt(uint8_t port, uint8_t** ptr, uint8_t rxLen, uint16_t idx) {
	uint8_t len = 0;
	/* */
	if ((l2TmLstRx[port] > INTER_CHAR_SILENCE) && 
		(l2TmLstRx[port] < INTER_FRAME_SILENCE)) { // abort Rx Todo
		l2RxPktDesc[port].abort = true;
	}

	l2TmLstRx[port] = 0;

	if (l2RxPktDesc[port].abort) {
		return len;
	}

	// validate message early
	if (mst_token[port]) { // there should be no active RX when mst
		l2AbortTx(port);
		l2RxPktDesc[port].abort = true;

		// maybe some special message needs response from slave?
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

		if (idx + rxLen > (sizeof(L2Hdr) + sizeof(l2Crc))) {
			l2RxPktDesc[port].abort = true;
			return len;
		}

		// next is CRC
		*ptr = ((uint8_t*)&l2RxPktDesc[port].l2RxPkt.crc);
		len = 1;
		return len;
	default:
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

static inline void l2StartTx(uint8_t port) {
	l2TmLstRx[port] = 0.0;
	txActive[port] = true;
	l1StartTx(port);
}

void l2SendMst(uint8_t port, uint8_t addr) {
	l2TxPktDesc[port].l2TxPkt.hdr.addr = addr;  // Best way to find next table in line ? 
	l2TxPktDesc[port].l2TxPkt.hdr.type = L2_PKT_TYPE_MST;
	l2TxPktDesc[port].l2TxPkt.msg.mst.nextMst = getNxtMst(port, addr); // so we can select next MST
	l2TxPktDesc[port].l2TxPkt.crc = 0xFF; //TODO
	l2StartTx(port);
}


void l2SendPdu(uint8_t port, bool mstPass, uint8_t addr) {
	l2TxPktDesc[port].l2TxPkt.hdr.addr = addr;  // Best way to find next table in line ? 
	l2TxPktDesc[port].l2TxPkt.hdr.type = L2_PKT_TYPE_PDU | (mstPass? L2_PKT_TYPE_MST: 0);
	l2TxPktDesc[port].l2TxPkt.crc = 0xFF; // TODO
	l2StartTx(port);
}

void l2CmtRx(port) {
	l1RxCmplt(port);

	if (l2RxPktDesc[port].abort) {
		l2RxPktDesc[port].abort = false;
		return;
	}

	uint8_t rxType = l2RxPktDesc[port].l2RxPkt.hdr.type;
	l2RxPktDesc[port].l2RxPkt.hdr.type = L2_PKT_TYPE_INVALID; // invalidate msg for future

	if (rxType == L2_PKT_TYPE_INVALID) {
		return;
	}

	bool crcValid = l2RxPktDesc[port].l2RxPkt.crc == 0xFF;
	// validate CRC TODO
	if (!crcValid) { // silently drop
		return;
	}

	switch (rxType) {
	case L2_PKT_TYPE_MST:
		mst_token[port] = true;
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
		
		// increment rx timer
		if (txActive[port]) {
			l2TmLstRx[port] = 0;
		}
		else {
			l2TmLstRx[port] += ms;
		}

		if (l2TmLstRx[port] > (((uint8_t)port_addr[port]) * LINE_SILENT)) {
			mst_token[port] = true;
		}

		if (l2TmLstRx[port] > INTER_FRAME_SILENCE) {
			l2CmtRx(port);
			if (mst_token[port]) {
				if (l2TxPktDesc[port].l2TxPkt.hdr.type == L2_PKT_TYPE_INVALID) {// pas MST token immediatly no packet to send
					// first request pkt from l3
					bool passMST;
					uint8_t l2Addr;
					if (getl3Pkt(port, &l2TxPktDesc[port].l2TxPkt.msg.pdu, &passMST, &l2Addr)) {
						l2SendPdu(port, passMST, l2Addr);
					}
					else {
						l2SendMst(port, getNxtMst(port, port_addr[port]));
					}
				}
				else if (l2TxPktDesc[port].l2TxPkt.hdr.type == L2_PKT_TYPE_MST &&
					l2TmLstRx[port] > (LINE_SILENT / 2)) {
					l2SendMst(port, l2TxPktDesc[port].l2TxPkt.msg.mst.nextMst);
				}
			}
#if 0
			if (l2TxPktDesc[port].l2TxPkt.hdr.type != L2_PKT_TYPE_INVALID &&
				l2TxPktDesc[port].time == 0xFF) {
				l2TmLstRx[port] = 0; // zero rx timer
				l1StartTx(port);
			}
#endif
		}
	}
}

#if 0
void l2Send(L3Packet packet, uint8_t addr) {

}
#endif