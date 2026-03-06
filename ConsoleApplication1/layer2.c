#include "network.h"
#include "layer2.h"
#include "pit.h"

static bool mst_token[MAX_PORT];
L2TxPktDesc l2TxPktDesc[MAX_PORT];
L2RxPktDesc l2RxPktDesc[MAX_PORT];
uint8_t maxL2Addr[MAX_PORT];

#define L2_PIT_TIMER_START_IDX 1
#define BUS_CLK_HZ 48000

#define SILENT_TIMER (((uint8_t)port_addr[port]) * LINE_SILENT)

#define XFER_DIR 2 // TX RX

/*! Macro to convert a microsecond period to raw count value */
#define USEC_TO_COUNT(us, clockFreqInHz) (uint64_t)(((uint64_t)(us) * (clockFreqInHz)) / 1000000U)

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
	l2TxPktDesc[port].l2TxPkt.msg.mst.mstCrc = 0xFF; //TODO
	l1StartTx(port);
}

void l2SendMsg(uint8_t port) {
	bool passMST;
	uint8_t l2Addr;
	if (getl3Pkt(port, &l2TxPktDesc[port].l2TxPkt.msg.pdu, &passMST, &l2Addr)) {
		l2SendPdu(port, passMST, l2Addr);
	}
	else {
		const uint8_t txAddr = l2TxPktDesc[port].l2TxPkt.hdr.addr > 0 ? l2TxPktDesc[port].l2TxPkt.hdr.addr : port_addr[port];
		l2SendMst(port, getNxtMst(port, txAddr));
	}
}

void mstTmOut(uint8_t pitChnl) {
	uint8_t port = pitChnl - L2_PIT_TIMER_START_IDX;
	mst_token[port] = true;
	l2SendMsg(port);
}

void mstPassTmOut(uint8_t pitChnl) {
	uint8_t port = pitChnl - L2_PIT_TIMER_START_IDX;
	l2SendMsg(port);
}

void l2Init() {
	memset(mst_token, 0, sizeof mst_token);
	memset(l2TxPktDesc, 0x0, sizeof l2TxPktDesc);
	//memset(maxL2Addr, 0xFF, sizeof maxL2Addr);
	memset(l2RxPktDesc, 0x0, sizeof l2RxPktDesc);
	for (int port = 0; port < MAX_PORT; port++) {
		l2RxPktDesc[port].abort = false;
		if (port_addr[port] > 0) {
			PITEnableTimerSingleShot((L2_PIT_TIMER_START_IDX + port), USEC_TO_COUNT(SILENT_TIMER, BUS_CLK_HZ), &mstTmOut);
		}
	}
}

static inline uint8_t getPktType(L2Pkt* pkt) {
	uint8_t type = pkt->hdr.type;

	if (type & ~L2_PKT_TYPE_MST) {
		type &= ~L2_PKT_TYPE_MST;
	}
	return type;
}

void l2CmtRx(port) {
	l1RxCmplt(port);

	l2RxPktDesc[port].abort = false;

	uint8_t rxType = l2RxPktDesc[port].l2RxPkt.hdr.type;

	if (rxType == L2_PKT_TYPE_INVALID) {
		return;
	}

	l2RxPktDesc[port].l2RxPkt.hdr.type = L2_PKT_TYPE_INVALID; // invalidate msg for future

	bool crcValid = l2RxPktDesc[port].l2RxPkt.crc == 0xFF;
	// validate CRC TODO
	if (!crcValid) { // silently drop
		return;
	}

	if (rxType & L2_PKT_TYPE_MST) {
		mst_token[port] = true;
	}
	rxType = getPktType(&l2RxPktDesc[port].l2RxPkt);

	switch (rxType) { //TODO
	default:
		break;
	}

	if (!mst_token[port]) {
		// wait for MST pass
		PITEnableTimerSingleShot((L2_PIT_TIMER_START_IDX + port), USEC_TO_COUNT((SILENT_TIMER - INTER_FRAME_SILENCE), BUS_CLK_HZ), &mstTmOut);
	}
	else {
		l2SendMsg(port);
	}
}

void interFrmeSlnce(uint8_t pitChnl) {
	uint8_t port = pitChnl - L2_PIT_TIMER_START_IDX;
	if (!mst_token[port]) { // mst would have failed early on RX ... 
		l2CmtRx(port);
	}
	else {
		l2SendMsg(port); // send next mst message
	}
}

void interChrSlnce(uint8_t pitChnl) {
	uint8_t port = pitChnl - L2_PIT_TIMER_START_IDX;
	l2RxPktDesc[port].abort = true; // abort future RX
	PITEnableTimerSingleShot((L2_PIT_TIMER_START_IDX + port), USEC_TO_COUNT((INTER_FRAME_SILENCE - INTER_CHAR_SILENCE), BUS_CLK_HZ), &interFrmeSlnce);
}

void l2TxCmplt(uint8_t port) {
	if (getPktType(&l2TxPktDesc[port].l2TxPkt) == L2_PKT_TYPE_PDU) {
		l3TxCmplt(&l2TxPktDesc[port].l2TxPkt.msg.pdu);
	}
	// prime for next msg
	//l2TxPktDesc[port].l2TxPkt.hdr.type = L2_PKT_TYPE_INVALID;
	if (l2TxPktDesc[port].l2TxPkt.hdr.type & L2_PKT_TYPE_MST) {
		PITEnableTimerSingleShot((L2_PIT_TIMER_START_IDX + port), USEC_TO_COUNT((LINE_SILENT / 2), BUS_CLK_HZ), &mstPassTmOut);
	}
	else {
		PITEnableTimerSingleShot((L2_PIT_TIMER_START_IDX + port), USEC_TO_COUNT((INTER_FRAME_SILENCE + INTER_FRAME_SILENCE_JITTER), BUS_CLK_HZ), &interFrmeSlnce); // give some threshold to account for slave jitter
	}
}

static inline void l2AbortTx(uint8_t port) {
	mst_token[port] = false;
}

/*void l2Send(L2Packet) {

}*/

bool l2GetTxPkt(uint8_t port, uint8_t ** ptr, uint8_t * len, uint16_t idx, uint8_t txRxFifoLen, uint8_t xfer) {

	static uint8_t tx_pdu_head[MAX_PORT];
	static uint8_t tx_pdu_hd_off[MAX_PORT];
	static uint8_t* ptrEcho[MAX_PORT];
	static uint8_t lenEcho[MAX_PORT];
	uint8_t type = getPktType(&l2TxPktDesc[port].l2TxPkt);

	if (xfer == XFER_RX_ECHO) {
		*ptr = ptrEcho[port];
		*len = lenEcho[port];

		if ((type != L2_PKT_TYPE_PDU) || ptrEcho == &l2TxPktDesc[port].l2TxPkt.crc) { // if this is CRC we are at the end
			return true;
		}
		return false; // how to determine complete ?
	}

	if (idx < sizeof(L2Hdr)) { // give header
		*ptr = ptrEcho[port] = ((uint8_t*)&l2TxPktDesc[port].l2TxPkt.hdr) + idx;
		*len = lenEcho[port] = sizeof(L2Hdr) - idx;
	}

	switch (type) {
	case L2_PKT_TYPE_MST:
		// next is CRC
		lenEcho[port] = *len += sizeof(l2Crc);
		return true;
	case L2_PKT_TYPE_PDU:
		L3Pkt* l3Pkt = &l2TxPktDesc[port].l2TxPkt.msg.pdu;
		if (idx < sizeof(PduHdr)) {
			*len = lenEcho[port] = sizeof(PduHdr) - idx;

			tx_pdu_head[port] = getL3PktHd(l3Pkt, &tx_pdu_hd_off[port]);
			//tx_pdu_head[xfer][port] = ps->head_page;
			//tx_pdu_hd_off[xfer][port] = ps->head_off;
			return false;
		}
		else {

			if (tx_pdu_head[port] == INVALID_PAGE) { // tx complete give crc
				*ptr = ptrEcho[port] = &l2TxPktDesc[port].l2TxPkt.crc;
				*len = lenEcho[port] = sizeof(l2Crc);
				return true;
			}

			getL3PktFrag(&l2TxPktDesc[port].l2TxPkt.msg.pdu, 
						  ptr, len, 
						 &tx_pdu_head[port],
						 &tx_pdu_hd_off[port],
						 txRxFifoLen);

			// prime it for echo
			ptrEcho[port] = *ptr;
			lenEcho[port] = *len;
		}
	}

	return false;
}

bool l2RxAborted(uint8_t port) {
	if (l2RxPktDesc[port].abort) { // if aborted wait for next message
		PITEnableTimerSingleShot((L2_PIT_TIMER_START_IDX + port), USEC_TO_COUNT(INTER_FRAME_SILENCE, BUS_CLK_HZ), &interFrmeSlnce);
	} else {
		PITEnableTimerSingleShot((L2_PIT_TIMER_START_IDX + port), USEC_TO_COUNT(INTER_CHAR_SILENCE, BUS_CLK_HZ), &interChrSlnce);
	}
	return l2RxPktDesc[port].abort;
}

static inline void l2AbortRx(uint8_t port) {
	l2RxPktDesc[port].abort = true;
	l2TxPktDesc[port].l2TxPkt.hdr.type = L2_PKT_TYPE_INVALID;
	l2TxPktDesc[port].l2TxPkt.hdr.addr = 0x00;
}

void l2AbortXfer(uint8_t port) {
	l2AbortTx(port);
	l2AbortRx(port);
}

uint8_t l2GetRxPkt(uint8_t port, uint8_t** ptr, uint8_t rxLen, uint16_t idx) {
	uint8_t len = 0;

	if (mst_token[port]) {
		return len;
	}

	// validate message early
	if (idx >= sizeof(l2RxPktDesc[port].l2RxPkt.hdr.addr)) {
		if (l2RxPktDesc[port].l2RxPkt.hdr.addr != ((uint8_t)port_addr[port])) {
			return len; // abort rx early pkt not for this dev let mst timeout retry since cannot distinguish if its due to a if due to a tx error
		}
	}
	
	if (idx < sizeof(L2Hdr)) { // give header
		*ptr = ((uint8_t*)&l2RxPktDesc[port].l2RxPkt.hdr) + idx;
		len = sizeof(L2Hdr) - idx;
		return len; // let Hdr finish first
	}

	uint8_t type = getPktType(&l2RxPktDesc[port].l2RxPkt);

	// at this point confirmed message is for this device
	switch (type) {
	case L2_PKT_TYPE_MST:

		if (idx + rxLen > (sizeof(L2Hdr) + sizeof(l2Crc))) {
			return len;
		}

		// next is CRC
		*ptr = ((uint8_t*)&l2RxPktDesc[port].l2RxPkt.crc);
		len = sizeof(l2Crc);
		return len;
	default:
		return len;
	}
	
	//return len;
}

void l2SendPdu(uint8_t port, bool mstPass, uint8_t addr) {
	l2TxPktDesc[port].l2TxPkt.hdr.addr = addr;  // Best way to find next table in line ? 
	l2TxPktDesc[port].l2TxPkt.hdr.type = L2_PKT_TYPE_PDU | (mstPass? L2_PKT_TYPE_MST: 0);
	l2TxPktDesc[port].l2TxPkt.crc = 0xFF; // TODO
	l1StartTx(port);
}

#if 0
void l2Tick() { // ms is milliseconds since last tick 
	for (int port = 0; port < MAX_PORT; port++) {
		if (maxL2Addr[port] <= 1) { // only single device in port consider dead
			continue;
		}

#if 0
		// increment rx timer
		if (txActive[port]) {
			l2TmLstRx[port] = 0;
		}
		else {
			l2TmLstRx[port] += ms;
		}
#endif

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
#endif
#if 0
void l2Send(L3Packet packet, uint8_t addr) {

}
#endif
