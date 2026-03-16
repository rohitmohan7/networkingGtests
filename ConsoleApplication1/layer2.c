#include "network.h"
#include "layer1.h"
#include "layer2.h"
#include "pit.h"

static bool mst_token[MAX_PORT];
L2TxPktDesc l2TxPktDesc[MAX_PORT];
L2RxPktDesc l2RxPktDesc[MAX_PORT];
uint8_t maxL2Addr[MAX_PORT];

#define L2_PIT_TIMER_START_IDX 1
#define BUS_CLK_HZ 48000

#define SILENT_TIMER (((uint8_t)l3AddrTblPrio[myPos][port]) * LINE_SILENT)

#define XFER_DIR 2 // TX RX

/*! Macro to convert a microsecond period to raw count value */
#define USEC_TO_COUNT(us, clockFreqInHz) (uint64_t)(((uint64_t)(us) * (clockFreqInHz)) / 1000000U)

uint8_t getNxtMst(uint8_t port, uint8_t addr) {
	uint8_t max = maxL2Addr[port];     // valid range: 1..max
	uint8_t r = l3AddrTblPrio[myPos][port];

	uint8_t next = (uint8_t)(addr + 1u);
	if (next == 0u || next > max) next = 1u;   // wrap, and also handles uint8_t overflow

	if (next == r) {                            // skip reserved
		next++;
		if (next == 0u || next > max) next = 1u;
	}
	return next;
}

static inline void l2SendMst(uint8_t port, uint8_t addr) {
	l2TxPktDesc[port].l2TxPkt.hdr.addr = addr;  // Best way to find next table in line ? 
	l2TxPktDesc[port].l2TxPkt.hdr.type = L2_PKT_TYPE_MST;
	l2TxPktDesc[port].l2TxPkt.hdr.crc = 0xFF; //TODO
	l1StartTx(port);
}

static inline void l2SendPdu(uint8_t port, bool mstPass, uint8_t addr)
{
	l2TxPktDesc[port].l2TxPkt.hdr.addr = addr; // Best way to find next table in line ?
	l2TxPktDesc[port].l2TxPkt.hdr.type = L2_PKT_TYPE_PDU | (mstPass ? L2_PKT_TYPE_MST : 0);
	l2TxPktDesc[port].l2TxPkt.hdr.crc = 0xFF; // TODO
	l1StartTx(port);
}

void l2SendMsg(uint8_t port) {
	bool passMST;
	uint8_t l2Addr;
	if (getl3Pkt(port, &l2TxPktDesc[port].l2TxPkt.msg.pdu, &passMST, &l2Addr)) {
		l2SendPdu(port, passMST, l2Addr);
	}
	else {
		const uint8_t txAddr = l2TxPktDesc[port].l2TxPkt.hdr.addr > 0 ? l2TxPktDesc[port].l2TxPkt.hdr.addr : l3AddrTblPrio[myPos][port];
		l2SendMst(port, getNxtMst(port, txAddr));
	}
}

void mstTmOut(uint8_t pitChnl) {
	uint8_t port = pitChnl - L2_PIT_TIMER_START_IDX;
	mst_token[port] = true;
	l2SendMsg(port);
}

void l2Init() {
	memset(mst_token, 0, sizeof mst_token);
	memset(l2TxPktDesc, 0x0, sizeof l2TxPktDesc);
	//memset(maxL2Addr, 0xFF, sizeof maxL2Addr);
	memset(l2RxPktDesc, 0x0, sizeof l2RxPktDesc);
	for (int port = 0; port < MAX_PORT; port++) {
		l2RxPktDesc[port].abort = false;
		if (l3AddrTblPrio[myPos][port] > 0) {
			pitEnableTimerSingleShot((L2_PIT_TIMER_START_IDX + port), USEC_TO_COUNT(SILENT_TIMER, BUS_CLK_HZ), &mstTmOut);
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

	bool crcValid = l2RxPktDesc[port].l2RxPkt.hdr.crc == 0xFF;
	// validate CRC TODO
	if (!crcValid) { // silently drop
		return;
	}

	if (rxType & L2_PKT_TYPE_MST) {
		mst_token[port] = true;
	}
	rxType = getPktType(&l2RxPktDesc[port].l2RxPkt);
	l2RxPktDesc[port].l2RxPkt.hdr.type = L2_PKT_TYPE_INVALID; // invalidate msg for future

	switch (rxType) { //TODO
	case L2_PKT_TYPE_PDU:
		l3CmtRx(&l2RxPktDesc[port].l2RxPkt.msg.pdu, port);
		break;
	default:
		break;
	}

	if (!mst_token[port]) {
		// wait for MST pass
		pitEnableTimerSingleShot((L2_PIT_TIMER_START_IDX + port), USEC_TO_COUNT((SILENT_TIMER - INTER_FRAME_SILENCE), BUS_CLK_HZ), &mstTmOut);
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
	pitEnableTimerSingleShot((L2_PIT_TIMER_START_IDX + port), USEC_TO_COUNT((INTER_FRAME_SILENCE - INTER_CHAR_SILENCE), BUS_CLK_HZ), &interFrmeSlnce);
}

void l2TxCmplt(uint8_t port) {
	if (getPktType(&l2TxPktDesc[port].l2TxPkt) == L2_PKT_TYPE_PDU) {
		l3TxCmplt(&l2TxPktDesc[port].l2TxPkt.msg.pdu);
	}
	// prime for next msg
	//l2TxPktDesc[port].l2TxPkt.hdr.type = L2_PKT_TYPE_INVALID;
	if (l2TxPktDesc[port].l2TxPkt.hdr.type & L2_PKT_TYPE_MST) {
		// loose token
		mst_token[port] = false;
		pitEnableTimerSingleShot((L2_PIT_TIMER_START_IDX + port), USEC_TO_COUNT((LINE_SILENT / 2), BUS_CLK_HZ), &mstTmOut);
	}
	else {
		pitEnableTimerSingleShot((L2_PIT_TIMER_START_IDX + port), USEC_TO_COUNT((INTER_FRAME_SILENCE + INTER_FRAME_SILENCE_JITTER), BUS_CLK_HZ), &interFrmeSlnce); // give some threshold to account for slave jitter
	}
}

static inline void l2AbortTx(uint8_t port) {
	mst_token[port] = false;
}

/*void l2Send(L2Packet) {

}*/

bool l2GetTxPkt(uint8_t port, uint8_t ** ptr, uint8_t * len, uint16_t idx, uint8_t txRxFifoLen, const L2XferDir_t xfer) {

	static uint8_t tx_pdu_head[L2_XFER_SIZE][MAX_PORT]; // TBD can it be moved to L4?
	static uint8_t tx_pdu_hd_off[L2_XFER_SIZE][MAX_PORT];

	uint8_t type = getPktType(&l2TxPktDesc[port].l2TxPkt);

	if (idx < sizeof(L2Hdr)) { // give header
		*ptr = ((uint8_t*)&l2TxPktDesc[port].l2TxPkt.hdr) + idx;
		*len = sizeof(L2Hdr) - idx;
	}

	switch (type) {
	case L2_PKT_TYPE_MST:
		// pkt has only hdr
		return true;
	case L2_PKT_TYPE_PDU:
		L3Pkt* l3Pkt = &l2TxPktDesc[port].l2TxPkt.msg.pdu;
		if (idx < sizeof(PduHdr)) {
			
			if (!ptr) { // if ptr was not already set
				*ptr = ((uint8_t *)&l2TxPktDesc[port].l2TxPkt.hdr) + idx;
			}
			
			*len = sizeof(PduHdr) - idx; // reset length

			return getL3PktHd(l3Pkt, &tx_pdu_head[xfer][port], &tx_pdu_hd_off[xfer][port]);
		}
		else {

			*len = getL3PktFrag(&l2TxPktDesc[port].l2TxPkt.msg.pdu,
								ptr, (idx - sizeof(PduHdr)),
								&tx_pdu_head[xfer][port],
								&tx_pdu_hd_off[xfer][port],
								txRxFifoLen);

			if (tx_pdu_head[xfer][port] == INVALID_PAGE)
			{
				return true;
			}
		}
	}

	return false;
}

bool l2RxAborted(uint8_t port) {
	if (l2RxPktDesc[port].abort) { // if aborted wait for next message
		pitEnableTimerSingleShot((L2_PIT_TIMER_START_IDX + port), USEC_TO_COUNT(INTER_FRAME_SILENCE, BUS_CLK_HZ), &interFrmeSlnce);
	} else {
		pitEnableTimerSingleShot((L2_PIT_TIMER_START_IDX + port), USEC_TO_COUNT(INTER_CHAR_SILENCE, BUS_CLK_HZ), &interChrSlnce);
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
		if (l2RxPktDesc[port].l2RxPkt.hdr.addr != ((uint8_t)l3AddrTblPrio[myPos][port])) {
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
	case L2_PKT_TYPE_PDU:

		if (idx < sizeof(PduHdr)) {
			*ptr = ((uint8_t *)&l2RxPktDesc[port].l2RxPkt.hdr) + idx;
			len = sizeof(PduHdr) - idx;
		} else { // todo
			if (idx == sizeof(PduHdr)) { // we just got hdr identify and prime rx
				if (!l3CmtRxHd(&l2RxPktDesc[port].l2RxPkt.msg.pdu, port)) {
					return 0;
				}
			}

			len = getL3RxPktFrag(&l2RxPktDesc[port].l2RxPkt.msg.pdu, ptr, idx, rxLen);
		}

		return len;
	case L2_PKT_TYPE_MST:// this packet should only have header
	default:
		return 0;
	}
	
	//return len;
}
