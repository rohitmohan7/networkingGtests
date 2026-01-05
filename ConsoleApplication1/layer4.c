//#include "layer4.h"
#include "layer3.h"
#include "allocator.h"
#include "layer2.h"

#define L2_FRAME_SIZE (RS485_FRAME_SIZE - (sizeof(L2Hdr) + sizeof(((L2Pkt*)0)->crc)))
#define L3_FRAME_SIZE (L2_FRAME_SIZE - sizeof(L3Hdr))
#define L4_FRAME_SIZE (L3_FRAME_SIZE - sizeof(L4Hdr))

static stream_t streams[MAX_POS];

void initL4() {
	for (int prio = 0; prio < MAX_PRIORITY; prio++) {
		for (int pos = 0; pos < MAX_POS; pos++) {
			stream_t* s = &streams[pos];
			prio_stream_t* ps = &s->prio[prio];
			ps->msgLen = 0;
		}
	}
}

void l4Ack(uint8_t prio, uint16_t dstAddr) {
	for (int pos = 0; pos < MAX_POS; pos++) {
		stream_t* s = &streams[pos];
		if (dstAddr == s->dst) {
			

		}
	}
}

bool getL4Pkt(L4Pkt* l4Pkt, uint8_t portSubnet, uint8_t prio, uint16_t* dstAddr) {
	for (int pos = 0; pos < MAX_POS; pos++) {
		stream_t* s = &streams[pos];
		uint8_t gatewaySubnet = (s->gateway & 0xFF00) >> 8;

		if (gatewaySubnet != portSubnet) {
			continue;
		}

		prio_stream_t* ps = &s->prio[prio];
		if (ps->head_page == INVALID_PAGE) { // nothing to send
			continue;
		}
		
		*dstAddr = s->dst;

		// set hdr
		L4Hdr* hdr = &l4Pkt->hdr;
		hdr->msgNo = ps->msgNo;
		hdr->len = (ps->msgLen > L4_FRAME_SIZE)? (ps->msgLen - L4_FRAME_SIZE): 0; // remaining msg len

		// feed packet
		l4Pkt->head_page = ps->head_page;
		l4Pkt->head_off = ps->head_off;

		uint8_t currHd = l4Pkt->head_page;
		uint8_t len = min(L4_FRAME_SIZE, ps->msgLen); // write until min msg len

		while (len > 0) {
			if (/*currHd == ps->tail_page ||*/
				len < (UNIT)) { // reached end
				l4Pkt->tail_page = currHd;
				l4Pkt->tail_used = /*currHd == ps->tail_page ? min(len, ps->tail_used) :*/ len;
				break;
			}

			currHd = g_next[currHd];
			len -= UNIT;
		}

#ifndef TRANSPORT_ACK // no need to keep in transport layer if L2 fails to TX it will abort MSG
		if (l4Pkt->tail_used == UNIT) { // full tail used 
			ps->head_page = g_next[l4Pkt->tail_page];
			ps->head_off = 0;
		} else {
			ps->head_page = l4Pkt->tail_page;
			ps->head_off = l4Pkt->tail_used + 1;
		}

		if (hdr->len == 0) {
			ps->msgNo++; // increment seq number

			ps->msgLen = g_pool[(ps->head_page * UNIT) + (ps->head_off)];
			ps->head_off++;
			if (ps->head_off == UNIT) {
				ps->head_page = g_next[ps->head_page];
				ps->head_off = 0;
			}

			ps->msgLen |= (g_pool[(ps->head_page * UNIT) + (ps->head_off)]) << 8;
			ps->head_off++;
		}
#endif
	}
	return false;
}