#include "global.h"
#include "allocator.h"

extern bool mst_token[MAX_PORT];
extern uint8_t maxL2Addr[MAX_PORT];
void l2TmLstRxRst(uint8_t port);

#define L2_PKT_TYPE_INVALID 0xFF
#define L2_PKT_TYPE_ACK 1
#define L2_PKT_TYPE_NAK 2
#define L2_PKT_TYPE_MST 0x80

typedef struct {
	uint8_t addr;
	uint8_t type;

	// payload ptr
	union {
		struct {
			uint8_t  head_page;
			uint8_t  tail_page;
			uint8_t  head_off;       /* 0..UNIT-1 */
			uint8_t  tail_used;
		} pdu;
		uint32_t mst;
	} msgDesc;
	
	uint8_t crc;

} L2Pkt; // size 7 bytes

typedef struct {
	L2Pkt l2Pkt;
	uint8_t time;
	uint8_t retry;
} L2PktDesc; // 9 bytes

//struct L3Packet;
//void l2Send(L3Packet packet, uint8_t addr);

void l2Init();

void l2Tick(uint8_t ms); // ms is milliseconds since last tick