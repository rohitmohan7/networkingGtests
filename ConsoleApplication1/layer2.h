#include "global.h"
#include "allocator.h"

extern bool mst_token[MAX_PORT];
extern uint8_t maxL2Addr[MAX_PORT];

#if 0
static inline L2TxPktDesc* L2_GetPktDesc(uint8_t port)
{
	return &l2PktDesc[port];
}
#endif

#define L2_PKT_TYPE_INVALID 0xFF
#define L2_PKT_TYPE_ACK 1
#define L2_PKT_TYPE_NAK 2
#define L2_PKT_TYPE_MST 0x80

#define LINE_SILENT 100

typedef struct __attribute__((packed)) {
	uint8_t addr;
	uint8_t type;
} L2Hdr;
_Static_assert(sizeof(L2Hdr) == 2, "L2Hdr wrong size");

typedef struct __attribute__((packed)) {
	L2Hdr hdr;

	// payload ptr
	union {
		struct {
			uint8_t  head_page;
			uint8_t  tail_page;
			uint8_t  head_off;       /* 0..UNIT-1 */
			uint8_t  tail_used;
		} pdu;

		struct {
			uint8_t mstCrc;
			uint8_t nextMst;
		} mst;
	} msg;
	
	uint8_t crc;

} L2Pkt; // size 7 bytes
_Static_assert(sizeof(L2Pkt) == 7, "L2Pkt wrong size");

typedef struct __attribute__((packed)) {
	L2Pkt l2Pkt;
	uint8_t time;
	uint8_t retry;
} L2TxPktDesc; // 9 bytes
_Static_assert(sizeof(L2TxPktDesc) == 9, "L2Pkt wrong size");

extern L2TxPktDesc l2PktDesc[MAX_PORT];

//struct L3Packet;
//void l2Send(L3Packet packet, uint8_t addr);

void l2Init();

void l2Tick(uint8_t ms); // ms is milliseconds since last tick

bool l2GetTxPkt(uint8_t port, uint8_t** ptr, uint8_t* len, uint8_t idx);

uint8_t l2GetRxPkt(uint8_t port, uint8_t* ptr, uint8_t len, uint8_t idx);

void l2TxCmplt(uint8_t port);

void l2AbortTx(uint8_t port);