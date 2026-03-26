#include "layer2.h"
#include "layer3.h"
//#include "layer4.h"
#include "network.h"

#define L3_DST_ADDR_IDX 0
#define L3_ADDR_SUBNET_SHIFT 8

#define L3_FRAME_SIZE (L2_FRAME_SIZE - sizeof(L3Hdr))
#define L4_FRAME_SIZE (L3_FRAME_SIZE - sizeof(L4Hdr))

#define IP_REASS_BLOCK_SIZE       8U
#define IP_REASS_MAX_LEN          2048U
#define IP_REASS_MAX_BLOCKS       ((IP_REASS_MAX_LEN + IP_REASS_BLOCK_SIZE - 1U) / IP_REASS_BLOCK_SIZE)
#define IP_REASS_BITMAP_BYTES     ((IP_REASS_MAX_BLOCKS + 7U) / 8U)

#define IP_MORE_FRAG_MASK 0x2000U
#define IP_IHL_WORDS_MASK 0x0FU
#define IP_FRAG_OFFSET_MASK 0x1FFFU

#define IPV4_VERSION 4
#define IPV4_VERSION_SHIFT 4
#define IPV4_VERSION_MASK 0xF0
#define IPV4_WORD_MASK 0x0FU

/*defines the largest IP packet size (in bytes) that an interface can transmit without needing fragmentation, typically defaulting to 1500 bytes IEEE networks*/
#define IP_MTU(port) L2_FRAME_SIZE
#define L3_FRAG_SIZE(port)   ((IP_MTU(port) - sizeof(L3Hdr)) & ~((uint16_t)0x7U)) /* 8 byte aligned RFC 791 */

IpAddrType_t l3AddrTblPrio[MAX_POS][MAX_PORT]; // pos addresses ordered by tx priority
IpAddrType_t l3RouteTable[MAX_SUBNET]; // next best gateway for subnet
uint8_t l3BcastInSubnetForSrcPort[MAX_POS][MAX_PORT];
uint8_t l3RouteHops[MAX_SUBNET]; // l3 hop table for my pos
uint8_t l3MaxHops = 0;

static PgPtrHd_t l3TxRxPgPtrHd[MAX_PORT][L2_XFER_SIZE];

#define IP_REASS_TIMEOUT_TICKS 10*1000 // 10 seconds

typedef struct __attribute__((packed)) IpReassKey_st
{
	uint16_t src;
	uint16_t dst;
	uint16_t id;
	uint8_t proto;
} IpReassKey_t;

typedef struct __attribute__((packed)) IpTxKey_st
{
	uint16_t dst;
	uint16_t id;
	uint8_t proto;
} IpTxKey_t;

typedef struct __attribute__((packed)) IpReassQueue_st {
	IpReassKey_t key;
	uint32_t expireTick;
	PgPtr_t pgPtr;
	uint16_t maxLen;
	uint16_t rxLen;
	uint8_t  blockMap[IP_REASS_BITMAP_BYTES];
} IpReassQueue_t;

typedef struct __attribute__((packed)) IpTxQueue_st {
	IpTxKey_t key;
	//uint32_t expireTick; TODO expiry?
	PgPtr_t pgPtr;
	uint8_t fragCnt; // TODO check if need bigger size to prvent rollover
} IpTxQueue_t;

#define IP_MAX_TX_QUEUES 4
IpTxQueue_t g_ipTxQ[MAX_PORT][MAX_PRIORITY][IP_MAX_TX_QUEUES];
static uint8_t l3ipTxQHead[MAX_PORT][MAX_PRIORITY];
static uint8_t l3ipTxQTail[MAX_PORT][MAX_PRIORITY];
static uint8_t l3ipTxQCount[MAX_PORT][MAX_PRIORITY];
static uint16_t l3FragId = 0; // for tx, TODO do we keep distinct one for each port?

#define IP_MAX_REASS_QUEUES 10
/*src pos*/ /*dst pos*/ /* proto */
IpReassQueue_t g_ipReass[IP_MAX_REASS_QUEUES];

#define LOW_PRIO_IDX 1
#define BROADCAST_ADDR 0x0000

static inline bool l3TxBrdcstPkt(const L3Hdr *const l3Hdr)
{
	return l3Hdr->dst == BROADCAST_ADDR;
}
static inline void l3InitIpTxQ(IpTxQueue_t* const q) {
	memset(&q->key, 0, sizeof(q->key));
	pgPtrInit(&q->pgPtr);
	q->fragCnt = 0;
}

static inline IpTxQueue_t* l3PushIpTxQ(const uint8_t port, const uint8_t prio) {

	if (l3ipTxQCount[port][prio] >= IP_MAX_TX_QUEUES) {
		return NULL;
	}

	const uint8_t ipTxQTl = l3ipTxQTail[port][prio];
	IpTxQueue_t* l3IpTxQ = &g_ipTxQ[port][prio][ipTxQTl];
	l3InitIpTxQ(l3IpTxQ);

	l3ipTxQTail[port][prio] = (uint8_t)((l3ipTxQTail[port][prio] + 1u) % IP_MAX_TX_QUEUES);
	l3ipTxQCount[port][prio]++;
	return l3IpTxQ;
}

void l3InitTxPkt(L3Pkt* l3Pkt) {
	l3Pkt->hdr.verIhl = (uint8_t)((IPV4_VERSION << IPV4_VERSION_SHIFT) | (sizeof(L3Hdr) >> 2U));
}

bool l3SendUdp(const UdpHdr_t * const udpHdr, uint8_t * data, uint16_t len, const uint8_t prio) {
	if (udpHdr->dstPort >= MAX_POS) {
		return false;
	}

	bool ret = false;
	const bool brdcst = udpHdr->dstPort == 0;

	uint16_t dstAddr;
	uint8_t dstSubnet;
	uint8_t gatewaySubnet;

	PgPtr_t* pgPtr = NULL;

	if (!brdcst) {
		dstAddr = udpHdr->dstPort ? l3AddrTblPrio[udpHdr->dstPort][L3_DST_ADDR_IDX] : 0;
		dstSubnet = (dstAddr & 0xFF00) >> 8;
		gatewaySubnet = (l3RouteTable[dstSubnet] & 0xFF00) >> 8;
	} // else broadcast 

	for (uint8_t port = 0; port < MAX_PORT; port++) {
		const IpAddrType_t portAddr = l3AddrTblPrio[myPos][port];

		if (!portAddr) {
			continue;
		}

		bool txPort = brdcst;
		if (!brdcst) {
			const uint8_t portSubnet = ((l3AddrTblPrio[myPos][port] & 0xFF00) >> 8);
			txPort = (dstSubnet == portSubnet) || (gatewaySubnet == portSubnet);
		}

		if (txPort) {
			IpTxQueue_t* ipTxQ = l3PushIpTxQ(port, prio);

			if (ipTxQ) {

				if (!pgPtr) {
					writeValToPage(&ipTxQ->pgPtr, (uint8_t*)udpHdr, sizeof(UdpHdr_t));
					writeValToPage(&ipTxQ->pgPtr, data, len);
					pgPtr = &ipTxQ->pgPtr;
				} else {
					/* copy existing pg ptr */
					ipTxQ->pgPtr = *pgPtr;
					addUser(pgPtr);
				}

				/* set up the key */
				ipTxQ->key.dst = dstAddr;
				ipTxQ->key.proto = IP_PROTO_UDP;
				ipTxQ->key.id = ++l3FragId;
				ret = true;
				if (!brdcst) {
					return true;
				}
			}
		}
	}
	return ret; /* TODO: for a brodcast treat as successfull if we where able to send to atleas 1 port?
	, In actual networking a default route is used ...*/
}

static inline bool l3Multicst(const L3Hdr *const l3Hdr, const uint8_t port)
{
	const uint8_t portSubnet = l3AddrTblPrio[myPos][port] >> 8;
	const uint8_t dstSubnet = l3Hdr->dst >> 8;
	return (((uint8_t)l3Hdr->dst) == 0x00) && (portSubnet == dstSubnet);
}

static inline uint8_t getL3PktFragInternal(PgPtr_t* frwdPgPtr, uint8_t** ptr, uint8_t len, uint8_t* txHd, uint8_t* txHdOfst, uint8_t txLen) {
	const uint16_t base = pageOff(*txHd) + (uint16_t)(*txHdOfst);

	*ptr = &g_pool[base];

	if (len <= txLen)
	{ // we are at the end of current page
		*txHd = g_next[(*txHd)];
		*txHdOfst = 0;
	}
	else
	{
		len = min(txLen, len);
		*txHdOfst += len;
	}

	if ((*txHd == frwdPgPtr->tlPg &&
		frwdPgPtr->tlUsd == *txHdOfst)) {
		*txHd = INVALID_PAGE;
	}

	return len;
}

static inline void setL3TxRxHd(const PgPtr_t* const pgPtr, const uint8_t xferDir, const uint8_t port) {
	l3TxRxPgPtrHd[port][xferDir].hd = pgPtr->hdPg;
	l3TxRxPgPtrHd[port][xferDir].hdOfst = pgPtr->hdOfst;
}

static inline uint8_t getL3PktFragFrmPgPtr(PgPtr_t* pgPtr, uint8_t** ptr, uint8_t idx, uint8_t txLen, const L2XferDir_t xferDir, const uint8_t port)
{
	if (l3TxRxPgPtrHd[port][xferDir].hd == INVALID_PAGE ||
		(l3TxRxPgPtrHd[port][xferDir].hd == pgPtr->tlPg &&
			pgPtr->tlUsd == l3TxRxPgPtrHd[port][xferDir].hdOfst)) {
		return 0;
	}

	const uint16_t base = pageOff(l3TxRxPgPtrHd[port][xferDir].hd) + (uint16_t)(l3TxRxPgPtrHd[port][xferDir].hdOfst);

	*ptr = &g_pool[base];

	// cap tx len
	uint8_t len = (l3TxRxPgPtrHd[port][xferDir].hd == pgPtr->tlPg ?
		(pgPtr->tlUsd - l3TxRxPgPtrHd[port][xferDir].hdOfst) : (UNIT - (l3TxRxPgPtrHd[port][xferDir].hdOfst)));

	if (len <= txLen)
	{ // we are at the end of current page
		l3TxRxPgPtrHd[port][xferDir].hd = g_next[(l3TxRxPgPtrHd[port][xferDir].hd)];
		l3TxRxPgPtrHd[port][xferDir].hdOfst = 0;
	}
	else
	{
		len = min(txLen, len);
		l3TxRxPgPtrHd[port][xferDir].hdOfst += len;
	}

	return len;
}

static inline bool ipTxQPending(const uint8_t port, const uint8_t prio) {
	if (l3ipTxQCount[port][prio]) {
		const uint8_t ipTxQTl = l3ipTxQTail[port][prio];
		IpTxQueue_t* l3IpTxQ = &g_ipTxQ[port][prio][ipTxQTl];
		if (l3IpTxQ->fragCnt) {
			/* if a frag was sent and its still there then its pending */
			return true;
		}
	}
	return false;
}

static inline uint16_t getIpv4HdrLen(const L3Hdr* const l3Hdr) {
	const uint8_t ihlWords = (uint8_t)(l3Hdr->verIhl & IPV4_WORD_MASK);
	const uint16_t hdrLenBytes = (uint16_t)ihlWords << 2;
	return hdrLenBytes;
}

#if MAX_PORT > 1 /* Forwarding */
#define MAX_FORWARD_QUEUE 4 // power of 2
#define MAX_FORWARD_QUEUE_MASK (MAX_FORWARD_QUEUE - 1u)
L3Pkt l3FrwdQ[MAX_PORT][MAX_PRIORITY][MAX_FORWARD_QUEUE];
PgPtr_t l3FrwdPgPtrQ[MAX_PORT][MAX_PRIORITY][MAX_FORWARD_QUEUE];
/* Keep one page ptr for the RX Frwd Pkt */
PgPtr_t l3RxFrwdPktPgPtr[MAX_PORT];

static uint8_t l3FrwdQHead[MAX_PORT][MAX_PRIORITY];
static uint8_t l3FrwdQTail[MAX_PORT][MAX_PRIORITY];
static uint8_t l3FrwdQCount[MAX_PORT][MAX_PRIORITY];

static inline void l3FrwdQInit(void)
{
	memset(l3FrwdQ, 0, sizeof(l3FrwdQ));

	// invalidate the page ptrs
	for (uint8_t port = 0; port < MAX_PORT; port++)
	{
		pgPtrInit(&l3RxFrwdPktPgPtr[port]);
		for (uint8_t prio = 0; prio < MAX_PRIORITY; prio++)
		{
			for (uint8_t queueIdx = 0; queueIdx < MAX_FORWARD_QUEUE; queueIdx++)
			{
				pgPtrInit(&l3FrwdPgPtrQ[port][prio][queueIdx]);
			}
		}
	}

	memset(l3FrwdQHead, 0, sizeof(l3FrwdQHead));
	memset(l3FrwdQTail, 0, sizeof(l3FrwdQTail));
	memset(l3FrwdQCount, 0, sizeof(l3FrwdQCount));
}

static inline void setL3FrwdPktHd(const uint8_t port, const uint8_t prio, const uint8_t xferDir)
{
	const uint8_t frwdQHd = l3FrwdQHead[port][prio];
	PgPtr_t *frwdPgPtr = &l3FrwdPgPtrQ[port][prio][frwdQHd];
	setL3TxRxHd(frwdPgPtr, xferDir, port);
}

static inline void l3FwdTxCmplt(L3Pkt *l3Pkt, const uint8_t port)
{
	/* TODO Fragment? */

	const uint8_t prio = l3Pkt->hdr.prio;
	// free the allocated pages
	const uint8_t frwdQHd = l3FrwdQHead[port][prio];
	freePgPtr(&l3FrwdPgPtrQ[port][prio][frwdQHd]);
	// free the forward queue
	l3FrwdQHead[port][prio] = (uint8_t)((l3FrwdQHead[port][prio] + 1u) % MAX_FORWARD_QUEUE);
	l3FrwdQCount[port][prio]--;
}

static inline bool l3RxfrwdPkt(const L3Hdr *const l3Hdr, const uint8_t port)
{
	// dst addr is not to this device port
	const bool unicast = l3Hdr->dst == l3AddrTblPrio[myPos][port];
	return !l3TxBrdcstPkt(l3Hdr) && !l3Multicst(l3Hdr, port) && !unicast;
}

static inline bool l3TxfrwdPkt(const L3Hdr *const l3Hdr, const uint8_t port)
{
	// src addr is not from this device port
	return l3Hdr->src != l3AddrTblPrio[myPos][port];
}

static inline bool passMst(uint8_t port, uint8_t prioIdx);

static inline bool passMstFrwd(const uint8_t port, const uint8_t prioIdx)
{
	if (prioIdx < LOW_PRIO_IDX) {
		// more high prio frwd pkts waiting
		if (l3FrwdQCount[port][prioIdx] > 1 || l3ipTxQCount[port][prioIdx]) {
			return false;
		}
	} else if (ipTxQPending(port, prioIdx)) {
		/* tx head is pending for current low prio */
		return false;
	}

	return passMst(port, prioIdx+1); // check from pos 0 of this prio if there is a pending high prio stream or frwd pkt
}

static inline uint8_t getL3RxFrwdPktFrag(PgPtr_t *const frwdPktPtr, uint8_t **ptr, uint8_t rxLen)
{
	uint8_t len;
	*ptr = getPgPtr(frwdPktPtr, &len, rxLen);
	return len;
}

static inline PgPtr_t * l3PushFrwdPkt(L3Pkt *const l3Pkt, const uint8_t port)
{
	const uint8_t prio = l3Pkt->hdr.prio;

	if (l3FrwdQCount[port][prio] >= MAX_FORWARD_QUEUE) {
		// TODO do we tx L3 forward Queue full or prio error?
		return NULL; /* full */
	}
	
	const uint8_t frwdQIdx = l3FrwdQTail[port][prio];
	L3Pkt *l3FrwdPkt = &l3FrwdQ[port][prio][frwdQIdx];
	
	/* Copy the header */
	memcpy(l3FrwdPkt, l3Pkt, sizeof(L3Hdr));

	l3FrwdQTail[port][prio] = (uint8_t)((l3FrwdQTail[port][prio] + 1u) % MAX_FORWARD_QUEUE);
	l3FrwdQCount[port][prio]++;
	return &l3FrwdPgPtrQ[port][prio][frwdQIdx];
}

static inline PosType_t l3GetSrcPos(IpAddrType_t srcAddr) {
	for (PosType_t pos = 0; pos < MAX_POS; pos++) {
		if (pos == myPos) {
			continue;
		}

		for (uint8_t port = 0; port < MAX_PORT; port++) {
			if (l3AddrTblPrio[pos][port] == srcAddr) {
				return pos;
			}
		}
	}
	return MAX_POS;
}

static inline void l3BrdCst(const uint8_t port, L3Pkt *const l3Pkt, PgPtr_t * const rxPgPtr, uint16_t startByte, uint16_t payloadLen) {
	const L3Hdr *l3Hdr = &l3Pkt->hdr;
	const uint8_t portSubnet = l3AddrTblPrio[myPos][port] >> 8;
	const uint8_t prio = l3Hdr->prio;

	/* identify the src pos */
	PosType_t srcPos = l3GetSrcPos(l3Hdr->src);
	if (srcPos == MAX_POS) {
		return;
	}

	PgPtr_t* pgPtr = NULL;

	for (int peerPort = 0; peerPort < MAX_PORT; peerPort++)
	{
		if (peerPort == port) {
			continue;
		}

		// check brdcast table
		if (l3BcastInSubnetForSrcPort[srcPos][peerPort] == portSubnet) {
			PgPtr_t *frwdQPgPtr = l3PushFrwdPkt(l3Pkt, peerPort);

			if (frwdQPgPtr) {
				if (!pgPtr) {
					getPgPtrSpan(rxPgPtr, frwdQPgPtr, startByte, payloadLen);
					pgPtr = frwdQPgPtr;
				}
				else {
					*frwdQPgPtr = *pgPtr;
				}
				addUser(frwdQPgPtr);
			} else {
				/* Todo do we inform to sender that it couldnt broadcast? */
			}
		}
	}
}

#endif

static inline bool passMst(uint8_t port, uint8_t prioIdx) {

	for (uint8_t prio = prioIdx; prio < MAX_PRIORITY; prio++)
	{
		if (prio < LOW_PRIO_IDX) {
			if (
#if MAX_PORT > 1
				l3FrwdQCount[port][prio] ||
#endif				
				l3ipTxQCount[port][prio]) {
				return false;
			}
		}
		else {
			/* TODO for forward if its also fragmented ? */

			// check if a tx head is pending and was prempted by higher prio
			if (ipTxQPending(port, prio)) {
				return false;
			}
		}
	}
	return true;
}

static inline void l3InitIpReassQ(IpReassQueue_t * const q) {
	memset(&q->key, 0, sizeof(q->key));
	pgPtrInit(&q->pgPtr);
	q->rxLen = q->maxLen = 0;
	memset(&q->blockMap, 0x00, sizeof(q->blockMap));
}

static inline void l3ReleaseIpReassQ(IpReassQueue_t* const q) {
	if (q) {
		freePgPtr(&q->pgPtr);
		l3InitIpReassQ(q);
	}
}

void l3Init(void) {
	l3FragId = 0; // reset here for Unit tests and if init can be called again on same boot
#if MAX_PORT > 1
	l3FrwdQInit();
#endif
	for (uint8_t qIdx = 0; qIdx < IP_MAX_REASS_QUEUES; qIdx++) {
		l3InitIpReassQ(&g_ipReass[qIdx]);
	}
}

void l3TxCmplt(L3Pkt* l3Pkt, const uint8_t port) {
	L3Hdr* l3Hdr = &l3Pkt->hdr;

#if MAX_PORT > 1
	if (l3TxfrwdPkt(l3Hdr, port)) {
		l3FwdTxCmplt(l3Pkt, port);
		return;
	}
#endif

	const uint8_t prio = l3Pkt->hdr.prio;
	// free the allocated pages
	const uint8_t ipTxQHd = l3ipTxQHead[port][prio];
	IpTxQueue_t * const ipTxQ = &g_ipTxQ[port][prio][ipTxQHd];
	PgPtr_t* const pgPtr = &ipTxQ->pgPtr;

	if (l3Hdr->fragOfst & IP_MORE_FRAG_MASK) {
		/* advance page ptr by tx len */
		const uint16_t payloadLen = (l3Hdr->totalLen - getIpv4HdrLen(l3Hdr));
		freePgPtrLen(pgPtr, payloadLen);
		ipTxQ->fragCnt++;
	} else {
		freePgPtr(pgPtr);
		// free the tx queue
		l3ipTxQHead[port][prio] = (uint8_t)((l3ipTxQHead[port][prio] + 1u) % MAX_FORWARD_QUEUE);
		l3ipTxQCount[port][prio]--;
	}
}

void setL3PktHd(L3Pkt *l3Pkt, uint8_t port, uint8_t xferDir) {
	L3Hdr* l3Hdr = &l3Pkt->hdr;

#if MAX_PORT > 1
	if (l3TxfrwdPkt(l3Hdr, port)) {
		setL3FrwdPktHd(port, l3Hdr->prio, xferDir);
		return;
	}
#endif

	const uint8_t prio = l3Hdr->prio;
	const uint8_t ipTxQHd = l3ipTxQHead[port][prio];
	PgPtr_t* ipTxPgPtr = &g_ipTxQ[port][prio][ipTxQHd].pgPtr;
	setL3TxRxHd(ipTxPgPtr, xferDir, port);
}

void l3AbortRx(L3Pkt* const l3Pkt, const uint8_t port) {
	L3Hdr* l3Hdr = &l3Pkt->hdr;

#if MAX_PORT > 1
	if (l3TxfrwdPkt(l3Hdr, port)) {
		freePgPtr(&l3RxFrwdPktPgPtr[port]);
		return;
	}
#endif

	//l4AbortRx();
}

static inline void setL3Hdr(L3Hdr* const l3Hdr, const uint16_t src, const uint8_t prio, uint16_t dstAddr) {
	l3Hdr->src = src;
	l3Hdr->prio = prio;
	l3Hdr->dst = dstAddr;
}

bool passMstIpTx(const uint8_t port, const uint8_t prio, const uint8_t ipTxQCnt) {
	if (prio < LOW_PRIO_IDX && ipTxQCnt > 1) {
		return false;
	}
	return passMst(port, prio+1); // check from next prio
}

bool getl3Pkt(uint8_t port, L3Pkt* l3Pkt, bool* xferMst, uint8_t * l2Addr) {
	*xferMst = false;
	const uint16_t portAddr = l3AddrTblPrio[myPos][port];
	const uint8_t portSubnet = ((portAddr & 0xFF00) >> 8);
	L3Hdr* l3Hdr = &l3Pkt->hdr;

	for (uint8_t prio = 0; prio < MAX_PRIORITY; prio++) {
		uint8_t pos = 0;
		
#if MAX_PORT > 1
		// check forward packet first 
		if (l3FrwdQCount[port][prio])
		{
			/* TODO what if we are still being filled from other port */
			
			/* Copy the header */
			const uint8_t frwdQHd = l3FrwdQHead[port][prio];
			L3Pkt *l3FrwdPkt = &l3FrwdQ[port][prio][frwdQHd];
			memcpy(l3Pkt, l3FrwdPkt, sizeof(L3Hdr));
			const uint8_t dstSubnet = l3Pkt->hdr.dst >> 8;
			const uint8_t gateway = l3RouteTable[dstSubnet];
			*l2Addr = (gateway) ? gateway : l3Pkt->hdr.dst; /* if dst subnet is distant it will have gateway in route table */
			// TODO MST pass ?
			*xferMst = passMstFrwd(port, prio);
			return true;
		}
#endif	
		const uint8_t ipTxQCnt = l3ipTxQCount[port][prio];

		if (ipTxQCnt) {
			const uint8_t txQHd = l3ipTxQHead[port][prio];
			IpTxQueue_t* ipTxQ = &g_ipTxQ[port][prio][txQHd];

			/* set l3 header */
			l3Hdr->src = portAddr;
			l3Hdr->dst = ipTxQ->key.dst;
			l3Hdr->prio = prio;
			l3Hdr->proto = ipTxQ->key.proto;
			l3Hdr->fragId = ipTxQ->key.id;

			uint16_t offset = (L3_FRAG_SIZE(port) * ipTxQ->fragCnt) >> 3;

			if (ipTxQ->pgPtr.len > L3_FRAG_SIZE(port)) {
				l3Hdr->totalLen = sizeof(L3Hdr) + L3_FRAG_SIZE(port);
				l3Hdr->fragOfst = IP_MORE_FRAG_MASK | (offset & IP_FRAG_OFFSET_MASK);
			}
			else {
				l3Hdr->totalLen = sizeof(L3Hdr) + ipTxQ->pgPtr.len;
				l3Hdr->fragOfst = (offset & IP_FRAG_OFFSET_MASK);  // MF = 0
				*xferMst = passMstIpTx(port, prio, ipTxQCnt); // last fragment can pass MST if there are no other high prio or pending frags below ...
			}
			
			const uint8_t dstSubnet = l3Hdr->dst >> L3_ADDR_SUBNET_SHIFT;
			l3Hdr->ttl = (l3Hdr->dst)? l3RouteHops[dstSubnet] + 1: l3MaxHops + 1; // todo Brdcast TTL
			*l2Addr = ((!l3Hdr->dst || dstSubnet == portSubnet) ? l3Hdr->dst : l3RouteTable[dstSubnet]) & 0x00FF;
			return true;
		}
	}

	return false;
	//return false;
}

uint8_t getL3PktFrag(L3Pkt* const l3Pkt, uint8_t** ptr, uint8_t idx, uint8_t txLen, uint8_t port, uint8_t xferDir) {
	L3Hdr* l3Hdr = &l3Pkt->hdr;

	/* check if we have already transmitted total lenght */
	if (idx >= l3Hdr->totalLen) {
		return 0;
	} else if (idx + txLen >= l3Hdr->totalLen) {
		/* cap tx len */
		txLen = l3Hdr->totalLen - idx;
	}

#if MAX_PORT > 1
	if (l3TxfrwdPkt(l3Hdr, port)) {
		const uint8_t frwdQHd = l3FrwdQHead[port][l3Hdr->prio];
		PgPtr_t * frwdPgPtr = &l3FrwdPgPtrQ[port][l3Hdr->prio][frwdQHd];
		return getL3PktFragFrmPgPtr(frwdPgPtr, ptr, idx, txLen, xferDir, port);
	}
#endif
	
	const uint8_t txQHd = l3ipTxQHead[port][l3Hdr->prio];
	PgPtr_t* txPgPtr = &g_ipTxQ[port][l3Hdr->prio][txQHd].pgPtr;
	return getL3PktFragFrmPgPtr(txPgPtr, ptr, idx, txLen, xferDir, port);
}

static inline uint8_t * getPgPtrNoAlloc(PgPtr_t* pgPtr, uint8_t * availLen, uint8_t len) {
	if (pgPtr->hdPg == pgPtr->tlPg &&
		(pgPtr->hdOfst == pgPtr->tlUsd)) {
		return NULL;
	}

	const uint16_t base = pageOff(pgPtr->hdPg) + (uint16_t)(pgPtr->hdOfst); /* current head */
	uint8_t take;
	
	
	if (pgPtr->hdPg == pgPtr->tlPg) {
		/* cap take to tail used */
		take = min(len, (pgPtr->tlUsd - pgPtr->hdOfst));
	} else {
		take = min(len, (UNIT - pgPtr->hdOfst));
	}

	*availLen = take;
	pgPtr->hdOfst += take;

	if (pgPtr->hdPg == pgPtr->tlPg &&
		(pgPtr->hdOfst >= pgPtr->tlUsd)) {
		pgPtr->hdOfst = pgPtr->tlUsd;
	}
	else if (pgPtr->hdOfst == UNIT) {
		/* free and advance head */
		uint8_t currPage = pgPtr->hdPg;
		pgPtr->hdPg = g_next[currPage];

#if 0
		if (pgPtr->hdPg == INVALID_PAGE) {
			pgPtr->tlPg == INVALID_PAGE;
			return (origLen - len);
		}
#endif

		pgPtr->hdOfst = 0;
	}

	return &g_pool[base];
}

uint8_t getL3RxPktFrag(uint8_t port, L3Pkt* l3Pkt, uint8_t** ptr, uint8_t rxLen)
{
	L3Hdr* l3Hdr = &l3Pkt->hdr;

#if MAX_PORT > 1
	if (l3RxfrwdPkt(l3Hdr, port)) {
		return getL3RxFrwdPktFrag(&l3RxFrwdPktPgPtr[port], ptr, rxLen);
	}
#endif

	return getL3PktFragFrmPgPtr(&l3Pkt->ipReassQueue->pgPtr, ptr, 0, rxLen, L2_XFER_RX, port);
#if 0
	uint8_t len;
	*ptr = getPgPtrNoAlloc(&l3Pkt->ipReassQueue->rxPgPtr, &len, rxLen);
	return len;
#endif
	//return getL4RxPktFrag(&l3Pkt->l4Pkt, ptr, rxLen, l3Hdr->prio);
}

static inline bool ipReassKeyEqual(const IpReassKey_t* key, const L3Hdr* hdr)
{
	return (key->src == hdr->src) &&
		(key->dst == hdr->dst) &&
		(key->id == hdr->fragId) &&
		(key->proto == hdr->proto);
}

static bool ipReassLoadRxPgPTr(IpReassQueue_t* const ipReassObj, const L3Hdr * const l3Hdr, const uint8_t port)
{
	/* Todo check if there is enough pages else fail early */
	/* set up to frag idx */
	/* pre allocate */
	const uint16_t payloadLen = (l3Hdr->totalLen - getIpv4HdrLen(l3Hdr));
	uint16_t startByte = (uint16_t)((l3Hdr->fragOfst & IP_FRAG_OFFSET_MASK) << 3);
	uint32_t endByte = (uint32_t)startByte + (uint32_t)payloadLen;
	
	if (!allocPgPtr(&ipReassObj->pgPtr, endByte)) { /* first allocate up to start byte */
		return false;
	}

	PgPtrHd_t* const pgPtrHd = &l3TxRxPgPtrHd[port][L2_XFER_RX];
	pgPtrHd->hd = ipReassObj->pgPtr.hdPg;
	pgPtrHd->hdOfst = ipReassObj->pgPtr.hdOfst;
	advancePgPtrHd(pgPtrHd, startByte);
	//getPgPtrSpan(&ipReassObj->pgPtr, &ipReassObj->rxPgPtr, startByte, payloadLen);
	return true;
}

static inline bool ipReassBitGet(const uint8_t* map, uint16_t blockIdx) {
	return (map[blockIdx >> 3] & (uint8_t)(1U << (blockIdx & 0x7U))) != 0U;
}

static inline void ipReassBitSet(uint8_t* map, uint16_t blockIdx) {
	map[blockIdx >> 3] |= (uint8_t)(1U << (blockIdx & 0x7U));
}

static inline bool ipReassHdrValid(const L3Hdr* const l3Hdr, IpReassQueue_t* const ipReassQueue) {
	const uint8_t ipvVer = (l3Hdr->verIhl & IPV4_VERSION_MASK) >> IPV4_VERSION_SHIFT;

	if (ipvVer == IPV4_VERSION) {
		const uint16_t hdrLenBytes = getIpv4HdrLen(l3Hdr);

		if ((l3Hdr->totalLen < hdrLenBytes) ||
			(hdrLenBytes < IPV4_HDR_MIN_LEN) ||
			(hdrLenBytes > IPV4_HDR_MAX_LEN)) {
			return false;
		}

		const uint16_t payloadLen = (l3Hdr->totalLen - hdrLenBytes);

		// TODO future: Hdr Checksum validation ?, ipv6 has no header checksum validation

		if (payloadLen) {
			bool moreFragments = l3Hdr->fragOfst & IP_MORE_FRAG_MASK;
			const uint16_t startByte = (uint16_t)((l3Hdr->fragOfst & IP_FRAG_OFFSET_MASK) << 3);
			/* Use wider arithmetic for validation */
			const uint32_t endByte = (uint32_t)startByte + (uint32_t)payloadLen;
			if (endByte > IP_REASS_MAX_LEN) {
				/* if there is a q clear it */
				l3ReleaseIpReassQ(ipReassQueue);
				return false;
			}

			if (moreFragments && (payloadLen & 0x7U) != 0U) {
				/* in ipv4 every fragment except the last need to be a multiple of 8 bytes RFC 791 */
				// TODO drop whole q?
				return false;
			}

			/* check for overlap/dupe */
			if (ipReassQueue) {
				const uint16_t startBlk = (uint16_t)(startByte >> 3);
				const uint16_t endBlk = (uint16_t)((endByte - 1U) >> 3);

				for (uint16_t blk = startBlk; blk <= endBlk; blk++) {
					if (ipReassBitGet(ipReassQueue->blockMap, blk)) {
						/* TODO future: secure IP will drop entire q (linux behaviour) for overlap (dupes are dropped but q is kept), 
							but for now this network is assumed to be secure so no need to verify if overlap and drop q */
						return false; // dupe/overlap packet
					}
				}
			}
		} else {
			/* TODO Hdr messages should be processed directly here in l3*/
			return false;
		}
	} else {
		/* TODO future: ipv6 */
		return false;
	}

	return true;
}

static inline IpReassQueue_t * ipReassFindQueue(const L3Hdr * hdr)
{
	uint8_t i;
	for (i = 0U; i < IP_MAX_REASS_QUEUES; i++)
	{
		if (ipReassKeyEqual(&g_ipReass[i].key, hdr) == true) {
			return &g_ipReass[i];
		}
	}
	return NULL;
}

static inline IpReassQueue_t * ipReassAllocQueue(const L3Hdr * hdr)
{
    uint8_t i;
    for (i = 0U; i < IP_MAX_REASS_QUEUES; i++)
    {
		if (g_ipReass[i].key.proto == IP_PROTO_UNKOWN)
		{
			g_ipReass[i].key.src = hdr->src;
			g_ipReass[i].key.dst = hdr->dst;
			g_ipReass[i].key.proto = hdr->proto;
			g_ipReass[i].key.id = hdr->fragId;
			g_ipReass[i].expireTick = pitGetCurrMS() + IP_REASS_TIMEOUT_TICKS;
			return &g_ipReass[i];
        }
    }
    return NULL;
}

bool l3CmtRxHd(L3Pkt *l3Pkt, const uint8_t port) { // called from L2
	L3Hdr *l3Hdr = &l3Pkt->hdr;

	if (!l3Hdr->ttl) {
		return false; // drop packet
	}

#if MAX_PORT > 1
	if (l3RxfrwdPkt(l3Hdr, port)) {
		if (l3Hdr->ttl > 1) {
			const uint8_t dstSubnet = l3Hdr->dst >> L3_ADDR_SUBNET_SHIFT;

			const uint8_t gatewaySubnet = l3RouteTable[dstSubnet] >> 8;

			for (uint8_t peerPort = 0; peerPort < MAX_PORT; peerPort++)
			{
				const uint16_t peerPortL3Addr = l3AddrTblPrio[myPos][peerPort];

				if (peerPort == port || !peerPortL3Addr)
				{
					continue;
				}

				// should not happen with the routing model but just incase
				if (peerPortL3Addr == l3Hdr->dst)
				{
					// update the dst for rx cmt so it dosnt think its forwarded pkt
					l3Hdr->dst = l3AddrTblPrio[myPos][port];
					return true;
				}

				const uint8_t peerPortSubnet = peerPortL3Addr >> 8;
				if ((dstSubnet == peerPortSubnet) ||
					(gatewaySubnet == peerPortSubnet))
				{ // forward on peer port
					l3Pkt->frwdPkt.dstPort = peerPort;
					return true;
				}
			}
			return false;
		}
	}
#endif

	IpReassQueue_t *q = ipReassFindQueue(l3Hdr);

	if (!ipReassHdrValid(l3Hdr, q)) {
		return false;
	}

	if (!q) {
		q = ipReassAllocQueue(l3Hdr);
	}
	
	if (!q || !ipReassLoadRxPgPTr(q, l3Hdr, port)) {
		return false;
	}

	l3Pkt->ipReassQueue = q;
	return true;
}

bool l3RxCmplt(uint8_t rxIdx) {
	if (rxIdx <= sizeof(L3Hdr)) { // only recieved hdr or partial hdr
		/*
			if we accept l3 hdr only message TODO? commit l3 hd
		*/
		return false;
	}
	return true;
}

void l3CmtRx(L3Pkt * const l3Pkt, const uint8_t port, uint8_t l3RxLen)
{
	// check if message if for this device
	L3Hdr * const l3Hdr = &l3Pkt->hdr;

#if MAX_PORT > 1
	
	if (l3RxfrwdPkt(l3Hdr, port)) {
		if (l3Hdr->ttl > 1) {
			l3Hdr->ttl--;
			PgPtr_t *frwdQPgPtr = l3PushFrwdPkt(l3Pkt, l3Pkt->frwdPkt.dstPort);
			if (frwdQPgPtr) {
				/* copy pg ptr */
				memcpy(frwdQPgPtr, &l3RxFrwdPktPgPtr[port], sizeof(PgPtr_t));
				pgPtrInit(&l3RxFrwdPktPgPtr[port]);
			} else {
				/* Todo do we let the sender know  that frwd queue is full?*/
				freePgPtr(&l3RxFrwdPktPgPtr[port]);
			}
		}
		return;
	}
#endif

	IpReassQueue_t *q = l3Pkt->ipReassQueue;
	bool moreFragments = l3Hdr->fragOfst & IP_MORE_FRAG_MASK;
	const uint16_t payloadLen = (l3Hdr->totalLen - getIpv4HdrLen(l3Hdr));
	q->rxLen += payloadLen;
	const uint16_t startByte = (uint16_t)((l3Hdr->fragOfst & IP_FRAG_OFFSET_MASK) << 3);
	const uint32_t endByte = (uint32_t)startByte + (uint32_t)payloadLen;

#if MAX_PORT > 1
	/* Check to brodcast */
	if (l3TxBrdcstPkt(l3Hdr) && l3Hdr->ttl > 1) {
		l3Hdr->ttl--;
		l3BrdCst(port, l3Pkt, &l3Pkt->ipReassQueue->pgPtr, startByte, payloadLen);
	}
#endif
	
	if (!moreFragments) { // this is the last frag
		// set max len
		q->maxLen = endByte;
	}

	if (q->maxLen == q->rxLen) {
		l4CmtRx(&q->pgPtr, q->key.proto, q->maxLen);
		l3InitIpReassQ(q); // release the q without clearing pgPtr (l4 might still use it ...)
	} else {
		/* set block map */
		uint16_t  startBlk = (uint16_t)(startByte / 8U);
		uint16_t  endBlk = (uint16_t)((endByte - 1U) / 8U);
		for (uint16_t blk = startBlk; blk <= endBlk; blk++) {
			ipReassBitSet(q->blockMap, blk);
		}
	}
}
