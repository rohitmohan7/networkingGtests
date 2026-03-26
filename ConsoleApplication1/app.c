#include "app.h"
#include "layer4.h"
#include "layer3.h"


bool appBrdcast(const uint8_t* data, MsgLenType_t len, uint8_t priority) {
    return l3TxBrdcstMsg(data, len, priority);
}

bool appSendUdp(const uint8_t* data, const uint16_t len, const uint8_t priority, const PosType_t pos) {
    return l4SendUdp(data, len, priority, pos);
}

bool appSend(const uint8_t* data, uint16_t len, uint16_t pos, uint8_t priority, bool retry)
{
    if ((data == NULL) || (len == 0U)) { return false; }
    if ((pos >= (uint16_t)MAX_POS) || (priority >= (uint8_t)MAX_PRIORITY)) { return false; }

    stream_t* s = &streams[pos][priority];
    PgPtr_t * pgPtr = &s->txPgPtr;

    /* --- Deterministic capacity check ---
     * Worst-case extra pages needed if tail has some free space:
     * bytes can fit into tail slack first, then pages.
     */
    uint8_t tail_free = 0U;

    if (pgPtr->tlPg != INVALID_PAGE)
    {
        tail_free = UNIT - pgPtr->tlUsd;  /* 0..UNIT */
    }
    uint16_t reqLen = len + ((pgPtr->tlPg != INVALID_PAGE) ? (sizeof(len) + sizeof(txOrder) + sizeof(uint8_t)): 0);
    const uint8_t bytes_after_tail = (len > tail_free) ? (uint16_t)((reqLen) - tail_free) : 0U;
    const uint8_t need_pages = (bytes_after_tail == 0U) ? 0U : ceilPages(bytes_after_tail);

    if (g_free_count < need_pages)
    {
        return false; /* deterministic fail */
    }

    uint8_t msgFlgs = (0 & ~L4_MSG_FLAG_TYPE_ACK) | (retry && pos > 0? L4_MSG_FLAG_REQ_ACK : 0);

    if (pgPtr->tlPg != INVALID_PAGE) { // stream is corrupted here ? should free everything ? TODO
        /* Brdcast frame does not have any msgFlgs from app */
        if (pos) {
            writeValToPage(pgPtr, (uint8_t*)&msgFlgs, sizeof(msgFlgs));
        }
        writeValToPage(pgPtr, (uint8_t*)&len, sizeof(len));
        writeValToPage(pgPtr, (uint8_t*)&txOrder, sizeof(txOrder));
    }

    /* --- Append bytes into stream --- */
    uint16_t in = 0U;

    /* Ensure a tail page exists if stream empty */
    if (pgPtr->tlPg == INVALID_PAGE)
    {
        uint8_t p = page_alloc();
        pgPtr->hdPg = p;
        pgPtr->tlPg = p;
        // s->head_off = 0U;
        pgPtr->tlUsd = 0U;

        // stream is empty init with msg len
        s->txMsgHdr.msgLen = len;
        s->txMsgHdr.msgFlgs = msgFlgs;
        s->retryCnt =
        s->retryTmr = 0;
        // set tx Order
        s->txOrder = txOrder;
    }

    while (in < (uint16_t)len)
    {
        /* If current tail page is full, allocate a new one */
        if ((uint16_t)pgPtr->tlUsd >= (uint16_t)UNIT)
        {
            uint8_t p = page_alloc();
            g_next[pgPtr->tlPg] = p;
            pgPtr->tlPg = p;
            pgPtr->tlUsd = 0U;
        }

        /* Copy into tail page */
        const uint16_t space = (uint16_t)UNIT - (uint16_t)pgPtr->tlUsd;
        uint16_t take = (uint16_t)len - in;
        if (take > space) { take = space; }

        {
            const uint32_t base = pageOff(pgPtr->tlPg) + (uint32_t)pgPtr->tlUsd;
            for (uint16_t i = 0U; i < take; ++i)
            {
                g_pool[base + (uint32_t)i] = data[in + i];
            }
        }

        pgPtr->tlUsd = (uint8_t)((uint16_t)pgPtr->tlUsd + take);
        //   s->queued_bytes = (uint16_t)(s->queued_bytes + take);
        in = (uint16_t)(in + take);
    }

    txOrder++;
    return true;
}

#ifdef NETWORK_ISR_RECV
#ifndef UNIT_TEST
void appRecv(uint16_t pos, const uint8_t * const data, uint16_t len) { // TODO
    
    //uint8_t msgType = ; 
}
#endif
#endif
