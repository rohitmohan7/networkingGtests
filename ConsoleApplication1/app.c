#include "app.h"
#include "allocator.h"
#include "layer4.h"


bool appSend(const uint8_t* data, uint16_t len, uint16_t pos, uint8_t priority, bool retry)
{
    if ((data == NULL) || (len == 0U)) { return false; }
    if ((pos >= (uint16_t)MAX_POS) || (priority >= (uint8_t)MAX_PRIORITY)) { return false; }

    stream_t* s = &streams[pos][priority];

    /* --- Deterministic capacity check ---
     * Worst-case extra pages needed if tail has some free space:
     * bytes can fit into tail slack first, then pages.
     */
    uint8_t tail_free = 0U;
    if (s->tail_page != INVALID_PAGE)
    {
        tail_free = UNIT - s->tail_used;  /* 0..UNIT */
    }
    uint16_t reqLen = len + ((s->tail_page != INVALID_PAGE) ? (sizeof(len) + sizeof(txOrder)): 0);
    const uint8_t bytes_after_tail = (len > tail_free) ? (uint16_t)((reqLen) - tail_free) : 0U;
    const uint8_t need_pages = (bytes_after_tail == 0U) ? 0U : ceilPages(bytes_after_tail);

    if (g_free_count < need_pages)
    {
        return false; /* deterministic fail */
    }

    uint8_t msgFlgs = (0 & ~L4_MSG_FLAG_TYPE_ACK) | (retry > 0 ? L4_MSG_FLAG_REQ_ACK : 0);

    if (s->tail_page != INVALID_PAGE) { // stream is corrupted here ? should free everything ? TODO
        writeValToPage(s, (uint8_t*)&len, sizeof(len));
        writeValToPage(s, (uint8_t*)&txOrder, sizeof(txOrder));
        writeValToPage(s, (uint8_t*)&msgFlgs, sizeof(msgFlgs));
    }

    /* --- Append bytes into stream --- */
    uint16_t in = 0U;

    /* Ensure a tail page exists if stream empty */
    if (s->tail_page == INVALID_PAGE)
    {
        uint8_t p = page_alloc();
        s->head_page = p;
        s->tail_page = p;
        // s->head_off = 0U;
        s->tail_used = 0U;

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
        if ((uint16_t)s->tail_used >= (uint16_t)UNIT)
        {
            uint8_t p = page_alloc();
            g_next[s->tail_page] = p;
            s->tail_page = p;
            s->tail_used = 0U;
        }

        /* Copy into tail page */
        const uint16_t space = (uint16_t)UNIT - (uint16_t)s->tail_used;
        uint16_t take = (uint16_t)len - in;
        if (take > space) { take = space; }

        {
            const uint32_t base = pageOff(s->tail_page) + (uint32_t)s->tail_used;
            for (uint16_t i = 0U; i < take; ++i)
            {
                g_pool[base + (uint32_t)i] = data[in + i];
            }
        }

        s->tail_used = (uint8_t)((uint16_t)s->tail_used + take);
        //   s->queued_bytes = (uint16_t)(s->queued_bytes + take);
        in = (uint16_t)(in + take);
    }
    txOrder++;
    return true;
}

bool appRecv(uint16_t pos, uint8_t priority) { // TODO
    
}
