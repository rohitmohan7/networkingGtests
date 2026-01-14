#include "app.h"
#include "allocator.h"
#include "layer4.h"




bool appSend(const uint8_t* data, uint16_t len, uint16_t pos, uint8_t priority)
{
    if ((data == NULL) || (len == 0U)) { return false; }
    if ((pos >= (uint16_t)MAX_POS) || (priority >= (uint8_t)MAX_PRIORITY)) { return false; }

    stream_t* sp = &streams[pos];
    prio_stream_t* s = &sp->prio[priority];

    /* --- Deterministic capacity check (optional but recommended) ---
     * Worst-case extra pages needed if tail has some free space:
     * bytes can fit into tail slack first, then pages.
     */
    uint8_t tail_free = 0U;
    if (s->tail_page != INVALID_PAGE)
    {
        for (int size = 0; size < sizeof(len); size++)
        {
            if (s->tail_used == UNIT) {
                uint8_t p = page_alloc();
                if (p == INVALID_PAGE)
                {
                    /* We already checked capacity, so this should not happen,
                       but if it does, return false deterministically without corrupting chain.
                       (No rollback needed because nothing allocated in this iteration if fail here.) */
                    return false;
                }
                g_next[s->tail_page] = p;
                s->tail_page = page_alloc();
                g_pool[pageOff(s->tail_page)] = (uint8_t)((len >> (size * 8)) & 0xFFu);
                s->tail_used = 1U;
            }
            else {
                const uint32_t base = pageOff(s->tail_page) + (uint32_t)s->tail_used;
                g_pool[base] = (uint8_t)((len >> (size * 8)) & 0xFFu);
                s->tail_used++;
            }
        }
        tail_free = UNIT - s->tail_used;  /* 0..UNIT */
    }

    const uint8_t bytes_after_tail = (len > tail_free) ? (uint16_t)(len - tail_free) : 0U;
    const uint8_t need_pages = (bytes_after_tail == 0U) ? 0U : ceilPages(bytes_after_tail);

    if (g_free_count < need_pages)
    {
        return false; /* deterministic fail */
    }

    /* --- Append bytes into stream --- */
    uint16_t in = 0U;

    /* Ensure a tail page exists if stream empty */
    if (s->tail_page == INVALID_PAGE)
    {
        uint8_t p = page_alloc();
        if (p == INVALID_PAGE) { return false; } /* should not happen after check */
        s->head_page = p;
        s->tail_page = p;
        // s->head_off = 0U;
        s->tail_used = 0U;

        // stream is empty init with msg len
        s->msgLen = len;
    }

    while (in < (uint16_t)len)
    {
        /* If current tail page is full, allocate a new one */
        if ((uint16_t)s->tail_used >= (uint16_t)UNIT)
        {
            uint8_t p = page_alloc();
            if (p == INVALID_PAGE)
            {
                /* We already checked capacity, so this should not happen,
                   but if it does, return false deterministically without corrupting chain.
                   (No rollback needed because nothing allocated in this iteration if fail here.) */
                return false;
            }
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

    
    for (int i = 0; i < sizeof(((L4Hdr*)0)->len); i++) { // zero out msg len
        if (s->tail_used == UNIT) { 
            break;
        }
        const uint32_t base = pageOff(s->tail_page) + (uint32_t)s->tail_used;
        g_pool[base + (uint32_t)i] = 0;
    }

    return true;
}