#if 0
#include "global.h"
#include "allocator.h"
#include "layer3.h"
#include "layer4.h"

void scheduler() { // check streams enqueue

    for (uint8_t prio = 0; prio < MAX_PRIORITY; prio++) {
        for (int pos = 0; pos < MAX_POS; pos++) {
            stream_t* s = &streams[pos][prio];
            // check if stream has anything to send

            if (s->head_page == INVALID_PAGE) { // stream has no data
                continue;
            }

            // get port
            uint16_t dst_ip = pos_addr_table[pos];
            uint8_t dst_subnet = dst_ip >> 8;

            // check gateway
            uint16_t gateway_ip = route_table[dst_subnet];

            uint16_t localdst_ip = gateway_ip > 0 ? gateway_ip : dst_ip;
            uint8_t localdst_subnet = (uint8_t)(localdst_ip >> 8);

            // check port subnet
            for (int port = 0; port < MAX_PORT; port++) {
#if 0
                if (l1tx[port].head_page == INVALID_PAGE) { // TX Uart free
                    continue;
                }

                uint8_t port_subnet = port_ip[port] >> 8;

                if (localdst_subnet == port_subnet) { // check if port subnet matches

                    // send one window
                    l1tx[port].hdr = {
                        .l2hdr = {
                            .addr = (uint8_t)localdst_ip,
                            .type = L2_MSG_TYPE_PDU
                        },
                        .l3hdr = {
                            .src = port_ip[port],
                            .dst = dst_ip,
                            .prio = prio,
                            .ttl = g_dist_to_bus[currPos][dst_subnet]
                        },
                        .l4hdr = {
                            .frame_id = p->snd_nxt,
                            .rsv = s->snd_wnd--
                        }
                    };

                    // write header
                    UART_WriteNonBlocking((uint8_t*)&l1tx[port].hdr, sizeof(packet_hdr));

                    /* Get the bytes that available to TX. */
                    uint8_t count = UART_FIFO_SIZE - sizeof(packet_hdr);

                    page_id_t page = l1tx[port].head_page = p->head_page;


                    // TODO write data pages from stream

                    UART->C2 |= (uint8_t)(UART_C2_TIE_MASK | UART_C2_TE_MASK); // enable transmit 
                }
#endif
            }
        }
    }
}
#endif