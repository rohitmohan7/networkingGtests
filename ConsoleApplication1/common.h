#include "global.h"

/* -------------------- Main -------------------- */

#define __IO
#define __I

typedef struct {
    __IO uint8_t BDH;                                /**< UART Baud Rate Registers: High, offset: 0x0 */
    __IO uint8_t BDL;                                /**< UART Baud Rate Registers: Low, offset: 0x1 */
    __IO uint8_t C1;                                 /**< UART Control Register 1, offset: 0x2 */
    __IO uint8_t C2;                                 /**< UART Control Register 2, offset: 0x3 */
    __I  uint8_t S1;                                 /**< UART Status Register 1, offset: 0x4 */
    __IO uint8_t S2;                                 /**< UART Status Register 2, offset: 0x5 */
    __IO uint8_t C3;                                 /**< UART Control Register 3, offset: 0x6 */
    __IO uint8_t D;                                  /**< UART Data Register, offset: 0x7 */
    __IO uint8_t MA1;                                /**< UART Match Address Registers 1, offset: 0x8 */
    __IO uint8_t MA2;                                /**< UART Match Address Registers 2, offset: 0x9 */
    __IO uint8_t C4;                                 /**< UART Control Register 4, offset: 0xA */
    __IO uint8_t C5;                                 /**< UART Control Register 5, offset: 0xB */
    __I  uint8_t ED;                                 /**< UART Extended Data Register, offset: 0xC */
    __IO uint8_t MODEM;                              /**< UART Modem Register, offset: 0xD */
    __IO uint8_t IR;                                 /**< UART Infrared Register, offset: 0xE */
    uint8_t RESERVED_0[1];
    __IO uint8_t PFIFO;                              /**< UART FIFO Parameters, offset: 0x10 */
    __IO uint8_t CFIFO;                              /**< UART FIFO Control Register, offset: 0x11 */
    __IO uint8_t SFIFO;                              /**< UART FIFO Status Register, offset: 0x12 */
    __IO uint8_t TWFIFO;                             /**< UART FIFO Transmit Watermark, offset: 0x13 */
    __I  uint8_t TCFIFO;                             /**< UART FIFO Transmit Count, offset: 0x14 */
    __IO uint8_t RWFIFO;                             /**< UART FIFO Receive Watermark, offset: 0x15 */
    __I  uint8_t RCFIFO;                             /**< UART FIFO Receive Count, offset: 0x16 */
    uint8_t RESERVED_1[1];
    __IO uint8_t C7816;                              /**< UART 7816 Control Register, offset: 0x18, available only on: UART0 (missing on UART1, UART2) */
    __IO uint8_t IE7816;                             /**< UART 7816 Interrupt Enable Register, offset: 0x19, available only on: UART0 (missing on UART1, UART2) */
    __IO uint8_t IS7816;                             /**< UART 7816 Interrupt Status Register, offset: 0x1A, available only on: UART0 (missing on UART1, UART2) */
    __IO uint8_t WP7816;                             /**< UART 7816 Wait Parameter Register, offset: 0x1B, available only on: UART0 (missing on UART1, UART2) */
    __IO uint8_t WN7816;                             /**< UART 7816 Wait N Register, offset: 0x1C, available only on: UART0 (missing on UART1, UART2) */
    __IO uint8_t WF7816;                             /**< UART 7816 Wait FD Register, offset: 0x1D, available only on: UART0 (missing on UART1, UART2) */
    __IO uint8_t ET7816;                             /**< UART 7816 Error Threshold Register, offset: 0x1E, available only on: UART0 (missing on UART1, UART2) */
    __IO uint8_t TL7816;                             /**< UART 7816 Transmit Length Register, offset: 0x1F, available only on: UART0 (missing on UART1, UART2) */
    uint8_t RESERVED_2[26];
    __IO uint8_t AP7816A_T0;                         /**< UART 7816 ATR Duration Timer Register A, offset: 0x3A, available only on: UART0 (missing on UART1, UART2) */
    __IO uint8_t AP7816B_T0;                         /**< UART 7816 ATR Duration Timer Register B, offset: 0x3B, available only on: UART0 (missing on UART1, UART2) */
    union {                                          /* offset: 0x3C */
        struct {                                         /* offset: 0x3C */
            __IO uint8_t WP7816A_T0;                         /**< UART 7816 Wait Parameter Register A, offset: 0x3C, available only on: UART0 (missing on UART1, UART2) */
            __IO uint8_t WP7816B_T0;                         /**< UART 7816 Wait Parameter Register B, offset: 0x3D, available only on: UART0 (missing on UART1, UART2) */
        } TYPE0;
        struct {                                         /* offset: 0x3C */
            __IO uint8_t WP7816A_T1;                         /**< UART 7816 Wait Parameter Register A, offset: 0x3C, available only on: UART0 (missing on UART1, UART2) */
            __IO uint8_t WP7816B_T1;                         /**< UART 7816 Wait Parameter Register B, offset: 0x3D, available only on: UART0 (missing on UART1, UART2) */
        } TYPE1;
    };
    __IO uint8_t WGP7816_T1;                         /**< UART 7816 Wait and Guard Parameter Register, offset: 0x3E, available only on: UART0 (missing on UART1, UART2) */
    __IO uint8_t WP7816C_T1;                         /**< UART 7816 Wait Parameter Register C, offset: 0x3F, available only on: UART0 (missing on UART1, UART2) */
} UART_Type;
#define UART_C2_TE_MASK                          (0x8U)
#define UART_S1_RDRF_MASK                        (0x20U)
#define UART_C2_TIE_MASK                         (0x80U)
#define UART_C2_TCIE_MASK                        (0x40U)

/*! C2_RIE - Receiver Full Interrupt or DMA Transfer Enable
 *  0b0..RDRF interrupt and DMA transfer requests disabled.
 *  0b1..RDRF interrupt or DMA transfer requests enabled.
 */
#define UART_C2_RIE_MASK                         (0x20U)