// sim.c - RS-485 multidrop + routed overlay simulation (adjacency-only config)
//
// Build: gcc -std=c11 -Wall -Wextra -O2 -pthread sim.c -o sim
// Run:   ./sim
//
// Properties:
//  - L2 (RS-485 bus): shared medium broadcast. Every node on a bus receives every frame,
//    but ACCEPTS it only if unit_id (Modbus address) matches its position.
//  - L3 (overlay): src/dst/txid/type/ttl/from/target_bus + payload.
//  - Only config input is the adjacency list (neighbor + local out_port).
//  - Physical buses (BUS1..BUSn) are DERIVED from adjacency by union-find over interfaces (pos,port).
//  - Routing uses: from + MUST-DECREASE-METRIC rule.
//      metric = dist_to_bus[pos][target_bus] (computed once by BFS).
//      forward only to a neighbor with strictly smaller metric.
//
// Notes:
//  - BUS numbering is derived (BUS1/BUS2/...) and may not match your drawing labels.
//  - This is a simulation; bus �hub� broadcasts frames, nodes filter by unit_id.

//#include "global.h"
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <algorithm>
#include <cstring>
#include <array>

#define _Static_assert(cond, msg) static_assert((cond), msg)

extern "C" {

#include "network.h"
#include "layer2.h"
#include "layer1.h"
#include "app.h"
#include "pit.h"
#include "pmm.h"

void PITCallback(uint8_t channel);
}


struct Case {
    int pos;
    std::array<std::array<uint16_t, MAX_PORT>, MAX_POS> l3AddrTblPrio;
    std::array<uint16_t, MAX_SUBNET> l3RouteTable;
    std::array<std::array<uint8_t, MAX_PORT>, MAX_POS> l3BcastInSubnetForSrcPort;
    std::array<uint8_t, MAX_PORT> devCnt;
    //uin
};
#define L2_PIT_TIMER_START_IDX 1

ConfigFile_st config;

class MultiHop : public ::testing::TestWithParam<Case> {
protected:
    static void SetUpTestSuite() {
        
        memset(&config, 0, sizeof(config));
        config.topology[1] = NodeCfg_t{{1, 3}};
        config.topology[2] = NodeCfg_t{{2, 3}};
        config.topology[3] = NodeCfg_t{{1, 2}};
        config.topology[5] = NodeCfg_t{{2, 0}};
        config.topology[7] = NodeCfg_t{{1, 0}};
        g_pmm.config = &config;
        
#if 0
        ::topology[1] = NodeCfg_t{ {1,3} };
        ::topology[2] = NodeCfg_t{ {2,3} };
        ::topology[3] = NodeCfg_t{ {1,2} };
        ::topology[5] = NodeCfg_t{ {2,0} };
        ::topology[7] = NodeCfg_t{ {1,0} };
#endif
        
        for (int i = 0; i < MAX_PORT; ++i) {
            uart_ptrs[i] = &uart_objs[i];
        }
    }

    void SetUp() override {
        pitInit();
        // runs before each TEST_F(MyFixture, ...)
        //myPos = GetParam().pos;
        config.pos = GetParam().pos;

        for (int i = 0; i < MAX_PORT; ++i) {
            memset(uart_ptrs[i], 0, sizeof(UART_Type));
        }

        netInit(uart_ptrs);
    }

    void TearDown() override {
        // runs after each TEST_F(MyFixture, ...)
        //value = 0;
    }

    static UART_Type  uart_objs[MAX_PORT];   // objects
    
public:
    static UART_Type* uart_ptrs[MAX_PORT];   // pointers passed to netInit
};

UART_Type MultiHop::uart_objs[MAX_PORT];
UART_Type* MultiHop::uart_ptrs[MAX_PORT];


struct MockUart {
    MOCK_METHOD(void, l1UARTWriteNonBlocking, (UART_Type* UART, const uint8_t* data, size_t length), ());
    MOCK_METHOD(bool, l1UARTCmpNonBlocking, (UART_Type* UART, const uint8_t* data, size_t length), ());
    MOCK_METHOD(void, l1UARTReadNonBlocking, (UART_Type* UART, uint8_t* data, size_t length), ());
    MOCK_METHOD(uint32_t, pitGetCurrMS, (), ());
};

static MockUart* g_mock = nullptr;

// 3) The linker will redirect calls to hw_read() to __wrap_hw_read()
extern "C" void l1UARTWriteNonBlocking(UART_Type* UART, const uint8_t* data, size_t length)
{
    ASSERT_NE(g_mock, nullptr);
    g_mock->l1UARTWriteNonBlocking(UART, data, length);

    // echo 
#if 0
    for (int port = 0; port < MAX_PORT; port++) {
        if (UART != MultiHop::uart_ptrs[port]) {
            continue;
        }
        UART->S1 |= UART_S1_RDRF_MASK;
        UART->RCFIFO = length;
        UART->S1 &= ~UART_S1_TDRE_MASK; // now data register will no longer be empty
        l1TransferHandleIRQ(UART, port);
        UART->S1 &= ~UART_S1_RDRF_MASK;
        break;
    }
#endif
}

extern "C" void l1UARTReadNonBlocking(UART_Type* UART, uint8_t* data, size_t length)
{
    ASSERT_NE(g_mock, nullptr);
    g_mock->l1UARTReadNonBlocking(UART, data, length);
}

extern "C" bool l1UARTCmpNonBlocking(UART_Type* UART, const uint8_t* data, size_t length)
{
    EXPECT_NE(g_mock, nullptr);
    return  g_mock->l1UARTCmpNonBlocking(UART, data, length);
}

extern "C" uint32_t pitGetCurrMS()
{
    EXPECT_NE(g_mock, nullptr);
    return  g_mock->pitGetCurrMS();
}

static void expectMst(MockUart& mock, uint8_t addr, UART_Type* uart, uint8_t port) {
    std::array<uint8_t, sizeof(L2Hdr)> expected{{addr, L2_PKT_TYPE_MST, 0xFF}};
    const uint8_t expectedSize = (sizeof(L2Hdr));

    EXPECT_CALL(mock, l1UARTWriteNonBlocking(uart, testing::NotNull(), expectedSize))
        .Times(1)
        .WillOnce(testing::Invoke([expected](UART_Type* UART, const uint8_t* data, size_t len) {
            ASSERT_EQ(0, std::memcmp(data, expected.data(), expected.size()));
            }))
        .RetiresOnSaturation();

    uart->S1 = (uint8_t)((uart->S1 & (uint8_t)~UART_S1_TC_MASK) | UART_S1_TDRE_MASK);
    l1TransferHandleIRQ(uart, port); // complete TX of single frame

    ASSERT_TRUE(
        ((uart->C2 & (uint8_t)(UART_C2_TCIE_MASK | UART_C2_TE_MASK)) != 0U) &&
        ((uart->C2 & (uint8_t)(UART_C2_TIE_MASK)) == 0U)
    );

    // send TX complete
    uart->S1 =
        (uint8_t)((uart->S1 & (uint8_t)~UART_S1_TDRE_MASK) | UART_S1_TC_MASK);
    l1TransferHandleIRQ(uart, port);

    // confirm UART TX is disabled
    ASSERT_NE((uart->C2 & ((uint8_t)UART_C2_TIE_MASK | (uint8_t)UART_C2_TCIE_MASK | (uint8_t)UART_C2_TE_MASK)) != 0, true);

    // set the echo register 
    uart->S1 |= UART_S1_RDRF_MASK;
    uart->RCFIFO = expectedSize;
    // echo
    EXPECT_CALL(mock, l1UARTCmpNonBlocking(uart, testing::NotNull(), expectedSize))
        .Times(1)
        .WillOnce(testing::Invoke([expected](UART_Type* UART, const uint8_t* data, size_t len) {
        EXPECT_EQ(0, std::memcmp(data, expected.data(), expected.size()));
        return true;
            }))
        .RetiresOnSaturation();

    // echo isr
    l1TransferHandleIRQ(uart, port);
    uart->S1 &= ~UART_S1_RDRF_MASK;
}

void sendMstToken(MockUart& mock, UART_Type* UART, const uint8_t& l2Addr, const uint8_t& port) {
    std::array<uint8_t, sizeof(L2Hdr)> pkt{{l2Addr, L2_PKT_TYPE_MST, 0xFF}};

    UART->S1 |= UART_S1_RDRF_MASK;
    UART->RCFIFO = pkt.size();

    // call to copy header
    EXPECT_CALL(mock, l1UARTReadNonBlocking(UART, testing::NotNull(), sizeof(L2Hdr)))
        .Times(1)
        .WillOnce(testing::Invoke([pkt](UART_Type* UART, uint8_t* data, size_t len) {
        std::memcpy(data, pkt.data(), len);
            }))
        .RetiresOnSaturation();

    l1TransferHandleIRQ(UART, port);

    UART->S1 &= ~UART_S1_RDRF_MASK;
    UART->RCFIFO = 0;

    PITCallback(port + L2_PIT_TIMER_START_IDX); // interchar silence

    PITCallback(port + L2_PIT_TIMER_START_IDX); // interframe silence
}

void sendMsgAck(MockUart& mock, UART_Type* UART, const PduHdr& pduHdr, const uint8_t& port) {
    std::array<uint8_t, sizeof(PduHdr)> pkt{};

    std::memcpy(pkt.data(), &pduHdr, sizeof(pduHdr));

    UART->S1 |= UART_S1_RDRF_MASK;
    UART->RCFIFO = pkt.size();

    // call to copy header
    EXPECT_CALL(mock, l1UARTReadNonBlocking(UART, testing::NotNull(), sizeof(L2Hdr)))
        .Times(1)
        .WillOnce(testing::Invoke([pkt](UART_Type* UART, uint8_t* data, size_t len) {
        std::memcpy(data, pkt.data(), len);
            }))
        .RetiresOnSaturation();

    EXPECT_CALL(mock, l1UARTReadNonBlocking(UART, testing::NotNull(), sizeof(PduHdr) - sizeof(L2Hdr)))
        .Times(1)
        .WillOnce(testing::Invoke([pkt](UART_Type* UART, uint8_t* data, size_t len) {
        std::memcpy(data, pkt.data() + sizeof(L2Hdr), len);
            }))
        .RetiresOnSaturation();

    l1TransferHandleIRQ(UART, port);

    UART->S1 &= ~UART_S1_RDRF_MASK;
    UART->RCFIFO = 0;

    PITCallback(port + L2_PIT_TIMER_START_IDX); // interchar silence

    PITCallback(port + L2_PIT_TIMER_START_IDX); // interframe silence
}

void sendPduMsg(MockUart &mock, UART_Type *UART, uint8_t &rxPgOfst, const uint8_t *msg,
                const int &origSize, const uint8_t &port)
{
    UART->S1 |= UART_S1_RDRF_MASK;
    int msgSize = origSize;
    UART->RCFIFO = sizeof(PduHdr);

    // call to copy header
    EXPECT_CALL(mock, l1UARTReadNonBlocking(UART, testing::NotNull(), sizeof(L2Hdr)))
        .Times(1)
        .WillOnce(testing::Invoke([msg](UART_Type *UART, uint8_t *data, size_t len)
                                  { std::memcpy(data, msg, len); }))
        .RetiresOnSaturation();

    EXPECT_CALL(mock, l1UARTReadNonBlocking(UART, testing::NotNull(), sizeof(PduHdr) - sizeof(L2Hdr)))
        .Times(1)
        .WillOnce(testing::Invoke([msg](UART_Type *UART, uint8_t *data, size_t len)
                                  { std::memcpy(data, msg + sizeof(L2Hdr), len); }))
        .RetiresOnSaturation();

    l1TransferHandleIRQ(UART, port);
    msgSize -= sizeof(PduHdr);

    while (msgSize)
    {
        uint8_t size = std::min(static_cast<int>(UNIT - rxPgOfst), msgSize);

        const uint8_t *msgAftHdr = msg + sizeof(PduHdr);

        EXPECT_CALL(mock, l1UARTReadNonBlocking(UART, testing::NotNull(), size))
            .Times(1)
            .WillOnce(testing::Invoke([msg, origSize, msgSize](const UART_Type *const UART, uint8_t *data, size_t len)
                                      { 
                            const uint8_t * msgAftHdr2 = msg + (origSize - msgSize);
                            std::memcpy(data, msg + (origSize - msgSize), len); }))
            .RetiresOnSaturation();

        UART->RCFIFO = size; // keep rx fifo full for now
        l1TransferHandleIRQ(UART, port);
        msgSize -= size;
    }

    UART->S1 &= ~UART_S1_RDRF_MASK;
    UART->RCFIFO = 0;

    PITCallback(port + L2_PIT_TIMER_START_IDX); // interchar silence

    PITCallback(port + L2_PIT_TIMER_START_IDX); // interframe silence
}

#define L2_FRAME_SIZE (RS485_FRAME_SIZE - sizeof(L2Hdr))
#define L3_FRAME_SIZE (L2_FRAME_SIZE - sizeof(L3Hdr))
#define L4_FRAME_SIZE (L3_FRAME_SIZE - sizeof(L4Hdr))

void expectHdr(MockUart& mock, UART_Type* UART, const PduHdr& pduHdr, const uint8_t& port, const bool& echo) {

    if (!echo) {
        EXPECT_CALL(mock, l1UARTWriteNonBlocking(UART, testing::NotNull(), sizeof(PduHdr)))
            .Times(1)
            .WillOnce(testing::Invoke([pduHdr](UART_Type* UART, const uint8_t* data, size_t len) {
            EXPECT_EQ(0, std::memcmp(data, (uint8_t*)&pduHdr, len));
                }))
            .RetiresOnSaturation();
    }
    else {
        EXPECT_CALL(mock, l1UARTCmpNonBlocking(UART, testing::NotNull(), sizeof(PduHdr)))
            .Times(1)
            .WillOnce(testing::Invoke([pduHdr](UART_Type* UART, const uint8_t* data, size_t len) {
            EXPECT_EQ(0, std::memcmp(data, (uint8_t*)&pduHdr, len));
            return true;
                }))
            .RetiresOnSaturation();

        UART->S1 |= UART_S1_RDRF_MASK;
        UART->RCFIFO = sizeof(PduHdr);

        l1TransferHandleIRQ(UART, port);

        UART->S1 &= ~UART_S1_RDRF_MASK;
        UART->RCFIFO = 0;
    }
}

void expectFrwdHdr(MockUart &mock, UART_Type *UART, const PduHdr &pduHdr, const uint8_t &port, const bool &echo)
{

    if (!echo)
    {
        EXPECT_CALL(mock, l1UARTWriteNonBlocking(UART, testing::NotNull(), sizeof(PduHdr) - sizeof(L4Hdr)))
            .Times(1)
            .WillOnce(testing::Invoke([pduHdr](UART_Type *UART, const uint8_t *data, size_t len)
                                      { EXPECT_EQ(0, std::memcmp(data, (uint8_t *)&pduHdr, len)); }))
            .RetiresOnSaturation();
    }
    else
    {
        EXPECT_CALL(mock, l1UARTCmpNonBlocking(UART, testing::NotNull(), sizeof(L2Hdr) + sizeof(L3Hdr)))
            .Times(1)
            .WillOnce(testing::Invoke([pduHdr](UART_Type *UART, const uint8_t *data, size_t len)
                                      {
            EXPECT_EQ(0, std::memcmp(data, (uint8_t*)&pduHdr, len));
            return true; }))
            .RetiresOnSaturation();

        UART->S1 |= UART_S1_RDRF_MASK;
        UART->RCFIFO = sizeof(L2Hdr) + sizeof(L3Hdr);

        l1TransferHandleIRQ(UART, port);

        UART->S1 &= ~UART_S1_RDRF_MASK;
        UART->RCFIFO = 0;
    }
}

void expectHdrMsg(MockUart &mock, UART_Type *UART, const PduHdr &pduHdr, const uint8_t &port)
{
    // confirm tx is primed
    ASSERT_TRUE(
        ((UART->C2 & (uint8_t)(UART_C2_TCIE_MASK)) == 0U) &&
        ((UART->C2 & (uint8_t)(UART_C2_TE_MASK)) == 0U) &&
        ((UART->C2 & (uint8_t)(UART_C2_TIE_MASK)) != 0U));

    expectHdr(mock, UART, pduHdr, port, false);

    UART->S1 = (uint8_t)((UART->S1 & (uint8_t)~UART_S1_TC_MASK) | UART_S1_TDRE_MASK);
    l1TransferHandleIRQ(UART, port); // complete TX of single frame

    // expect TC complete
    ASSERT_TRUE(
        ((UART->C2 & (uint8_t)(UART_C2_TCIE_MASK)) != 0U) &&
        ((UART->C2 & (uint8_t)(UART_C2_TE_MASK)) != 0U) &&
        ((UART->C2 & (uint8_t)(UART_C2_TIE_MASK)) == 0U));

    UART->S1 = (uint8_t)((UART->S1 & (uint8_t)~UART_S1_TDRE_MASK) | UART_S1_TC_MASK);
    l1TransferHandleIRQ(UART, port); // complete TX of single frame

    // confirm UART TX is disabled
    ASSERT_NE((UART->C2 & ((uint8_t)UART_C2_TIE_MASK | (uint8_t)UART_C2_TCIE_MASK | (uint8_t)UART_C2_TE_MASK)) != 0, true);

    expectHdr(mock, UART, pduHdr, port, true);
}

enum class TxMultiFrameType {
    TX_MULTI_FRAME_FIRST_MSG,
    TX_MULTI_FRAME_NEW_MSG,
    TX_MULTI_FRAME_NEW_FRAME,
    TX_MULTI_FRAME_CONT,
};

void expectTxMultiFrame(MockUart& mock,
    UART_Type* UART,
    const uint8_t& port,
    uint8_t& idxIn,
    uint8_t& sizeIn,
    const TxMultiFrameType& type,
    const uint8_t* msg,
    const int& msgSize,
    const PduHdr& hdr = PduHdr(),
    const bool& mstAftLstFrame = false,
    const uint32_t& milliSeconds = 0,
    const bool& echo = false) {
    uint8_t idx = idxIn;
    uint8_t size = sizeIn;

    if (type == TxMultiFrameType::TX_MULTI_FRAME_FIRST_MSG) {
        size = 0;
    }

    if (type == TxMultiFrameType::TX_MULTI_FRAME_NEW_MSG) {
        idx = 0; // zero the idx
    }

    if (type != TxMultiFrameType::TX_MULTI_FRAME_CONT) {
        if (!echo) {
            // confirm is primed
            ASSERT_TRUE(
                ((UART->C2 & (uint8_t)(UART_C2_TCIE_MASK)) == 0U) &&
                ((UART->C2 & (uint8_t)(UART_C2_TE_MASK)) == 0U) &&
                ((UART->C2 & (uint8_t)(UART_C2_TIE_MASK)) != 0U)
            );
        }
        // expect header
        expectHdr(mock, UART, hdr, port, echo);
    }

    size = UNIT - (size + (type == TxMultiFrameType::TX_MULTI_FRAME_NEW_MSG ? sizeof(L4Hdr::msgLen) + sizeof(txOrder) + sizeof(L4Hdr::msgFlgs) : 0));
    uint8_t len = UART_FIFO_SIZE - size - (type != TxMultiFrameType::TX_MULTI_FRAME_CONT ? sizeof(PduHdr) : 0);

    for (;;) {
        uint8_t idx_loc = idx;
        if (idx_loc + size > msgSize) {
            size = msgSize - idx_loc;
            len = 0;
        }

        if (!echo) {
            EXPECT_CALL(mock, l1UARTWriteNonBlocking(UART, testing::NotNull(), size))
                .Times(1)
                .WillOnce(testing::Invoke([msg, idx_loc](UART_Type* UART, const uint8_t* data, size_t len) {

                EXPECT_EQ(0, std::memcmp(data, (msg + idx_loc), len));
                    }))
                .RetiresOnSaturation();
        }
        else {
            // echo
            EXPECT_CALL(mock, l1UARTCmpNonBlocking(UART, testing::NotNull(), size))
                .Times(1)
                .WillOnce(testing::Invoke([msg, idx_loc](UART_Type* UART, const uint8_t* data, size_t len) {
                EXPECT_EQ(0, std::memcmp(data, (msg + idx_loc), len));
                return true;
                    }))
                .RetiresOnSaturation();

            if ((idx + size >= msgSize) && !(len) && (hdr.l4hdr.msgFlgs & L4_MSG_FLAG_REQ_ACK)) {
                EXPECT_CALL(mock, pitGetCurrMS())
                    .Times(1)
                    .WillOnce(testing::Return(milliSeconds))
                    .RetiresOnSaturation();
            }

            UART->S1 |= UART_S1_RDRF_MASK;
            UART->RCFIFO = size;

            l1TransferHandleIRQ(UART, port);

            UART->S1 &= ~UART_S1_RDRF_MASK;
            UART->RCFIFO = 0;
        }
        
        idx += size;

        if (len == 0) {
            break;
        }

        size = std::min((uint8_t)UNIT, len);
        if (idx + size > L4_FRAME_SIZE) {
            size = L4_FRAME_SIZE - idx;
            len = 0;        
        }
        else {
            len -= size;
        }
    }

    if (echo) {
        sizeIn = size;
        idxIn = idx;
        if (!(idx % L4_FRAME_SIZE) || (msgSize - idx) < UART_FIFO_SIZE) {

            // tx complete check to send next frame
            if ((msgSize - idx) > 0) {
                // inter frame silence
                PITCallback(port + L2_PIT_TIMER_START_IDX);

                PduHdr hdrCpy = hdr;
                if ((msgSize - idx) <= L4_FRAME_SIZE) {
                    hdrCpy.l2hdr.type |= L2_PKT_TYPE_MST;
                }
                hdrCpy.l4hdr.msgLen = (msgSize - idx) > L4_FRAME_SIZE? (msgSize - idx) - L4_FRAME_SIZE: 0;

                expectTxMultiFrame(mock,
                    UART,
                    port,
                    idxIn,
                    sizeIn,
                    TxMultiFrameType::TX_MULTI_FRAME_NEW_FRAME,
                    msg,
                    msgSize,
                    hdrCpy,
                    mstAftLstFrame,
                    milliSeconds);
            }
        } else {
            expectTxMultiFrame(mock,
                UART,
                port,
                idxIn,
                sizeIn,
                TxMultiFrameType::TX_MULTI_FRAME_CONT,
                msg,
                msgSize,
                hdr,
                mstAftLstFrame,
                milliSeconds);
        }
    }
    else {
        UART->S1 = (uint8_t)((UART->S1 & (uint8_t)~UART_S1_TC_MASK) | UART_S1_TDRE_MASK);
        l1TransferHandleIRQ(UART, port); // complete TX of single frame

        if (!(idx % L4_FRAME_SIZE) || (msgSize - idx) < UART_FIFO_SIZE) {
            // expect TC complete 
            ASSERT_TRUE(
                ((UART->C2 & (uint8_t)(UART_C2_TCIE_MASK)) != 0U) &&
                ((UART->C2 & (uint8_t)(UART_C2_TE_MASK)) != 0U) &&
                ((UART->C2 & (uint8_t)(UART_C2_TIE_MASK)) == 0U)
            );

            UART->S1 = (uint8_t)((UART->S1 & (uint8_t)~UART_S1_TDRE_MASK) | UART_S1_TC_MASK);
            l1TransferHandleIRQ(UART, port); // complete TX of single frame

            // confirm UART TX is disabled
            ASSERT_NE((UART->C2 & ((uint8_t)UART_C2_TIE_MASK | (uint8_t)UART_C2_TCIE_MASK | (uint8_t)UART_C2_TE_MASK)) != 0, true);
        } else {
            // confirm TE is still primed
            ASSERT_TRUE(
                ((UART->C2 & (uint8_t)(UART_C2_TCIE_MASK)) == 0U) &&
                ((UART->C2 & (uint8_t)(UART_C2_TE_MASK)) != 0U) &&
                ((UART->C2 & (uint8_t)(UART_C2_TIE_MASK)) != 0U)
            );
            UART->S1 &= ~UART_S1_TDRE_MASK; // wait for echo to be processed
        }
        
        expectTxMultiFrame(mock,
            UART,
            port,
            idxIn,
            sizeIn,
            type,
            msg,
            msgSize,
            hdr,
            mstAftLstFrame,
            milliSeconds,
            true);
    }
}

void expectFrwdFrame(MockUart &mock,
                        UART_Type *UART,
                        const uint8_t &port,
                        uint8_t &idxIn,
                        uint8_t &sizeIn,
                        const TxMultiFrameType &type,
                        const uint8_t *msg,
                        const int &msgSize,
                        const PduHdr &hdr = PduHdr(),
                        const bool &mstAftLstFrame = false,
                        const uint32_t &milliSeconds = 0,
                        const bool &echo = false)
{
    uint8_t idx = idxIn;
    uint8_t size = sizeIn;

    if (type == TxMultiFrameType::TX_MULTI_FRAME_FIRST_MSG)
    {
        size = 0;
        idx = 0;
    }

    if (type == TxMultiFrameType::TX_MULTI_FRAME_NEW_MSG)
    {
        idx = 0; // zero the idx
    }

    if (type != TxMultiFrameType::TX_MULTI_FRAME_CONT)
    {
        if (!echo)
        {
            // confirm is primed
            ASSERT_TRUE(
                ((UART->C2 & (uint8_t)(UART_C2_TCIE_MASK)) == 0U) &&
                ((UART->C2 & (uint8_t)(UART_C2_TE_MASK)) == 0U) &&
                ((UART->C2 & (uint8_t)(UART_C2_TIE_MASK)) != 0U));
        }
       // expect header
        expectHdr(mock, UART, hdr, port, echo);
    }

    size = UNIT - (size /*+ (type == TxMultiFrameType::TX_MULTI_FRAME_NEW_MSG ? sizeof(L4Hdr::msgLen) + sizeof(txOrder) + sizeof(L4Hdr::msgFlgs) : 0)*/);
    uint8_t len = (msgSize > size)? msgSize - size: msgSize; /*- (type != TxMultiFrameType::TX_MULTI_FRAME_CONT ? (sizeof(PduHdr)) : 0)*/;

    for (;;)
    {
        uint8_t idx_loc = idx;
        if (idx_loc + size > msgSize)
        {
            size = msgSize - idx_loc;
            len = 0;
        }

        if (!echo)
        {
            EXPECT_CALL(mock, l1UARTWriteNonBlocking(UART, testing::NotNull(), size))
                .Times(1)
                .WillOnce(testing::Invoke([msg, idx_loc](UART_Type *UART, const uint8_t *data, size_t len)
                                          { EXPECT_EQ(0, std::memcmp(data, (msg + idx_loc), len)); }))
                .RetiresOnSaturation();
        }
        else
        {
            // echo
            EXPECT_CALL(mock, l1UARTCmpNonBlocking(UART, testing::NotNull(), size))
                .Times(1)
                .WillOnce(testing::Invoke([msg, idx_loc](UART_Type *UART, const uint8_t *data, size_t len)
                                          {
                EXPECT_EQ(0, std::memcmp(data, (msg + idx_loc), len));
                return true; }))
                .RetiresOnSaturation();


            UART->S1 |= UART_S1_RDRF_MASK;
            UART->RCFIFO = size;

            l1TransferHandleIRQ(UART, port);

            UART->S1 &= ~UART_S1_RDRF_MASK;
            UART->RCFIFO = 0;
        }

        idx += size;

        if (len == 0)
        {
            break;
        }

        size = std::min((uint8_t)UNIT, len);
        if (idx + size > L4_FRAME_SIZE)
        {
            size = L4_FRAME_SIZE - idx;
            len = 0;
        }
        else
        {
            len -= size;
        }
    }

    if (echo)
    {
        sizeIn = size;
        idxIn = idx;
        if (!(idx % L4_FRAME_SIZE) || (msgSize - idx) < UART_FIFO_SIZE)
        {

            // tx complete check to send next frame
            if ((msgSize - idx) > 0)
            {
                // inter frame silence
                PITCallback(port + L2_PIT_TIMER_START_IDX);

                PduHdr hdrCpy = hdr;
                if ((msgSize - idx) <= L4_FRAME_SIZE)
                {
                    hdrCpy.l2hdr.type |= L2_PKT_TYPE_MST;
                }
                hdrCpy.l4hdr.msgLen = (msgSize - idx) > L4_FRAME_SIZE ? (msgSize - idx) - L4_FRAME_SIZE : 0;

                expectFrwdFrame(mock,
                                UART,
                                port,
                                idxIn,
                                sizeIn,
                                TxMultiFrameType::TX_MULTI_FRAME_NEW_FRAME,
                                msg,
                                msgSize,
                                hdrCpy,
                                mstAftLstFrame,
                                milliSeconds);
            }
        }
        else
        {
            expectFrwdFrame(mock,
                            UART,
                            port,
                            idxIn,
                            sizeIn,
                            TxMultiFrameType::TX_MULTI_FRAME_CONT,
                            msg,
                            msgSize,
                            hdr,
                            mstAftLstFrame,
                            milliSeconds);
        }
    }
    else
    {
        UART->S1 = (uint8_t)((UART->S1 & (uint8_t)~UART_S1_TC_MASK) | UART_S1_TDRE_MASK);
        l1TransferHandleIRQ(UART, port); // complete TX of single frame

        if (!(idx % L4_FRAME_SIZE) || (msgSize - idx) < UART_FIFO_SIZE)
        {
            // expect TC complete
            ASSERT_TRUE(
                ((UART->C2 & (uint8_t)(UART_C2_TCIE_MASK)) != 0U) &&
                ((UART->C2 & (uint8_t)(UART_C2_TE_MASK)) != 0U) &&
                ((UART->C2 & (uint8_t)(UART_C2_TIE_MASK)) == 0U));

            UART->S1 = (uint8_t)((UART->S1 & (uint8_t)~UART_S1_TDRE_MASK) | UART_S1_TC_MASK);
            l1TransferHandleIRQ(UART, port); // complete TX of single frame

            // confirm UART TX is disabled
            ASSERT_NE((UART->C2 & ((uint8_t)UART_C2_TIE_MASK | (uint8_t)UART_C2_TCIE_MASK | (uint8_t)UART_C2_TE_MASK)) != 0, true);
        }
        else
        {
            // confirm TE is still primed
            ASSERT_TRUE(
                ((UART->C2 & (uint8_t)(UART_C2_TCIE_MASK)) == 0U) &&
                ((UART->C2 & (uint8_t)(UART_C2_TE_MASK)) != 0U) &&
                ((UART->C2 & (uint8_t)(UART_C2_TIE_MASK)) != 0U));
            UART->S1 &= ~UART_S1_TDRE_MASK; // wait for echo to be processed
        }

        expectFrwdFrame(mock,
                        UART,
                        port,
                        idxIn,
                        sizeIn,
                        type,
                        msg,
                        msgSize,
                        hdr,
                        mstAftLstFrame,
                        milliSeconds,
                        true);
    }
}

//#if 0
TEST_P(MultiHop, addr) {
    // check correct addr table
    ASSERT_EQ(0, std::memcmp(GetParam().l3AddrTblPrio.data(), l3AddrTblPrio, sizeof(l3AddrTblPrio)));

    // check route table
    ASSERT_EQ(0, std::memcmp(GetParam().l3RouteTable.data(), l3RouteTable, sizeof(l3RouteTable)));

    // check correct brdcst table
    ASSERT_EQ(0, std::memcmp(GetParam().l3BcastInSubnetForSrcPort.data(), l3BcastInSubnetForSrcPort, sizeof(l3BcastInSubnetForSrcPort)));
}
//#endif


//#if 0
TEST_P(MultiHop, mstPassFail) {

    MockUart mock;
    g_mock = &mock;

    for (int port = 0; port < MAX_PORT; port++) {
        // confirm UART TX is disabled
        ASSERT_NE((uart_objs[port].C2 & ((uint8_t)UART_C2_TIE_MASK | (uint8_t)UART_C2_TCIE_MASK | (uint8_t)UART_C2_TE_MASK)) != 0, true);

        // confirm RX is enabled
        ASSERT_EQ((uart_objs[port].C2 & (UART_C2_RE_MASK | UART_C2_RIE_MASK)) != 0, true);

        // fire MST timeout timer
        PITCallback(port + L2_PIT_TIMER_START_IDX);
        uint8_t l2Addr = GetParam().l3AddrTblPrio[GetParam().pos][port] & 0x00FF;

        // confirm TX TIE 
        if (l2Addr > 0) {
            // fire TDRE ISR
            uint8_t nxtMst = l2Addr;
            while (true) {
                ASSERT_EQ((uart_objs[port].C2 & ((uint8_t)UART_C2_TIE_MASK)) != 0, true);
                nxtMst = (nxtMst + 1) > GetParam().devCnt[port] ? 1 : nxtMst + 1;

                bool rollover = false;
                if (nxtMst == l2Addr) // rollover test done for port
                {
                    rollover = true;
                    nxtMst = (nxtMst + 1) > GetParam().devCnt[port] ? 1 : nxtMst + 1;
                }

                expectMst(mock, nxtMst, &uart_objs[port], port);

                if (rollover) {
                    break;
                }

                PITCallback(port + L2_PIT_TIMER_START_IDX);
            }
        }
        else {
            ASSERT_NE((uart_objs[port].C2 & ((uint8_t)UART_C2_TIE_MASK)) != 0, true);
        }
    }
}
//#endif

//#if 0
TEST_P(MultiHop, mstPassMsg) {
    MockUart mock;
    g_mock = &mock;

    for (int port = 0; port < MAX_PORT; port++) {
        // confirm tx is disabled
        ASSERT_NE((uart_objs[port].C2 & ((uint8_t)UART_C2_TIE_MASK | (uint8_t)UART_C2_TCIE_MASK | (uint8_t)UART_C2_TE_MASK)) != 0, true);
        uint8_t l2Addr = GetParam().l3AddrTblPrio[GetParam().pos][port] & 0x00FF;
        if (l2Addr > 0) {
            ASSERT_EQ((uart_objs[port].C2 & (UART_C2_RE_MASK | UART_C2_RIE_MASK)) != 0, true); // confirm RX is enabled
            sendMstToken(mock, uart_ptrs[port], l2Addr, port);

            // since there is no message should pass token immediatly
            PITCallback(port + L2_PIT_TIMER_START_IDX); // inter char silence
            PITCallback(port + L2_PIT_TIMER_START_IDX); // inter frame - inter char silence

            ASSERT_EQ((uart_objs[port].C2 & ((uint8_t)UART_C2_TIE_MASK)) != 0, true);
            uint8_t nxtMst = (l2Addr + 1) > GetParam().devCnt[port] ? 1 : l2Addr + 1;
            expectMst(mock, nxtMst, &uart_objs[port], port);
        }
        else {
            // TODO Disabled port
            //ASSERT_NE((uart_objs[port].C2 & (UART_C2_RE_MASK | UART_C2_RIE_MASK)) != 0, true); // confirm RX is disabled
        }
    }
    // TODO Fails
}
//#endif

//#if 0
TEST_P(MultiHop, pduNoHopSingleFrameNoRetry) {

    if (GetParam().pos != 7) {
        const auto* info = ::testing::UnitTest::GetInstance()->current_test_info();
        GTEST_SKIP() << "pos " << GetParam().pos << " test " << info->name() << " TODO";
    }

    MockUart mock;
    g_mock = &mock;
    testing::InSequence seq;
    // construct message
    static const int MSG_SIZE = 100;
    std::array<uint8_t, MSG_SIZE> msg;
    for (int i = 0; i < MSG_SIZE; i++) {
        msg[i] = i;
    }

    static const int MSG2_SIZE = L4_FRAME_SIZE+1;
    std::array<uint8_t, MSG2_SIZE> msg2;
    for (int i = 0; i < MSG2_SIZE; i++) { // send multiframe msg
        msg2[i] = i;
    }
    uint8_t size;
    PduHdr pduHdr{};

    uint8_t idx[MAX_PORT];

    for (int port = 0; port < MAX_PORT; port++) {

        // confirm UART TX is disabled
        ASSERT_NE((uart_objs[port].C2 & ((uint8_t)UART_C2_TIE_MASK | (uint8_t)UART_C2_TCIE_MASK | (uint8_t)UART_C2_TE_MASK)) != 0, true);
        uint16_t l3Addr = GetParam().l3AddrTblPrio[GetParam().pos][port];
        uint8_t l2Addr = (l3Addr & 0x00FF);
        if (l2Addr == 0 /* ||
               portsTested[port]*/) {
            continue;
        }

        // confirm RX is enabled
        ASSERT_EQ((uart_objs[port].C2 & (UART_C2_RE_MASK | UART_C2_RIE_MASK)) != 0, true);

        // send msg
        appSend(msg.data(), MSG_SIZE, 1, 0, false);
        appSend(msg2.data(), MSG2_SIZE, 1, 0, false);

        // send MST
        sendMstToken(mock, uart_ptrs[port], l2Addr, port);

        // will send message

        // first expect hdr
        pduHdr = PduHdr{
            .l2hdr = { 0x1, L2_PKT_TYPE_PDU, 0xFF },
            .l3hdr = { l3Addr, 0x101, 1, 0 },
            .l4hdr = { 0, 0, 0 }
        };

        expectTxMultiFrame(mock,
            uart_ptrs[port],
            port,
            idx[port],
            size,
            TxMultiFrameType::TX_MULTI_FRAME_FIRST_MSG,
            msg.data(),
            msg.size(),
            pduHdr);

        // inter frame silence
        PITCallback(port + L2_PIT_TIMER_START_IDX);

        pduHdr.l4hdr = L4Hdr{ 1, 0, 1 };

        // expect msg size single frame
        expectTxMultiFrame(mock,
            uart_ptrs[port],
            port,
            idx[port],
            size,
            TxMultiFrameType::TX_MULTI_FRAME_NEW_MSG,
            msg2.data(),
            msg2.size(),
            pduHdr,
            true);
    }
}
//#endif

#define MAX_L4_RETRY 3
#define L4_RETRY_TIMER 100 // 100 ms

//#if 0
TEST_P(MultiHop, pduNoHopSingleFrameRetryNoAck) {
    if (GetParam().pos != 7) {
        const auto* info = ::testing::UnitTest::GetInstance()->current_test_info();
        GTEST_SKIP() << "pos " << GetParam().pos << " test " << info->name() << " TODO";
    }

    MockUart mock;
    g_mock = &mock;
    testing::InSequence seq;
    // construct message
    static const int MSG_SIZE = 100;
    std::array<uint8_t, MSG_SIZE> msg;
    for (int i = 0; i < MSG_SIZE; i++) {
        msg[i] = i;
    }

    static const int MSG2_SIZE = L4_FRAME_SIZE + 1;
    std::array<uint8_t, MSG2_SIZE> msg2;
    for (int i = 0; i < MSG2_SIZE; i++) { // send multiframe msg
        msg2[i] = i;
    }
    uint8_t size;
    PduHdr pduHdr{};

    uint8_t idx[MAX_PORT];
    uint32_t milliSeconds = 0;

    for (int port = 0; port < MAX_PORT; port++) {

        // confirm UART TX is disabled
        ASSERT_NE((uart_objs[port].C2 & ((uint8_t)UART_C2_TIE_MASK | (uint8_t)UART_C2_TCIE_MASK | (uint8_t)UART_C2_TE_MASK)) != 0, true);

        uint16_t l3Addr = GetParam().l3AddrTblPrio[GetParam().pos][port];
        uint8_t l2Addr = (l3Addr & 0x00FF);
        if (l2Addr == 0 /* ||
               portsTested[port]*/) {
            continue;
        }

        // confirm RX is enabled
        ASSERT_EQ((uart_objs[port].C2 & (UART_C2_RE_MASK | UART_C2_RIE_MASK)) != 0, true);

        // send msg
        appSend(msg.data(), MSG_SIZE, 1, 0, true);
        appSend(msg2.data(), MSG2_SIZE, 1, 0, true);

        // send MST
        sendMstToken(mock, uart_ptrs[port], l2Addr, port);

        // will send message

        // first expect hdr
        pduHdr = PduHdr{
            .l2hdr = { 0x1, (L2_PKT_TYPE_PDU | L2_PKT_TYPE_MST), 0xFF },
            .l3hdr = { l3Addr, 0x101, 1, 0 },
            .l4hdr = { 0, L4_MSG_FLAG_REQ_ACK, 0 }
        };

        uint8_t sizePrev = size;
        uint8_t idxPrev = idx[port];

        for (int retry = 0; retry <= MAX_L4_RETRY; retry++) {
            expectTxMultiFrame(mock,
                uart_ptrs[port],
                port,
                idx[port],
                size,
                TxMultiFrameType::TX_MULTI_FRAME_FIRST_MSG,
                msg.data(),
                msg.size(),
                pduHdr,
                true,
                milliSeconds);

            // mst timeout silence
            milliSeconds += L4_RETRY_TIMER + 1;

            EXPECT_CALL(mock, pitGetCurrMS())
                .Times(1)
                .WillOnce(testing::Return(milliSeconds))
                .RetiresOnSaturation();

            PITCallback(port + L2_PIT_TIMER_START_IDX);

            if (retry < MAX_L4_RETRY) {
                size = sizePrev;
                idx[port] = idxPrev;
            }
        }
        sizePrev = size;
        idxPrev = idx[port];
        pduHdr.l2hdr.type &= ~L2_PKT_TYPE_MST; // will be set on last frame of message
        pduHdr.l4hdr = L4Hdr{ 1, L4_MSG_FLAG_REQ_ACK, 1 };
        // message is dropped and move to next message
        for (int retry = 0; retry <= MAX_L4_RETRY; retry++) {
            // expect msg size single frame
            expectTxMultiFrame(mock,
                uart_ptrs[port],
                port,
                idx[port],
                size,
                TxMultiFrameType::TX_MULTI_FRAME_NEW_MSG,
                msg2.data(),
                msg2.size(),
                pduHdr,
                true,
                milliSeconds);

            // mst timeout silence
            milliSeconds += L4_RETRY_TIMER + 1;

            EXPECT_CALL(mock, pitGetCurrMS())
                .Times(1)
                .WillOnce(testing::Return(milliSeconds))
                .RetiresOnSaturation();

            PITCallback(port + L2_PIT_TIMER_START_IDX); // mst timeout

            size = sizePrev;
            idx[port] = idxPrev;
        }
    }
}
//#endif

//#if 0
TEST_P(MultiHop, pduNoHopAck)
{
    if (GetParam().pos != 7) {
        const auto* info = ::testing::UnitTest::GetInstance()->current_test_info();
        GTEST_SKIP() << "pos " << GetParam().pos << " test " << info->name() << " TODO";
    }

    MockUart mock;
    g_mock = &mock;
    testing::InSequence seq;
    // construct message
    static const int MSG_SIZE = 100;
    std::array<uint8_t, MSG_SIZE> msg;
    for (int i = 0; i < MSG_SIZE; i++)
    {
        msg[i] = i;
    }

    static const int MSG2_SIZE = L4_FRAME_SIZE + 1;
    std::array<uint8_t, MSG2_SIZE> msg2;
    for (int i = 0; i < MSG2_SIZE; i++)
    { // send multiframe msg
        msg2[i] = i;
    }
    uint8_t size;
    PduHdr pduHdr{};

    uint8_t idx[MAX_PORT];
    uint32_t milliSeconds = 0;

    for (int port = 0; port < MAX_PORT; port++)
    {

        // confirm UART TX is disabled
        ASSERT_NE((uart_objs[port].C2 & ((uint8_t)UART_C2_TIE_MASK | (uint8_t)UART_C2_TCIE_MASK | (uint8_t)UART_C2_TE_MASK)) != 0, true);

        uint16_t l3Addr = GetParam().l3AddrTblPrio[GetParam().pos][port];
        uint8_t l2Addr = (l3Addr & 0x00FF);
        if (l2Addr == 0)
        {
            continue;
        }

        // confirm RX is enabled
        ASSERT_EQ((uart_objs[port].C2 & (UART_C2_RE_MASK | UART_C2_RIE_MASK)) != 0, true);

        // send msg
        appSend(msg.data(), MSG_SIZE, 1, 0, true);
        appSend(msg2.data(), MSG2_SIZE, 1, 0, true);

        // send MST
        sendMstToken(mock, uart_ptrs[port], l2Addr, port);

        // will send message

        // first expect hdr
        pduHdr = PduHdr{
            .l2hdr = {0x1, (L2_PKT_TYPE_PDU | L2_PKT_TYPE_MST), 0xFF},
            .l3hdr = {l3Addr, 0x101, 1, 0},
            .l4hdr = {0, L4_MSG_FLAG_REQ_ACK, 0}};

        uint8_t sizePrev = size;
        uint8_t idxPrev = idx[port];

        expectTxMultiFrame(mock,
                           uart_ptrs[port],
                           port,
                           idx[port],
                           size,
                           TxMultiFrameType::TX_MULTI_FRAME_FIRST_MSG,
                           msg.data(),
                           msg.size(),
                           pduHdr,
                           true,
                           milliSeconds);

        PduHdr pduAck = PduHdr{
            .l2hdr = {0x3, (L2_PKT_TYPE_PDU | L2_PKT_TYPE_MST), 0xFF},
            .l3hdr = {0x101, l3Addr, 1, 0},
            .l4hdr = {0, L4_MSG_FLAG_TYPE_ACK, 0}};

        // send ack
        sendMsgAck(mock, uart_ptrs[port], pduAck, port);

        pduHdr.l2hdr.type &= ~L2_PKT_TYPE_MST; // will be set on last frame of message
        pduHdr.l4hdr = L4Hdr{1, L4_MSG_FLAG_REQ_ACK, 1};
        // message is dropped and move to next message

        // expect msg size single frame
        expectTxMultiFrame(mock,
                           uart_ptrs[port],
                           port,
                           idx[port],
                           size,
                           TxMultiFrameType::TX_MULTI_FRAME_NEW_MSG,
                           msg2.data(),
                           msg2.size(),
                           pduHdr,
                           true,
                           milliSeconds);

        pduAck.l4hdr.msgNo++;
        sendMsgAck(mock, uart_ptrs[port], pduAck, port);
    }
}
//endif

//#if 0
TEST_P(MultiHop, pduNoHopRx)
{
    if (GetParam().pos != 7) {
        const auto* info = ::testing::UnitTest::GetInstance()->current_test_info();
        GTEST_SKIP() << "pos " << GetParam().pos << " test " << info->name() << " TODO";
    }

    MockUart mock;
    g_mock = &mock;
    testing::InSequence seq;
    // construct message
    static const int MSG_SIZE = 100;
    std::array<uint8_t, MSG_SIZE> msg;
    for (int i = 0; i < MSG_SIZE; i++)
    {
        msg[i] = i;
    }

    static const int MSG2_SIZE = L4_FRAME_SIZE + 1;
    std::array<uint8_t, MSG2_SIZE> msg2;
    for (int i = 0; i < MSG2_SIZE; i++)
    { // send multiframe msg
        msg2[i] = i;
    }
    uint8_t size;
    PduHdr pduHdr{};

    uint8_t idx[MAX_PORT];
    uint32_t milliSeconds = 0;

    for (int port = 0; port < MAX_PORT; port++)
    {

        // confirm UART TX is disabled
        ASSERT_NE((uart_objs[port].C2 & ((uint8_t)UART_C2_TIE_MASK | (uint8_t)UART_C2_TCIE_MASK | (uint8_t)UART_C2_TE_MASK)) != 0, true);

        uint16_t l3Addr = GetParam().l3AddrTblPrio[GetParam().pos][port];
        uint8_t l2Addr = (l3Addr & 0x00FF);
        if (l2Addr == 0 /* ||
               portsTested[port]*/
        )
        {
            continue;
        }

        // confirm RX is enabled
        ASSERT_EQ((uart_objs[port].C2 & (UART_C2_RE_MASK | UART_C2_RIE_MASK)) != 0, true);

        uint8_t rxPgOfst = 0;

        PduHdr pduHdr = PduHdr{
            .l2hdr = {0x3, (L2_PKT_TYPE_PDU), 0xFF},
            .l3hdr = {0x101, l3Addr, 1, 0},
            .l4hdr = {0, 0, 0}};

        memcpy(msg.data(), (uint8_t *)&pduHdr, sizeof(PduHdr));

        // send msg
        sendPduMsg(mock, &uart_objs[port], rxPgOfst, msg.data(),
                   msg.size(), port);
    }
}
//#endif

//#if 0
TEST_P(MultiHop, pduNoHopRxAck)
{
    if (GetParam().pos != 7) {
        const auto* info = ::testing::UnitTest::GetInstance()->current_test_info();
        GTEST_SKIP() << "pos " << GetParam().pos << " test " << info->name() << " TODO";
    }

    MockUart mock;
    g_mock = &mock;
    testing::InSequence seq;
    // construct message
    static const int MSG_SIZE = 100;
    std::array<uint8_t, MSG_SIZE> msg;
    for (int i = 0; i < MSG_SIZE; i++)
    {
        msg[i] = i;
    }

    static const int MSG2_SIZE = L4_FRAME_SIZE + 1;
    std::array<uint8_t, MSG2_SIZE> msg2;
    for (int i = 0; i < MSG2_SIZE; i++)
    { // send multiframe msg
        msg2[i] = i;
    }
    uint8_t size;
    PduHdr pduHdr{};

    uint8_t idx[MAX_PORT];
    uint32_t milliSeconds = 0;

    for (int port = 0; port < MAX_PORT; port++)
    {

        // confirm UART TX is disabled
        ASSERT_NE((uart_objs[port].C2 & ((uint8_t)UART_C2_TIE_MASK | (uint8_t)UART_C2_TCIE_MASK | (uint8_t)UART_C2_TE_MASK)) != 0, true);

        uint16_t l3Addr = GetParam().l3AddrTblPrio[GetParam().pos][port];
        uint8_t l2Addr = (l3Addr & 0x00FF);
        if (l2Addr == 0)
        {
            continue;
        }

        // confirm RX is enabled
        ASSERT_EQ((uart_objs[port].C2 & (UART_C2_RE_MASK | UART_C2_RIE_MASK)) != 0, true);

        uint8_t rxPgOfst = 0;

        PduHdr pduHdr = PduHdr{
            .l2hdr = {0x3, (L2_PKT_TYPE_PDU | L2_PKT_TYPE_MST), 0xFF},
            .l3hdr = {0x101, l3Addr, 1, 0},
            .l4hdr = {0, L4_MSG_FLAG_REQ_ACK, 0}};

        memcpy(msg.data(), (uint8_t *)&pduHdr, sizeof(PduHdr));

        // send msg
        sendPduMsg(mock, &uart_objs[port], rxPgOfst, msg.data(),
                   msg.size(), port);

        PduHdr pduAck = PduHdr{
            .l2hdr = {0x1, (L2_PKT_TYPE_PDU | L2_PKT_TYPE_MST), 0xFF},
            .l3hdr = {l3Addr, 0x101, 1, 0},
            .l4hdr = {0, L4_MSG_FLAG_TYPE_ACK, 0}};

        expectHdrMsg(mock, &uart_objs[port], pduAck, port);
    }
}
//#endif

//#if 0
TEST_P(MultiHop, pduHopFrwd)
{
    MockUart mock;
    g_mock = &mock;
    testing::InSequence seq;
    // construct message
    static const int MSG_SIZE = 100;
    std::array<uint8_t, MSG_SIZE> msg;
    for (int i = 0; i < MSG_SIZE; i++)
    {
        msg[i] = i;
    }

    PduHdr pduHdr{};

    uint8_t idx[MAX_PORT];

    for (int port = 0; port < MAX_PORT; port++)
    {
        // confirm UART TX is disabled
        ASSERT_NE((uart_objs[port].C2 & ((uint8_t)UART_C2_TIE_MASK | (uint8_t)UART_C2_TCIE_MASK | (uint8_t)UART_C2_TE_MASK)) != 0, true);

        uint16_t l3Addr = GetParam().l3AddrTblPrio[GetParam().pos][port];
        uint8_t l2Addr = (l3Addr & 0x00FF);
        if (l2Addr == 0 || GetParam().devCnt[port] <= 1) // TODO what if in debug if we keep addr
        {
            continue;
        }

        ASSERT_EQ((uart_objs[port].C2 & (UART_C2_RE_MASK | UART_C2_RIE_MASK)) != 0, true);

        uint16_t l3SrcAddr = 0;

        for (int pos = 0; pos < MAX_POS; pos++)
        {
            // find a dst dev on port to forward
            if (pos == GetParam().pos)
            {
                continue;
            }
            
            for (int port3 = 0; port3 < MAX_PORT; port3++) {
                uint8_t subnet = GetParam().l3AddrTblPrio[pos][port3] >> 8;
                if (subnet == (l3Addr >> 8))
                {
                    l3SrcAddr = GetParam().l3AddrTblPrio[pos][port3];
                    break;
                }
            }

            if (l3SrcAddr) {
                break;
            }
        }

        ASSERT_NE(l3SrcAddr, 0);

        /* Packet arriving in this port should forward to other port */
        for (int port2 = 0; port2 < MAX_PORT; port2++)
        {
            if (port == port2) {
                continue;
            }

            uint16_t l3Addr2 = GetParam().l3AddrTblPrio[GetParam().pos][port2];
            uint8_t l2Addr2 = (l3Addr2 & 0x00FF);

            if (l2Addr2 == 0 || GetParam().devCnt[port] <= 1) // TODO what if in debug if we keep addr
            {
                continue;
            }

            // confirm RX is enabled
            ASSERT_EQ((uart_objs[port2].C2 & (UART_C2_RE_MASK | UART_C2_RIE_MASK)) != 0, true);

            uint16_t l3DstAddr = 0;
            
            for (int pos = 0; pos < MAX_POS; pos++) {
                // find a dst dev on port to forward
                if (pos == GetParam().pos) {
                    continue;
                }
                for (int port3 = 0; port3 < MAX_PORT; port3++) {
                    uint8_t subnet = GetParam().l3AddrTblPrio[pos][port3] >> 8;
                    if (subnet == (l3Addr2 >> 8))
                    {
                        l3DstAddr = GetParam().l3AddrTblPrio[pos][port3];
                        break;
                    }
                }

                if (l3DstAddr) {
                    break;
                }
            }

            ASSERT_NE(l3DstAddr, 0);

            uint8_t rxPgOfst = 0; // page will have L4 Hdr

            PduHdr pduHdr = PduHdr{
                .l2hdr = {l2Addr, (L2_PKT_TYPE_PDU), 0xFF},
                .l3hdr = {l3SrcAddr, l3DstAddr, 1, 0},
                .l4hdr = {0, 0, 0}};

            memcpy(msg.data(), (uint8_t *)&pduHdr, sizeof(PduHdr));

            // send msg to port 1
            sendPduMsg(mock, &uart_objs[port], rxPgOfst, msg.data(),
                       msg.size(), port);

            // send MST to port 2
            sendMstToken(mock, uart_ptrs[port2], l2Addr2, port2);

            // will send message

            // first expect hdr
            pduHdr = PduHdr{
                .l2hdr = {l3DstAddr, (L2_PKT_TYPE_PDU | L2_PKT_TYPE_MST), 0xFF},
                .l3hdr = {l3SrcAddr, l3DstAddr, 1, 0},
                .l4hdr = {0, 0, 0}};

            uint8_t size;

            expectFrwdFrame(mock,
                            uart_ptrs[port2],
                            port2,
                            idx[port2],
                            size,
                            TxMultiFrameType::TX_MULTI_FRAME_FIRST_MSG,
                            (msg.data() + (sizeof(PduHdr))),
                            (msg.size() - (sizeof(PduHdr))), /* L4 hdr is part of page buffer */
                            pduHdr);

            // check for a gateway in port2 ...
            uint8_t port2Subnet = l3Addr2 >> 8;
            for (int subnet = 0; subnet < MAX_SUBNET; subnet++) {
                uint8_t gatewaySubnet = GetParam().l3RouteTable[subnet] >> 8;

                if (gatewaySubnet == port2Subnet)
                {
                    // find a dst addr to this dist subnet
                    l3DstAddr = (subnet << 8) | 1; // choose a dst address to this distant subnet
                    ASSERT_NE(l3DstAddr, 0);

                    uint8_t rxPgOfst = 0; // page will have L4 Hdr

                    PduHdr pduHdr = PduHdr{
                        .l2hdr = {l2Addr, (L2_PKT_TYPE_PDU), 0xFF},
                        .l3hdr = {l3SrcAddr, l3DstAddr, 1, 0},
                        .l4hdr = {0, 0, 0}};

                    memcpy(msg.data(), (uint8_t *)&pduHdr, sizeof(PduHdr));

                    // send msg to port 1
                    sendPduMsg(mock, &uart_objs[port], rxPgOfst, msg.data(),
                               msg.size(), port);

                    // send MST to port 2
                    sendMstToken(mock, uart_ptrs[port2], l2Addr2, port2);

                    // will send message

                    // first expect hdr
                    pduHdr = PduHdr{
                        .l2hdr = {GetParam().l3RouteTable[subnet], (L2_PKT_TYPE_PDU | L2_PKT_TYPE_MST), 0xFF},
                        .l3hdr = {l3SrcAddr, l3DstAddr, 1, 0},
                        .l4hdr = {0, 0, 0}};

                    uint8_t size;

                    expectFrwdFrame(mock,
                                    uart_ptrs[port2],
                                    port2,
                                    idx[port2],
                                    size,
                                    TxMultiFrameType::TX_MULTI_FRAME_FIRST_MSG,
                                    (msg.data() + (sizeof(PduHdr))),
                                    (msg.size() - (sizeof(PduHdr))), /* L4 hdr is part of page buffer */
                                    pduHdr);
                }
            }
        }
    }
}
//#endif

//#if 0
TEST_P(MultiHop, pduHopFrwdBrdCst)
{
    MockUart mock;
    g_mock = &mock;
    testing::InSequence seq;
    // construct message
    static const int MSG_SIZE = 100;
    std::array<uint8_t, MSG_SIZE> msg;
    for (int i = 0; i < MSG_SIZE; i++)
    {
        msg[i] = i;
    }

    PduHdr pduHdr{};

    uint8_t idx[MAX_PORT];

    for (int port = 0; port < MAX_PORT; port++)
    {
        // confirm UART TX is disabled
        ASSERT_NE((uart_objs[port].C2 & ((uint8_t)UART_C2_TIE_MASK | (uint8_t)UART_C2_TCIE_MASK | (uint8_t)UART_C2_TE_MASK)) != 0, true);

        uint16_t l3Addr = GetParam().l3AddrTblPrio[GetParam().pos][port];
        uint8_t portSubnet = l3Addr >> 8;
        uint8_t l2Addr = (l3Addr & 0x00FF);
        if (l2Addr == 0 || GetParam().devCnt[port] <= 1) // TODO what if in debug if we keep addr
        {
            continue;
        }

        ASSERT_EQ((uart_objs[port].C2 & (UART_C2_RE_MASK | UART_C2_RIE_MASK)) != 0, true);


        /* Packet arriving in this port should forward to other port */
        for (int port2 = 0; port2 < MAX_PORT; port2++)
        {
            if (port == port2)
            {
                continue;
            }

            uint16_t l3Addr2 = GetParam().l3AddrTblPrio[GetParam().pos][port2];
            uint8_t l2Addr2 = (l3Addr2 & 0x00FF);

            if (l2Addr2 == 0 || GetParam().devCnt[port] <= 1) // TODO what if in debug if we keep addr
            {
                continue;
            }

            // confirm RX is enabled
            ASSERT_EQ((uart_objs[port2].C2 & (UART_C2_RE_MASK | UART_C2_RIE_MASK)) != 0, true);

            uint16_t l3SrcAddr = 0;

            for (int pos = 0; pos < MAX_POS; pos++)
            {
                // find a dst dev on port to forward
                if (pos == GetParam().pos)
                {
                    continue;
                }
                
                /* check brdcst table*/
                if (GetParam().l3BcastInSubnetForSrcPort[pos][port2] != portSubnet)
                {
                    continue;
                }

                for (int port3 = 0; port3 < MAX_PORT; port3++)
                {
                    uint8_t subnet = GetParam().l3AddrTblPrio[pos][port3] >> 8;
                    if (subnet == (l3Addr >> 8))
                    {
                        l3SrcAddr = GetParam().l3AddrTblPrio[pos][port3];
                        break;
                    }
                }
                
                //

                if (l3SrcAddr)
                {
                    break;
                }
            }

            if (!l3SrcAddr) {
                continue;
            }

            uint16_t l3DstAddr = 0;
            uint8_t rxPgOfst = 0; // page will have L4 Hdr

            PduHdr pduHdr = PduHdr{
                .l2hdr = {l2Addr, (L2_PKT_TYPE_PDU), 0xFF},
                .l3hdr = {l3SrcAddr, l3DstAddr, 1, 0},
                .l4hdr = {0, 0, 0}};

            memcpy(msg.data(), (uint8_t *)&pduHdr, sizeof(PduHdr));

            // send msg to port 1
            sendPduMsg(mock, &uart_objs[port], rxPgOfst, msg.data(),
                       msg.size(), port);

            // send MST to port 2
            sendMstToken(mock, uart_ptrs[port2], l2Addr2, port2);

            // will send message

            // first expect hdr
            pduHdr = PduHdr{
                .l2hdr = {l3DstAddr, (L2_PKT_TYPE_PDU | L2_PKT_TYPE_MST), 0xFF},
                .l3hdr = {l3SrcAddr, l3DstAddr, 1, 0},
                .l4hdr = {0, 0, 0}};

            uint8_t size;

            expectFrwdFrame(mock,
                            uart_ptrs[port2],
                            port2,
                            idx[port2],
                            size,
                            TxMultiFrameType::TX_MULTI_FRAME_FIRST_MSG,
                            (msg.data() + (sizeof(PduHdr))),
                            (msg.size() - (sizeof(PduHdr))), /* L4 hdr is part of page buffer */
                            pduHdr);
        }
    }
}
//#endif

// route tables
static constexpr std::array<uint16_t, MAX_SUBNET> makeL3RouteTablePos1()
{
    std::array<uint16_t, MAX_SUBNET> a{};  // all zero
    a[2] = 0x0302; // gateway to subnet 2 for pos 1
    return a;
}

static constexpr std::array<uint16_t, MAX_SUBNET> makeL3RouteTablePos2()
{
    std::array<uint16_t, MAX_SUBNET> a{};  // all zero
    a[1] = 0x0301; // gateway to subnet 1 for pos 2
    return a;
}

static constexpr std::array<uint16_t, MAX_SUBNET> makeL3RouteTablePos3()
{
    std::array<uint16_t, MAX_SUBNET> a{};  // all zero
    /* Todo does 3 need this gateway ? since it can reach all pos directly? */
    a[3] = 0x0101;
    return a;
}

static constexpr std::array<uint16_t, MAX_SUBNET> makeL3RouteTablePos5()
{
    std::array<uint16_t, MAX_SUBNET> a{};  // all zero
    a[1] = 0x0202; // gateway to subnet 1 for pos 5
    a[3] = 0x0201; // gateway to subnet 3 for pos 5
    return a;
}

static constexpr std::array<uint16_t, MAX_SUBNET> makeL3RouteTablePos7()
{
    std::array<uint16_t, MAX_SUBNET> a{};  // all zero
    a[2] = 0x0102; // gateway to subnet 2 for pos 7
    a[3] = 0x0101; // gateway to subnet 3 for pos 7
    return a;
}

/* brdcst tables */
static constexpr std::array<std::array<uint8_t, MAX_PORT>, MAX_POS> l3BcastInSubnetForSrcPortPos1 = {{
    {},
    {}, // pos 1
    {3}, // pos 2
    {0, 1}, // pos 3
    {},
    {}, // pos 5
    {},
    {0, 1} // pos 7
}};

static constexpr std::array<std::array<uint8_t, MAX_PORT>, MAX_POS> l3BcastInSubnetForSrcPortPos2 = {{
    {},
    {3},     // pos 1
    {},    // pos 2
    {}, // pos 3
    {},
    {0, 2}, // pos 5
    {},
    {} // pos 7
}};

static constexpr std::array<std::array<uint8_t, MAX_PORT>, MAX_POS> l3BcastInSubnetForSrcPortPos3 = {{
    {},
    {}, // pos 1
    {},  // pos 2
    {},  // pos 3
    {},
    {2}, // pos 5
    {},
    {0, 1} // pos 7
}};

static constexpr std::array<std::array<uint8_t, MAX_PORT>, MAX_POS> l3BcastInSubnetForSrcPortPos5 = {{
    {},
    {}, // pos 1
    {}, // pos 2
    {}, // pos 3
    {},
    {}, // pos 5
    {},
    {} // pos 7
}};

static constexpr std::array<std::array<uint8_t, MAX_PORT>, MAX_POS> l3BcastInSubnetForSrcPortPos7 = {{
    {},
    {}, // pos 1
    {}, // pos 2
    {}, // pos 3
    {},
    {}, // pos 5
    {},
    {} // pos 7
}};

/* Addr Tables */
static constexpr std::array<std::array<uint16_t, MAX_PORT>, MAX_POS> l3AddrTblPrioPos1 = { {
    {},
    {0x0101, 0x0301}, // pos 1
    {0x0302, 0x0201}, // pos 2
    {0x0102, 0x0202}, // pos 3
    {},
    {0x0203, 0x0000}, // pos 5
    {},
    {0x0103, 0x0000}  // pos 7
} };

static constexpr std::array<std::array<uint16_t, MAX_PORT>, MAX_POS> l3AddrTblPrioPos2 = { {
    {},
    {0x0301, 0x0101}, // pos 1
    {0x0201, 0x0302}, // pos 2
    {0x0202, 0x0102}, // pos 3
    {},
    {0x0203, 0x0000}, // pos 5
    {},
    {0x0103, 0x0000}  // pos 7
} };


static constexpr std::array<std::array<uint16_t, MAX_PORT>, MAX_POS> l3AddrTblPrioPos3 = { {
    {},
    {0x0101, 0x0301}, // pos 1
    {0x0201, 0x0302}, // pos 2
    {0x0102, 0x0202}, // pos 3
    {},
    {0x0203, 0x0000}, // pos 5
    {},
    {0x0103, 0x0000}  // pos 7
} };

static constexpr std::array<std::array<uint16_t, MAX_PORT>, MAX_POS> l3AddrTblPrioPos5 = { {
    {},
    {0x0301, 0x0101}, // pos 1
    {0x0201, 0x0302}, // pos 2
    {0x0202, 0x0102}, // pos 3
    {},
    {0x0203, 0x0000}, // pos 5
    {},
    {0x0103, 0x0000}  // pos 7
} };

static constexpr std::array<std::array<uint16_t, MAX_PORT>, MAX_POS> l3AddrTblPrioPos7 = { {
    {},
    {0x0101, 0x0301}, // pos 1
    {0x0302, 0x0201}, // pos 2
    {0x0102, 0x0202}, // pos 3
    {},
    {0x0203, 0x0000}, // pos 5
    {},
    {0x0103, 0x0000}  // pos 7
} };

INSTANTIATE_TEST_SUITE_P(
    Runs, MultiHop,
    ::testing::Values(
      //     pos port1   port2
      Case{ 1, l3AddrTblPrioPos1, makeL3RouteTablePos1(), l3BcastInSubnetForSrcPortPos1, {3, 2} },
      Case{ 2, l3AddrTblPrioPos2, makeL3RouteTablePos2(), l3BcastInSubnetForSrcPortPos2, {3, 2} },
      Case{ 3, l3AddrTblPrioPos3, makeL3RouteTablePos3(), l3BcastInSubnetForSrcPortPos3, {3, 3} },
      Case{ 5, l3AddrTblPrioPos5, makeL3RouteTablePos5(), l3BcastInSubnetForSrcPortPos5, {3, 0} },
      Case{ 7, l3AddrTblPrioPos7, makeL3RouteTablePos7(), l3BcastInSubnetForSrcPortPos7, {3, 0} }
    )
);

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

