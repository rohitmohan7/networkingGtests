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
    std::array<uint8_t, MAX_SUBNET> l3RouteHops;
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
    MOCK_METHOD(void, l1UARTWriteNonBlocking, (UART_Type * UART, const uint8_t *data, size_t length, L2Crc_t *crc), ());
    MOCK_METHOD(bool, l1UARTCmpNonBlocking, (UART_Type* UART, const uint8_t* data, size_t length), ());
    MOCK_METHOD(void, l1UARTReadNonBlocking, (UART_Type * UART, uint8_t *data, size_t length, L2Crc_t *crc), ());
    MOCK_METHOD(void, appRecv, (uint16_t pos, const uint8_t* const data, MsgLenType_t len), ());
    MOCK_METHOD(uint32_t, pitGetCurrMS, (), ());
};

static MockUart* g_mock = nullptr;

// 3) The linker will redirect calls to hw_read() to __wrap_hw_read()
extern "C" void l1UARTWriteNonBlocking(UART_Type *UART, const uint8_t *data, size_t length, L2Crc_t *crc)
{
    ASSERT_NE(g_mock, nullptr);
    g_mock->l1UARTWriteNonBlocking(UART, data, length, crc);
}

extern "C" void l1UARTReadNonBlocking(UART_Type *UART, uint8_t *data, size_t length, L2Crc_t *crc)
{
    ASSERT_NE(g_mock, nullptr);
    g_mock->l1UARTReadNonBlocking(UART, data, length, crc);
}

extern "C" bool l1UARTCmpNonBlocking(UART_Type* UART, const uint8_t* data, size_t length)
{
    EXPECT_NE(g_mock, nullptr);
    return  g_mock->l1UARTCmpNonBlocking(UART, data, length);
}

extern "C" void appRecv(uint16_t pos, const uint8_t* const data, MsgLenType_t len) { // TODO

    EXPECT_NE(g_mock, nullptr);
    return  g_mock->appRecv(pos, data, len);
}

extern "C" uint32_t pitGetCurrMS()
{
    EXPECT_NE(g_mock, nullptr);
    return  g_mock->pitGetCurrMS();
}

static inline L2Crc_t crcTestContinous(L2Crc_t crc,
                                       const uint8_t *data,
                                       size_t size)
{
    const uint8_t poly = 0x07;

    for (size_t i = 0; i < size; i++)
    {
        uint8_t byte = data[i];
        crc ^= byte;

        for (uint8_t bit = 0; bit < 8; bit++)
        {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ poly)
                               : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static void expectMst(MockUart& mock, uint8_t addr, UART_Type* uart, uint8_t port) {
    std::array<uint8_t, sizeof(L2Hdr)> expected{{addr, L2_PKT_TYPE_MST}};
    const uint8_t expectedSize = (sizeof(L2Hdr));
    L2Crc_t expectCrc = crcTestContinous(0x00, expected.data(), expected.size());

    EXPECT_CALL(mock, l1UARTWriteNonBlocking(uart, testing::NotNull(), expectedSize, testing::NotNull()))
        .Times(1)
        .WillOnce(testing::Invoke([expected, expectCrc](UART_Type *UART, const uint8_t *data, size_t len, L2Crc_t *crc)
                                  { 
                                    ASSERT_EQ(0, std::memcmp(data, expected.data(), expected.size())); 
                                    ASSERT_EQ(0, *crc);
                                    *crc = expectCrc; }))
        .RetiresOnSaturation();

    EXPECT_CALL(mock, l1UARTWriteNonBlocking(uart, testing::NotNull(), sizeof(L2Crc_t), testing::IsNull()))
        .Times(1)
        .WillOnce(testing::Invoke([expectCrc](UART_Type *UART, const uint8_t *data, size_t len, L2Crc_t *crc)
                                  { EXPECT_EQ(0, std::memcmp(data, (&expectCrc), len)); }))
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
    uart->RCFIFO = expectedSize + sizeof(L2Crc_t);
    // echo
    EXPECT_CALL(mock, l1UARTCmpNonBlocking(uart, testing::NotNull(), expectedSize))
        .Times(1)
        .WillOnce(testing::Invoke([expected](UART_Type* UART, const uint8_t* data, size_t len) {
        EXPECT_EQ(0, std::memcmp(data, expected.data(), expected.size()));
        return true;
            }))
        .RetiresOnSaturation();

    EXPECT_CALL(mock, l1UARTCmpNonBlocking(uart, testing::NotNull(), sizeof(L2Crc_t)))
        .Times(1)
        .WillOnce(testing::Invoke([expectCrc](UART_Type *UART, const uint8_t *data, size_t len)
                                  {
        EXPECT_EQ(0, std::memcmp(data, (&expectCrc), len));
        return true; }))
        .RetiresOnSaturation();

    // echo isr
    l1TransferHandleIRQ(uart, port);
    uart->S1 &= ~UART_S1_RDRF_MASK;
}

void sendMstToken(MockUart& mock, UART_Type* UART, const uint8_t& l2Addr, const uint8_t& port) {
    std::array<uint8_t, sizeof(L2Hdr) + sizeof(L2Crc_t)> pkt{{l2Addr, L2_PKT_TYPE_MST}};
    L2Crc_t expectCrc = crcTestContinous(0x00, pkt.data(), pkt.size() - sizeof(L2Crc_t));
    memcpy(pkt.data() + sizeof(L2Hdr), &expectCrc, sizeof(L2Crc_t));

    UART->S1 |= UART_S1_RDRF_MASK;
    UART->RCFIFO = pkt.size();

    // first it will read up to CRC len
    EXPECT_CALL(mock, l1UARTReadNonBlocking(UART, testing::NotNull(), sizeof(L2Crc_t), testing::IsNull()))
        .Times(1)
        .WillOnce(testing::Invoke([pkt](UART_Type *UART, uint8_t *data, size_t len, L2Crc_t * crc)
                                  { std::memcpy(data, pkt.data(), len); }))
        .RetiresOnSaturation();

    // next it will read data up to crc len into ptr
    EXPECT_CALL(mock, l1UARTReadNonBlocking(UART, testing::NotNull(), pkt.size() - (2 * sizeof(L2Crc_t)), testing::NotNull()))
        .Times(1)
        .WillOnce(testing::Invoke([pkt](UART_Type *UART, uint8_t *data, size_t len, L2Crc_t * crc)
                                  { 
                                    std::memcpy(data, pkt.data() + sizeof(L2Crc_t), len);
                                    *crc = crcTestContinous(*crc, data, len);
                                }))
        .RetiresOnSaturation();

    // finaly read crc ..
    EXPECT_CALL(mock, l1UARTReadNonBlocking(UART, testing::NotNull(), sizeof(L2Crc_t), testing::IsNull()))
        .Times(1)
        .WillOnce(testing::Invoke([pkt](UART_Type *UART, uint8_t *data, size_t len, L2Crc_t *crc)
                                  { std::memcpy(data, pkt.data() + pkt.size() - sizeof(L2Crc_t), len); }))
        .RetiresOnSaturation();

    l1TransferHandleIRQ(UART, port);

    UART->S1 &= ~UART_S1_RDRF_MASK;
    UART->RCFIFO = 0;

    PITCallback(port + L2_PIT_TIMER_START_IDX); // interchar silence

    PITCallback(port + L2_PIT_TIMER_START_IDX); // interframe silence
}

void sendMsgAck(MockUart &mock, UART_Type *UART, const PduHdr &pduHdr, const uint8_t &port)
{
    std::array<uint8_t, sizeof(PduHdr) + sizeof(L2Crc_t)> pkt{};
    std::memcpy(pkt.data(), &pduHdr, sizeof(pduHdr));
    L2Crc_t expectCrc = crcTestContinous(0x00, pkt.data(), pkt.size() - sizeof(L2Crc_t));
    memcpy(pkt.data() + sizeof(pduHdr), &expectCrc, sizeof(L2Crc_t));

    const size_t crcSize = sizeof(L2Crc_t);
    const size_t totalSize = pkt.size();

    size_t streamPos = 0; // bytes already supplied to the mock
    size_t totalRx = 0;   // bytes l1Rx has consumed from UART so far
    size_t remaining = totalSize;

    auto expectRead = [&](size_t len, bool withCalcCrc)
    {
        if (withCalcCrc)
        {
            EXPECT_CALL(
                mock,
                l1UARTReadNonBlocking(
                    UART,
                    testing::NotNull(),
                    len,
                    testing::NotNull()))
                .Times(1)
                .WillOnce(testing::Invoke(
                    [&, len](UART_Type *, uint8_t *data, size_t gotLen, L2Crc_t *crc)
                    {
                        ASSERT_EQ(gotLen, len);
                        ASSERT_LE(streamPos + gotLen, totalSize);

                        std::memcpy(data, pkt.data() + streamPos, gotLen);
                        *crc = crcTestContinous(*crc, data, gotLen);

                        streamPos += gotLen;
                    }))
                .RetiresOnSaturation();
        }
        else
        {
            EXPECT_CALL(
                mock,
                l1UARTReadNonBlocking(
                    UART,
                    testing::NotNull(),
                    len,
                    testing::IsNull()))
                .Times(1)
                .WillOnce(testing::Invoke(
                    [&, len](UART_Type *, uint8_t *data, size_t gotLen, L2Crc_t *crc)
                    {
                        ASSERT_EQ(gotLen, len);
                        ASSERT_LE(streamPos + gotLen, totalSize);

                        std::memcpy(data, pkt.data() + streamPos, gotLen);
                        ASSERT_EQ(crc, nullptr);

                        streamPos += gotLen;
                    }))
                .RetiresOnSaturation();
        }
    };

    auto contiguousAvail = [](size_t idx) -> size_t
    {
        if (idx < sizeof(L2Hdr))
        {
            return sizeof(L2Hdr) - idx;
        }

        if (idx < sizeof(L2Hdr) + sizeof(L3Hdr)) {
            return (sizeof(L2Hdr) + sizeof(L3Hdr)) - idx;
        }
        
        if (idx < sizeof(PduHdr))
        {
            return sizeof(PduHdr) - idx;
        }
        return 0;
    };

    UART->S1 |= UART_S1_RDRF_MASK;
    UART->RCFIFO = totalSize;

    {
        while (remaining > 0)
        {
            const size_t crcFill = std::min(totalRx, crcSize);

            // First fill the rolling CRC tail
            if (crcFill < crcSize)
            {
                const size_t fill = std::min(crcSize - crcFill, remaining);
                expectRead(fill, false);
                totalRx += fill;
                remaining -= fill;
                continue;
            }

            // l2GetRxPkt() exposes L2Hdr first, then the rest of PduHdr
            const size_t payloadIndex = totalRx - crcSize;
            const size_t avail = contiguousAvail(payloadIndex);
            ASSERT_GT(avail, 0u);

            const size_t n = std::min(avail, remaining);

            if (n < crcSize)
            {
                // Only refill the tail from UART; released bytes come from old crcBuf
                expectRead(n, false);
            }
            else
            {
                const size_t direct = n - crcSize;

                // Direct bytes read into payload contribute to calculated CRC
                if (direct > 0)
                {
                    expectRead(direct, true);
                }

                // Final crcSize bytes of this step refill the rolling tail
                expectRead(crcSize, false);
            }

            totalRx += n;
            remaining -= n;
        }
    }

    l1TransferHandleIRQ(UART, port);

    EXPECT_EQ(streamPos, totalSize);

    UART->S1 &= ~UART_S1_RDRF_MASK;
    UART->RCFIFO = 0;

    PITCallback(port + L2_PIT_TIMER_START_IDX); // interchar silence
    PITCallback(port + L2_PIT_TIMER_START_IDX); // interframe silence
}

void sendPduMsg(MockUart &mock,
                UART_Type *UART,
                uint8_t &rxPgOfst,
                const uint8_t *msg,
                const int &origSize,
                const uint8_t &port,
                const bool& forward = false)
{
    using ::testing::InSequence;

    const size_t crcSize = sizeof(L2Crc_t);
    size_t streamPos = 0;
    size_t totalRx = 0;

    auto contiguousAvail = [forward](size_t idx) -> size_t
    {
        if (idx < sizeof(L2Hdr))
        {
            return sizeof(L2Hdr) - idx;
        }
        if (idx < sizeof(L2Hdr) + sizeof(L3Hdr))
        {
            return (sizeof(L2Hdr) + sizeof(L3Hdr)) - idx;
        }
        if (!forward && idx < sizeof(PduHdr))
        {
            return sizeof(PduHdr) - idx;
        }
        return 0;
    };

    auto expectExactRead = [&](size_t len, bool withCalcCrc)
    {
        if (withCalcCrc)
        {
            EXPECT_CALL(mock,
                        l1UARTReadNonBlocking(UART,
                                              testing::NotNull(),
                                              len,
                                              testing::NotNull()))
                .Times(1)
                .WillOnce(testing::Invoke(
                    [&, len](UART_Type *, uint8_t *data, size_t gotLen, L2Crc_t *crc)
                    {
                        ASSERT_EQ(gotLen, len);
                        ASSERT_NE(crc, nullptr);
                        ASSERT_LE(streamPos + gotLen, static_cast<size_t>(origSize));

                        std::memcpy(data, msg + streamPos, gotLen);
                        *crc = crcTestContinous(*crc, data, gotLen);
                        streamPos += gotLen;
                    }))
                .RetiresOnSaturation();
        }
        else
        {
            EXPECT_CALL(mock,
                        l1UARTReadNonBlocking(UART,
                                              testing::NotNull(),
                                              len,
                                              testing::IsNull()))
                .Times(1)
                .WillOnce(testing::Invoke(
                    [&, len](UART_Type *, uint8_t *data, size_t gotLen, L2Crc_t *crc)
                    {
                        ASSERT_EQ(gotLen, len);
                        ASSERT_EQ(crc, nullptr);
                        ASSERT_LE(streamPos + gotLen, static_cast<size_t>(origSize));

                        std::memcpy(data, msg + streamPos, gotLen);
                        streamPos += gotLen;
                    }))
                .RetiresOnSaturation();
        }
    };

    auto expectHeaderChunkReads = [&](size_t chunkLen)
    {
        size_t rxLen = chunkLen;

        while (rxLen > 0)
        {
            const size_t crcFill = std::min(totalRx, crcSize);

            if (crcFill < crcSize)
            {
                const size_t fill = std::min(crcSize - crcFill, rxLen);
                expectExactRead(fill, false);
                totalRx += fill;
                rxLen -= fill;
                continue;
            }

            const size_t payloadIndex = totalRx - crcSize;
            const size_t avail = contiguousAvail(payloadIndex);
            ASSERT_GT(avail, 0u);

            const size_t n = std::min(avail, rxLen);

            if (n < crcSize)
            {
                expectExactRead(n, false);
            }
            else
            {
                const size_t direct = n - crcSize;
                if (direct > 0)
                {
                    expectExactRead(direct, true);
                }
                expectExactRead(crcSize, false);
            }

            totalRx += n;
            rxLen -= n;
        }
    };

    UART->S1 |= UART_S1_RDRF_MASK;

    uint8_t hdrSize = forward ? sizeof(L2Hdr) + sizeof(L3Hdr) : sizeof(PduHdr);

    {
        InSequence seq;

        int msgSize = origSize;

        /* First IRQ: header area only */
        UART->RCFIFO = hdrSize;
        expectHeaderChunkReads(hdrSize);
        l1TransferHandleIRQ(UART, port);
        msgSize -= hdrSize;
    }

    /* After PduHdr, let the mock stream bytes in whatever chunking
    getL3RxPktFrag() causes. */
    EXPECT_CALL(mock,
                l1UARTReadNonBlocking(UART,
                                      testing::NotNull(),
                                      testing::_,
                                      testing::_))
        .WillRepeatedly(testing::Invoke(
            [&](UART_Type *, uint8_t *data, size_t len, L2Crc_t *crc)
            {
                ASSERT_LE(streamPos + len, static_cast<size_t>(origSize));
                std::memcpy(data, msg + streamPos, len);

                if (crc != nullptr)
                {
                    *crc = crcTestContinous(*crc, data, len);
                }

                streamPos += len;
            }));

    int msgSize = origSize - hdrSize;
    while (msgSize > 0)
    {
        uint8_t size = std::min(static_cast<int>(UNIT - rxPgOfst), msgSize);
        UART->RCFIFO = size;
        l1TransferHandleIRQ(UART, port);
        msgSize -= size;
    }

    EXPECT_EQ(streamPos, static_cast<size_t>(origSize));

    UART->S1 &= ~UART_S1_RDRF_MASK;
    UART->RCFIFO = 0;

    PITCallback(port + L2_PIT_TIMER_START_IDX);
    PITCallback(port + L2_PIT_TIMER_START_IDX);
}

#define L2_FRAME_SIZE (RS485_FRAME_SIZE - sizeof(L2Hdr) - sizeof(L2Crc_t))
#define L3_FRAME_SIZE   ((L2_FRAME_SIZE - sizeof(L3Hdr)) & ~((uint16_t)0x7U)) /* 8 byte aligned */
#define L4_FRAME_SIZE (L3_FRAME_SIZE - sizeof(L4Hdr))

L2Crc_t expectHdr(MockUart &mock, UART_Type *UART, const PduHdr &pduHdr, const uint8_t &port, const bool &echo, const bool & forward = false, const L2Crc_t expectCrcIn = 0)
{
    uint8_t hdrSize = forward ? sizeof(L2Hdr) + sizeof(L3Hdr) : sizeof(PduHdr);
    if (!echo) {
        L2Crc_t expectCrc = crcTestContinous(0x00, (uint8_t *)&pduHdr, sizeof(pduHdr));
        

        EXPECT_CALL(mock, l1UARTWriteNonBlocking(UART, testing::NotNull(), hdrSize, testing::NotNull()))
            .Times(1)
            .WillOnce(testing::Invoke([pduHdr, expectCrc](UART_Type *UART, const uint8_t *data, size_t len, L2Crc_t *crc)
                                      { 
                                        EXPECT_EQ(0, std::memcmp(data, (uint8_t *)&pduHdr, len));
                                        ASSERT_EQ(*crc, 0);
                                        *crc = expectCrc;
                                      }))
            .RetiresOnSaturation();

        return expectCrc;
    }
    else {
        EXPECT_CALL(mock, l1UARTCmpNonBlocking(UART, testing::NotNull(), hdrSize))
            .Times(1)
            .WillOnce(testing::Invoke([pduHdr](UART_Type* UART, const uint8_t* data, size_t len) {
            EXPECT_EQ(0, std::memcmp(data, (uint8_t*)&pduHdr, len));
            return true;
                }))
            .RetiresOnSaturation();

        UART->S1 |= UART_S1_RDRF_MASK;
        UART->RCFIFO = hdrSize;
        
         /* if CRC is provided this is a hdr only message */
        if (expectCrcIn)
        {
            UART->RCFIFO += sizeof(L2Crc_t);
            EXPECT_CALL(mock, l1UARTCmpNonBlocking(UART, testing::NotNull(), sizeof(L2Crc_t)))
                .Times(1)
                .WillOnce(testing::Invoke([expectCrcIn](UART_Type *UART, const uint8_t *data, size_t len)
                                          {
            EXPECT_EQ(0, std::memcmp(data, (&expectCrcIn), len));
            return true; }))
                .RetiresOnSaturation();
        }

        l1TransferHandleIRQ(UART, port);

        UART->S1 &= ~UART_S1_RDRF_MASK;
        UART->RCFIFO = 0;
    }
    
    return 0;
}

void expectFrwdHdr(MockUart &mock, UART_Type *UART, const PduHdr &pduHdr, const uint8_t &port, const bool &echo)
{

    if (!echo)
    {
        EXPECT_CALL(mock, l1UARTWriteNonBlocking(UART, testing::NotNull(), sizeof(PduHdr) - sizeof(L4Hdr), testing::NotNull()))
            .Times(1)
            .WillOnce(testing::Invoke([pduHdr](UART_Type *UART, const uint8_t *data, size_t len, L2Crc_t *crc)
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

    L2Crc_t expectCrc = expectHdr(mock, UART, pduHdr, port, false);
    EXPECT_CALL(mock, l1UARTWriteNonBlocking(UART, testing::NotNull(), sizeof(L2Crc_t), testing::IsNull()))
        .Times(1)
        .WillOnce(testing::Invoke([expectCrc](UART_Type *UART, const uint8_t *data, size_t len, L2Crc_t *crc)
                                  { EXPECT_EQ(0, std::memcmp(data, (&expectCrc), len)); }))
        .RetiresOnSaturation();

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

    expectHdr(mock, UART, pduHdr, port, true, false, expectCrc);
}

enum class TxMultiFrameType {
    TX_MULTI_FRAME_FIRST_MSG,
    TX_MULTI_FRAME_NEW_MSG,
    TX_MULTI_FRAME_NEW_FRAME,
    TX_MULTI_FRAME_CONT,
};

void expectTxMultiFrame(MockUart &mock,
                        UART_Type *UART,
                        const uint8_t &port,
                        uint8_t &idxIn,
                        uint8_t &sizeIn,
                        const TxMultiFrameType &type,
                        const uint8_t *msg,
                        const int &msgSize,
                        const PduHdr &hdr = PduHdr(),
                        const bool &mstAftLstFrame = true,
                        const uint32_t &milliSeconds = 0,
                        const bool &echo = false,
                        L2Crc_t expectCrc = 0)
{
    uint8_t idx = idxIn;
    uint8_t size = sizeIn;

    if (type == TxMultiFrameType::TX_MULTI_FRAME_FIRST_MSG) {
        size = 0;
        idx = 0;
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
        if (!echo) {
            expectCrc = expectHdr(mock, UART, hdr, port, echo);
        } else {
            expectHdr(mock, UART, hdr, port, echo);
        }
    }

    size = UNIT - (size + (type == TxMultiFrameType::TX_MULTI_FRAME_NEW_MSG ? sizeof(L4Hdr::msgLen) + sizeof(txOrder) + (hdr.l3hdr.dst == 0 ? 0 : sizeof(L4Hdr::msgFlgs)) : 0));
    uint8_t len = UART_FIFO_SIZE - size - (type != TxMultiFrameType::TX_MULTI_FRAME_CONT ? sizeof(PduHdr) : 0);

    for (;;) {
        uint8_t idx_loc = idx;
        if (idx_loc + size > msgSize) {
            size = msgSize - idx_loc;
            len = 0;
        }
        
        if (!echo) {
            L2Crc_t nxtExpectCrc = crcTestContinous(expectCrc, (msg + idx_loc), size);
            EXPECT_CALL(mock, l1UARTWriteNonBlocking(UART, testing::NotNull(), size, testing::NotNull()))
                .Times(1)
                .WillOnce(testing::Invoke([msg, idx_loc, expectCrc, nxtExpectCrc](UART_Type *UART, const uint8_t *data, size_t len, L2Crc_t *crc)
                                          { 
                                            EXPECT_EQ(0, std::memcmp(data, (msg + idx_loc), len));
                                            ASSERT_EQ(expectCrc, *crc);
                                            *crc = nxtExpectCrc;
                                          }))
                .RetiresOnSaturation();
            expectCrc = nxtExpectCrc;
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
#if 0
            if ((idx + size >= msgSize) && !(len) && (hdr.l4hdr.msgFlgs & L4_MSG_FLAG_REQ_ACK)) {
                EXPECT_CALL(mock, pitGetCurrMS())
                    .Times(1)
                    .WillOnce(testing::Return(milliSeconds))
                    .RetiresOnSaturation();
            }
#endif

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
            
            /* CRC echo */
            // set the echo register
            UART->S1 |= UART_S1_RDRF_MASK;
            UART->RCFIFO = sizeof(L2Crc_t);
            // echo
                                            
            EXPECT_CALL(mock, l1UARTCmpNonBlocking(UART, testing::NotNull(), sizeof(L2Crc_t)))
                    .Times(1)
                    .WillOnce(testing::Invoke([expectCrc](UART_Type *UART, const uint8_t *data, size_t len)
                                              {
            EXPECT_EQ(0, std::memcmp(data, (&expectCrc), len));
            return true; }))
                    .RetiresOnSaturation();

            if ((idx >= msgSize) && /*!(len) && */(hdr.l4hdr.msgFlgs & L4_MSG_FLAG_REQ_ACK))
            {
                EXPECT_CALL(mock, pitGetCurrMS())
                    .Times(1)
                    .WillOnce(testing::Return(milliSeconds))
                    .RetiresOnSaturation();
            }

            // echo isr
            l1TransferHandleIRQ(UART, port);
            UART->S1 &= ~UART_S1_RDRF_MASK;

            // tx complete check to send next frame
            if ((msgSize - idx) > 0) {
                // inter frame silence
                PITCallback(port + L2_PIT_TIMER_START_IDX);

                PduHdr hdrCpy = hdr;
                if (mstAftLstFrame && (msgSize - idx) <= L4_FRAME_SIZE) {
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
                               milliSeconds,
                               false,
                               expectCrc);
        }
    }
    else {
        UART->S1 = (uint8_t)((UART->S1 & (uint8_t)~UART_S1_TC_MASK) | UART_S1_TDRE_MASK);
        bool frameCmplt = (!(idx % L4_FRAME_SIZE) || (msgSize - idx) < UART_FIFO_SIZE);
        
        if (frameCmplt) {
            EXPECT_CALL(mock, l1UARTWriteNonBlocking(UART, testing::NotNull(), sizeof(L2Crc_t), testing::IsNull()))
                .Times(1)
                .WillOnce(testing::Invoke([expectCrc](UART_Type *UART, const uint8_t *data, size_t len, L2Crc_t *crc)
                                          { EXPECT_EQ(0, std::memcmp(data, (&expectCrc), len)); }))
                .RetiresOnSaturation();
        }
        
        l1TransferHandleIRQ(UART, port); // complete TX of single frame

        if (frameCmplt) {
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
        }
        else
        {
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
                           true,
                           expectCrc);
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
                     const bool &echo = false,
                     L2Crc_t expectCrc = 0)
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
        // expect header
        if (!echo) {
            expectCrc = expectHdr(mock, UART, hdr, port, echo, true);
        } else {
            expectHdr(mock, UART, hdr, port, echo, true);
        }
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
            L2Crc_t nxtExpectCrc = crcTestContinous(expectCrc, (msg + idx_loc), size);
            EXPECT_CALL(mock, l1UARTWriteNonBlocking(UART, testing::NotNull(), size, testing::NotNull()))
                .Times(1)
                .WillOnce(testing::Invoke([msg, idx_loc, expectCrc, nxtExpectCrc](UART_Type *UART, const uint8_t *data, size_t len, L2Crc_t *crc)
                                          { EXPECT_EQ(0, std::memcmp(data, (msg + idx_loc), len));
                                            ASSERT_EQ(expectCrc, *crc);
                                            *crc = nxtExpectCrc; 
                                            }))
                .RetiresOnSaturation();
            expectCrc = nxtExpectCrc;
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
            UART->S1 |= UART_S1_RDRF_MASK;
            UART->RCFIFO = sizeof(L2Crc_t);
            // echo

            EXPECT_CALL(mock, l1UARTCmpNonBlocking(UART, testing::NotNull(), sizeof(L2Crc_t)))
                .Times(1)
                .WillOnce(testing::Invoke([expectCrc](UART_Type *UART, const uint8_t *data, size_t len)
                                          {
            EXPECT_EQ(0, std::memcmp(data, (&expectCrc), len));
            return true; }))
                .RetiresOnSaturation();

            // echo isr
            l1TransferHandleIRQ(UART, port);
            UART->S1 &= ~UART_S1_RDRF_MASK;

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
                            milliSeconds,
                            false,
                            expectCrc);
        }
    }
    else
    {
        UART->S1 = (uint8_t)((UART->S1 & (uint8_t)~UART_S1_TC_MASK) | UART_S1_TDRE_MASK);
        bool frameCmplt = (!(idx % L4_FRAME_SIZE) || (msgSize - idx) < UART_FIFO_SIZE);

        if (frameCmplt)
        {
            EXPECT_CALL(mock, l1UARTWriteNonBlocking(UART, testing::NotNull(), sizeof(L2Crc_t), testing::IsNull()))
                .Times(1)
                .WillOnce(testing::Invoke([expectCrc](UART_Type *UART, const uint8_t *data, size_t len, L2Crc_t *crc)
                                          { EXPECT_EQ(0, std::memcmp(data, (&expectCrc), len)); }))
                .RetiresOnSaturation();
        }

        l1TransferHandleIRQ(UART, port); // complete TX of single frame
        
        if (frameCmplt)
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
                        true,
                        expectCrc);
    }
}

//#if 0
TEST_P(MultiHop, addr) {
    // check correct addr table
    ASSERT_EQ(0, std::memcmp(GetParam().l3AddrTblPrio.data(), l3AddrTblPrio, sizeof(l3AddrTblPrio)));

    // check route table
    ASSERT_EQ(0, std::memcmp(GetParam().l3RouteTable.data(), l3RouteTable, sizeof(l3RouteTable)));

    ASSERT_EQ(0, std::memcmp(GetParam().l3RouteHops.data(), l3RouteHops, sizeof(l3RouteHops)));

    // check correct brdcst table
    ASSERT_EQ(0, std::memcmp(GetParam().l3BcastInSubnetForSrcPort.data(), l3BcastInSubnetForSrcPort, sizeof(l3BcastInSubnetForSrcPort)));
}
//#endif

#define IPV4_VERSION 4
#define IPV4_VERSION_SHIFT 4

TEST_P(MultiHop, udpPduRx)
{
    if (GetParam().pos != 7) {
        const auto* info = ::testing::UnitTest::GetInstance()->current_test_info();
        GTEST_SKIP() << "pos " << GetParam().pos << " test " << info->name() << " TODO";
    }

    MockUart mock;
    g_mock = &mock;
    testing::InSequence seq;
    // construct message
    static const int MSG_SIZE = (3*L3_FRAME_SIZE) - sizeof(UdpHdr_t);
    std::array<uint8_t, MSG_SIZE> msg;
    for (int i = 0; i < MSG_SIZE; i++)
    {
        msg[i] = i;
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


        L2Hdr l2Hdr = { 0x3, (L2_PKT_TYPE_PDU) };
        UdpHdr_t udpHdr = { .srcPort = 3, .dstPort = 7, .length = MSG_SIZE };

        const uint16_t fragSize = L3_FRAME_SIZE + sizeof(L2Hdr) + sizeof(L3Hdr) + sizeof(L2Crc_t);

        std::array<uint8_t, fragSize> pktFrag1;

        memcpy(pktFrag1.data(), &l2Hdr, sizeof(L2Hdr));

        L3Hdr l3Hdr = { .verIhl = (uint8_t)((IPV4_VERSION << IPV4_VERSION_SHIFT) | (sizeof(L3Hdr) >> 2U)),
                        .prio = 0,
                        .totalLen = sizeof(L3Hdr) + L3_FRAME_SIZE,
                        .fragId = 1,
                        .fragOfst = ((1 << 13) | 0),
                        .ttl = 1,
                        .proto = IP_PROTO_UDP,
                        .src = 0x101, 
                        .dst = l3Addr
                      };

        memcpy(pktFrag1.data() + sizeof(L2Hdr), &l3Hdr, sizeof(L3Hdr));
        memcpy(pktFrag1.data() + sizeof(L2Hdr) + sizeof(L3Hdr), &udpHdr, sizeof(UdpHdr_t));
        const int size = (L3_FRAME_SIZE);
        memcpy(pktFrag1.data() + sizeof(L2Hdr) + sizeof(L3Hdr) + sizeof(UdpHdr_t), msg.data(), (L3_FRAME_SIZE-sizeof(UdpHdr_t)));

        L2Crc_t expectCrc = crcTestContinous(0x00,
            pktFrag1.data(),
            pktFrag1.size() - sizeof(L2Crc_t));
        std::memcpy(pktFrag1.data() + pktFrag1.size() - sizeof(L2Crc_t),
            &expectCrc,
            sizeof(L2Crc_t));

        std::array<uint8_t, fragSize> pktFrag2;
        memcpy(pktFrag2.data(), &l2Hdr, sizeof(L2Hdr));

        uint16_t l3HdrSize = L3_FRAME_SIZE;
        l3Hdr.fragOfst = ((1 << 13) | (L3_FRAME_SIZE >> 3));
        memcpy(pktFrag2.data() + sizeof(L2Hdr), &l3Hdr, sizeof(L3Hdr));
        memcpy(pktFrag2.data() + sizeof(L2Hdr) + sizeof(L3Hdr), msg.data() + (L3_FRAME_SIZE - sizeof(UdpHdr_t)), (L3_FRAME_SIZE));

        expectCrc = crcTestContinous(0x00,
            pktFrag2.data(),
            pktFrag2.size() - sizeof(L2Crc_t));
        std::memcpy(pktFrag2.data() + pktFrag2.size() - sizeof(L2Crc_t),
            &expectCrc,
            sizeof(L2Crc_t));

        std::array<uint8_t, fragSize> pktFrag3;
        memcpy(pktFrag3.data(), &l2Hdr, sizeof(L2Hdr));

        l3Hdr.fragOfst = (2*L3_FRAME_SIZE) >> 3;
        memcpy(pktFrag3.data() + sizeof(L2Hdr), &l3Hdr, sizeof(L3Hdr));
        memcpy(pktFrag3.data() + sizeof(L2Hdr) + sizeof(L3Hdr), msg.data() + ((2*L3_FRAME_SIZE) - sizeof(UdpHdr_t)), (L3_FRAME_SIZE));

        expectCrc = crcTestContinous(0x00,
            pktFrag3.data(),
            pktFrag3.size() - sizeof(L2Crc_t));
        std::memcpy(pktFrag3.data() + pktFrag3.size() - sizeof(L2Crc_t),
            &expectCrc,
            sizeof(L2Crc_t));

        /* Send Frag 1 first */
        sendPduMsg(mock, &uart_objs[port], rxPgOfst, pktFrag1.data(),
            pktFrag1.size(), port, true);

        /* Send Out of order Frag 3 next */
        sendPduMsg(mock, &uart_objs[port], rxPgOfst, pktFrag3.data(),
            pktFrag3.size(), port, true);

#ifdef NETWORK_ISR_RECV
        EXPECT_CALL(mock, appRecv(3, testing::NotNull(), msg.size()))
            .Times(1)
            .WillOnce(testing::Invoke([msg](uint16_t pos, const uint8_t* data, size_t len)
                {
                    for (int i = 0; i < len; i++) {
                        if (data[i] != msg[i]) {
                            std::cout << "data mismatch at idx " << i << std::endl;
                            break;
                        }
                    }

                    ASSERT_EQ(0, std::memcmp(data, msg.data(), len));
                }))
            .RetiresOnSaturation();
#endif
        /* Send Out Frag 2 next*/
        sendPduMsg(mock, &uart_objs[port], rxPgOfst, pktFrag2.data(),
            pktFrag2.size(), port, true);
    }
    ASSERT_EQ(g_free_count, (uint16_t)NUM_PAGES);
}

#if 0
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
    testing::InSequence seq;

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
            .l2hdr = { 0x1, L2_PKT_TYPE_PDU },
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

    ASSERT_EQ(g_free_count, (uint16_t)NUM_PAGES);
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
            .l2hdr = { 0x1, (L2_PKT_TYPE_PDU | L2_PKT_TYPE_MST) },
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

    ASSERT_EQ(g_free_count, (uint16_t)NUM_PAGES);
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
            .l2hdr = {0x1, (L2_PKT_TYPE_PDU | L2_PKT_TYPE_MST)},
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
            .l2hdr = {0x3, (L2_PKT_TYPE_PDU | L2_PKT_TYPE_MST)},
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

    ASSERT_EQ(g_free_count, (uint16_t)NUM_PAGES);
}
//#endif

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
            .l2hdr = {0x3, (L2_PKT_TYPE_PDU)},
            .l3hdr = {0x101, l3Addr, 1, 0},
            .l4hdr = {0, 0, 0}};

        memcpy(msg.data(), (uint8_t *)&pduHdr, sizeof(PduHdr));
        
        // give crc
        L2Crc_t expectCrc = crcTestContinous(0x00,
                                             msg.data(),
                                             msg.size() - sizeof(L2Crc_t));

        std::memcpy(msg.data() + msg.size() - sizeof(L2Crc_t),
                    &expectCrc,
                    sizeof(L2Crc_t));

        // send msg
        sendPduMsg(mock, &uart_objs[port], rxPgOfst, msg.data(),
                   msg.size(), port);
    }
    ASSERT_EQ(g_free_count, (uint16_t)NUM_PAGES);
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
            .l2hdr = {0x3, (L2_PKT_TYPE_PDU | L2_PKT_TYPE_MST)},
            .l3hdr = {0x101, l3Addr, 1, 0},
            .l4hdr = {0, L4_MSG_FLAG_REQ_ACK, 0}};

        memcpy(msg.data(), (uint8_t *)&pduHdr, sizeof(PduHdr));

        // give crc
        L2Crc_t expectCrc = crcTestContinous(0x00,
                                             msg.data(),
                                             msg.size() - sizeof(L2Crc_t));

        std::memcpy(msg.data() + msg.size() - sizeof(L2Crc_t),
                    &expectCrc,
                    sizeof(L2Crc_t));

        // send msg
        sendPduMsg(mock, &uart_objs[port], rxPgOfst, msg.data(),
                   msg.size(), port);

        PduHdr pduAck = PduHdr{
            .l2hdr = {0x1, (L2_PKT_TYPE_PDU | L2_PKT_TYPE_MST)},
            .l3hdr = {l3Addr, 0x101, 1, 0},
            .l4hdr = {0, L4_MSG_FLAG_TYPE_ACK, 0}};

        expectHdrMsg(mock, &uart_objs[port], pduAck, port);
    }
    ASSERT_EQ(g_free_count, (uint16_t)NUM_PAGES);
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
    std::array<uint8_t, MSG_SIZE + sizeof(L2Crc_t)> msg;
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
                .l2hdr = {l2Addr, (L2_PKT_TYPE_PDU)},
                .l3hdr = {l3SrcAddr, l3DstAddr, 2, 0},
                .l4hdr = {0, 0, 0}};

            memcpy(msg.data(), (uint8_t *)&pduHdr, sizeof(PduHdr));

            // give crc
            L2Crc_t expectCrc = crcTestContinous(0x00,
                                                 msg.data(),
                                                 msg.size() - sizeof(L2Crc_t));

            std::memcpy(msg.data() + msg.size() - sizeof(L2Crc_t),
                        &expectCrc,
                        sizeof(L2Crc_t));

            // send msg to port 1
            sendPduMsg(mock, &uart_objs[port], rxPgOfst, msg.data(),
                       msg.size(), port, true);

            // send MST to port 2
            sendMstToken(mock, uart_ptrs[port2], l2Addr2, port2);

            // will send message

            // first expect hdr
            pduHdr = PduHdr{
                .l2hdr = {l3DstAddr, (L2_PKT_TYPE_PDU | L2_PKT_TYPE_MST)},
                .l3hdr = {l3SrcAddr, l3DstAddr, 1, 0},
                .l4hdr = {0, 0, 0}};

            uint8_t size;

            expectFrwdFrame(mock,
                            uart_ptrs[port2],
                            port2,
                            idx[port2],
                            size,
                            TxMultiFrameType::TX_MULTI_FRAME_FIRST_MSG,
                            (msg.data() + (sizeof(L2Hdr) + sizeof(L3Hdr))),
                            (MSG_SIZE - (sizeof(L2Hdr) + sizeof(L3Hdr))), /* L4 hdr is part of page buffer */
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
                        .l2hdr = {l2Addr, (L2_PKT_TYPE_PDU)},
                        .l3hdr = {l3SrcAddr, l3DstAddr, 2, 0},
                        .l4hdr = {0, 0, 0}};

                    memcpy(msg.data(), (uint8_t *)&pduHdr, sizeof(PduHdr));

                    // give crc
                    L2Crc_t expectCrc = crcTestContinous(0x00,
                                                         msg.data(),
                                                         msg.size() - sizeof(L2Crc_t));

                    std::memcpy(msg.data() + msg.size() - sizeof(L2Crc_t),
                                &expectCrc,
                                sizeof(L2Crc_t));

                    // send msg to port 1
                    sendPduMsg(mock, &uart_objs[port], rxPgOfst, msg.data(),
                               msg.size(), port, true);

                    // send MST to port 2
                    sendMstToken(mock, uart_ptrs[port2], l2Addr2, port2);

                    // will send message

                    // first expect hdr
                    pduHdr = PduHdr{
                        .l2hdr = {GetParam().l3RouteTable[subnet], (L2_PKT_TYPE_PDU | L2_PKT_TYPE_MST)},
                        .l3hdr = {l3SrcAddr, l3DstAddr, 1, 0},
                        .l4hdr = {0, 0, 0}};

                    uint8_t size;

                    expectFrwdFrame(mock,
                                    uart_ptrs[port2],
                                    port2,
                                    idx[port2],
                                    size,
                                    TxMultiFrameType::TX_MULTI_FRAME_FIRST_MSG,
                                    (msg.data() + (sizeof(L2Hdr) + sizeof(L3Hdr))),
                                    (MSG_SIZE - (sizeof(L2Hdr) + sizeof(L3Hdr))), /* L4 hdr is part of page buffer */
                                    pduHdr);
                }
            }
        }
    }
    ASSERT_EQ(g_free_count, (uint16_t)NUM_PAGES);
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
    std::array<uint8_t, MSG_SIZE + sizeof(L2Crc_t)> msg;
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
                .l2hdr = {l2Addr, (L2_PKT_TYPE_PDU)},
                .l3hdr = {l3SrcAddr, l3DstAddr, 2, 0},
                .l4hdr = {0, 0, 0}};

            memcpy(msg.data(), (uint8_t *)&pduHdr, sizeof(PduHdr));

            // give crc
            L2Crc_t expectCrc = crcTestContinous(0x00,
                                                 msg.data(),
                                                 msg.size() - sizeof(L2Crc_t));

            std::memcpy(msg.data() + msg.size() - sizeof(L2Crc_t),
                        &expectCrc,
                        sizeof(L2Crc_t));

            // send msg to port 1
            sendPduMsg(mock, &uart_objs[port], rxPgOfst, msg.data(),
                       msg.size(), port, true);

            // send MST to port 2
            sendMstToken(mock, uart_ptrs[port2], l2Addr2, port2);

            // will send message

            // first expect hdr
            pduHdr = PduHdr{
                .l2hdr = {l3DstAddr, (L2_PKT_TYPE_PDU | L2_PKT_TYPE_MST)},
                .l3hdr = {l3SrcAddr, l3DstAddr, 1, 0},
                .l4hdr = {0, 0, 0}};

            uint8_t size;

            /* TODO L4 Hdr */

            expectFrwdFrame(mock,
                            uart_ptrs[port2],
                            port2,
                            idx[port2],
                            size,
                            TxMultiFrameType::TX_MULTI_FRAME_FIRST_MSG,
                            (msg.data() + (sizeof(L3Hdr) + sizeof(L2Hdr))),
                            (MSG_SIZE - (sizeof(L3Hdr) + sizeof(L2Hdr))), /* L4 hdr is part of page buffer */
                            pduHdr);
        }
    }
    ASSERT_EQ(g_free_count, (uint16_t)NUM_PAGES);
}
//#endif

//#if 0
TEST_P(MultiHop, pduBrdCst) {
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

    // send msg
    appBrdcast(msg.data(), MSG_SIZE, 0);

    /* Test all ports brodcast message */

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

        //appSend(msg2.data(), MSG2_SIZE, 1, 0, false);

        // send MST
        sendMstToken(mock, uart_ptrs[port], l2Addr, port);

        // will send message

        // first expect hdr
        pduHdr = PduHdr{
            .l2hdr = { 0x00, (L2_PKT_TYPE_PDU | L2_PKT_TYPE_MST) },
            .l3hdr = { l3Addr, 0x0000, 1, 0 },
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
    }

    /* confirm all pages are free */
    ASSERT_EQ(g_free_count, (uint16_t)NUM_PAGES);

    appBrdcast(msg2.data(), MSG2_SIZE, 0); /* broadcast multiframe message */

    uint8_t port;
    uint16_t l3AddrPrev;

    for (port = 0; port < MAX_PORT; port++) {

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

        //appSend(msg2.data(), MSG2_SIZE, 1, 0, false);

        // send MST
        sendMstToken(mock, uart_ptrs[port], l2Addr, port);

        // will send message

        // first expect hdr
        pduHdr = PduHdr{
            .l2hdr = { 0x00, (L2_PKT_TYPE_PDU) },
            .l3hdr = { l3Addr, 0x0000, 1, 0 },
            .l4hdr = { 0, 0, 0 }
        };

        expectTxMultiFrame(mock,
            uart_ptrs[port],
            port,
            idx[port],
            size,
            TxMultiFrameType::TX_MULTI_FRAME_FIRST_MSG,
            msg2.data(),
            msg2.size(),
            pduHdr);

        l3AddrPrev = l3Addr;
        break;
    }
    // send another brodcast msg becore the other ports have sent broadcast message 
    appBrdcast(msg.data(), MSG_SIZE, 0); /* broadcast multiframe message */
    uint8_t prevPort = port;
    bool peerPortAvail = false;

    for (port = 0; port < MAX_PORT; port++) {
        if (port == prevPort) {
            continue;
        }

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
        peerPortAvail = true;
        //appSend(msg2.data(), MSG2_SIZE, 1, 0, false);

        // send MST
        sendMstToken(mock, uart_ptrs[port], l2Addr, port);

        // will send message

        // first expect hdr
        pduHdr = PduHdr{
            .l2hdr = { 0x00, (L2_PKT_TYPE_PDU) },
            .l3hdr = { l3Addr, 0x0000, 1, 0 },
            .l4hdr = { 0, 0, 0 }
        };

        expectTxMultiFrame(mock,
            uart_ptrs[port],
            port,
            idx[port],
            size,
            TxMultiFrameType::TX_MULTI_FRAME_FIRST_MSG,
            msg2.data(),
            msg2.size(),
            pduHdr,
            false);

        size = UNIT - (8 + sizeof(txOrder) + sizeof(MsgLenType_t)); // TODO check why expectTxMultiFrame above is not setting size properly?
        //size += sizeof(txOrder) + sizeof(uint8_t);
        PITCallback(port + L2_PIT_TIMER_START_IDX); // interframe silence

        pduHdr.l2hdr.type |= L2_PKT_TYPE_MST;

        expectTxMultiFrame(mock,
            uart_ptrs[port],
            port,
            idx[port],
            size,
            TxMultiFrameType::TX_MULTI_FRAME_NEW_MSG,
            msg.data(),
            msg.size(),
            pduHdr);
    }

    /* Check if first port sends next brdcast message */
    port = prevPort;

    // first expect hdr
    pduHdr = PduHdr{
        .l2hdr = { 0x00, (L2_PKT_TYPE_PDU | L2_PKT_TYPE_MST) },
        .l3hdr = { l3AddrPrev, 0x0000, 1, 0 },
        .l4hdr = { 0, 0, 0 }
    };

    sendMstToken(mock, uart_ptrs[port], (l3AddrPrev & 0x00FF), port);

    size = UNIT - (8 + sizeof(txOrder) + sizeof(MsgLenType_t)); // TODO check

    expectTxMultiFrame(mock,
        uart_ptrs[port],
        port,
        idx[port],
        size,
        (peerPortAvail? TxMultiFrameType::TX_MULTI_FRAME_NEW_MSG: 
            TxMultiFrameType::TX_MULTI_FRAME_FIRST_MSG),
        msg.data(),
        msg.size(),
        pduHdr);

    /*confirm all pages are free */
    ASSERT_EQ(g_free_count, (uint16_t)NUM_PAGES);
}
#endif

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

// route hops
static constexpr std::array<uint8_t, MAX_SUBNET> makeL3RouteHopsPos1()
{
    std::array<uint8_t, MAX_SUBNET> a{}; // all zero
    a.fill(255);
    a[1] = 0x00;
    a[2] = 0x01;                        // gateway to subnet 2 for pos 1
    a[3] = 0x00;
    return a;
}

static constexpr std::array<uint8_t, MAX_SUBNET> makeL3RouteHopsPos2()
{
    std::array<uint8_t, MAX_SUBNET> a{}; // all zero
    a.fill(255);
    a[1] = 0x01;                        // gateway to subnet 1 for pos 2
    a[2] = 0x00;
    a[3] = 0x00;
    return a;
}

static constexpr std::array<uint8_t, MAX_SUBNET> makeL3RouteHopsPos3()
{
    std::array<uint8_t, MAX_SUBNET> a{}; // all zero
    a.fill(255);
    /* Todo does 3 need this gateway ? since it can reach all pos directly? */
    a[1] = 0x00;
    a[2] = 0x00;
    a[3] = 0x01;
    return a;
}

static constexpr std::array<uint8_t, MAX_SUBNET> makeL3RouteHopsPos5()
{
    std::array<uint8_t, MAX_SUBNET> a{}; // all zero
    a.fill(255);
    a[1] = 0x01;
    a[2] = 0x00;                        
    a[3] = 0x01;                        
    return a;
}

static constexpr std::array<uint8_t, MAX_SUBNET> makeL3RouteHopsPos7()
{
    std::array<uint8_t, MAX_SUBNET> a{}; // all zero
    a.fill(255);
    a[1] = 0x00;
    a[2] = 0x01;                        
    a[3] = 0x01;                        
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
      Case{ 1, l3AddrTblPrioPos1, makeL3RouteTablePos1(), l3BcastInSubnetForSrcPortPos1, makeL3RouteHopsPos1(), {3, 2} },
      Case{ 2, l3AddrTblPrioPos2, makeL3RouteTablePos2(), l3BcastInSubnetForSrcPortPos2, makeL3RouteHopsPos2(), {3, 2} },
      Case{ 3, l3AddrTblPrioPos3, makeL3RouteTablePos3(), l3BcastInSubnetForSrcPortPos3, makeL3RouteHopsPos3(), {3, 3} },
      Case{ 5, l3AddrTblPrioPos5, makeL3RouteTablePos5(), l3BcastInSubnetForSrcPortPos5, makeL3RouteHopsPos5(), {3, 0} },
      Case{ 7, l3AddrTblPrioPos7, makeL3RouteTablePos7(), l3BcastInSubnetForSrcPortPos7, makeL3RouteHopsPos7(), {3, 0} }
    )
);

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

