/*
 * QTest for the 3C515 bus-master TX-DMA "DownComplete" handshake.
 *
 * Verifies the contract the 3cpd.exe Phase-1a busmaster test-before-trust probe
 * (start.asm phase_validate_dma) depends on:
 *   1. A single-transfer down-descriptor is posted at DownListPtr (+0x24).
 *   2. StartDmaDown (0xA001) makes the adapter bus-master READ the descriptor + buffer.
 *   3. The descriptor STATUS is written back with DN_COMPLETE (0x4000) -- the exact bit the
 *      cold probe polls (test word [tx_descs+4], 0x4000) to decide whether to keep g_use_dma.
 *   4. The DMA'd payload reaches the wire unchanged (proves the bus-master read content).
 *
 * If this fails, the driver's probe times out and falls back to PIO -> DMA is never trusted.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/iov.h"
#include "libqtest.h"

/* 3C515 register map (default iobase 0x300). Command/Status at +0x0E; the bus-master DMA
 * register block is at +0x20, with DownListPtr at region offset 0x04 -> +0x24. */
#define IOBASE          0x300
#define REG_CMD_STATUS  (IOBASE + 0x0E)   /* write = command, read = status */
#define REG_DOWNLISTPTR (IOBASE + 0x24)   /* 32-bit TX descriptor pointer */

/* Command encoding: opcode in bits 15..11, param in bits 10..0. */
#define CMD_TX_ENABLE     (0x09 << 11)             /* 0x4800 */
#define CMD_START_DMADOWN ((0x14 << 11) | 0x001)   /* 0xA001 -- param 1 = down (TX) */

/* Descriptor STATUS bits (written back into guest RAM). */
#define DESC_DN_COMPLETE 0x00004000

/* Guest physical layout we drive by hand. */
#define DESC_PHYS       0x00200000u
#define BUF_PHYS        0x00201000u
#define BM_TEST_LEN     60u

/* The probe frame the driver downloads: dst = broadcast, rest a recognizable pattern. */
static uint8_t make_pattern(unsigned i)
{
    return (i < 6) ? 0xFF : (uint8_t)(0xA0 + (i & 0x1F));
}

static void test_txdma_dncomplete(void)
{
    int sockets[2];
    int ret = socketpair(PF_UNIX, SOCK_STREAM, 0, sockets);
    g_assert_cmpint(ret, !=, -1);

    QTestState *qts = qtest_initf(
        "-machine pc -m 128 "
        "-device 3c515,netdev=n1,iobase=0x%x,irq=10 "
        "-netdev socket,id=n1,fd=%d", IOBASE, sockets[1]);

    qtest_outw(qts, REG_CMD_STATUS, CMD_TX_ENABLE);

    /* EL3DownDesc { u32 next; u32 status; u32 addr; u32 length; } little-endian. */
    qtest_writel(qts, DESC_PHYS + 0,  0);            /* next   = 0 (single transfer) */
    qtest_writel(qts, DESC_PHYS + 4,  0);            /* status = 0 (clear)           */
    qtest_writel(qts, DESC_PHYS + 8,  BUF_PHYS);     /* addr   = data buffer         */
    qtest_writel(qts, DESC_PHYS + 12, BM_TEST_LEN);  /* length = frame length        */

    uint8_t pattern[BM_TEST_LEN];
    for (unsigned i = 0; i < BM_TEST_LEN; i++) {
        pattern[i] = make_pattern(i);
    }
    qtest_memwrite(qts, BUF_PHYS, pattern, sizeof(pattern));

    /* Post the descriptor and kick the download -- exactly what phase_validate_dma does. */
    qtest_outl(qts, REG_DOWNLISTPTR, DESC_PHYS);
    qtest_outw(qts, REG_CMD_STATUS, CMD_START_DMADOWN);

    /* (1) DN_COMPLETE is written back synchronously by el3_core_dma_tx_single (before the
     * realtiming TxComplete defer) -- this is the bit the cold probe polls. */
    uint32_t desc_status = qtest_readl(qts, DESC_PHYS + 4);
    g_assert_cmphex(desc_status & DESC_DN_COMPLETE, ==, DESC_DN_COMPLETE);

    /* (2) The DMA'd frame reached the wire unchanged (socket netdev: 4-byte BE length prefix). */
    uint32_t be_len = 0;
    ret = recv(sockets[0], &be_len, sizeof(be_len), 0);
    g_assert_cmpint(ret, ==, sizeof(be_len));
    uint32_t wire_len = ntohl(be_len);
    g_assert_cmpint(wire_len, ==, BM_TEST_LEN);

    uint8_t got[BM_TEST_LEN];
    ret = recv(sockets[0], got, sizeof(got), 0);
    g_assert_cmpint(ret, ==, (int)sizeof(got));
    g_assert_cmpmem(got, sizeof(got), pattern, sizeof(pattern));

    qtest_quit(qts);
    close(sockets[0]);
    close(sockets[1]);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/el3/txdma/dncomplete", test_txdma_dncomplete);
    return g_test_run();
}
