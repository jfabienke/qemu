/*
 * QTest for the 3C515 bus-master RX-DMA "UpComplete" handshake.
 *
 * Verifies the contract the 3cpd.exe Phase 8 xms_rx_deliver path depends on:
 *   1. StartDmaUp arms a single-transfer up-descriptor at UpListPtr.
 *   2. An incoming frame is bus-mastered into the descriptor's buffer.
 *   3. The descriptor STATUS is written back as UP_COMPLETE(0x4000) | length(13-bit).
 *   4. The adapter IntStatus raises UpComplete = bit 10 (0x0400) -- NOT RxComplete.
 *   5. AckIntr with an 11-bit param (incl. 0x0400) clears the readable UpComplete bit
 *      (i.e. the param is masked to 0x7FF, not 0xFF) so the IRQ does not storm.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include "qemu/iov.h"
#include "libqtest.h"

/* 3C515 register map (default iobase 0x300). Command/Status is at +0x0E; the bus-master
 * DMA register block is at +0x20, with UpListPtr at region offset 0x18 -> +0x38. */
#define IOBASE          0x300
#define REG_CMD_STATUS  (IOBASE + 0x0E)   /* write = command, read = status */
#define REG_UPLISTPTR   (IOBASE + 0x38)   /* 32-bit RX descriptor pointer */

/* Command encoding: opcode in bits 15..11, param in bits 10..0. */
#define CMD_RX_ENABLE   (0x04 << 11)               /* 0x2000 */
#define CMD_SET_INTENB  (0x0E << 11)               /* 0x7000 | mask */
#define CMD_ACK_INTR    (0x0D << 11)               /* 0x6800 | bits  */
#define CMD_START_DMAUP (0x14 << 11)               /* 0xA000, param 0 = up (RX) */

/* IntStatus bits. */
#define ST_INT_LATCH    0x0001
#define ST_RX_COMPLETE  0x0010
#define ST_UP_COMPLETE  0x0400   /* bit 10 -- the real 3c515/3c59x bus-master RX indication */

/* Descriptor STATUS bits (written back into guest RAM). */
#define DESC_UP_COMPLETE 0x00004000
#define DESC_LEN_MASK    0x00001FFF

/* Guest physical layout we drive by hand (default pc machine, plenty of RAM here). */
#define DESC_PHYS       0x00200000u
#define BUF_PHYS        0x00201000u
#define BUF_CAP         2048u

/* A broadcast Ethernet frame (accepted by the device's default RX filter). 64 bytes so it
 * is not treated as a runt. */
static const uint8_t frame[64] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff,   /* dst = broadcast */
    0x52, 0x54, 0x00, 0x12, 0x34, 0x56,   /* src */
    0x08, 0x00,                           /* ethertype = IPv4 */
    /* payload */
    0xde, 0xad, 0xbe, 0xef, 0x01, 0x02, 0x03, 0x04,
    0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c,
};

static void inject_frame(int fd)
{
    uint32_t be_len = htonl(sizeof(frame));   /* socket netdev: 4-byte BE length prefix */
    struct iovec iov[] = {
        { .iov_base = &be_len, .iov_len = sizeof(be_len) },
        { .iov_base = (void *)frame, .iov_len = sizeof(frame) },
    };
    int ret = iov_send(fd, iov, 2, 0, sizeof(be_len) + sizeof(frame));
    g_assert_cmpint(ret, ==, sizeof(be_len) + sizeof(frame));
}

static void write_up_descriptor(QTestState *qts)
{
    /* EL3UpDesc { u32 next; u32 status; u32 addr; u32 length; } little-endian. */
    qtest_writel(qts, DESC_PHYS + 0,  0);        /* next   = 0 (single transfer) */
    qtest_writel(qts, DESC_PHYS + 4,  0);        /* status = 0 (clear)           */
    qtest_writel(qts, DESC_PHYS + 8,  BUF_PHYS); /* addr   = data buffer         */
    qtest_writel(qts, DESC_PHYS + 12, BUF_CAP);  /* length = buffer capacity     */
}

static void test_rxdma_upcomplete(void)
{
    int sockets[2];
    int ret = socketpair(PF_UNIX, SOCK_STREAM, 0, sockets);
    g_assert_cmpint(ret, !=, -1);

    QTestState *qts = qtest_initf(
        "-machine pc -m 128 "
        "-device 3c515,netdev=n1,iobase=0x%x,irq=10 "
        "-netdev socket,id=n1,fd=%d", IOBASE, sockets[1]);

    /* Arm the RX-DMA single-transfer up-descriptor. */
    qtest_outw(qts, REG_CMD_STATUS, CMD_RX_ENABLE);
    /* SetRxFilter (opcode 0x10) -> promiscuous (0x08): accept the injected frame regardless. */
    qtest_outw(qts, REG_CMD_STATUS, (0x10 << 11) | 0x08);
    write_up_descriptor(qts);
    qtest_outw(qts, REG_CMD_STATUS, CMD_SET_INTENB |
               (ST_RX_COMPLETE | ST_INT_LATCH | ST_UP_COMPLETE));
    qtest_outl(qts, REG_UPLISTPTR, DESC_PHYS);
    qtest_outw(qts, REG_CMD_STATUS, CMD_START_DMAUP);

    /* Deliver a frame on the wire and let the main loop process the socket. */
    inject_frame(sockets[0]);

    uint16_t status = 0;
    for (int i = 0; i < 20; i++) {
        status = qtest_inw(qts, REG_CMD_STATUS);
        if (status & ST_UP_COMPLETE) {
            break;
        }
        qtest_clock_step(qts, 1000000);
    }

    /* (1) The completion indication MUST be UpComplete (bit 10), not RxComplete. */
    g_assert_cmphex(status & ST_UP_COMPLETE, ==, ST_UP_COMPLETE);

    /* (2) The descriptor STATUS was written back as UP_COMPLETE | received length. */
    uint32_t desc_status = qtest_readl(qts, DESC_PHYS + 4);
    g_assert_cmphex(desc_status & DESC_UP_COMPLETE, ==, DESC_UP_COMPLETE);
    g_assert_cmpint(desc_status & DESC_LEN_MASK, ==, sizeof(frame));

    /* (3) The frame bytes were DMA'd into the descriptor's buffer. */
    uint8_t got[sizeof(frame)];
    qtest_memread(qts, BUF_PHYS, got, sizeof(got));
    g_assert_cmpmem(got, sizeof(got), frame, sizeof(frame));

    /* (4) AckIntr with an 11-bit param (incl. 0x0400) clears the readable UpComplete bit --
     * proves the AckIntr mask reaches bit 10 (param & 0x7FF, not & 0xFF) so no IRQ storm. */
    qtest_outw(qts, REG_CMD_STATUS, CMD_ACK_INTR | 0x07FF);
    status = qtest_inw(qts, REG_CMD_STATUS);
    g_assert_cmphex(status & ST_UP_COMPLETE, ==, 0);

    qtest_quit(qts);
    close(sockets[0]);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/el3/rxdma/upcomplete", test_rxdma_upcomplete);
    return g_test_run();
}
