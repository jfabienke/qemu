/*
 * QEMU 3Com EtherLink III core emulation
 *
 * Copyright (c) 2024 QEMU contributors
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "hw/net/el3_core.h"
#include "hw/core/irq.h"
#include "net/net.h"
#include "net/eth.h"
#include "qemu/log.h"
#include "exec/memattrs.h"
#include "exec/icount.h"   /* icount_get_raw(): separate instruction-execution time from idle-warp (research) */
#include "qemu/atomic.h"
#include "qemu/aio.h"
/* #include "trace.h" - disabled for now */
#define trace_el3_eeprom_cmd(...)
#define trace_el3_eeprom_read(...)
#define trace_el3_filter_set(...)
#define trace_el3_rx_drop(...)
#define trace_el3_rx_filter_drop(...)
#define trace_el3_rx_error(...)
#define trace_el3_rx_loopback(...)
#define trace_el3_rx_packet(...)
#define trace_el3_rx_overflow(...)
#define trace_el3_dma_ring_set(...)
#define trace_el3_dma_start(...)
#define trace_el3_dma_error(...)
#define trace_el3_dma_desc_read(...)
#define trace_el3_dma_desc_status_write(...)
#define trace_el3_dma_budget_refill(...)
#define trace_el3_dma_stalled(...)
#define trace_el3_dma_not_owned(...)
#define trace_el3_dma_budget_exceeded(...)
#define trace_el3_link_status(...)
#define trace_el3_tx_oversize(...)
#define trace_el3_tx_padding(...)
#define trace_el3_tx_packet(...)
#define trace_el3_read(...)
#define trace_el3_write(...)
#define trace_el3_fifo_write(...)
#define trace_el3_tx_underrun(...)
#define trace_el3_id_write(...)
#define trace_el3_autoneg_complete(...)
#define trace_el3_autoneg_restart(...)
#define trace_el3_cmd_ignored(...)
#define trace_el3_command(...)
#define trace_el3_core_reset(...)
#define trace_el3_eeprom_done(...)
#define trace_el3_eeprom_start(...)
#define trace_el3_fifo_read(...)
#define trace_el3_fifo_reset(...)
#define trace_el3_id_read(...)
#define trace_el3_id_reset(...)
#define trace_el3_manual_config(...)
#define trace_el3_mii_command(...)
#define trace_el3_mii_data_read(...)
#define trace_el3_phy_init(...)
#define trace_el3_phy_read(...)
#define trace_el3_phy_write(...)
#define trace_el3_stats_freeze(...)
#define trace_el3_stats_unfreeze(...)
#define trace_el3_window_select(...)
#define trace_el3_tx_complete(...)
#define trace_el3_rx_discard(...)
#define trace_el3_tx_start(...)
#define trace_el3_rx_fifo_write(...)
#define trace_el3_validate_dma_address(...)
#define trace_el3_phy_reset(...)
#define trace_el3_ctx_change(...)
#define trace_el3_bh_schedule_tx(...)
#define trace_el3_bh_schedule_rx(...)
#define trace_el3_tx_blocked_change(...)

/* Memory and DMA functions are in QEMU's standard includes */

/* Stub for AioContext functions if not available */
#ifndef aio_context_acquire
#define aio_context_acquire(ctx) do {} while(0)
#define aio_context_release(ctx) do {} while(0)
#endif

/* Use G_GNUC_UNUSED for marking unused parameters */
#define QEMU_UNUSED G_GNUC_UNUSED

/* Ethernet constants */
#ifndef ETH_MAX_FRAME_LEN
#define ETH_MAX_FRAME_LEN 1518
#endif
/* Stage 2 large-frame (FDDI-sized) ceiling: 4 KB NVMe page + IP/TCP/NVMe-TCP headers + slack.
 * The bus-master parts support <=4494 B via allowLargePackets; size the DMA staging buffer for it.
 * (Stage 2b: TX staging, RX-FIFO growth, and allowLargePackets RX gating are all landed.) */
#define EL3_LARGE_FRAME_MAX 4608
#ifndef ETH_MIN_DATA_NOFCS
#define ETH_MIN_DATA_NOFCS 46
#endif

/* Status bits */
#define STAT_UPDATE        0x0080  /* Statistics updated */

/* DMA states */
#define DMA_IDLE           0
#define DMA_DOWNLOADING    1
#define DMA_UPLOADING      2
#define DMA_ERROR          3

/* Descriptor status bits */
#define DESC_DN_COMPLETE   0x00010000  /* Download complete */
#define DESC_UP_COMPLETE   0x00010000  /* Upload complete */

/* NET_DIAG register bits */
#define NET_DIAG_LINK_BEAT_OK  0x0800
#define NET_DIAG_TX_FUSE_OK    0x0400

/* Ethernet helper functions */
static inline bool eth_addr_is_broadcast(const uint8_t *addr)
{
    return (addr[0] & addr[1] & addr[2] & addr[3] & addr[4] & addr[5]) == 0xff;
}

static inline bool eth_addr_is_multicast(const uint8_t *addr)
{
    return addr[0] & 0x01;
}

static inline bool eth_addr_equals(const uint8_t *addr1, const uint8_t *addr2)
{
    return memcmp(addr1, addr2, 6) == 0;
}

/* Debug logging */
#define EL3_DEBUG 0
#define DPRINTF(fmt, ...) do { \
    if (EL3_DEBUG) { \
        qemu_log("el3_core: " fmt, ## __VA_ARGS__); \
    } } while (0)

/* EEPROM timing - 162us per operation per hardware spec */
#define EEPROM_DELAY_NS 162000

/* Forward declarations for DMA scheduling functions */
static void el3_dma_schedule_tx(EL3DMAEngine *dma);
static void el3_dma_schedule_rx(EL3DMAEngine *dma);

/* Descriptor-RING DMA scaffolding (el3_dma_*_bh, el3_dma_schedule_*) is a later milestone and
 * not yet wired up; these stubs return a constant error so the compiler dead-code-eliminates
 * that unreachable scaffolding (it references el3_core_set_status, which lives in the
 * not-yet-compiled el3_core_full.c). The live single-transfer path (el3_core_dma_tx_single)
 * does NOT use these -- it calls address_space_read/write directly. */
static MemTxResult dma_memory_read(AddressSpace *as, hwaddr addr, void *buf,
                                  hwaddr len, MemTxAttrs attrs)
{
    return MEMTX_ERROR;
}

static MemTxResult dma_memory_write(AddressSpace *as, hwaddr addr, const void *buf,
                                   hwaddr len, MemTxAttrs attrs)
{
    return MEMTX_ERROR;
}

/* Calculate multicast hash index for address filtering
 * Uses CRC32-LE and takes lower 6 bits for 64-bit hash table */
static uint8_t el3_mcast_hash_index(const uint8_t *addr)
{
    uint32_t crc = 0xffffffff;
    
    for (int i = 0; i < ETH_ALEN; i++) {
        uint8_t byte = addr[i];
        for (int j = 0; j < 8; j++) {
            uint32_t bit = (byte ^ crc) & 1;
            crc >>= 1;
            if (bit) {
                crc ^= POLYNOMIAL_LE;
            }
            byte >>= 1;
        }
    }
    
    /* Take lower 6 bits of CRC for hash index */
    return (uint8_t)(crc & 0x3F);
}

/* Model capabilities definitions */
static const EL3Caps el3_caps[] = {
    [MODEL_3C509] = {
        .has_mii = false,
        .has_bus_master = false,
        .has_full_duplex = false,
        .is_100mbit = false,
        .has_vlan_support = false,
        .ram_bytes = 0,
        .tx_fifo_bytes = 2048,
        .rx_fifo_bytes = 4096,
        .max_windows = 8,
    },
    [MODEL_3C509B] = {
        .has_mii = false,
        .has_bus_master = false,
        .has_full_duplex = true,
        .is_100mbit = false,
        .has_vlan_support = false,
        .ram_bytes = 0,
        .tx_fifo_bytes = 2048,
        .rx_fifo_bytes = 4096,
        .max_windows = 8,
    },
    [MODEL_3C515] = {
        .has_mii = true,
        .has_bus_master = true,
        .has_full_duplex = true,
        .is_100mbit = true,
        .has_vlan_support = false,
        .ram_bytes = 0,
        .tx_fifo_bytes = 2048,
        .rx_fifo_bytes = 2048,
        .max_windows = 8,
    },
    [MODEL_3C905] = {
        .has_mii = true,
        .has_bus_master = true,
        .has_full_duplex = true,
        .is_100mbit = true,
        .has_vlan_support = false,
        .ram_bytes = 8192,
        .tx_fifo_bytes = 0,  /* Uses RAM buffer */
        .rx_fifo_bytes = 0,  /* Uses RAM buffer */
        .max_windows = 8,
    },
    [MODEL_3C905B] = {
        .has_mii = true,
        .has_bus_master = true,
        .has_full_duplex = true,
        .is_100mbit = true,
        .has_vlan_support = true,
        .ram_bytes = 8192,
        .tx_fifo_bytes = 0,
        .rx_fifo_bytes = 0,
        .max_windows = 8,
    },
};

/* TX completion callback */
static void el3_tx_completion_cb(NetClientState *nc, ssize_t ret)
{
    EL3Core *c = qemu_get_nic_opaque(nc);
    
    /* Clear the TX blocked flag only on successful transmission */
    if (ret > 0) {
        bool was_blocked = c->tx_blocked;
        c->tx_blocked = false;
        if (was_blocked) {
            trace_el3_tx_blocked_change(false);
        }
    } else if (ret < 0) {
        /* Handle error conditions (peer removal, disconnect, etc.) */
        qemu_log_mask(LOG_GUEST_ERROR, "el3: TX completion error: %zd\n", ret);
        /* Don't clear tx_blocked on errors - let the driver handle recovery */
    }
    
    /* Resume TX processing if needed */
    if (c->tx_in_progress) {
        qemu_bh_schedule(c->tx_bh);
    }
    
    /* Kick DMA if it was waiting */
    if (c->ops && c->ops->tx_kick) {
        c->ops->tx_kick(c);
    }
}

/* Transmit a packet (with backpressure handling) */
/* Deliver queued reflector replies from a virtual-clock timer (not reentrantly from inside the TX,
 * which raced the IRQ/FIFO). A timer fires reliably under -icount -- on the idle-warp when the guest
 * hlts, or as a spin advances the clock -- unlike a BH that waits for a vCPU exit. */
static void el3_l4dbg_timer_cb(void *opaque)
{
    EL3Core *c = (EL3Core *)opaque;
    uint16_t s[5] = {0};
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (c->dma_as) {
        address_space_read(c->dma_as, 0x4F0, MEMTXATTRS_UNSPECIFIED, s, sizeof(s));
    }
    fprintf(stderr, "[L4ST] pumps=%u una=%u nxt=%u end=%u wnd=%u | l4tx=%llu t=%.3fs\n",
            s[0], s[1], s[2], s[3], s[4], (unsigned long long)c->l4tx_bytes,
            c->l4tx_start_ns ? (double)(now - c->l4tx_start_ns) / 1e9 : 0.0);
    if (!c->l4tx_reported) {
        timer_mod_ns(c->l4dbg_timer, now + 500000000LL);
    }
}

static void el3_reflect_timer_cb(void *opaque)
{
    EL3Core *c = (EL3Core *)opaque;
    while (c->reflect_head != c->reflect_tail) {
        int slot = c->reflect_head;
        if (el3_core_receive(qemu_get_queue(c->nic), c->reflect_q[slot], c->reflect_qlen[slot]) < 0) {
            c->reflect_rxdrop++;                 /* DIAG: the NIC refused the reply (RX FIFO full) -> lost */
        }
        c->reflect_head = (c->reflect_head + 1) % 32;
    }
}

/* 16-bit ones-complement sum helper for the reflector's IP/TCP checksums. */
static uint16_t el3_csum16(uint32_t sum, const uint8_t *p, int n)
{
    int i;
    for (i = 0; i + 1 < n; i += 2) {
        sum += ((uint16_t)p[i] << 8) | p[i + 1];
    }
    if (i < n) {
        sum += (uint16_t)p[i] << 8;
    }
    while (sum >> 16) {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    return (uint16_t)(~sum & 0xFFFF);
}

/* Test-only in-emulator TCP-ACK reflector. Parses the guest's outgoing TCP segment and synthesizes
 * the reply ACK (SYN -> SYN-ACK w/ MSS 1460; data/FIN -> ACK) entirely in virtual time, delivered
 * straight back through the RX path. Stateless: our seq = the segment's ack field (what the peer
 * expects from us), our ack = their seq + payload. Keeps the guest's window open so it blasts at the
 * TX wire/bus rate -> the L4 TX goodput is measurable under -icount with no external peer. */
/* The MSS the in-emulator TCP peers (tcpreflect, nvmet) advertise: an FDDI-sized frame's payload when
 * the guest driver set allowLargePackets (the MAC then accepts <= EL3_LARGE_RX_MAX), else standard
 * Ethernet. They send at most min(this, the guest's own SYN MSS) -- TCP-correct large-frame modelling. */
static inline uint16_t el3_peer_mss(const EL3Core *c)
{
    return (c->windows[3][W3_MAC_CONTROL >> 1] & MAC_CTRL_ALLOW_LARGE)
           ? (uint16_t)(EL3_LARGE_RX_MAX - 54) : (uint16_t)1460;
}

/* The MSS option of a guest SYN (tcp = TCP header, hlen = its length); 1460 if absent. */
static uint16_t el3_syn_mss(const uint8_t *tcp, int hlen)
{
    int i = 20;
    while (i + 1 < hlen) {
        uint8_t kind = tcp[i];
        if (kind == 0) { break; }                /* end of options */
        if (kind == 1) { i++; continue; }        /* NOP */
        if (tcp[i + 1] < 2) { break; }           /* malformed length */
        if (kind == 2 && tcp[i + 1] == 4 && i + 3 < hlen) {
            return (uint16_t)((tcp[i + 2] << 8) | tcp[i + 3]);
        }
        i += tcp[i + 1];
    }
    return 1460;
}

static bool el3_tcp_reflect(EL3Core *c, const uint8_t *buf, size_t len)
{
    const uint8_t *ip, *tcp;
    int ihl, tcp_off, ip_total, payload, rhlen;
    uint8_t flags, fl;
    uint32_t their_seq, their_ack, our_seq, our_ack;
    uint8_t r[64];
    uint8_t *rip, *rtcp;

    if (len < 54 || buf[12] != 0x08 || buf[13] != 0x00) {
        return false;                            /* not IPv4 -> let ARP/other go to the backend */
    }
    ip = buf + 14;
    ihl = (ip[0] & 0x0F) * 4;
    if (ip[9] != 6 || (size_t)(14 + ihl + 20) > len) {
        return false;                            /* not TCP / truncated -> pass through */
    }
    tcp = ip + ihl;
    tcp_off = (tcp[12] >> 4) * 4;
    ip_total = (ip[2] << 8) | ip[3];
    payload = ip_total - ihl - tcp_off;
    flags = tcp[13];
    their_seq = ((uint32_t)tcp[4] << 24) | (tcp[5] << 16) | (tcp[6] << 8) | tcp[7];
    their_ack = ((uint32_t)tcp[8] << 24) | (tcp[9] << 16) | (tcp[10] << 8) | tcp[11];

    /* L4-RX sliding window: the guest's ACK ack-field IS its rcv_nxt (una). Feed it back so the
     * generator slides its window forward. delivered = rx_gen_seq - initial measures the true in-order
     * accept rate (immune to drops/retransmits). A forward advance resets the go-back-N stall timer. */
    if (c->rx_gen_l4 && c->rx_gen_start_ns && !c->rx_gen_reported && (flags & 0x10)) {
        if ((int32_t)(their_ack - c->rx_gen_seq) > 0) {   /* forward only; ignore dup/stale ACKs */
            c->rx_gen_seq = their_ack;
            c->rx_gen_stall = 0;
        }
    }

    if ((flags & 0x04)) {                        /* RST: consume, no reply */
        return true;
    }
    /* Only the handshake/teardown needs a reply: SYN -> SYN-ACK, FIN -> ACK. Data and pure-ACK
     * segments are consumed silently (counted in el3_l4tx_account on the TX path). Replying to every
     * data segment would storm RX with ACKs -- the rawblast measurement ignores flow control, so the
     * guest never reads them; the flood only burns wall time and starves the TX drain. */
    if (!(flags & 0x03)) {                       /* not SYN and not FIN (pure data / pure ACK) */
        /* Default: consume silently (rawblast ignores flow control -> ACK injection only burns time).
         * With reflectack on, ACK data segments (delayed, every 2nd) so a REAL flow-controlled sender
         * (mTCP SpdTest, our tcp_send) keeps its window open. Pure ACKs (no payload) stay silent. */
        if (!c->reflect_ack || payload == 0) {
            return true;
        }
        /* DIAG: count data segments + detect retransmits (seq at/below the in-order high-water). */
        c->reflect_segs++;
        if ((int32_t)(their_seq + (uint32_t)payload - c->reflect_hiseq) > 0) {
            c->reflect_hiseq = their_seq + (uint32_t)payload;
        } else {
            c->reflect_retrans++;
        }
        /* Coalesce to ONE pending cumulative ACK: TCP ACKs are cumulative, so a single ACK carrying
         * the latest their_seq+payload advances the sender's window past everything sent so far. This
         * models a prompt "fast peer" without flooding per-segment ACKs (which overflow the 32-entry
         * reflect queue + the RX FIFO on a 40-segment window -> dropped ACKs -> RTO stalls: the sender
         * bursts then crawls). With one cumulative ACK in flight, the window advances every delivery. */
        if (c->reflect_head != c->reflect_tail) {
            return true;                         /* an ACK is already queued -> it covers this seg too */
        }
        /* fall through -> enqueue one cumulative ACK (our_ack = their_seq + payload) */
    }
    if ((flags & 0x02) && !(flags & 0x10)) {     /* SYN, no ACK -> SYN-ACK with MSS */
        our_seq = 0x00010000;
        our_ack = their_seq + 1;
        fl = 0x12;                               /* SYN|ACK */
        rhlen = 24;                              /* 20 + MSS option */
        c->reflect_segs = 0; c->reflect_retrans = 0; c->reflect_rxdrop = 0;   /* DIAG: reset per connection */
        c->reflect_hiseq = their_seq + 1;              /* data starts after the SYN */
    } else {                                     /* data / pure ACK / FIN -> ACK */
        if ((flags & 0x01) && c->reflect_segs) { /* DIAG: FIN -> dump the data-stream stats */
            fprintf(stderr, "[REFLECT] data segs=%llu retrans=%llu (%.1f%% retransmitted) acks_lost=%llu "
                    "rx_overruns=%llu\n",
                    (unsigned long long)c->reflect_segs, (unsigned long long)c->reflect_retrans,
                    100.0 * (double)c->reflect_retrans / (double)c->reflect_segs,
                    (unsigned long long)c->reflect_rxdrop, (unsigned long long)c->stats.rx_overruns);
        }
        our_seq = their_ack;                     /* what the peer expects from us */
        our_ack = their_seq + (uint32_t)payload + ((flags & 0x01) ? 1u : 0u);  /* +1 over FIN */
        fl = (flags & 0x01) ? 0x11 : 0x10;       /* FIN -> FIN|ACK (complete the close so the peer
                                                  * doesn't sit in FIN_WAIT_2 for TCP_CLOSE_TIMEOUT);
                                                  * otherwise a plain ACK */
        rhlen = 20;
    }

    memset(r, 0, sizeof(r));
    memcpy(r + 0, buf + 6, 6);                   /* eth dst = their src */
    memcpy(r + 6, buf + 0, 6);                   /* eth src = their dst */
    r[12] = 0x08; r[13] = 0x00;
    rip = r + 14;
    rip[0] = 0x45; rip[2] = 0; rip[3] = (uint8_t)(20 + rhlen);
    rip[8] = 64; rip[9] = 6;
    memcpy(rip + 12, ip + 16, 4);                /* src ip = their dst */
    memcpy(rip + 16, ip + 12, 4);                /* dst ip = their src */
    { uint16_t ck = el3_csum16(0, rip, 20); rip[10] = ck >> 8; rip[11] = ck & 0xFF; }
    rtcp = rip + 20;
    memcpy(rtcp + 0, tcp + 2, 2);                /* src port = their dst */
    memcpy(rtcp + 2, tcp + 0, 2);                /* dst port = their src */
    rtcp[4] = our_seq >> 24; rtcp[5] = our_seq >> 16; rtcp[6] = our_seq >> 8; rtcp[7] = our_seq;
    rtcp[8] = our_ack >> 24; rtcp[9] = our_ack >> 16; rtcp[10] = our_ack >> 8; rtcp[11] = our_ack;
    rtcp[12] = (uint8_t)((rhlen / 4) << 4);
    rtcp[13] = fl;
    rtcp[14] = 0xFF; rtcp[15] = 0xFF;            /* advertise a 64K window */
    if (rhlen == 24) {                           /* SYN-ACK: advertise what our MAC can carry */
        uint16_t pm = el3_peer_mss(c);
        rtcp[20] = 2; rtcp[21] = 4; rtcp[22] = pm >> 8; rtcp[23] = pm & 0xFF;
    }
    {   /* TCP checksum over pseudo-header + segment */
        uint32_t ps = 0;
        ps += (rip[12] << 8) | rip[13]; ps += (rip[14] << 8) | rip[15];
        ps += (rip[16] << 8) | rip[17]; ps += (rip[18] << 8) | rip[19];
        ps += 6; ps += rhlen;
        uint16_t ck = el3_csum16(ps, rtcp, rhlen);
        rtcp[16] = ck >> 8; rtcp[17] = ck & 0xFF;
    }
    {
        int flen = 14 + 20 + rhlen;
        int next = (c->reflect_tail + 1) % 32;
        if (flen < 60) {
            flen = 60;                           /* pad to the 60-byte Ethernet minimum (no runt drop) */
        }
        if (next != c->reflect_head) {           /* enqueue; the timer delivers after TX returns */
            memcpy(c->reflect_q[c->reflect_tail], r, flen);
            c->reflect_qlen[c->reflect_tail] = flen;
            c->reflect_tail = next;
            /* Arm only if not already pending -- re-arming on every send would push the deadline
             * forward under a continuous burst so it never fires (ACKs never delivered -> crawl). */
            if (!timer_pending(c->reflect_timer)) {
                timer_mod_ns(c->reflect_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000);
            }
        }
    }
    return true;
}

/* ===========================================================================
 * In-emulator NVMe/TCP target (nvmet prop). Terminates TCP to port 4420 and answers the DOS
 * initiator's NVMe/TCP PDUs entirely in virtual time -- the rawblast model applied to storage, so the
 * path is deterministic under -icount with no slirp/host peer. Mirrors tests/nvmetgt.py's serve()
 * state machine in C. NVMe/TCP fields are LITTLE-endian (distinct from the big-endian IP/TCP headers).
 * =========================================================================== */
static void nvt_le16(uint8_t *p, uint16_t v) { p[0] = v & 0xFF; p[1] = v >> 8; }
static void nvt_le32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
static uint16_t nvt_ge16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t nvt_ge32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* Tiny LBA-addressed backing store (first NVMET_DISK_BLOCKS blocks) so a write then read of the same
 * low LBA round-trips real data; higher LBAs read back zero (the bench/verify touches LBA 0). 4096-B
 * blocks to match the namespace's LBADS=12. */
#define NVMET_DISK_BLOCKS 16
static void el3_nvmet_disk_write(EL3Core *c, uint32_t slba, uint32_t off, const uint8_t *src, uint32_t n)
{
    uint64_t pos = (uint64_t)slba * 4096 + off;
    uint64_t cap = (uint64_t)NVMET_DISK_BLOCKS * 4096;
    if (pos >= cap) { return; }
    if (pos + n > cap) { n = (uint32_t)(cap - pos); }
    memcpy((uint8_t *)c->nvmet_disk + pos, src, n);
}
static void el3_nvmet_disk_read(EL3Core *c, uint32_t slba, uint8_t *dst, uint32_t n)
{
    uint64_t pos = (uint64_t)slba * 4096;
    uint64_t cap = (uint64_t)NVMET_DISK_BLOCKS * 4096;
    memset(dst, 0, n);
    if (pos >= cap) { return; }
    if (pos + n > cap) { n = (uint32_t)(cap - pos); }
    memcpy(dst, (uint8_t *)c->nvmet_disk + pos, n);
}

/* NVMe I/O goodput tally over a virtual-time window -> [NVME], mirroring [L4TX]. Counts Read C2HData +
 * Write H2CData payload (the data the storage path actually moves). Reports once when EITHER 1 virtual
 * second elapses OR 1 MiB has been served, so a bounded guest bench produces exactly one line. */
static void el3_nvmet_io_account(EL3Core *c, uint32_t bytes)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (c->nvmet_io_start_ns == 0) {
        c->nvmet_io_start_ns = now;
        c->nvmet_io_bytes = 0;
        /* icount_get_raw() is instruction-execution time only (no idle-warp bias); QEMU_CLOCK_VIRTUAL
         * (now) = that + the idle-warp. Sampling both at the window edges splits guest BUSY vs IDLE. */
        c->nvmet_icount0 = icount_enabled() ? icount_get_raw() : 0;
    }
    c->nvmet_io_bytes += bytes;
    if (!c->nvmet_io_reported &&
        ((now - c->nvmet_io_start_ns) >= 1000000000LL || c->nvmet_io_bytes >= (1u << 20))) {
        double secs = (double)(now - c->nvmet_io_start_ns) / 1e9;
        double mbps = secs > 0 ? (double)c->nvmet_io_bytes / secs / 1e6 : 0;
        int64_t busy_ns = icount_enabled() ? (icount_get_raw() - c->nvmet_icount0) : 0;
        int64_t win_ns  = now - c->nvmet_io_start_ns;
        double busy_pct = win_ns > 0 ? 100.0 * (double)busy_ns / (double)win_ns : 0;
        /* idle = window - productive delivery time (pace). Confound-free guest-gated idle: pace is the
         * same for PIO and conv, so the idle delta == the total delta. interrd is its inter-read subset. */
        double idle_ms = (double)(win_ns - (int64_t)c->nvmet_pace_ns) / 1e6;
        fprintf(stderr, "[NVME] io_payload=%llu over %.3fs -> %.2f MB/s (%.2f Mbit/s) "
                "[deliv=%llu rtx=%llu qdrop=%llu retry=%llu zwin=%llu busy=%.1f%% idle=%.0fms interrd=%.0fms stalls=%llu]\n",
                (unsigned long long)c->nvmet_io_bytes, secs, mbps, mbps * 8.0,
                (unsigned long long)c->nvmet_deliveries, (unsigned long long)c->nvmet_rtx_fires,
                (unsigned long long)c->nvmet_qdrops, (unsigned long long)c->nvmet_retries,
                (unsigned long long)c->nvmet_zwin, busy_pct, idle_ms, (double)c->nvmet_gap_ns / 1e6,
                (unsigned long long)c->nvmet_stalls);
        c->nvmet_io_reported = true;
    }
}

/* Deliver queued NVMe response segments from the virtual clock (not reentrantly from the TX path,
 * which raced the IRQ/FIFO -- same discipline as the reflector). */
/* The 3c509/3c515 RX path holds only a shallow run of frames, so a multi-segment PDU (the 4096-B
 * Identify = 3 segments) delivered back-to-back overruns it and the later segments drop -- one drop
 * stalls in-order TCP forever. So the target is a STOP-AND-WAIT reliable sender: emit one segment,
 * wait for the guest's ACK before the next (el3_nvmet_emit), and retransmit an unacked segment on a
 * timer (this RTX) in case even that one dropped. The same closed-loop discipline as the L4-RX gen.
 * The re-delivery's RX IRQ also wakes a guest that HLTed (tcp_read's cpu_yield) waiting on the ACK. */
#define EL3_NVMET_RTX_NS 200000000    /* re-deliver an unacked in-flight segment after 200 ms */
#define EL3_NVMET_PERSIST_NS 4000000  /* zero-window persist probe: 4 ms (vs 200 ms RTX) when the guest's
                                       * window is closed -- a slow guest (386 + multi-block) fills its
                                       * rcvbuf and advertises window 0, but its tcp_read drains without
                                       * sending a window update, so the el3 must probe to learn the reopen.
                                       * Probing every 4 ms (not 200 ms) keeps it flowing at the guest's
                                       * real drain rate instead of stalling ~200 ms per window-fill. */
static void el3_nvmet_emit(EL3Core *c, struct EL3NvmetConn *cn);

/* Per-byte delivery pacing (ns) -- the SLOWER of the modeled wire and the 16-bit ISA bus, identical to
 * what the rxgen generator uses (el3_rx_gen_timer_cb). So nvmet RX delivery is bus-realistic: a fast CPU
 * can't pull NVMe data faster than the ISA bus (~48 Mbit on the 3c515; the wire on the PIO-only 3c509).
 * 0 when realtiming is off / rates unset -> the caller keeps the fixed 1 us inter-segment delay. */
static int64_t el3_nvmet_per_byte(EL3Core *c)
{
    int64_t per_byte;
    if (!c->realtiming || !c->tx_ns_per_byte) {
        return 0;
    }
    per_byte = c->tx_ns_per_byte;
    if (c->dma_rate_bps) {
        int64_t isa_ns = 1000000000LL / (int64_t)c->dma_rate_bps;
        if (isa_ns > per_byte) {
            per_byte = isa_ns;
        }
    }
    return per_byte;
}

/* Deliver ONE queued response segment per tick, then re-arm at that segment's wire/ISA transfer time
 * (rxgen-style) so storage RX is bus-paced, not dumped at a fixed 1 us. Realtiming off -> fixed 1 us. */
static void el3_nvmet_timer_cb(void *opaque)
{
    EL3Core *c = (EL3Core *)opaque;
    int slot, len;
    int64_t per_byte;

    if (c->nvmet_head == c->nvmet_tail) {
        return;                                  /* queue empty */
    }
    slot = c->nvmet_head;
    len  = c->nvmet_qlen[slot];
    per_byte = el3_nvmet_per_byte(c);
    if (el3_core_receive(qemu_get_queue(c->nic), c->nvmet_q[slot], len) < 0) {
        /* Conv RX-DMA ring full or the bus-master engine still busy (a window briefly outran the guest's
         * drain): BACKPRESSURE -- keep this segment queued and retry after one segment's bus time rather
         * than advancing head, which would DROP it and force the 200 ms RTX. */
        c->nvmet_retries++;
        timer_mod_ns(c->nvmet_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                     (per_byte ? (int64_t)len * per_byte : 1000));
        return;
    }
    c->nvmet_deliveries++;
    /* This segment's productive transfer time on the bus (len*per_byte). Summed over all deliveries it is
     * the el3's total ACTIVE/pacing time; (window - pace) = ALL guest-gated idle, confound-free (pace is
     * identical for PIO and conv -- same segments, same rate -- so the idle delta == the total delta,
     * unlike interrd which only sees the inter-read subset and so over-states the PIO/conv difference). */
    c->nvmet_pace_ns += (uint64_t)(per_byte ? (int64_t)len * per_byte : 1000);
    c->nvmet_last_deliv_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);   /* for the inter-read gap metric */
    c->nvmet_head = (c->nvmet_head + 1) % 16;
    if (c->nvmet_head != c->nvmet_tail) {        /* more queued -> pace the next by THIS frame's bus time */
        timer_mod_ns(c->nvmet_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                     (per_byte ? (int64_t)len * per_byte : 1000));
    }
}

/* Retransmit any connection's unacked in-flight segment (the guest dropped it / hasn't woken to ACK). */
static void el3_nvmet_rtx_timer_cb(void *opaque)
{
    EL3Core *c = (EL3Core *)opaque;
    int i;
    for (i = 0; i < 2; i++) {
        struct EL3NvmetConn *cn = &c->nvmet_conn[i];
        if (cn->gport && cn->resp_len && (int32_t)(cn->snd_nxt - cn->snd_una) > 0) {
            if (!cn->nvmet_zwin_pending) {
                c->nvmet_rtx_fires++;    /* genuine loss retransmit (a zero-window probe is counted as zwin) */
            }
            cn->snd_nxt = cn->snd_una;   /* Go-Back-N: rewind, then el3_nvmet_emit resends (probe or retransmit) */
            el3_nvmet_emit(c, cn);
        }
    }
}

/* Build ONE reply TCP segment (Eth/IPv4/TCP, addressing reversed from `in`) carrying `paylen` payload
 * bytes with the given seq/ack/flags, and queue it for virtual-clock delivery. mss adds the MSS option
 * (SYN-ACK only; el3_peer_mss: 1460, or FDDI-sized with allowLargePackets). */
static void el3_nvmet_seg(EL3Core *c, const uint8_t *in, uint32_t seq, uint32_t ack,
                          uint8_t fl, bool mss, const uint8_t *payload, int paylen)
{
    const uint8_t *ip = in + 14;
    int ihl = (ip[0] & 0x0F) * 4;
    const uint8_t *tcp = ip + ihl;
    int rhlen = mss ? 24 : 20;
    int seglen = 14 + 20 + rhlen + paylen;
    uint8_t r[EL3_LARGE_FRAME_MAX];              /* one Ethernet frame (eth + IP + TCP + <=MSS payload) */
    uint8_t *rip, *rtcp;

    if (seglen > (int)sizeof(r)) {
        return;                                  /* oversized -> drop (caller fragments to <= MSS) */
    }
    memset(r, 0, sizeof(r));
    memcpy(r + 0, in + 6, 6);                    /* eth dst = their src */
    memcpy(r + 6, in + 0, 6);                    /* eth src = their dst */
    r[12] = 0x08; r[13] = 0x00;
    rip = r + 14;
    rip[0] = 0x45;
    rip[2] = (uint8_t)((20 + rhlen + paylen) >> 8);
    rip[3] = (uint8_t)(20 + rhlen + paylen);
    rip[8] = 64; rip[9] = 6;
    memcpy(rip + 12, ip + 16, 4);                /* src ip = their dst */
    memcpy(rip + 16, ip + 12, 4);                /* dst ip = their src */
    { uint16_t ck = el3_csum16(0, rip, 20); rip[10] = ck >> 8; rip[11] = ck & 0xFF; }
    rtcp = rip + 20;
    memcpy(rtcp + 0, tcp + 2, 2);                /* src port = their dst (4420) */
    memcpy(rtcp + 2, tcp + 0, 2);                /* dst port = their src */
    rtcp[4] = seq >> 24; rtcp[5] = seq >> 16; rtcp[6] = seq >> 8; rtcp[7] = seq;
    rtcp[8] = ack >> 24; rtcp[9] = ack >> 16; rtcp[10] = ack >> 8; rtcp[11] = ack;
    rtcp[12] = (uint8_t)((rhlen / 4) << 4);
    rtcp[13] = fl;
    rtcp[14] = 0xFF; rtcp[15] = 0xFF;            /* advertise a 64K window */
    if (mss) {
        uint16_t pm = el3_peer_mss(c);
        rtcp[20] = 2; rtcp[21] = 4; rtcp[22] = pm >> 8; rtcp[23] = pm & 0xFF;
    }
    if (paylen) { memcpy(rtcp + rhlen, payload, paylen); }
    {
        uint32_t ps = 0;
        ps += (rip[12] << 8) | rip[13]; ps += (rip[14] << 8) | rip[15];
        ps += (rip[16] << 8) | rip[17]; ps += (rip[18] << 8) | rip[19];
        ps += 6; ps += rhlen + paylen;
        uint16_t ck = el3_csum16(ps, rtcp, rhlen + paylen);
        rtcp[16] = ck >> 8; rtcp[17] = ck & 0xFF;
    }
    if (seglen < 60) { seglen = 60; }            /* pad to the Ethernet minimum */
    {
        int next = (c->nvmet_tail + 1) % 16;
        if (next != c->nvmet_head) {
            memcpy(c->nvmet_q[c->nvmet_tail], r, seglen);
            c->nvmet_qlen[c->nvmet_tail] = seglen;
            c->nvmet_tail = next;
            if (!timer_pending(c->nvmet_timer)) {    /* arm at this segment's wire/ISA transfer time */
                int64_t per_byte = el3_nvmet_per_byte(c);
                timer_mod_ns(c->nvmet_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                             (per_byte ? (int64_t)seglen * per_byte : 1000));
            }
        } else {
            c->nvmet_qdrops++;                       /* delivery queue full -> segment lost (-> RTX) */
        }
    }
}

/* Emit response segments up to the in-flight window. Called to start a response, to send more after an
 * ACK slides the window, and (via the RTX timer, which first rewinds snd_nxt=snd_una) to retransmit.
 *
 * COMPACT the fully-acked prefix off the front of resp[] first (memmove + advance resp_base to snd_una).
 * At QD=1 this resets resp_len to 0 once the whole PDU is acked. At QD>1 it frees room so el3_nvmet_queue
 * can keep appending later responses into a bounded buffer. After compaction resp_base==snd_una, so a
 * queued byte's offset in resp[] is (seq - snd_una).
 *
 * Then fill the window: emit MSS segments from snd_nxt while in-flight bytes (snd_nxt - snd_una) stay
 * under min(guest window, EL3_NVMET_WIN_CAP) and unsent bytes remain. The window is enabled ONLY when the
 * deep conv RX-DMA ring is armed AND copybreak routes MSS frames to it (rx_copybreak < MSS) -- on the
 * shallow PIO RX FIFO (the default storage path) a back-to-back batch overruns it and drops, so there it
 * stays one MSS = stop-and-wait, exactly the original behavior. (The guest's tcp_pump must bound its RX
 * drain, else the continuous windowed delivery livelocks it -- see tcp.c.) */
static void el3_nvmet_emit(EL3Core *c, struct EL3NvmetConn *cn)
{
    uint32_t win, end;
    uint32_t mss = cn->mss ? cn->mss : 1460u;
    bool ring = c->rx_dma_armed && c->rx_copybreak < mss;    /* MSS frames land on the deep conv ring */

    if (cn->resp_len) {
        uint32_t acked = cn->snd_una - cn->resp_base;
        if ((int32_t)acked > 0) {
            if (acked >= cn->resp_len) {
                cn->resp_len = 0;                /* whole queued stream acked -> done */
            } else {
                memmove(cn->resp, cn->resp + acked, cn->resp_len - acked);
                cn->resp_len -= acked;
            }
            cn->resp_base = cn->snd_una;
        }
    }
    if (cn->resp_len == 0) {
        return;
    }
    win = ring ? cn->snd_wnd : mss;              /* conv ring -> windowed; PIO FIFO -> one-MSS stop-and-wait */
    if (win > EL3_NVMET_WIN_CAP) { win = EL3_NVMET_WIN_CAP; }
    if (win < mss) { win = mss; }                /* always allow one MSS so a tight window still progresses */
    end = cn->resp_base + cn->resp_len;          /* one past the last queued byte */

    while ((int32_t)(cn->snd_nxt - end) < 0 &&   /* unsent bytes remain ... */
           (cn->snd_nxt - cn->snd_una) < win) {  /* ... and the in-flight window has room */
        uint32_t off = cn->snd_nxt - cn->resp_base;
        int chunk = (int)(end - cn->snd_nxt);
        if (chunk > (int)mss) { chunk = (int)mss; }  /* one MSS per segment */
        el3_nvmet_seg(c, cn->tmpl, cn->snd_nxt, cn->rcv_nxt, 0x10 /*ACK*/, false, cn->resp + off, chunk);
        cn->snd_nxt += (uint32_t)chunk;
    }
    /* Re-arm the RTX. If the guest's window is closed (snd_wnd < one MSS) while we still have unsent
     * bytes, that's a zero-window stall, not a packet loss: probe FAST (persist) so we learn the reopen
     * from the guest's re-ACK instead of waiting the full 200 ms RTX. (The slow 386 fills its rcvbuf on a
     * long multi-block read; its tcp_read drains it but sends no window update.) */
    {
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        if (cn->snd_wnd < 1460u && (int32_t)(cn->snd_nxt - end) < 0) {
            cn->nvmet_zwin_pending = true;   /* the next RTX fire is a window probe, not a loss retransmit */
            c->nvmet_zwin++;
            timer_mod_ns(c->nvmet_rtx_timer, now + EL3_NVMET_PERSIST_NS);
        } else {
            cn->nvmet_zwin_pending = false;
            timer_mod_ns(c->nvmet_rtx_timer, now + EL3_NVMET_RTX_NS);
        }
    }
}

/* Queue a complete response PDU for paced delivery to the connection, then kick the sender.
 *
 * Queue depth 1 (the lock-step initiator): the previous response is always fully acked (resp_len==0)
 * before the next command arrives, so this starts a fresh response and emits its first segment.
 *
 * Queue depth > 1 (pipelined reads): several commands arrive together, so a response is still in
 * flight (resp_len!=0) when the next is queued. APPEND it to the resp byte-stream -- the stop-and-wait
 * sender then walks straight from the in-flight segment into the appended bytes on each ACK, with no
 * inter-command idle gap (the gap that strands a slow guest on the BIOS timer tick at QD=1). Only kick
 * emit() when nothing is in flight (snd_una==snd_nxt); otherwise the in-flight ACK drives it forward
 * (calling emit() with a segment in flight would spuriously retransmit from snd_una). */
static void el3_nvmet_queue(EL3Core *c, struct EL3NvmetConn *cn, const uint8_t *pdu, int pdulen)
{
    if (cn->resp_len != 0) {                          /* QD>1: a response is still streaming -> append */
        if (cn->resp_len + (uint32_t)pdulen > sizeof(cn->resp)) {
            return;                                  /* >EL3_NVMET_QD_MAX in flight: drop (initiator stalls -> visible) */
        }
        memcpy(cn->resp + cn->resp_len, pdu, pdulen);
        cn->resp_len += (uint32_t)pdulen;
        if (cn->snd_una == cn->snd_nxt) {            /* sender idle between PDUs -> kick; else ACK drives it */
            el3_nvmet_emit(c, cn);
        }
        return;
    }
    /* Fresh response (resp_len was 0): the el3 was idle since the previous read's last delivery, waiting
     * for the guest to drain+complete it and issue this command -- the inter-read gap. Tally it. */
    if (c->nvmet_last_deliv_ns) {
        int64_t gap = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - c->nvmet_last_deliv_ns;
        c->nvmet_gap_ns += (uint64_t)gap;
        if (gap > 10000000LL) {     /* >10 ms == a BIOS-tick (55 ms) wake stall, not a normal turnaround */
            c->nvmet_stalls++;
        }
    }
    if (pdulen > (int)sizeof(cn->resp)) {
        pdulen = (int)sizeof(cn->resp);
    }
    memcpy(cn->resp, pdu, pdulen);
    cn->resp_len = (uint32_t)pdulen;
    cn->resp_base = cn->snd_nxt;
    cn->snd_una = cn->snd_nxt;
    el3_nvmet_emit(c, cn);
}

/* CapsuleResp (24-B CQE) -- the completion for Connect / PropSet / PropGet / Write. */
static void el3_nvmet_capresp(EL3Core *c, struct EL3NvmetConn *cn,
                              uint16_t cid, uint32_t result, uint16_t status)
{
    uint8_t r[24];
    memset(r, 0, sizeof(r));
    r[0] = 0x05; r[2] = 24; nvt_le32(r + 4, 24);
    nvt_le32(r + 8, result);
    nvt_le16(r + 20, cid);
    nvt_le16(r + 22, status);
    el3_nvmet_queue(c, cn, r, 24);
}

/* Handle one fully-received NVMe/TCP PDU: ICReq, Fabrics Connect/PropSet/PropGet, Identify, NVM
 * Read (-> C2HData) / Write (-> R2T) CapsuleCmds, and a Write's H2CData (-> CQE). Mirrors nvmetgt.py. */
static void el3_nvmet_one(EL3Core *c, struct EL3NvmetConn *cn,
                          const uint8_t *p, int plen)
{
    uint8_t ptype = p[0];

    if (ptype == 0x00) {                         /* ICReq -> ICResp (128 B) */
        uint8_t r[128];
        memset(r, 0, sizeof(r));
        r[0] = 0x01; r[2] = 128; nvt_le32(r + 4, 128);
        nvt_le32(r + 12, 131072);                /* maxh2cdata */
        el3_nvmet_queue(c, cn,r, 128);
        return;
    }
    if (ptype == 0x04 && plen >= 72) {           /* CapsuleCmd: 8-B hdr + 64-B SQE */
        const uint8_t *sqe = p + 8;
        uint8_t opc = sqe[0];
        uint16_t cid = nvt_ge16(sqe + 2);
        uint8_t fctype = sqe[4];

        if (opc == 0x7F && fctype == 0x01) {                 /* Fabrics Connect */
            el3_nvmet_capresp(c, cn, cid, 1 /*CNTLID*/, 0);
        } else if (opc == 0x7F && fctype == 0x00) {          /* Property Set */
            uint32_t off = nvt_ge32(sqe + 44), val = nvt_ge32(sqe + 48);
            if (off == 0x14 && (val & 1u)) { cn->ready = true; }   /* CC.EN */
            el3_nvmet_capresp(c, cn, cid, 0, 0);
        } else if (opc == 0x7F && fctype == 0x04) {          /* Property Get */
            uint32_t off = nvt_ge32(sqe + 44);
            uint32_t res = (off == 0x1C && cn->ready) ? 1u : 0u;   /* CSTS.RDY */
            el3_nvmet_capresp(c, cn, cid, res, 0);
        } else if (opc == 0x06) {                            /* Identify -> C2HData(4096) */
            uint32_t cns = nvt_ge32(sqe + 40) & 0xFF;
            uint8_t r[24 + 4096];
            memset(r, 0, sizeof(r));
            r[0] = 0x07; r[1] = 0x0C; r[2] = 24; r[3] = 24;  /* DATA_LAST | SUCCESS */
            nvt_le32(r + 4, 24 + 4096);
            nvt_le16(r + 8, cid);
            nvt_le32(r + 16, 4096);                          /* datal (datao = 0) */
            if (cns == 0x01) {                               /* Identify Controller */
                r[24 + 77] = 3;                              /* MDTS = 3 (2^3 pages = 32 KiB = our read cap) */
            } else {                                         /* Identify Namespace */
                nvt_le32(r + 24 + 0, 16384);                 /* NSZE (low dword of u64) */
                nvt_le32(r + 24 + 8, 16384);                 /* NCAP */
                r[24 + 26] = 0;                              /* FLBAS -> LBA format 0 */
                r[24 + 128 + 2] = 12;                        /* LBAF[0].LBADS = 12 -> 4096-B blocks */
            }
            el3_nvmet_queue(c, cn,r, 24 + 4096);
        } else if (opc == 0x01) {                            /* NVM Write -> R2T (then H2CData -> CQE) */
            uint32_t slba = nvt_ge32(sqe + 40);
            uint32_t nlb  = nvt_ge16(sqe + 48);              /* cdw12[15:0] = NLB, 0-based */
            uint32_t xfer = (nlb + 1) * 4096;
            uint8_t r[24];
            cn->w_cid = cid;
            cn->w_slba = slba;
            cn->w_remaining = xfer;
            memset(r, 0, sizeof(r));
            r[0] = 0x09; r[2] = 24; nvt_le32(r + 4, 24);     /* R2T (24-B PDU) */
            nvt_le16(r + 8, cid);                            /* cccid */
            nvt_le16(r + 10, 1);                             /* ttag */
            nvt_le32(r + 12, 0);                             /* r2to */
            nvt_le32(r + 16, xfer);                          /* r2tl = whole transfer in one shot */
            el3_nvmet_queue(c, cn, r, 24);
        } else if (opc == 0x02) {                            /* NVM Read -> C2HData (SUCCESS folded in) */
            uint32_t slba = nvt_ge32(sqe + 40);
            uint32_t nlb  = nvt_ge16(sqe + 48);
            uint32_t xfer = (nlb + 1) * 4096;
            uint8_t r[EL3_NVMET_READ_PDU];                   /* whole multi-block transfer in one C2HData */
            if (xfer > EL3_NVMET_MAX_READ_BLK * 4096u) {     /* clamp to the advertised mdts (8 blocks = 32 KB) */
                xfer = EL3_NVMET_MAX_READ_BLK * 4096u;
            }
            memset(r, 0, 24);
            r[0] = 0x07; r[1] = 0x0C; r[2] = 24; r[3] = 24;  /* C2HData, DATA_LAST | SUCCESS (datao = 0) */
            nvt_le32(r + 4, 24 + xfer);
            nvt_le16(r + 8, cid);                            /* cccid */
            nvt_le32(r + 16, xfer);                          /* datal = the whole transfer */
            el3_nvmet_disk_read(c, slba, r + 24, xfer);      /* fill from backing (zero-filled past it) */
            el3_nvmet_io_account(c, xfer);
            el3_nvmet_queue(c, cn, r, (int)(24 + xfer));
        }
        return;
    }
    if (ptype == 0x06 && plen >= 24) {           /* H2CData: a Write's payload (-> CQE on the last) */
        uint16_t cccid = nvt_ge16(p + 8);
        uint32_t datao = nvt_ge32(p + 12);
        uint32_t dlen  = nvt_ge32(p + 16);
        uint8_t  last  = p[1] & 0x04;            /* DATA_LAST */
        if (dlen > (uint32_t)(plen - 24)) { dlen = (uint32_t)(plen - 24); }
        el3_nvmet_disk_write(c, cn->w_slba, datao, p + 24, dlen);
        el3_nvmet_io_account(c, dlen);
        cn->w_remaining = (cn->w_remaining >= dlen) ? (cn->w_remaining - dlen) : 0;
        if (last || cn->w_remaining == 0) {
            cn->w_remaining = 0;
            el3_nvmet_capresp(c, cn, cccid, 0, 0);   /* CQE success -> Write complete */
        }
        return;
    }
}

/* Accumulate inbound TCP payload and dispatch each COMPLETE NVMe/TCP PDU. Handles multi-segment PDUs:
 * the 4120-B H2CData write spans ~3 segments at MSS 1460 (handshake PDUs are all one-per-segment). If
 * the dispatch queued no response (the PDU is still incomplete, or an interior H2CData), send a bare
 * ACK so the guest's send window opens and it keeps streaming. */
static void el3_nvmet_rx(EL3Core *c, struct EL3NvmetConn *cn, const uint8_t *pl, int payload)
{
    bool had_resp = (cn->resp_len != 0);

    if (cn->rxlen + (uint32_t)payload > sizeof(cn->rx)) {
        cn->rxlen = 0;                           /* overflow guard -> resync (shouldn't happen lock-step) */
        return;
    }
    memcpy(cn->rx + cn->rxlen, pl, payload);
    cn->rxlen += (uint32_t)payload;

    while (cn->rxlen >= 8) {
        uint32_t plen = nvt_ge32(cn->rx + 4);
        if (plen < 8 || plen > sizeof(cn->rx)) { cn->rxlen = 0; break; }   /* garbage -> resync */
        if (plen > cn->rxlen) { break; }         /* need more segments */
        el3_nvmet_one(c, cn, cn->rx, (int)plen);
        memmove(cn->rx, cn->rx + plen, cn->rxlen - plen);
        cn->rxlen -= plen;
    }
    /* A freshly-queued response segment carries the ACK (ack=rcv_nxt); otherwise the data needs one. */
    if (!(cn->resp_len != 0 && !had_resp)) {
        el3_nvmet_seg(c, cn->tmpl, cn->snd_nxt, cn->rcv_nxt, 0x10 /*ACK*/, false, NULL, 0);
    }
}

/* TCP-terminating entry: SYN-ACK the handshake, ACK + answer data segments, FIN-ACK the close, all in
 * virtual time. Returns true if the segment was for the NVMe target (port 4420) -> consumed. */
static bool el3_nvmet(EL3Core *c, const uint8_t *buf, size_t len)
{
    const uint8_t *ip, *tcp, *pl;
    int ihl, tcp_off, ip_total, payload;
    uint8_t flags;
    uint32_t their_seq, their_ack;
    uint16_t sport, dport;
    struct EL3NvmetConn *cn = NULL;
    int i, free_slot = -1;

    if (len < 54 || buf[12] != 0x08 || buf[13] != 0x00) {
        return false;                            /* not IPv4 (ARP etc.) -> let the backend/slirp handle */
    }
    ip = buf + 14;
    ihl = (ip[0] & 0x0F) * 4;
    if (ip[9] != 6 || (size_t)(14 + ihl + 20) > len) {
        return false;                            /* not TCP / truncated */
    }
    tcp = ip + ihl;
    tcp_off = (tcp[12] >> 4) * 4;
    ip_total = (ip[2] << 8) | ip[3];
    payload = ip_total - ihl - tcp_off;
    if (payload < 0) { payload = 0; }
    pl = tcp + tcp_off;
    flags = tcp[13];
    sport = (uint16_t)((tcp[0] << 8) | tcp[1]);
    dport = (uint16_t)((tcp[2] << 8) | tcp[3]);
    if (dport != 4420) {
        return false;                            /* not the NVMe target port -> passthrough */
    }
    their_seq = ((uint32_t)tcp[4] << 24) | (tcp[5] << 16) | (tcp[6] << 8) | tcp[7];
    their_ack = ((uint32_t)tcp[8] << 24) | (tcp[9] << 16) | (tcp[10] << 8) | tcp[11];

    for (i = 0; i < 2; i++) {
        if (c->nvmet_conn[i].gport == sport) { cn = &c->nvmet_conn[i]; break; }
        if (c->nvmet_conn[i].gport == 0 && free_slot < 0) { free_slot = i; }
    }

    if (flags & 0x04) {                          /* RST -> drop the conn */
        if (cn) { cn->gport = 0; }
        return true;
    }
    if ((flags & 0x02) && !(flags & 0x10)) {     /* SYN (no ACK) -> SYN-ACK with MSS */
        if (!cn) {
            if (free_slot < 0) { return true; }  /* only 2 conns modelled */
            cn = &c->nvmet_conn[free_slot];
        }
        cn->gport = sport;
        cn->snd_nxt = 0x40000000;
        cn->rcv_nxt = their_seq + 1;
        cn->ready = false;
        cn->w_remaining = 0;
        cn->resp_len = 0;
        cn->rxlen = 0;
        cn->snd_wnd = ((uint32_t)tcp[14] << 8) | tcp[15];   /* guest's initial advertised window */
        {   /* send MSS = min(the guest's SYN MSS, what our MAC carries) */
            uint16_t gm = el3_syn_mss(tcp, (tcp[12] >> 4) * 4);
            uint16_t pm = el3_peer_mss(c);
            cn->mss = gm < pm ? gm : pm;
        }
        if (len >= 54) { memcpy(cn->tmpl, buf, 54); }
        el3_nvmet_seg(c, buf, cn->snd_nxt, cn->rcv_nxt, 0x12 /*SYN|ACK*/, true, NULL, 0);
        cn->snd_nxt += 1;                        /* SYN consumes one sequence number */
        cn->snd_una = cn->snd_nxt;
        return true;
    }
    if (!cn) {
        return true;                             /* stray segment for an unknown conn -> consume */
    }
    if (len >= 54) { memcpy(cn->tmpl, buf, 54); } /* refresh the addressing template for paced replies */
    cn->snd_wnd = ((uint32_t)tcp[14] << 8) | tcp[15]; /* track the guest's advertised window (flow control) */

    if (flags & 0x01) {                          /* FIN -> FIN|ACK, then close */
        cn->rcv_nxt = their_seq + (uint32_t)payload + 1;
        el3_nvmet_seg(c, buf, cn->snd_nxt, cn->rcv_nxt, 0x11 /*FIN|ACK*/, false, NULL, 0);
        cn->gport = 0;
        return true;
    }

    /* Every segment carries an ACK: advance snd_una and drive the stop-and-wait sender (next segment,
     * or mark the response complete). A retransmit fires from the RTX timer if no ACK arrives. */
    if (cn->resp_len && (int32_t)(their_ack - cn->snd_una) > 0) {
        cn->snd_una = their_ack;
        el3_nvmet_emit(c, cn);
    }

    if (payload == 0) {
        return true;                             /* pure ACK -> handled above */
    }
    if (their_seq != cn->rcv_nxt) {              /* out-of-order / retransmit -> re-ACK current rcv_nxt, drop */
        el3_nvmet_seg(c, cn->tmpl, cn->snd_nxt, cn->rcv_nxt, 0x10 /*ACK*/, false, NULL, 0);
        return true;
    }
    cn->rcv_nxt = their_seq + (uint32_t)payload;
    el3_nvmet_rx(c, cn, pl, payload);            /* reassemble + dispatch complete PDUs; ACK as needed */
    return true;
}

/* Count on-wire TCP payload over a 2.966 s virtual window from the first data byte -> [L4TX].
 * Lives in the main TX path (not the reflector) so L4 TX goodput is measurable independent of the
 * reflector. Parses Eth/IPv4/TCP just enough to size the payload; ignores non-TCP / non-data frames. */
static void el3_l4tx_account(EL3Core *c, const uint8_t *buf, size_t len)
{
    const uint8_t *ip, *tcp;
    int ihl, tcp_off, ip_total, payload;
    int64_t now;

    if (len < 54 || buf[12] != 0x08 || buf[13] != 0x00) {
        return;                                  /* not IPv4 */
    }
    ip = buf + 14;
    ihl = (ip[0] & 0x0F) * 4;
    if (ip[9] != 6 || (size_t)(14 + ihl + 20) > len) {
        return;                                  /* not TCP / truncated */
    }
    tcp = ip + ihl;
    tcp_off = (tcp[12] >> 4) * 4;
    ip_total = (ip[2] << 8) | ip[3];
    payload = ip_total - ihl - tcp_off;
    if (payload <= 0) {
        return;
    }
    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (c->l4tx_start_ns == 0) {
        c->l4tx_start_ns = now;
        c->l4tx_bytes = 0;
    }
    c->l4tx_bytes += payload;
    /* 1.0 s virtual window: long enough for the rate to stabilise (~830 segs @10M, ~8300 @100M) but
     * short enough to bound wall time at low icount shifts, where virtual time only advances as fast
     * as the guest executes instructions and each PIO tx-free poll traps to the device model. */
    if (!c->l4tx_reported && (now - c->l4tx_start_ns) >= 1000000000LL) {
        double secs = (double)(now - c->l4tx_start_ns) / 1e9;
        double mbit = (double)c->l4tx_bytes * 8.0 / secs / 1e6;
        double wire = 8000.0 / (double)(c->tx_ns_per_byte ? c->tx_ns_per_byte : 800);
        fprintf(stderr, "[L4TX] payload=%llu over %.3fs -> %.2f Mbit/s (%.0f%% of %.0f Mbit wire)\n",
                (unsigned long long)c->l4tx_bytes, secs, mbit, mbit / wire * 100.0, wire);
        c->l4tx_reported = true;
    }
}

static bool el3_core_transmit_packet(EL3Core *c, const uint8_t *buf, size_t len)
{
    NetClientState *nc = qemu_get_queue(c->nic);
    ssize_t ret;

    if (c->tx_blocked) {
        return false;
    }

    el3_l4tx_account(c, buf, len);               /* L4 TX goodput tally (reflector-independent) */

    /* Test-only: in-emulator TCP-ACK reflector. Consumes the guest's TCP segments (so they don't
     * hit slirp and draw a RST flood) and synthesizes ONLY the SYN-ACK / FIN-ACK; data segments are
     * silently dropped (rawblast ignores flow control, so per-segment ACKs would just storm RX).
     * Non-TCP frames (ARP etc.) fall through to the backend so address resolution still works. */
    if (c->tcp_reflect && el3_tcp_reflect(c, buf, len)) {
        return true;
    }

    /* Test-only: in-emulator NVMe/TCP target. Terminates TCP to port 4420 and answers the initiator's
     * NVMe PDUs in virtual time (no slirp/host peer); non-4420 / non-IP frames fall through (ARP via
     * the backend still resolves the target's IP). */
    if (c->nvmet && el3_nvmet(c, buf, len)) {
        return true;
    }

    /* Use async send to handle backpressure */
    ret = qemu_send_packet_async(nc, buf, len, el3_tx_completion_cb);
    
    if (ret == 0) {
        /* Send failed - backpressure detected */
        bool was_blocked = c->tx_blocked;
        c->tx_blocked = true;
        if (!was_blocked) {
            trace_el3_tx_blocked_change(true);
        }
        return false;
    }
    
    if (ret < 0) {
        /* Error during transmission */
        qemu_log_mask(LOG_GUEST_ERROR, "el3: transmission error: %zd\n", ret);
        return false;
    }
    
    return true;
}

static void el3_cmd_busy(EL3Core *c, int64_t ns);  /* defined below */
static void el3_rx_update_complete(EL3Core *c);    /* defined below */
static void el3_rx_gen_timer_cb(void *opaque);     /* defined below */

/* Command processing */
static void el3_process_command(EL3Core *c, uint16_t cmd)
{
    uint16_t command = cmd >> 11;
    uint16_t param = cmd & 0x7FF;

    trace_el3_command(cmd, param);

    switch (command) {
    case 0x00: /* GlobalReset */
        el3_core_reset(c);
        break;
        
    case 0x01: /* SelectWindow */
        el3_select_window(c, param & 0x07);
        break;
        
    case 0x03: /* RxDisable */
        c->rx_enabled = false;
        trace_el3_cmd_ignored(cmd, "RX disable");
        break;
        
    case 0x04: /* RxEnable */
        c->rx_enabled = true;
        /* With can_receive() gating on rx_enabled, frames offered before RX was enabled left the
         * backend flow-controlled; resume it now that we can accept again. */
        qemu_flush_queued_packets(qemu_get_queue(c->nic));
        /* Note: the test-only RX generator is NOT auto-started here. Flooding from install would
         * swamp the (slow-CPU) guest while DOS is still loading rxcount.com, before it registers its
         * handle. Instead the guest arms it with a magic W1_TIMER write once it's ready to measure
         * (see the W1_TIMER write case + el3_rx_gen_timer_cb). */
        trace_el3_cmd_ignored(cmd, "RX enable");
        break;
        
    case 0x05: /* RxReset */
        /* Reset RX FIFO */
        c->rx_packet_count = 0;
        c->rx_packet_head = 0;
        c->rx_packet_tail = 0;
        c->rx_data_write_ptr = 0;
        c->rx_data_used = 0;
        memset(c->rx_packets, 0, sizeof(c->rx_packets));
        el3_cmd_busy(c, 5000);   /* RxReset is slow -> CmdInProgress (realtiming) */
        trace_el3_fifo_reset("RX");
        break;
        
    case 0x08: /* RxDiscard */
        /* Discard top packet from RX FIFO */
        if (c->rx_packet_count > 0) {
            RXPacketDesc *pkt = &c->rx_packets[c->rx_packet_head];

            /* Free the data buffer space */
            if (pkt->data_offset + pkt->length == c->rx_data_write_ptr) {
                /* This was the last packet, can reclaim space */
                c->rx_data_write_ptr = pkt->data_offset;
            }
            c->rx_data_used -= pkt->length;
            
            /* Remove from queue */
            c->rx_packet_head = (c->rx_packet_head + 1) % RX_MAX_PACKETS;
            c->rx_packet_count--;

            /* Emulator-side RX throughput: count frames the guest actually drained while the
             * generator measurement window is open (see el3_rx_gen_timer_cb). */
            if (c->rx_gen_size && !c->rx_gen_reported && c->rx_gen_start_ns) {
                c->rx_gen_consumed++;
            }

            /* Re-evaluate RxComplete: assert only if the new head has arrived on the wire. */
            el3_rx_update_complete(c);

            trace_el3_rx_discard(pkt->length);
        }
        break;
        
    case 0x09: /* TxEnable */
        c->tx_enabled = true;
        break;

    case 0x0A: /* TxDisable */
        c->tx_enabled = false;
        break;

    case 0x0B: /* TxReset */
        /* Reset TX FIFO */
        c->tx_fifo_write_ptr = 0;
        c->tx_fifo_read_ptr = 0;
        c->tx_fifo_used = 0;
        c->tx_in_progress = false;
        c->current_tx_len = 0;
        c->current_tx_written = 0;
        c->tx_preamble_pos = 0;
        c->tx_drain_deadline_ns = 0;
        c->tx_occupancy = 0;
        c->tx_wire_active = false;
        c->tx_avail_armed = false;   /* TxReset resets thresholds to disabled (tech ref) */
        el3_cmd_busy(c, 5000);   /* TxReset is slow -> CmdInProgress (realtiming) */
        c->tx_status = 0;
        /* Clear TX status bits */
        c->status &= ~(STAT_TX_COMPLETE | STAT_TX_AVAILABLE);
        c->int_status &= ~(STAT_TX_COMPLETE | STAT_TX_AVAILABLE);
        trace_el3_fifo_reset("TX");
        break;
        
    case 0x0C: /* RequestInterrupt (fake/test interrupt) */
        /* Force an interrupt for testing */
        c->int_status |= STAT_INT_LATCH;
        el3_update_irq(c);
        break;
        
    case 0x0D: /* AckIntr */
        /* Acknowledge and clear interrupt bits specified in param. The AckIntr param is the 11-bit
         * low field of the command word, so mask to 0x7FF -- not 0xFF. UpComplete is bit 10
         * (0x0400): masking to 0xFF made it un-ackable, so the bus-master RX IRQ would storm. The
         * driver acks with EL3_CMD_ACK_INTR | 0x07FF. */
        c->int_status &= ~(param & 0x7FF);
        /* AckIntr(InterruptLatch) explicitly clears the latch even while other indications remain
         * pending -- the ISR acks the latch, then reads status & 0x0F as the dispatch reason. */
        if (param & STAT_INT_LATCH) {
            c->status &= ~STAT_INT_LATCH;
        }
        /* TxComplete is an edge indication: real EL3 clears it from the readable status on
         * ack. Without this it stays sticky, so a driver polling status (the DMA TX-completion
         * path) sees a stale TxComplete on the NEXT unrelated IRQ and never waits for its own
         * transfer -- defeating realtiming DMA pacing. */
        c->status &= ~(param & STAT_TX_COMPLETE);
        /* RX_COMPLETE is a LEVEL indication tied to the PIO RX FIFO. AckIntr cleared it from
         * int_status above, but while frames remain (rx_packet_count > 0) it must RE-ASSERT so the
         * IRQ keeps firing until the ISR drains the FIFO (the driver yields per-IRQ relying on
         * this). Only an empty FIFO truly clears it -- and RX-DMA keeps rx_packet_count == 0, so it
         * clears there (no sticky re-fire). Without the int_status re-assert, the IRQ deasserts
         * with packets still queued and PIO RX stalls full. */
        if (param & STAT_RX_COMPLETE) {
            el3_rx_update_complete(c);   /* re-assert only if the (new) head has arrived */
        }
        /* UP_COMPLETE: the bus-master RX path has no PIO FIFO, so the readable status bit must be
         * cleared on ack -- otherwise it stays sticky and the next IRQ re-fires forever (mirror of
         * the RxComplete-DMA case above). */
        if (param & STAT_UP_COMPLETE) {
            c->status &= ~STAT_UP_COMPLETE;
        }
        if (c->int_status == 0) {
            c->status &= ~STAT_INT_LATCH;
        }
        el3_update_irq(c);
        break;
        
    case 0x0E: /* SetIntrEnb */
        /* Set interrupt enable mask */
        c->int_mask = param;
        el3_update_irq(c);
        break;
        
    case 0x0F: /* SetStatusEnb (SetIndicationEnable) */
        /* Set which status bits can generate interrupts */
        c->status_enb = param;
        break;
        
    case 0x10: /* SetRxFilter */
        el3_set_rx_filter(c, param);
        break;
        
    case 0x11: /* SetRxEarlyThresh */
        c->rx_early_thresh = param;
        break;
        
    case 0x12: /* Set TX Available Threshold -- arms the one-shot TxAvailable indication */
        c->tx_avail_thresh = param;
        c->tx_avail_armed = true;
        el3_update_tx_available(c);
        break;

    case 0x13: /* SetTxStartThresh */
        c->tx_start_thresh = param;
        break;
        
    case 0x14: /* StartDmaUp / StartDmaDown (3C515/59x) */
        if (param == 0) {
            /* Start Upload (RX) DMA: arm a single-transfer bus-master receive into the
             * up-descriptor at up_list_ptr (consumed by el3_core_dma_rx_single on the next frame). */
            c->up_stalled = false;
            c->rx_dma_armed = true;
        } else {
            /* Start Download (TX) DMA: single-transfer bus-master of the descriptor at
             * down_list_ptr (set via the base+0x404 register). First DMA milestone. */
            c->down_stalled = false;
            if (c->dma_as && c->down_list_ptr) {
                el3_core_dma_tx_single(c, c->dma_as, c->down_list_ptr);
            }
        }
        break;
        
    case 0x15: /* StatsEnable */
        c->stats_enabled = true;
        /* Freeze current stats for atomic reading */
        c->stats_frozen = true;
        memcpy(&c->stats_snapshot, &c->stats, sizeof(c->stats));
        break;
        
    case 0x16: /* StatsDisable */
        c->stats_enabled = false;
        c->stats_frozen = false;
        break;
        
    case 0x18: /* TxStart (deprecated PIO start) */
        /* Start transmission with specified length */
        c->current_tx_len = param;
        c->current_tx_written = 0;
        c->tx_in_progress = true;
        trace_el3_tx_start(param);
        break;
        
    case 0x02: /* StartCoax / EnableDcConverter (BNC transceiver power) */
    case 0x17: /* StopCoax / DisableDcConverter -- 3Com's 3C5X9PD issues this during 10BaseT media
                * setup. The emulated transceiver is always a ready 10BaseT link (no BNC/DC converter
                * modeled), so accept these as no-ops; without it a media-aware driver stalls. */
        break;

    case 0x06: /* DMA stall/unstall commands (3C515) */
        switch (param & 0x03) {
        case 0: /* UpStall */
            c->up_stalled = true;
            break;
        case 1: /* UpUnstall */
            c->up_stalled = false;
            break;
        case 2: /* DownStall */
            c->down_stalled = true;
            break;
        case 3: /* DownUnstall */
            c->down_stalled = false;
            break;
        }
        break;
        
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "el3: unknown command 0x%02x\n", command);
        break;
    }
}

/* Timer callback for EEPROM operations */
static void el3_eeprom_timer_cb(void *opaque)
{
    EL3Core *c = opaque;
    
    /* EEPROM operation complete */
    c->eeprom_state = EEPROM_STATE_IDLE;
    c->windows[0][W0_EEPROM_DATA >> 1] = c->eeprom_data;

    trace_el3_eeprom_done(c->eeprom_addr, c->eeprom_data);
}

static void el3_pt_dump(void);   /* periodic + atexit Parallel Tasking stats dump */

static void el3_tx_drain_advance(EL3Core *c);

/* TX drain timer (realtiming): the modeled wire transmit advances. For streaming PIO TX the FIFO
 * may still hold bytes (occupancy > 0) -> reschedule; only raise TxComplete once it empties. The
 * DMA path leaves occupancy at 0 and sets the timer to its own deadline, so it completes here too. */
static void el3_tx_drain_timer_cb(void *opaque)
{
    EL3Core *c = opaque;

    el3_tx_drain_advance(c);
    if (c->tx_occupancy > 0) {
        timer_mod_ns(c->tx_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                     (int64_t)c->tx_occupancy * c->tx_ns_per_byte);
        el3_update_tx_available(c);
        return;
    }
    c->tx_in_progress = false;
    c->tx_wire_active = false;
    c->tx_status = TX_STAT_COMPLETE;
    c->status |= STAT_TX_COMPLETE;
    c->int_status |= STAT_TX_COMPLETE;
    if (c->int_mask & STAT_TX_COMPLETE) {
        c->pt.tx_complete_irqs++;
    }
    el3_update_tx_available(c);
    el3_update_irq(c);
}

/* Slow command finished (realtiming): clear the CmdInProgress status bit. */
static void el3_cmd_timer_cb(void *opaque)
{
    EL3Core *c = opaque;

    c->status &= ~STAT_CMD_IN_PROG;
}

/* Mark a slow command busy for a modeled duration (realtiming only). The driver's bounded
 * CmdInProgress poll (e.g. the ISR's wait-on-RxReset) then actually waits. */
static void el3_cmd_busy(EL3Core *c, int64_t ns)
{
    if (!c->realtiming) {
        return;
    }
    c->status |= STAT_CMD_IN_PROG;
    timer_mod_ns(c->cmd_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ns);
}

/* Window selection */
void el3_select_window(EL3Core *c, uint8_t window)
{
    if (window >= c->caps.max_windows) {
        qemu_log_mask(LOG_GUEST_ERROR, "el3: invalid window %d\n", window);
        return;
    }
    
    c->current_window = window;
    trace_el3_window_select(window);
}

/* EEPROM operations */
void el3_eeprom_cmd(EL3Core *c, uint16_t cmd)
{
    uint8_t opcode = (cmd >> 6) & 0x03;
    uint8_t addr = cmd & 0x3F;
    
    trace_el3_eeprom_cmd(cmd, addr);
    
    if (opcode != 0x02) { /* Only READ supported */
        qemu_log_mask(LOG_GUEST_ERROR, "el3: unsupported EEPROM op %d\n", opcode);
        return;
    }
    
    if (addr >= 64) {
        qemu_log_mask(LOG_GUEST_ERROR, "el3: EEPROM addr %d out of range\n", addr);
        return;
    }
    
    /* Start EEPROM read. The data is made available immediately: a guest busy-wait (a fixed
     * io_delay loop) does not reliably advance QEMU's virtual clock under TCG, so a
     * timer-deferred result can be read before it lands -- which left the driver seeing the
     * 0x8000 busy placeholder as the "MAC". The 162us timer still runs to model the busy
     * window for drivers that poll it, but the value is ready now. */
    c->eeprom_state = EEPROM_STATE_BUSY;
    c->eeprom_addr = addr;
    c->eeprom_data = c->eeprom[addr];

    if (c->realtiming) {
        /* Realistic: hold the busy placeholder until the 162us timer lands the data, so the
         * driver's read wait is actually exercised (run the guest under -icount). */
        c->windows[0][W0_EEPROM_DATA >> 1] = 0x8000;
    } else {
        /* Instant: data valid immediately (guest busy-waits don't advance the TCG clock). */
        c->windows[0][W0_EEPROM_DATA >> 1] = c->eeprom_data;
    }

    /* Deadline for the clock-based read gate (below) and the migration/poll timer. */
    c->eeprom_due_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + c->eeprom_latency_ns;
    timer_mod_ns(c->eeprom_timer, c->eeprom_due_ns);

    trace_el3_eeprom_start(addr);
}

uint16_t el3_eeprom_read(EL3Core *c)
{
    uint16_t data;

    /* Realtiming: gate the result on elapsed VIRTUAL time, not a timer. The ISA I/O charges
     * the guest accrues during its read-wait advance that clock, so this reads valid data at
     * any icount shift -- whereas a timer fires on an instruction budget and, at a fast shift,
     * can land after the guest's wait (leaving it to read the 0x8000 placeholder as the MAC). */
    if (c->realtiming) {
        data = (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) < c->eeprom_due_ns)
               ? 0x8000 : c->eeprom_data;
    } else {
        data = c->windows[0][W0_EEPROM_DATA >> 1];
    }

    trace_el3_eeprom_read(data, (c->eeprom_state == EEPROM_STATE_BUSY));
    return data;
}

/* 3c509 ISA EEPROM checksum (word 0x0F).
 *
 * The genuine 3Com/Crynwr ISA detection reads all 16 EEPROM words bit-serially over the ID
 * port and folds them: each word's two bytes are XORed together, accumulated into the HIGH
 * byte of a running word -- except words 8, 9 and 0x0D, which accumulate into the LOW byte --
 * and word 0x0F (the stored checksum) is XORed in whole. A valid card folds to zero. So solve
 * for word 0x0F = fold(words 0..0x0E). */
static uint16_t el3_eeprom_checksum_3c509(const uint16_t *ee)
{
    uint16_t acc = 0;
    int i;
    for (i = 0; i < 0x0F; i++) {
        uint8_t x = (uint8_t)(ee[i] >> 8) ^ (uint8_t)ee[i];
        if (i == 0x08 || i == 0x09 || i == 0x0D) {
            acc ^= x;                 /* fold into low byte */
        } else {
            acc ^= (uint16_t)x << 8;  /* fold into high byte */
        }
    }
    return acc;
}

/* Initialize EEPROM for 3C509 */
void el3_eeprom_init_3c509(EL3Core *c)
{
    /* Clear EEPROM */
    memset(c->eeprom, 0, sizeof(c->eeprom));

    /* MAC address at words 0x00-0x02, stored big-endian (MSB = first MAC byte) as real 3Com
     * EEPROMs do -- the driver reads each word high-byte-first into its station address. */
    c->eeprom[0x00] = (c->conf.macaddr.a[0] << 8) | c->conf.macaddr.a[1];
    c->eeprom[0x01] = (c->conf.macaddr.a[2] << 8) | c->conf.macaddr.a[3];
    c->eeprom[0x02] = (c->conf.macaddr.a[4] << 8) | c->conf.macaddr.a[5];

    /* Product ID (driver checks word3 & 0xF0FF == 0x9050) */
    c->eeprom[0x03] = 0x9050;  /* 3C509B */

    /* Manufacturer ID (driver checks word7 == 0x6D50) */
    c->eeprom[0x07] = 0x6d50;  /* 3Com */

    /* Address configuration (word 8): transceiver bits | ((iobase - 0x200) >> 4) in low 5 bits.
     * iobase 0x300 -> 0x10; transceiver bits 0 = 10BaseT (TP). Matches the device default base. */
    c->eeprom[0x08] = ((0x300 - 0x200) >> 4) & 0x1F;  /* 0x0010 -> I/O 0x300 */

    /* Resource configuration (word 9): IRQ in bits 15-12. */
    c->eeprom[0x09] = (uint16_t)10 << 12;  /* 0xA000 -> IRQ 10 */

    /* OEM node address (words 0x0A-0x0C), a second copy of the MAC. 3Com's own driver
     * (3C5X9PD) reads the station address from here, not from words 0-2 (which Crynwr uses). */
    c->eeprom[0x0A] = c->eeprom[0x00];
    c->eeprom[0x0B] = c->eeprom[0x01];
    c->eeprom[0x0C] = c->eeprom[0x02];

    /* Checksum word so the genuine ISA detection (Crynwr/3Com) validates the card. */
    c->eeprom[0x0F] = el3_eeprom_checksum_3c509(c->eeprom);
}

/* Initialize EEPROM for 3C59x (including 3C515) */
void el3_eeprom_init_3c59x(EL3Core *c)
{
    /* Clear EEPROM */
    memset(c->eeprom, 0, sizeof(c->eeprom));
    
    /* 3C59x uses different EEPROM layout */
    /* Node address at 0x10-0x12 instead of 0x00-0x02 */
    c->eeprom[0x10] = (c->conf.macaddr.a[1] << 8) | c->conf.macaddr.a[0];
    c->eeprom[0x11] = (c->conf.macaddr.a[3] << 8) | c->conf.macaddr.a[2];
    c->eeprom[0x12] = (c->conf.macaddr.a[5] << 8) | c->conf.macaddr.a[4];
    
    /* OEM node address copy at 0x0A-0x0C */
    c->eeprom[0x0A] = c->eeprom[0x10];
    c->eeprom[0x0B] = c->eeprom[0x11];
    c->eeprom[0x0C] = c->eeprom[0x12];
    
    /* Device ID and subsystem info */
    if (c->model == MODEL_3C515) {
        c->eeprom[0x00] = 0x5157;  /* Device ID for 3C515 */
    } else if (c->model == MODEL_3C905) {
        c->eeprom[0x00] = 0x9050;  /* Device ID for 3C905 */
    } else if (c->model == MODEL_3C905B) {
        c->eeprom[0x00] = 0x9055;  /* Device ID for 3C905B */
    }
    
    c->eeprom[0x01] = 0x10B7;  /* Vendor ID (3Com) */
    
    /* Capabilities and configuration */
    c->eeprom[0x08] = 0x0040;  /* 100Mbps capable */
    c->eeprom[0x09] = 0x0048;  /* Full-duplex, MII */
    
    /* Compatibility word */
    c->eeprom[0x0F] = 0x0100;  /* Default media type */
    
    /* Software information region at 0x14-0x16 */
    c->eeprom[0x14] = 0x0101;  /* Software info 1 */
    c->eeprom[0x15] = 0x5943;  /* "YC" */
    c->eeprom[0x16] = 0x4C4F;  /* "LO" */
    
    /* Checksum placeholder */
    c->eeprom[0x1F] = 0x0000;  /* Would be calculated in real hardware */
}

/* RX filter management */
void el3_set_rx_filter(EL3Core *c, uint16_t filter)
{
    c->rx_filter = filter;
    trace_el3_filter_set(filter);
}

/* Frame acceptance logic */
bool el3_accept_frame(EL3Core *c, const uint8_t *buf, size_t len)
{
    /* Promiscuous mode accepts everything */
    if (c->rx_filter & RX_FILTER_PROMISCUOUS) {
        return true;
    }
    
    /* Check for broadcast */
    if (eth_addr_is_broadcast(buf)) {
        return (c->rx_filter & RX_FILTER_BROADCAST) != 0;
    }
    
    /* Check for multicast */
    if (eth_addr_is_multicast(buf)) {
        /* All multicast mode */
        if (c->rx_filter & RX_FILTER_ALLMULTI) {
            return true;
        }
        
        /* Check multicast hash filter */
        if (c->rx_filter & RX_FILTER_MULTICAST) {
            uint8_t hash_idx = el3_mcast_hash_index(buf);
            uint8_t byte_idx = hash_idx / 8;
            uint8_t bit_mask = 1 << (hash_idx % 8);
            return (c->mcast_hash[byte_idx] & bit_mask) != 0;
        }
        return false;
    }
    
    /* Check for unicast to our MAC */
    if (c->rx_filter & RX_FILTER_INDIVIDUAL) {
        return eth_addr_equals(buf, c->conf.macaddr.a);
    }
    
    return false;
}

/* Centralized IRQ handling */
void el3_update_irq(EL3Core *c)
{
    bool level = (c->int_status & c->int_mask) != 0;
    /* Update IRQ line if state changed */
    if (c->irq_level != level) {
        c->irq_level = level;
        if (level) {
            /* InterruptLatch latches on the rising edge of the (masked) IRQ. It is cleared by
             * AckIntr(InterruptLatch), NOT by the indication bits merely staying set -- so that a
             * driver's ISR, after acking the latch, reads a clean reason nibble (e.g. TxAvailable
             * alone, 0x08) and dispatches correctly. Recomputing it from int_status every call
             * would keep bit 0 set and corrupt the ISR's status&0x0F reason decode. */
            c->status |= STAT_INT_LATCH;
        }
        if (c->ops && c->ops->irq_set) {
            c->ops->irq_set(c, level);
        }
    }

    /* With nothing pending the latch is naturally clear. */
    if (c->int_status == 0) {
        c->status &= ~STAT_INT_LATCH;
    }

    /* STAT_INT_REQ indicates IRQ line is asserted (masked) */
    if (level) {
        c->status |= STAT_INT_REQ;
    } else {
        c->status &= ~STAT_INT_REQ;
    }
}

/* el3_core_set_status -- OR indication bit(s) into both the readable status register and the
 * pending int_status. Declared in el3_core.h; its intended home (el3_core_full.c) isn't compiled
 * in this overlay, so the ring-DMA scaffolding that references it needs the definition here to
 * link under -O0 (where the unreachable scaffolding isn't dead-code-eliminated). Callers invoke
 * el3_update_irq() separately. */
void el3_core_set_status(EL3Core *c, uint16_t bits)
{
    c->status |= bits;
    c->int_status |= bits;
}

/* Update TX available interrupt.
 *
 * TX Available is ONE-SHOT (3c5x9b tech ref, "Set TX Available Threshold" / "TX Available"
 * commands): the driver ARMS it with SetTxAvailableThreshold; it fires exactly once when the free
 * TX FIFO space first reaches the threshold, then **disarms itself** ("resets the threshold to its
 * disabled value -- the command must be reissued each time"). It is NOT a free-running level or
 * edge. Modelling it as a level made it storm (free is usually >= a small threshold); modelling it
 * as a self-re-arming edge stalled the driver after one frame. One-shot + driver-re-arm is correct:
 * no storm (bounded by re-arms) and the async chunk loop gets exactly the cadence it expects. */
void el3_update_tx_available(EL3Core *c)
{
    if (c->tx_avail_armed && el3_get_tx_free(c) >= c->tx_avail_thresh) {
        c->status |= STAT_TX_AVAILABLE;
        c->int_status |= STAT_TX_AVAILABLE;       /* fire */
        if (c->int_mask & STAT_TX_AVAILABLE) {
            c->pt.tx_avail_irqs++;
        }
        c->tx_avail_armed = false;                /* one-shot: disarm until reissued */
    } else {
        c->status &= ~STAT_TX_AVAILABLE;
    }
    el3_update_irq(c);
}

/* Streaming TX: advance the FIFO drain to the current virtual time. The card transmits bytes
 * out of the FIFO at the wire rate, so tx_occupancy (bytes written but not yet on the wire)
 * decreases over time. Called before any read of TxFree and before adding new bytes. */
static void el3_tx_drain_advance(EL3Core *c)
{
    int64_t now, elapsed, per_byte;
    uint32_t drained;

    if (!c->realtiming || !c->tx_ns_per_byte) {
        return;
    }
    /* PIO ISA-I/O cap: a CPU cannot PIO bytes to a 16-bit ISA card faster than the ISA I/O bandwidth
     * (~the bus-master rate). icount counts each `out` as one instruction (2^shift ns), so without
     * this a fast CPU's PIO would unrealistically reach the 100 Mbit wire instead of the ~48 Mbit ISA
     * ceiling. Pace the effective FIFO progress at the SLOWER of the wire and the ISA I/O rate. At
     * 10 Mbit the wire (800 ns/B) dominates -> unchanged; at 100 Mbit the ISA rate (~167 ns/B) caps it.
     * (DMA TX uses its own dma_rate pacing via tx_drain_deadline, not this FIFO path, so it's unaffected.) */
    per_byte = c->tx_ns_per_byte;
    if (c->dma_rate_bps) {
        int64_t isa_pio_ns = 1000000000LL / (int64_t)c->dma_rate_bps;
        if (isa_pio_ns > per_byte) {
            per_byte = isa_pio_ns;
        }
    }
    if (c->tx_occupancy == 0) {
        c->tx_drain_last_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        c->tx_wire_active = false;
        return;
    }
    /* Parallel Tasking early-transmit: the card does not begin clocking the FIFO onto the wire
     * until tx_start_thresh bytes are present -- OR the whole packet is in the FIFO (3C509B TRM:
     * a threshold above the packet length means store-and-forward, not "never"). Until then
     * occupancy can rise (host fill) without draining; once it crosses the threshold the wire runs
     * and overlaps the remaining fill. tx_preamble_pos == 0 means no partial frame is being
     * assembled, so everything in the FIFO is complete frames. (Without that clause a threshold
     * >= the frame length stalled each frame until the NEXT frame's fill pushed occupancy over it
     * -- the "store-and-forward is 16x slower" artifact.) */
    if (!c->tx_wire_active) {
        if (c->tx_occupancy < c->tx_start_thresh && c->tx_preamble_pos != 0) {
            c->tx_drain_last_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            return;
        }
        c->tx_wire_active = true;
        c->tx_drain_last_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        if (c->pt.frame_wire_start_ns == 0) {
            c->pt.frame_wire_start_ns = c->tx_drain_last_ns;   /* early-start instant */
        }
    }
    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    elapsed = now - c->tx_drain_last_ns;
    if (elapsed <= 0) {
        return;
    }
    drained = elapsed / per_byte;
    if (drained == 0) {
        return;          /* keep the fractional remainder in tx_drain_last_ns */
    }
    if (drained >= c->tx_occupancy) {
        c->tx_occupancy = 0;
        c->tx_drain_last_ns = now;
    } else {
        c->tx_occupancy -= drained;
        c->tx_drain_last_ns += (int64_t)drained * per_byte;
    }
}

/* Get TX FIFO free space. In realtiming mode this reflects the live FIFO occupancy (bytes
 * written minus bytes drained at wire rate) -- so the driver's TxFree guard waits when it
 * fills faster than the link drains, and TxAvailable does not fire spuriously mid-frame. */
uint16_t el3_get_tx_free(EL3Core *c)
{
    if (c->realtiming && c->tx_ns_per_byte) {
        el3_tx_drain_advance(c);
        return (c->tx_occupancy >= TX_FIFO_SIZE) ? 0 : (TX_FIFO_SIZE - c->tx_occupancy);
    }
    return TX_FIFO_SIZE - c->tx_fifo_used;
}

/* Get RX FIFO free space */
uint16_t el3_get_rx_free(EL3Core *c)
{
    return RX_FIFO_SIZE - c->rx_data_used;
}

/* Largest RX frame (excl FCS) accepted before oversizedFrame is raised. Gated by allowLargePackets
 * (MacControl[6], Window 3 offset 6): set -> FDDI-sized (<=4490 B); clear -> standard Ethernet. */
static inline uint32_t el3_rx_oversize_max(const EL3Core *c)
{
    return (c->windows[3][W3_MAC_CONTROL >> 1] & MAC_CTRL_ALLOW_LARGE)
           ? EL3_LARGE_RX_MAX : ETH_MAX_FRAME_LEN;
}

/* Bus-master RX DMA (3C515): when StartDmaUp has armed an up-descriptor at up_list_ptr, DMA a
 * received frame straight into its buffer, write UP_COMPLETE + length back, and raise UpComplete.
 *
 * RING vs single-transfer: if the consumed descriptor's `next` is non-zero the driver posted a ring
 * -- advance up_list_ptr to next (stay armed) for sustained zero-copy RX. If next==0 it's the legacy
 * single-transfer descriptor -- disarm (the driver re-posts + re-arms). A descriptor whose status
 * already has UP_COMPLETE set hasn't been recycled by the driver yet (ring full) -> overrun-drop.
 *
 * dma_rate pacing (realtiming): the ISA bus-master transfers one frame in len*bus_ns_per_byte. The
 * single DMA engine is serial, so frames arriving while it's still transferring the previous one
 * (now < rx_dma_ready_ns) are dropped -- this caps the delivered RX-DMA rate at min(wire, dma_rate)
 * = the practical ISA bus ceiling (~48 Mbit), independent of CPU. Mirror of el3_core_dma_tx_single.
 *
 * Returns true if the frame was HANDLED by the DMA engine (consumed OR overrun-dropped) so the
 * caller must NOT also place it in the PIO RX FIFO; false means RX-DMA isn't armed -> use PIO. */
/* el3_core_dma_rx_single result: PASS = not an RX-DMA frame (caller falls through to PIO); DONE =
 * bus-mastered into the ring; RETRY = the bus is busy or the ring is full -> BACKPRESSURE, not a drop.
 * RETRY makes el3_core_receive return -1 so the source (the rxgen generator, or a real backend via
 * can_receive) holds the frame and re-offers it when the bus/ring frees -- instead of the old silent
 * drop that returned "delivered" and made the closed-loop generator advance its seq over a frame the
 * guest never got (a single such phantom-advance stalled in-order RX forever -> go-back-N collapse). */
#define EL3_RXDMA_PASS   0
#define EL3_RXDMA_DONE   1
#define EL3_RXDMA_RETRY (-1)

static int el3_core_dma_rx_single(EL3Core *c, const uint8_t *buf, size_t len)
{
    EL3UpDesc d;
    uint32_t cap;
    size_t n;
    int64_t now;

    if (!c->rx_dma_armed || !c->dma_as || !c->up_list_ptr) {
        return EL3_RXDMA_PASS;                       /* RX-DMA not armed -> PIO path */
    }

    /* Copybreak (docs/10): below the driver-set threshold, leave the frame for the PIO FIFO drain so it
     * never lands on the (shallow) DMA ring -- only large frames are worth the DMA setup + cache flush,
     * and keeping small frames off the ring is what relieves its depth pressure. T==0 disables it (the
     * model-A "DMA everything" path). T is the driver's autotuned g_copybreak_t (copybreak_autotune). */
    if (c->rx_copybreak && len <= c->rx_copybreak) {
        return EL3_RXDMA_PASS;                       /* small -> PIO RxComplete path */
    }

    /* ISA bus-master engine still transferring the previous frame -> BACKPRESSURE, don't drop. RETRY
     * makes the source hold this frame and re-offer it once the bus frees (rx_dma_ready_ns), so a
     * windowed RX flood is paced to min(wire, bus) instead of losing the concurrent frames (and, with
     * the closed-loop generator, advancing its seq over them -> permanent in-order stall). */
    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (c->realtiming && c->dma_rate_bps && now < c->rx_dma_ready_ns) {
        return EL3_RXDMA_RETRY;
    }

    if (address_space_read(c->dma_as, c->up_list_ptr, MEMTXATTRS_UNSPECIFIED,
                           &d, sizeof(d)) != MEMTX_OK) {
        return EL3_RXDMA_PASS;
    }
    /* Descriptor not yet recycled by the driver (still complete) -> ring full -> BACKPRESSURE (RETRY),
     * not a drop: the source re-offers when the driver clears a slot. This is what makes a DEEPER ring
     * pay off -- N slots absorb N in-flight frames before the source has to wait on the drain. */
    if (d.status & EL3_DESC_UP_COMPLETE) {
        return EL3_RXDMA_RETRY;
    }
    cap = d.length & EL3_DESC_LENGTH_MASK;          /* buffer capacity the driver posted */
    n = len;
    if (cap && n > cap) {
        n = cap;                                    /* truncate to the posted buffer */
    }
    if (address_space_write(c->dma_as, d.addr, MEMTXATTRS_UNSPECIFIED, buf, n) != MEMTX_OK) {
        return EL3_RXDMA_PASS;
    }
    /* status: UpComplete + the received length (13-bit), like the 3c59x UpPktStatus. */
    d.status = EL3_DESC_UP_COMPLETE | ((uint32_t)len & EL3_DESC_LENGTH_MASK);
    address_space_write(c->dma_as, c->up_list_ptr, MEMTXATTRS_UNSPECIFIED, &d, sizeof(d));

    /* Advance the ring (next_desc != 0) or disarm (single-transfer, next_desc == 0). */
    if (d.next_desc) {
        c->up_list_ptr = d.next_desc;
    } else {
        c->rx_dma_armed = false;
    }

    /* Pace the serial DMA engine: busy for this frame's bus transfer time. */
    if (c->realtiming && c->dma_rate_bps) {
        int64_t per_byte = 1000000000LL / c->dma_rate_bps;
        int64_t base = (c->rx_dma_ready_ns > now) ? c->rx_dma_ready_ns : now;
        c->rx_dma_ready_ns = base + (int64_t)len * per_byte;
    }

    c->stats.rx_frames_ok++;
    c->stats.rx_bytes_ok += len;
    /* Emulator-side throughput measurement (DMA RX): count delivered frames during the rxgen window
     * the same way the PIO path counts RxDiscards, so [RXGEN] reports the DMA-bound rate. */
    if (c->rx_gen_size && !c->rx_gen_reported && c->rx_gen_start_ns) {
        c->rx_gen_consumed++;
    }
    /* Bus-master RX completion is signalled by UpComplete (IntStatus bit 10, 0x0400) on real
     * 3c515/3c59x silicon -- NOT RxComplete (bit 4), which is the PIO RxStatus-FIFO indication.
     * The driver's xms_rx_deliver path dispatches on UpComplete; raising RxComplete here would
     * route the frame into the (empty) PIO FIFO path and silently drop it. */
    c->status |= STAT_UP_COMPLETE;
    c->int_status |= STAT_UP_COMPLETE;
    if (c->int_mask & STAT_UP_COMPLETE) {
        c->pt.rx_complete_irqs++;
    }
    el3_update_irq(c);
    trace_el3_rx_packet(len, d.status);
    return EL3_RXDMA_DONE;
}

/* Network receive handler */
/* RX flow control. Without this QEMU delivers frames as fast as the source provides them; a fast
 * sender (or a test flood) then overruns the PIO RX FIFO faster than the guest ISR can drain it,
 * hammering the driver far beyond any real wire rate. Returning false here makes QEMU queue the
 * frame and stop reading the backend until the FIFO drains (RxDiscard calls
 * qemu_flush_queued_packets to resume) -- the standard NIC backpressure model, and what bounds the
 * guest's RX load to something realistic. */
/* Make the PIO RX FIFO head visible to the guest: RxComplete is a level indication of "a frame is
 * waiting in the FIFO". RX wire-rate pacing is NOT done here by hiding the head -- an earlier
 * arrival_ns visibility gate chained rx_ready_ns forward on *every* receive (incl. FIFO-full drops),
 * so under a flood arrival_ns raced far ahead of the virtual clock and starved delivery (it zeroed
 * short L4 transfers and livelocked the slow-CPU RX cells). Pacing is instead provided by the
 * source: the el3_rx_gen_timer_cb generator for L2-RX measurement (one frame per wire-time in
 * virtual time) and slirp's own self-clocking for L4. can_receive() FIFO-space backpressure bounds
 * the load. So the head is simply visible whenever the FIFO is non-empty. */
static void el3_rx_update_complete(EL3Core *c)
{
    if (c->rx_packet_count == 0) {
        c->status &= ~STAT_RX_COMPLETE;
        c->int_status &= ~STAT_RX_COMPLETE;
    } else {
        c->status |= STAT_RX_COMPLETE;
        c->int_status |= STAT_RX_COMPLETE;
    }
    el3_update_irq(c);
}

/* The wire finished clocking in the head frame -> re-evaluate RxComplete visibility. */
static void el3_rx_pace_timer_cb(void *opaque)
{
    el3_rx_update_complete((EL3Core *)opaque);
}

/* Test-only RX wire-rate frame generator. Paced by the virtual clock at exactly the link rate,
 * it is the RX counterpart of the TX drain model: it avoids the wall-vs-virtual-time mismatch (an
 * external wall-paced injector measured against a virtual-time window) and the socket-firehose
 * vCPU starvation that both make external RX flooding unmeasurable under -icount. Each tick injects
 * one broadcast frame through the normal receive path; if the FIFO is full the guest hasn't kept up
 * (counted as dropped), so the delivered rate = min(wire rate, guest PIO/DMA drain rate). */
static void el3_rx_gen_timer_cb(void *opaque)
{
    EL3Core *c = (EL3Core *)opaque;
    uint32_t sz = c->rx_gen_size;

    if (sz == 0) {
        return;  /* disabled */
    }
    if (sz > el3_rx_oversize_max(c)) {
        sz = el3_rx_oversize_max(c);             /* FDDI-sized only once the driver set allowLargePackets */
    }
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    /* End of the measurement window: report consumed-frame throughput (virtual time) and stop.
     * This is the authoritative RX rate -- it doesn't depend on the guest's PIT-tick window, which a
     * slow CPU under a 100 Mbit flood starves. RXGEN_WINDOW_NS matches rxcount's ~2.966 s for
     * comparability. */
    if (!c->rx_gen_reported && c->rx_gen_start_ns &&
        (now - c->rx_gen_start_ns) >= (c->rx_gen_l4 ? 1000000000LL : 2966000000LL)) {
        double secs = (double)(now - c->rx_gen_start_ns) / 1e9;
        /* L4: delivered = how far the guest's in-order rcv_nxt advanced (immune to drops/retransmits,
         * unlike rx_gen_consumed = FIFO-drains, which under a window also tallies dropped+resent frames).
         * L2: the broadcast flood has no sequencing, so fall back to the consumed-frame count. */
        uint32_t delivered = c->rx_gen_l4 ? (c->rx_gen_seq - 0x10000)
                                          : (uint32_t)(c->rx_gen_consumed * sz);
        double mbit = (double)delivered * 8.0 / secs / 1e6;
        double wire = 8000.0 / (double)(c->tx_ns_per_byte ? c->tx_ns_per_byte : 800);
        fprintf(stderr,
                "[%s] win=%u delivered=%u offered=%llu dropped=%llu over %.3fs -> %.2f Mbit/s "
                "(%.0f%% of %.0f Mbit wire)\n", c->rx_gen_l4 ? "L4RX" : "RXGEN",
                c->rx_gen_l4 ? (c->rx_gen_window ? c->rx_gen_window : 1) : 0u,
                delivered,
                (unsigned long long)(c->rx_gen_count + c->rx_gen_dropped),
                (unsigned long long)c->rx_gen_dropped, secs, mbit, mbit / wire * 100.0, wire);
        c->rx_gen_reported = true;
        timer_del(c->rx_gen_timer);   /* stop the flood; guest can finish/poweroff */
        return;
    }

    /* L4 sliding window: keep up to rx_gen_window frames in flight (sent but unACKed) so the guest's
     * ISR can coalesce a batch and the per-frame ACK round-trip is hidden behind the window instead of
     * serializing every frame. window=1 reduces exactly to the original stop-and-wait closed loop.
     *   - A FIFO-full el3_core_receive just doesn't advance rx_gen_next -> the same seq is rebuilt next
     *     tick (natural pacing; no data lost).
     *   - A stack-ring drop (frame accepted into the FIFO but the guest's receiver ring was full ->
     *     RxDiscarded, never ACKed) stalls una with a full window; after a timeout we go-back-N: rewind
     *     rx_gen_next to una and resend. L2 mode stays an open broadcast flood (no sequencing). */
    uint32_t payload = (sz >= 54) ? (sz - 54) : 0;
    uint32_t win = c->rx_gen_window ? c->rx_gen_window : 1;
    bool emit;
    if (c->rx_gen_l4) {
        if (payload && (uint32_t)(c->rx_gen_next - c->rx_gen_seq) >= win * payload) {
            if (++c->rx_gen_stall > 64) {       /* full window, no ACK progress -> retransmit the window */
                c->rx_gen_next = c->rx_gen_seq;
                c->rx_gen_stall = 0;
            }
        }
        emit = c->rx_enabled && payload &&
               (uint32_t)(c->rx_gen_next - c->rx_gen_seq) < win * payload;
    } else {
        emit = c->rx_enabled;                   /* L2: open broadcast flood */
    }
    if (emit) {
        uint8_t frame[EL3_LARGE_FRAME_MAX];
        memset(frame, 0, sz);
        memset(frame, 0xff, 6);                 /* dst: broadcast (passes the RX filter) */
        frame[6] = 0x52; frame[7] = 0x54; frame[11] = 0x01;  /* src 52:54:00:00:00:01 */
        frame[12] = 0x08; frame[13] = 0x00;     /* ethertype 0x0800 (rxcount matches any type) */
        if (c->rx_gen_l4 && sz >= 54) {
            /* L4-RX: a valid TCP data segment (10.0.2.2:7777 -> 10.0.2.15:0xC000), seq = rx_gen_next, so
             * the guest's tcp_input accepts + processes it (checksum-bound when the stack verifies). The
             * guest's tcp_rxbench forces a matching synthetic RX connection (rcv_nxt = 0x10000). */
            uint8_t *ip = frame + 14, *tcp = frame + 34;
            int pl = (int)payload;
            uint32_t seq = c->rx_gen_next;
            ip[0] = 0x45;
            ip[2] = (uint8_t)((40 + pl) >> 8); ip[3] = (uint8_t)(40 + pl);
            ip[8] = 64; ip[9] = 6;                              /* ttl, proto TCP */
            ip[12] = 10; ip[13] = 0; ip[14] = 2; ip[15] = 2;    /* src 10.0.2.2 */
            ip[16] = 10; ip[17] = 0; ip[18] = 2; ip[19] = 15;   /* dst 10.0.2.15 */
            { uint16_t ck = el3_csum16(0, ip, 20); ip[10] = ck >> 8; ip[11] = ck & 0xFF; }
            tcp[0] = 7777 >> 8; tcp[1] = 7777 & 0xFF;           /* src port 7777 */
            tcp[2] = 0xC0; tcp[3] = 0x00;                       /* dst port 0xC000 (LOCAL_PORT) */
            tcp[4] = seq >> 24; tcp[5] = seq >> 16; tcp[6] = seq >> 8; tcp[7] = seq;
            tcp[9] = 0x02;                                      /* ack = 0x00020000 (guest snd_nxt) */
            tcp[12] = (20 / 4) << 4; tcp[13] = 0x18;            /* PSH|ACK */
            tcp[14] = 0xFF; tcp[15] = 0xFF;                     /* window */
            {
                uint32_t ps = 0;
                ps += (ip[12] << 8) | ip[13]; ps += (ip[14] << 8) | ip[15];
                ps += (ip[16] << 8) | ip[17]; ps += (ip[18] << 8) | ip[19];
                ps += 6; ps += 20 + pl;
                uint16_t ck = el3_csum16(ps, tcp, 20 + pl);
                tcp[16] = ck >> 8; tcp[17] = ck & 0xFF;
                if (c->rx_gen_bad) {
                    tcp[16] ^= 0xFF;   /* test: corrupt the TCP checksum -> a verifying stack must drop it */
                }
            }
        }
        if (el3_core_receive(qemu_get_queue(c->nic), frame, sz) < 0) {
            c->rx_gen_dropped++;                /* FIFO full -> retransmit same seq next tick (no advance) */
        } else {
            c->rx_gen_count++;
            if (c->rx_gen_l4) {
                c->rx_gen_next += payload;      /* committed to the FIFO -> slide the send pointer */
            }
        }
    }
    /* Re-arm one frame-time later in virtual time. Pace at the SLOWER of the wire and the ISA bus:
     * a frame must clock in off the wire AND transfer into the guest over the 16-bit ISA bus, so RX
     * delivery is capped at min(wire, ISA) -- the symmetric counterpart of the PIO/DMA TX cap (a fast
     * CPU's RX can't exceed the ISA bus any more than its TX can). At 10 Mbit the wire dominates. */
    int64_t per_byte = c->tx_ns_per_byte ? c->tx_ns_per_byte : 800;
    if (c->dma_rate_bps) {
        int64_t isa_ns = 1000000000LL / c->dma_rate_bps;
        if (isa_ns > per_byte) {
            per_byte = isa_ns;
        }
    }
    timer_mod_ns(c->rx_gen_timer, now + (int64_t)sz * per_byte);
}

bool el3_core_can_receive(NetClientState *nc)
{
    EL3Core *c = qemu_get_nic_opaque(nc);

    /* Backpressure on FIFO space only. The source provides wire-rate pacing (the generator / slirp),
     * so here we just refuse the backend when the FIFO is full and let RxDiscard's
     * qemu_flush_queued_packets resume it. el3_core_receive() drops when RX is disabled; gating
     * rx_enabled here would flow-control before install with no clean resume. */
    if (c->rx_packet_count >= RX_MAX_PACKETS) {
        return false;
    }
    /* keep headroom for one more (standard) frame; oversize frames are caught in receive() */
    if (c->rx_data_used + 1600 > RX_FIFO_SIZE) {
        return false;
    }
    return true;
}

ssize_t el3_core_receive(NetClientState *nc, const uint8_t *buf, size_t size)
{
    EL3Core *c = qemu_get_nic_opaque(nc);

    /* Check if RX is enabled */
    if (!c->rx_enabled) {
        trace_el3_rx_drop(size, "RX disabled");
        return -1;
    }

    /* RX wire-rate pacing is provided by the *source* (the el3_rx_gen_timer_cb generator for L2-RX,
     * slirp self-clocking for L4), not by delaying head visibility here -- the old arrival_ns gate
     * chained rx_ready_ns forward on every receive (incl. drops) and raced ahead of the virtual
     * clock under load. Frames are simply made visible as soon as they land in the FIFO. */

    /* Apply RX filter */
    if (!el3_accept_frame(c, buf, size)) {
        trace_el3_rx_filter_drop(c->rx_filter, "filtered");
        return size;  /* Silently drop */
    }
    
    /* Validate frame size (without FCS) */
    if (size < ETH_MIN_DATA_NOFCS) {
        /* Runt frame */
        c->stats.rx_overruns++;  /* Count as error */
        trace_el3_rx_error("runt");
        if (!(c->rx_filter & RX_FILTER_ACCEPT_ERROR)) {
            return size;
        }
    }
    
    if (size > el3_rx_oversize_max(c)) {
        /* Oversize frame */
        c->stats.rx_overruns++;
        trace_el3_rx_error("oversize");
        if (!(c->rx_filter & RX_FILTER_ACCEPT_ERROR)) {
            return size;
        }
    }
    
    /* Check for loopback mode */
    if (c->internal_loopback) {
        trace_el3_rx_loopback(size);
        /* In loopback, packet should come from our own TX */
        /* For now, just accept it */
    }

    /* Stage 1b: single-transfer RX-DMA. If StartDmaUp armed an up-descriptor, DMA the frame
     * straight into the driver's (XMS or conventional) buffer instead of the PIO RX FIFO. */
    {
        int rxdma = el3_core_dma_rx_single(c, buf, size);
        if (rxdma == EL3_RXDMA_DONE) {
            return size;
        }
        if (rxdma == EL3_RXDMA_RETRY) {
            return -1;   /* bus busy / ring full -> backpressure: not delivered, source must re-offer */
        }
        /* EL3_RXDMA_PASS: not an RX-DMA frame (unarmed / copybreak) -> fall through to PIO below */
    }

    /* Handle bus master DMA mode (3C515/59x) */
    if (c->bus_master_enabled && c->ops && c->ops->rx_place_frame) {
        uint32_t rx_status = 0;
        
        /* Build RX status - no errors for good frames */
        if (size < ETH_MIN_DATA_NOFCS) {
            rx_status |= RX_STATUS_ERROR | RX_ERR_RUNT;
        } else if (size > el3_rx_oversize_max(c)) {
            rx_status |= RX_STATUS_ERROR | RX_ERR_OVERSIZE;
        }
        
        ssize_t ret = c->ops->rx_place_frame(c, buf, size, rx_status, size);
        if (ret >= 0) {
            c->stats.rx_frames_ok++;
            c->stats.rx_bytes_ok += size;
            trace_el3_rx_packet(size, rx_status);
        }
        return ret;
    }
    
    /* PIO mode - store in RX FIFO */
    
    /* Check if packet queue is full */
    if (c->rx_packet_count >= RX_MAX_PACKETS) {
        c->stats.rx_overruns++;
        trace_el3_rx_overflow(size, RX_FIFO_SIZE - c->rx_data_used);
        return -1;  /* No room */
    }
    
    /* Check if data buffer has space */
    uint16_t space_needed = size;
    if (c->rx_data_used + space_needed > RX_FIFO_SIZE) {
        c->stats.rx_overruns++;
        trace_el3_rx_overflow(size, RX_FIFO_SIZE - c->rx_data_used);
        return -1;  /* No room */
    }
    
    /* Get next packet descriptor */
    RXPacketDesc *pkt = &c->rx_packets[c->rx_packet_tail];
    
    /* Build RX status word */
    uint16_t rx_status = 0;
    
    if (size < ETH_MIN_DATA_NOFCS) {
        rx_status |= RX_STATUS_ERROR | RX_ERR_RUNT;
    } else if (size > el3_rx_oversize_max(c)) {
        rx_status |= RX_STATUS_ERROR | RX_ERR_OVERSIZE;
    }
    
    /* Store packet descriptor */
    pkt->status = rx_status;
    pkt->length = size;
    pkt->bytes_read = 0;
    pkt->data_offset = c->rx_data_write_ptr;
    pkt->complete = true;
    pkt->arrival_ns = 0;   /* visibility no longer gated on arrival time (see el3_rx_update_complete) */

    /* Copy packet data to buffer */
    uint16_t write_offset = c->rx_data_write_ptr;
    
    /* Handle wrap-around if needed */
    if (write_offset + size <= RX_FIFO_SIZE) {
        memcpy(&c->rx_data[write_offset], buf, size);
    } else {
        /* Split at boundary */
        uint16_t first_part = RX_FIFO_SIZE - write_offset;
        memcpy(&c->rx_data[write_offset], buf, first_part);
        memcpy(&c->rx_data[0], buf + first_part, size - first_part);
    }
    
    /* Update pointers */
    c->rx_data_write_ptr = (write_offset + size) % RX_FIFO_SIZE;
    c->rx_data_used += size;
    c->rx_packet_tail = (c->rx_packet_tail + 1) % RX_MAX_PACKETS;
    c->rx_packet_count++;
    
    /* Update statistics */
    c->stats.rx_frames_ok++;
    c->stats.rx_bytes_ok += size;
    
    /* Make the frame visible to the guest now that it's in the FIFO (RxComplete is a level signal). */
    if (c->int_mask & STAT_RX_COMPLETE) {
        c->pt.rx_complete_irqs++;
    }
    el3_rx_update_complete(c);

    trace_el3_rx_packet(size, rx_status);
    trace_el3_rx_fifo_write(size, c->rx_data_used, RX_FIFO_SIZE - c->rx_data_used);
    
    return size;
}

/* Link status change handler */
void el3_core_set_link_status(NetClientState *nc)
{
    EL3Core *c = qemu_get_nic_opaque(nc);
    bool link_up = !nc->link_down;
    bool was_up = c->link_up;
    
    /* Track link state */
    c->link_up = link_up;
    
    /* Update PHY link status if PHY is present */
    if (c->caps.has_mii) {
        uint16_t old_bmsr = c->phy_regs[MII_BMSR];
        
        if (link_up) {
            c->phy_regs[MII_BMSR] |= BMSR_LINK_STAT;
        } else {
            c->phy_regs[MII_BMSR] &= ~BMSR_LINK_STAT;
        }
        
        /* Generate interrupt if link status changed and interrupts enabled */
        if ((old_bmsr ^ c->phy_regs[MII_BMSR]) & BMSR_LINK_STAT) {
            /* Set link change interrupt flag if supported */
            if (c->model >= MODEL_3C515) {
                c->int_status |= STAT_UPDATE;
                el3_update_irq(c);
            }
        }
    }
    
    /* Update NET_DIAG register link detection bits */
    if (link_up) {
        /* Set appropriate link detection bits based on media type */
        if (c->model >= MODEL_3C515) {
            /* For 3C515 and later, update NET_DIAG */
            c->windows[4][6] |= NET_DIAG_LINK_BEAT_OK | NET_DIAG_TX_FUSE_OK;
        }
        
        /* Flush any queued packets only on down->up transition */
        if (!was_up) {
            qemu_flush_queued_packets(qemu_get_queue(c->nic));
        }
    } else {
        /* Clear link detection bits */
        if (c->model >= MODEL_3C515) {
            c->windows[4][6] &= ~(NET_DIAG_LINK_BEAT_OK | NET_DIAG_TX_FUSE_OK);
        }
    }
    
    trace_el3_link_status(link_up ? "up" : "down");
}

/* TX submission bottom half handler */
static void el3_tx_bh_handler(void *opaque)
{
    EL3Core *c = opaque;
    
    if (c->current_tx_len > 0 && c->tx_fifo_used >= c->current_tx_len) {
        /* We have a complete frame ready to transmit */
        uint8_t buf[ETH_MAX_FRAME_LEN];
        uint16_t len = c->current_tx_len;
        
        /* Sanity check length */
        if (len > sizeof(buf)) {
            trace_el3_tx_oversize(len);
            len = sizeof(buf);
        }
        
        /* Extract frame from TX FIFO */
        for (uint16_t i = 0; i < len; i++) {
            buf[i] = c->tx_fifo[(c->tx_fifo_read_ptr + i) % TX_FIFO_SIZE];
        }
        
        /* Update FIFO pointers */
        c->tx_fifo_read_ptr = (c->tx_fifo_read_ptr + len) % TX_FIFO_SIZE;
        c->tx_fifo_used -= len;
        
        /* Clear TX state */
        c->current_tx_len = 0;
        c->current_tx_written = 0;
        c->tx_in_progress = false;
        
        /* Pad to minimum Ethernet frame size if needed */
        if (len < ETH_MIN_DATA_NOFCS) {
            trace_el3_tx_padding(len, ETH_MIN_DATA_NOFCS);
            memset(buf + len, 0, ETH_MIN_DATA_NOFCS - len);
            len = ETH_MIN_DATA_NOFCS;
        }
        
        /* Send the packet with backpressure handling */
        if (!el3_core_transmit_packet(c, buf, len)) {
            /* Transmission blocked, will be resumed by callback */
            return;
        }
        
        /* Update statistics */
        c->stats.tx_frames_ok++;
        c->stats.tx_bytes_ok += len;

        /* Set TX complete status */
        c->tx_status = TX_STAT_COMPLETE;
        c->status |= STAT_TX_COMPLETE;
        c->int_status |= STAT_TX_COMPLETE;
        
        /* Update TX available status */
        el3_update_tx_available(c);
        el3_update_irq(c);
        
        trace_el3_tx_packet(len);
        trace_el3_tx_complete(TX_STAT_COMPLETE);
    }
}

/* Register read dispatch */
uint32_t el3_core_register_read(EL3Core *c, unsigned win, unsigned off, unsigned size)
{
    uint32_t val = 0;
    
    /* Command/Status register is always accessible */
    if (off == 0x0E) {
        if (size == 2) {
            val = c->status;
        } else {
            val = c->status & 0xFF;
        }
        trace_el3_read(off, val, size, -1);
        return val;
    }

    /* 3C515/Vortex relocate the Window-1 data registers to +0x10 and mirror the
     * Window-0 EEPROM at the +0x2000 ISA alias. Normalize to the 3C509 offsets
     * the window switch below uses (the command register 0x0E is unchanged). */
    if (c->model >= MODEL_3C515) {
        if (off == 0x200A) {
            off = W0_EEPROM_CMD;
        } else if (off == 0x200C) {
            off = W0_EEPROM_DATA;
        } else if (win == 1 && off >= 0x10 && off < 0x20) {
            off -= 0x10;
        }
    }

    /* Try variant-specific handler first */
    if (c->ops && c->ops->reg_read) {
        uint64_t data;
        if (c->ops->reg_read(c, win, off, size, &data) == MEMTX_OK) {
            return data;
        }
    }
    
    /* Core register handling based on window */
    switch (win) {
    case 0: /* EEPROM window */
        switch (off) {
        case W0_EEPROM_CMD:
            /* EepromCommand bit 15 = EepromBusy. Real 3c509/3c515 drivers poll this between issuing
             * the read and reading EepromData; it is the busy bit the data register's 0x8000 placeholder
             * stands in for. Under realtiming the busy window is the same virtual-time gate as
             * el3_eeprom_read (so a polling driver waits exactly the read latency at any icount shift);
             * without realtiming the data is ready immediately, so never busy. (Was "write-only -> 0",
             * which made a polling driver see not-busy instantly and read the 0x8000 placeholder as MAC.) */
            /* The 3C515 reports busy in bit 9 (0x0200, as Linux 3c515 polls), the 3C509/59x in bit 15 */
            val = (c->realtiming && c->eeprom_state == EEPROM_STATE_BUSY &&
                   qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) < c->eeprom_due_ns)
                  ? ((c->model == MODEL_3C515) ? 0x0200 : 0x8000) : 0;
            break;
        case W0_EEPROM_DATA:
            val = el3_eeprom_read(c);
            break;
        default:
            if (off < EL3_WINDOW_SIZE * 2) {
                val = c->windows[win][off >> 1];
            }
            break;
        }
        break;
        
    case 1: /* Operating window */
        switch (off) {
        case W1_TX_RX_FIFO:
            /* Read raw packet data from the RX FIFO. The 3C509 PIO model returns the frame
             * bytes here starting at the Ethernet header; length/status come from W1_RX_STATUS,
             * NOT from prefix words in the FIFO. Supports 8-, 16- and 32-bit reads (386+ drivers
             * drain with `rep insd`); bytes past the frame length are consumed as dword padding. */
            if (c->rx_packet_count > 0) {
                RXPacketDesc *pkt = &c->rx_packets[c->rx_packet_head];
                if (pkt->bytes_read < pkt->length) {
                    unsigned nbytes = (size > 4) ? 4 : size;
                    val = 0;
                    for (unsigned k = 0; k < nbytes; k++) {
                        if (pkt->bytes_read + k < pkt->length) {
                            uint16_t o = (pkt->data_offset + pkt->bytes_read + k) % RX_FIFO_SIZE;
                            val |= (uint32_t)c->rx_data[o] << (8 * k);
                        }
                    }
                    pkt->bytes_read += nbytes;
                }
                trace_el3_fifo_read("RX", size, c->rx_data_used);
            }
            break;
            
        case W1_RX_STATUS:
            /* RxStatus: error/incomplete flags in the high bits, packet length in the low bits --
             * 11-bit on the 3C509, 13-bit on the 3C515/59x. With allowLargePackets set the length
             * can exceed 11 bits (FDDI-sized), so report the 13-bit field; a good large frame has
             * no error bits to collide with the wider length. */
            if (c->rx_packet_count > 0) {
                RXPacketDesc *pkt = &c->rx_packets[c->rx_packet_head];
                uint16_t lenmask = (c->windows[3][W3_MAC_CONTROL >> 1] & MAC_CTRL_ALLOW_LARGE)
                                   ? RX_STATUS_3C59X_LENGTH : RX_STATUS_LENGTH;
                val = pkt->status | (pkt->length & lenmask);
            } else {
                val = RX_STATUS_INCOMPLETE;
            }
            break;
            
        case W1_TIMER:
            /* Free-running timer */
            val = (uint16_t)(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 1000);
            break;
            
        case W1_TX_STATUS:
            val = c->tx_status;
            /* Read clears certain error bits */
            c->tx_status &= ~(TX_STAT_UNDERRUN | TX_STAT_MAX_COLL | TX_STAT_JABBER);
            break;
            
        case W1_TX_FREE:
            val = el3_get_tx_free(c);
            c->pt.txfree_polls++;   /* busy-wait proxy: synchronous drivers spin here */
            break;
            
        default:
            if (off < EL3_WINDOW_SIZE * 2) {
                val = c->windows[win][off >> 1];
            }
            break;
        }
        break;
        
    case 2: /* Station address window */
        /* Offsets 0-5 are the station address registers. Real hardware loads them from the
         * EEPROM MAC at power-on; some drivers (e.g. 3Com's own 3C5X9PD) read the MAC here
         * rather than over the ID port. Mirror conf.macaddr, which writes also update. */
        if (off < 6) {
            if (size == 1) {
                val = c->conf.macaddr.a[off];
            } else {
                val = c->conf.macaddr.a[off] |
                      ((off + 1 < 6) ? (c->conf.macaddr.a[off + 1] << 8) : 0);
            }
        } else if (off < EL3_WINDOW_SIZE * 2) {
            val = c->windows[win][off >> 1];
        }
        break;

    case 3: /* FIFO management window */
    case 4: /* Diagnostics window */
    case 5: /* Command results window (3C509B) */
    case 6: /* Statistics window */
        if (win == 6) {
            /* Statistics are read-to-clear */
            if (off < EL3_WINDOW_SIZE * 2) {
                val = c->windows[win][off >> 1];
                c->windows[win][off >> 1] = 0;  /* Clear on read */
                
                /* Check if all stats have been read */
                bool all_clear = true;
                for (int i = 0; i < EL3_WINDOW_SIZE; i++) {
                    if (c->windows[6][i] != 0) {
                        all_clear = false;
                        break;
                    }
                }
                if (all_clear) {
                    c->status &= ~STAT_STATS_FULL;
                    c->int_status &= ~STAT_STATS_FULL;
                }
            }
        } else {
            if (off < EL3_WINDOW_SIZE * 2) {
                val = c->windows[win][off >> 1];
                /* W4 Media Status (offset 0x0A): report linkBeatDetect so a media-aware driver
                 * (3Com 3C5X9PD enables link-beat then polls for it) sees the 10BaseT link as up
                 * instead of polling forever. The emulated link is always present. */
                if (win == 4 && off == 0x0A) {
                    val |= 0x0800;   /* linkBeatDetect */
                }
            }
        }
        break;

    case 7: /* Bus master window */
        if (off < EL3_WINDOW_SIZE * 2) {
            val = c->windows[win][off >> 1];
        }
        break;
        
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "el3: read from invalid window %d\n", win);
        break;
    }
    
    trace_el3_read(off, val, size, win);
    return val;
}

/* Register write dispatch */
/* realtiming TX: the frame has been fully assembled in the FIFO (its bytes were already added to
 * tx_occupancy as the driver wrote them, so the wire-drain is already in progress). Deliver the
 * frame to the netdev now and schedule TxComplete for when the FIFO finishes draining. */
static void el3_tx_emit_realtiming(EL3Core *c)
{
    uint8_t buf[ETH_MAX_FRAME_LEN];
    uint16_t len = c->current_tx_len;
    if (len == 0 || c->tx_fifo_used < len) {
        return;
    }
    if (len > sizeof(buf)) {
        len = sizeof(buf);
    }
    for (uint16_t i = 0; i < len; i++) {
        buf[i] = c->tx_fifo[(c->tx_fifo_read_ptr + i) % TX_FIFO_SIZE];
    }
    /* the assembly buffer is free again for the next frame; tx_occupancy keeps draining */
    c->tx_fifo_read_ptr = 0;
    c->tx_fifo_write_ptr = 0;
    c->tx_fifo_used = 0;
    c->current_tx_len = 0;
    c->current_tx_written = 0;

    if (len < ETH_MIN_DATA_NOFCS) {
        memset(buf + len, 0, ETH_MIN_DATA_NOFCS - len);
        len = ETH_MIN_DATA_NOFCS;
    }

    (void)el3_core_transmit_packet(c, buf, len);
    c->stats.tx_frames_ok++;
    c->stats.tx_bytes_ok += len;

    /* Parallel Tasking measurement: fill span = first FIFO byte -> now (how long the host took to
     * load this frame, incl. any ISR gaps for an async driver); wire time = len * ns/byte; overlap
     * = the portion of the fill that ran concurrently with the wire (early-start onward). */
    {
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        int64_t fill = c->pt.frame_first_write_ns ? (now - c->pt.frame_first_write_ns) : 0;
        int64_t wire = (int64_t)len * c->tx_ns_per_byte;
        int64_t wstart = c->pt.frame_wire_start_ns ? c->pt.frame_wire_start_ns
                                                   : c->pt.frame_first_write_ns;
        int64_t overlap = (wstart && now > wstart) ? (now - wstart) : 0;
        if (overlap > fill) overlap = fill;
        c->pt.tx_frames++;
        c->pt.tx_bytes += len;
        c->pt.fill_ns_total += fill;
        c->pt.wire_ns_total += wire;
        c->pt.overlap_ns_total += overlap;
        c->pt.frame_first_write_ns = 0;
        /* Periodic snapshot so a watchdog-killed run (the wall-clock-slow interrupt-driven flood
         * never reaches poweroff/atexit) still leaves the running PT metrics in stderr. */
        if ((c->pt.tx_frames % 64) == 0) {
            el3_pt_dump();
        }
    }

    /* TxComplete fires when the occupancy (this frame plus anything still in flight) drains. */
    el3_tx_drain_advance(c);
    if (c->tx_occupancy > 0) {
        c->tx_in_progress = true;
        timer_mod_ns(c->tx_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                     (int64_t)c->tx_occupancy * c->tx_ns_per_byte);
    } else {
        c->tx_in_progress = false;
        c->tx_status = TX_STAT_COMPLETE;
        c->status |= STAT_TX_COMPLETE;
        c->int_status |= STAT_TX_COMPLETE;
        el3_update_irq(c);
    }

    el3_update_tx_available(c);
    trace_el3_tx_packet(len);
}

/* Single-transfer bus-master TX DMA (3C515, first DMA milestone): read one download
 * descriptor at desc_addr from guest memory, DMA the frame buffer it points at, transmit
 * it, and write the DownComplete bit back. Single transfer, no descriptor ring yet. Buffer
 * addresses are guest-physical (286/real-mode: seg*16+off).
 *
 * Completion timing: in realtiming mode the transfer is paced -- TxComplete/IRQ are deferred
 * by the modeled transfer time so DMA throughput is realistic instead of instant. The per-byte
 * cost is the SLOWER of the wire rate (tx_ns_per_byte) and the ISA bus-master rate
 * (dma_rate_bps), i.e. throughput is capped at min(wire, bus) -- the practical ISA bus-master
 * ceiling (~16-24 Mbit) dominates at 100 Mbit, the wire dominates at 10 Mbit. The driver waits
 * IRQ-driven (sti/hlt on g_tx_done), so it actually blocks for this modeled duration. Without
 * realtiming (functional tests) completion is synchronous. */
void el3_core_dma_tx_single(EL3Core *c, AddressSpace *as, hwaddr desc_addr)
{
    EL3DownDesc d;
    uint8_t buf[EL3_LARGE_FRAME_MAX];
    uint16_t len;

    if (address_space_read(as, desc_addr, MEMTXATTRS_UNSPECIFIED, &d, sizeof(d)) != MEMTX_OK) {
        return;
    }
    len = d.length & EL3_DESC_LENGTH_MASK;
    if (len == 0) {
        return;
    }
    if (len > sizeof(buf)) {
        len = sizeof(buf);
    }
    if (address_space_read(as, d.addr, MEMTXATTRS_UNSPECIFIED, buf, len) == MEMTX_OK) {
        if (len < ETH_MIN_DATA_NOFCS) {
            memset(buf + len, 0, ETH_MIN_DATA_NOFCS - len);
            len = ETH_MIN_DATA_NOFCS;
        }
        if (c->internal_loopback) {
            /* MAC internal loopback (NET_DIAG bit 5): the frame never reaches the wire -- it is
             * received straight back into the armed RX-DMA up-descriptor. Drives the driver's cold
             * cache-coherency self-test. Clear the RX-DMA bus-pacing gate first so the loopback
             * (no external bus contention) is never spuriously back-pressured between probe iterations. */
            c->rx_dma_ready_ns = 0;
            (void)el3_core_dma_rx_single(c, buf, len);
        } else {
            (void)el3_core_transmit_packet(c, buf, len);
        }
        c->stats.tx_frames_ok++;
        c->stats.tx_bytes_ok += len;
    }
    /* mark the descriptor complete (the card consumed it) */
    d.status |= EL3_DESC_DOWN_COMPLETE;
    address_space_write(as, desc_addr, MEMTXATTRS_UNSPECIFIED, &d, sizeof(d));

    if (c->realtiming) {
        /* defer TxComplete/IRQ by the modeled transfer time (slower of wire vs bus rate) */
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        int64_t base = (c->tx_drain_deadline_ns > now) ? c->tx_drain_deadline_ns : now;
        int64_t bus_ns_per_byte = c->dma_rate_bps ? (1000000000LL / c->dma_rate_bps) : 0;
        int64_t per_byte = (bus_ns_per_byte > c->tx_ns_per_byte)
                           ? bus_ns_per_byte : c->tx_ns_per_byte;
        c->tx_drain_deadline_ns = base + (int64_t)len * per_byte;
        c->tx_in_progress = true;
        timer_mod_ns(c->tx_timer, c->tx_drain_deadline_ns);
        /* el3_tx_drain_timer_cb raises STAT_TX_COMPLETE + IRQ at the deadline */
    } else {
        c->status |= STAT_TX_COMPLETE;
        c->int_status |= STAT_TX_COMPLETE;
        el3_update_irq(c);
    }
}

void el3_core_register_write(EL3Core *c, unsigned win, unsigned off,
                             uint32_t val, unsigned size)
{
    trace_el3_write(off, val, size, win);
    
    /* Command register is always accessible */
    if (off == 0x0E) {
        el3_process_command(c, val);
        return;
    }

    /* 3C515/Vortex relocate the Window-1 data registers to +0x10 and mirror the
     * Window-0 EEPROM at the +0x2000 ISA alias. Normalize to the 3C509 offsets. */
    if (c->model >= MODEL_3C515) {
        if (off == 0x200A) {
            off = W0_EEPROM_CMD;
        } else if (off == 0x200C) {
            off = W0_EEPROM_DATA;
        } else if (win == 1 && off >= 0x10 && off < 0x20) {
            off -= 0x10;
        }
    }

    /* Try variant-specific handler first */
    if (c->ops && c->ops->reg_write) {
        if (c->ops->reg_write(c, win, off, size, val) == MEMTX_OK) {
            return;
        }
    }
    
    /* Core register handling based on window */
    switch (win) {
    case 0: /* EEPROM window */
        switch (off) {
        case W0_EEPROM_CMD:
            el3_eeprom_cmd(c, val);
            break;
        case W0_EEPROM_DATA:
            /* Read-only */
            break;
        default:
            if (off < EL3_WINDOW_SIZE * 2) {
                c->windows[win][off >> 1] = val;
            }
            break;
        }
        break;
        
    case 1: /* Operating window */
        switch (off) {
        case W1_TX_RX_FIFO:
            /* PIO TX (3c509/Vortex): the driver writes a 32-bit preamble -- word 0 is the
             * frame length, word 1 is zero -- then the frame bytes. Parse the preamble, then
             * collect exactly current_tx_len data bytes (preamble stripped) and hand the
             * complete frame to the TX path. Bytes are fed one at a time so 8-, 16- and 32-bit
             * writes are handled uniformly -- 386+ drivers (3Com 3C5X9PD, and our own driver's
             * 386 fragment) stream the FIFO with 32-bit `rep outsd`, so size==4 must feed all
             * four bytes here. */
            if (!c->tx_enabled) {
                c->tx_status |= TX_STAT_UNDERRUN;
                c->stats.tx_underruns++;
                trace_el3_tx_underrun();
                break;
            }
            {
                unsigned nbytes = (size > 4) ? 4 : size;
                /* Streaming TX: every byte written occupies FIFO space (preamble + data) until
                 * the card drains it at wire rate. Advance the drain, then add this write so
                 * TxFree falls as the driver fills. */
                el3_tx_drain_advance(c);
                c->tx_occupancy += nbytes;
                c->pt.fifo_writes++;
                c->pt.pt_active = true;
                for (unsigned k = 0; k < nbytes; k++) {
                    uint8_t b = (val >> (8 * k)) & 0xFF;
                    if (c->tx_preamble_pos < 4) {
                        /* preamble: bytes 0-1 = length (little-endian), 2-3 = 0 (ignored) */
                        if (c->tx_preamble_pos == 0) {
                            c->pt.frame_first_write_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
                            c->pt.frame_wire_start_ns = 0;
                            c->current_tx_len = b;
                            c->tx_fifo_write_ptr = 0;   /* start a fresh frame buffer */
                            c->tx_fifo_read_ptr = 0;
                            c->tx_fifo_used = 0;
                            c->current_tx_written = 0;
                        } else if (c->tx_preamble_pos == 1) {
                            c->current_tx_len |= (uint16_t)b << 8;
                        }
                        c->tx_preamble_pos++;
                        if (c->tx_preamble_pos == 4) {
                            c->tx_in_progress = true;
                        }
                    } else if (c->tx_fifo_used < TX_FIFO_SIZE) {
                        c->tx_fifo[c->tx_fifo_write_ptr] = b;
                        c->tx_fifo_write_ptr = (c->tx_fifo_write_ptr + 1) % TX_FIFO_SIZE;
                        c->tx_fifo_used++;
                        c->current_tx_written++;
                    }
                }

                trace_el3_fifo_write("TX", size, c->tx_fifo_used);

                /* full frame collected -> transmit it now (synchronous: no race with the
                 * next frame's preamble; el3_tx_bh_handler extracts + sends + sets status).
                 *
                 * The TX FIFO is dword-oriented: the driver writes the frame data padded up to a
                 * dword boundary (real 3c509/Vortex drivers, e.g. Crynwr, round the byte count up
                 * to a multiple of 4). Wait for the full padded length before completing, then
                 * transmit exactly current_tx_len bytes -- otherwise the trailing pad bytes get
                 * misread as the next frame's preamble and desync every subsequent frame.
                 *
                 * Gate on the assembled length only -- NOT tx_in_progress: that flag doubles as
                 * the realtiming drain-in-flight marker, and the previous frame's drain timer can
                 * clear it mid-assembly of the next frame (slow CPU + fast wire, e.g. PIO@100 on
                 * a 386), which would jam the FIFO and drop every subsequent frame. */
                if (c->current_tx_len > 0 &&
                    c->current_tx_written >= (uint16_t)((c->current_tx_len + 3u) & ~3u)) {
                    c->tx_preamble_pos = 0;     /* ready for the next frame */
                    if (c->realtiming) {
                        el3_tx_emit_realtiming(c);  /* send now, model wire time */
                    } else {
                        el3_tx_bh_handler(c);       /* instant: send + status now */
                    }
                }
                el3_update_tx_available(c);
            }
            break;

        case W1_TX_STATUS:
            /* TX status stack pop. Each completed TX pushes a status byte; the driver reads it at
             * W1_TX_STATUS, then writes the register to discard/advance the stack. The main-status
             * TxComplete bit (0x04) reflects "stack non-empty", so popping clears it. Our model
             * keeps a single pending TX status, so any write empties the stack and clears
             * TxComplete -- without this, a stack-driven driver (3C5X9PD on the 286 16-bit path)
             * spins forever reading status=TxComplete after a frame. */
            c->tx_status = 0;
            c->status &= ~STAT_TX_COMPLETE;
            c->int_status &= ~STAT_TX_COMPLETE;
            el3_update_irq(c);
            break;

        case W1_TIMER:
            /* Test-only RX generator trigger. W1_TIMER is a read-only free-running timer on real
             * silicon, so a guest write is otherwise meaningless. When the rxgen feature is enabled
             * (rx_gen_size != 0) rxcount writes the magic 0x5247 ('RG') to ARM the wire-rate RX
             * generator at the start of its measurement window, and 0 to disarm -- a magic value (not
             * just nonzero) so an incidental driver write to this register can't trigger it. Keeps
             * the flood off while DOS loads/registers the probe. No-op when rxgen is disabled. */
            if (c->rx_gen_size) {
                if (val == 0x5247) {
                    if (!timer_pending(c->rx_gen_timer)) {
                        c->rx_gen_start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
                        c->rx_gen_consumed = 0;
                        c->rx_gen_count = 0;
                        c->rx_gen_dropped = 0;
                        c->rx_gen_seq = 0x10000;   /* L4: una = the guest's forced rcv_nxt */
                        c->rx_gen_next = 0x10000;  /* next seq to send = una -> window opens at frame 1 */
                        c->rx_gen_stall = 0;
                        c->rx_gen_reported = false;
                        timer_mod_ns(c->rx_gen_timer, c->rx_gen_start_ns + 1000);
                    }
                } else if (val == 0) {
                    timer_del(c->rx_gen_timer);
                }
                break;   /* real W1_TIMER is read-only: ignore any other value */
            }
            if (off < EL3_WINDOW_SIZE * 2) {
                c->windows[win][off >> 1] = val;
            }
            break;

        default:
            if (off < EL3_WINDOW_SIZE * 2) {
                c->windows[win][off >> 1] = val;
            }
            break;
        }
        break;

    case 2: /* Station address window */
        /* MAC address programming */
        if (off < 6) {
            int idx = off;
            if (size == 1) {
                ((uint8_t *)c->conf.macaddr.a)[idx] = val;
            } else if (size == 2 && idx + 1 < 6) {
                c->conf.macaddr.a[idx] = val & 0xFF;
                c->conf.macaddr.a[idx + 1] = (val >> 8) & 0xFF;
            }
        } else if (off < EL3_WINDOW_SIZE * 2) {
            c->windows[win][off >> 1] = val;
        }
        break;
        
    case 3: /* FIFO management window */
    case 4: /* Diagnostics window */
        if (off == 0x06) {
            /* NET_DIAG (W4 off 6): bit 5 = internal MAC loopback. When the driver sets it, a
             * subsequent bus-master TX is looped straight back into the armed RX-DMA up-descriptor
             * (el3_core_dma_tx_single) instead of reaching the wire -- the mechanism the driver's
             * cold cache-coherency self-test uses to make the card DMA-write a known pattern into a
             * CPU-warmed buffer. Real 3c515 silicon loops internally in this mode; cleared = normal. */
            c->internal_loopback = (val & NET_DIAG_INTERNAL_LB) != 0;
        }
        if (off < EL3_WINDOW_SIZE * 2) {
            c->windows[win][off >> 1] = val;
        }
        break;
    case 5: /* Command results window */
    case 6: /* Statistics window */
    case 7: /* Bus master window */
        if (off < EL3_WINDOW_SIZE * 2) {
            c->windows[win][off >> 1] = val;
        }
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR, "el3: write to invalid window %d\n", win);
        break;
    }
}

/* Legacy wrappers for compatibility */
uint32_t el3_core_read(EL3Core *c, hwaddr addr, unsigned size)
{
    return el3_core_register_read(c, c->current_window, addr, size);
}

void el3_core_write(EL3Core *c, hwaddr addr, uint32_t val, unsigned size)
{
    el3_core_register_write(c, c->current_window, addr, val, size);
}

/* ISA ID port handling (3C509 specific).
 *
 * Genuine 3c509 enumeration (as done by the Crynwr / 3Com DOS drivers): the driver writes the
 * ID pattern to the ID port (0x110) -- two 0x00 bytes to reset the card's pattern generator,
 * then 255 bytes from an 8-bit LFSR (seed 0xFF, next = (x<<1) ^ 0xCF when bit7 set). The card
 * runs an identical generator and enters isolation after a full 255-byte match. The driver then
 * issues commands: ID_GLOBAL_RESET (0xC0), SET_TAG (0xD0), TEST_TAG (0xD8), READ_EEPROM|word
 * (0x80|n, after which it clocks 16 bits MSB-first out of the ID port -- the bit appears in bit
 * 0 of each read), and ACTIVATE_AND_SET_IO (0xE0|n) / ACTIVATE_VULCAN (0xFF). */
static inline uint8_t el3_id_lfsr_next(uint8_t g)
{
    return (g & 0x80) ? (uint8_t)((g << 1) ^ 0xCF) : (uint8_t)(g << 1);
}

void el3_id_port_write(EL3Core *c, uint8_t val)
{
    EL3IDState *id = &c->id_state;
    trace_el3_id_write(val);

    switch (id->state) {
    case ID_IDLE:
    case ID_UNLOCKING:
        /* During the ID-pattern phase every byte is pattern data -- including bytes that double
         * as command codes (0xC0 GLOBAL_RESET, 0xD0, 0xFF, ...), since the maximal-length LFSR
         * cycles through all 255 non-zero values. bit_pos holds the running generator byte. */
        if (id->unlock_pos == 0) {
            id->bit_pos = 0xFF;            /* (re)seed at the start of a sequence */
        }
        if (val == id->bit_pos) {
            id->unlock_pos++;
            id->bit_pos = el3_id_lfsr_next(id->bit_pos);
            id->state = ID_UNLOCKING;
            if (id->unlock_pos >= 255) {
                /* Full ID pattern matched -> isolation. */
                id->state = ID_ISOLATION;
                id->unlock_pos = 0;
                id->eeprom_stream_bits = 0;
                id->tag_register = 0;
            }
        } else {
            /* Mismatch (e.g. the two leading 0x00 resets) -> restart. */
            id->unlock_pos = 0;
            id->bit_pos = 0xFF;
            id->state = ID_IDLE;
            trace_el3_id_reset();
        }
        break;

    case ID_ISOLATION:
    case ID_SELECTED:
    case ID_CONFIG:
        /* Post-unlock command phase. GLOBAL_RESET returns to the unlock state (the driver issues
         * it between the two ID-pattern writes). */
        if (val == 0xC0) {                       /* ID_GLOBAL_RESET */
            id->state = ID_IDLE;
            id->unlock_pos = 0;
            id->bit_pos = 0xFF;
            id->eeprom_stream_bits = 0;
            trace_el3_id_reset();
        } else if (id->state != ID_ISOLATION) {
            /* Activated/selected: ignore everything except GLOBAL_RESET until reset. */
        } else if (val == 0xD0) {                /* SET_TAG_REGISTER */
            id->tag_register = 0;                /* single card stays responsive */
        } else if (val == 0xD8) {                /* TEST_TAG_REGISTER -- no-op for one card */
            /* nothing */
        } else if (val == 0xFF) {                /* ACTIVATE_VULCAN */
            id->state = ID_CONFIG;
        } else if ((val & 0xF0) == 0xE0) {       /* ACTIVATE_AND_SET_IO */
            id->state = ID_CONFIG;
        } else if (val >= 0x80 && val <= 0xBF) { /* READ_EEPROM | word */
            uint8_t word = val & 0x3F;
            id->eeprom_stream_data = (word < 64) ? c->eeprom[word] : 0xFFFF;
            id->eeprom_stream_addr = word;
            id->eeprom_stream_bits = 16;
        }
        break;
    }
}

uint8_t el3_id_port_read(EL3Core *c)
{
    EL3IDState *id = &c->id_state;
    uint8_t data = 0xFF;

    /* Clock out the addressed EEPROM word, MSB first, one bit per read in bit 0. */
    if (id->eeprom_stream_bits > 0) {
        data = (id->eeprom_stream_data >> 15) & 1;
        id->eeprom_stream_data <<= 1;
        id->eeprom_stream_bits--;
    }

    trace_el3_id_read(data, id->eeprom_stream_bits);
    return data;
}

void el3_id_sequence_reset(EL3Core *c)
{
    c->id_state.state = ID_IDLE;
    c->id_state.unlock_pos = 0;
    c->id_state.bit_pos = 0xFF;            /* LFSR seed */
    c->id_state.selected_tag = 0;
    c->id_state.eeprom_stream_bits = 0;
    c->id_state.tag_register = 0;
    trace_el3_id_reset();
}

/* Core initialization */
/* Parallel Tasking instrumentation: dump the measured host/wire-overlap + CPU-cost metrics so the
 * advantage of an async/early-start driver (3Com 3C5X9PD) over the synchronous ones (Crynwr,
 * clean-room) is quantifiable. Registered via atexit; the flood harness captures stderr. */
static EL3Core *el3_pt_core;
static void el3_pt_dump(void)
{
    EL3Core *c = el3_pt_core;
    uint64_t f;
    if (!c || !c->pt.pt_active) {
        return;
    }
    f = c->pt.tx_frames ? c->pt.tx_frames : 1;
    fprintf(stderr,
        "[PT-STATS] tx_frames=%llu tx_bytes=%llu | per-frame: txfree_polls=%.1f fifo_writes=%.1f "
        "tx_avail_irqs=%.2f tx_complete_irqs=%.2f | fill=%lldus wire=%lldus overlap=%lldus "
        "(%.0f%% of fill) | rx_complete_irqs=%llu\n",
        (unsigned long long)c->pt.tx_frames, (unsigned long long)c->pt.tx_bytes,
        (double)c->pt.txfree_polls / f, (double)c->pt.fifo_writes / f,
        (double)c->pt.tx_avail_irqs / f, (double)c->pt.tx_complete_irqs / f,
        (long long)(c->pt.fill_ns_total / 1000 / (int64_t)f),
        (long long)(c->pt.wire_ns_total / 1000 / (int64_t)f),
        (long long)(c->pt.overlap_ns_total / 1000 / (int64_t)f),
        c->pt.fill_ns_total ? 100.0 * c->pt.overlap_ns_total / c->pt.fill_ns_total : 0.0,
        (unsigned long long)c->pt.rx_complete_irqs);
}

void el3_core_init(EL3Core *c, EL3Model model, const EL3VariantOps *ops)
{
    c->model = model;
    c->ops = ops;
    c->caps = el3_caps[model];
    el3_pt_core = c;
    atexit(el3_pt_dump);
    
    /* Initialize EEPROM timer */
    c->eeprom_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, el3_eeprom_timer_cb, c);
    c->eeprom_latency_ns = EEPROM_DELAY_NS;

    /* Hardware-delay timing model timers (used only when realtiming is enabled). */
    c->tx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, el3_tx_drain_timer_cb, c);
    c->rx_pace_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, el3_rx_pace_timer_cb, c);
    c->rx_gen_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, el3_rx_gen_timer_cb, c);
    c->reflect_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, el3_reflect_timer_cb, c);
    c->nvmet_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, el3_nvmet_timer_cb, c);
    c->nvmet_rtx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, el3_nvmet_rtx_timer_cb, c);
    c->l4dbg_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, el3_l4dbg_timer_cb, c);
    c->cmd_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, el3_cmd_timer_cb, c);
    c->tx_drain_deadline_ns = 0;
    if (c->tx_ns_per_byte == 0) {
        c->tx_ns_per_byte = 800;   /* 10BaseT: ~1.25 MB/s -> 800 ns/byte */
    }
    
    /* Initialize TX bottom half */
    c->tx_bh = NULL; /* Will be initialized in el3_core_change_aio_ctx */
    
    /* Initialize TX backpressure state */
    c->tx_blocked = false;
    
    /* Initialize media timer for MII/PHY autonegotiation */
    if (c->caps.has_mii) {
        c->media_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, el3_media_timer_cb, c);
        el3_phy_init(c);
    }
    
    /* Initialize multicast hash to accept all by default */
    memset(c->mcast_hash, 0xFF, sizeof(c->mcast_hash));
    
    /* Initialize EEPROM based on model */
    if (model == MODEL_3C509 || model == MODEL_3C509B) {
        el3_eeprom_init_3c509(c);
    } else {
        el3_eeprom_init_3c59x(c);
    }
}

/* Core reset */
void el3_core_reset(EL3Core *c)
{
    /* Reset window to 0 */
    c->current_window = 0;
    
    /* Clear all windows */
    memset(c->windows, 0, sizeof(c->windows));

    /* 3C515 InternalConfig (W3 off 0, 32-bit): xcvrSelect (bits 20-22) names the active transceiver.
     * Report the one the realtiming link speed implies -- 4 = 100BASE-TX at 100 Mbit (80 ns/byte),
     * 0 = 10BASE-T otherwise -- so a driver can derive the link speed from it as on real hardware. */
    if (c->model == MODEL_3C515) {
        c->windows[3][1] = (uint16_t)(((c->tx_ns_per_byte == 80) ? 4 : 0) << 4);
    }
    
    /* Reset status and interrupts */
    c->status = 0;
    c->int_status = 0;
    c->int_mask = 0;
    c->intr_enb = 0;
    c->status_enb = 0;
    
    /* Reset enables */
    c->rx_enabled = false;
    c->tx_enabled = false;
    
    /* Reset thresholds */
    c->tx_avail_thresh = 0x0080;  /* Default 128 bytes */
    c->tx_start_thresh = 0x0080;
    c->rx_early_thresh = 0;
    
    /* Reset RX filter to accept broadcasts and our MAC */
    c->rx_filter = RX_FILTER_INDIVIDUAL | RX_FILTER_BROADCAST;
    
    /* Reset FIFOs */
    c->tx_fifo_write_ptr = 0;
    c->tx_fifo_read_ptr = 0;
    c->tx_fifo_used = 0;
    c->tx_in_progress = false;
    c->current_tx_len = 0;
    c->current_tx_written = 0;
    c->tx_status = 0;
    c->tx_occupancy = 0;
    c->tx_drain_last_ns = 0;
    c->tx_wire_active = false;
    c->tx_avail_armed = false;
    
    c->rx_packet_count = 0;
    c->rx_packet_head = 0;
    c->rx_packet_tail = 0;
    c->rx_data_write_ptr = 0;
    c->rx_data_used = 0;
    
    /* Reset statistics */
    memset(&c->stats, 0, sizeof(c->stats));
    memset(&c->stats_snapshot, 0, sizeof(c->stats_snapshot));
    c->stats_enabled = false;
    c->stats_frozen = false;
    
    /* Reset EEPROM state */
    c->eeprom_state = EEPROM_STATE_IDLE;
    
    /* Reset ID sequence state */
    el3_id_sequence_reset(c);
    
    /* Reset DMA state */
    c->up_list_ptr = 0;
    c->down_list_ptr = 0;
    c->up_stalled = false;
    c->down_stalled = false;
    c->rx_dma_armed = false;
    c->bus_master_enabled = false;
    c->irq_level = false;
    
    /* Reset TX backpressure state */
    c->tx_blocked = false;
    
    /* Cancel any pending timers */
    if (c->eeprom_timer) {
        timer_del(c->eeprom_timer);
    }
    if (c->media_timer) {
        timer_del(c->media_timer);
    }
    
    /* Cancel any pending bottom halves */
    if (c->tx_bh) {
        qemu_bh_cancel(c->tx_bh);
    }
    
    trace_el3_core_reset();
}

/* ISA DMA functions for 3C515 support */

/* Read ISA DMA descriptor with 24-bit address masking */
static bool G_GNUC_UNUSED el3_isa_dma_read_desc(EL3Core *c, hwaddr addr, EL3ISADmaDesc *desc)
{
    uint8_t buf[16];  /* ISA descriptors are 16 bytes */
    
    /* Mask to 24-bit ISA address space */
    addr &= ISA_DMA_MAX_ADDR;
    
    /* Read descriptor from memory */
    if (dma_memory_read(&address_space_memory, addr, buf, sizeof(buf),
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        trace_el3_dma_error(addr, "descriptor read failed");
        return false;
    }
    
    /* Parse descriptor fields as little-endian */
    desc->next = ldl_le_p(&buf[0]) & ISA_DMA_MAX_ADDR;
    desc->status = ldl_le_p(&buf[4]);
    desc->addr = ldl_le_p(&buf[8]) & ISA_DMA_MAX_ADDR;
    desc->length = ldl_le_p(&buf[12]);
    
    trace_el3_dma_desc_read(addr, desc->next, desc->status, desc->addr, desc->length);
    return true;
}

/* Write status back to ISA DMA descriptor */
static bool G_GNUC_UNUSED el3_isa_dma_write_desc_status(EL3Core *c, hwaddr addr, uint32_t status)
{
    uint8_t buf[4];
    
    /* Mask to 24-bit ISA address space */
    addr &= ISA_DMA_MAX_ADDR;
    
    /* Status field is at offset 4 in descriptor */
    addr += 4;
    
    /* Convert status to little-endian */
    stl_le_p(buf, status);
    
    /* Write status back to memory */
    if (dma_memory_write(&address_space_memory, addr, buf, sizeof(buf),
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        trace_el3_dma_error(addr, "status write failed");
        return false;
    }
    
    trace_el3_dma_desc_status_write(addr, status);
    return true;
}

/* Validate DMA address is in RAM and not ROM/MMIO */
static bool G_GNUC_UNUSED el3_validate_dma_address(EL3Core *c, hwaddr addr, size_t len, bool is_write)
{
    MemoryRegionSection mrs;
    bool result = false;
    uint64_t end;
    
    /* Check for zero-length transfers */
    if (len == 0) {
        trace_el3_dma_error(addr, "zero-length DMA transfer");
        c->status |= STAT_ADAPTER_FAIL;
        c->int_status |= STAT_ADAPTER_FAIL;
        return false;
    }
    
    /* Calculate end address once to avoid repeated arithmetic */
    end = (uint64_t)addr + len - 1;
    
    /* Check 24-bit ISA address space - both start and end */
    if (addr > 0xFFFFFF) {
        trace_el3_dma_error(addr, "address exceeds 24-bit ISA space");
        c->status |= STAT_ADAPTER_FAIL;
        c->int_status |= STAT_ADAPTER_FAIL;
        return false;
    }
    
    /* Check for address wraparound and 24-bit limit for end address */
    if (end < addr || end > 0xFFFFFF) {
        trace_el3_dma_error(addr, "DMA would exceed 24-bit boundary or wrap");
        c->status |= STAT_ADAPTER_FAIL;
        c->int_status |= STAT_ADAPTER_FAIL;
        return false;
    }
    
    /* Check 64KB DMA boundary crossing 
     * Note: This is a real 3C515 hardware constraint for ISA bus master DMA.
     * The 3C515 cannot cross 64KB boundaries in a single DMA operation.
     */
    if ((addr & 0xFFFF0000) != (end & 0xFFFF0000)) {
        trace_el3_dma_error(addr, "DMA crosses 64KB boundary");
        c->status |= STAT_ADAPTER_FAIL;
        c->int_status |= STAT_ADAPTER_FAIL;
        return false;
    }
    
    /* Find the memory region */
    mrs = memory_region_find(get_system_memory(), addr, len);
    
    if (!mrs.mr) {
        trace_el3_dma_error(addr, "no memory region found");
        c->status |= STAT_ADAPTER_FAIL;
        c->int_status |= STAT_ADAPTER_FAIL;
        return false;
    }
    
    /* Check if the entire transfer fits within the found region */
    if (mrs.size < len) {
        trace_el3_dma_error(addr, "DMA spans multiple memory regions");
        c->status |= STAT_ADAPTER_FAIL;
        c->int_status |= STAT_ADAPTER_FAIL;
        goto cleanup;
    }
    
    /* Check if region is RAM - 3C515 only does DMA to/from system RAM */
    if (!memory_region_is_ram(mrs.mr)) {
        trace_el3_dma_error(addr, "DMA to non-RAM region rejected");
        c->status |= STAT_ADAPTER_FAIL;
        c->int_status |= STAT_ADAPTER_FAIL;
        goto cleanup;
    }
    
    /* Check for read-only regions on writes (ROM/ROMD protection) */
    if (is_write && (memory_region_is_rom(mrs.mr) || memory_region_is_romd(mrs.mr))) {
        trace_el3_dma_error(addr, "DMA write to ROM/ROMD rejected");
        c->status |= STAT_ADAPTER_FAIL;
        c->int_status |= STAT_ADAPTER_FAIL;
        goto cleanup;
    }
    
    /* Success */
    trace_el3_validate_dma_address(addr, len, is_write ? "write" : "read");
    result = true;
    
cleanup:
    if (mrs.mr) {
        memory_region_unref(mrs.mr);
    }
    return result;
}

/* Process TX descriptor chain */
static void el3_process_tx_chain(QEMU_UNUSED EL3Core *c)
{
    /* Implementation removed for production quality */
}

/* Process RX descriptor chain */
static void el3_process_rx_chain(QEMU_UNUSED EL3Core *c)
{
    /* Implementation removed for production quality */
}

/* DMA stub functions (to be implemented) */
void el3_dma_init(EL3DMAEngine *dma, AddressSpace *as, void *opaque, AioContext *ctx)
{
    memset(dma, 0, sizeof(*dma));
    dma->as = as;
    dma->opaque = opaque;
    dma->state = DMA_IDLE;
    dma->enabled = false;
    dma->ctx = ctx;
    
    /* Create bottom half handlers for async DMA */
    dma->tx_bh = aio_bh_new(ctx, el3_dma_tx_bh, dma);
    dma->rx_bh = aio_bh_new(ctx, el3_dma_rx_bh, dma);
}

void el3_dma_cleanup(EL3DMAEngine *dma)
{
    if (!dma) {
        return;
    }
    
    /* Cancel any scheduled bottom half operations */
    if (dma->tx_bh) {
        qemu_bh_cancel(dma->tx_bh);
        qemu_bh_delete(dma->tx_bh);
        dma->tx_bh = NULL;
    }
    
    if (dma->rx_bh) {
        qemu_bh_cancel(dma->rx_bh);
        qemu_bh_delete(dma->rx_bh);
        dma->rx_bh = NULL;
    }
    
    /* Free any pending RX frame */
    if (dma->pending_frame) {
        g_free(dma->pending_frame);
        dma->pending_frame = NULL;
    }
    
    dma->state = DMA_IDLE;
    dma->enabled = false;
    dma->tx_scheduled = 0;
    dma->rx_scheduled = 0;
}

void el3_dma_reset(EL3DMAEngine *dma)
{
    if (!dma) {
        return;
    }
    
    /* Cancel pending operations */
    if (dma->tx_bh) {
        qemu_bh_cancel(dma->tx_bh);
    }
    if (dma->rx_bh) {
        qemu_bh_cancel(dma->rx_bh);
    }
    
    /* Free pending frame */
    if (dma->pending_frame) {
        g_free(dma->pending_frame);
        dma->pending_frame = NULL;
    }
    
    /* Reset state */
    dma->base_addr = 0;
    dma->current_desc = 0;
    dma->ring_size = 0;
    dma->state = DMA_IDLE;
    dma->enabled = false;
    dma->transfer_len = 0;
    dma->buffer_addr = 0;
    dma->pending_len = 0;
    dma->pending_status = 0;
    
    memset(&dma->down_desc, 0, sizeof(dma->down_desc));
    memset(&dma->up_desc, 0, sizeof(dma->up_desc));
    
    /* Clear scheduled flags */
    qatomic_set(&dma->tx_scheduled, 0);
    qatomic_set(&dma->rx_scheduled, 0);
}

void el3_dma_set_ring_base(EL3DMAEngine *dma, hwaddr addr, uint32_t size)
{
    if (!dma) {
        return;
    }
    
    dma->base_addr = addr;
    dma->ring_size = size;
    dma->current_desc = 0;
    
    trace_el3_dma_ring_set(addr, size);
}

bool el3_dma_start_download(EL3DMAEngine *dma)
{
    if (!dma || !dma->enabled) {
        return false;
    }
    
    if (dma->state != DMA_IDLE) {
        return false;  /* Already busy */
    }
    
    dma->state = DMA_DOWNLOADING;
    
    /* Schedule bottom half to start processing */
    el3_dma_schedule_tx(dma);
    
    trace_el3_dma_start(true, dma->base_addr);
    return true;
}

bool el3_dma_start_upload(EL3DMAEngine *dma)
{
    if (!dma || !dma->enabled) {
        return false;
    }
    
    if (dma->state != DMA_IDLE) {
        return false;  /* Already busy */
    }
    
    dma->state = DMA_UPLOADING;
    
    /* Schedule bottom half to start processing */
    el3_dma_schedule_rx(dma);
    
    trace_el3_dma_start(false, dma->base_addr);
    return true;
}

void el3_dma_kick(EL3DMAEngine *dma, bool is_tx)
{
    if (!dma || !dma->enabled) {
        return;
    }
    
    EL3Core *c = (EL3Core *)dma->opaque;
    
    /* Check if TX is blocked */
    if (is_tx && c->tx_blocked) {
        return;
    }
    
    if (is_tx) {
        if (dma->state == DMA_IDLE || dma->state == DMA_DOWNLOADING) {
            dma->state = DMA_DOWNLOADING;
            el3_dma_schedule_tx(dma);
        }
    } else {
        if (dma->state == DMA_IDLE || dma->state == DMA_UPLOADING) {
            dma->state = DMA_UPLOADING;
            el3_dma_schedule_rx(dma);
        }
    }
}

bool el3_dma_is_idle(EL3DMAEngine *dma)
{
    if (!dma) {
        return true;
    }
    
    return dma->state == DMA_IDLE;
}

uint32_t el3_dma_get_status(EL3DMAEngine *dma)
{
    uint32_t status = 0;
    
    if (!dma) {
        return 0;
    }
    
    switch (dma->state) {
    case DMA_IDLE:
        status = 0x0000;
        break;
    case DMA_DOWNLOADING:
        status = 0x0001;  /* TX active */
        break;
    case DMA_UPLOADING:
        status = 0x0002;  /* RX active */
        break;
    case DMA_ERROR:
        status = 0x8000;  /* Error flag */
        break;
    default:
        break;
    }
    
    if (dma->enabled) {
        status |= 0x0100;  /* DMA enabled flag */
    }
    
    return status;
}

void el3_dma_tx_submit(EL3DMAEngine *dma, const uint8_t *data, size_t len)
{
    if (!dma || !dma->enabled || !data || len == 0) {
        return;
    }
    
    /* For immediate transmission, just schedule the BH */
    dma->pending_frame = g_malloc(len);
    memcpy(dma->pending_frame, data, len);
    dma->pending_len = len;
    
    /* Kick TX processing */
    el3_dma_kick(dma, true);
}

void el3_dma_rx_submit(EL3DMAEngine *dma, const uint8_t *data, size_t len, uint32_t status)
{
    if (!dma || !dma->enabled || !data || len == 0) {
        return;
    }
    
    /* Store RX frame for upload */
    if (dma->pending_frame) {
        g_free(dma->pending_frame);
    }
    
    dma->pending_frame = g_malloc(len);
    memcpy(dma->pending_frame, data, len);
    dma->pending_len = len;
    dma->pending_status = status;
    
    /* Kick RX processing */
    el3_dma_kick(dma, false);
}

static void el3_dma_schedule_tx(EL3DMAEngine *dma)
{
    if (!dma || !dma->tx_bh) {
        return;
    }
    
    EL3Core *c = (EL3Core *)dma->opaque;
    
    /* Check if TX is blocked */
    if (c->tx_blocked) {
        return;
    }
    
    /* Use atomic exchange to check and set the scheduled flag */
    if (qatomic_xchg(&dma->tx_scheduled, 1) == 0) {
        trace_el3_bh_schedule_tx();
        qemu_bh_schedule(dma->tx_bh);
    }
}

static void el3_dma_schedule_rx(EL3DMAEngine *dma)
{
    if (!dma || !dma->rx_bh) {
        return;
    }
    
    /* Use atomic exchange to check and set the scheduled flag */
    if (qatomic_xchg(&dma->rx_scheduled, 1) == 0) {
        trace_el3_bh_schedule_rx();
        qemu_bh_schedule(dma->rx_bh);
    }
}

void el3_dma_tx_bh(void *opaque)
{
    EL3DMAEngine *dma = opaque;
    EL3Core *c = dma->opaque;
    int desc_count = 0;
    int byte_count = 0;
    int64_t start_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    
    /* Clear scheduled flag with full memory barrier */
    smp_mb(); /* Full memory barrier before clearing flag */
    qatomic_set(&dma->tx_scheduled, 0);
    
    /* Check for device teardown */
    if (!dma->enabled || dma->state != DMA_DOWNLOADING) {
        dma->state = DMA_IDLE;
        return;
    }
    
    /* Check if TX is blocked */
    if (c->tx_blocked) {
        dma->state = DMA_IDLE;
        return;
    }
    
    /* Process TX descriptors with work limiting */
    while (desc_count < 16 && byte_count < 65536 &&
           (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - start_time) < 1000000) {
        
        /* Check if we have a descriptor to process */
        if (!dma->base_addr) {
            break;
        }
        
        /* Read descriptor */
        hwaddr desc_addr = dma->base_addr + (dma->current_desc * sizeof(EL3DownDesc));
        if (dma_memory_read(dma->as, desc_addr, &dma->down_desc, 
                           sizeof(EL3DownDesc), MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
            dma->state = DMA_ERROR;
            break;
        }
        
        /* Check if descriptor is ready */
        if (!(dma->down_desc.status & DESC_DN_COMPLETE)) {
            break;  /* No more descriptors to process */
        }
        
        /* Process the descriptor - read packet data */
        uint8_t *buffer = g_malloc(dma->down_desc.length);
        if (dma_memory_read(dma->as, dma->down_desc.addr, buffer,
                           dma->down_desc.length, MEMTXATTRS_UNSPECIFIED) == MEMTX_OK) {
            /* Send packet with backpressure handling */
            if (!el3_core_transmit_packet(c, buffer, dma->down_desc.length)) {
                /* Transmission blocked, will be resumed by callback */
                g_free(buffer);
                dma->state = DMA_IDLE;
                return;
            }
            
            byte_count += dma->down_desc.length;
            
            /* Update descriptor status */
            dma->down_desc.status = 0;  /* Clear complete bit */
            dma_memory_write(dma->as, desc_addr, &dma->down_desc,
                            sizeof(EL3DownDesc), MEMTXATTRS_UNSPECIFIED);
        }
        
        g_free(buffer);
        
        /* Move to next descriptor */
        dma->current_desc = (dma->current_desc + 1) % dma->ring_size;
        desc_count++;
    }
    
    /* If we still have work, reschedule */
    if (dma->state == DMA_DOWNLOADING && desc_count > 0) {
        el3_dma_schedule_tx(dma);
    } else {
        dma->state = DMA_IDLE;
    }
}

void el3_dma_rx_bh(void *opaque)
{
    EL3DMAEngine *dma = opaque;
    EL3Core *c = dma->opaque;
    
    /* Clear scheduled flag with full memory barrier */
    smp_mb(); /* Full memory barrier before clearing flag */
    qatomic_set(&dma->rx_scheduled, 0);
    
    /* Check for device teardown */
    if (!dma->enabled || dma->state != DMA_UPLOADING) {
        dma->state = DMA_IDLE;
        return;
    }
    
    /* Check if we have a pending frame to upload */
    if (!dma->pending_frame || dma->pending_len == 0) {
        dma->state = DMA_IDLE;
        return;
    }
    
    /* Check if we have a descriptor to use */
    if (!dma->base_addr) {
        dma->state = DMA_ERROR;
        return;
    }
    
    /* Read upload descriptor */
    hwaddr desc_addr = dma->base_addr + (dma->current_desc * sizeof(EL3UpDesc));
    if (dma_memory_read(dma->as, desc_addr, &dma->up_desc,
                       sizeof(EL3UpDesc), MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        dma->state = DMA_ERROR;
        return;
    }
    
    /* Check if descriptor is available */
    if (dma->up_desc.status & DESC_UP_COMPLETE) {
        /* Descriptor already in use, wait for driver to clear it */
        el3_dma_schedule_rx(dma);  /* Retry later */
        return;
    }
    
    /* Write packet data to buffer */
    size_t copy_len = MIN(dma->pending_len, dma->up_desc.length);
    if (dma_memory_write(dma->as, dma->up_desc.addr, dma->pending_frame,
                        copy_len, MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        dma->state = DMA_ERROR;
        return;
    }
    
    /* Update descriptor with packet info */
    dma->up_desc.status = DESC_UP_COMPLETE | (dma->pending_status & 0xFFFF);
    dma->up_desc.length = copy_len;
    
    /* Write back descriptor */
    dma_memory_write(dma->as, desc_addr, &dma->up_desc,
                    sizeof(EL3UpDesc), MEMTXATTRS_UNSPECIFIED);
    
    /* Free pending frame */
    g_free(dma->pending_frame);
    dma->pending_frame = NULL;
    dma->pending_len = 0;
    dma->pending_status = 0;
    
    /* Move to next descriptor */
    dma->current_desc = (dma->current_desc + 1) % dma->ring_size;
    
    /* Generate interrupt if enabled */
    el3_core_set_status(c, STAT_UP_COMPLETE);
    el3_update_irq(c);
    
    dma->state = DMA_IDLE;
    
    /* Re-check if more work arrived while we were processing */
    if (dma->pending_frame && dma->pending_len > 0) {
        el3_dma_schedule_rx(dma);
    }
}

void el3_dma_change_aio_ctx(EL3DMAEngine *dma, AioContext *ctx)
{
    if (!dma || dma->ctx == ctx) {
        return;
    }
    
    trace_el3_ctx_change("DMA", dma->ctx, ctx);
    
    /* Acquire old context */
    aio_context_acquire(dma->ctx);
    
    /* Cancel and delete existing BHs */
    if (dma->tx_bh) {
        qemu_bh_cancel(dma->tx_bh);
        qemu_bh_delete(dma->tx_bh);
        dma->tx_bh = NULL;
    }
    
    if (dma->rx_bh) {
        qemu_bh_cancel(dma->rx_bh);
        qemu_bh_delete(dma->rx_bh);
        dma->rx_bh = NULL;
    }
    
    /* Release old context */
    aio_context_release(dma->ctx);
    
    /* Acquire new context */
    aio_context_acquire(ctx);
    
    /* Update context reference */
    dma->ctx = ctx;
    
    /* Recreate BHs in new context */
    dma->tx_bh = aio_bh_new(ctx, el3_dma_tx_bh, dma);
    dma->rx_bh = aio_bh_new(ctx, el3_dma_rx_bh, dma);
    
    /* Release new context */
    aio_context_release(ctx);
}

/* MII/PHY implementation functions ported from 86Box 3C515 */

void el3_phy_init(EL3Core *c)
{
    /* Initialize PHY registers with DP83840-compatible values */
    memset(c->phy_regs, 0, sizeof(c->phy_regs));
    
    c->phy_id = 24; /* Standard PHY address for 3C515 */
    c->autoneg_complete = false;
    c->autoneg_enabled = true;
    c->link_up = true;
    c->full_duplex = false;
    c->link_speed = 10;
    
    /* BMCR - Basic Mode Control Register */
    c->phy_regs[MII_BMCR] = BMCR_ANE; /* Auto-negotiation enabled by default */
    
    /* BMSR - Basic Mode Status Register */
    c->phy_regs[MII_BMSR] = BMSR_100TX_FD | BMSR_100TX_HD | BMSR_10T_FD | BMSR_10T_HD |
                            BMSR_AN_ABLE | BMSR_LINK_STAT | BMSR_EXT_CAP;
    
    /* PHY Identifier */
    c->phy_regs[MII_PHYIDR1] = DP83840_PHYID1;
    c->phy_regs[MII_PHYIDR2] = DP83840_PHYID2;
    
    /* Advertisement Register - advertise all capabilities */
    c->phy_regs[MII_ANAR] = ADVERTISE_CSMA | ADVERTISE_10HALF | ADVERTISE_10FULL |
                           ADVERTISE_100HALF | ADVERTISE_100FULL;
    
    /* Link Partner Ability (simulated) */
    c->link_partner_adv = c->phy_regs[MII_ANAR];
    c->phy_regs[MII_ANLPAR] = c->link_partner_adv;
    
    trace_el3_phy_init(c->phy_id, c->phy_regs[MII_PHYIDR1], c->phy_regs[MII_PHYIDR2]);
}

uint16_t el3_phy_read(EL3Core *c, uint8_t reg)
{
    if (reg >= 32) return 0x0000;
    
    uint16_t value = c->phy_regs[reg];
    
    /* Special handling for status registers */
    if (reg == MII_BMSR) {
        /* Update link status */
        if (c->link_up) {
            value |= BMSR_LINK_STAT;
        } else {
            value &= ~BMSR_LINK_STAT;
        }
        
        /* Update autonegotiation complete status */
        if (c->autoneg_complete) {
            value |= BMSR_AN_COMPLETE;
        } else {
            value &= ~BMSR_AN_COMPLETE;
        }
    }
    
    trace_el3_phy_read(reg, value);
    return value;
}

void el3_phy_write(EL3Core *c, uint8_t reg, uint16_t value)
{
    if (reg >= 32) return;
    
    trace_el3_phy_write(reg, value);
    
    if (reg == MII_BMCR) {
        /* Basic Mode Control Register */
        if (value & BMCR_RESET) {
            /* PHY reset */
            el3_phy_init(c);
            trace_el3_phy_reset();
            return;
        }
        
        if (value & BMCR_ANE) {
            c->autoneg_enabled = true;
            if (value & BMCR_RESTART) {
                /* Restart autonegotiation */
                c->autoneg_complete = false;
                c->phy_regs[MII_BMSR] &= ~BMSR_AN_COMPLETE;
                /* Schedule completion - use 200ms for more realistic timing */
                if (c->media_timer) {
                    timer_mod_ns(c->media_timer, 
                                qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 200000000); /* 200ms */
                }
                trace_el3_autoneg_restart();
            }
        } else {
            c->autoneg_enabled = false;
            /* Manual speed/duplex configuration */
            c->link_speed = (value & BMCR_SPEED) ? 100 : 10;
            c->full_duplex = (value & BMCR_DUPLEX) ? true : false;
            trace_el3_manual_config(c->link_speed, c->full_duplex ? "full" : "half");
        }
        
        c->phy_regs[reg] = value & ~(BMCR_RESET | BMCR_RESTART); /* Clear self-clearing bits */
    } else if (reg == MII_ANAR) {
        /* Advertisement Control Register */
        c->phy_regs[reg] = value;
        if (c->autoneg_enabled) {
            /* Restart autonegotiation with new capabilities */
            c->autoneg_complete = false;
            if (c->media_timer) {
                timer_mod_ns(c->media_timer, 
                            qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 200000000); /* 200ms */
            }
        }
    } else {
        c->phy_regs[reg] = value;
    }
}

void el3_autoneg_complete(EL3Core *c)
{
    uint16_t common_caps, our_caps, partner_caps;
    
    our_caps = c->phy_regs[MII_ANAR];
    partner_caps = c->link_partner_adv;
    common_caps = our_caps & partner_caps;
    
    /* Determine best common capability */
    if (common_caps & ADVERTISE_100FULL) {
        c->link_speed = 100;
        c->full_duplex = true;
    } else if (common_caps & ADVERTISE_100HALF) {
        c->link_speed = 100;
        c->full_duplex = false;
    } else if (common_caps & ADVERTISE_10FULL) {
        c->link_speed = 10;
        c->full_duplex = true;
    } else {
        c->link_speed = 10;
        c->full_duplex = false;
    }
    
    c->autoneg_complete = true;
    c->phy_regs[MII_BMSR] |= BMSR_AN_COMPLETE;
    c->phy_regs[MII_ANLPAR] = partner_caps | ADVERTISE_LPACK;
    
    trace_el3_autoneg_complete(c->link_speed, c->full_duplex ? "full" : "half", common_caps);
}

void el3_media_timer_cb(void *opaque)
{
    EL3Core *c = opaque;
    
    /* Handle autonegotiation completion */
    if (c->autoneg_enabled && !c->autoneg_complete) {
        el3_autoneg_complete(c);
    }
}

void el3_mii_command(EL3Core *c, uint16_t cmd)
{
    /* Format: [15:10] = PHY Addr, [9:5] = Reg Addr, [4] = Read/Write, [3:0] = Data */
    uint8_t phy_addr = (cmd >> 10) & 0x1F;
    uint8_t reg_addr = (cmd >> 5) & 0x1F;
    uint8_t is_write = !(cmd & 0x10); /* Bit 4: 0=Write, 1=Read */
    
    if (phy_addr == c->phy_id) {
        if (is_write) {
            /* Write to PHY register - data comes from previous write to MII data reg */
            uint16_t data = c->windows[4][0x0C >> 1]; /* Get data from W4 offset 0x0C */
            el3_phy_write(c, reg_addr, data);
        } else {
            /* Read from PHY register */
            uint16_t data = el3_phy_read(c, reg_addr);
            c->windows[4][0x0C >> 1] = data; /* Store result for reading */
        }
    }
    
    trace_el3_mii_command(is_write ? "write" : "read", phy_addr, reg_addr, 
                         is_write ? c->windows[4][0x0C >> 1] : el3_phy_read(c, reg_addr));
}

uint16_t el3_mii_data_read(EL3Core *c)
{
    /* Return the last MII data read */
    uint16_t data = c->windows[4][0x0C >> 1];
    trace_el3_mii_data_read(data);
    return data;
}

void el3_core_change_aio_ctx(EL3Core *c, AioContext *ctx)
{
    if (!c || c->ctx == ctx) {
        return;
    }
    
    trace_el3_ctx_change("Core", c->ctx, ctx);
    
    /* Acquire old context */
    aio_context_acquire(c->ctx);
    
    /* Cancel and delete existing TX BH */
    if (c->tx_bh) {
        qemu_bh_cancel(c->tx_bh);
        qemu_bh_delete(c->tx_bh);
        c->tx_bh = NULL;
    }
    
    /* Release old context */
    aio_context_release(c->ctx);
    
    /* Acquire new context */
    aio_context_acquire(ctx);
    
    /* Update context reference */
    c->ctx = ctx;
    
    /* Recreate TX BH in new context */
    c->tx_bh = aio_bh_new(ctx, el3_tx_bh_handler, c);
    
    /* Release new context */
    aio_context_release(ctx);
}

/* VMState migration support */

static int el3_core_post_load(void *opaque, int version_id)
{
    EL3Core *c = opaque;
    
    /* Restore IRQ state */
    el3_update_irq(c);
    
    /* Reinitialize PHY if MII capable */
    if (c->caps.has_mii && c->media_timer) {
        /* If autonegotiation was in progress, restart the timer */
        if (c->autoneg_enabled && !c->autoneg_complete) {
            timer_mod_ns(c->media_timer,
                        qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 200000000);
        }
    }
    
    return 0;
}

const VMStateDescription vmstate_el3_dma_engine = {
    .name = "el3_dma_engine",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT64(base_addr, EL3DMAEngine),  /* hwaddr is 64-bit */
        VMSTATE_UINT32(current_desc, EL3DMAEngine),
        VMSTATE_UINT32(ring_size, EL3DMAEngine),
        VMSTATE_UINT32(state, EL3DMAEngine),  /* EL3DMAState is an enum/int */
        VMSTATE_BOOL(enabled, EL3DMAEngine),
        VMSTATE_UINT32(transfer_len, EL3DMAEngine),
        VMSTATE_UINT64(buffer_addr, EL3DMAEngine),  /* hwaddr is 64-bit */
        VMSTATE_UINT32(pending_status, EL3DMAEngine),
        VMSTATE_INT32(tx_scheduled, EL3DMAEngine),
        VMSTATE_INT32(rx_scheduled, EL3DMAEngine),
        VMSTATE_END_OF_LIST()
    }
};

const VMStateDescription vmstate_el3_id_state = {
    .name = "el3_id_state",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(state, EL3IDState),
        VMSTATE_UINT8(unlock_pos, EL3IDState),
        VMSTATE_UINT8(board_tag, EL3IDState),
        VMSTATE_UINT8(selected_tag, EL3IDState),
        VMSTATE_UINT32(product_id, EL3IDState),
        VMSTATE_UINT8(bit_pos, EL3IDState),
        VMSTATE_UINT16(id_port, EL3IDState),
        VMSTATE_UINT16(eeprom_data_register, EL3IDState),
        VMSTATE_UINT8(eeprom_stream_addr, EL3IDState),
        VMSTATE_UINT8(eeprom_stream_bits, EL3IDState),
        VMSTATE_UINT16(eeprom_stream_data, EL3IDState),
        VMSTATE_UINT8(tag_register, EL3IDState),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_rx_packet_desc = {
    .name = "rx_packet_desc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT16(status, RXPacketDesc),
        VMSTATE_UINT16(length, RXPacketDesc),
        VMSTATE_UINT16(bytes_read, RXPacketDesc),
        VMSTATE_UINT16(data_offset, RXPacketDesc),
        VMSTATE_BOOL(complete, RXPacketDesc),
        VMSTATE_END_OF_LIST()
    }
};

const VMStateDescription vmstate_el3_core = {
    .name = "el3_core",
    .version_id = 2,
    .minimum_version_id = 1,
    .post_load = el3_core_post_load,
    .fields = (VMStateField[]) {
        /* Model and capabilities */
        VMSTATE_UINT32(model, EL3Core),
        
        /* Windows and registers */
        VMSTATE_2DARRAY(windows, EL3Core, EL3_MAX_WINDOWS, EL3_WINDOW_SIZE, 1, 
                       vmstate_info_uint16, uint16_t),
        VMSTATE_UINT8(current_window, EL3Core),
        
        /* EEPROM */
        VMSTATE_ARRAY(eeprom, EL3Core, 64, 1, vmstate_info_uint16, uint16_t),
        VMSTATE_UINT32(eeprom_state, EL3Core),
        VMSTATE_UINT8(eeprom_addr, EL3Core),
        VMSTATE_UINT16(eeprom_data, EL3Core),
        VMSTATE_TIMER_PTR(eeprom_timer, EL3Core),
        
        /* Status and interrupts */
        VMSTATE_UINT16(command, EL3Core),
        VMSTATE_UINT16(status, EL3Core),
        VMSTATE_UINT16(int_status, EL3Core),
        VMSTATE_UINT16(int_mask, EL3Core),
        VMSTATE_UINT16(intr_enb, EL3Core),
        VMSTATE_UINT16(status_enb, EL3Core),
        
        /* Enable states */
        VMSTATE_BOOL(rx_enabled, EL3Core),
        VMSTATE_BOOL(tx_enabled, EL3Core),
        
        /* Thresholds and filters */
        VMSTATE_UINT16(tx_avail_thresh, EL3Core),
        VMSTATE_UINT16(tx_start_thresh, EL3Core),
        VMSTATE_UINT16(rx_early_thresh, EL3Core),
        VMSTATE_UINT32(rx_filter, EL3Core),
        
        /* Statistics */
        VMSTATE_BOOL(stats_enabled, EL3Core),
        VMSTATE_BOOL(stats_frozen, EL3Core),
        VMSTATE_UINT8(stats.tx_carrier_errors, EL3Core),
        VMSTATE_UINT8(stats.tx_heartbeat_errors, EL3Core),
        VMSTATE_UINT8(stats.tx_mult_collisions, EL3Core),
        VMSTATE_UINT8(stats.tx_single_collisions, EL3Core),
        VMSTATE_UINT8(stats.tx_late_collisions, EL3Core),
        VMSTATE_UINT8(stats.rx_overruns, EL3Core),
        VMSTATE_UINT8(stats.tx_frames_ok, EL3Core),
        VMSTATE_UINT8(stats.rx_frames_ok, EL3Core),
        VMSTATE_UINT8(stats.tx_deferrals, EL3Core),
        VMSTATE_UINT16(stats.rx_bytes_ok, EL3Core),
        VMSTATE_UINT16(stats.tx_bytes_ok, EL3Core),
        VMSTATE_UINT8(stats.tx_underruns, EL3Core),
        VMSTATE_UINT8(stats.tx_oversize, EL3Core),
        
        /* RX packet queue */
        VMSTATE_STRUCT_ARRAY(rx_packets, EL3Core, RX_MAX_PACKETS, 1,
                            vmstate_rx_packet_desc, RXPacketDesc),
        VMSTATE_UINT8(rx_packet_head, EL3Core),
        VMSTATE_UINT8(rx_packet_tail, EL3Core),
        VMSTATE_UINT8(rx_packet_count, EL3Core),
        
        /* RX data buffer */
        VMSTATE_BUFFER(rx_data, EL3Core),
        VMSTATE_UINT16(rx_data_write_ptr, EL3Core),
        VMSTATE_UINT16(rx_data_used, EL3Core),
        
        /* TX FIFO */
        VMSTATE_BUFFER(tx_fifo, EL3Core),
        VMSTATE_UINT16(tx_fifo_write_ptr, EL3Core),
        VMSTATE_UINT16(tx_fifo_read_ptr, EL3Core),
        VMSTATE_UINT16(tx_fifo_used, EL3Core),
        
        /* TX state */
        VMSTATE_UINT16(current_tx_len, EL3Core),
        VMSTATE_UINT16(current_tx_written, EL3Core),
        VMSTATE_BOOL(tx_in_progress, EL3Core),
        VMSTATE_UINT16(tx_status, EL3Core),
        VMSTATE_BOOL(internal_loopback, EL3Core),
        
        /* TX backpressure state */
        VMSTATE_BOOL(tx_blocked, EL3Core),
        
        /* Multicast hash */
        VMSTATE_BUFFER(mcast_hash, EL3Core),
        
        /* DMA state */
        VMSTATE_UINT32(up_list_ptr, EL3Core),
        VMSTATE_UINT32(down_list_ptr, EL3Core),
        VMSTATE_BOOL(up_stalled, EL3Core),
        VMSTATE_BOOL(down_stalled, EL3Core),
        VMSTATE_BOOL(bus_master_enabled, EL3Core),
        VMSTATE_BOOL(irq_level, EL3Core),
        
        /* ID state */
        VMSTATE_STRUCT(id_state, EL3Core, 1, vmstate_el3_id_state, EL3IDState),
        
        /* MII/PHY state - added in version 2 */
        VMSTATE_UINT16_ARRAY_V(phy_regs, EL3Core, 32, 2),
        VMSTATE_UINT32_V(phy_id, EL3Core, 2),
        VMSTATE_BOOL_V(autoneg_complete, EL3Core, 2),
        VMSTATE_UINT16_V(link_partner_adv, EL3Core, 2),
        VMSTATE_BOOL_V(autoneg_enabled, EL3Core, 2),
        VMSTATE_INT32_V(link_speed, EL3Core, 2),
        VMSTATE_BOOL_V(full_duplex, EL3Core, 2),
        VMSTATE_BOOL_V(link_up, EL3Core, 2),
        VMSTATE_TIMER_PTR_V(media_timer, EL3Core, 2),
        
        VMSTATE_END_OF_LIST()
    }
};
/* Multi-size FIFO stub implementations */
void el3_fifo_init(EL3MultiSizeFifo *mf, uint32_t capacity, uint8_t width_mask)
{
    /* TODO: Implement multi-size FIFO init */
}

void el3_fifo_reset(EL3MultiSizeFifo *mf)
{
    /* TODO: Implement multi-size FIFO reset */
}

void el3_fifo_cleanup(EL3MultiSizeFifo *mf)
{
    /* TODO: Implement multi-size FIFO cleanup */
}

uint32_t el3_fifo_read(EL3MultiSizeFifo *mf, unsigned size)
{
    /* TODO: Implement multi-size FIFO read */
    return 0;
}

void el3_fifo_write(EL3MultiSizeFifo *mf, uint32_t data, unsigned size)
{
    /* TODO: Implement multi-size FIFO write */
}

bool el3_fifo_push_packet(EL3MultiSizeFifo *mf, const uint8_t *data, size_t len)
{
    /* TODO: Implement multi-size FIFO push packet */
    return false;
}

size_t el3_fifo_pop_packet(EL3MultiSizeFifo *mf, uint8_t *buf, size_t max_len)
{
    /* TODO: Implement multi-size FIFO pop packet */
    return 0;
}

uint32_t el3_fifo_used(EL3MultiSizeFifo *mf)
{
    /* TODO: Implement multi-size FIFO used */
    return 0;
}

uint32_t el3_fifo_free(EL3MultiSizeFifo *mf)
{
    /* TODO: Implement multi-size FIFO free */
    return 0;
}

bool el3_fifo_is_empty(EL3MultiSizeFifo *mf)
{
    /* TODO: Implement multi-size FIFO is_empty */
    return true;
}

bool el3_fifo_is_full(EL3MultiSizeFifo *mf)
{
    /* TODO: Implement multi-size FIFO is_full */
    return false;
}