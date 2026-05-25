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
#include "hw/irq.h"
#include "net/net.h"
#include "net/eth.h"
#include "qemu/log.h"
#include "exec/memattrs.h"
#include "qemu/atomic.h"
#include "block/aio.h"
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
 * (Spike/2b: TX path only for now; proper allowLargePackets gating + RX-FIFO growth to follow.) */
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
static bool el3_core_transmit_packet(EL3Core *c, const uint8_t *buf, size_t len)
{
    NetClientState *nc = qemu_get_queue(c->nic);
    ssize_t ret;
    
    if (c->tx_blocked) {
        return false;
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
            
            /* Clear RX complete status if no more packets */
            if (c->rx_packet_count == 0) {
                c->status &= ~STAT_RX_COMPLETE;
                c->int_status &= ~STAT_RX_COMPLETE;
            }
            
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
        /* Acknowledge and clear interrupt bits specified in param */
        c->int_status &= ~(param & 0xFF);
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
        /* RX_COMPLETE: for PIO the ISR re-checks the RX FIFO and RX_DISCARD clears it, so leave it
         * while frames remain (rx_packet_count > 0). For RX-DMA there is no FIFO (rx_packet_count
         * stays 0), so AckIntr(RxComplete) must clear the readable RX_COMPLETE -- else it stays
         * sticky and the next IRQ re-fires forever. */
        if ((param & STAT_RX_COMPLETE) && c->rx_packet_count == 0) {
            c->status &= ~STAT_RX_COMPLETE;
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
    int64_t now, elapsed;
    uint32_t drained;

    if (!c->realtiming || !c->tx_ns_per_byte) {
        return;
    }
    if (c->tx_occupancy == 0) {
        c->tx_drain_last_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        c->tx_wire_active = false;
        return;
    }
    /* Parallel Tasking early-transmit: the card does not begin clocking the FIFO onto the wire
     * until tx_start_thresh bytes are present. Until then occupancy can rise (host fill) without
     * draining; once it crosses the threshold the wire runs and overlaps the remaining fill. */
    if (!c->tx_wire_active) {
        if (c->tx_occupancy < c->tx_start_thresh) {
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
    drained = elapsed / c->tx_ns_per_byte;
    if (drained == 0) {
        return;          /* keep the fractional remainder in tx_drain_last_ns */
    }
    if (drained >= c->tx_occupancy) {
        c->tx_occupancy = 0;
        c->tx_drain_last_ns = now;
    } else {
        c->tx_occupancy -= drained;
        c->tx_drain_last_ns += (int64_t)drained * c->tx_ns_per_byte;
    }
}

/* PIO TX FIFO capacity. Gated on allowLargePackets so normal operation is byte-identical to the
 * 2 KB FIFO, but an FDDI-sized frame (allowLargePackets set) gets a FIFO that can hold it -- the
 * store-and-forward PIO model can't transmit until tx_fifo_used >= the frame length. */
static inline uint16_t el3_tx_fifo_cap(const EL3Core *c)
{
    return (c->windows[3][W3_MAC_CONTROL >> 1] & MAC_CTRL_ALLOW_LARGE)
           ? EL3_LARGE_FRAME_MAX : TX_FIFO_SIZE;
}

/* Get TX FIFO free space. In realtiming mode this reflects the live FIFO occupancy (bytes
 * written minus bytes drained at wire rate) -- so the driver's TxFree guard waits when it
 * fills faster than the link drains, and TxAvailable does not fire spuriously mid-frame. */
uint16_t el3_get_tx_free(EL3Core *c)
{
    uint16_t cap = el3_tx_fifo_cap(c);
    if (c->realtiming && c->tx_ns_per_byte) {
        el3_tx_drain_advance(c);
        return (c->tx_occupancy >= cap) ? 0 : (cap - c->tx_occupancy);
    }
    return cap - c->tx_fifo_used;
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

/* Single-transfer bus-master RX DMA (3C515): when StartDmaUp has armed an up-descriptor at
 * up_list_ptr, DMA a received frame straight into its buffer, write UP_COMPLETE + length back, and
 * raise RxComplete. Single transfer -- disarms after one frame; the driver re-posts + re-arms.
 * Mirror of el3_core_dma_tx_single. Returns true if the frame was consumed via DMA (the caller
 * must then NOT also place it in the PIO RX FIFO). */
static bool el3_core_dma_rx_single(EL3Core *c, const uint8_t *buf, size_t len)
{
    EL3UpDesc d;
    uint32_t cap;
    size_t n;

    if (!c->rx_dma_armed || !c->dma_as || !c->up_list_ptr) {
        return false;
    }
    if (address_space_read(c->dma_as, c->up_list_ptr, MEMTXATTRS_UNSPECIFIED,
                           &d, sizeof(d)) != MEMTX_OK) {
        return false;
    }
    cap = d.length & EL3_DESC_LENGTH_MASK;          /* buffer capacity the driver posted */
    n = len;
    if (cap && n > cap) {
        n = cap;                                    /* truncate to the posted buffer */
    }
    if (address_space_write(c->dma_as, d.addr, MEMTXATTRS_UNSPECIFIED, buf, n) != MEMTX_OK) {
        return false;
    }
    /* status: UpComplete + the received length (13-bit), like the 3c59x UpPktStatus. */
    d.status = EL3_DESC_UP_COMPLETE | ((uint32_t)len & EL3_DESC_LENGTH_MASK);
    address_space_write(c->dma_as, c->up_list_ptr, MEMTXATTRS_UNSPECIFIED, &d, sizeof(d));
    c->rx_dma_armed = false;                        /* single transfer */
    c->stats.rx_frames_ok++;
    c->stats.rx_bytes_ok += len;
    c->status |= STAT_RX_COMPLETE;
    c->int_status |= STAT_RX_COMPLETE;
    if (c->int_mask & STAT_RX_COMPLETE) {
        c->pt.rx_complete_irqs++;
    }
    el3_update_irq(c);
    trace_el3_rx_packet(len, d.status);
    return true;
}

/* Network receive handler */
ssize_t el3_core_receive(NetClientState *nc, const uint8_t *buf, size_t size)
{
    EL3Core *c = qemu_get_nic_opaque(nc);

    /* Check if RX is enabled */
    if (!c->rx_enabled) {
        trace_el3_rx_drop(size, "RX disabled");
        return -1;
    }

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
    if (el3_core_dma_rx_single(c, buf, size)) {
        return size;
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
    
    /* Set RX complete status and interrupt */
    c->status |= STAT_RX_COMPLETE;
    c->int_status |= STAT_RX_COMPLETE;
    if (c->int_mask & STAT_RX_COMPLETE) {
        c->pt.rx_complete_irqs++;
    }
    el3_update_irq(c);
    
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
        uint8_t buf[EL3_LARGE_FRAME_MAX];
        uint16_t cap = el3_tx_fifo_cap(c);
        uint16_t len = c->current_tx_len;

        /* Sanity check length */
        if (len > sizeof(buf)) {
            trace_el3_tx_oversize(len);
            len = sizeof(buf);
        }

        /* Extract frame from TX FIFO */
        for (uint16_t i = 0; i < len; i++) {
            buf[i] = c->tx_fifo[(c->tx_fifo_read_ptr + i) % cap];
        }

        /* Update FIFO pointers */
        c->tx_fifo_read_ptr = (c->tx_fifo_read_ptr + len) % cap;
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
            val = 0;  /* Write-only */
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
    uint8_t buf[EL3_LARGE_FRAME_MAX];
    uint16_t cap = el3_tx_fifo_cap(c);
    uint16_t len = c->current_tx_len;
    if (len == 0 || c->tx_fifo_used < len) {
        return;
    }
    if (len > sizeof(buf)) {
        len = sizeof(buf);
    }
    for (uint16_t i = 0; i < len; i++) {
        buf[i] = c->tx_fifo[(c->tx_fifo_read_ptr + i) % cap];
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
        (void)el3_core_transmit_packet(c, buf, len);
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
             * writes are handled uniformly -- 386+ drivers (e.g. 3Com's 3C5X9PD) stream the
             * FIFO with 32-bit `rep outsd`, so size==4 must feed all four bytes here. */
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
                    } else if (c->tx_fifo_used < el3_tx_fifo_cap(c)) {
                        c->tx_fifo[c->tx_fifo_write_ptr] = b;
                        c->tx_fifo_write_ptr = (c->tx_fifo_write_ptr + 1) % el3_tx_fifo_cap(c);
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