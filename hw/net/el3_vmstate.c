#include "qemu/osdep.h"
#include "hw/net/el3_core.h"
#include "migration/vmstate.h"

/* Feature flags for migration subsections */
#define EL3_FEATURE_FASTPATH    (1 << 0)
#define EL3_FEATURE_INTMOD      (1 << 1)
#define EL3_FEATURE_PM_WOL      (1 << 2)
#define EL3_FEATURE_DMA_V2      (1 << 3)

/* Pre-save handlers */
static void el3_core_pre_save(void *opaque)
{
    EL3Core *c = opaque;
    
    /* Flush any pending TX operations */
    if (c->tx_in_progress) {
        /* Complete current TX to avoid packet loss */
        el3_core_tx_complete(c, TXD_COMPLETE);
    }
    
    /* Snapshot statistics for atomic read */
    c->stats_snapshot.tx_carrier_errors = c->stats.tx_carrier_errors;
    c->stats_snapshot.tx_heartbeat_errors = c->stats.tx_heartbeat_errors;
    c->stats_snapshot.tx_mult_collisions = c->stats.tx_mult_collisions;
    c->stats_snapshot.tx_single_collisions = c->stats.tx_single_collisions;
    c->stats_snapshot.tx_late_collisions = c->stats.tx_late_collisions;
    c->stats_snapshot.rx_overruns = c->stats.rx_overruns;
    c->stats_snapshot.tx_frames_ok = c->stats.tx_frames_ok;
    c->stats_snapshot.rx_frames_ok = c->stats.rx_frames_ok;
    c->stats_snapshot.tx_deferrals = c->stats.tx_deferrals;
    c->stats_snapshot.rx_bytes_ok = c->stats.rx_bytes_ok;
    c->stats_snapshot.tx_bytes_ok = c->stats.tx_bytes_ok;
    c->stats_snapshot.tx_underruns = c->stats.tx_underruns;
    c->stats_snapshot.tx_oversize = c->stats.tx_oversize;
}

/* Post-load handlers */
static int el3_core_post_load(void *opaque, int version_id)
{
    EL3Core *c = opaque;
    
    /* Restore statistics from snapshot */
    c->stats.tx_carrier_errors = c->stats_snapshot.tx_carrier_errors;
    c->stats.tx_heartbeat_errors = c->stats_snapshot.tx_heartbeat_errors;
    c->stats.tx_mult_collisions = c->stats_snapshot.tx_mult_collisions;
    c->stats.tx_single_collisions = c->stats_snapshot.tx_single_collisions;
    c->stats.tx_late_collisions = c->stats_snapshot.tx_late_collisions;
    c->stats.rx_overruns = c->stats_snapshot.rx_overruns;
    c->stats.tx_frames_ok = c->stats_snapshot.tx_frames_ok;
    c->stats.rx_frames_ok = c->stats_snapshot.rx_frames_ok;
    c->stats.tx_deferrals = c->stats_snapshot.tx_deferrals;
    c->stats.rx_bytes_ok = c->stats_snapshot.rx_bytes_ok;
    c->stats.tx_bytes_ok = c->stats_snapshot.tx_bytes_ok;
    c->stats.tx_underruns = c->stats_snapshot.tx_underruns;
    c->stats.tx_oversize = c->stats_snapshot.tx_oversize;
    
    return 0;
}

/* Subsection needed functions */
static bool el3_fastpath_needed(void *opaque)
{
    EL3Core *c = opaque;
    return c->caps.has_bus_master; /* Fast path only on bus master models */
}

static bool el3_intmod_needed(void *opaque)
{
    EL3Core *c = opaque;
    return c->caps.has_bus_master; /* Interrupt moderation only on bus master models */
}

static bool el3_pm_wol_needed(void *opaque)
{
    EL3Core *c = opaque;
    return c->caps.is_100mbit; /* PM/WoL features only on 100MBit models */
}

static bool el3_dma_v2_needed(void *opaque)
{
    EL3Core *c = opaque;
    return c->caps.has_bus_master; /* DMA v2 only on bus master models */
}

/* Core VMState description */
const VMStateDescription vmstate_el3_core = {
    .name = "el3_core",
    .version_id = 3,
    .minimum_version_id = 2,
    .pre_save = el3_core_pre_save,
    .post_load = el3_core_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(current_window, EL3Core),
        VMSTATE_UINT16_2DARRAY(windows, EL3Core, EL3_MAX_WINDOWS, EL3_WINDOW_SIZE),
        VMSTATE_UINT16(command, EL3Core),
        VMSTATE_UINT16(status, EL3Core),
        VMSTATE_UINT16(int_status, EL3Core),
        VMSTATE_UINT16(int_mask, EL3Core),
        VMSTATE_UINT16(intr_enb, EL3Core),
        VMSTATE_UINT16(status_enb, EL3Core),
        VMSTATE_BOOL(rx_enabled, EL3Core),
        VMSTATE_BOOL(tx_enabled, EL3Core),
        VMSTATE_UINT16(tx_avail_thresh, EL3Core),
        VMSTATE_UINT16(tx_start_thresh, EL3Core),
        VMSTATE_UINT16(rx_early_thresh, EL3Core),
        VMSTATE_UINT32(rx_filter, EL3Core),
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
        VMSTATE_UINT16(rx_data_write_ptr, EL3Core),
        VMSTATE_UINT16(rx_data_used, EL3Core),
        VMSTATE_UINT8_ARRAY(rx_data, EL3Core, 4096),
        VMSTATE_UINT8(rx_packet_head, EL3Core),
        VMSTATE_UINT8(rx_packet_tail, EL3Core),
        VMSTATE_UINT8(rx_packet_count, EL3Core),
        VMSTATE_STRUCT_ARRAY(rx_packets, EL3Core, RX_MAX_PACKETS, 0, vmstate_rx_packet_desc, RXPacketDesc),
        VMSTATE_UINT16(tx_fifo_write_ptr, EL3Core),
        VMSTATE_UINT16(tx_fifo_read_ptr, EL3Core),
        VMSTATE_UINT16(tx_fifo_used, EL3Core),
        VMSTATE_UINT8_ARRAY(tx_fifo, EL3Core, 4096),
        VMSTATE_UINT16(current_tx_len, EL3Core),
        VMSTATE_UINT16(current_tx_written, EL3Core),
        VMSTATE_BOOL(tx_in_progress, EL3Core),
        VMSTATE_UINT16(tx_status, EL3Core),
        VMSTATE_UINT8_ARRAY(mcast_hash, EL3Core, 8),
        VMSTATE_UINT16(up_status, EL3Core),
        VMSTATE_UINT16(down_status, EL3Core),
        VMSTATE_UINT32(up_list_ptr, EL3Core),
        VMSTATE_UINT32(down_list_ptr, EL3Core),
        VMSTATE_BOOL(up_stalled, EL3Core),
        VMSTATE_BOOL(down_stalled, EL3Core),
        VMSTATE_BOOL(bus_master_enabled, EL3Core),
        VMSTATE_BOOL(irq_level, EL3Core),
        VMSTATE_UINT16_ARRAY(phy_regs, EL3Core, 32),
        VMSTATE_UINT32(phy_id, EL3Core),
        VMSTATE_BOOL(autoneg_complete, EL3Core),
        VMSTATE_UINT16(link_partner_adv, EL3Core),
        VMSTATE_BOOL(autoneg_enabled, EL3Core),
        VMSTATE_INT32(link_speed, EL3Core),
        VMSTATE_BOOL(full_duplex, EL3Core),
        VMSTATE_BOOL(link_up, EL3Core),
        VMSTATE_TIMER_PTR(media_timer, EL3Core),
        VMSTATE_END_OF_LIST()
    },
    .subsections = (const VMStateDescription*[]) {
        &vmstate_el3_core_fastpath,
        &vmstate_el3_core_intmod,
        &vmstate_el3_core_pm_wol,
        &vmstate_el3_core_dma_v2,
        NULL
    }
};

/* Fast Path Subsection */
const VMStateDescription vmstate_el3_core_fastpath = {
    .name = "el3_core/fastpath",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = el3_fastpath_needed,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_V(batch_tx_count, EL3Core, 1),
        VMSTATE_UINT32_V(batch_rx_count, EL3Core, 1),
        VMSTATE_UINT32_V(current_batch_size, EL3Core, 1),
        VMSTATE_UINT32_V(zero_copy_pool_size, EL3Core, 1),
        VMSTATE_BOOL_V(dma_coalescing_enabled, EL3Core, 1),
        VMSTATE_UINT64_V(performance_metrics, EL3Core, 1),
        VMSTATE_END_OF_LIST()
    }
};

/* Interrupt Moderation Subsection */
const VMStateDescription vmstate_el3_core_intmod = {
    .name = "el3_core/intmod",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = el3_intmod_needed,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_V(intmod_state, EL3Core, 1),
        VMSTATE_UINT64_V(holdoff_timer_ns, EL3Core, 1),
        VMSTATE_UINT32_V(pending_interrupts, EL3Core, 1),
        VMSTATE_UINT32_V(frame_counter, EL3Core, 1),
        VMSTATE_UINT32_V(adaptive_tuning, EL3Core, 1),
        VMSTATE_UINT64_V(rate_calc_timestamp, EL3Core, 1),
        VMSTATE_END_OF_LIST()
    }
};

/* Power Management/WoL Subsection */
const VMStateDescription vmstate_el3_core_pm_wol = {
    .name = "el3_core/pm_wol",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = el3_pm_wol_needed,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8_V(power_state, EL3Core, 1),
        VMSTATE_BOOL_V(pme_enabled, EL3Core, 1),
        VMSTATE_BOOL_V(pme_status, EL3Core, 1),
        VMSTATE_BOOL_V(wol_enabled, EL3Core, 1),
        VMSTATE_UINT32_V(wake_patterns, EL3Core, 1),
        VMSTATE_UINT32_V(wake_masks, EL3Core, 1),
        VMSTATE_BOOL_V(link_wake_enabled, EL3Core, 1),
        VMSTATE_END_OF_LIST()
    }
};

/* DMA Engine Subsection */
const VMStateDescription vmstate_el3_core_dma_v2 = {
    .name = "el3_core/dma_v2",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = el3_dma_v2_needed,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT_V(dma_engine, EL3Core, 1, vmstate_el3_dma_engine, EL3DMAEngine),
        VMSTATE_UINT32_V(dma_budget_bytes, EL3Core, 1),
        VMSTATE_UINT64_V(dma_budget_ns, EL3Core, 1),
        VMSTATE_UINT32_V(dma_rate_bps, EL3Core, 1),
        VMSTATE_TIMER_PTR_V(dma_timer, EL3Core, 1),
        VMSTATE_END_OF_LIST()
    }
};

/* RX Packet Descriptor VMState */
const VMStateDescription vmstate_rx_packet_desc = {
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

/* DMA Engine VMState */
const VMStateDescription vmstate_el3_dma_engine = {
    .name = "el3_dma_engine",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT64(base_addr, EL3DMAEngine),
        VMSTATE_UINT32(current_desc, EL3DMAEngine),
        VMSTATE_UINT32(ring_size, EL3DMAEngine),
        VMSTATE_INT32(state, EL3DMAEngine),
        VMSTATE_BOOL(enabled, EL3DMAEngine),
        VMSTATE_STRUCT(down_desc, EL3DMAEngine, 0, vmstate_el3_down_desc, EL3DownDesc),
        VMSTATE_STRUCT(up_desc, EL3DMAEngine, 0, vmstate_el3_up_desc, EL3UpDesc),
        VMSTATE_UINT32(transfer_len, EL3DMAEngine),
        VMSTATE_UINT64(buffer_addr, EL3DMAEngine),
        VMSTATE_UINT8_ARRAY(pending_frame, EL3DMAEngine, 1536),
        VMSTATE_UINT32(pending_len, EL3DMAEngine),
        VMSTATE_UINT32(pending_status, EL3DMAEngine),
        VMSTATE_END_OF_LIST()
    }
};

/* Download Descriptor VMState */
const VMStateDescription vmstate_el3_down_desc = {
    .name = "el3_down_desc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(next_desc, EL3DownDesc),
        VMSTATE_UINT32(status, EL3DownDesc),
        VMSTATE_UINT32(addr, EL3DownDesc),
        VMSTATE_UINT32(length, EL3DownDesc),
        VMSTATE_END_OF_LIST()
    }
};

/* Upload Descriptor VMState */
const VMStateDescription vmstate_el3_up_desc = {
    .name = "el3_up_desc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(next_desc, EL3UpDesc),
        VMSTATE_UINT32(status, EL3UpDesc),
        VMSTATE_UINT32(addr, EL3UpDesc),
        VMSTATE_UINT32(length, EL3UpDesc),
        VMSTATE_END_OF_LIST()
    }
};

/* EEPROM VMState */
const VMStateDescription vmstate_el3_eeprom = {
    .name = "el3_eeprom",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT16_ARRAY(eeprom, EL3Core, 64),
        VMSTATE_INT32(eeprom_state, EL3Core),
        VMSTATE_UINT8(eeprom_addr, EL3Core),
        VMSTATE_UINT16(eeprom_data, EL3Core),
        VMSTATE_INT64(eeprom_due_ns, EL3Core),
        VMSTATE_UINT64(eeprom_latency_ns, EL3Core),
        VMSTATE_TIMER_PTR(eeprom_timer, EL3Core),
        VMSTATE_END_OF_LIST()
    }
};

/* ISA ID State VMState */
const VMStateDescription vmstate_el3_id_state = {
    .name = "el3_id_state",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_INT32(state, EL3IDState),
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

/* Multi-size FIFO VMState */
const VMStateDescription vmstate_el3_multisize_fifo = {
    .name = "el3_multisize_fifo",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_FIFO8(fifo, EL3MultiSizeFifo),
        VMSTATE_UINT32(capacity, EL3MultiSizeFifo),
        VMSTATE_UINT8(width_mask, EL3MultiSizeFifo),
        VMSTATE_END_OF_LIST()
    }
};