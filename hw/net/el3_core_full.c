#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "hw/net/el3_core.h"
#include "hw/qdev-properties.h"
#include "qemu/log.h"
#include "net/eth.h"
#include "net/net.h"

/* Include trace events */
#include "trace.h"

/* Compatibility macros for trace events */
#define TRACE_EL3_WINDOW_SELECT(w) trace_el3_window_select(w)
#define TRACE_EL3_COMMAND(cmd, param) trace_el3_command(cmd, param)
#define TRACE_EL3_READ(addr, val, size, window) trace_el3_read(addr, val, size, window)
#define TRACE_EL3_WRITE(addr, val, size, window) trace_el3_write(addr, val, size, window)
#define TRACE_EL3_CMD(name, arg) trace_el3_cmd(0, arg)

/* Model capabilities */
static EL3Caps el3_get_caps(EL3Model model)
{
    EL3Caps caps = {0};
    
    switch (model) {
    case MODEL_3C509:
        caps.ram_bytes = 4096;
        caps.tx_fifo_bytes = 2048;
        caps.rx_fifo_bytes = 2048;
        caps.max_windows = 8;
        caps.has_mii = false;
        caps.has_bus_master = false;
        caps.has_vlan_support = false;  /* Early model, no VLAN support */
        break;
        
    case MODEL_3C509B:
        caps.ram_bytes = 8192;
        caps.tx_fifo_bytes = 4096;
        caps.rx_fifo_bytes = 4096;
        caps.max_windows = 8;
        caps.has_mii = false;
        caps.has_bus_master = false;
        caps.has_vlan_support = false;  /* Early model, no VLAN support */
        break;
        
    case MODEL_3C515:
        caps.ram_bytes = 8192;
        caps.tx_fifo_bytes = 4096;
        caps.rx_fifo_bytes = 4096;
        caps.max_windows = 8;
        caps.has_mii = true;
        caps.has_bus_master = true;
        caps.is_100mbit = true;
        caps.has_vlan_support = false;  /* Early ISA model, no VLAN */
        break;
        
    case MODEL_3C905:
        caps.ram_bytes = 32768;
        caps.tx_fifo_bytes = 0;  /* Uses DMA */
        caps.rx_fifo_bytes = 0;  /* Uses DMA */
        caps.max_windows = 8;  /* Windows 0-7, same as others */
        caps.has_mii = true;
        caps.has_bus_master = true;
        caps.is_100mbit = true;
        caps.has_full_duplex = true;
        caps.has_vlan_support = false;  /* 3C905 did not support VLAN */
        break;
        
    case MODEL_3C905B:
        caps.ram_bytes = 32768;
        caps.tx_fifo_bytes = 0;  /* Uses DMA */
        caps.rx_fifo_bytes = 0;  /* Uses DMA */
        caps.max_windows = 8;  /* Windows 0-7, same as others */
        caps.has_mii = true;
        caps.has_bus_master = true;
        caps.is_100mbit = true;
        caps.has_full_duplex = true;
        caps.has_vlan_support = true;  /* 3C905B (Cyclone) supports VLAN frames */
        break;
        
    default:
        caps.max_windows = 8;
        break;
    }
    
    return caps;
}

/* Command completion timer */
static void el3_cmd_complete(void *opaque)
{
    EL3Core *c = opaque;
    c->status &= ~STAT_CMD_IN_PROG;
}

/* Window selection */
void el3_select_window(EL3Core *c, uint8_t window)
{
    if (window >= c->caps.max_windows) {
        qemu_log_mask(LOG_GUEST_ERROR, "el3: invalid window %d (max %d)\n",
                      window, c->caps.max_windows - 1);
        return;
    }
    c->current_window = window;
    TRACE_EL3_WINDOW_SELECT(window);
}

/* Command processing */

/* Core initialization */
void el3_core_init(EL3Core *c, EL3Model model, const EL3VariantOps *ops)
{
    memset(c, 0, sizeof(*c));
    c->ops = ops;
    c->model = model;
    c->caps = el3_get_caps(model);
    
    /* Note: FIFO allocation is now handled by variants */
    
    /* Create timers */
    c->cmd_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, el3_cmd_complete, c);
    c->eeprom_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, el3_eeprom_timer_cb, c);
    
    /* Create TX bottom half */
    c->tx_bh = qemu_bh_new(el3_tx_bh, c);
    
    /* Initialize EEPROM state */
    c->eeprom_state = EEPROM_IDLE;
    c->eeprom_latency_ns = 162000; /* 162 microseconds */
    
    /* Initialize EEPROM based on model */
    if (model == MODEL_3C509 || model == MODEL_3C509B || model == MODEL_3C515) {
        el3_eeprom_init_3c509(c);
        /* Initialize ID sequence for ISA cards */
        if (model == MODEL_3C509 || model == MODEL_3C509B) {
            el3_id_sequence_init(c);
        }
    } else {
        el3_eeprom_init_3c59x(c);
    }
    
    /* Initialize Window 3 internal config */
    uint16_t ram_size_bits = 0;
    if (c->caps.ram_bytes == 8192) {
        ram_size_bits = 0x4000;
    } else if (c->caps.ram_bytes >= 16384) {
        ram_size_bits = 0x8000;
    }
    c->windows[3][W3_INTERNAL_CONFIG >> 1] = ram_size_bits;
}

/* Core reset - follows hardware initialization sequence */
void el3_core_reset(EL3Core *c)
{
    /* Phase 1: Hardware Reset */
    /* Reset to window 0 */
    c->current_window = 0;
    
    /* Reset FIFOs */
    el3_reset_fifos(c);
    
    /* Clear status but preserve CMD_IN_PROG if set */
    c->status &= STAT_CMD_IN_PROG;  /* Keep only CMD_IN_PROG bit */
    c->int_status = 0;
    c->command = 0;
    
    /* Reset interrupt and status enable masks */
    c->intr_enb = 0;
    c->status_enb = 0;
    
    /* Reset enable states */
    c->rx_enabled = false;
    c->tx_enabled = false;
    
    /* Reset TX state */
    c->tx_status = 0;
    c->tx_in_progress = false;
    c->current_tx_len = 0;
    c->current_tx_written = 0;
    c->tx_avail_thresh = 0;  /* Default TX available threshold */
    c->internal_loopback = false;
    if (c->tx_bh) {
        qemu_bh_cancel(c->tx_bh);
    }
    
    /* Reset thresholds */
    c->tx_avail_thresh = 0;
    c->tx_start_thresh = 0;
    c->rx_early_thresh = 0;
    
    /* Reset EEPROM state */
    if (c->eeprom_timer) {
        timer_del(c->eeprom_timer);
    }
    c->eeprom_state = EEPROM_IDLE;
    c->eeprom_data = 0;
    c->eeprom_due_ns = 0;
    
    /* Clear windows but preserve EEPROM */
    memset(c->windows, 0, sizeof(c->windows));
    
    /* Phase 2: MAC Address Configuration */
    /* Copy MAC address from EEPROM to Window 2 */
    /* MAC is in EEPROM addresses 0x00-0x02 */
    c->windows[2][0] = c->eeprom[0x00];  /* MAC bytes 0-1 */
    c->windows[2][1] = c->eeprom[0x01];  /* MAC bytes 2-3 */
    c->windows[2][2] = c->eeprom[0x02];  /* MAC bytes 4-5 */
    
    /* Phase 3: Configuration Setup */
    /* Initialize internal config in Window 3 */
    uint16_t ram_size_bits = 0;
    if (c->caps.ram_bytes == 8192) {
        ram_size_bits = 0x4000;
    } else if (c->caps.ram_bytes >= 16384) {
        ram_size_bits = 0x8000;
    }
    
    /* Set default transceiver type based on model */
    uint32_t config = ram_size_bits;
    if (c->model == MODEL_3C509 || c->model == MODEL_3C509B) {
        /* 10BaseT for 3C509 */
        config |= (0x02 << 20);  /* 10BaseT transceiver */
    } else if (c->model == MODEL_3C515) {
        /* 100BaseTx for 3C515 */
        config |= (0x04 << 20);  /* 100BaseTx transceiver */
    }
    
    c->windows[3][W3_INTERNAL_CONFIG >> 1] = config & 0xFFFF;
    c->windows[3][(W3_INTERNAL_CONFIG >> 1) + 1] = (config >> 16) & 0xFFFF;
    
    /* Phase 4: Media Configuration */
    /* Initialize Window 4 media status */
    if (c->model == MODEL_3C509 || c->model == MODEL_3C509B) {
        /* 10BaseT with link beat enabled */
        c->windows[4][W4_MEDIA_STATUS >> 1] = 0x00C0;  /* Link beat enabled */
    } else if (c->model == MODEL_3C515) {
        /* 100BaseTx with auto-negotiation */
        c->windows[4][W4_MEDIA_STATUS >> 1] = 0x8080;  /* Auto-neg + link */
    }
    
    /* Set diagnostic registers to indicate healthy state */
    c->windows[4][W4_FIFO_DIAG >> 1] = 0x0080;  /* FIFO_OK */
    c->windows[4][W4_NET_DIAG >> 1] = 0x1F80;   /* All tests OK */
    
    /* Phase 5: Operating Parameters */
    /* Set reasonable defaults for operation */
    c->windows[1][W1_TX_STATUS >> 1] = 0x0001;  /* TX complete */
    c->windows[1][W1_TX_FREE >> 1] = c->caps.tx_fifo_bytes;  /* TX FIFO empty */
    
    /* Initialize statistics to zero (Window 6) */
    memset(&c->stats, 0, sizeof(c->stats));
    
    /* Set default RX filter: individual + broadcast */
    c->rx_filter = RXF_IND_ADDR | RXF_BROADCAST;
    
    /* Initialize FIFO state */
    el3_reset_fifos(c);
    
    /* Clear multicast hash table */
    memset(c->mcast_hash, 0, sizeof(c->mcast_hash));
    
    /* Clear DMA status registers */
    c->up_status = 0;
    c->down_status = 0;
    
    /* Clear interrupt state bits */
    c->status &= ~(STAT_INT_LATCH | STAT_INT_REQ);
    
    /* Deassert IRQ line */
    if (c->ops && c->ops->irq_set) {
        c->ops->irq_set(c, false);
    }
    
    /* Simulate 2ms hardware reset delay */
    if (c->cmd_timer) {
        timer_mod_ns(c->cmd_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 2000000);
    }
}

/* Register read */
uint32_t el3_core_read(EL3Core *c, hwaddr addr, unsigned size)
{
    /* Legacy compatibility wrapper that handles global registers and maps to window system */
    
    if (addr < 0x0E) {
        /* Global registers (accessible in all windows) */
        uint32_t val = 0;
        switch (addr & ~1) {
        case 0x00:  /* Manufacturer ID low */
            val = 0xB750;  /* 3Com ID low */
            break;
        case 0x02:  /* Manufacturer ID high / Product ID */
            if (c->model == MODEL_3C509 || c->model == MODEL_3C509B) {
                val = 0x5090;  /* Product ID for 3C509 */
            } else if (c->model == MODEL_3C515) {
                val = 0x5157;  /* Product ID for 3C515 */
            } else {
                val = 0x9055;  /* Product ID for 3C59x */
            }
            break;
        case 0x04:  /* Configuration control */
            val = el3_core_register_read(c, 3, 4, 2);  /* Mirror from Window 3 */
            break;
        case 0x06:  /* Address configuration */
            val = el3_core_register_read(c, 0, 6, 2);  /* Mirror from Window 0 */
            break;
        case 0x08:  /* Resource configuration */
            val = el3_core_register_read(c, 0, 8, 2);  /* Mirror from Window 0 */
            break;
        case 0x0A:  /* Window-specific register */
            return el3_core_register_read(c, c->current_window, 0x0A, size);
        case 0x0C:  /* RX Status or Command (alt location) */
            val = el3_core_register_read(c, 1, 0x08, 2);  /* Mirror from Window 1 */
            break;
        }
        
        if (size == 1) {
            val = (addr & 1) ? (val >> 8) : (val & 0xFF);
        }
        TRACE_EL3_READ(addr, val, size, c->current_window);
        return val;
    } else if (addr == 0x0E) {
        /* Status register */
        TRACE_EL3_READ(addr, c->status, size, c->current_window);
        return c->status;
    } else {
        /* Window registers - map to new dispatch system */
        int offset = addr - 0x10;
        return el3_core_register_read(c, c->current_window, offset, size);
    }
}

/* Register write */
void el3_core_write(EL3Core *c, hwaddr addr, uint32_t val, unsigned size)
{
    TRACE_EL3_WRITE(addr, val, size, c->current_window);
    
    if (addr < 0x0E) {
        /* Global registers (accessible in all windows) */
        switch (addr & ~1) {
        case 0x00:  /* TX Data register (variant-specific) */
            el3_core_register_write(c, c->current_window, 0x00, val, size);
            break;
        case 0x04:  /* Configuration control */
            el3_core_register_write(c, 3, 4, val, 2);  /* Mirror to Window 3 */
            break;
        case 0x06:  /* Address configuration */
            el3_core_register_write(c, 0, 6, val, 2);  /* Mirror to Window 0 */
            break;
        case 0x08:  /* Resource configuration */
            el3_core_register_write(c, 0, 8, val, 2);  /* Mirror to Window 0 */
            break;
        case 0x0C:  /* Command register (alternate location) */
            el3_core_process_command(c, val);
            return;
        }
        return;
    }
    
    if (addr == 0x0E) {
        /* Command/Status register
         * Writes go to command register
         * Some status bits support write-1-to-clear
         */
        if (val & 0xF800) {
            /* Upper bits indicate command */
            el3_core_process_command(c, val);
        } else {
            /* Lower bits might be status clear - write-1-to-clear semantics */
            /* Clear interrupt latch and specified status bits */
            c->status &= ~(val & 0x00FF);  /* Clear lower 8 status bits */
            if (val & STAT_INT_LATCH) {
                c->int_status = 0;  /* Clear all interrupt sources */
            }
            el3_update_irq(c);
        }
        return;
    }
    
    if (addr >= 0x10 && addr < 0x20) {
        /* Window registers - map to new dispatch system */
        int offset = addr - 0x10;
        el3_core_register_write(c, c->current_window, offset, val, size);
    }
}

/* EEPROM timing callback */
static void el3_eeprom_timer_cb(void *opaque)
{
    EL3Core *c = opaque;
    
    /* Complete the asynchronous read */
    c->eeprom_data = c->eeprom[c->eeprom_addr & 0x3F];
    c->eeprom_state = EEPROM_IDLE;
    
    /* Optional: Set completion status bit if hardware has one */
    /* c->status |= STAT_EEPROM_DONE; */
    /* el3_update_irq(c); */
    
    trace_el3_eeprom_done(c->eeprom_addr, c->eeprom_data);
}

/* EEPROM command */
void el3_eeprom_cmd(EL3Core *c, uint16_t cmd)
{
    if (cmd & 0x8000) {  /* Read command */
        /* Ignore command if EEPROM is busy - hardware behavior */
        if (c->eeprom_state == EEPROM_BUSY) {
            trace_el3_eeprom_cmd_ignored(cmd, "EEPROM busy");
            return;
        }
        
        /* Validate and mask address to 6 bits (0x00-0x3F) */
        c->eeprom_addr = (cmd >> 6) & 0x3F;
        c->eeprom_state = EEPROM_BUSY;
        c->eeprom_due_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + c->eeprom_latency_ns;
        
        if (!c->eeprom_timer) {
            c->eeprom_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                          el3_eeprom_timer_cb, c);
        }
        
        timer_mod_ns(c->eeprom_timer, c->eeprom_due_ns);
        trace_el3_eeprom_start(c->eeprom_addr);
    } else {
        /* Invalid opcode - ignore non-read commands */
        trace_el3_eeprom_cmd_ignored(cmd, "invalid opcode");
    }
}

/* EEPROM read */
uint16_t el3_eeprom_read(EL3Core *c)
{
    if (c->eeprom_state == EEPROM_BUSY) {
        return 0x8000;  /* Busy bit */
    }
    return c->eeprom_data;
}

/* Initialize EEPROM for 3C509 */
void el3_eeprom_init_3c509(EL3Core *c)
{
    memset(c->eeprom, 0, sizeof(c->eeprom));
    
    /* MAC at 0x00-0x02 (Node Address) */
    c->eeprom[0x00] = (c->conf.macaddr.a[1] << 8) | c->conf.macaddr.a[0];
    c->eeprom[0x01] = (c->conf.macaddr.a[3] << 8) | c->conf.macaddr.a[2];
    c->eeprom[0x02] = (c->conf.macaddr.a[5] << 8) | c->conf.macaddr.a[4];
    
    /* Product ID at 0x03 - Critical for detection! */
    c->eeprom[0x03] = 0x6D50;  /* 3Com ID for 3C509B */
    
    /* Configuration registers */
    c->eeprom[0x04] = 0x0000;  /* Configuration control */
    c->eeprom[0x06] = 0x0300;  /* Address configuration (I/O base) */
    c->eeprom[0x08] = 0x000A;  /* Resource configuration (IRQ 10) */
    
    /* Manufacturing date (optional) */
    c->eeprom[0x08] = 0x1234;  /* Manufacturing date */
    
    /* OEM Node Address (usually zeros) */
    c->eeprom[0x0A] = 0x0000;
    c->eeprom[0x0B] = 0x0000;
    c->eeprom[0x0C] = 0x0000;
    
    /* Software configuration */
    c->eeprom[0x0D] = 0x0000;
    
    /* Calculate checksum */
    uint16_t checksum = 0;
    for (int i = 0; i < 0x0F; i++) {
        checksum ^= c->eeprom[i];
    }
    c->eeprom[0x0F] = checksum;
}

/* Initialize EEPROM for 3C59x */
void el3_eeprom_init_3c59x(EL3Core *c)
{
    memset(c->eeprom, 0, sizeof(c->eeprom));
    
    /* Model info */
    c->eeprom[0x00] = 0x9055;  /* 3C905B */
    
    /* PCI subsystem */
    c->eeprom[0x07] = 0x10B7;  /* 3Com vendor */
    c->eeprom[0x08] = 0x9055;  /* Subsystem ID */
    
    /* MAC at 0x10-0x12 - DIFFERENT! */
    c->eeprom[0x10] = (c->conf.macaddr.a[1] << 8) | c->conf.macaddr.a[0];
    c->eeprom[0x11] = (c->conf.macaddr.a[3] << 8) | c->conf.macaddr.a[2];
    c->eeprom[0x12] = (c->conf.macaddr.a[5] << 8) | c->conf.macaddr.a[4];
    
    /* Capabilities */
    c->eeprom[0x13] = 0x0142;
    
    /* Calculate checksum */
    uint16_t checksum = 0;
    for (int i = 0; i < 0x0F; i++) {
        checksum ^= c->eeprom[i];
    }
    c->eeprom[0x0F] = checksum;
}

/* Helper functions */
uint16_t el3_get_tx_free(EL3Core *c)
{
    if (c->caps.tx_fifo_bytes == 0) {
        return 0;  /* DMA mode */
    }
    return c->caps.tx_fifo_bytes - c->tx_used;
}

uint16_t el3_get_rx_free(EL3Core *c)
{
    if (c->caps.rx_fifo_bytes == 0) {
        return 0;  /* DMA mode */
    }
    return c->caps.rx_fifo_bytes - c->rx_used;
}

/* Update interrupts via variant ops */
void el3_update_irq(EL3Core *c)
{
    /* Use separate latched interrupt events for IRQ generation */
    bool level = (c->int_status & c->intr_enb) != 0;
    
    /* Update INT_LATCH and INT_REQ bits in status register */
    if (c->int_status != 0) {
        c->status |= STAT_INT_LATCH;
        if (level) {
            c->status |= STAT_INT_REQ;
        } else {
            c->status &= ~STAT_INT_REQ;
        }
    } else {
        c->status &= ~(STAT_INT_LATCH | STAT_INT_REQ);
    }
    
    if (c->ops && c->ops->irq_set) {
        c->ops->irq_set(c, level);
    }
    
    trace_el3_irq_update(level, c->status, c->intr_enb);
}

/* Update TX available status and generate events */
void el3_update_tx_available(EL3Core *c)
{
    bool was_available = (c->status & STAT_TX_AVAILABLE) != 0;
    bool is_available = (c->tx_avail_thresh > 0) && 
                        (el3_tx_fifo_free(c) >= c->tx_avail_thresh);
    
    /* Update live status bit */
    if (is_available) {
        c->status |= STAT_TX_AVAILABLE;
    } else {
        c->status &= ~STAT_TX_AVAILABLE;
    }
    
    /* Generate latched event on false->true transition */
    if (!was_available && is_available) {
        c->int_status |= STAT_TX_AVAILABLE;
        el3_update_irq(c);
    }
}

/* Packet receive */

/*
 * VALIDATION TESTS for packet filtering:
 * 
 * 1. Multicast Hash Test:
 *    - Set multicast MAC: 01:00:5e:00:00:01
 *    - Compute CRC32-LE: should match Linux ether_crc_le()
 *    - Verify hash bit = (crc >> 26) & 0x3f matches driver
 *    - Test with ethtool -K eth0 allmulti on/off
 * 
 * 2. Station Address Filter Test:
 *    - Clear FILTER_STATION bit (0x0001)
 *    - Send unicast packet to device MAC
 *    - Verify packet is dropped
 *    - Set FILTER_STATION bit, verify packet accepted
 * 
 * 3. Frame Size Tests:
 *    - Send 59-byte frame (< 60): should set RUNT error
 *    - Send 60-byte frame: should be accepted
 *    - Send 1514-byte frame: should be accepted
 *    - Send 1515-byte frame (> 1514): should set OVERSIZE error
 *    - Test VLAN-tagged 1518-byte wire frame (1514 w/o FCS)
 * 
 * 4. FIFO Overrun Test:
 *    - Fill RX FIFO with frames
 *    - Send additional frame to trigger overrun
 *    - Verify STAT_ADAPTER_FAIL (0x0002) set for 3C509B
 *    - Verify driver recovery via RX_RESET command
 * 
 * 5. DMA Interrupt Test (3C59x only):
 *    - Complete DMA upload (RX)
 *    - Verify STAT_UP_COMPLETE (0x0400) set
 *    - Verify STAT_DMA_DONE (0x0100) also set
 *    - Check driver handles both bits correctly
 */

/* Packet filtering helper */
static bool el3_packet_filter(EL3Core *c, const uint8_t *buf, size_t size)
{
    static const uint8_t broadcast[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    uint16_t rx_filter = c->rx_filter;
    
    /* 
     * Filter evaluation order (per hardware spec):
     * 1. Promiscuous mode bypasses all other checks
     * 2. Broadcast checked only if broadcast bit set
     * 3. Multicast checked only if multicast bit set
     * 4. Unicast checked only if station bit set
     */
    
    /* Promiscuous mode - accept all */
    if (rx_filter & RX_FILTER_PROMISCUOUS) {
        return true;
    }
    
    /* Check for broadcast packet */
    if (!memcmp(buf, broadcast, 6)) {
        /* Accept only if broadcast filter enabled */
        return (rx_filter & RX_FILTER_BROADCAST) != 0;
    }
    
    /* Check for multicast packet (but not broadcast) */
    if (buf[0] & 0x01) {
        /* Check for all-multicast mode (shortcut for now) */
        if (rx_filter & RX_FILTER_ALLMULTI) {
            return true;  /* Accept all multicast */
        }
        
        /* Reject if multicast filter disabled */
        if (!(rx_filter & RX_FILTER_MULTICAST)) {
            return false;
        }
        
        /* Apply multicast hash filter (64-bit hash table) */
        /* Use QEMU's LE CRC32 which matches Linux ether_crc_le() */
        uint32_t crc = net_crc32_le(buf, ETH_ALEN);
        int bit = (crc >> 26) & 0x3f;  /* 6-bit hash */
        int byte = bit / 8;
        int mask = 1 << (bit % 8);
        
        /* Check hash bit in raw byte array */
        return (c->mcast_hash[byte] & mask) != 0;
    }
    
    /* Unicast packet - check station address filter */
    if (!(rx_filter & RX_FILTER_INDIVIDUAL)) {
        return false;
    }
    
    /* Compare against station MAC address */
    /* MAC is stored in Window 2, offsets 0x00-0x05 */
    return (buf[0] == (c->windows[2][0] & 0xff) &&
            buf[1] == (c->windows[2][0] >> 8) &&
            buf[2] == (c->windows[2][1] & 0xff) &&
            buf[3] == (c->windows[2][1] >> 8) &&
            buf[4] == (c->windows[2][2] & 0xff) &&
            buf[5] == (c->windows[2][2] >> 8));
}

/* Update RX statistics */
static void el3_update_rx_stats(EL3Core *c, uint16_t rx_status)
{
    if (rx_status & RX_STATUS_ERROR) {
        uint16_t error = rx_status & RX_STATUS_ERR_MASK;
        switch (error) {
        case 0x0000:  /* RX_ERR_OVERRUN - special case, no bits set */
            /* Overrun is handled via interrupt bit, not in RX status */
            if (c->stats.rx_overruns < 0xFF) {
                c->stats.rx_overruns++;
            }
            break;
        case 0x2800:  /* RX_ERR_CRC */
            /* CRC errors not in basic stats */
            break;
        case 0x1000:  /* RX_ERR_ALIGNMENT */
        case 0x2000:  /* RX_ERR_DRIBBLE */
            /* Frame errors not in basic stats */
            break;
        case 0x1800:  /* RX_ERR_RUNT */
        case 0x0800:  /* RX_ERR_OVERSIZE */
            /* Length errors not in basic stats */
            break;
        }
    } else {
        if (c->stats.rx_frames_ok < 0xFF) {
            c->stats.rx_frames_ok++;
        }
        uint16_t len = rx_status & RX_STATUS_LENGTH;
        uint32_t new_bytes = c->stats.rx_bytes_ok + len;
        if (new_bytes < 0xFFFF) {
            c->stats.rx_bytes_ok = new_bytes;
        } else {
            c->stats.rx_bytes_ok = 0xFFFF;
        }
    }
    
    /* Check for statistics overflow */
    if (c->stats.rx_frames_ok == 0xFF || c->stats.rx_overruns == 0xFF) {
        c->status |= STAT_STATS_FULL;
        if (c->status_enb & STAT_STATS_FULL) {
            el3_update_irq(c);
        }
    }
}

ssize_t el3_core_receive(NetClientState *nc, const uint8_t *buf, size_t size)
{
    EL3Core *c = qemu_get_nic_opaque(nc);
    uint16_t rx_status = 0;
    
    /*
     * Model-specific behavior:
     * - 3C509B: PIO with 4KB FIFO, uses STAT_ADAPTER_FAIL for overrun
     * - 3C59x: Bus-master DMA, uses UP_COMPLETE/DMA_DONE interrupts
     * - 3C515: ISA bus-master with pacing constraints
     */"
    
    /* Check if RX is enabled */
    if (!c->rx_enabled) {
        return -1;  /* Drop packet if RX disabled */
    }
    
    /* Apply packet filters */
    if (!el3_packet_filter(c, buf, size)) {
        return size;  /* Silently drop filtered packets */
    }
    
    /* Build RX status word */
    rx_status = size & RX_STATUS_LENGTH;  /* Packet length in bits 10-0 */
    
    /* Check for errors - wire frame includes 4-byte FCS not in QEMU */
    /* So actual wire limits are 64 and 1518, but QEMU gives us 60 and 1514 */
    bool has_error = false;
    if (size < 60) {  /* Wire frame would be < 64 bytes */
        rx_status |= RX_STATUS_ERROR | RX_ERR_RUNT;
        has_error = true;
    } else if (size > 1514) {  /* Wire frame would be > 1518 bytes */
        rx_status |= RX_STATUS_ERROR | RX_ERR_OVERSIZE;
        has_error = true;
    }
    
    /* If frame has errors and accept-error-frames filter is not set, drop it */
    if (has_error && !(c->rx_filter & RX_FILTER_ACCEPT_ERROR)) {
        /* Update error statistics */
        if ((rx_status & 0x3800) == RX_ERR_RUNT || 
            (rx_status & 0x3800) == RX_ERR_OVERSIZE) {
            /* Length errors */
            if (c->stats.rx_frames_ok < 0xFF) {
                /* Note: We don't increment rx_frames_ok for errors */
            }
        }
        return size;  /* Silently drop error frames unless filter is set */
    }
    
    /* Try to place frame via variant operations */
    if (c->ops && c->ops->rx_place_frame) {
        /* Check if variant has space */
        if (c->ops->rx_has_space && !c->ops->rx_has_space(c, size)) {
            /* No space - handle overflow */
            if (c->stats.rx_overruns < 0xFF) {
                c->stats.rx_overruns++;
            } else {
                /* Counter saturated - signal STATS_FULL */
                c->status |= STAT_STATS_FULL;
                if (c->status_enb & STAT_STATS_FULL) {
                    el3_update_irq(c);
                }
            }
            
            /* Overrun is a resource condition, not gated by accept-error.
             * Do not synthesize zero-length packets for overrun - the 
             * hardware signals this via statistics and STATS_FULL interrupt.
             */
            return size;  /* Accept packet at wire level but drop from FIFO */
        }
        
        /* Let variant handle frame placement */
        ssize_t result = c->ops->rx_place_frame(c, buf, size, rx_status, size);
        if (result < 0) {
            /* Variant rejected frame */
            return size;  /* Accept at wire level */
        }
    } else {
        /* No variant ops - for basic 3C509B, add to packet queue */
        if (!el3_rx_can_receive(c, size)) {
            /* No space for incoming packet - drop it */
            if (c->stats.rx_overruns < 0xFF) {
                c->stats.rx_overruns++;
            } else {
                /* Counter saturated - signal STATS_FULL */
                c->status |= STAT_STATS_FULL;
                if (c->status_enb & STAT_STATS_FULL) {
                    el3_update_irq(c);
                }
            }
            
            /* Overrun is a resource condition, not gated by accept-error.
             * Do not synthesize zero-length packets for overrun - the 
             * hardware signals this via statistics and STATS_FULL interrupt.
             */
            return size;  /* Accept packet at wire level but drop from FIFO */
        }
        
        /* Add packet to queue */
        if (!el3_rx_add_packet(c, buf, size, rx_status)) {
            /* Should not happen as we checked space above */
            return -1;
        }
        
        /* Update Window 1 RX status to show packet available */
        if (c->rx_packet_count > 0) {
            c->windows[1][W1_RX_STATUS >> 1] = c->rx_packets[c->rx_packet_head].status;
        }
    }
    
    /* Update status and generate interrupt */
    c->status |= STAT_RX_COMPLETE;
    
    /* Update statistics */
    el3_update_rx_stats(c, rx_status);
    
    /* Generate interrupt if enabled */
    if (c->status_enb & STAT_RX_COMPLETE) {
        el3_update_irq(c);
    }
    
    /* Kick variant if needed */
    if (c->ops && c->ops->kick) {
        c->ops->kick(c, EL3_KICK_RX);
    }
    
    trace_el3_rx_packet(size, rx_status);
    
    /* Return size to indicate packet consumed */
    return size;
}

/* Link status */
void el3_core_set_link_status(NetClientState *nc)
{
    EL3Core *c = qemu_get_nic_opaque(nc);
    bool link_up = nc->link_down ? false : true;
    
    trace_el3_link_status(link_up ? "up" : "down");
    
    /* Update NET_DIAG register (Window 4, offset 0x06) */
    if (link_up) {
        /* Set link beat detect for 10BaseT */
        c->windows[4][W4_NET_DIAG >> 1] |= NET_DIAG_LINK_BEAT;
        
        /* Set other diagnostic bits to indicate healthy state */
        c->windows[4][W4_NET_DIAG >> 1] |= NET_DIAG_TX_OK | NET_DIAG_RX_OK | 
                                            NET_DIAG_FIFO_OK;
    } else {
        /* Clear link beat */
        c->windows[4][W4_NET_DIAG >> 1] &= ~NET_DIAG_LINK_BEAT;
    }
    
    /* Some drivers expect link status changes to generate interrupts */
    /* But 3C509B doesn't have a specific link change interrupt */
    /* The adapter failure interrupt is sometimes used for major issues */
}

/* FIFO management functions */

/* Reset FIFO state */
static void el3_reset_fifos(EL3Core *c)
{
    /* Reset RX packet queue */
    c->rx_data_write_ptr = 0;
    c->rx_data_used = 0;
    c->rx_packet_head = 0;
    c->rx_packet_tail = 0;
    c->rx_packet_count = 0;
    memset(c->rx_packets, 0, sizeof(c->rx_packets));
    
    /* Reset TX FIFO */
    c->tx_fifo_write_ptr = 0;
    c->tx_fifo_read_ptr = 0;
    c->tx_fifo_used = 0;
}

/* RX packet descriptor management */
static bool el3_rx_can_receive(EL3Core *c, size_t len)
{
    /* Check descriptor availability first */
    if (c->rx_packet_count >= RX_MAX_PACKETS) {
        return false;  /* No descriptor slots available */
    }
    
    /* Check if we have enough data buffer space (with padding) */
    size_t needed = len;
    if (len & 1) {
        needed++;  /* Account for word alignment padding */
    }
    
    return (c->rx_data_used + needed) <= 4096;
}

/* Add packet to RX queue */
static bool el3_rx_add_packet(EL3Core *c, const uint8_t *data, size_t len, 
                              uint16_t status)
{
    if (!el3_rx_can_receive(c, len)) {
        return false;
    }
    
    /* Get descriptor for new packet */
    RXPacketDesc *desc = &c->rx_packets[c->rx_packet_tail];
    desc->status = status;
    desc->length = len;
    desc->bytes_read = 0;
    desc->data_offset = c->rx_data_write_ptr;
    desc->complete = true;
    
    /* Copy data to buffer */
    size_t copy_len = len;
    for (size_t i = 0; i < copy_len; i++) {
        c->rx_data[c->rx_data_write_ptr] = data[i];
        c->rx_data_write_ptr = (c->rx_data_write_ptr + 1) % 4096;
    }
    
    /* Add padding for word alignment */
    if (len & 1) {
        c->rx_data[c->rx_data_write_ptr] = 0;
        c->rx_data_write_ptr = (c->rx_data_write_ptr + 1) % 4096;
        copy_len++;
    }
    
    c->rx_data_used += copy_len;
    
    /* Update queue pointers */
    c->rx_packet_tail = (c->rx_packet_tail + 1) % RX_MAX_PACKETS;
    c->rx_packet_count++;
    
    return true;
}

/* Read data from current RX packet */
static size_t el3_rx_read_data(EL3Core *c, uint8_t *data, size_t len)
{
    if (c->rx_packet_count == 0) {
        return 0;  /* No packets available */
    }
    
    RXPacketDesc *desc = &c->rx_packets[c->rx_packet_head];
    
    /* Calculate how many bytes available in current packet */
    size_t available = desc->length - desc->bytes_read;
    size_t to_read = MIN(len, available);
    
    if (to_read == 0) {
        return 0;
    }
    
    /* Read from data buffer */
    size_t read_ptr = (desc->data_offset + desc->bytes_read) % 4096;
    for (size_t i = 0; i < to_read; i++) {
        data[i] = c->rx_data[read_ptr];
        read_ptr = (read_ptr + 1) % 4096;
    }
    
    desc->bytes_read += to_read;
    return to_read;
}

/* Get bytes available in current RX packet (for Window 1 0x0A) */
static uint16_t el3_rx_bytes_available(EL3Core *c)
{
    if (c->rx_packet_count == 0) {
        return 0;
    }
    
    RXPacketDesc *desc = &c->rx_packets[c->rx_packet_head];
    return (desc->length - desc->bytes_read) & 0x7FF;  /* Mask to 11 bits */
}

/* Discard current RX packet */
static void el3_rx_discard_packet(EL3Core *c)
{
    if (c->rx_packet_count == 0) {
        return;  /* No packet to discard */
    }
    
    RXPacketDesc *desc = &c->rx_packets[c->rx_packet_head];
    
    /* Calculate space to free (including padding) */
    size_t to_free = desc->length;
    if (desc->length & 1) {
        to_free++;  /* Account for padding */
    }
    
    /* If this was the only packet, reset pointers for efficiency */
    if (c->rx_packet_count == 1) {
        c->rx_data_write_ptr = 0;
        c->rx_data_used = 0;
    } else {
        c->rx_data_used -= to_free;
    }
    
    /* Move to next packet */
    c->rx_packet_head = (c->rx_packet_head + 1) % RX_MAX_PACKETS;
    c->rx_packet_count--;
}

/* Write to TX FIFO */
static bool el3_tx_fifo_write(EL3Core *c, const uint8_t *data, size_t len)
{
    if (c->tx_fifo_used + len > 4096) {
        return false;  /* FIFO full */
    }
    
    for (size_t i = 0; i < len; i++) {
        c->tx_fifo[c->tx_fifo_write_ptr] = data[i];
        c->tx_fifo_write_ptr = (c->tx_fifo_write_ptr + 1) % 4096;
    }
    
    c->tx_fifo_used += len;
    return true;
}

/* Read from TX FIFO */
static size_t el3_tx_fifo_read(EL3Core *c, uint8_t *data, size_t len)
{
    size_t to_read = MIN(len, c->tx_fifo_used);
    
    for (size_t i = 0; i < to_read; i++) {
        data[i] = c->tx_fifo[c->tx_fifo_read_ptr];
        c->tx_fifo_read_ptr = (c->tx_fifo_read_ptr + 1) % 4096;
    }
    
    c->tx_fifo_used -= to_read;
    return to_read;
}

/* Get used space in TX FIFO */
static uint16_t el3_tx_fifo_used(EL3Core *c)
{
    return c->tx_fifo_used;
}

/* Get free space in TX FIFO */
static uint16_t el3_tx_fifo_free(EL3Core *c)
{
    /* Report actual free space in FIFO */
    return TX_FIFO_SIZE - el3_tx_fifo_used(c);
}

/* Reset TX FIFO */
static void el3_reset_tx_fifo(EL3Core *c)
{
    c->tx_fifo_write_ptr = 0;
    c->tx_fifo_read_ptr = 0;
    c->tx_fifo_used = 0;
}

/* Forward declarations for TX functions */
static void el3_tx_bh(void *opaque);

/* Check if TX threshold reached and schedule transmission */
static void el3_check_tx_threshold(EL3Core *c)
{
    if (!c->tx_in_progress || !c->tx_enabled) {
        return;
    }
    
    /* For now, only start TX when full packet is available
     * This avoids spurious underruns with threshold starts */
    if (c->current_tx_written >= c->current_tx_len) {
        if (c->tx_bh) {
            qemu_bh_schedule(c->tx_bh);
        }
    }
}

/**
 * TX Bottom Half - handles actual packet transmission
 * 
 * Hardware model assumptions:
 * - Frame length excludes FCS (host provides data, hardware adds FCS)
 * - TX does not parse EtherTypes for length validation (transmits what host provides)
 * - No length-based jabber detection (jabber is time-based in real hardware)
 * - Statistics include FCS in byte counts (802.3 standard)
 * - Minimum frame padded to 60 bytes (before FCS) by hardware
 * - Errors reported via TX status register, not separate interrupts
 */
static void el3_tx_bh(void *opaque)
{
    EL3Core *c = opaque;
    uint8_t chunk_buf[512];  /* Safe chunk size for reading FIFO */
    g_autofree uint8_t *frame_buf = NULL;
    size_t frame_len = 0;
    size_t remaining = c->current_tx_len;
    bool send_frame = true;
    
    /* Track per-TX error status separately from sticky tx_status register */
    uint16_t tx_bits = TX_STAT_COMPLETE;
    uint16_t tx_err_bits = 0;
    
    /* Check if TX is still enabled (could have been reset) */
    if (!c->tx_enabled || !c->tx_in_progress) {
        /* TX was disabled/reset - drain FIFO and abort cleanly */
        while (remaining > 0) {
            size_t chunk = MIN(remaining, sizeof(chunk_buf));
            size_t got = el3_tx_fifo_read(c, chunk_buf, chunk);
            if (got == 0) break;  /* FIFO empty */
            remaining -= got;
        }
        
        /* Clean state transition: clear progress flag after FIFO drain */
        c->tx_status |= tx_bits;  /* Mark complete even on abort */
        c->tx_in_progress = false;
        
        /* Ensure TX available status reflects freed FIFO space */
        el3_update_tx_available(c);
        goto tx_done;
    }
    
    /* Apply emulator defensive sanity cap - not a hardware limit */
    if (c->current_tx_len > EL3_TX_SANITY_MAX) {
        /* Use JABBER for oversize frames - this is the correct 3Com hardware behavior */
        tx_err_bits |= TX_STAT_JABBER;
        send_frame = false;
        /* Update oversize counter for diagnostics */
        if (c->stats.tx_oversize < 0xFF) {
            c->stats.tx_oversize++;
        }
        trace_el3_tx_oversize(c->current_tx_len);
    }
    
    /* Special case: zero-length frame is an error */
    if (c->current_tx_len == 0) {
        /* Zero-length transmit is invalid - bad length case, use JABBER like other length errors */
        tx_err_bits |= TX_STAT_JABBER;
        send_frame = false;
        /* Update oversize counter for diagnostics - covers all "bad length" cases */
        if (c->stats.tx_oversize < 0xFF) {
            c->stats.tx_oversize++;
        }
        trace_el3_tx_zero_length();
        c->tx_status |= tx_bits | tx_err_bits;
        c->tx_in_progress = false;
        goto tx_done;
    }
    
    /* Allocate frame buffer if we need to send */
    if (send_frame) {
        frame_buf = g_malloc(MAX(c->current_tx_len, ETH_MIN_DATA_NOFCS));
    }
    
    /* Read frame data from FIFO in safe chunks */
    while (remaining > 0) {
        size_t chunk = MIN(remaining, sizeof(chunk_buf));
        size_t got = el3_tx_fifo_read(c, chunk_buf, chunk);
        
        if (got == 0) {
            /* FIFO underrun - not enough data available */
            trace_el3_tx_fifo_empty(c->current_tx_len, c->tx_fifo_used);
            tx_err_bits |= TX_STAT_UNDERRUN;
            send_frame = false;
            break;
        }
        
        /* Copy to frame buffer if sending, otherwise just discard */
        if (send_frame && frame_buf && (frame_len + got <= c->current_tx_len)) {
            memcpy(frame_buf + frame_len, chunk_buf, got);
        }
        
        frame_len += got;
        remaining -= got;
    }
    
    /* Drain any remaining FIFO data on errors to prevent resource leaks */
    if (tx_err_bits && remaining > 0) {
        while (remaining > 0) {
            size_t chunk = MIN(remaining, sizeof(chunk_buf));
            size_t got = el3_tx_fifo_read(c, chunk_buf, chunk);
            if (got == 0) break;  /* FIFO truly empty */
            remaining -= got;
        }
    }
    
    /* Update 3Com-specific TX error counters */
    if (c->stats_enabled && !c->stats_frozen) {
        if (tx_err_bits & TX_STAT_UNDERRUN) {
            if (c->stats.tx_underruns < 0xFF) {
                c->stats.tx_underruns++;
            }
        }
    }
    /* Note: No jabber counter since we removed length-based jabber detection */
    
    /* Always mark transmission complete (even on errors) */
    c->tx_status |= tx_bits | tx_err_bits;
    c->tx_in_progress = false;
    
    /* Send frame if no errors occurred */
    if (send_frame && frame_buf) {
        size_t on_wire_len;
        
        /* For ISA 16-bit PIO, odd-length frames are internally zero-padded by hardware */
        size_t padded_len = frame_len;
        if (c->model == EL3_509B && (frame_len & 1)) {
            /* 3C509B ISA: pad odd-length frames with zero byte for 16-bit PIO alignment */
            frame_buf[frame_len] = 0;
            padded_len = frame_len + 1;
        }
        
        /* Pad to minimum Ethernet frame if needed */
        if (padded_len < ETH_MIN_DATA_NOFCS) {
            trace_el3_tx_padding(padded_len, ETH_MIN_DATA_NOFCS);
            memset(frame_buf + padded_len, 0, ETH_MIN_DATA_NOFCS - padded_len);
            on_wire_len = ETH_MIN_DATA_NOFCS;
        } else {
            on_wire_len = padded_len;
        }
        
        /* Send packet or deliver to loopback */
        if (c->internal_loopback) {
            /* Internal loopback: deliver to RX engine if RX enabled */
            if (c->rx_enabled) {
                el3_core_receive(qemu_get_queue(c->nic), frame_buf, on_wire_len);
            }
        } else {
            /* Normal transmission: send to network */
            qemu_send_packet(qemu_get_queue(c->nic), frame_buf, on_wire_len);
        }
        
        /* Update statistics - only on successful transmission (no per-TX errors) */
        if (!tx_err_bits && c->stats_enabled && !c->stats_frozen) {
            /* Calculate statistics length including FCS (on_wire_len already padded to 60 bytes minimum) */
            size_t stats_len = on_wire_len + ETH_FCS_LEN;
            
            /* tx_frames_ok is 8-bit counter (per 3C509B docs) */
            if (c->stats.tx_frames_ok < 0xFF) {
                c->stats.tx_frames_ok++;
            }
            
            /* tx_bytes_ok with FCS, saturate at 16-bit */
            uint32_t new_bytes = c->stats.tx_bytes_ok + stats_len;
            if (new_bytes > 0xFFFF) {
                c->stats.tx_bytes_ok = 0xFFFF;
            } else {
                c->stats.tx_bytes_ok = (uint16_t)new_bytes;
            }
        }
    }
    
    /* frame_buf automatically freed by g_autofree */
    
tx_done:
    /* Generate TX complete interrupt event - always set for all frames */
    c->int_status |= STAT_TX_COMPLETE;
    
    /* 3Com TX errors are reported via TX status register, not separate interrupt */
    /* This matches hardware behavior - drivers read TX status after TX complete IRQ */
    /* Only use ADAPTER_FAIL for true hardware failures, not normal TX errors */
    
    el3_update_irq(c);
    
    /* Update TX available status after freeing FIFO space */
    el3_update_tx_available(c);
    
    trace_el3_tx_packet(c->current_tx_len);
    trace_el3_tx_complete(c->tx_status);
}

/* ISA ID Sequence Implementation (3C509) */

/* Linear Feedback Shift Register for ID sequence */
static uint8_t el3_lfsr_step(uint8_t state)
{
    /* LFSR polynomial from Linux driver: x^8 + x^6 + x^3 + x^2 + 1 (0xCF) */
    state <<= 1;
    if (state & 0x100) {
        state ^= 0xCF;
    }
    return state & 0xFF;
}

/* Initialize ID sequence data from EEPROM */
static void el3_id_sequence_init(EL3Core *c)
{
    /* Initialize ID port FSM */
    c->id_state.state = ID_IDLE;
    c->id_state.unlock_pos = 0;
    c->id_state.board_tag = 0;
    c->id_state.selected_tag = 0;
    c->id_state.bit_pos = 0;
    
    /* Set 32-bit EISA product ID based on model (Linux 3c509.c compatible) */
    switch (c->model) {
    case MODEL_3C509:
        /* 3C509: Vendor "TCM" 0x506D, Product 0x5090 -> 0x506D5090 */
        c->id_state.product_id = 0x506D5090;
        break;
    case MODEL_3C509B:
        /* 3C509B: Vendor "TCM" 0x506D, Product 0x5091 -> 0x506D5091 */
        c->id_state.product_id = 0x506D5091;
        break;
    default:
        c->id_state.product_id = 0x506D5090;
        break;
    }
}

/* Reset ID sequence */
void el3_id_sequence_reset(EL3Core *c)
{
    c->id_state.state = ID_IDLE;
    c->id_state.unlock_pos = 0;
    c->id_state.board_tag = 0;
    c->id_state.selected_tag = 0;
    c->id_state.bit_pos = 0;
}

/* ID port write handler */
void el3_id_port_write(EL3Core *c, uint8_t val)
{
    if (c->model != MODEL_3C509 && c->model != MODEL_3C509B) {
        return;  /* ID sequence only for ISA 3C509 */
    }
    
    /* Standard 32-byte unlock sequence as expected by Linux 3c509.c */
    static const uint8_t unlock_sequence[32] = {
        0x00, 0x00, 0x6a, 0xb5, 0xda, 0xed, 0xf6, 0xfb, 
        0x7d, 0xbe, 0xdf, 0x6f, 0x37, 0x1b, 0x0d, 0x86,
        0xc3, 0x61, 0xb0, 0x58, 0x2c, 0x16, 0x8b, 0x45, 
        0xa2, 0xd1, 0x68, 0x34, 0x1a, 0x0d, 0x86, 0xc3
    };
    
    switch (c->id_state.state) {
    case ID_IDLE:
        if (val == 0x00) {
            /* Start unlock sequence */
            c->id_state.state = ID_UNLOCKING;
            c->id_state.unlock_pos = 1;  /* Next expected byte */
        }
        break;
        
    case ID_UNLOCKING:
        if (val == unlock_sequence[c->id_state.unlock_pos]) {
            c->id_state.unlock_pos++;
            if (c->id_state.unlock_pos >= 32) {
                /* Unlock complete - enter isolation mode */
                c->id_state.state = ID_ISOLATION;
                c->id_state.bit_pos = 0;
                trace_el3_id_unlocked();
            }
        } else {
            /* Wrong byte - reset to idle */
            c->id_state.state = ID_IDLE;
            c->id_state.unlock_pos = 0;
            /* Check if this byte could start the sequence again */
            if (val == 0x00) {
                c->id_state.state = ID_UNLOCKING;
                c->id_state.unlock_pos = 1;
            }
        }
        break;
        
    case ID_ISOLATION:
        if (val == 0xFF) {
            /* Add tag command - increment our board tag */
            c->id_state.board_tag++;
            trace_el3_id_add_tag(c->id_state.board_tag);
        } else if ((val & 0xF0) == 0xD0) {
            /* Select board command - Linux 3c509.c uses 0xD0 | tag */
            c->id_state.selected_tag = val & 0x0F;
            if (c->id_state.selected_tag == c->id_state.board_tag) {
                c->id_state.state = ID_SELECTED;
                trace_el3_id_selected(c->id_state.selected_tag);
            }
        }
        break;
        
    case ID_SELECTED:
        if (val == 0xFF) {
            /* Activate command - move to config state */
            c->id_state.state = ID_CONFIG;
            trace_el3_id_activated();
        } else if ((val & 0xF0) == 0xD0) {
            /* Different tag selected */
            c->id_state.selected_tag = val & 0x0F;
            if (c->id_state.selected_tag != c->id_state.board_tag) {
                c->id_state.state = ID_ISOLATION;
            }
        }
        break;
        
    case ID_CONFIG:
        /* Card is activated - ID port no longer responds */
        break;
    }
}

/* ID port read handler */
uint8_t el3_id_port_read(EL3Core *c)
{
    if (c->model != MODEL_3C509 && c->model != MODEL_3C509B) {
        return 0xFF;  /* ID sequence only for ISA 3C509 */
    }
    
    /* Only respond in isolation or selected states */
    if (c->id_state.state != ID_ISOLATION && c->id_state.state != ID_SELECTED) {
        return 0xFF;
    }
    
    /* Return 32-bit EISA product ID bit by bit */
    if (c->id_state.bit_pos < 32) {
        /* Get the bit from MSB to LSB */
        uint8_t bit = (c->id_state.product_id >> (31 - c->id_state.bit_pos)) & 1;
        c->id_state.bit_pos++;
        
        /* Return contention pattern: 1 = 0x55, 0 = 0xAA */
        return bit ? 0x55 : 0xAA;
    }
    
    /* After 32 bits, return 0xFF (no more data) */
    return 0xFF;
}

/* RX filter functions */

/* Helper functions for packet filtering */
static bool is_broadcast(const uint8_t *addr)
{
    static const uint8_t bcast[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    return memcmp(addr, bcast, 6) == 0;
}

static bool is_multicast(const uint8_t *addr)
{
    return (addr[0] & 0x01) != 0;
}

static bool mac_eq(const uint8_t *a, const uint8_t *b)
{
    return memcmp(a, b, 6) == 0;
}

/* Set RX filter configuration */
void el3_set_rx_filter(EL3Core *c, uint16_t filter)
{
    c->rx_filter = filter;
    trace_el3_filter_set(filter);
}

/* Check if frame should be accepted based on filter settings */
bool el3_accept_frame(EL3Core *c, const uint8_t *buf, size_t len)
{
    if (len < 6) {
        /* Too short for valid Ethernet frame */
        return false;
    }
    
    const uint8_t *dst = buf;

    /* Promiscuous mode accepts everything */
    if (c->rx_filter & RX_FILTER_PROMISCUOUS) {
        trace_el3_filter_accept("promiscuous");
        return true;
    }

    /* Check individual (unicast) address - use Window 2 station address */
    if (c->rx_filter & RX_FILTER_INDIVIDUAL) {
        /* Station address is in Window 2, offsets 0x00-0x05 */
        uint8_t station_addr[6];
        station_addr[0] = c->windows[2][0] & 0xFF;
        station_addr[1] = (c->windows[2][0] >> 8) & 0xFF;
        station_addr[2] = c->windows[2][1] & 0xFF;
        station_addr[3] = (c->windows[2][1] >> 8) & 0xFF;
        station_addr[4] = c->windows[2][2] & 0xFF;
        station_addr[5] = (c->windows[2][2] >> 8) & 0xFF;
        
        if (memcmp(dst, station_addr, 6) == 0) {
            trace_el3_filter_accept("individual");
            return true;
        }
    }

    /* Check broadcast */
    if ((c->rx_filter & RX_FILTER_BROADCAST) && is_broadcast(dst)) {
        trace_el3_filter_accept("broadcast");
        return true;
    }

    /* Check multicast */
    if (is_multicast(dst)) {
        if (c->rx_filter & RX_FILTER_ALLMULTI) {
            trace_el3_filter_accept("all_multicast");
            return true;
        }
        if (c->rx_filter & RX_FILTER_MULTICAST) {
            /* For now, accept all multicast if bit is set */
            /* TODO: Add proper multicast hash table support */
            trace_el3_filter_accept("multicast");
            return true;
        }
        trace_el3_filter_reject("multicast_filtered");
        return false;
    }

    /* Not accepted by any filter */
    trace_el3_filter_reject("no_match");
    return false;
}

/* Core state management functions */
uint8_t el3_core_get_window(EL3Core *c)
{
    return c->current_window;
}

bool el3_core_is_rx_enabled(EL3Core *c)
{
    /* RX enabled when RX_ENABLE command sent and not explicitly disabled */
    return (c->status & STAT_RX_COMPLETE) == 0; /* Simple check for now */
}

bool el3_core_is_tx_enabled(EL3Core *c)
{
    /* TX enabled when TX_ENABLE command sent and not explicitly disabled */
    return (c->status & STAT_TX_COMPLETE) == 0; /* Simple check for now */
}

uint16_t el3_core_get_status(EL3Core *c)
{
    return c->status;
}

void el3_core_set_status(EL3Core *c, uint16_t bits)
{
    c->status |= bits;
    el3_update_irq(c);
}

void el3_core_clear_status(EL3Core *c, uint16_t bits)
{
    c->status &= ~bits;
    el3_update_irq(c);
}

uint16_t el3_core_get_int_status(EL3Core *c)
{
    return c->int_status;
}

void el3_core_set_int_status(EL3Core *c, uint16_t bits)
{
    c->int_status |= bits;
    el3_update_irq(c);
}

void el3_core_clear_int_status(EL3Core *c, uint16_t bits)
{
    /* Clear acknowledged interrupt events from int_status */
    c->int_status &= ~bits;
    
    /* Clear acknowledged bits from status register EXCEPT DMA_DONE (handle it specially) */
    uint16_t ackable = bits & ~(STAT_INT_LATCH | STAT_INT_REQ | STAT_CMD_IN_PROG | STAT_DMA_DONE);
    c->status &= ~ackable;
    
    /* Special handling for DMA_DONE - only clear if UpStatus/DownStatus are clear */
    if (bits & STAT_DMA_DONE) {
        if (c->up_status == 0 && c->down_status == 0) {
            c->status &= ~STAT_DMA_DONE;
        }
        /* else leave DMA_DONE set */
    }
    
    el3_update_irq(c);
}

bool el3_core_should_interrupt(EL3Core *c)
{
    /* Check if any enabled interrupts are pending */
    return ((c->status & c->status_enb) & c->intr_enb) != 0;
}

/* Network interface management functions */
bool el3_core_init_nic(EL3Core *c, DeviceState *dev, Error **errp)
{
    /* This should be called by variant after setting up conf */
    if (!c->conf.macaddr.a[0] && !c->conf.macaddr.a[1] && !c->conf.macaddr.a[2] &&
        !c->conf.macaddr.a[3] && !c->conf.macaddr.a[4] && !c->conf.macaddr.a[5]) {
        qemu_macaddr_default_if_unset(&c->conf.macaddr);
    }
    
    /* NIC creation is variant-specific, so this is just a helper for common setup */
    return true;
}

void el3_core_post_realize(EL3Core *c)
{
    /* Post-realization setup */
    if (c->nic) {
        qemu_format_nic_info_str(qemu_get_queue(c->nic), c->conf.macaddr.a);
    }
}

void el3_core_unrealize(EL3Core *c)
{
    /* Cleanup timers */
    if (c->eeprom_timer) {
        timer_free(c->eeprom_timer);
        c->eeprom_timer = NULL;
    }
    if (c->cmd_timer) {
        timer_free(c->cmd_timer);
        c->cmd_timer = NULL;
    }
}

/* TX/RX management functions */
uint32_t el3_core_get_pending_tx_len(EL3Core *c)
{
    /* This is variant-specific - return 0 for now */
    /* Variants will override this behavior */
    return 0;
}

bool el3_core_has_tx_space(EL3Core *c, size_t len)
{
    if (c->ops && c->ops->tx_kick) {
        /* Let variant decide */
        c->ops->tx_kick(c);
        return true; /* Optimistic for now */
    }
    return false;
}

void el3_core_tx_complete(EL3Core *c, uint32_t status)
{
    /* Set TX completion status */
    el3_core_set_status(c, STAT_TX_COMPLETE | STAT_TX_AVAILABLE);
    el3_core_set_int_status(c, STAT_TX_COMPLETE);
    
    /* Call variant completion handler */
    if (c->ops && c->ops->tx_on_core_sent) {
        c->ops->tx_on_core_sent(c, status);
    }
}

void el3_core_rx_notify(EL3Core *c, size_t len)
{
    /* Set RX completion status */
    el3_core_set_status(c, STAT_RX_COMPLETE);
    el3_core_set_int_status(c, STAT_RX_COMPLETE);
    
    /* Update statistics */
    if (c->stats_enabled) {
        c->stats.rx_frames_ok++;
        c->stats.rx_bytes_ok += len;
    }
}

/* Statistics management functions */
void el3_core_update_stats(EL3Core *c, bool tx, bool ok, size_t bytes)
{
    if (!c->stats_enabled) {
        return;
    }
    
    if (tx) {
        if (ok) {
            /* tx_frames_ok is 8-bit counter with saturation */
            if (c->stats.tx_frames_ok < 0xFF) {
                c->stats.tx_frames_ok++;
            } else {
                trace_el3_stats_saturate("tx_frames_ok", 0xFF);
            }
            /* tx_bytes_ok is 16-bit counter with saturation */
            if (c->stats.tx_bytes_ok <= 0xFFFF - bytes) {
                c->stats.tx_bytes_ok += bytes;
            } else {
                c->stats.tx_bytes_ok = 0xFFFF; /* Saturate */
                trace_el3_stats_saturate("tx_bytes_ok", 0xFFFF);
            }
            trace_el3_stats_update(tx, ok, bytes, c->stats.tx_frames_ok);
        }
        /* TODO: Add error counters based on status */
    } else {
        if (ok) {
            /* rx_frames_ok is 8-bit counter with saturation */
            if (c->stats.rx_frames_ok < 0xFF) {
                c->stats.rx_frames_ok++;
            } else {
                trace_el3_stats_saturate("rx_frames_ok", 0xFF);
            }
            /* rx_bytes_ok is 16-bit counter with saturation */
            if (c->stats.rx_bytes_ok <= 0xFFFF - bytes) {
                c->stats.rx_bytes_ok += bytes;
            } else {
                c->stats.rx_bytes_ok = 0xFFFF; /* Saturate */
                trace_el3_stats_saturate("rx_bytes_ok", 0xFFFF);
            }
            trace_el3_stats_update(tx, ok, bytes, c->stats.rx_frames_ok);
        } else {
            /* rx_overruns is 8-bit counter with saturation */
            if (c->stats.rx_overruns < 0xFF) {
                c->stats.rx_overruns++;
            } else {
                trace_el3_stats_saturate("rx_overruns", 0xFF);
            }
        }
    }
}

void el3_core_clear_stats(EL3Core *c)
{
    memset(&c->stats, 0, sizeof(c->stats));
    memset(&c->stats_snapshot, 0, sizeof(c->stats_snapshot));
    c->stats_frozen = false;
    trace_el3_stats_zero();
}

/* Command timing table for CIP semantics */
typedef struct {
    uint16_t command;
    uint64_t duration_ns;  /* Duration in nanoseconds */
    bool allow_while_cip;  /* Allow command while CIP is set */
} CommandTiming;

static const CommandTiming command_timings[] = {
    { CMD_GLOBAL_RESET,     1000000, false },  /* 1ms */
    { CMD_SELECT_WINDOW,          0, true  },  /* Immediate */
    { CMD_RX_ENABLE,              0, false },  /* Immediate */
    { CMD_RX_DISABLE,             0, false },  /* Immediate */
    { CMD_TX_ENABLE,              0, false },  /* Immediate */
    { CMD_TX_DISABLE,             0, false },  /* Immediate */
    { CMD_RX_RESET,          200000, false },  /* 200μs */
    { CMD_TX_RESET,          200000, false },  /* 200μs */
    { CMD_ACK_INTR,               0, true  },  /* Always allowed */
    { CMD_REQUEST_INTR,           0, false },  /* Immediate */
    { CMD_SET_INTR_ENB,           0, false },  /* Immediate */
    { CMD_SET_STATUS_ENB,         0, false },  /* Immediate */
    { CMD_SET_RX_FILTER,      50000, false },  /* 50μs */
    { CMD_SET_RX_EARLY_THRESH,    0, false },  /* Immediate */
    { CMD_SET_TX_AVAIL,           0, false },  /* Immediate */
    { CMD_SET_TX_START_THRESH,    0, false },  /* Immediate */
    { CMD_TX_START,           20000, false },  /* 20μs */
    { CMD_STATS_ENABLE,           0, false },  /* Immediate */
    { CMD_STATS_DISABLE,          0, false },  /* Immediate */
    { CMD_RX_DISCARD,        100000, false },  /* 100μs */
    { CMD_START_COAX,         50000, false },  /* 50μs */
    { CMD_STOP_COAX,          50000, false },  /* 50μs */
    { CMD_UP_STALL,               0, false },  /* Immediate */
    { CMD_DOWN_STALL,             0, false },  /* Immediate */
    { CMD_START_DMA_UP,       10000, false },  /* 10μs */
    { CMD_START_DMA_DOWN,     10000, false },  /* 10μs */
    { 0, 0, false }  /* End marker */
};

/* Centralized CIP handling helper */
static bool el3_try_start_cmd(EL3Core *c, uint16_t command, uint16_t param)
{
    const CommandTiming *timing = NULL;
    
    /* Find timing for this command */
    for (int i = 0; command_timings[i].command != 0; i++) {
        if (command_timings[i].command == command) {
            timing = &command_timings[i];
            break;
        }
    }
    
    /* If no timing found, use defaults */
    bool allow_while_cip = (timing ? timing->allow_while_cip : false);
    uint64_t duration_ns = (timing ? timing->duration_ns : 0);
    
    /* Check if command is allowed while CIP is set */
    if (c->status & STAT_CMD_IN_PROG) {
        if (!allow_while_cip) {
            trace_el3_cmd_ignored(command | param, "CIP set");
            return false;
        }
        /* Command allowed while CIP set - execute immediately */
        return true;
    }
    
    /* Set CIP if command has duration */
    if (duration_ns > 0) {
        c->status |= STAT_CMD_IN_PROG;
        timer_mod_ns(c->cmd_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + duration_ns);
    }
    
    return true;
}

/* Command processing function */
void el3_core_process_command(EL3Core *c, uint16_t cmd)
{
    uint16_t command = cmd & 0xF800;  /* Extract command bits 15-11 */
    uint16_t param = cmd & 0x07FF;    /* Extract parameter bits 10-0 */
    
    trace_el3_command(command, param);
    
    /* Check CIP and timing constraints */
    if (!el3_try_start_cmd(c, command, param)) {
        return;  /* Command ignored due to CIP constraints */
    }
    
    switch (command) {
    case CMD_GLOBAL_RESET:
        /* Perform the reset */
        el3_core_reset(c);
        break;
        
    case CMD_SELECT_WINDOW:
        el3_select_window(c, param & 0x07);
        break;
        
    case CMD_RX_ENABLE:
        c->rx_enabled = true;
        /* Recompute RX status based on actual state */
        if (c->ops && c->ops->kick) {
            c->ops->kick(c, EL3_KICK_RX);
        }
        break;
        
    case CMD_RX_DISABLE:
        c->rx_enabled = false;
        /* Clear RX-related status bits */
        el3_core_clear_status(c, STAT_RX_COMPLETE | STAT_RX_EARLY);
        break;
        
    case CMD_TX_ENABLE:
        c->tx_enabled = true;
        /* Recompute TX available status based on FIFO and threshold */
        if (c->ops && c->ops->kick) {
            c->ops->kick(c, EL3_KICK_TX);
        }
        break;
        
    case CMD_TX_DISABLE:
        c->tx_enabled = false;
        /* Abort any in-progress TX */
        if (c->tx_in_progress) {
            c->tx_status |= TX_STAT_UNDERRUN;
            c->tx_in_progress = false;
            /* Generate TX complete event for aborted transmission */
            c->int_status |= STAT_TX_COMPLETE;
            if (c->tx_bh) {
                qemu_bh_cancel(c->tx_bh);
            }
            el3_update_irq(c);
        }
        /* Clear TX-related live status bits */
        c->status &= ~(STAT_TX_AVAILABLE | STAT_TX_COMPLETE);
        break;
        
    case CMD_RX_RESET:
        /* Reset RX path */
        if (c->ops && c->ops->kick) {
            c->ops->kick(c, EL3_KICK_RX);
        }
        break;
        
    case CMD_TX_RESET:
        /* Reset TX path and FIFO completely */
        el3_reset_tx_fifo(c);
        c->tx_status = 0;
        c->tx_in_progress = false;
        c->current_tx_len = 0;
        c->current_tx_written = 0;
        c->tx_enabled = false;  /* TX_RESET disables transmitter like real hardware */
        
        /* Clear latched TX-related interrupt events */
        c->int_status &= ~(STAT_TX_COMPLETE | STAT_TX_AVAILABLE);
        
        /* Cancel any pending TX operation */
        if (c->tx_bh) {
            qemu_bh_cancel(c->tx_bh);
        }
        
        /* Clear TX-related live status bits */
        c->status &= ~(STAT_TX_AVAILABLE | STAT_TX_COMPLETE);
        
        /* Update interrupt status */
        el3_update_irq(c);
        
        if (c->ops && c->ops->kick) {
            c->ops->kick(c, EL3_KICK_TX);
        }
        break;
        
    case CMD_ACK_INTR:
        /* Acknowledge interrupts */
        el3_core_clear_int_status(c, param);
        
        /* If TX_AVAILABLE was acknowledged, check if it should be re-latched */
        if (param & STAT_TX_AVAILABLE) {
            el3_update_tx_available(c);
        }
        
        /* If RX_COMPLETE was acknowledged, check if it should be re-latched */
        if (param & STAT_RX_COMPLETE) {
            /* Re-latch RX_COMPLETE if more complete packets remain */
            if (c->rx_packet_count > 0) {
                RXPacketDesc *desc = &c->rx_packets[c->rx_packet_head];
                
                /* Check if packet is complete (should always be true in our impl) */
                if (desc->complete && !(desc->status & RX_STATUS_INCOMPLETE)) {
                    /* Re-latch RX_COMPLETE */
                    c->status |= STAT_RX_COMPLETE;
                    el3_update_irq(c);
                }
            }
        }
        
        /* If STATS_FULL was acknowledged, check if it should be re-latched */
        if (param & STAT_STATS_FULL) {
            /* Re-latch STATS_FULL if any counter is still saturated */
            if (c->stats.rx_frames_ok == 0xFF || c->stats.rx_overruns == 0xFF ||
                c->stats.tx_frames_ok == 0xFF || c->stats.tx_collisions == 0xFF ||
                c->stats.tx_carrier_errors == 0xFF || c->stats.tx_heartbeat_errors == 0xFF ||
                c->stats.tx_mult_collisions == 0xFF || c->stats.tx_single_collisions == 0xFF ||
                c->stats.tx_late_collisions == 0xFF || c->stats.tx_underruns == 0xFF ||
                c->stats.rx_bytes_ok == 0xFFFF || c->stats.tx_bytes_ok == 0xFFFF ||
                c->stats.rx_discards_ok == 0xFF || c->stats.tx_discards_ok == 0xFF) {
                /* Re-latch STATS_FULL */
                c->status |= STAT_STATS_FULL;
                el3_update_irq(c);
            }
        }
        break;
        
    case CMD_REQUEST_INTR:
        /* Request/Force interrupt - test IRQ line */
        if (c->intr_enb) {
            c->status |= STAT_INT_REQ;
            el3_update_irq(c);
        }
        break;
        
    case CMD_SET_INTR_ENB:
        /* Set interrupt enable mask */
        c->intr_enb = param;
        el3_update_irq(c);
        break;
        
    case CMD_SET_STATUS_ENB:
        /* Set status enable mask */
        c->status_enb = param;
        el3_update_irq(c);
        break;
        
    case CMD_SET_RX_FILTER:
        el3_set_rx_filter(c, param);
        break;
        
    case CMD_SET_RX_EARLY_THRESH:
        /* Set RX early interrupt threshold */
        c->rx_early_thresh = param;
        break;
        
    case CMD_SET_TX_AVAIL:
        /* Set TX available interrupt threshold */
        c->tx_avail_thresh = param & 0xFFF;  /* 12-bit value */
        
        /* Clamp to FIFO capacity to prevent unreachable conditions */
        if (c->tx_avail_thresh > TX_FIFO_SIZE) {
            c->tx_avail_thresh = TX_FIFO_SIZE;
        }
        
        /* Special case: 0 disables the TxAvailable event */
        if (c->tx_avail_thresh == 0) {
            c->status &= ~STAT_TX_AVAILABLE;
            c->int_status &= ~STAT_TX_AVAILABLE;
            el3_update_irq(c);
        } else {
            /* Update TX available status with new threshold */
            el3_update_tx_available(c);
        }
        break;
        
    case CMD_SET_TX_START_THRESH:
        c->tx_start_thresh = param & 0x7FF;  /* 11-bit value */
        break;
        
    case CMD_TX_START:
        /* Start transmission with length parameter */
        if (!c->tx_enabled) {
            /* Ignore if TX disabled */
            break;
        }
        
        /* Safety checks */
        uint16_t tx_len = param & 0x7FF;  /* 11-bit length */
        if (tx_len == 0) {
            /* Invalid length - ignore */
            break;
        }
        if (c->tx_in_progress) {
            /* Already transmitting - reject new start */
            break;
        }
        if (tx_len > TX_FIFO_SIZE) {
            /* Length exceeds FIFO capacity - clamp */
            tx_len = TX_FIFO_SIZE;
        }
        
        c->current_tx_len = tx_len;
        c->current_tx_written = 0;
        c->tx_in_progress = true;
        trace_el3_tx_start(c->current_tx_len);
        /* Check if we can start transmission immediately */
        el3_check_tx_threshold(c);
        break;
        
    case CMD_STATS_ENABLE:
        /* Freeze current statistics for atomic reading (Linux driver behavior) */
        c->stats_enabled = true;
        c->stats_frozen = true;
        /* Copy live counters to frozen snapshot */
        memcpy(&c->stats_snapshot, &c->stats, sizeof(c->stats));
        trace_el3_stats_enable();
        break;
        
    case CMD_STATS_DISABLE:
        /* Resume live statistics accumulation */
        c->stats_enabled = false;
        c->stats_frozen = false;
        /* Clear STATS_FULL as per hardware behavior */
        c->status &= ~STAT_STATS_FULL;
        trace_el3_stats_disable();
        break;
        
    case CMD_RX_DISCARD:
        /* Discard top packet if any */
        if (c->rx_packet_count > 0) {
            RXPacketDesc *desc = &c->rx_packets[c->rx_packet_head];
            size_t discard_len = desc->length;
            
            /* Discard the packet */
            el3_rx_discard_packet(c);
            
            trace_el3_rx_discard(discard_len);
            
            /* Update Window 1 RX status with next packet's status or clear if empty */
            if (c->rx_packet_count > 0) {
                c->windows[1][W1_RX_STATUS >> 1] = c->rx_packets[c->rx_packet_head].status;
            } else {
                /* Queue empty - clear RX status register */
                c->windows[1][W1_RX_STATUS >> 1] = 0;
            }
            /* Note: RX_COMPLETE is NOT cleared here, only by ACK */
        }
        
        if (c->ops && c->ops->kick) {
            c->ops->kick(c, EL3_KICK_RX);
        }
        break;
        
    case CMD_START_COAX:
        /* Start coax transceiver - no-op for most implementations */
        trace_el3_command_start_coax();
        break;
        
    case CMD_STOP_COAX:
        /* Stop coax transceiver - no-op for most implementations */
        trace_el3_command_stop_coax();
        break;
        
    /* 3C515 DMA commands - variant-gated */
    case CMD_UP_STALL:       /* Same as CMD_DOWN_STALL - 0x3000 */
    case CMD_DOWN_STALL:
        if (c->model == MODEL_3C515) {
            /* Handle DMA stall/unstall based on parameter */
            switch (param) {
            case 0: /* UP_STALL */
                qemu_log_mask(LOG_UNIMP, "el3_core: UP_STALL not implemented\n");
                break;
            case 1: /* UP_UNSTALL */
                qemu_log_mask(LOG_UNIMP, "el3_core: UP_UNSTALL not implemented\n");
                break;
            case 2: /* DOWN_STALL */
                qemu_log_mask(LOG_UNIMP, "el3_core: DOWN_STALL not implemented\n");
                break;
            case 3: /* DOWN_UNSTALL */
                qemu_log_mask(LOG_UNIMP, "el3_core: DOWN_UNSTALL not implemented\n");
                break;
            }
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "el3_core: DMA stall command on non-3C515 model\n");
        }
        break;
        
    case CMD_START_DMA_UP:   /* Same as CMD_START_DMA_DOWN - 0xA000 */
    case CMD_START_DMA_DOWN:
        if (c->model == MODEL_3C515) {
            /* Handle DMA start based on parameter */
            if (param == 0) {
                /* START_DMA_UP */
                qemu_log_mask(LOG_UNIMP, "el3_core: START_DMA_UP not implemented\n");
            } else if (param == 1) {
                /* START_DMA_DOWN */
                qemu_log_mask(LOG_UNIMP, "el3_core: START_DMA_DOWN not implemented\n");
            }
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "el3_core: DMA start command on non-3C515 model\n");
        }
        break;
        
    default:
        qemu_log_mask(LOG_UNIMP, "el3_core: unimplemented command 0x%04x\n", command);
        break;
    }
}

/* Register access dispatch system */
uint32_t el3_core_register_read(EL3Core *c, unsigned win, unsigned off, unsigned size)
{
    uint64_t data = 0;
    MemTxResult result = MEMTX_DECODE_ERROR;
    
    /* Try variant-specific register first */
    if (c->ops && c->ops->reg_read) {
        result = c->ops->reg_read(c, win, off, size, &data);
        if (result == MEMTX_OK) {
            return (uint32_t)data;
        }
    }
    
    /* Fall back to core registers */
    if (win < EL3_MAX_WINDOWS && off < EL3_WINDOW_SIZE) {
        /* Sync Window 3 multicast hash from canonical byte array on reads */
        if (win == 3 && off < 8) {
            /* Regenerate window registers from mcast_hash */
            for (int i = 0; i < 4; i++) {
                c->windows[3][i] = c->mcast_hash[i * 2] | 
                                  (c->mcast_hash[i * 2 + 1] << 8);
            }
        }
        
        /* Special handling for Window 1 TX/RX registers */
        if (win == 1) {
            if (off == W1_TX_RX_FIFO) {
                /* RX data read (Window 1 offset 0x00 on read) - returns only payload bytes */
                if (size == 2) {
                    uint8_t data_bytes[2];
                    size_t read = el3_rx_read_data(c, data_bytes, 2);
                    if (read == 2) {
                        data = data_bytes[0] | (data_bytes[1] << 8);
                    } else if (read == 1) {
                        data = data_bytes[0] | 0xFF00;  /* Pad with 0xFF */
                    } else {
                        data = 0xFFFF;  /* No data available */
                    }
                } else if (size == 1) {
                    uint8_t data_byte;
                    if (el3_rx_read_data(c, &data_byte, 1) == 1) {
                        data = data_byte;
                    } else {
                        data = 0xFF;  /* No data available */
                    }
                }
                trace_el3_read(off, data, size, win);
                return (uint32_t)data;
            } else if (off == W1_TX_STATUS && size == 1) {
                /* TX Status register */
                data = c->tx_status;
                trace_el3_read(off, data, size, win);
                return (uint32_t)data;
            } else if (off == W1_TX_FREE && size == 2) {
                /* TX Free space register */
                data = el3_tx_fifo_free(c);
                trace_el3_read(off, data, size, win);
                return (uint32_t)data;
            } else if (off == W1_RX_STATUS && size == 2) {
                /* RX Status register - return status of packet at head of queue */
                if (c->rx_packet_count > 0) {
                    data = c->rx_packets[c->rx_packet_head].status;
                } else {
                    data = c->windows[1][W1_RX_STATUS >> 1];
                }
                trace_el3_read(off, data, size, win);
                return (uint32_t)data;
            } else if (off == W1_TIMER && size == 2) {
                /* RX Bytes Available register (Window 1 offset 0x0A) */
                /* Returns bytes available in current packet, NOT free space */
                data = el3_rx_bytes_available(c);
                trace_el3_read(off, data, size, win);
                return (uint32_t)data;
            }
        }
        
        /* Special handling for Window 6 statistics (clear-on-read with freeze-latch) */
        if (win == 6 && size == 1) {
            switch (off) {
            case 0x00: /* TX_CARRIER_ERRORS */
                data = c->stats_frozen ? c->stats_snapshot.tx_carrier_errors : c->stats.tx_carrier_errors;
                if (c->stats_frozen) c->stats_snapshot.tx_carrier_errors = 0;
                else c->stats.tx_carrier_errors = 0;
                break;
            case 0x01: /* TX_HEARTBEAT_ERRORS */
                data = c->stats_frozen ? c->stats_snapshot.tx_heartbeat_errors : c->stats.tx_heartbeat_errors;
                if (c->stats_frozen) c->stats_snapshot.tx_heartbeat_errors = 0;
                else c->stats.tx_heartbeat_errors = 0;
                break;
            case 0x02: /* TX_MULT_COLLISIONS */
                data = c->stats_frozen ? c->stats_snapshot.tx_mult_collisions : c->stats.tx_mult_collisions;
                if (c->stats_frozen) c->stats_snapshot.tx_mult_collisions = 0;
                else c->stats.tx_mult_collisions = 0;
                break;
            case 0x03: /* TX_SINGLE_COLLISIONS */
                data = c->stats_frozen ? c->stats_snapshot.tx_single_collisions : c->stats.tx_single_collisions;
                if (c->stats_frozen) c->stats_snapshot.tx_single_collisions = 0;
                else c->stats.tx_single_collisions = 0;
                break;
            case 0x04: /* TX_LATE_COLLISIONS */
                data = c->stats_frozen ? c->stats_snapshot.tx_late_collisions : c->stats.tx_late_collisions;
                if (c->stats_frozen) c->stats_snapshot.tx_late_collisions = 0;
                else c->stats.tx_late_collisions = 0;
                break;
            case 0x05: /* RX_OVERRUNS */
                data = c->stats_frozen ? c->stats_snapshot.rx_overruns : c->stats.rx_overruns;
                if (c->stats_frozen) c->stats_snapshot.rx_overruns = 0;
                else c->stats.rx_overruns = 0;
                break;
            case 0x06: /* TX_FRAMES_OK */
                data = c->stats_frozen ? c->stats_snapshot.tx_frames_ok : c->stats.tx_frames_ok;
                if (c->stats_frozen) c->stats_snapshot.tx_frames_ok = 0;
                else c->stats.tx_frames_ok = 0;
                break;
            case 0x07: /* RX_FRAMES_OK */
                data = c->stats_frozen ? c->stats_snapshot.rx_frames_ok : c->stats.rx_frames_ok;
                if (c->stats_frozen) c->stats_snapshot.rx_frames_ok = 0;
                else c->stats.rx_frames_ok = 0;
                break;
            case 0x08: /* TX_DEFERRALS */
                data = c->stats_frozen ? c->stats_snapshot.tx_deferrals : c->stats.tx_deferrals;
                if (c->stats_frozen) c->stats_snapshot.tx_deferrals = 0;
                else c->stats.tx_deferrals = 0;
                break;
            default:
                /* Fall through to generic handling for other offsets */
                break;
            }
            if (off <= 0x08) {
                trace_el3_read(off, data, size, win);
                return (uint32_t)data;
            }
        }
        
        /* Special handling for Window 6 16-bit byte counters */
        if (win == 6 && size == 2) {
            if (off == 0x0A) { /* RX_BYTES_OK */
                data = c->stats_frozen ? c->stats_snapshot.rx_bytes_ok : c->stats.rx_bytes_ok;
                if (c->stats_frozen) c->stats_snapshot.rx_bytes_ok = 0;
                else c->stats.rx_bytes_ok = 0;
                trace_el3_read(off, data, size, win);
                return (uint32_t)data;
            } else if (off == 0x0C) { /* TX_BYTES_OK */
                data = c->stats_frozen ? c->stats_snapshot.tx_bytes_ok : c->stats.tx_bytes_ok;
                if (c->stats_frozen) c->stats_snapshot.tx_bytes_ok = 0;
                else c->stats.tx_bytes_ok = 0;
                trace_el3_read(off, data, size, win);
                return (uint32_t)data;
            }
        }

        /* Handle Window 0 EEPROM data register */
        if (win == 0 && off == W0_EEPROM_DATA && size == 2) {
            data = el3_eeprom_read(c);
            trace_el3_read(off, data, size, win);
            return (uint32_t)data;
        }

        /* Special handling for Window 7 registers */
        if (win == 7) {
            if (off == W7_VENDOR_ID && size == 2) {
                /* Manufacturer ID - EISA "TCM" (not PCI vendor) */
                data = 0x6D50;  /* Little-endian of 0x506D = "TCM" */
                trace_el3_read(off, data, size, win);
                return (uint32_t)data;
            } else if (off == W7_DEVICE_ID && size == 2) {
                /* Product ID based on model */
                switch (c->model) {
                case MODEL_3C509:
                    data = 0x5090;  /* 3C509 */
                    break;
                case MODEL_3C509B:
                    data = 0x5091;  /* 3C509B */
                    break;
                case MODEL_3C515:
                    data = 0x5151;  /* 3C515 */
                    break;
                case MODEL_3C905:
                    data = 0x9050;  /* 3C905 */
                    break;
                case MODEL_3C905B:
                    data = 0x9051;  /* 3C905B */
                    break;
                default:
                    data = 0x5090;
                    break;
                }
                trace_el3_read(off, data, size, win);
                return (uint32_t)data;
            }
            
            /* DMA status registers for bus-master models only */
            if (c->model != MODEL_3C509 && c->model != MODEL_3C509B) {
                if (off == 0x0C && size == 2) {
                    /* UpStatus */
                    data = c->up_status;
                    trace_el3_read(off, data, size, win);
                    return (uint32_t)data;
                } else if (off == 0x0A && size == 2) {
                    /* DownStatus */
                    data = c->down_status;
                    trace_el3_read(off, data, size, win);
                    return (uint32_t)data;
                }
            }
        }
        
        switch (size) {
        case 1:
            data = ((uint8_t *)c->windows[win])[off];
            break;
        case 2:
            data = c->windows[win][off / 2];
            break;
        case 4:
            if (off + 4 <= EL3_WINDOW_SIZE) {
                data = c->windows[win][off / 2] | 
                      ((uint32_t)c->windows[win][(off + 2) / 2] << 16);
            }
            break;
        }
        trace_el3_read(off, data, size, win);
        return (uint32_t)data;
    }
    
    qemu_log_mask(LOG_GUEST_ERROR, "el3_core: invalid register read win=%d off=0x%02x size=%d\n", 
                  win, off, size);
    return 0;
}

void el3_core_register_write(EL3Core *c, unsigned win, unsigned off, uint32_t val, unsigned size)
{
    MemTxResult result = MEMTX_DECODE_ERROR;
    
    /* Try variant-specific register first */
    if (c->ops && c->ops->reg_write) {
        result = c->ops->reg_write(c, win, off, size, val);
        if (result == MEMTX_OK) {
            return;
        }
    }
    
    /* Fall back to core registers */
    if (win < EL3_MAX_WINDOWS && off < EL3_WINDOW_SIZE) {
        trace_el3_write(off, val, size, win);
        
        /* Special handling for Window 1 TX registers */
        if (win == 1) {
            if (off == W1_TX_RX_FIFO) {
                /* TX FIFO write (Window 1 offset 0x00 on write) */
                if (c->tx_in_progress && c->tx_enabled) {
                    /* Accept data writes, track against current_tx_len */
                    size_t remaining = c->current_tx_len - c->current_tx_written;
                    size_t can_write = MIN(size, remaining);
                    /* Also ensure we don't overflow our buffer */
                    can_write = MIN(can_write, 1518 - c->current_tx_written);
                    /* Also ensure we don't overflow the FIFO */
                    can_write = MIN(can_write, el3_tx_fifo_free(c));
                    if (can_write > 0) {
                        uint8_t buf[4];
                        switch (size) {
                        case 1:
                            buf[0] = val & 0xFF;
                            el3_tx_fifo_write(c, buf, 1);
                            c->current_tx_written += 1;
                            break;
                        case 2:
                            buf[0] = val & 0xFF;
                            buf[1] = (val >> 8) & 0xFF;
                            el3_tx_fifo_write(c, buf, MIN(can_write, 2));
                            c->current_tx_written += MIN(can_write, 2);
                            break;
                        case 4:
                            buf[0] = val & 0xFF;
                            buf[1] = (val >> 8) & 0xFF;
                            buf[2] = (val >> 16) & 0xFF;
                            buf[3] = (val >> 24) & 0xFF;
                            el3_tx_fifo_write(c, buf, MIN(can_write, 4));
                            c->current_tx_written += MIN(can_write, 4);
                            break;
                        }
                        /* Update TX available status after write */
                        el3_update_tx_available(c);
                        /* Check if we can start transmission */
                        el3_check_tx_threshold(c);
                    }
                }
                return;
            } else if (off == W1_TX_STATUS && (size == 1 || size == 2)) {
                /* TX Status - Write-1-to-Clear (8-bit or 16-bit access) */
                uint16_t clear_mask = (size == 1) ? (val & 0xFF) : (val & 0xFFFF);
                c->tx_status &= ~clear_mask;
                el3_update_irq(c);
                return;
            }
        }
        
        /* Special handling for Window 7 registers */
        if (win == 7) {
            /* Product ID registers are read-only */
            if (off == W7_VENDOR_ID || off == W7_DEVICE_ID) {
                /* Ignore writes to read-only product ID registers */
                return;
            }
            
            /* DMA status registers (W1C) for bus-master models only */
            if (c->model != MODEL_3C509 && c->model != MODEL_3C509B) {
                if (off == 0x0C && size == 2) {
                    /* UpStatus - write-one-to-clear */
                    c->up_status &= ~(val & 0xFFFF);
                    /* Check if DMA_DONE should be cleared */
                    if (c->up_status == 0 && c->down_status == 0) {
                        c->status &= ~STAT_DMA_DONE;
                    }
                    el3_update_irq(c);
                    return;
                } else if (off == 0x0A && size == 2) {
                    /* DownStatus - write-one-to-clear */
                    c->down_status &= ~(val & 0xFFFF);
                    /* Check if DMA_DONE should be cleared */
                    if (c->up_status == 0 && c->down_status == 0) {
                        c->status &= ~STAT_DMA_DONE;
                    }
                    el3_update_irq(c);
                    return;
                }
            }
        }
        
        switch (size) {
        case 1:
            ((uint8_t *)c->windows[win])[off] = val & 0xFF;
            break;
        case 2:
            c->windows[win][off / 2] = val & 0xFFFF;
            break;
        case 4:
            if (off + 4 <= EL3_WINDOW_SIZE) {
                c->windows[win][off / 2] = val & 0xFFFF;
                c->windows[win][(off + 2) / 2] = (val >> 16) & 0xFFFF;
            }
            break;
        }
        
        /* Handle Window 4 NET_DIAG register for loopback mode */
        if (win == 4 && off == W4_NET_DIAG && size == 2) {
            /* Bit 5 controls internal loopback mode */
            c->internal_loopback = (val & NET_DIAG_INTERNAL_LB) ? true : false;
            /* Only allow write to writable bits (6, 5, 15) */
            uint16_t writable_mask = NET_DIAG_FD_ENABLE | NET_DIAG_STATS_ENABLE | NET_DIAG_INTERNAL_LB;
            uint16_t old_val = c->windows[4][W4_NET_DIAG >> 1];
            c->windows[4][W4_NET_DIAG >> 1] = (old_val & ~writable_mask) | (val & writable_mask);
            return;
        }
        
        /* Handle Window 0 EEPROM command register */
        if (win == 0 && off == W0_EEPROM_CMD && size == 2) {
            el3_eeprom_cmd(c, val);
            /* Don't store the command value in the register */
            return;
        }
        
        /* Sync multicast hash table for Window 3, offsets 0x00-0x07 */
        if (win == 3 && off < 8) {
            /* Update only the specific bytes that were written */
            if (size == 1) {
                c->mcast_hash[off] = val & 0xFF;
            } else if (size == 2 && off < 8) {
                c->mcast_hash[off] = val & 0xFF;
                if (off + 1 < 8) {
                    c->mcast_hash[off + 1] = (val >> 8) & 0xFF;
                }
            } else if (size == 4 && off < 8) {
                for (int i = 0; i < 4 && (off + i) < 8; i++) {
                    c->mcast_hash[off + i] = (val >> (i * 8)) & 0xFF;
                }
            }
        }
        
        return;
    }
    
    qemu_log_mask(LOG_GUEST_ERROR, "el3_core: invalid register write win=%d off=0x%02x val=0x%08x size=%d\n", 
                  win, off, val, size);
}

/* Multi-size FIFO operations */
void el3_fifo_init(EL3MultiSizeFifo *mf, uint32_t capacity, uint8_t width_mask)
{
    mf->capacity = capacity;
    mf->width_mask = width_mask;
    fifo8_create(&mf->fifo, capacity);
}

void el3_fifo_reset(EL3MultiSizeFifo *mf)
{
    fifo8_reset(&mf->fifo);
}

void el3_fifo_cleanup(EL3MultiSizeFifo *mf)
{
    fifo8_destroy(&mf->fifo);
}

uint32_t el3_fifo_read(EL3MultiSizeFifo *mf, unsigned size)
{
    uint32_t data = 0;
    
    if (fifo8_is_empty(&mf->fifo)) {
        return 0;
    }
    
    /* Check if this access size is supported */
    if (!(mf->width_mask & (1 << (size - 1)))) {
        size = 1;  /* Fall back to byte access */
    }
    
    switch (size) {
    case 1:
        if (fifo8_num_used(&mf->fifo) >= 1) {
            data = fifo8_pop(&mf->fifo);
        }
        break;
    case 2:
        if (fifo8_num_used(&mf->fifo) >= 2) {
            data = fifo8_pop(&mf->fifo);
            data |= fifo8_pop(&mf->fifo) << 8;
        }
        break;
    case 4:
        if (fifo8_num_used(&mf->fifo) >= 4) {
            data = fifo8_pop(&mf->fifo);
            data |= fifo8_pop(&mf->fifo) << 8;
            data |= fifo8_pop(&mf->fifo) << 16;
            data |= fifo8_pop(&mf->fifo) << 24;
        }
        break;
    }
    
    return data;
}

void el3_fifo_write(EL3MultiSizeFifo *mf, uint32_t data, unsigned size)
{
    /* Check if this access size is supported */
    if (!(mf->width_mask & (1 << (size - 1)))) {
        size = 1;  /* Fall back to byte access */
    }
    
    switch (size) {
    case 1:
        if (fifo8_num_free(&mf->fifo) >= 1) {
            fifo8_push(&mf->fifo, data & 0xFF);
        }
        break;
    case 2:
        if (fifo8_num_free(&mf->fifo) >= 2) {
            fifo8_push(&mf->fifo, data & 0xFF);
            fifo8_push(&mf->fifo, (data >> 8) & 0xFF);
        }
        break;
    case 4:
        if (fifo8_num_free(&mf->fifo) >= 4) {
            fifo8_push(&mf->fifo, data & 0xFF);
            fifo8_push(&mf->fifo, (data >> 8) & 0xFF);
            fifo8_push(&mf->fifo, (data >> 16) & 0xFF);
            fifo8_push(&mf->fifo, (data >> 24) & 0xFF);
        }
        break;
    }
}

bool el3_fifo_push_packet(EL3MultiSizeFifo *mf, const uint8_t *data, size_t len)
{
    /* Check if we have space for length header (2 bytes) + data */
    if (fifo8_num_free(&mf->fifo) < len + 2) {
        return false;
    }
    
    /* Push length header (little-endian) */
    fifo8_push(&mf->fifo, len & 0xFF);
    fifo8_push(&mf->fifo, (len >> 8) & 0xFF);
    
    /* Push packet data */
    for (size_t i = 0; i < len; i++) {
        fifo8_push(&mf->fifo, data[i]);
    }
    
    return true;
}

size_t el3_fifo_pop_packet(EL3MultiSizeFifo *mf, uint8_t *buf, size_t max_len)
{
    /* Check if we have at least a length header */
    if (fifo8_num_used(&mf->fifo) < 2) {
        return 0;
    }
    
    /* Peek at length header without consuming */
    uint8_t len_lo = fifo8_pop(&mf->fifo);
    uint8_t len_hi = fifo8_pop(&mf->fifo);
    size_t packet_len = len_lo | (len_hi << 8);
    
    /* Check if full packet is available */
    if (fifo8_num_used(&mf->fifo) < packet_len) {
        /* Put length header back - this is tricky with fifo8 */
        /* For now, we'll lose the header - proper implementation would need pushback */
        return 0;
    }
    
    /* Check buffer size */
    if (packet_len > max_len) {
        /* Skip the packet */
        for (size_t i = 0; i < packet_len; i++) {
            fifo8_pop(&mf->fifo);
        }
        return 0;
    }
    
    /* Copy packet data */
    for (size_t i = 0; i < packet_len; i++) {
        buf[i] = fifo8_pop(&mf->fifo);
    }
    
    return packet_len;
}

uint32_t el3_fifo_used(EL3MultiSizeFifo *mf)
{
    return fifo8_num_used(&mf->fifo);
}

uint32_t el3_fifo_free(EL3MultiSizeFifo *mf)
{
    return fifo8_num_free(&mf->fifo);
}

bool el3_fifo_is_empty(EL3MultiSizeFifo *mf)
{
    return fifo8_is_empty(&mf->fifo);
}

bool el3_fifo_is_full(EL3MultiSizeFifo *mf)
{
    return fifo8_is_full(&mf->fifo);
}

/* PCI DMA functions (3C59x) */
void el3_dma_init(EL3DMAEngine *dma, AddressSpace *as, void *opaque)
{
    memset(dma, 0, sizeof(*dma));
    dma->state = EL3_DMA_IDLE;
    dma->ring_size = EL3_DMA_RING_SIZE;
    dma->as = as;
    dma->opaque = opaque;
    
    /* Create bottom half handlers */
    dma->tx_bh = qemu_bh_new(el3_dma_tx_bh, dma);
    dma->rx_bh = qemu_bh_new(el3_dma_rx_bh, dma);
}

void el3_dma_cleanup(EL3DMAEngine *dma)
{
    if (dma->tx_bh) {
        qemu_bh_delete(dma->tx_bh);
        dma->tx_bh = NULL;
    }
    if (dma->rx_bh) {
        qemu_bh_delete(dma->rx_bh);
        dma->rx_bh = NULL;
    }
    if (dma->pending_frame) {
        g_free(dma->pending_frame);
        dma->pending_frame = NULL;
    }
}

void el3_dma_reset(EL3DMAEngine *dma)
{
    dma->state = EL3_DMA_IDLE;
    dma->enabled = false;
    dma->current_desc = 0;
    dma->transfer_len = 0;
    dma->buffer_addr = 0;
    memset(&dma->down_desc, 0, sizeof(dma->down_desc));
    memset(&dma->up_desc, 0, sizeof(dma->up_desc));
    
    /* Cancel any pending BH */
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
        dma->pending_len = 0;
        dma->pending_status = 0;
    }
}

void el3_dma_set_ring_base(EL3DMAEngine *dma, hwaddr addr, uint32_t size)
{
    dma->base_addr = addr;
    dma->ring_size = size > 0 ? size : EL3_DMA_RING_SIZE;
    dma->current_desc = 0;
}

bool el3_dma_start_download(EL3DMAEngine *dma)
{
    if (dma->state != EL3_DMA_IDLE || !dma->enabled) {
        return false;
    }
    
    /* Enable the DMA engine - actual operations happen in BH */
    dma->state = EL3_DMA_FETCHING_DESC;
    
    /* Schedule TX BH to handle descriptor fetching */
    qemu_bh_schedule(dma->tx_bh);
    
    return true;
}

bool el3_dma_start_upload(EL3DMAEngine *dma)
{
    if (dma->state != EL3_DMA_IDLE || !dma->enabled) {
        return false;
    }
    
    /* Enable the DMA engine - actual operations happen in BH */
    dma->state = EL3_DMA_FETCHING_DESC;
    
    /* Schedule RX BH to handle descriptor fetching */
    qemu_bh_schedule(dma->rx_bh);
    
    return true;
}

void el3_dma_kick(EL3DMAEngine *dma, bool is_tx)
{
    /* Schedule the appropriate BH handler */
    if (is_tx && dma->tx_bh) {
        qemu_bh_schedule(dma->tx_bh);
    } else if (!is_tx && dma->rx_bh) {
        qemu_bh_schedule(dma->rx_bh);
    }
}

bool el3_dma_is_idle(EL3DMAEngine *dma)
{
    return dma->state == EL3_DMA_IDLE;
}

uint32_t el3_dma_get_status(EL3DMAEngine *dma)
{
    uint32_t status = 0;
    
    if (dma->enabled) {
        status |= 0x00000001;  /* DMA enabled */
    }
    
    switch (dma->state) {
    case EL3_DMA_IDLE:
        break;
    case EL3_DMA_FETCHING_DESC:
    case EL3_DMA_TRANSFERRING:
    case EL3_DMA_UPDATING_DESC:
        status |= 0x00000002;  /* DMA busy */
        break;
    }
    
    return status;
}

/* DMA packet submission functions */
void el3_dma_tx_submit(EL3DMAEngine *dma, const uint8_t *data, size_t len)
{
    /* For TX, we don't need to buffer data - just kick the DMA engine */
    /* The BH will fetch the data from guest memory via descriptors */
    if (dma->enabled && dma->state == EL3_DMA_IDLE) {
        el3_dma_start_download(dma);
    }
}

void el3_dma_rx_submit(EL3DMAEngine *dma, const uint8_t *data, size_t len, uint32_t status)
{
    /* For RX, we need to buffer the incoming frame until we can DMA it */
    if (!dma->enabled) {
        return;
    }
    
    /* Free any existing pending frame */
    if (dma->pending_frame) {
        g_free(dma->pending_frame);
    }
    
    /* Copy the frame data */
    dma->pending_frame = g_malloc(len);
    memcpy(dma->pending_frame, data, len);
    dma->pending_len = len;
    dma->pending_status = status;
    
    /* Start upload DMA if idle */
    if (dma->state == EL3_DMA_IDLE) {
        el3_dma_start_upload(dma);
    }
}

/* Bottom Half handlers */
void el3_dma_tx_bh(void *opaque)
{
    EL3DMAEngine *dma = opaque;
    
    switch (dma->state) {
    case EL3_DMA_FETCHING_DESC:
        /* Fetch download descriptor */
        {
            hwaddr desc_addr = dma->base_addr + (dma->current_desc * sizeof(EL3DownDesc));
            
            if (address_space_read(dma->as, desc_addr, MEMTXATTRS_UNSPECIFIED,
                                  (uint8_t *)&dma->down_desc, sizeof(EL3DownDesc)) != MEMTX_OK) {
                dma->state = EL3_DMA_IDLE;
                return;
            }
            
            /* Convert from little-endian */
            dma->down_desc.next_desc = le32_to_cpu(dma->down_desc.next_desc);
            dma->down_desc.status = le32_to_cpu(dma->down_desc.status);
            dma->down_desc.addr = le32_to_cpu(dma->down_desc.addr);
            dma->down_desc.length = le32_to_cpu(dma->down_desc.length);
            
            dma->transfer_len = dma->down_desc.length & EL3_DESC_LENGTH_MASK;
            dma->buffer_addr = dma->down_desc.addr;
            dma->state = EL3_DMA_TRANSFERRING;
            
            /* Continue processing */
            qemu_bh_schedule(dma->tx_bh);
        }
        break;
        
    case EL3_DMA_TRANSFERRING:
        /* Perform the TX transfer */
        {
            /* For now, just mark as complete - full implementation would:
             * 1. Read data from guest buffer
             * 2. Submit to network
             * 3. Update descriptor status
             */
            dma->down_desc.status |= EL3_DESC_DOWN_COMPLETE | EL3_DESC_DMA_DONE;
            dma->state = EL3_DMA_UPDATING_DESC;
            
            /* Continue to update descriptor */
            qemu_bh_schedule(dma->tx_bh);
        }
        break;
        
    case EL3_DMA_UPDATING_DESC:
        /* Write back updated descriptor */
        {
            hwaddr desc_addr = dma->base_addr + (dma->current_desc * sizeof(EL3DownDesc));
            EL3DownDesc desc_le;
            
            /* Convert to little-endian */
            desc_le.next_desc = cpu_to_le32(dma->down_desc.next_desc);
            desc_le.status = cpu_to_le32(dma->down_desc.status);
            desc_le.addr = cpu_to_le32(dma->down_desc.addr);
            desc_le.length = cpu_to_le32(dma->down_desc.length);
            
            address_space_write(dma->as, desc_addr, MEMTXATTRS_UNSPECIFIED,
                              (uint8_t *)&desc_le, sizeof(EL3DownDesc));
            
            /* Advance to next descriptor or complete */
            dma->current_desc = (dma->current_desc + 1) % dma->ring_size;
            dma->state = EL3_DMA_IDLE;
            
            /* Generate interrupt if EL3_DESC_DMA_INDICATE is set */
            if (dma->down_desc.status & EL3_DESC_DMA_INDICATE) {
                /* Generate DOWN_COMPLETE interrupt for TX DMA */
                if (dma->opaque) {
                    EL3Core *c = (EL3Core *)dma->opaque;
                    c->status |= STAT_DOWN_COMPLETE | STAT_DMA_DONE;
                    if (c->status_enb & (STAT_DOWN_COMPLETE | STAT_DMA_DONE)) {
                        el3_update_irq(c);
                    }
                }
            }
        }
        break;
        
    default:
        dma->state = EL3_DMA_IDLE;
        break;
    }
}

void el3_dma_rx_bh(void *opaque)
{
    EL3DMAEngine *dma = opaque;
    
    switch (dma->state) {
    case EL3_DMA_FETCHING_DESC:
        /* Fetch upload descriptor */
        {
            hwaddr desc_addr = dma->base_addr + (dma->current_desc * sizeof(EL3UpDesc));
            
            if (address_space_read(dma->as, desc_addr, MEMTXATTRS_UNSPECIFIED,
                                  (uint8_t *)&dma->up_desc, sizeof(EL3UpDesc)) != MEMTX_OK) {
                dma->state = EL3_DMA_IDLE;
                return;
            }
            
            /* Convert from little-endian */
            dma->up_desc.next_desc = le32_to_cpu(dma->up_desc.next_desc);
            dma->up_desc.status = le32_to_cpu(dma->up_desc.status);
            dma->up_desc.addr = le32_to_cpu(dma->up_desc.addr);
            dma->up_desc.length = le32_to_cpu(dma->up_desc.length);
            
            dma->transfer_len = dma->up_desc.length & EL3_DESC_LENGTH_MASK;
            dma->buffer_addr = dma->up_desc.addr;
            dma->state = EL3_DMA_TRANSFERRING;
            
            /* Continue processing */
            qemu_bh_schedule(dma->rx_bh);
        }
        break;
        
    case EL3_DMA_TRANSFERRING:
        /* Perform the RX transfer */
        {
            if (dma->pending_frame && dma->pending_len > 0) {
                /* Transfer pending frame to guest memory */
                size_t transfer_size = MIN(dma->pending_len, dma->transfer_len);
                
                if (address_space_write(dma->as, dma->buffer_addr, MEMTXATTRS_UNSPECIFIED,
                                      dma->pending_frame, transfer_size) == MEMTX_OK) {
                    dma->up_desc.status |= EL3_DESC_UP_COMPLETE | EL3_DESC_DMA_DONE;
                    dma->up_desc.status |= (dma->pending_status << 16);
                    dma->up_desc.length = (dma->up_desc.length & ~EL3_DESC_LENGTH_MASK) | transfer_size;
                } else {
                    dma->up_desc.status |= EL3_DESC_UP_ERROR;
                }
                
                /* Free the pending frame */
                g_free(dma->pending_frame);
                dma->pending_frame = NULL;
                dma->pending_len = 0;
                dma->pending_status = 0;
            } else {
                dma->up_desc.status |= EL3_DESC_UP_ERROR;
            }
            
            dma->state = EL3_DMA_UPDATING_DESC;
            
            /* Continue to update descriptor */
            qemu_bh_schedule(dma->rx_bh);
        }
        break;
        
    case EL3_DMA_UPDATING_DESC:
        /* Write back updated descriptor */
        {
            hwaddr desc_addr = dma->base_addr + (dma->current_desc * sizeof(EL3UpDesc));
            EL3UpDesc desc_le;
            
            /* Convert to little-endian */
            desc_le.next_desc = cpu_to_le32(dma->up_desc.next_desc);
            desc_le.status = cpu_to_le32(dma->up_desc.status);
            desc_le.addr = cpu_to_le32(dma->up_desc.addr);
            desc_le.length = cpu_to_le32(dma->up_desc.length);
            
            address_space_write(dma->as, desc_addr, MEMTXATTRS_UNSPECIFIED,
                              (uint8_t *)&desc_le, sizeof(EL3UpDesc));
            
            /* Advance to next descriptor or complete */
            dma->current_desc = (dma->current_desc + 1) % dma->ring_size;
            dma->state = EL3_DMA_IDLE;
            
            /* Generate interrupt if EL3_DESC_DMA_INDICATE is set */
            if (dma->up_desc.status & EL3_DESC_DMA_INDICATE) {
                /* Generate UP_COMPLETE interrupt for RX DMA */
                if (dma->opaque) {
                    EL3Core *c = (EL3Core *)dma->opaque;
                    c->status |= STAT_UP_COMPLETE | STAT_DMA_DONE;
                    if (c->status_enb & (STAT_UP_COMPLETE | STAT_DMA_DONE)) {
                        el3_update_irq(c);
                    }
                }
            }
        }
        break;
        
    default:
        dma->state = EL3_DMA_IDLE;
        break;
    }
}
/* VMState descriptors for migration support */

/* Forward declarations for helper functions */
static int el3_core_post_load(void *opaque, int version_id);
static int el3_core_pre_save(void *opaque);

/* VMState for ID sequence state (ISA cards only) */
const VMStateDescription vmstate_el3_id_state = {
    .name = "el3_id_state",
    .version_id = 2,  /* Bump version for new fields */
    .minimum_version_id = 2,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(state, EL3IDState),      /* IDPortState enum */
        VMSTATE_UINT8(unlock_pos, EL3IDState),
        VMSTATE_UINT8(board_tag, EL3IDState),
        VMSTATE_UINT8(selected_tag, EL3IDState),
        VMSTATE_UINT32(product_id, EL3IDState),
        VMSTATE_UINT8(bit_pos, EL3IDState),
        VMSTATE_UINT16(id_port, EL3IDState),
        VMSTATE_END_OF_LIST()
    }
};

/* VMState for multi-size FIFO */
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

/* VMState for DMA engine (PCI cards only) */
const VMStateDescription vmstate_el3_dma_engine = {
    .name = "el3_dma_engine",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT64(base_addr, EL3DMAEngine),
        VMSTATE_UINT32(current_desc, EL3DMAEngine),
        VMSTATE_UINT32(ring_size, EL3DMAEngine),
        VMSTATE_UINT32(state, EL3DMAEngine),
        VMSTATE_BOOL(enabled, EL3DMAEngine),
        
        /* Current operation state */
        VMSTATE_UINT32(down_desc.next_desc, EL3DMAEngine),
        VMSTATE_UINT32(down_desc.status, EL3DMAEngine),
        VMSTATE_UINT32(down_desc.addr, EL3DMAEngine),
        VMSTATE_UINT32(down_desc.length, EL3DMAEngine),
        
        VMSTATE_UINT32(up_desc.next_desc, EL3DMAEngine),
        VMSTATE_UINT32(up_desc.status, EL3DMAEngine),
        VMSTATE_UINT32(up_desc.addr, EL3DMAEngine),
        VMSTATE_UINT32(up_desc.length, EL3DMAEngine),
        
        VMSTATE_UINT32(transfer_len, EL3DMAEngine),
        VMSTATE_UINT64(buffer_addr, EL3DMAEngine),
        
        /* Pending frame data */
        VMSTATE_VARRAY_UINT32_ALLOC(pending_frame, EL3DMAEngine, pending_len, 0,
                                   vmstate_info_uint8, uint8_t),
        VMSTATE_UINT32(pending_status, EL3DMAEngine),
        
        VMSTATE_END_OF_LIST()
    },
    .post_load = NULL,  /* BH handlers will be recreated by variant */
    .pre_save = NULL
};

/* Main VMState for EL3Core */
const VMStateDescription vmstate_el3_core = {
    .name = "el3_core",
    .version_id = 5,  /* Version 5 to include timer migration */
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        /* Model and capabilities */
        VMSTATE_UINT32(model, EL3Core),
        VMSTATE_BOOL(caps.has_mii, EL3Core),
        VMSTATE_BOOL(caps.has_bus_master, EL3Core),
        VMSTATE_BOOL(caps.has_full_duplex, EL3Core),
        VMSTATE_BOOL(caps.is_100mbit, EL3Core),
        VMSTATE_BOOL(caps.has_vlan_support, EL3Core),
        VMSTATE_UINT16(caps.ram_bytes, EL3Core),
        VMSTATE_UINT16(caps.tx_fifo_bytes, EL3Core),
        VMSTATE_UINT16(caps.rx_fifo_bytes, EL3Core),
        VMSTATE_UINT8(caps.max_windows, EL3Core),
        
        /* Register windows */
        VMSTATE_UINT16_2DARRAY(windows, EL3Core, EL3_MAX_WINDOWS, EL3_WINDOW_SIZE),
        VMSTATE_UINT8(current_window, EL3Core),
        
        /* EEPROM state */
        VMSTATE_UINT16_ARRAY(eeprom, EL3Core, 64),
        VMSTATE_UINT32(eeprom_state, EL3Core),
        VMSTATE_UINT8(eeprom_addr, EL3Core),
        VMSTATE_UINT16(eeprom_data, EL3Core),
        VMSTATE_INT64(eeprom_due_ns, EL3Core),
        VMSTATE_UINT64(eeprom_latency_ns, EL3Core),
        
        /* Status and interrupts */
        VMSTATE_UINT16(command, EL3Core),
        VMSTATE_UINT16(status, EL3Core),
        VMSTATE_UINT16(int_status, EL3Core),
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
        
        /* NIC configuration */
        VMSTATE_NIC(nic, EL3Core),
        
        /* ISA ID sequence (conditional) */
        VMSTATE_STRUCT(id_state, EL3Core, 0, vmstate_el3_id_state, EL3IDState),
        
        /* Statistics */
        VMSTATE_BOOL(stats_enabled, EL3Core),
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
        
        /* 3Com-specific TX error counters */
        VMSTATE_UINT8(stats.tx_underruns, EL3Core),
        VMSTATE_UINT8(stats.tx_oversize, EL3Core),
        
        /* Statistics freeze-latch functionality (version 3+) */
        VMSTATE_BOOL_V(stats_frozen, EL3Core, 3),
        VMSTATE_UINT8_V(stats_snapshot.tx_carrier_errors, EL3Core, 3),
        VMSTATE_UINT8_V(stats_snapshot.tx_heartbeat_errors, EL3Core, 3),
        VMSTATE_UINT8_V(stats_snapshot.tx_mult_collisions, EL3Core, 3),
        VMSTATE_UINT8_V(stats_snapshot.tx_single_collisions, EL3Core, 3),
        VMSTATE_UINT8_V(stats_snapshot.tx_late_collisions, EL3Core, 3),
        VMSTATE_UINT8_V(stats_snapshot.rx_overruns, EL3Core, 3),
        VMSTATE_UINT8_V(stats_snapshot.tx_frames_ok, EL3Core, 3),
        VMSTATE_UINT8_V(stats_snapshot.rx_frames_ok, EL3Core, 3),
        VMSTATE_UINT8_V(stats_snapshot.tx_deferrals, EL3Core, 3),
        VMSTATE_UINT16_V(stats_snapshot.rx_bytes_ok, EL3Core, 3),
        VMSTATE_UINT16_V(stats_snapshot.tx_bytes_ok, EL3Core, 3),
        VMSTATE_UINT8_V(stats_snapshot.tx_underruns, EL3Core, 3),
        VMSTATE_UINT8_V(stats_snapshot.tx_oversize, EL3Core, 3),
        
        /* RX packet queue state (version 4+) */
        VMSTATE_UINT8_ARRAY_V(rx_data, EL3Core, 4096, 4),
        VMSTATE_UINT16_V(rx_data_write_ptr, EL3Core, 4),
        VMSTATE_UINT16_V(rx_data_used, EL3Core, 4),
        VMSTATE_UINT8_V(rx_packet_head, EL3Core, 4),
        VMSTATE_UINT8_V(rx_packet_tail, EL3Core, 4),
        VMSTATE_UINT8_V(rx_packet_count, EL3Core, 4),
        /* Note: rx_packets array needs custom handling for migration */
        
        /* TX FIFO state (version 4+) */
        VMSTATE_UINT8_ARRAY_V(tx_fifo, EL3Core, 4096, 4),
        VMSTATE_UINT16_V(tx_fifo_write_ptr, EL3Core, 4),
        VMSTATE_UINT16_V(tx_fifo_read_ptr, EL3Core, 4),
        VMSTATE_UINT16_V(tx_fifo_used, EL3Core, 4),
        
        /* TX state (version 4+) */
        VMSTATE_BOOL_V(tx_in_progress, EL3Core, 4),
        VMSTATE_UINT16_V(current_tx_len, EL3Core, 4),
        VMSTATE_UINT16_V(current_tx_written, EL3Core, 4),
        VMSTATE_UINT8_V(tx_status, EL3Core, 4),
        VMSTATE_BOOL_V(internal_loopback, EL3Core, 4),
        
        /* Timers - critical for migration safety */
        VMSTATE_TIMER(cmd_timer, EL3Core),
        VMSTATE_TIMER(eeprom_timer, EL3Core),
        
        VMSTATE_END_OF_LIST()
    },
    .post_load = el3_core_post_load,
    .pre_save = el3_core_pre_save
};

/* Migration helper functions */
static int el3_core_post_load(void *opaque, int version_id)
{
    EL3Core *c = opaque;
    
    /* Recreate timers */
    if (!c->cmd_timer) {
        c->cmd_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, el3_cmd_complete, c);
    }
    if (!c->eeprom_timer) {
        c->eeprom_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, el3_eeprom_timer_cb, c);
    }
    
    /* Restart EEPROM timer if needed */
    if (c->eeprom_state == EEPROM_BUSY && c->eeprom_due_ns > 0) {
        timer_mod_ns(c->eeprom_timer, c->eeprom_due_ns);
    }
    
    /* Initialize new fields for older migration versions */
    if (version_id < 3) {
        /* Initialize freeze-latch functionality */
        c->stats_frozen = false;
        memset(&c->stats_snapshot, 0, sizeof(c->stats_snapshot));
    }
    
    /* Update IRQ state */
    el3_update_irq(c);
    
    return 0;
}

static int el3_core_pre_save(void *opaque)
{
    EL3Core *c = opaque;
    
    /* Save EEPROM timer state */
    if (c->eeprom_timer && timer_pending(c->eeprom_timer)) {
        c->eeprom_due_ns = timer_expire_time_ns(c->eeprom_timer);
    } else {
        c->eeprom_due_ns = 0;
    }
    
    return 0;
}