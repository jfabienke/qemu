#include "qemu/osdep.h"
#include "hw/net/el3_core.h"
#include "hw/net/3c509-perf.h"
#include "qemu/timer.h"
#include "qemu/sockets.h"
#include "qapi/error.h"
#include "qapi/qmp/qdict.h"
#include "qapi/qmp/qnum.h"
#include "qapi/qmp/qstring.h"
#include "qemu/log.h"
#include "trace.h"

/* Trace points */
#define TRACE_PERF_START() trace_el3_perf_start()
#define TRACE_PERF_STOP() trace_el3_perf_stop()
#define TRACE_PERF_RESET() trace_el3_perf_reset()
#define TRACE_PERF_SAMPLE() trace_el3_perf_sample()
#define TRACE_PERF_IRQ_RATE(rate) trace_el3_perf_irq_rate(rate)
#define TRACE_PERF_FIFO_UTIL(tx_util, rx_util) trace_el3_perf_fifo_util(tx_util, rx_util)
#define TRACE_PERF_PKT_RATE(tx_rate, rx_rate) trace_el3_perf_pkt_rate(tx_rate, rx_rate)
#define TRACE_PERF_THROUGHPUT(tx_bps, rx_bps) trace_el3_perf_throughput(tx_bps, rx_bps)
#define TRACE_PERF_ERROR_RATE(tx_errors, rx_errors) trace_el3_perf_error_rate(tx_errors, rx_errors)
#define TRACE_PERF_LATENCY(min_lat, max_lat, avg_lat) trace_el3_perf_latency(min_lat, max_lat, avg_lat)
#define TRACE_PERF_CMD_TIMING(cmd, time) trace_el3_perf_cmd_timing(cmd, time)
#define TRACE_PERF_DMA_EFFICIENCY(tx_eff, rx_eff) trace_el3_perf_dma_efficiency(tx_eff, rx_eff)

/* Performance monitoring state */
typedef struct {
    bool enabled;
    int64_t start_time_ns;
    int64_t last_sample_time_ns;
    
    /* Packet counters */
    uint64_t tx_packets;
    uint64_t rx_packets;
    uint64_t tx_bytes;
    uint64_t rx_bytes;
    uint64_t tx_errors;
    uint64_t rx_errors;
    
    /* Interrupt counters */
    uint64_t irq_count;
    
    /* FIFO statistics */
    uint64_t tx_fifo_max_used;
    uint64_t rx_fifo_max_used;
    uint64_t tx_fifo_samples;
    uint64_t rx_fifo_samples;
    
    /* Timing analysis */
    uint64_t pkt_processing_time_ns;
    uint64_t pkt_processing_count;
    uint64_t min_latency_ns;
    uint64_t max_latency_ns;
    uint64_t total_latency_ns;
    
    /* Command timing */
    uint64_t cmd_start_time_ns[32]; /* Track start time for each command */
    
    /* DMA efficiency */
    uint64_t dma_tx_bytes;
    uint64_t dma_rx_bytes;
    uint64_t dma_tx_expected;
    uint64_t dma_rx_expected;
    
    /* Ring buffer diagnostics */
    uint64_t rx_ring_max_depth;
    uint64_t tx_ring_max_depth;
    
    /* Resource utilization */
    uint64_t cpu_time_ns;
    uint64_t mem_bandwidth_bytes;
    
    /* Historical data */
    uint64_t samples_collected;
} EL3PerfState;

/* Initialize performance monitoring */
void el3_perf_init(EL3Core *c)
{
    c->perf_state = g_malloc0(sizeof(EL3PerfState));
    c->perf_state->enabled = false;
    c->perf_state->min_latency_ns = UINT64_MAX;
    
    TRACE_PERF_START();
}

/* Enable performance monitoring */
void el3_perf_enable(EL3Core *c)
{
    if (!c->perf_state) {
        el3_perf_init(c);
    }
    
    c->perf_state->enabled = true;
    c->perf_state->start_time_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    c->perf_state->last_sample_time_ns = c->perf_state->start_time_ns;
    
    TRACE_PERF_START();
}

/* Disable performance monitoring */
void el3_perf_disable(EL3Core *c)
{
    if (c->perf_state) {
        c->perf_state->enabled = false;
    }
    
    TRACE_PERF_STOP();
}

/* Reset performance counters */
void el3_perf_reset(EL3Core *c)
{
    if (!c->perf_state) {
        return;
    }
    
    memset(c->perf_state, 0, sizeof(EL3PerfState));
    c->perf_state->min_latency_ns = UINT64_MAX;
    
    TRACE_PERF_RESET();
}

/* Sample performance data */
static void el3_perf_sample(EL3Core *c)
{
    if (!c->perf_state || !c->perf_state->enabled) {
        return;
    }
    
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t interval = now - c->perf_state->last_sample_time_ns;
    
    if (interval < 1000000000) { /* Sample every second */
        return;
    }
    
    c->perf_state->last_sample_time_ns = now;
    c->perf_state->samples_collected++;
    
    /* Calculate rates */
    uint64_t tx_pkt_rate = (c->perf_state->tx_packets * 1000000000) / interval;
    uint64_t rx_pkt_rate = (c->perf_state->rx_packets * 1000000000) / interval;
    
    uint64_t tx_bps = (c->perf_state->tx_bytes * 8 * 1000000000) / interval;
    uint64_t rx_bps = (c->perf_state->rx_bytes * 8 * 1000000000) / interval;
    
    /* Calculate FIFO utilization */
    uint64_t tx_util = 0;
    uint64_t rx_util = 0;
    
    if (c->perf_state->tx_fifo_samples > 0) {
        tx_util = (c->perf_state->tx_fifo_max_used * 100) / 
                  (c->caps.tx_fifo_bytes * c->perf_state->tx_fifo_samples);
    }
    
    if (c->perf_state->rx_fifo_samples > 0) {
        rx_util = (c->perf_state->rx_fifo_max_used * 100) / 
                  (c->caps.rx_fifo_bytes * c->perf_state->rx_fifo_samples);
    }
    
    /* Calculate interrupt rate */
    uint64_t irq_rate = (c->perf_state->irq_count * 1000000000) / interval;
    
    /* Calculate error rates */
    uint64_t tx_error_rate = 0;
    uint64_t rx_error_rate = 0;
    
    if (c->perf_state->tx_packets > 0) {
        tx_error_rate = (c->perf_state->tx_errors * 1000000) / c->perf_state->tx_packets;
    }
    
    if (c->perf_state->rx_packets > 0) {
        rx_error_rate = (c->perf_state->rx_errors * 1000000) / c->perf_state->rx_packets;
    }
    
    /* Calculate latency statistics */
    uint64_t avg_latency = 0;
    if (c->perf_state->pkt_processing_count > 0) {
        avg_latency = c->perf_state->total_latency_ns / c->perf_state->pkt_processing_count;
    }
    
    /* Calculate DMA efficiency */
    uint64_t tx_dma_eff = 0;
    uint64_t rx_dma_eff = 0;
    
    if (c->perf_state->dma_tx_expected > 0) {
        tx_dma_eff = (c->perf_state->dma_tx_bytes * 100) / c->perf_state->dma_tx_expected;
    }
    
    if (c->perf_state->dma_rx_expected > 0) {
        rx_dma_eff = (c->perf_state->dma_rx_bytes * 100) / c->perf_state->dma_rx_expected;
    }
    
    /* Trace the sampled data */
    TRACE_PERF_SAMPLE();
    TRACE_PERF_IRQ_RATE(irq_rate);
    TRACE_PERF_FIFO_UTIL(tx_util, rx_util);
    TRACE_PERF_PKT_RATE(tx_pkt_rate, rx_pkt_rate);
    TRACE_PERF_THROUGHPUT(tx_bps, rx_bps);
    TRACE_PERF_ERROR_RATE(tx_error_rate, rx_error_rate);
    TRACE_PERF_LATENCY(c->perf_state->min_latency_ns, 
                       c->perf_state->max_latency_ns, 
                       avg_latency);
    TRACE_PERF_DMA_EFFICIENCY(tx_dma_eff, rx_dma_eff);
    
    /* Reset counters for next sample */
    c->perf_state->tx_packets = 0;
    c->perf_state->rx_packets = 0;
    c->perf_state->tx_bytes = 0;
    c->perf_state->rx_bytes = 0;
    c->perf_state->tx_errors = 0;
    c->perf_state->rx_errors = 0;
    c->perf_state->irq_count = 0;
    c->perf_state->tx_fifo_max_used = 0;
    c->perf_state->rx_fifo_max_used = 0;
    c->perf_state->tx_fifo_samples = 0;
    c->perf_state->rx_fifo_samples = 0;
    c->perf_state->pkt_processing_time_ns = 0;
    c->perf_state->pkt_processing_count = 0;
    c->perf_state->min_latency_ns = UINT64_MAX;
    c->perf_state->max_latency_ns = 0;
    c->perf_state->total_latency_ns = 0;
    c->perf_state->dma_tx_bytes = 0;
    c->perf_state->dma_rx_bytes = 0;
    c->perf_state->dma_tx_expected = 0;
    c->perf_state->dma_rx_expected = 0;
}

/* Record packet transmission */
void el3_perf_record_tx(EL3Core *c, size_t len, bool error)
{
    if (!c->perf_state || !c->perf_state->enabled) {
        return;
    }
    
    c->perf_state->tx_packets++;
    c->perf_state->tx_bytes += len;
    
    if (error) {
        c->perf_state->tx_errors++;
    }
    
    el3_perf_sample(c);
}

/* Record packet reception */
void el3_perf_record_rx(EL3Core *c, size_t len, bool error)
{
    if (!c->perf_state || !c->perf_state->enabled) {
        return;
    }
    
    c->perf_state->rx_packets++;
    c->perf_state->rx_bytes += len;
    
    if (error) {
        c->perf_state->rx_errors++;
    }
    
    el3_perf_sample(c);
}

/* Record interrupt */
void el3_perf_record_irq(EL3Core *c)
{
    if (!c->perf_state || !c->perf_state->enabled) {
        return;
    }
    
    c->perf_state->irq_count++;
}

/* Record FIFO usage */
void el3_perf_record_fifo_usage(EL3Core *c, uint16_t tx_used, uint16_t rx_used)
{
    if (!c->perf_state || !c->perf_state->enabled) {
        return;
    }
    
    if (tx_used > c->perf_state->tx_fifo_max_used) {
        c->perf_state->tx_fifo_max_used = tx_used;
    }
    
    if (rx_used > c->perf_state->rx_fifo_max_used) {
        c->perf_state->rx_fifo_max_used = rx_used;
    }
    
    c->perf_state->tx_fifo_samples++;
    c->perf_state->rx_fifo_samples++;
}

/* Record packet processing time */
void el3_perf_record_pkt_processing(EL3Core *c, uint64_t start_time_ns, uint64_t end_time_ns)
{
    if (!c->perf_state || !c->perf_state->enabled) {
        return;
    }
    
    uint64_t processing_time = end_time_ns - start_time_ns;
    c->perf_state->pkt_processing_time_ns += processing_time;
    c->perf_state->pkt_processing_count++;
    
    if (processing_time < c->perf_state->min_latency_ns) {
        c->perf_state->min_latency_ns = processing_time;
    }
    
    if (processing_time > c->perf_state->max_latency_ns) {
        c->perf_state->max_latency_ns = processing_time;
    }
    
    c->perf_state->total_latency_ns += processing_time;
}

/* Record command start */
void el3_perf_record_cmd_start(EL3Core *c, uint16_t cmd)
{
    if (!c->perf_state || !c->perf_state->enabled) {
        return;
    }
    
    uint8_t cmd_index = (cmd >> 11) & 0x1F;
    c->perf_state->cmd_start_time_ns[cmd_index] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

/* Record command completion */
void el3_perf_record_cmd_complete(EL3Core *c, uint16_t cmd)
{
    if (!c->perf_state || !c->perf_state->enabled) {
        return;
    }
    
    uint8_t cmd_index = (cmd >> 11) & 0x1F;
    if (c->perf_state->cmd_start_time_ns[cmd_index] != 0) {
        uint64_t cmd_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - 
                            c->perf_state->cmd_start_time_ns[cmd_index];
        TRACE_PERF_CMD_TIMING(cmd, cmd_time);
        c->perf_state->cmd_start_time_ns[cmd_index] = 0;
    }
}

/* Record DMA operation */
void el3_perf_record_dma(EL3Core *c, bool is_tx, size_t bytes, size_t expected)
{
    if (!c->perf_state || !c->perf_state->enabled) {
        return;
    }
    
    if (is_tx) {
        c->perf_state->dma_tx_bytes += bytes;
        c->perf_state->dma_tx_expected += expected;
    } else {
        c->perf_state->dma_rx_bytes += bytes;
        c->perf_state->dma_rx_expected += expected;
    }
}

/* Record ring buffer depth */
void el3_perf_record_ring_depth(EL3Core *c, bool is_tx, size_t depth)
{
    if (!c->perf_state || !c->perf_state->enabled) {
        return;
    }
    
    if (is_tx) {
        if (depth > c->perf_state->tx_ring_max_depth) {
            c->perf_state->tx_ring_max_depth = depth;
        }
    } else {
        if (depth > c->perf_state->rx_ring_max_depth) {
            c->perf_state->rx_ring_max_depth = depth;
        }
    }
}

/* Record CPU time */
void el3_perf_record_cpu_time(EL3Core *c, uint64_t time_ns)
{
    if (!c->perf_state || !c->perf_state->enabled) {
        return;
    }
    
    c->perf_state->cpu_time_ns += time_ns;
}

/* Record memory bandwidth */
void el3_perf_record_mem_bandwidth(EL3Core *c, size_t bytes)
{
    if (!c->perf_state || !c->perf_state->enabled) {
        return;
    }
    
    c->perf_state->mem_bandwidth_bytes += bytes;
}

/* QMP command handlers */
void qmp_el3_perf_enable(Error **errp)
{
    /* This would need to be implemented with device lookup */
    error_setg(errp, "Not implemented");
}

void qmp_el3_perf_disable(Error **errp)
{
    /* This would need to be implemented with device lookup */
    error_setg(errp, "Not implemented");
}

void qmp_el3_perf_reset(Error **errp)
{
    /* This would need to be implemented with device lookup */
    error_setg(errp, "Not implemented");
}

/* Query performance data */
 perf_info(Error **errp)
{
    /* This would need to be implemented with device lookup */
    error_setg(errp, "Not implemented");
    return NULL;
}

/* Debug register interface */
uint32_t el3_perf_debug_read(EL3Core *c, unsigned reg, unsigned size)
{
    if (!c->perf_state) {
        return 0;
    }
    
    switch (reg) {
    case 0x00: /* TX packet count (low) */
        return c->perf_state->tx_packets & 0xFFFF;
    case 0x02: /* TX packet count (high) */
        return (c->perf_state->tx_packets >> 16) & 0xFFFF;
    case 0x04: /* RX packet count (low) */
        return c->perf_state->rx_packets & 0xFFFF;
    case 0x06: /* RX packet count (high) */
        return (c->perf_state->rx_packets >> 16) & 0xFFFF;
    case 0x08: /* TX bytes (low) */
        return c->perf_state->tx_bytes & 0xFFFF;
    case 0x0A: /* TX bytes (high) */
        return (c->perf_state->tx_bytes >> 16) & 0xFFFF;
    case 0x0C: /* RX bytes (low) */
        return c->perf_state->rx_bytes & 0xFFFF;
    case 0x0E: /* RX bytes (high) */
        return (c->perf_state->rx_bytes >> 16) & 0xFFFF;
    case 0x10: /* TX errors */
        return c->perf_state->tx_errors & 0xFFFF;
    case 0x12: /* RX errors */
        return c->perf_state->rx_errors & 0xFFFF;
    case 0x14: /* IRQ count */
        return c->perf_state->irq_count & 0xFFFF;
    case 0x16: /* FIFO utilization */
        {
            uint32_t tx_util = 0;
            uint32_t rx_util = 0;
            
            if (c->perf_state->tx_fifo_samples > 0) {
                tx_util = (c->perf_state->tx_fifo_max_used * 100) / 
                          (c->caps.tx_fifo_bytes * c->perf_state->tx_fifo_samples);
            }
            
            if (c->perf_state->rx_fifo_samples > 0) {
                rx_util = (c->perf_state->rx_fifo_max_used * 100) / 
                          (c->caps.rx_fifo_bytes * c->perf_state->rx_fifo_samples);
            }
            
            return (rx_util << 8) | tx_util;
        }
    default:
        return 0;
    }
}

void el3_perf_debug_write(EL3Core *c, unsigned reg, uint32_t val, unsigned size)
{
    switch (reg) {
    case 0x00: /* Control register */
        if (val & 0x01) {
            el3_perf_enable(c);
        } else {
            el3_perf_disable(c);
        }
        
        if (val & 0x02) {
            el3_perf_reset(c);
        }
        break;
    default:
        break;
    }
}

/* Integration with existing systems */

/* Hook into packet RX path */
ssize_t el3_perf_receive(NetClientState *nc, const uint8_t *buf, size_t size)
{
    EL3Core *c = qemu_get_nic_opaque(nc);
    uint64_t start_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    
    ssize_t result = el3_core_receive(nc, buf, size);
    
    uint64_t end_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    el3_perf_record_pkt_processing(c, start_time, end_time);
    el3_perf_record_rx(c, size, result < 0);
    
    return result;
}

/* Hook into packet TX path */
void el3_perf_tx_submit(EL3Core *c, const uint8_t *buf, size_t len)
{
    uint64_t start_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    
    el3_core_tx_submit(c, buf, len);
    
    uint64_t end_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    el3_perf_record_pkt_processing(c, start_time, end_time);
    el3_perf_record_tx(c, len, false);
}

/* Hook into interrupt system */
void el3_perf_update_irq(EL3Core *c)
{
    el3_perf_record_irq(c);
    el3_update_irq(c);
}

/* Hook into FIFO operations */
void el3_perf_update_tx_fifo(EL3Core *c, uint16_t used)
{
    el3_perf_record_fifo_usage(c, used, c->rx_used);
    c->tx_used = used;
}

void el3_perf_update_rx_fifo(EL3Core *c, uint16_t used)
{
    el3_perf_record_fifo_usage(c, c->tx_used, used);
    c->rx_used = used;
}

/* Hook into command processing */
void el3_perf_process_command(EL3Core *c, uint16_t cmd)
{
    el3_perf_record_cmd_start(c, cmd);
    el3_core_process_command(c, cmd);
    el3_perf_record_cmd_complete(c, cmd);
}

/* Hook into DMA operations */
void el3_perf_dma_tx_submit(EL3DMAEngine *dma, const uint8_t *data, size_t len)
{
    if (dma->opaque) {
        EL3Core *c = (EL3Core *)dma->opaque;
        el3_perf_record_dma(c, true, len, len);
    }
    el3_dma_tx_submit(dma, data, len);
}

void el3_perf_dma_rx_submit(EL3DMAEngine *dma, const uint8_t *data, size_t len, uint32_t status)
{
    if (dma->opaque) {
        EL3Core *c = (EL3Core *)dma->opaque;
        el3_perf_record_dma(c, false, len, len);
    }
    el3_dma_rx_submit(dma, data, len, status);
}

/* VMState for performance monitoring */
const VMStateDescription vmstate_el3_perf = {
    .name = "el3_perf",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_BOOL(enabled, EL3PerfState),
        VMSTATE_INT64(start_time_ns, EL3PerfState),
        VMSTATE_INT64(last_sample_time_ns, EL3PerfState),
        VMSTATE_UINT64(tx_packets, EL3PerfState),
        VMSTATE_UINT64(rx_packets, EL3PerfState),
        VMSTATE_UINT64(tx_bytes, EL3PerfState),
        VMSTATE_UINT64(rx_bytes, EL3PerfState),
        VMSTATE_UINT64(tx_errors, EL3PerfState),
        VMSTATE_UINT64(rx_errors, EL3PerfState),
        VMSTATE_UINT64(irq_count, EL3PerfState),
        VMSTATE_UINT64(tx_fifo_max_used, EL3PerfState),
        VMSTATE_UINT64(rx_fifo_max_used, EL3PerfState),
        VMSTATE_UINT64(tx_fifo_samples, EL3PerfState),
        VMSTATE_UINT64(rx_fifo_samples, EL3PerfState),
        VMSTATE_UINT64(pkt_processing_time_ns, EL3PerfState),
        VMSTATE_UINT64(pkt_processing_count, EL3PerfState),
        VMSTATE_UINT64(min_latency_ns, EL3PerfState),
        VMSTATE_UINT64(max_latency_ns, EL3PerfState),
        VMSTATE_UINT64(total_latency_ns, EL3PerfState),
        VMSTATE_UINT64_ARRAY(cmd_start_time_ns, EL3PerfState, 32),
        VMSTATE_UINT64(dma_tx_bytes, EL3PerfState),
        VMSTATE_UINT64(dma_rx_bytes, EL3PerfState),
        VMSTATE_UINT64(dma_tx_expected, EL3PerfState),
        VMSTATE_UINT64(dma_rx_expected, EL3PerfState),
        VMSTATE_UINT64(rx_ring_max_depth, EL3PerfState),
        VMSTATE_UINT64(tx_ring_max_depth, EL3PerfState),
        VMSTATE_UINT64(cpu_time_ns, EL3PerfState),
        VMSTATE_UINT64(mem_bandwidth_bytes, EL3PerfState),
        VMSTATE_UINT64(samples_collected, EL3PerfState),
        VMSTATE_END_OF_LIST()
    }
};

/* Integration with core VMState */
const VMStateDescription vmstate_el3_core_with_perf = {
    .name = "el3_core_with_perf",
    .version_id = 6,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        /* Include all core fields */
        VMSTATE_STRUCT_POINTER(perf_state, EL3Core, vmstate_el3_perf, EL3PerfState),
        VMSTATE_END_OF_LIST()
    }
};

/* Cleanup function */
void el3_perf_cleanup(EL3Core *c)
{
    if (c->perf_state) {
        g_free(c->perf_state);
        c->perf_state = NULL;
    }
}