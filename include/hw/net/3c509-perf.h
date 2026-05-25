#ifndef HW_NET_3C509_PERF_H
#define HW_NET_3C509_PERF_H

#include "qemu/osdep.h"

/* Forward declaration to avoid circular dependency */
typedef struct EL3Core EL3Core;
#include "qemu/timer.h"
#include "migration/vmstate.h"
#include "qapi/error.h"

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

/* Performance interfaces */
void el3_perf_init(EL3Core *c);
void el3_perf_enable(EL3Core *c);
void el3_perf_disable(EL3Core *c);
void el3_perf_reset(EL3Core *c);

/* Metric collection functions */
void el3_perf_record_tx(EL3Core *c, size_t len, bool error);
void el3_perf_record_rx(EL3Core *c, size_t len, bool error);
void el3_perf_record_irq(EL3Core *c);
void el3_perf_record_fifo_usage(EL3Core *c, uint16_t tx_used, uint16_t rx_used);
void el3_perf_record_pkt_processing(EL3Core *c, uint64_t start_time_ns, uint64_t end_time_ns);
void el3_perf_record_cmd_start(EL3Core *c, uint16_t cmd);
void el3_perf_record_cmd_complete(EL3Core *c, uint16_t cmd);
void el3_perf_record_dma(EL3Core *c, bool is_tx, size_t bytes, size_t expected);
void el3_perf_record_ring_depth(EL3Core *c, bool is_tx, size_t depth);
void el3_perf_record_cpu_time(EL3Core *c, uint64_t time_ns);
void el3_perf_record_mem_bandwidth(EL3Core *c, size_t bytes);

/* Diagnostic query functions */
uint32_t el3_perf_debug_read(EL3Core *c, unsigned reg, unsigned size);
void el3_perf_debug_write(EL3Core *c, unsigned reg, uint32_t val, unsigned size);

/* Integration hooks */
ssize_t el3_perf_receive(NetClientState *nc, const uint8_t *buf, size_t size);
void el3_perf_tx_submit(EL3Core *c, const uint8_t *buf, size_t len);
void el3_perf_update_irq(EL3Core *c);
void el3_perf_update_tx_fifo(EL3Core *c, uint16_t used);
void el3_perf_update_rx_fifo(EL3Core *c, uint16_t used);
void el3_perf_process_command(EL3Core *c, uint16_t cmd);
void el3_perf_dma_tx_submit(EL3DMAEngine *dma, const uint8_t *data, size_t len);
void el3_perf_dma_rx_submit(EL3DMAEngine *dma, const uint8_t *data, size_t len, uint32_t status);

/* QMP command handlers */
void qmp_el3_perf_enable(Error **errp);
void qmp_el3_perf_disable(Error **errp);
void qmp_el3_perf_reset(Error **errp);

/* VMState descriptor */
extern const VMStateDescription vmstate_el3_perf;

/* Cleanup function */
void el3_perf_cleanup(EL3Core *c);

#endif /* HW_NET_3C509_PERF_H */