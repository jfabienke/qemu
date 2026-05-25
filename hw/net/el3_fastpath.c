#include "qemu/osdep.h"
#include "hw/net/el3_core.h"
#include "net/net.h"
#include "net/eth.h"
#include "qemu/iov.h"
#include "qemu/main-loop.h"
#include "qemu/timer.h"
#include "exec/memory.h"
#include "hw/pci/pci.h"

/* Performance counters */
static uint64_t tx_batch_count;
static uint64_t rx_batch_count;
static uint64_t tx_packets_processed;
static uint64_t rx_packets_processed;
static uint64_t tx_dma_coalesced;
static uint64_t rx_dma_coalesced;
static uint64_t tx_cache_hits;
static uint64_t rx_cache_hits;

/* Auto-tuning parameters */
static int current_tx_batch_size = 4;
static int current_rx_batch_size = 4;
static int64_t last_tx_time;
static int64_t last_rx_time;

/* Prefetch distance for descriptors */
#define PREFETCH_DISTANCE 2

/* Cache line size for alignment */
#define CACHE_LINE_SIZE 64

/* Bounce buffer size for small transfers */
#define BOUNCE_BUFFER_SIZE 2048

/* Align structure to cache line */
typedef struct EL3CachedDesc {
    union {
        EL3DownDesc down;
        EL3UpDesc up;
    } desc;
    uint8_t data[BOUNCE_BUFFER_SIZE];
    bool valid;
    bool in_use;
} __attribute__((aligned(CACHE_LINE_SIZE))) EL3CachedDesc;

/* Descriptor cache pool */
static EL3CachedDesc desc_cache[32];
static int desc_cache_index;

/* Initialize descriptor cache */
static void el3_desc_cache_init(void)
{
    memset(desc_cache, 0, sizeof(desc_cache));
    desc_cache_index = 0;
}

/* Get cached descriptor */
static EL3CachedDesc *el3_get_cached_desc(void)
{
    EL3CachedDesc *desc = &desc_cache[desc_cache_index];
    desc_cache_index = (desc_cache_index + 1) & 31;
    return desc;
}

/* Prefetch next descriptors */
static inline void el3_prefetch_next_desc(EL3Core *c, hwaddr addr, int count)
{
    int i;
    for (i = 0; i < count && i < 32; i++) {
        __builtin_prefetch(&desc_cache[(desc_cache_index + i) & 31], 0, 3);
    }
}

/* Fast path TX kick - returns true if work was done */
bool el3_fastpath_tx_kick(EL3Core *c)
{
    int processed;
    
    if (unlikely(!c->tx_enabled)) {
        return false;
    }
    
    processed = el3_fastpath_tx_batch(c, current_tx_batch_size);
    
    /* Auto-tune batch size based on processing efficiency */
    if (likely(processed > 0)) {
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        int64_t elapsed = now - last_tx_time;
        
        if (elapsed < 1000000) { /* Less than 1ms */
            current_tx_batch_size = MIN(current_tx_batch_size + 1, 32);
        } else if (elapsed > 10000000) { /* More than 10ms */
            current_tx_batch_size = MAX(current_tx_batch_size - 1, 1);
        }
        
        last_tx_time = now;
        tx_batch_count++;
        tx_packets_processed += processed;
        return true;
    }
    
    return false;
}

/* Fast path RX deliver - returns true if frame was accepted */
bool el3_fastpath_rx_deliver(EL3Core *c, const uint8_t *buf, size_t len)
{
    if (unlikely(!c->rx_enabled)) {
        return false;
    }
    
    if (el3_accept_frame(c, buf, len)) {
        el3_core_rx_notify(c, len);
        return true;
    }
    
    return false;
}

/* Batch process TX descriptors */
int el3_fastpath_tx_batch(EL3Core *c, int max_batch)
{
    int count = 0;
    hwaddr desc_addr = c->down_list_ptr;
    EL3DMAEngine *dma = &c->dma_engine;
    EL3CachedDesc *cached_desc;
    
    if (unlikely(!dma->enabled || max_batch <= 0)) {
        return 0;
    }
    
    while (count < max_batch && desc_addr != 0) {
        /* Get cached descriptor */
        cached_desc = el3_get_cached_desc();
        
        /* Prefetch next descriptors */
        el3_prefetch_next_desc(c, desc_addr, PREFETCH_DISTANCE);
        
        /* Read descriptor in single DMA operation */
        address_space_read(c->dma_as, desc_addr, MEMTXATTRS_UNSPECIFIED,
                         &cached_desc->desc.down, sizeof(EL3DownDesc));
        
        /* Check if descriptor is valid and ready */
        if (unlikely(!(cached_desc->desc.down.status & EL3_DESC_DOWN_COMPLETE))) {
            break;
        }
        
        /* Process packet if length is valid */
        if (likely(cached_desc->desc.down.length > 0 && 
                  cached_desc->desc.down.length <= EL3_TX_SANITY_MAX)) {
            
            /* Read packet data directly into cached buffer */
            address_space_read(c->dma_as, cached_desc->desc.down.addr,
                             MEMTXATTRS_UNSPECIFIED,
                             cached_desc->data, cached_desc->desc.down.length);
            
            /* Submit to network backend with zero-copy */
            el3_core_tx_submit(c, cached_desc->data, cached_desc->desc.down.length);
            
            /* Update descriptor status */
            cached_desc->desc.down.status |= EL3_DESC_DMA_DONE;
            if (cached_desc->desc.down.status & EL3_DESC_DMA_INDICATE) {
                el3_core_set_int_status(c, STAT_DOWN_COMPLETE);
            }
            
            /* Write updated descriptor back */
            address_space_write(c->dma_as, desc_addr, MEMTXATTRS_UNSPECIFIED,
                              &cached_desc->desc.down, sizeof(EL3DownDesc));
            
            tx_dma_coalesced++;
        }
        
        /* Move to next descriptor */
        desc_addr = cached_desc->desc.down.next_desc;
        count++;
    }
    
    return count;
}

/* Batch process RX descriptors */
int el3_fastpath_rx_batch(EL3Core *c, int max_batch)
{
    int count = 0;
    hwaddr desc_addr = c->up_list_ptr;
    EL3DMAEngine *dma = &c->dma_engine;
    EL3CachedDesc *cached_desc;
    uint32_t rx_status = RXD_COMPLETE;
    
    if (unlikely(!dma->enabled || max_batch <= 0)) {
        return 0;
    }
    
    while (count < max_batch && desc_addr != 0) {
        /* Get cached descriptor */
        cached_desc = el3_get_cached_desc();
        
        /* Prefetch next descriptors */
        el3_prefetch_next_desc(c, desc_addr, PREFETCH_DISTANCE);
        
        /* Read descriptor in single DMA operation */
        address_space_read(c->dma_as, desc_addr, MEMTXATTRS_UNSPECIFIED,
                         &cached_desc->desc.up, sizeof(EL3UpDesc));
        
        /* Check if descriptor is valid */
        if (unlikely(cached_desc->desc.up.length == 0)) {
            break;
        }
        
        /* Check for RX errors */
        if (unlikely(c->stats.rx_overruns > 0)) {
            rx_status |= RXD_ERROR | RXD_OVERRUN;
        }
        
        /* Write packet data directly from cached buffer */
        address_space_write(c->dma_as, cached_desc->desc.up.addr,
                          MEMTXATTRS_UNSPECIFIED,
                          cached_desc->data, cached_desc->desc.up.length);
        
        /* Update descriptor status */
        cached_desc->desc.up.status = rx_status | (cached_desc->desc.up.length & RXD_LENGTH_MASK);
        cached_desc->desc.up.status |= EL3_DESC_DMA_DONE;
        
        if (cached_desc->desc.up.status & EL3_DESC_DMA_INDICATE) {
            el3_core_set_int_status(c, STAT_UP_COMPLETE);
        }
        
        /* Write updated descriptor back */
        address_space_write(c->dma_as, desc_addr, MEMTXATTRS_UNSPECIFIED,
                          &cached_desc->desc.up, sizeof(EL3UpDesc));
        
        rx_dma_coalesced++;
        
        /* Move to next descriptor */
        desc_addr = cached_desc->desc.up.next_desc;
        count++;
    }
    
    rx_batch_count++;
    rx_packets_processed += count;
    
    return count;
}

/* Fast path TX zero-copy using iovec */
void el3_fastpath_tx_zerocopy(EL3Core *c, struct iovec *iov, int iovcnt)
{
    int i;
    hwaddr desc_addr = c->down_list_ptr;
    EL3DMAEngine *dma = &c->dma_engine;
    EL3CachedDesc *cached_desc;
    
    if (unlikely(!dma->enabled)) {
        return;
    }
    
    for (i = 0; i < iovcnt && desc_addr != 0; i++) {
        /* Get cached descriptor */
        cached_desc = el3_get_cached_desc();
        
        /* Prefetch next descriptors */
        el3_prefetch_next_desc(c, desc_addr, PREFETCH_DISTANCE);
        
        /* Read descriptor */
        address_space_read(c->dma_as, desc_addr, MEMTXATTRS_UNSPECIFIED,
                         &cached_desc->desc.down, sizeof(EL3DownDesc));
        
        /* Write iovec data directly to buffer */
        iov_to_buf(iov, iovcnt, 0, cached_desc->data, iov_size(iov, iovcnt));
        
        /* Write packet data */
        address_space_write(c->dma_as, cached_desc->desc.down.addr,
                          MEMTXATTRS_UNSPECIFIED,
                          cached_desc->data, iov_size(iov, iovcnt));
        
        /* Update descriptor status */
        cached_desc->desc.down.status |= EL3_DESC_DMA_DONE;
        if (cached_desc->desc.down.status & EL3_DESC_DMA_INDICATE) {
            el3_core_set_int_status(c, STAT_DOWN_COMPLETE);
        }
        
        /* Write updated descriptor back */
        address_space_write(c->dma_as, desc_addr, MEMTXATTRS_UNSPECIFIED,
                          &cached_desc->desc.down, sizeof(EL3DownDesc));
        
        tx_cache_hits++;
        
        /* Move to next descriptor */
        desc_addr = cached_desc->desc.down.next_desc;
    }
}

/* Fast path RX zero-copy using iovec */
void el3_fastpath_rx_zerocopy(EL3Core *c, struct iovec *iov, int iovcnt)
{
    int i;
    hwaddr desc_addr = c->up_list_ptr;
    EL3DMAEngine *dma = &c->dma_engine;
    EL3CachedDesc *cached_desc;
    uint32_t rx_status = RXD_COMPLETE;
    
    if (unlikely(!dma->enabled)) {
        return;
    }
    
    for (i = 0; i < iovcnt && desc_addr != 0; i++) {
        size_t iov_len = iov_size(iov, iovcnt);
        
        /* Get cached descriptor */
        cached_desc = el3_get_cached_desc();
        
        /* Prefetch next descriptors */
        el3_prefetch_next_desc(c, desc_addr, PREFETCH_DISTANCE);
        
        /* Read descriptor */
        address_space_read(c->dma_as, desc_addr, MEMTXATTRS_UNSPECIFIED,
                         &cached_desc->desc.up, sizeof(EL3UpDesc));
        
        /* Read packet data directly into iovec */
        iov_from_buf(iov, iovcnt, 0, cached_desc->data, iov_len);
        
        /* Update descriptor status */
        cached_desc->desc.up.status = rx_status | (iov_len & RXD_LENGTH_MASK);
        cached_desc->desc.up.status |= EL3_DESC_DMA_DONE;
        
        if (cached_desc->desc.up.status & EL3_DESC_DMA_INDICATE) {
            el3_core_set_int_status(c, STAT_UP_COMPLETE);
        }
        
        /* Write updated descriptor back */
        address_space_write(c->dma_as, desc_addr, MEMTXATTRS_UNSPECIFIED,
                          &cached_desc->desc.up, sizeof(EL3UpDesc));
        
        rx_cache_hits++;
        
        /* Move to next descriptor */
        desc_addr = cached_desc->desc.up.next_desc;
    }
}

/* QMP query function for performance metrics */
void el3_query_performance_metrics(EL3Core *c, QObject **ret_data)
{
    QDict *dict = qdict_new();
    
    qdict_put_int(dict, "tx_batch_count", tx_batch_count);
    qdict_put_int(dict, "rx_batch_count", rx_batch_count);
    qdict_put_int(dict, "tx_packets_processed", tx_packets_processed);
    qdict_put_int(dict, "rx_packets_processed", rx_packets_processed);
    qdict_put_int(dict, "tx_dma_coalesced", tx_dma_coalesced);
    qdict_put_int(dict, "rx_dma_coalesced", rx_dma_coalesced);
    qdict_put_int(dict, "tx_cache_hits", tx_cache_hits);
    qdict_put_int(dict, "rx_cache_hits", rx_cache_hits);
    qdict_put_int(dict, "current_tx_batch_size", current_tx_batch_size);
    qdict_put_int(dict, "current_rx_batch_size", current_rx_batch_size);
    
    *ret_data = QOBJECT(dict);
}

/* Initialize fast path components */
void el3_fastpath_init(EL3Core *c)
{
    el3_desc_cache_init();
    last_tx_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    last_rx_time = last_tx_time;
}

/* Reset fast path state */
void el3_fastpath_reset(EL3Core *c)
{
    tx_batch_count = 0;
    rx_batch_count = 0;
    tx_packets_processed = 0;
    rx_packets_processed = 0;
    tx_dma_coalesced = 0;
    rx_dma_coalesced = 0;
    tx_cache_hits = 0;
    rx_cache_hits = 0;
    
    current_tx_batch_size = 4;
    current_rx_batch_size = 4;
    
    el3_desc_cache_init();
}