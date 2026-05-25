#include "qemu/osdep.h"
#include "hw/net/el3_core.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"
#include "net/net.h"
#include "qemu/iov.h"

/* IOMMU-safe DMA mapping with bounce buffer fallback */
void *el3_dma_map(EL3Core *c, dma_addr_t addr, dma_addr_t *len,
                  DMADirection dir, bool *bounce_buffer_used)
{
    hwaddr l = *len;
    void *buffer;

    /* Validate address based on device capabilities */
    if (c->model == MODEL_3C515) {
        if (addr > ISA_DMA_MAX_ADDR) {
            /* Use bounce buffer for >24-bit addresses */
            buffer = g_malloc(l);
            *bounce_buffer_used = true;
            return buffer;
        }
    } else {
        if (addr >= (1ULL << 32)) {
            /* Use bounce buffer for >32-bit addresses */
            buffer = g_malloc(l);
            *bounce_buffer_used = true;
            return buffer;
        }
    }

    /* Try zero-copy mapping */
    buffer = dma_memory_map(address_space_memory, addr, &l, dir,
                           MEMTXATTRS_UNSPECIFIED);
    if (!buffer) {
        /* Fallback to bounce buffer on mapping failure */
        buffer = g_malloc(*len);
        *bounce_buffer_used = true;
        return buffer;
    }

    *len = l;
    *bounce_buffer_used = false;
    return buffer;
}

/* Unmap DMA region or free bounce buffer */
void el3_dma_unmap(EL3Core *c, void *buffer, dma_addr_t len,
                   DMADirection dir, dma_addr_t access_len)
{
    if (!buffer) {
        return;
    }

    if (access_len <= len) {
        if (dir == DMA_DIRECTION_TO_DEVICE) {
            dma_memory_unmap(address_space_memory, buffer, len,
                            dir, access_len, MEMTXATTRS_UNSPECIFIED);
        } else {
            dma_memory_unmap(address_space_memory, buffer, len,
                            dir, access_len, MEMTXATTRS_UNSPECIFIED);
        }
    } else {
        /* Bounce buffer case - just free it */
        g_free(buffer);
    }
}

/* Read DMA descriptor with IOMMU translation */
int el3_dma_read_desc(EL3Core *c, dma_addr_t addr, EL3DMADesc *desc)
{
    dma_addr_t len = sizeof(EL3DMADesc);
    bool bounce_used;
    void *mapped;
    int ret = 0;

    mapped = el3_dma_map(c, addr, &len, DMA_DIRECTION_FROM_DEVICE,
                         &bounce_used);
    if (!mapped) {
        return -1;
    }

    if (len >= sizeof(EL3DMADesc)) {
        if (!bounce_used) {
            /* Direct mapping successful - copy from guest memory */
            desc->next_desc = ldl_le_p((uint32_t *)mapped);
            desc->status = ldl_le_p((uint32_t *)mapped + 1);
            desc->addr = ldl_le_p((uint32_t *)mapped + 2);
            desc->length = ldl_le_p((uint32_t *)mapped + 3);
        } else {
            /* Bounce buffer case - read from temporary buffer */
            memcpy(desc, mapped, sizeof(EL3DMADesc));
        }
    } else {
        /* Incomplete mapping */
        ret = -1;
    }

    el3_dma_unmap(c, mapped, sizeof(EL3DMADesc),
                  DMA_DIRECTION_FROM_DEVICE, len);
    return ret;
}

/* Write descriptor status back with IOMMU translation */
int el3_dma_write_desc_status(EL3Core *c, dma_addr_t addr,
                              uint32_t status)
{
    dma_addr_t len = sizeof(uint32_t);
    bool bounce_used;
    void *mapped;
    int ret = 0;

    mapped = el3_dma_map(c, addr + offsetof(EL3DMADesc, status),
                         &len, DMA_DIRECTION_TO_DEVICE, &bounce_used);
    if (!mapped) {
        return -1;
    }

    if (len >= sizeof(uint32_t)) {
        if (!bounce_used) {
            /* Direct mapping - write to guest memory */
            stl_le_p((uint32_t *)mapped, status);
        } else {
            /* Bounce buffer case - write to temporary buffer and copy back */
            uint32_t temp_status = status;
            MemTxResult result = dma_memory_write(address_space_memory,
                                                 addr + offsetof(EL3DMADesc, status),
                                                 &temp_status, sizeof(temp_status),
                                                 MEMTXATTRS_UNSPECIFIED);
            if (result != MEMTX_OK) {
                ret = -1;
            }
        }
    } else {
        ret = -1;
    }

    el3_dma_unmap(c, mapped, sizeof(uint32_t),
                  DMA_DIRECTION_TO_DEVICE, len);
    return ret;
}

/* Scatter-gather DMA mapping */
int el3_dma_map_sg(EL3Core *c, EL3DMADesc *desc, int max_frags,
                   struct iovec *iov, unsigned int *iov_cnt,
                   DMADirection dir)
{
    int i = 0;
    dma_addr_t addr = desc->addr;
    dma_addr_t len = desc->length;
    bool bounce_used;
    void *mapped;

    while (i < max_frags && addr && len) {
        mapped = el3_dma_map(c, addr, &len, dir, &bounce_used);
        if (!mapped) {
            /* Unmap previously mapped fragments on error */
            for (int j = 0; j < i; j++) {
                el3_dma_unmap(c, iov[j].iov_base, iov[j].iov_len,
                              dir, iov[j].iov_len);
            }
            return -1;
        }

        iov[i].iov_base = mapped;
        iov[i].iov_len = len;
        i++;

        /* Handle chained descriptors for scatter-gather */
        if (el3_dma_read_desc(c, desc->next_desc, desc) < 0) {
            el3_dma_unmap(c, mapped, len, dir, len);
            for (int j = 0; j < i - 1; j++) {
                el3_dma_unmap(c, iov[j].iov_base, iov[j].iov_len,
                              dir, iov[j].iov_len);
            }
            return -1;
        }

        addr = desc->addr;
        len = desc->length;
    }

    *iov_cnt = i;
    return 0;
}

/* Scatter-gather DMA unmapping */
void el3_dma_unmap_sg(EL3Core *c, struct iovec *iov,
                      unsigned int iov_cnt, DMADirection dir)
{
    for (unsigned int i = 0; i < iov_cnt; i++) {
        el3_dma_unmap(c, iov[i].iov_base, iov[i].iov_len,
                      dir, iov[i].iov_len);
    }
}

/* Check for backpressure and stop processing if needed */
bool el3_dma_check_backpressure(EL3Core *c)
{
    if (qemu_net_queue_full(c->nic->ncs)) {
        /* Schedule bottom half to resume processing later */
        qemu_bh_schedule(c->dma_bh);
        return true;
    }
    return false;
}

/* Process descriptors with work limiting */
void el3_dma_process_descriptors(EL3Core *c, bool is_tx)
{
    int desc_count = 0;
    int byte_count = 0;
    int64_t start_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    EL3DMADesc desc;
    struct iovec iov[EL3_DMA_RING_SIZE];
    unsigned int iov_cnt;

    while (desc_count < EL3_DMA_RING_SIZE && 
           byte_count < 65536 &&  /* Max bytes per kick */
           (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - start_time) < 1000000) { /* Max time 1ms */

        if (el3_dma_check_backpressure(c)) {
            break;
        }

        /* Read current descriptor */
        if (is_tx) {
            if (el3_dma_read_desc(c, c->down_list_ptr, &desc) < 0) {
                break;
            }
        } else {
            if (el3_dma_read_desc(c, c->up_list_ptr, &desc) < 0) {
                break;
            }
        }

        /* Map scatter-gather fragments */
        if (el3_dma_map_sg(c, &desc, EL3_DMA_RING_SIZE, iov, &iov_cnt,
                           is_tx ? DMA_DIRECTION_TO_DEVICE : DMA_DIRECTION_FROM_DEVICE) < 0) {
            el3_core_set_status(c, STAT_ADAPTER_FAIL);
            el3_update_irq(c);
            break;
        }

        if (is_tx) {
            /* Transmit packet */
            qemu_send_packet(c->nic->ncs, iov[0].iov_base, iov[0].iov_len);
            byte_count += iov[0].iov_len;
            
            /* Update descriptor status */
            el3_dma_write_desc_status(c, c->down_list_ptr,
                                      desc.status | EL3_DESC_DMA_DONE);
            c->down_list_ptr = desc.next_desc;
        } else {
            /* Receive packet */
            uint8_t *rx_buf = g_malloc(desc.length);
            int rx_len = 0;
            
            for (unsigned int i = 0; i < iov_cnt; i++) {
                memcpy(rx_buf + rx_len, iov[i].iov_base, iov[i].iov_len);
                rx_len += iov[i].iov_len;
            }
            
            byte_count += rx_len;
            
            /* Update descriptor status */
            el3_dma_write_desc_status(c, c->up_list_ptr,
                                      desc.status | EL3_DESC_DMA_DONE | EL3_DESC_UP_COMPLETE);
            c->up_list_ptr = desc.next_desc;
            
            g_free(rx_buf);
        }

        /* Unmap scatter-gather fragments */
        el3_dma_unmap_sg(c, iov, iov_cnt, 
                         is_tx ? DMA_DIRECTION_TO_DEVICE : DMA_DIRECTION_FROM_DEVICE);

        desc_count++;
        
        /* Check for end of ring */
        if (desc.next_desc == 0) {
            break;
        }
    }

    /* Schedule bottom half if there's more work */
    if (desc_count == EL3_DMA_RING_SIZE || 
        byte_count >= 65536 ||
        (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - start_time) >= 1000000) {
        qemu_bh_schedule(c->dma_bh);
    }
}