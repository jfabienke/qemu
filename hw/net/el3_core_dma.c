#include "hw/net/el3_core.h"

/* TX Descriptor Error Handling */

void el3_dma_update_tx_status(EL3Core *core, EL3DMAEngine *dma, uint32_t error_bits)
{
    /* Set the error bits in the descriptor status */
    dma->down_desc.status |= error_bits;
    
    /* Always set the download complete bit when an error occurs */
    dma->down_desc.status |= TXD_DN_COMPLETE;
    
    /* Check if interrupt on upload to FIFO is requested */
    if (dma->down_desc.status & TXD_INTR_UPLOADED) {
        el3_core_set_int_status(core, STAT_DOWN_COMPLETE);
        el3_update_irq(core);
    }
    
    /* Update statistics counters */
    if (error_bits & TXD_UNDERRUN) {
        core->stats.tx_underruns++;
    }
    if (error_bits & TXD_MAX_COLL) {
        core->stats.tx_mult_collisions++;
    }
    if (error_bits & TXD_JABBER) {
        core->stats.tx_heartbeat_errors++;
    }
    if (error_bits & TXD_LATE_COLL) {
        core->stats.tx_late_collisions++;
    }
    if (error_bits & TXD_CARRIER_LOST) {
        core->stats.tx_carrier_errors++;
    }
}

void el3_dma_tx_error_recovery(EL3Core *core, EL3DMAEngine *dma)
{
    /* Reset current TX operation */
    dma->transfer_len = 0;
    dma->buffer_addr = 0;
    dma->state = EL3_DMA_IDLE;
    
    /* Clear download status */
    dma->down_desc.status = 0;
    
    /* Update core status */
    el3_core_set_status(core, STAT_DOWN_COMPLETE);
    el3_update_irq(core);
}

void el3_dma_reset_tx_chain(EL3DMAEngine *dma)
{
    /* Reset the descriptor chain to initial state */
    dma->current_desc = 0;
    dma->transfer_len = 0;
    dma->buffer_addr = 0;
    dma->state = EL3_DMA_IDLE;
    
    /* Clear descriptor status */
    dma->down_desc.status = 0;
}

/* RX Descriptor Error Handling */

void el3_dma_update_rx_status(EL3Core *core, EL3DMAEngine *dma, uint32_t error_bits)
{
    /* Set the error bits in the descriptor status */
    dma->up_desc.status |= error_bits;
    
    /* Set RXD_ERROR bit if any error occurred */
    if (error_bits & (RXD_OVERRUN | RXD_RUNT | RXD_ALIGN_ERROR | 
                      RXD_CRC_ERROR | RXD_OVERSIZE | RXD_DRIBBLE)) {
        dma->up_desc.status |= RXD_ERROR;
    }
    
    /* Always set the complete bit */
    dma->up_desc.status |= RXD_COMPLETE;
    
    /* Update statistics counters */
    if (error_bits & RXD_OVERRUN) {
        core->stats.rx_overruns++;
    }
}

void el3_dma_rx_error_recovery(EL3Core *core, EL3DMAEngine *dma)
{
    /* Reset current RX operation */
    dma->transfer_len = 0;
    dma->buffer_addr = 0;
    dma->state = EL3_DMA_IDLE;
    
    /* Clear upload status */
    dma->up_desc.status = 0;
    
    /* Update core status */
    el3_core_set_status(core, STAT_UP_COMPLETE);
    el3_update_irq(core);
}

void el3_dma_reset_rx_chain(EL3DMAEngine *dma)
{
    /* Reset the descriptor chain to initial state */
    dma->current_desc = 0;
    dma->transfer_len = 0;
    dma->buffer_addr = 0;
    dma->state = EL3_DMA_IDLE;
    
    /* Clear descriptor status */
    dma->up_desc.status = 0;
}

bool el3_dma_check_ownership(EL3DMAEngine *dma)
{
    /* For both TX and RX descriptors, bit 31 indicates ownership */
    return (dma->down_desc.status & 0x80000000) || 
           (dma->up_desc.status & 0x80000000);
}

void el3_dma_tx_submit(EL3DMAEngine *dma, const uint8_t *data, size_t len)
{
    /* Check for address space limitations */
    if (!dma->as) {
        return;
    }
    
    /* Handle ISA bus master (24-bit) constraints */
    if (!dma->opaque && (dma->buffer_addr + len) > ISA_DMA_MAX_ADDR) {
        el3_dma_update_tx_status(dma->opaque, dma, TXD_UNDERRUN);
        el3_dma_tx_error_recovery(dma->opaque, dma);
        return;
    }
    
    /* Check for boundary crossing */
    if ((dma->buffer_addr & ~(ISA_DMA_BOUNDARY - 1)) != 
        ((dma->buffer_addr + len - 1) & ~(ISA_DMA_BOUNDARY - 1))) {
        el3_dma_update_tx_status(dma->opaque, dma, TXD_UNDERRUN);
        el3_dma_tx_error_recovery(dma->opaque, dma);
        return;
    }
    
    /* Perform DMA transfer */
    dma_memory_write(dma->as, dma->buffer_addr, data, len, MEMTXATTRS_UNSPECIFIED);
    
    /* Update descriptor status */
    dma->down_desc.status |= TXD_DN_COMPLETE;
    
    /* Check if interrupt on upload to FIFO is requested */
    if (dma->down_desc.status & TXD_INTR_UPLOADED) {
        EL3Core *core = dma->opaque;
        el3_core_set_int_status(core, STAT_DOWN_COMPLETE);
        el3_update_irq(core);
    }
}

void el3_dma_rx_submit(EL3DMAEngine *dma, const uint8_t *data, size_t len, uint32_t status)
{
    /* Check for address space limitations */
    if (!dma->as) {
        return;
    }
    
    /* Handle ISA bus master (24-bit) constraints */
    if (!dma->opaque && (dma->buffer_addr + len) > ISA_DMA_MAX_ADDR) {
        el3_dma_update_rx_status(dma->opaque, dma, RXD_OVERRUN);
        el3_dma_rx_error_recovery(dma->opaque, dma);
        return;
    }
    
    /* Check for boundary crossing */
    if ((dma->buffer_addr & ~(ISA_DMA_BOUNDARY - 1)) != 
        ((dma->buffer_addr + len - 1) & ~(ISA_DMA_BOUNDARY - 1))) {
        el3_dma_update_rx_status(dma->opaque, dma, RXD_OVERRUN);
        el3_dma_rx_error_recovery(dma->opaque, dma);
        return;
    }
    
    /* Perform DMA transfer */
    dma_memory_write(dma->as, dma->buffer_addr, data, len, MEMTXATTRS_UNSPECIFIED);
    
    /* Update descriptor with actual length and status */
    dma->up_desc.status = status | RXD_COMPLETE;
    dma->up_desc.length = len;
}