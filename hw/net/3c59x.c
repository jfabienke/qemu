/*
 * QEMU 3Com 3C59x Vortex/Boomerang emulation
 *
 * Copyright (c) 2024 QEMU contributors
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "hw/pci/pci_device.h"
#include "hw/net/el3_core.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/error-report.h"
#include "net/net.h"
#include "net/eth.h"
/* DMA functions are included via pci headers */

#define TYPE_PCI_3C59X "3c59x"
OBJECT_DECLARE_SIMPLE_TYPE(PCI3C59XState, PCI_3C59X)

#define PCI_3C59X_IO_SIZE    0x80
#define PCI_3C59X_MMIO_SIZE  0x80

struct PCI3C59XState {
    PCIDevice parent_obj;
    EL3Core core;
    EL3DMAEngine dma_engine;
    MemoryRegion io;
    MemoryRegion mmio;
};

/* PCI DMA descriptor structure (16 bytes) */
typedef struct {
    uint32_t next;      /* Next descriptor physical address */
    uint32_t status;    /* Status and control bits */
    uint32_t addr;      /* Data buffer physical address */
    uint32_t length;    /* Fragment length and flags */
} EL3PciDesc;

/* Read PCI DMA descriptor */
static bool el3_pci_dma_read_desc(PCI3C59XState *s, hwaddr addr, EL3PciDesc *desc)
{
    PCIDevice *pci = PCI_DEVICE(s);
    
    /* Read 16-byte descriptor */
    if (pci_dma_read(pci, addr, desc, sizeof(*desc)) != 0) {
        return false;
    }
    
    /* Convert from little-endian */
    desc->next = le32_to_cpu(desc->next);
    desc->status = le32_to_cpu(desc->status);
    desc->addr = le32_to_cpu(desc->addr);
    desc->length = le32_to_cpu(desc->length);
    
    return true;
}

/* Write back descriptor status */
static void el3_pci_dma_write_status(PCI3C59XState *s, hwaddr addr, uint32_t status)
{
    PCIDevice *pci = PCI_DEVICE(s);
    uint32_t le_status = cpu_to_le32(status);
    pci_dma_write(pci, addr + 4, &le_status, 4);
}

/* Process TX descriptor chain */
static void el3_pci_process_tx_chain(PCI3C59XState *s)
{
    EL3PciDesc desc;
    uint8_t buf[1536];  /* ETH_MAX_FRAME_LEN */
    uint32_t total_len = 0;
    hwaddr current = s->core.down_list_ptr;
    int processed = 0;
    const int max_descriptors = 64;
    
    while (current && processed < max_descriptors) {
        /* Read descriptor */
        if (!el3_pci_dma_read_desc(s, current, &desc)) {
            s->core.status |= STAT_ADAPTER_FAIL;
            s->core.int_status |= STAT_ADAPTER_FAIL;
            el3_update_irq(&s->core);
            break;
        }
        
        /* Check ownership (bit 31) */
        if (!(desc.status & 0x80000000)) {
            /* Driver owns it, stall */
            s->core.down_stalled = true;
            break;
        }
        
        /* Extract packet length */
        uint32_t len = desc.length & 0x1FFF;
        
        /* Check for last fragment (bit 31 of length) */
        bool last_frag = (desc.length & 0x80000000) != 0;
        
        /* Read fragment data */
        if (total_len + len > sizeof(buf)) {
            len = sizeof(buf) - total_len;
        }
        
        if (pci_dma_read(PCI_DEVICE(s), desc.addr, buf + total_len, len) != 0) {
            s->core.status |= STAT_ADAPTER_FAIL;
            break;
        }
        
        total_len += len;
        
        /* If last fragment, send packet */
        if (last_frag) {
            qemu_send_packet(qemu_get_queue(s->core.nic), buf, total_len);
            s->core.stats.tx_frames_ok++;
            s->core.stats.tx_bytes_ok += total_len;
            total_len = 0;
        }
        
        /* Update descriptor status */
        desc.status &= ~0x80000000;  /* Clear ownership */
        desc.status |= 0x00004000;   /* Set download complete */
        el3_pci_dma_write_status(s, current, desc.status);
        
        /* Generate interrupt if requested (bit 15) */
        if (desc.status & 0x00008000) {
            s->core.status |= STAT_DOWN_COMPLETE;
            s->core.int_status |= STAT_DOWN_COMPLETE;
            el3_update_irq(&s->core);
        }
        
        /* Move to next descriptor */
        current = desc.next;
        processed++;
    }
    
    s->core.down_list_ptr = current;
}

/* Process RX into descriptor chain */
static ssize_t el3_pci_rx_place_frame(EL3Core *c, const uint8_t *buf, size_t size,
                                      uint32_t rx_status, uint32_t rx_len)
{
    PCI3C59XState *s = container_of(c, PCI3C59XState, core);
    EL3PciDesc desc;
    hwaddr current = c->up_list_ptr;
    size_t offset = 0;
    
    while (current && offset < size) {
        /* Read descriptor */
        if (!el3_pci_dma_read_desc(s, current, &desc)) {
            return -1;
        }
        
        /* Check ownership */
        if (!(desc.status & 0x80000000)) {
            /* Driver owns it, drop packet */
            c->up_stalled = true;
            return -1;
        }
        
        /* Calculate fragment size */
        uint32_t frag_size = desc.length & 0x1FFF;
        if (frag_size > size - offset) {
            frag_size = size - offset;
        }
        
        /* Write data to buffer */
        if (pci_dma_write(PCI_DEVICE(s), desc.addr, buf + offset, frag_size) != 0) {
            return -1;
        }
        
        offset += frag_size;
        
        /* Update descriptor */
        desc.status &= ~0x80000000;  /* Clear ownership */
        desc.status |= 0x00004000;   /* Set upload complete */
        desc.status |= (size & 0x1FFF);  /* Store total packet length */
        
        /* Set error bits if needed */
        if (rx_status & RX_STATUS_ERROR) {
            desc.status |= 0x00002000;  /* Upload error */
        }
        
        /* Mark last fragment */
        if (offset >= size) {
            desc.length |= 0x80000000;  /* Last fragment bit */
        }
        
        el3_pci_dma_write_status(s, current, desc.status);
        
        /* Generate interrupt if requested */
        if (desc.status & 0x00008000) {
            c->status |= STAT_UP_COMPLETE;
            c->int_status |= STAT_UP_COMPLETE;
            el3_update_irq(c);
        }
        
        /* Move to next */
        current = desc.next;
        
        /* If packet complete, break */
        if (offset >= size) {
            break;
        }
    }
    
    c->up_list_ptr = current;
    return size;
}

/* TX DMA bottom half handler */
static void el3_pci_tx_bh(void *opaque)
{
    PCI3C59XState *s = opaque;
    
    if (!s->core.down_stalled && s->core.down_list_ptr) {
        el3_pci_process_tx_chain(s);
    }
}

/* RX DMA bottom half handler */
static void el3_pci_rx_bh(void *opaque)
{
    PCI3C59XState *s = opaque;
    
    /* Process any pending RX packets */
    if (!s->core.up_stalled && s->core.up_list_ptr) {
        /* Flush any queued packets from the network backend */
        qemu_flush_queued_packets(qemu_get_queue(s->core.nic));
    }
}

/* TX kick handler */
static void el3_pci_tx_kick(EL3Core *c)
{
    PCI3C59XState *s = container_of(c, PCI3C59XState, core);
    
    /* Schedule bottom half for async processing */
    if (s->dma_engine.tx_bh) {
        qemu_bh_schedule(s->dma_engine.tx_bh);
    }
}

/* I/O port read handler */
static uint64_t pci_3c59x_io_read(void *opaque, hwaddr addr, unsigned size)
{
    PCI3C59XState *s = opaque;
    return el3_core_read(&s->core, addr, size);
}

/* I/O port write handler */
static void pci_3c59x_io_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    PCI3C59XState *s = opaque;
    el3_core_write(&s->core, addr, val, size);
}

static const MemoryRegionOps pci_3c59x_io_ops = {
    .read = pci_3c59x_io_read,
    .write = pci_3c59x_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/* MMIO read handler */
static uint64_t pci_3c59x_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    PCI3C59XState *s = opaque;
    return el3_core_read(&s->core, addr, size);
}

/* MMIO write handler */
static void pci_3c59x_mmio_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    PCI3C59XState *s = opaque;
    el3_core_write(&s->core, addr, val, size);
}

static const MemoryRegionOps pci_3c59x_mmio_ops = {
    .read = pci_3c59x_mmio_read,
    .write = pci_3c59x_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/* Network client info */
static NetClientInfo net_3c59x_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .receive = el3_core_receive,
    .link_status_changed = el3_core_set_link_status,
};

/* Reset handler */
static void pci_3c59x_reset(DeviceState *dev)
{
    PCI3C59XState *s = PCI_3C59X(dev);
    
    /* Reset core */
    el3_core_reset(&s->core);
    
    /* Reset DMA engine state */
    s->dma_engine.state = EL3_DMA_IDLE;
    /* DMA engine will be initialized in realize */
    
    /* Cancel any pending bottom halves */
    if (s->dma_engine.tx_bh) {
        qemu_bh_cancel(s->dma_engine.tx_bh);
    }
    if (s->dma_engine.rx_bh) {
        qemu_bh_cancel(s->dma_engine.rx_bh);
    }
}

/* VMState */
static const VMStateDescription vmstate_pci_3c59x = {
    .name = "3c59x",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, struct PCI3C59XState),
        VMSTATE_STRUCT(core, PCI3C59XState, 0, vmstate_el3_core, EL3Core),
        VMSTATE_STRUCT(dma_engine, PCI3C59XState, 0, vmstate_el3_dma_engine, EL3DMAEngine),
        VMSTATE_END_OF_LIST()
    }
};

/* Properties */
static const Property pci_3c59x_properties[] = {
    DEFINE_NIC_PROPERTIES(PCI3C59XState, core.conf),
};

/* IRQ handler for PCI variant */
static void pci_3c59x_irq_handler(EL3Core *c, bool level)
{
    PCI3C59XState *s = container_of(c, PCI3C59XState, core);
    pci_set_irq(PCI_DEVICE(s), level);
}

/* Variant operations for PCI */
static const EL3VariantOps pci_3c59x_ops = {
    .irq_set = pci_3c59x_irq_handler,
    .tx_kick = el3_pci_tx_kick,
    .rx_place_frame = el3_pci_rx_place_frame,
    .has_dma = true,
};

/* Device realization */
static void pci_3c59x_realize(PCIDevice *dev, Error **errp)
{
    PCI3C59XState *s = PCI_3C59X(dev);
    uint8_t *pci_conf = dev->config;

    /* Set PCI config space IDs */
    pci_config_set_vendor_id(pci_conf, 0x10B7);  /* 3Com vendor ID */
    pci_config_set_device_id(pci_conf, 0x5900);  /* 3C590 Vortex device ID */
    pci_config_set_class(pci_conf, PCI_CLASS_NETWORK_ETHERNET);
    pci_conf[PCI_HEADER_TYPE] = PCI_HEADER_TYPE_NORMAL;
    
    /* Enable bus mastering */
    pci_conf[PCI_COMMAND] = PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER;
    
    /* Initialize core as Vortex with variant ops */
    el3_core_init(&s->core, MODEL_3C590, &pci_3c59x_ops);
    
    /* Enable bus master mode in core */
    s->core.bus_master_enabled = true;
    
    /* Setup MAC address */
    qemu_macaddr_default_if_unset(&s->core.conf.macaddr);
    
    /* Create network interface */
    s->core.nic = qemu_new_nic(&net_3c59x_info, &s->core.conf,
                                TYPE_PCI_3C59X, DEVICE(dev)->id,
                                &DEVICE(dev)->mem_reentrancy_guard, &s->core);
    qemu_format_nic_info_str(qemu_get_queue(s->core.nic),
                              s->core.conf.macaddr.a);
    
    /* Initialize EEPROM with MAC address */
    el3_eeprom_init_3c59x(&s->core);
    
    /* Setup I/O port BAR */
    memory_region_init_io(&s->io, OBJECT(dev), &pci_3c59x_io_ops,
                          s, TYPE_PCI_3C59X, PCI_3C59X_IO_SIZE);
    pci_register_bar(dev, 0, PCI_BASE_ADDRESS_SPACE_IO, &s->io);
    
    /* Setup MMIO BAR */
    memory_region_init_io(&s->mmio, OBJECT(dev), &pci_3c59x_mmio_ops,
                          s, TYPE_PCI_3C59X, PCI_3C59X_MMIO_SIZE);
    pci_register_bar(dev, 1, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->mmio);
    
    /* Setup IRQ - PCI devices handle IRQs internally */
    /* IRQs are raised via pci_set_irq() in the variant ops */
    
    /* Initialize DMA engine bottom halves */
    s->dma_engine.tx_bh = qemu_bh_new(el3_pci_tx_bh, s);
    s->dma_engine.rx_bh = qemu_bh_new(el3_pci_rx_bh, s);
    s->dma_engine.as = pci_get_address_space(dev);
    s->dma_engine.opaque = s;
    
    /* Wire up DMA BH to core for scheduling */
    s->core.dma_bh = s->dma_engine.tx_bh;
}

/* Device unrealize */
static void pci_3c59x_unrealize(PCIDevice *dev)
{
    PCI3C59XState *s = PCI_3C59X(dev);
    
    /* Cleanup bottom halves */
    if (s->dma_engine.tx_bh) {
        qemu_bh_delete(s->dma_engine.tx_bh);
    }
    if (s->dma_engine.rx_bh) {
        qemu_bh_delete(s->dma_engine.rx_bh);
    }
    
    /* Cleanup network interface */
    qemu_del_nic(s->core.nic);
}

/* Class initialization */
static void pci_3c59x_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(klass);
    
    pc->realize = pci_3c59x_realize;
    pc->exit = pci_3c59x_unrealize;
    dc->legacy_reset = pci_3c59x_reset;
    dc->desc = "3Com 3C590 Vortex";
    dc->vmsd = &vmstate_pci_3c59x;
    device_class_set_props(dc, pci_3c59x_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

/* Type info */
static const TypeInfo pci_3c59x_info = {
    .name = TYPE_PCI_3C59X,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(PCI3C59XState),
    .class_init = pci_3c59x_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

/* Type registration */
static void pci_3c59x_register_types(void)
{
    type_register_static(&pci_3c59x_info);
}

type_init(pci_3c59x_register_types)