/*
 * QEMU 3Com EtherLink III PCI family emulation
 *
 *   -device 3c590   3C590 Vortex     (PCI, windowed PIO datapath only)
 *   -device 3c905   3C905 Boomerang  (PCI, + DnListPtr/UpListPtr descriptor DMA)
 *
 * Two thin PCI badges over the shared EL3 core (el3_core.c), mirroring the ISA
 * 3c509/3c515 wrapper pattern. Hardware-true register/descriptor semantics are
 * taken from the authoritative open drivers:
 *   - Linux drivers/net/ethernet/3com/3c59x.c (Becker):
 *       DPD: { DnNextPtr, FrameStartHeader, frag pairs }, FSH DN_COMPLETE
 *       0x00010000 written back by the NIC, TxIntrUploaded 0x80000000 requests
 *       the DnComplete indication, LAST_FRAG 0x80000000 on the fragment length;
 *       UPD status RxDComplete 0x00008000; ring engines start on a list-pointer
 *       write while idle (DnListPtr reads 0 when the engine has consumed the
 *       list); StallCtl is command 6 (params 0..3 = UpStall/UpUnstall/DownStall/
 *       DownUnstall).
 *   - iPXE src/drivers/net/3c90x.{c,h}: fshDnComplete 0x10000, upComplete
 *       1<<15, up/downLastFrag 1<<31, DnStall -> patch DnNextPtr -> write
 *       DnListPtr if 0 -> DnUnstall append discipline.
 *
 * There is deliberately NO bit-31 "ownership" flag in descriptor status: real
 * silicon has none. TX ownership is implicit in list membership (the driver
 * links a DPD in only under DownStall); RX UPD "free" is UpPktStatus == 0
 * (the driver clears the status dword when it has consumed the packet).
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

#define TYPE_EL3_PCI "el3-pci"
OBJECT_DECLARE_TYPE(EL3PCIState, EL3PCIClass, EL3_PCI)

#define PCI_EL3_IO_SIZE      0x80    /* I/O BAR0 extent (3C590/3C905; no memory BAR) */

/* Boomerang DMA register block inside BAR0 (90x layout; the ISA 3C515 aliases
 * the same block at iobase+0x400). */
#define PCI_EL3_DMA_BASE     0x20
#define PCI_EL3_DMA_END      0x40
#define PCI_EL3_PKT_STATUS   0x20    /* DnPktStatus (RO, diagnostic) */
#define PCI_EL3_DN_LIST_PTR  0x24    /* DnListPtr (32-bit; 16-bit drivers write 0x24/0x26) */
#define PCI_EL3_UP_PKT_STAT  0x30    /* UpPktStatus (RO, diagnostic) */
#define PCI_EL3_UP_LIST_PTR  0x38    /* UpListPtr  (32-bit; halves at 0x38/0x3A) */

/* FrameStartHeader (DPD status dword) -- Linux 3c59x.c / iPXE 3c90x.h */
#define FSH_DN_COMPLETE      0x00010000  /* NIC write-back: packet downloaded */
#define FSH_TX_INDICATE      0x00008000  /* request TxComplete + TxStatus after transmission
                                          * (iPXE fshTxIndicate; polled drivers rely on it) */
#define FSH_DN_INDICATE      0x80000000  /* request DnComplete indication (TxIntrUploaded) */
#define FSH_PKT_LEN_MASK     0x00001FFF

/* Fragment length dword */
#define FRAG_LAST            0x80000000  /* last addr/len pair of this descriptor */
#define FRAG_LEN_MASK        0x00001FFF

/* UpPktStatus (UPD status dword) */
#define UPS_UP_COMPLETE      0x00008000  /* NIC write-back: packet uploaded */
#define UPS_UP_ERROR         0x00004000
#define UPS_UP_OVERRUN       0x00010000  /* buffer too small for the frame */
#define UPS_PKT_LEN_MASK     0x00001FFF

/* Walker safety caps (defensive: a corrupt guest list must not wedge QEMU). */
#define PCI_EL3_MAX_DPDS     64      /* descriptors per kick */
#define PCI_EL3_MAX_FRAGS    16      /* fragment pairs per DPD */
#define PCI_EL3_MAX_FRAME    4608    /* matches the core's FDDI-scale TX slot */

struct EL3PCIState {
    PCIDevice parent_obj;
    EL3Core core;
    EL3DMAEngine dma_engine;
    MemoryRegion io;
};

struct EL3PCIClass {
    PCIDeviceClass parent_class;
    uint16_t device_id;
    EL3Model model;
    bool has_dma;
};

/* ---- descriptor accessors -------------------------------------------------- */

static bool el3_pci_dma_read32(EL3PCIState *s, hwaddr addr, uint32_t *val)
{
    uint32_t le;
    if (pci_dma_read(PCI_DEVICE(s), addr, &le, 4) != 0) {
        return false;
    }
    *val = le32_to_cpu(le);
    return true;
}

/* Write back a descriptor's status dword (offset +4: FSH / UpPktStatus).
 * Status-dword-only, never the driver-owned next/addr/len fields. */
static void el3_pci_dma_write_status(EL3PCIState *s, hwaddr desc, uint32_t status)
{
    uint32_t le = cpu_to_le32(status);
    pci_dma_write(PCI_DEVICE(s), desc + 4, &le, 4);
}

/* ---- TX: download-list walker (Boomerang) ---------------------------------- */

/* Walk the DPD list from DnListPtr: per descriptor gather the fragment pairs
 * (addr/len at +8, +16, ... until FRAG_LAST), transmit the frame, write
 * FSH_DN_COMPLETE back into the FSH, honor FSH_DN_INDICATE, follow DnNextPtr.
 * The engine goes idle (DnListPtr = 0) at a null next pointer -- drivers rely
 * on reading 0 to know they may write a fresh list head. */
static void el3_pci_process_tx_chain(EL3PCIState *s)
{
    EL3Core *c = &s->core;
    uint8_t buf[PCI_EL3_MAX_FRAME];
    hwaddr current = c->down_list_ptr;
    int processed = 0;

    while (current && !c->down_stalled && processed < PCI_EL3_MAX_DPDS) {
        uint32_t next, fsh;
        uint32_t total_len = 0;
        hwaddr frag = current + 8;
        int nfrags = 0;
        bool last = false;

        if (!el3_pci_dma_read32(s, current, &next) ||
            !el3_pci_dma_read32(s, current + 4, &fsh)) {
            c->status |= STAT_ADAPTER_FAIL;
            c->int_status |= STAT_ADAPTER_FAIL;
            el3_update_irq(c);
            break;
        }

        /* A DPD the NIC already completed means the driver hasn't refreshed
         * this entry -- a live engine is never pointed at one. Stop rather
         * than re-transmit stale data. */
        if (fsh & FSH_DN_COMPLETE) {
            break;
        }

        /* Gather the fragment list embedded in this DPD. */
        while (!last && nfrags < PCI_EL3_MAX_FRAGS) {
            uint32_t faddr, flen, len;

            if (!el3_pci_dma_read32(s, frag, &faddr) ||
                !el3_pci_dma_read32(s, frag + 4, &flen)) {
                c->status |= STAT_ADAPTER_FAIL;
                c->int_status |= STAT_ADAPTER_FAIL;
                el3_update_irq(c);
                return;
            }
            last = (flen & FRAG_LAST) != 0;
            len = flen & FRAG_LEN_MASK;
            if (len > sizeof(buf) - total_len) {
                len = sizeof(buf) - total_len;   /* clamp corrupt lists */
            }
            if (len &&
                pci_dma_read(PCI_DEVICE(s), faddr, buf + total_len, len) != 0) {
                c->status |= STAT_ADAPTER_FAIL;
                c->int_status |= STAT_ADAPTER_FAIL;
                el3_update_irq(c);
                return;
            }
            total_len += len;
            frag += 8;
            nfrags++;
        }

        if (total_len) {
            qemu_send_packet(qemu_get_queue(c->nic), buf, total_len);
            c->stats.tx_frames_ok++;
            c->stats.tx_bytes_ok += total_len;
        }

        /* Hardware write-back: FSH |= dnComplete (never touches next/frags). */
        el3_pci_dma_write_status(s, current, fsh | FSH_DN_COMPLETE);

        if (fsh & FSH_DN_INDICATE) {
            c->status |= STAT_DOWN_COMPLETE;
            c->int_status |= STAT_DOWN_COMPLETE;
            el3_update_irq(c);
        }
        if (fsh & FSH_TX_INDICATE) {
            /* Classic EL3 TxComplete indication + TxStatus entry, requested
             * per-packet via the FSH (iPXE gates its whole poll on this). */
            c->tx_status = TX_STAT_COMPLETE;
            c->status |= STAT_TX_COMPLETE;
            c->int_status |= STAT_TX_COMPLETE;
            el3_update_irq(c);
        }

        current = next;
        processed++;
    }

    /* 0 at a natural end = engine idle; a stall preserves the resume point. */
    c->down_list_ptr = current;
}

/* ---- RX: upload-list placer (Boomerang) ------------------------------------ */

/* One UPD carries one packet. A UPD is free iff its UpPktStatus has been
 * cleared by the driver (upComplete not set). If the head UPD is still
 * complete, the up engine stalls: return 0 so the net layer queues the frame;
 * UpUnstall flushes the queue. */
static ssize_t el3_pci_rx_place_frame(EL3Core *c, const uint8_t *buf, size_t size,
                                      uint32_t rx_status, uint32_t rx_len)
{
    EL3PCIState *s = container_of(c, EL3PCIState, core);
    uint32_t next, status, addr, flen;
    uint32_t frag_size, wr;
    hwaddr current = c->up_list_ptr;

    if (!current) {
        return -1;                       /* no upload ring configured: drop */
    }

    if (!el3_pci_dma_read32(s, current, &next) ||
        !el3_pci_dma_read32(s, current + 4, &status) ||
        !el3_pci_dma_read32(s, current + 8, &addr) ||
        !el3_pci_dma_read32(s, current + 12, &flen)) {
        return -1;
    }

    if (status & UPS_UP_COMPLETE) {
        c->up_stalled = true;            /* driver hasn't freed the head UPD */
        return 0;                        /* queue; UpUnstall flushes */
    }

    frag_size = flen & FRAG_LEN_MASK;
    wr = size;
    status = size & UPS_PKT_LEN_MASK;
    if (wr > frag_size) {
        wr = frag_size;                  /* frame larger than the UPD buffer */
        status |= UPS_UP_ERROR | UPS_UP_OVERRUN;
    }
    if (rx_status & RX_STATUS_ERROR) {
        status |= UPS_UP_ERROR;
    }

    if (wr && pci_dma_write(PCI_DEVICE(s), addr, buf, wr) != 0) {
        return -1;
    }

    el3_pci_dma_write_status(s, current, status | UPS_UP_COMPLETE);
    c->up_list_ptr = next;               /* driver rings are circular (Linux/iPXE) */

    c->status |= STAT_UP_COMPLETE;
    c->int_status |= STAT_UP_COMPLETE;
    el3_update_irq(c);
    return size;
}

/* ---- kick plumbing ---------------------------------------------------------- */

static void el3_pci_tx_bh(void *opaque)
{
    EL3PCIState *s = opaque;

    if (!s->core.down_stalled && s->core.down_list_ptr) {
        el3_pci_process_tx_chain(s);
    }
}

static void el3_pci_rx_bh(void *opaque)
{
    EL3PCIState *s = opaque;

    if (!s->core.up_stalled && s->core.up_list_ptr) {
        qemu_flush_queued_packets(qemu_get_queue(s->core.nic));
    }
}

static void el3_pci_tx_kick(EL3Core *c)
{
    EL3PCIState *s = container_of(c, EL3PCIState, core);

    if (s->dma_engine.tx_bh) {
        qemu_bh_schedule(s->dma_engine.tx_bh);
    }
}

/* ---- BAR0 I/O --------------------------------------------------------------- */

/* Boomerang DMA registers live at 0x20-0x3F inside BAR0; the windowed core
 * register file handles everything else (including the Vortex Window-1 +0x10
 * relocation, which the core models per EL3Model). The 3C590 badge has no DMA
 * block: all offsets go to the core. */
static uint64_t el3_pci_io_read(void *opaque, hwaddr addr, unsigned size)
{
    EL3PCIState *s = opaque;
    EL3PCIClass *k = EL3_PCI_GET_CLASS(s);

    if (k->has_dma && addr >= PCI_EL3_DMA_BASE && addr < PCI_EL3_DMA_END) {
        switch (addr) {
        case PCI_EL3_DN_LIST_PTR:
            return (size == 4) ? s->core.down_list_ptr
                               : (s->core.down_list_ptr & 0xFFFFu);
        case PCI_EL3_DN_LIST_PTR + 2:
            return (s->core.down_list_ptr >> 16) & 0xFFFFu;
        case PCI_EL3_UP_LIST_PTR:
            return (size == 4) ? s->core.up_list_ptr
                               : (s->core.up_list_ptr & 0xFFFFu);
        case PCI_EL3_UP_LIST_PTR + 2:
            return (s->core.up_list_ptr >> 16) & 0xFFFFu;
        case PCI_EL3_PKT_STATUS:
        case PCI_EL3_UP_PKT_STAT:
        default:
            return 0;                    /* diagnostic regs: RAZ */
        }
    }
    return el3_core_read(&s->core, addr, size);
}

static void el3_pci_io_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    EL3PCIState *s = opaque;
    EL3PCIClass *k = EL3_PCI_GET_CLASS(s);

    if (k->has_dma && addr >= PCI_EL3_DMA_BASE && addr < PCI_EL3_DMA_END) {
        switch (addr) {
        case PCI_EL3_DN_LIST_PTR:
            if (size == 4) {
                s->core.down_list_ptr = (uint32_t)val;
            } else {
                s->core.down_list_ptr = (s->core.down_list_ptr & 0xFFFF0000u) |
                                        (val & 0xFFFFu);
            }
            /* Writing a non-zero head while the engine is idle starts the
             * fetch (drivers only write when DnListPtr reads 0). A 16-bit
             * driver writes low half then high half: kick on either write;
             * the BH re-reads the final pointer. */
            if (s->core.down_list_ptr && !s->core.down_stalled) {
                el3_pci_tx_kick(&s->core);
            }
            break;
        case PCI_EL3_DN_LIST_PTR + 2:
            s->core.down_list_ptr = (s->core.down_list_ptr & 0x0000FFFFu) |
                                    ((val & 0xFFFFu) << 16);
            if (s->core.down_list_ptr && !s->core.down_stalled) {
                el3_pci_tx_kick(&s->core);
            }
            break;
        case PCI_EL3_UP_LIST_PTR:
            if (size == 4) {
                s->core.up_list_ptr = (uint32_t)val;
            } else {
                s->core.up_list_ptr = (s->core.up_list_ptr & 0xFFFF0000u) |
                                      (val & 0xFFFFu);
            }
            if (s->dma_engine.rx_bh) {
                qemu_bh_schedule(s->dma_engine.rx_bh);
            }
            break;
        case PCI_EL3_UP_LIST_PTR + 2:
            s->core.up_list_ptr = (s->core.up_list_ptr & 0x0000FFFFu) |
                                  ((val & 0xFFFFu) << 16);
            if (s->dma_engine.rx_bh) {
                qemu_bh_schedule(s->dma_engine.rx_bh);
            }
            break;
        default:
            break;                       /* other DMA-block regs: WI */
        }
        return;
    }
    el3_core_write(&s->core, addr, val, size);
}

static const MemoryRegionOps el3_pci_io_ops = {
    .read = el3_pci_io_read,
    .write = el3_pci_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

/* ---- device plumbing --------------------------------------------------------- */

static NetClientInfo net_el3_pci_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .receive = el3_core_receive,
    .link_status_changed = el3_core_set_link_status,
};

static void el3_pci_reset(DeviceState *dev)
{
    EL3PCIState *s = EL3_PCI(dev);
    EL3PCIClass *k = EL3_PCI_GET_CLASS(s);

    el3_core_reset(&s->core);
    /* el3_core_reset clears bus_master_enabled (correct for the ISA parts,
     * whose drivers re-arm DMA explicitly); on Boomerang the descriptor
     * engines ARE the datapath -- keep them wired across reset. */
    s->core.bus_master_enabled = k->has_dma;

    s->dma_engine.state = EL3_DMA_IDLE;
    if (s->dma_engine.tx_bh) {
        qemu_bh_cancel(s->dma_engine.tx_bh);
    }
    if (s->dma_engine.rx_bh) {
        qemu_bh_cancel(s->dma_engine.rx_bh);
    }
}

static const VMStateDescription vmstate_el3_pci = {
    .name = "el3-pci",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, struct EL3PCIState),
        VMSTATE_STRUCT(core, EL3PCIState, 0, vmstate_el3_core, EL3Core),
        VMSTATE_STRUCT(dma_engine, EL3PCIState, 0, vmstate_el3_dma_engine, EL3DMAEngine),
        VMSTATE_END_OF_LIST()
    }
};

static const Property el3_pci_properties[] = {
    DEFINE_NIC_PROPERTIES(EL3PCIState, core.conf),
};

static void el3_pci_irq_handler(EL3Core *c, bool level)
{
    EL3PCIState *s = container_of(c, EL3PCIState, core);
    pci_set_irq(PCI_DEVICE(s), level);
}

/* 3C590 Vortex: windowed PIO datapath only (real Vortex has no descriptor
 * rings; its Wn7 MasterAddr single-shot DMA is not modeled). */
static const EL3VariantOps el3_pci_vortex_ops = {
    .irq_set = el3_pci_irq_handler,
    .has_dma = false,
};

/* 3C905 Boomerang: + descriptor-list bus-master engines. */
static const EL3VariantOps el3_pci_boomerang_ops = {
    .irq_set = el3_pci_irq_handler,
    .tx_kick = el3_pci_tx_kick,
    .rx_place_frame = el3_pci_rx_place_frame,
    .has_dma = true,
};

static void el3_pci_realize(PCIDevice *dev, Error **errp)
{
    EL3PCIState *s = EL3_PCI(dev);
    EL3PCIClass *k = EL3_PCI_GET_CLASS(s);
    uint8_t *pci_conf = dev->config;

    pci_config_set_vendor_id(pci_conf, 0x10B7);          /* 3Com */
    pci_config_set_device_id(pci_conf, k->device_id);
    pci_config_set_class(pci_conf, PCI_CLASS_NETWORK_ETHERNET);
    pci_conf[PCI_HEADER_TYPE] = PCI_HEADER_TYPE_NORMAL;
    pci_conf[PCI_INTERRUPT_PIN] = 1;                     /* INTA# */

    el3_core_init(&s->core, k->model,
                  k->has_dma ? &el3_pci_boomerang_ops : &el3_pci_vortex_ops);
    s->core.bus_master_enabled = k->has_dma;

    qemu_macaddr_default_if_unset(&s->core.conf.macaddr);
    s->core.nic = qemu_new_nic(&net_el3_pci_info, &s->core.conf,
                               object_get_typename(OBJECT(dev)), DEVICE(dev)->id,
                               &DEVICE(dev)->mem_reentrancy_guard, &s->core);
    qemu_format_nic_info_str(qemu_get_queue(s->core.nic),
                             s->core.conf.macaddr.a);

    el3_eeprom_init_3c59x(&s->core);

    memory_region_init_io(&s->io, OBJECT(dev), &el3_pci_io_ops,
                          s, object_get_typename(OBJECT(dev)), PCI_EL3_IO_SIZE);
    pci_register_bar(dev, 0, PCI_BASE_ADDRESS_SPACE_IO, &s->io);

    if (k->has_dma) {
        s->dma_engine.tx_bh = qemu_bh_new(el3_pci_tx_bh, s);
        s->dma_engine.rx_bh = qemu_bh_new(el3_pci_rx_bh, s);
        s->dma_engine.as = pci_get_address_space(dev);
        s->dma_engine.opaque = s;
        s->core.dma_bh = s->dma_engine.tx_bh;
        s->core.dma_as = pci_get_address_space(dev);
    }
}

static void el3_pci_unrealize(PCIDevice *dev)
{
    EL3PCIState *s = EL3_PCI(dev);

    if (s->dma_engine.tx_bh) {
        qemu_bh_delete(s->dma_engine.tx_bh);
    }
    if (s->dma_engine.rx_bh) {
        qemu_bh_delete(s->dma_engine.rx_bh);
    }
    qemu_del_nic(s->core.nic);
}

static void el3_pci_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(klass);

    pc->realize = el3_pci_realize;
    pc->exit = el3_pci_unrealize;
    dc->legacy_reset = el3_pci_reset;
    dc->vmsd = &vmstate_el3_pci;
    device_class_set_props(dc, el3_pci_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static void el3_pci_3c590_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    EL3PCIClass *k = EL3_PCI_CLASS(klass);

    dc->desc = "3Com 3C590 Vortex (PCI, PIO)";
    k->device_id = 0x5900;
    k->model = MODEL_3C590;
    k->has_dma = false;
}

static void el3_pci_3c905_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    EL3PCIClass *k = EL3_PCI_CLASS(klass);

    dc->desc = "3Com 3C905 Boomerang (PCI, descriptor DMA)";
    k->device_id = 0x9050;
    k->model = MODEL_3C905;
    k->has_dma = true;
}

static const TypeInfo el3_pci_types[] = {
    {
        .name          = TYPE_EL3_PCI,
        .parent        = TYPE_PCI_DEVICE,
        .instance_size = sizeof(EL3PCIState),
        .class_size    = sizeof(EL3PCIClass),
        .class_init    = el3_pci_class_init,
        .abstract      = true,
        .interfaces    = (InterfaceInfo[]) {
            { INTERFACE_CONVENTIONAL_PCI_DEVICE },
            { },
        },
    },
    {
        .name          = "3c590",
        .parent        = TYPE_EL3_PCI,
        .class_init    = el3_pci_3c590_class_init,
    },
    {
        .name          = "3c905",
        .parent        = TYPE_EL3_PCI,
        .class_init    = el3_pci_3c905_class_init,
    },
};

DEFINE_TYPES(el3_pci_types)
