/*
 * QEMU 3Com 3C515 Corkscrew emulation
 *
 * Copyright (c) 2024 QEMU contributors
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 * The 3C515 is an ISA PnP device that uses standard ISA enumeration
 * and configuration through iobase/irq properties, unlike the 3C509
 * which requires custom ID port-based enumeration at a fixed address.
 */

#include "qemu/osdep.h"
#include "hw/isa/isa.h"
#include "hw/qdev-properties.h"
#include "net/net.h"
#include "hw/net/el3_core.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/error-report.h"
#include "system/address-spaces.h"   /* get_system_io: DMA block needs overlap priority */

#define TYPE_ISA_3C515 "3c515"
OBJECT_DECLARE_SIMPLE_TYPE(ISA3C515State, ISA_3C515)

#define ISA_3C515_IO_SIZE       0x20
/* The 3C515 mirrors the Window-0 EEPROM at the +0x2000 ISA alias (cmd 0x200A/data
 * 0x200C). Map it as a separate small region so we don't span the standard ISA
 * ports (COM/floppy/etc.) between 0x20 and 0x2000. */
#define ISA_3C515_EEPROM_ALIAS  0x2000
#define ISA_3C515_EEPROM_SIZE   0x10
/* HARDWARE-TRUE: the real Corkscrew aliases the Boomerang master-control block
 * at iobase+0x400 (Becker 3c515.c `MasterCtrl`: DownListPtr 0x404, UpListPtr
 * 0x418); the card's windowed ISA extent stops at 0x1F, so the former +0x20
 * mapping (the 90x PCI layout) was never decoded by real silicon. On -M pc the
 * PCI host bridge shadows this port range, so the region is registered with
 * overlap priority (see realize). */
#define ISA_3C515_DMA_BASE      0x400
#define ISA_3C515_DMA_SIZE      0x20
#define ISA_3C515_DEFAULT_IOBASE 0x300
#define ISA_3C515_DEFAULT_IRQ   10
/* Practical 16-bit ISA bus-master throughput (bytes/s). A 0-wait-state 16-bit ISA cycle on a
 * Pentium/late-486 chipset is 2 BCLK @ 8.33 MHz = 2 bytes/240ns = 8.33 MB/s peak; minus
 * DRAM-refresh steal + bus-master arbitration/turnaround, ~6 MB/s (~48 Mbit) sustained -- which
 * matches the ~20-40 Mbit the real 3C515 reached (never its marketed 100, ISA being the wall).
 * Slower/wait-state chipsets do less; tunable via the dma_rate property (and the auto-tune
 * milestone, design S7, would measure it per-board). Used to pace el3_core_dma_tx_single. */
#define ISA_3C515_DEFAULT_DMA_RATE (6 * 1024 * 1024)  /* 6 MB/s ~= 48 Mbit (0WS 16-bit ISA) */
#define ISA_3C515_ROM_SIZE       0x4000  /* 16KB ROM */

typedef struct ISA3C515State {
    ISADevice parent_obj;
    EL3Core core;
    MemoryRegion io;
    MemoryRegion rom_io;     /* EEPROM +0x2000 alias region (formerly ROM) */
    MemoryRegion dma_io;     /* bus-master DMA register block at base+0x400 */
    
    uint16_t iobase;
    uint8_t irq;
    uint16_t linkspeed;  /* Mbit/s for the realtiming wire model (10 or 100) */
    qemu_irq irq_line;  /* Actual IRQ line */
    bool pnp_enable;
    uint32_t dma_rate_bps;
    char *romfile;
} ISA3C515State;

/* I/O read handler */
static uint64_t isa_3c515_io_read(void *opaque, hwaddr addr, unsigned size)
{
    ISA3C515State *s = opaque;
    return el3_core_read(&s->core, addr, size);
}

/* I/O write handler */
static void isa_3c515_io_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    ISA3C515State *s = opaque;
    el3_core_write(&s->core, addr, val, size);
}

/* EEPROM alias at iobase+0x2000: forward to el3_core with the +0x2000 offset so the
 * core's 3C515 normalization routes 0x200A/0x200C to the Window-0 EEPROM cmd/data. */
static uint64_t isa_3c515_eeprom_read(void *opaque, hwaddr addr, unsigned size)
{
    ISA3C515State *s = opaque;
    return el3_core_read(&s->core, ISA_3C515_EEPROM_ALIAS + addr, size);
}

static void isa_3c515_eeprom_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    ISA3C515State *s = opaque;
    el3_core_write(&s->core, ISA_3C515_EEPROM_ALIAS + addr, val, size);
}

static const MemoryRegionOps isa_3c515_eeprom_ops = {
    .read = isa_3c515_eeprom_read,
    .write = isa_3c515_eeprom_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 2 },
};

/* Bus-master DMA registers (base+0x400). DownListPtr (0x04)/UpListPtr (0x18) are 32-bit, but
 * a 286 driver writes each as two 16-bit halves -- handle both.
 *
 * HARDWARE-TRUE start semantics: writing a non-zero list pointer starts the engine (Linux
 * 3c515.c start_xmit writes DownListPtr alone; iPXE patches DnNextPtr under stall and writes
 * DnListPtr when it reads 0). With split 16-bit writes the pointer is complete only after the
 * HIGH half, so a 16-bit low-half write only latches; the high-half write (or a full 32-bit
 * write) triggers. StartDmaUp/Down (cmd 0x14) still works as the legacy trigger. A null
 * UpListPtr disarms the up engine (the driver's release path). */
static void isa_3c515_dma_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    ISA3C515State *s = opaque;
    bool dn_go = false, up_go = false;

    switch (addr) {
    case 0x04: /* DownListPtr (low half, or full 32-bit) */
        if (size == 4) {
            s->core.down_list_ptr = (uint32_t)val;
            dn_go = true;
        } else {
            s->core.down_list_ptr = (s->core.down_list_ptr & 0xFFFF0000u) | (val & 0xFFFFu);
        }
        break;
    case 0x06: /* DownListPtr high half -> pointer complete */
        s->core.down_list_ptr = (s->core.down_list_ptr & 0x0000FFFFu) | ((val & 0xFFFFu) << 16);
        dn_go = true;
        break;
    case 0x18: /* UpListPtr (low half, or full 32-bit) */
        if (size == 4) {
            s->core.up_list_ptr = (uint32_t)val;
            up_go = true;
        } else {
            s->core.up_list_ptr = (s->core.up_list_ptr & 0xFFFF0000u) | (val & 0xFFFFu);
        }
        break;
    case 0x1A: /* UpListPtr high half -> pointer complete */
        s->core.up_list_ptr = (s->core.up_list_ptr & 0x0000FFFFu) | ((val & 0xFFFFu) << 16);
        up_go = true;
        break;
    default:
        break;
    }

    if (dn_go && s->core.down_list_ptr && s->core.dma_as) {
        s->core.down_stalled = false;
        el3_core_dma_tx_single(&s->core, s->core.dma_as, s->core.down_list_ptr);
    }
    if (up_go) {
        s->core.up_stalled = false;
        s->core.rx_dma_armed = (s->core.up_list_ptr != 0);
    }
}

static uint64_t isa_3c515_dma_read(void *opaque, hwaddr addr, unsigned size)
{
    ISA3C515State *s = opaque;
    switch (addr) {
    case 0x04: return (size == 4) ? s->core.down_list_ptr : (s->core.down_list_ptr & 0xFFFFu);
    case 0x06: return (s->core.down_list_ptr >> 16) & 0xFFFFu;
    case 0x18: return (size == 4) ? s->core.up_list_ptr : (s->core.up_list_ptr & 0xFFFFu);
    case 0x1A: return (s->core.up_list_ptr >> 16) & 0xFFFFu;
    default: return 0;
    }
}

static const MemoryRegionOps isa_3c515_dma_ops = {
    .read = isa_3c515_dma_read,
    .write = isa_3c515_dma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static const MemoryRegionOps isa_3c515_io_ops = {
    .read = isa_3c515_io_read,
    .write = isa_3c515_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        /* 32-bit FIFO access (offset 0): 386+ drivers stream the TX/RX FIFO with `rep outsd`/
         * `insd`. el3_core handles size==4; see the note in 3c509.c. */
        .max_access_size = 4,
    },
};

/* Forward declaration */
static void isa_3c515_irq_handler(EL3Core *c, bool level);

/* IRQ handler for ISA variant */
static void isa_3c515_irq_handler(EL3Core *c, bool level)
{
    ISA3C515State *s = container_of(c, ISA3C515State, core);
    qemu_set_irq(s->irq_line, level);
}

/* Variant operations for ISA */
static const EL3VariantOps isa_3c515_ops = {
    .irq_set = isa_3c515_irq_handler,
    .has_dma = true,  /* 3C515 has ISA bus master DMA */
};

/* Network client info */
static NetClientInfo net_3c515_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = el3_core_can_receive,
    .receive = el3_core_receive,
    .link_status_changed = el3_core_set_link_status,
};

/* Reset handler */
static void isa_3c515_reset(DeviceState *dev)
{
    ISA3C515State *s = ISA_3C515(dev);
    el3_core_reset(&s->core);
}

/* VMState */
static const VMStateDescription vmstate_3c515 = {
    .name = "3c515",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT(core, ISA3C515State, 1, vmstate_el3_core, EL3Core),
        VMSTATE_UINT16(iobase, ISA3C515State),
        VMSTATE_UINT8(irq, ISA3C515State),
        VMSTATE_BOOL(pnp_enable, ISA3C515State),
        VMSTATE_UINT32(dma_rate_bps, ISA3C515State),
        VMSTATE_END_OF_LIST()
    }
};

/* Device realization */
static void isa_3c515_realize(DeviceState *dev, Error **errp)
{
    ISA3C515State *s = ISA_3C515(dev);
    ISADevice *isa = ISA_DEVICE(dev);
    
    /* Initialize core as 3C515 */
    el3_core_init(&s->core, MODEL_3C515, &isa_3c515_ops);

    /* realtiming wire rate: 100 Mbit -> 80 ns/byte, else 10BaseT -> 800 ns/byte */
    s->core.tx_ns_per_byte = (s->linkspeed >= 100) ? 80 : 800;
    
    /* Setup MAC address */
    qemu_macaddr_default_if_unset(&s->core.conf.macaddr);
    
    /* Create network interface */
    s->core.nic = qemu_new_nic(&net_3c515_info, &s->core.conf,
                                TYPE_ISA_3C515, DEVICE(dev)->id,
                                &dev->mem_reentrancy_guard, &s->core);
    qemu_format_nic_info_str(qemu_get_queue(s->core.nic),
                              s->core.conf.macaddr.a);
    
    /* Initialize EEPROM with MAC address */
    el3_eeprom_init_3c509(&s->core);
    
    /* Setup I/O region */
    memory_region_init_io(&s->io, OBJECT(dev), &isa_3c515_io_ops,
                          s, TYPE_ISA_3C515, ISA_3C515_IO_SIZE);
    isa_register_ioport(isa, &s->io, s->iobase);

    /* EEPROM alias region at iobase+0x2000 (reuses the spare rom_io member). */
    memory_region_init_io(&s->rom_io, OBJECT(dev), &isa_3c515_eeprom_ops,
                          s, "3c515-eeprom", ISA_3C515_EEPROM_SIZE);
    isa_register_ioport(isa, &s->rom_io, s->iobase + ISA_3C515_EEPROM_ALIAS);

    /* Bus-master DMA register block at iobase+0x400; DMA targets guest system memory. */
    s->core.dma_as = &address_space_memory;
    memory_region_init_io(&s->dma_io, OBJECT(dev), &isa_3c515_dma_ops,
                          s, "3c515-dma", ISA_3C515_DMA_SIZE);
    /* Register with OVERLAP PRIORITY 1: on -M pc the PCI host bridge claims a
     * broad I/O window that shadows iobase+0x400 (0x700) after BIOS setup, so a
     * plain isa_register_ioport there never sees the writes -- the reason this
     * block was previously (non-hardware-true) parked at +0x20. An ISA add-in
     * card's decode wins on a real bus; priority 1 models that. */
    memory_region_add_subregion_overlap(get_system_io(),
                                        s->iobase + ISA_3C515_DMA_BASE,
                                        &s->dma_io, 1);

    /* Setup IRQ */
    s->irq_line = isa_get_irq(isa, s->irq);
    
    /* Configure DMA pacing */
    s->core.dma_rate_bps = s->dma_rate_bps;
}

/* Properties */
static const Property isa_3c515_properties[] = {
    DEFINE_PROP_UINT16("iobase", ISA3C515State, iobase, ISA_3C515_DEFAULT_IOBASE),
    DEFINE_PROP_UINT8("irq", ISA3C515State, irq, ISA_3C515_DEFAULT_IRQ),
    DEFINE_PROP_BOOL("realtiming", ISA3C515State, core.realtiming, false),
    DEFINE_PROP_UINT16("linkspeed", ISA3C515State, linkspeed, 100),
    /* R2 (docs/12): suppress the fshDnComplete write-back to CI-test the driver's dual-evidence
     * retirement (retire iff DownListPtr==0). Default on = hardware-true Corkscrew behavior. */
    DEFINE_PROP_BOOL("dn_writeback", ISA3C515State, core.dn_writeback, true),
    DEFINE_PROP_BOOL("pnp", ISA3C515State, pnp_enable, true),
    DEFINE_PROP_UINT32("dma_rate", ISA3C515State, dma_rate_bps, ISA_3C515_DEFAULT_DMA_RATE),
    DEFINE_PROP_STRING("romfile", ISA3C515State, romfile),
    DEFINE_NIC_PROPERTIES(ISA3C515State, core.conf),
};

/* Class initialization */
static void isa_3c515_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    
    dc->realize = isa_3c515_realize;
    dc->legacy_reset = isa_3c515_reset;
    dc->desc = "3Com 3C515-TX ISA PnP Ethernet";
    dc->vmsd = &vmstate_3c515;
    device_class_set_props(dc, isa_3c515_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

/* Type info */
static const TypeInfo isa_3c515_info = {
    .name = TYPE_ISA_3C515,
    .parent = TYPE_ISA_DEVICE,
    .instance_size = sizeof(ISA3C515State),
    .class_init = isa_3c515_class_init,
};

/* Type registration */
static void isa_3c515_register_types(void)
{
    type_register_static(&isa_3c515_info);
}

type_init(isa_3c515_register_types)