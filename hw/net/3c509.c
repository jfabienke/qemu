/*
 * QEMU 3Com 3C509/3C509B EtherLink III emulation
 *
 * Copyright (c) 2024 QEMU contributors
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "hw/isa/isa.h"
#include "hw/qdev-properties.h"
#include "net/net.h"
#include "hw/net/el3_core.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/error-report.h"

#define TYPE_ISA_3C509 "3c509"
OBJECT_DECLARE_SIMPLE_TYPE(ISA3C509State, ISA_3C509)

#define ISA_3C509_IO_SIZE       0x20
#define ISA_3C509_ID_PORT_SIZE  0x10
#define ISA_3C509_ID_PORT_BASE  0x110   /* conventional 3c509 ID port (drivers probe 0x110) */
#define ISA_3C509_DEFAULT_IOBASE 0x300
#define ISA_3C509_DEFAULT_IRQ   10
#define ISA_3C509_EEPROM_SIZE   64

typedef struct ISA3C509State {
    ISADevice parent_obj;
    EL3Core core;
    MemoryRegion io;
    MemoryRegion id_io;  /* ID port region 0x100-0x10F */
    uint16_t iobase;
    uint8_t irq;
    uint16_t linkspeed;  /* Mbit/s for the realtiming wire model (10 or 100) */
    qemu_irq irq_line;  /* Actual IRQ line */
    
    /* VMState compatibility fields */
    uint8_t current_window_old;
    uint16_t windows_old[EL3_MAX_WINDOWS][EL3_WINDOW_SIZE];
    uint16_t eeprom_old[ISA_3C509_EEPROM_SIZE];
    uint16_t status_old;
    uint16_t command_old;
    uint16_t iobase_old;
    uint8_t irq_old;
} ISA3C509State;

/* I/O read handler */
static uint64_t isa_3c509_io_read(void *opaque, hwaddr addr, unsigned size)
{
    ISA3C509State *s = opaque;
    return el3_core_read(&s->core, addr, size);
}

/* I/O write handler */
static void isa_3c509_io_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    ISA3C509State *s = opaque;
    el3_core_write(&s->core, addr, val, size);
}

static const MemoryRegionOps isa_3c509_io_ops = {
    .read = isa_3c509_io_read,
    .write = isa_3c509_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        /* 32-bit: the TX/RX FIFO (offset 0) is a dword register; 386+ drivers stream it with
         * `rep outsd`/`insd`. Without this QEMU splits each dword and the high word lands on
         * register 0x02, corrupting every frame. el3_core handles size==4 on the FIFO. */
        .max_access_size = 4,
    },
};

/* ID port read handler (0x100-0x10F) */
static uint64_t isa_3c509_id_read(void *opaque, hwaddr addr, unsigned size)
{
    ISA3C509State *s = opaque;
    return el3_id_port_read(&s->core);
}

/* ID port write handler (0x100-0x10F) */
static void isa_3c509_id_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    ISA3C509State *s = opaque;
    el3_id_port_write(&s->core, val & 0xFF);
}

static const MemoryRegionOps isa_3c509_id_ops = {
    .read = isa_3c509_id_read,
    .write = isa_3c509_id_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

/* Forward declaration */
static void isa_3c509_irq_handler(EL3Core *c, bool level);

/* Variant operations for ISA */
static const EL3VariantOps isa_3c509_ops = {
    .irq_set = isa_3c509_irq_handler,
    .has_dma = false,
};

/* Network client info */
static NetClientInfo net_3c509_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    /* Backpressure RX (match the 3c515 path): queue at send time when the RX FIFO can't hold
     * another max frame instead of dropping host-speed slirp bursts on overflow. NOTE: slirp's
     * no-callback send can still drop on the net-queue flush path, so this reduces but does not
     * eliminate burst loss. */
    .can_receive = el3_core_can_receive,
    .receive = el3_core_receive,
    .link_status_changed = el3_core_set_link_status,
};

/* Reset handler */
static void isa_3c509_reset(DeviceState *dev)
{
    ISA3C509State *s = ISA_3C509(dev);
    el3_core_reset(&s->core);
}

/* Predicate function for version 1 fields */
static bool G_GNUC_UNUSED only_v1(void *opaque, int version_id)
{
    return version_id == 1;
}

/*
 * VMState version 1 field layout compatibility handler
 *
 * In version 1, the VMState fields were stored directly in ISA3C509State
 * rather than in the embedded core structure. This handler copies those
 * old fields into their proper locations within the core structure when
 * loading a version 1 snapshot.
 *
 * Note: version 2 does not migrate iobase or irq properties. These are
 * device configuration properties that are handled by the standard device
 * property migration mechanism.
 */
static int isa_3c509_post_load(void *opaque, int version_id)
{
    ISA3C509State *s = opaque;
    
    if (version_id == 1) {
        /* Validate current_window range */
        if (s->current_window_old >= EL3_MAX_WINDOWS) {
            error_report("%s: invalid current_window %u (max %u)",
                         __func__, s->current_window_old, EL3_MAX_WINDOWS - 1);
            return -EINVAL;
        }
        
        /* Copy old fields to core structure - preserve runtime state */
        s->core.current_window = s->current_window_old;
        
        /* Compile-time size checks */
        QEMU_BUILD_BUG_ON(sizeof(s->core.windows) != sizeof(s->windows_old));
        QEMU_BUILD_BUG_ON(sizeof(s->core.eeprom) != sizeof(s->eeprom_old));
        
        memcpy(s->core.windows, s->windows_old, sizeof(s->core.windows));
        memcpy(s->core.eeprom, s->eeprom_old, sizeof(s->core.eeprom));
        
        s->core.status = s->status_old;
        s->core.command = s->command_old;
    }
    
    return 0;
}

/* IRQ handler for ISA variant */
static void isa_3c509_irq_handler(EL3Core *c, bool level)
{
    ISA3C509State *s = container_of(c, ISA3C509State, core);
    qemu_set_irq(s->irq_line, level);
}

/* VMState */
static const VMStateDescription vmstate_3c509 = {
    .name = "3c509",
    .version_id = 2,
    .minimum_version_id = 1,
    .post_load = isa_3c509_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT(core, ISA3C509State, 2, vmstate_el3_core, EL3Core),
        
        /* Legacy fields removed - migration from v1 not supported */
        
        VMSTATE_END_OF_LIST()
    }
};

/* Device realization */
static void isa_3c509_realize(DeviceState *dev, Error **errp)
{
    ISA3C509State *s = ISA_3C509(dev);
    ISADevice *isa = ISA_DEVICE(dev);
    
    /* Initialize core as 3C509B */
    el3_core_init(&s->core, MODEL_3C509B, &isa_3c509_ops);

    /* realtiming wire rate: 100 Mbit -> 80 ns/byte, else 10BaseT -> 800 ns/byte */
    s->core.tx_ns_per_byte = (s->linkspeed >= 100) ? 80 : 800;
    
    /* Setup MAC address */
    qemu_macaddr_default_if_unset(&s->core.conf.macaddr);
    
    /* Create network interface */
    s->core.nic = qemu_new_nic(&net_3c509_info, &s->core.conf,
                                TYPE_ISA_3C509, DEVICE(dev)->id,
                                &dev->mem_reentrancy_guard, &s->core);
    qemu_format_nic_info_str(qemu_get_queue(s->core.nic),
                              s->core.conf.macaddr.a);
    
    /* Initialize EEPROM with MAC address */
    el3_eeprom_init_3c509(&s->core);
    
    /* Setup I/O region */
    memory_region_init_io(&s->io, OBJECT(dev), &isa_3c509_io_ops,
                          s, TYPE_ISA_3C509, ISA_3C509_IO_SIZE);
    isa_register_ioport(isa, &s->io, s->iobase);
    
    /* Setup ID port region (0x100-0x10F) for ISA PnP */
    memory_region_init_io(&s->id_io, OBJECT(dev), &isa_3c509_id_ops,
                          s, "3c509-id", ISA_3C509_ID_PORT_SIZE);
    isa_register_ioport(isa, &s->id_io, ISA_3C509_ID_PORT_BASE);
    
    /* Setup IRQ */
    s->irq_line = isa_get_irq(isa, s->irq);
}

/* Properties */
static const Property isa_3c509_properties[] = {
    DEFINE_PROP_UINT16("iobase", ISA3C509State, iobase, ISA_3C509_DEFAULT_IOBASE),
    DEFINE_PROP_UINT8("irq", ISA3C509State, irq, ISA_3C509_DEFAULT_IRQ),
    /* realtiming=on models hardware delays (TX wire time, EEPROM busy, CmdInProgress) off the
     * virtual clock -- run the guest under -icount. Default off = instant (fast). */
    DEFINE_PROP_BOOL("realtiming", ISA3C509State, core.realtiming, false),
    DEFINE_PROP_UINT16("linkspeed", ISA3C509State, linkspeed, 10),
    DEFINE_NIC_PROPERTIES(ISA3C509State, core.conf),
};

/* Class initialization */
static void isa_3c509_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    
    dc->realize = isa_3c509_realize;
    dc->legacy_reset = isa_3c509_reset;
    dc->desc = "3Com 3C509B EtherLink III";
    dc->vmsd = &vmstate_3c509;
    device_class_set_props(dc, isa_3c509_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

/* Type info */
static const TypeInfo isa_3c509_info = {
    .name = TYPE_ISA_3C509,
    .parent = TYPE_ISA_DEVICE,
    .instance_size = sizeof(ISA3C509State),
    .class_init = isa_3c509_class_init,
};

/* Type registration */
static void isa_3c509_register_types(void)
{
    type_register_static(&isa_3c509_info);
}

type_init(isa_3c509_register_types)