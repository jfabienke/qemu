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

#define TYPE_ISA_3C509 "3c509"
OBJECT_DECLARE_SIMPLE_TYPE(ISA3C509State, ISA_3C509)

typedef struct ISA3C509State {
    ISADevice parent_obj;
    EL3Core core;
    MemoryRegion io;
    MemoryRegion id_io;  /* ID port region 0x100-0x10F */
    uint16_t iobase;
    uint8_t irq;
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
        .max_access_size = 2,
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

/* Network client info */
static NetClientInfo net_3c509_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .receive = el3_core_receive,
    .link_status_changed = el3_core_set_link_status,
};

/* Reset handler */
static void isa_3c509_reset(DeviceState *dev)
{
    ISA3C509State *s = ISA_3C509(dev);
    el3_core_reset(&s->core);
}

/* Device realization */
static void isa_3c509_realize(ISADevice *dev, Error **errp)
{
    ISA3C509State *s = ISA_3C509(dev);
    
    /* Initialize core as 3C509B */
    el3_core_init(&s->core, MODEL_3C509B);
    
    /* Setup MAC address */
    qemu_macaddr_default_if_unset(&s->core.conf.macaddr);
    
    /* Create network interface */
    s->core.nic = qemu_new_nic(&net_3c509_info, &s->core.conf,
                                TYPE_ISA_3C509, dev->qdev.id, &s->core);
    qemu_format_nic_info_str(qemu_get_queue(s->core.nic),
                              s->core.conf.macaddr.a);
    
    /* Initialize EEPROM with MAC address */
    el3_eeprom_init_3c509(&s->core);
    
    /* Setup I/O region */
    memory_region_init_io(&s->io, OBJECT(dev), &isa_3c509_io_ops,
                          s, TYPE_ISA_3C509, 0x20);
    isa_register_ioport(dev, &s->io, s->iobase);
    
    /* Setup ID port region (0x100-0x10F) for ISA PnP */
    memory_region_init_io(&s->id_io, OBJECT(dev), &isa_3c509_id_ops,
                          s, "3c509-id", 0x10);
    isa_register_ioport(dev, &s->id_io, 0x100);
    
    /* Setup IRQ */
    s->core.irq = isa_get_irq(dev, s->irq);
}

/* Properties */
static Property isa_3c509_properties[] = {
    DEFINE_PROP_UINT16("iobase", ISA3C509State, iobase, 0x300),
    DEFINE_PROP_UINT8("irq", ISA3C509State, irq, 10),
    DEFINE_NIC_PROPERTIES(ISA3C509State, core.conf),
    DEFINE_PROP_END_OF_LIST(),
};

/* VMState */
static const VMStateDescription vmstate_3c509 = {
    .name = "3c509",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(core.current_window, ISA3C509State),
        VMSTATE_UINT16_2DARRAY(core.windows, ISA3C509State, 
                                EL3_MAX_WINDOWS, EL3_WINDOW_SIZE),
        VMSTATE_UINT16_ARRAY(core.eeprom, ISA3C509State, 64),
        VMSTATE_UINT16(core.status, ISA3C509State),
        VMSTATE_UINT16(core.command, ISA3C509State),
        VMSTATE_END_OF_LIST()
    }
};

/* Class initialization */
static void isa_3c509_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ISADeviceClass *ic = ISA_DEVICE_CLASS(klass);
    
    ic->realize = isa_3c509_realize;
    dc->reset = isa_3c509_reset;
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
