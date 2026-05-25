/*
 * 3Com ISA PnP ID Port Bus Implementation
 * 
 * This implements the shared ID port (0x279) used by all 3Com ISA devices.
 * The ID port uses wired-AND logic for multi-device enumeration.
 */

#include "qemu/osdep.h"
#include "hw/isa/isa.h"
#include "hw/qdev-properties.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "trace.h"
#include "3c509-idport.h"

/* Global ID port bus instance */
IDPortBus *g_id_port_bus = NULL;

/* Device interface storage */
static GHashTable *device_interfaces = NULL;

static void id_port_bus_init_interfaces(void)
{
    if (!device_interfaces) {
        device_interfaces = g_hash_table_new(g_direct_hash, g_direct_equal);
    }
}

/* Register a 3C509 device with the ID port bus */
void id_port_register_device(ISA3C509State *dev, IDPortDeviceInterface *iface)
{
    if (!g_id_port_bus) {
        qemu_log("3c509-idport: Warning - no ID port bus available for device registration\n");
        return;
    }
    
    id_port_bus_init_interfaces();
    
    /* Add device to the bus list */
    g_id_port_bus->devices = g_slist_prepend(g_id_port_bus->devices, dev);
    
    /* Store device interface */
    g_hash_table_insert(device_interfaces, dev, iface);
    
    qemu_log("3c509-idport: Registered device %p with ID port bus\n", dev);
}

/* Unregister a 3C509 device from the ID port bus */
void id_port_unregister_device(ISA3C509State *dev)
{
    if (!g_id_port_bus) {
        return;
    }
    
    /* Remove from device list */
    g_id_port_bus->devices = g_slist_remove(g_id_port_bus->devices, dev);
    
    /* Multiple devices can stream simultaneously, no single device tracking needed */
    
    /* Remove interface */
    if (device_interfaces) {
        g_hash_table_remove(device_interfaces, dev);
    }
    
    qemu_log("3c509-idport: Unregistered device %p from ID port bus\n", dev);
}

/* Get the global ID port bus */
IDPortBus *id_port_get_bus(void)
{
    return g_id_port_bus;
}

/* ID port read - aggregate all device contributions with wired-AND */
static uint64_t id_port_bus_read(void *opaque, hwaddr addr, unsigned size)
{
    IDPortBus *bus = opaque;
    uint8_t result = 0xFF;  /* Start with all bits high (idle state) */
    bool any_streaming = false;
    
    if (size != 1) {
        qemu_log("3c509-idport: Invalid read size %d\n", size);
        return 0xFF;
    }
    
    qemu_log("3c509-idport: Read from 0x110 in state %d\n", bus->state);
    
    /* Aggregate contributions from all devices using wired-AND */
    for (GSList *l = bus->devices; l; l = l->next) {
        ISA3C509State *dev = l->data;
        IDPortDeviceInterface *iface = g_hash_table_lookup(device_interfaces, dev);
        
        if (iface && iface->read_contribution) {
            uint8_t device_contribution = iface->read_contribution(dev);
            result &= device_contribution;
            
            /* Check if this device is streaming */
            if (iface->is_streaming && iface->is_streaming(dev)) {
                any_streaming = true;
            }
        }
    }
    
    trace_3c509_idport_read(result, bus->state, any_streaming);
    return result;
}

/* ID port write - broadcast to all devices */
static void id_port_bus_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    IDPortBus *bus = opaque;
    uint8_t byte_val = val & 0xFF;
    
    if (size != 1) {
        qemu_log("3c509-idport: Invalid write size %d\n", size);
        return;
    }
    
    qemu_log("3c509-idport: Write 0x%02x to 0x110 in state %d\n", byte_val, bus->state);
    
    trace_3c509_idport_write(byte_val, bus->state, bus->reset_count, bus->activate_count);
    
    /* Update global bus state */
    switch (bus->state) {
    case ID_BUS_SLEEP:
        if (byte_val == 0x00) {
            bus->reset_count++;
            if (bus->reset_count >= 2) {
                bus->state = ID_BUS_RESET;
                bus->activate_count = 0;
                /* streaming tracking removed */  /* Reset streaming */
                qemu_log("3c509-idport: Bus entered RESET state after 2x 0x00\n");
            }
        } else {
            bus->reset_count = 0;
        }
        break;
        
    case ID_BUS_RESET:
        if (byte_val == 0xFF) {
            bus->activate_count++;
            if (bus->activate_count >= 255) {
                bus->state = ID_BUS_ACTIVATE;
                qemu_log("3c509-idport: Bus entered ACTIVATE state after %d x 0xFF\n", 
                         bus->activate_count);
            }
        } else {
            /* Non-0xFF during activate - back to SLEEP */
            bus->state = ID_BUS_SLEEP;
            bus->reset_count = 0;
            bus->activate_count = 0;
            /* streaming tracking removed */
        }
        break;
        
    case ID_BUS_ACTIVATE:
        if (byte_val == 0x00) {
            /* Reset - back to SLEEP */
            bus->state = ID_BUS_SLEEP;
            bus->reset_count = 1;  /* Count this 0x00 */
            bus->activate_count = 0;
            /* streaming tracking removed */
        }
        /* Other commands handled by individual devices */
        break;
    }
    
    /* Broadcast write to all registered devices */
    for (GSList *l = bus->devices; l; l = l->next) {
        ISA3C509State *dev = l->data;
        IDPortDeviceInterface *iface = g_hash_table_lookup(device_interfaces, dev);
        
        if (iface && iface->process_write) {
            iface->process_write(dev, byte_val);
        }
    }
}

static const MemoryRegionOps id_port_bus_ops = {
    .read = id_port_bus_read,
    .write = id_port_bus_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

/* Device initialization */
static void id_port_bus_realize(DeviceState *dev, Error **errp)
{
    IDPortBus *bus = IDPORT_3C509_BUS(dev);
    ISADevice *isadev = ISA_DEVICE(dev);
    
    /* Initialize bus state */
    bus->state = ID_BUS_SLEEP;
    bus->reset_count = 0;
    bus->activate_count = 0;
    bus->devices = NULL;
    /* streaming tracking removed */
    
    /* Setup ID port I/O region at 0x110 (3Com's proprietary ISA ID port)
     * Note: 3Com uses 0x100-0x3F0 in steps of 0x10, not ISA PnP's 0x279 */
    memory_region_init_io(&bus->io, OBJECT(dev), &id_port_bus_ops,
                          bus, "3c509-idport", 1);
    isa_register_ioport(isadev, &bus->io, 0x110);
    
    /* Set global reference */
    g_id_port_bus = bus;
    
    qemu_log("3c509-idport: ID port bus initialized at 0x110\n");
}

static void id_port_bus_unrealize(DeviceState *dev)
{
    IDPortBus *bus = IDPORT_3C509_BUS(dev);
    
    /* Clean up device list */
    g_slist_free(bus->devices);
    bus->devices = NULL;
    
    /* Clear global reference */
    if (g_id_port_bus == bus) {
        g_id_port_bus = NULL;
    }
}

static void id_port_bus_reset(DeviceState *dev)
{
    IDPortBus *bus = IDPORT_3C509_BUS(dev);
    
    bus->state = ID_BUS_SLEEP;
    bus->reset_count = 0;
    bus->activate_count = 0;
    /* streaming tracking removed */
}

static void id_port_bus_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    
    dc->realize = id_port_bus_realize;
    dc->unrealize = id_port_bus_unrealize;
    device_class_set_legacy_reset(dc, id_port_bus_reset);
    dc->desc = "3Com ISA PnP ID Port Bus";
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo id_port_bus_info = {
    .name = TYPE_IDPORT_3C509_BUS,
    .parent = TYPE_ISA_DEVICE,
    .instance_size = sizeof(IDPortBus),
    .class_init = id_port_bus_class_init,
};

static void id_port_bus_register_types(void)
{
    type_register_static(&id_port_bus_info);
}

type_init(id_port_bus_register_types)