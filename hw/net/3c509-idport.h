/*
 * 3Com ISA PnP ID Port Bus - Shared bus for multiple 3C509 devices
 * 
 * The ID port (0x110 by default) is shared across all 3Com ISA devices on the bus.
 * This implements the wired-AND semantics required for proper enumeration
 * of multiple 3C509B cards. 3Com uses 0x100-0x3F0 in steps of 0x10.
 */

#ifndef HW_NET_3C509_IDPORT_H
#define HW_NET_3C509_IDPORT_H

#include "qemu/osdep.h"
#include "hw/isa/isa.h"

#define TYPE_IDPORT_3C509_BUS "idport-3c509-bus"

typedef struct ISA3C509State ISA3C509State;

/* ID port bus states */
typedef enum {
    ID_BUS_SLEEP = 0,
    ID_BUS_RESET = 1,
    ID_BUS_ACTIVATE = 2,
} IDPortBusState;

/* Device interface for ID port operations */
typedef struct IDPortDeviceInterface {
    /* Called when device should contribute to ID port read */
    uint8_t (*read_contribution)(ISA3C509State *dev);
    
    /* Called when device should process ID port write */
    void (*process_write)(ISA3C509State *dev, uint8_t val);
    
    /* Called to check if device is actively streaming EEPROM */
    bool (*is_streaming)(ISA3C509State *dev);
    
    /* Get device's current contention bits */
    uint8_t (*get_contention_bits)(ISA3C509State *dev);
} IDPortDeviceInterface;

typedef struct IDPortBus {
    ISADevice parent_obj;
    
    /* Memory region for ID port */
    MemoryRegion io;
    
    /* Bus state */
    IDPortBusState state;
    uint8_t reset_count;
    uint16_t activate_count;
    
    /* List of registered 3C509 devices */
    GSList *devices;
    
    /* Multiple devices can stream simultaneously with wired-AND aggregation */
} IDPortBus;

OBJECT_DECLARE_SIMPLE_TYPE(IDPortBus, IDPORT_3C509_BUS)

/* Global ID port bus instance */
extern IDPortBus *g_id_port_bus;

/* Interface functions */
void id_port_register_device(ISA3C509State *dev, IDPortDeviceInterface *iface);
void id_port_unregister_device(ISA3C509State *dev);
IDPortBus *id_port_get_bus(void);

#endif /* HW_NET_3C509_IDPORT_H */