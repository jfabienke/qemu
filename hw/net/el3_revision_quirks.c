#include "qemu/osdep.h"
#include "hw/net/el3_core.h"

/* Revision-specific quirks and capabilities */
typedef struct {
    uint16_t device_id;
    uint8_t pci_revision;
    uint16_t tx_fifo_size;
    uint16_t rx_fifo_size;
    bool has_pause_support;
    bool has_power_management;
    bool needs_tx_reset_quirk;
    bool no_pause_advertisement;
} EL3RevisionQuirks;

static const EL3RevisionQuirks el3_revision_quirks[] = {
    [MODEL_3C905B] = {
        .device_id = 0x9055,
        .pci_revision = 0x30,
        .tx_fifo_size = 2048,
        .rx_fifo_size = 2048,
        .has_pause_support = false,
        .has_power_management = false,
        .needs_tx_reset_quirk = true,
        .no_pause_advertisement = true
    },
    [MODEL_3C905C] = {
        .device_id = 0x9200,
        .pci_revision = 0x78,
        .tx_fifo_size = 4096,
        .rx_fifo_size = 4096,
        .has_pause_support = true,
        .has_power_management = true,
        .needs_tx_reset_quirk = false,
        .no_pause_advertisement = false
    }
};

/* Initialize capabilities based on model */
void el3_core_init_caps(EL3Core *c, EL3Model model)
{
    const EL3RevisionQuirks *quirks = &el3_revision_quirks[model];
    
    c->model = model;
    c->caps.has_mii = true;
    c->caps.has_bus_master = true;
    c->caps.has_full_duplex = true;
    c->caps.is_100mbit = true;
    c->caps.has_vlan_support = false;
    c->caps.ram_bytes = 0x20000; /* 128KB for both variants */
    c->caps.tx_fifo_bytes = quirks->tx_fifo_size;
    c->caps.rx_fifo_bytes = quirks->rx_fifo_size;
    c->caps.max_windows = 8;
    
    /* Initialize FIFOs with revision-specific sizes */
    el3_fifo_init(&c->tx_fifo, quirks->tx_fifo_size, 0x07);
    el3_fifo_init(&c->rx_fifo, quirks->rx_fifo_size, 0x07);
}

/* Apply revision-specific quirks during reset */
void el3_core_apply_revision_quirks(EL3Core *c)
{
    const EL3RevisionQuirks *quirks = &el3_revision_quirks[c->model];
    
    /* Set PCI revision ID */
    c->windows[7][W7_PRODUCT_ID] = (c->windows[7][W7_PRODUCT_ID] & 0xFF00) | 
                                   quirks->pci_revision;
    
    /* Set device ID */
    c->windows[7][W7_DEVICE_ID] = quirks->device_id;
    
    /* Initialize PHY registers with revision-specific capabilities */
    if (quirks->no_pause_advertisement) {
        c->phy_regs[MII_ANAR] &= ~ADVERTISE_PAUSE;
    } else {
        c->phy_regs[MII_ANAR] |= ADVERTISE_PAUSE;
    }
}

/* Handle TX reset quirk for 3C905B */
void el3_handle_tx_reset_quirk(EL3Core *c)
{
    const EL3RevisionQuirks *quirks = &el3_revision_quirks[c->model];
    
    if (quirks->needs_tx_reset_quirk) {
        /* Perform second TX reset or add delay */
        el3_core_process_command(c, CMD_TX_RESET);
        /* Optional: Add timer delay here if needed */
    }
}

/* Handle flow control configuration */
void el3_set_flow_control(EL3Core *c, bool enable)
{
    const EL3RevisionQuirks *quirks = &el3_revision_quirks[c->model];
    
    if (!quirks->has_pause_support && enable) {
        /* 3C905B doesn't support pause frames - ignore enable request */
        return;
    }
    
    /* Configure flow control in PHY registers */
    if (enable) {
        c->phy_regs[MII_ANAR] |= ADVERTISE_PAUSE;
    } else {
        c->phy_regs[MII_ANAR] &= ~ADVERTISE_PAUSE;
    }
}

/* Handle power management capability */
bool el3_has_power_management(EL3Core *c)
{
    const EL3RevisionQuirks *quirks = &el3_revision_quirks[c->model];
    return quirks->has_power_management;
}

/* Handle interrupt source differences */
void el3_update_interrupt_sources(EL3Core *c)
{
    const EL3RevisionQuirks *quirks = &el3_revision_quirks[c->model];
    
    /* Mask out unsupported interrupt sources */
    if (!quirks->has_pause_support) {
        c->status &= ~STAT_PAUSE_EVENT; /* Assuming such a status bit exists */
    }
    
    /* Update other interrupt-related behavior based on revision */
    el3_update_irq(c);
}

/* Property-based selection function */
EL3Model el3_parse_revision_property(const char *revision)
{
    if (revision == NULL) {
        return MODEL_3C905C; /* Default to 3C905C */
    }
    
    if (strcmp(revision, "905b") == 0) {
        return MODEL_3C905B;
    } else if (strcmp(revision, "905c") == 0) {
        return MODEL_3C905C;
    }
    
    /* Invalid revision - default to 3C905C */
    return MODEL_3C905C;
}

/* Integration with existing device state */
void el3_core_set_revision(EL3Core *c, const char *revision)
{
    EL3Model model = el3_parse_revision_property(revision);
    el3_core_init_caps(c, model);
}