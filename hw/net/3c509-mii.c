#include "qemu/osdep.h"
#include "hw/net/el3_core.h"
#include "hw/net/3c509-mii.h"
#include "hw/net/mii.h"
#include "qemu/timer.h"
#include "net/eth.h"
#include "trace.h"

/* MII register definitions */
#define MII_MAX_PHY_ADDR 31
#define MII_MAX_REG_ADDR 31

/* PHY identifiers for 3C509B */
#define MII_PHY_ID1_3COM    0x0000  /* 3Com OUI bits 3-18 */
#define MII_PHY_ID2_3COM    0x6097  /* 3Com OUI bits 19-24 + model */

/* Auto-negotiation advertisement bits */
#define MII_ANAR_100BASETX_FD   (1 << 8)
#define MII_ANAR_100BASETX_HD   (1 << 7)
#define MII_ANAR_10BASET_FD     (1 << 6)
#define MII_ANAR_10BASET_HD     (1 << 5)
#define MII_ANAR_SELECTOR       0x001F

/* Auto-negotiation link partner ability bits */
#define MII_ANLPAR_100BASETX_FD (1 << 8)
#define MII_ANLPAR_100BASETX_HD (1 << 7)
#define MII_ANLPAR_10BASET_FD   (1 << 6)
#define MII_ANLPAR_10BASET_HD   (1 << 5)
#define MII_ANLPAR_SELECTOR     0x001F

/* Extended status register bits */
#define MII_ESTATUS_1000BASET_FD (1 << 13)
#define MII_ESTATUS_1000BASET_HD (1 << 12)

/* PHY-specific registers for 3Com PHY */
#define MII_3COM_PCR            16 /* PHY Control Register */
#define MII_3COM_AUX_CTRL       17 /* Auxiliary Control Register */
#define MII_3COM_AUX_STATUS     18 /* Auxiliary Status Register */

/* MII Management Interface State Machine */
typedef enum {
    MII_STATE_IDLE,
    MII_STATE_READ,
    MII_STATE_WRITE
} MIIState;

/* MII Management Frame Protocol */
typedef enum {
    MII_OP_READ = 2,
    MII_OP_WRITE = 1
} MIIOperation;

/* PHY capabilities */
typedef struct {
    bool has_10mbit;
    bool has_100mbit;
    bool has_full_duplex;
    bool has_auto_negotiation;
} PHYCapabilities;

/* MII management interface */
typedef struct {
    MIIState state;
    uint8_t phy_addr;
    uint8_t reg_addr;
    uint16_t data;
    int bit_count;
    MIIOperation operation;
    QEMUTimer *timer;
    uint64_t operation_time_ns;
} MIIMgmtInterface;

/* PHY register state */
typedef struct {
    uint16_t regs[32];
    PHYCapabilities caps;
    bool link_up;
    uint64_t link_up_time_ns;
} PHYState;

/* Trace points */
#define TRACE_MII_READ(phy_addr, reg_addr, data) \
    trace_el3_mii_read(phy_addr, reg_addr, data)
#define TRACE_MII_WRITE(phy_addr, reg_addr, data) \
    trace_el3_mii_write(phy_addr, reg_addr, data)
#define TRACE_MII_OP_START(op, phy_addr, reg_addr) \
    trace_el3_mii_op_start(op, phy_addr, reg_addr)
#define TRACE_MII_OP_COMPLETE(op, phy_addr, reg_addr, data) \
    trace_el3_mii_op_complete(op, phy_addr, reg_addr, data)
#define TRACE_PHY_LINK_CHANGE(link_up) \
    trace_el3_phy_link_change(link_up)
#define TRACE_PHY_AUTONEG_COMPLETE() \
    trace_el3_phy_autoneg_complete()

/* Forward declarations */
static void mii_operation_complete(void *opaque);
static void phy_update_link_status(EL3Core *c);
static void phy_start_autonegotiation(EL3Core *c);

/* Initialize PHY capabilities based on model */
static void phy_init_capabilities(EL3Core *c)
{
    PHYState *phy = &c->phy;
    
    /* Default capabilities for 3C509B */
    phy->caps.has_10mbit = true;
    phy->caps.has_100mbit = false;
    phy->caps.has_full_duplex = false;
    phy->caps.has_auto_negotiation = true;
    
    /* Enhanced capabilities for 3C59x/3C905 models */
    if (c->caps.has_mii) {
        phy->caps.has_100mbit = c->caps.is_100mbit;
        phy->caps.has_full_duplex = c->caps.has_full_duplex;
    }
}

/* Initialize PHY registers with realistic values */
static void phy_init_registers(EL3Core *c)
{
    PHYState *phy = &c->phy;
    
    /* PHY Identifier Registers */
    phy->regs[MII_PHYID1] = MII_PHY_ID1_3COM;
    phy->regs[MII_PHYID2] = MII_PHY_ID2_3COM;
    
    /* Basic Mode Control Register */
    phy->regs[MII_BMCR] = 0x0000;
    
    /* Basic Mode Status Register */
    phy->regs[MII_BMSR] = MII_BMSR_EXTCAP | MII_BMSR_MFPS | MII_BMSR_AUTONEG |
                         MII_BMSR_10T_HD | MII_BMSR_10T_FD;
    
    if (phy->caps.has_100mbit) {
        phy->regs[MII_BMSR] |= MII_BMSR_100TX_HD | MII_BMSR_100TX_FD;
    }
    
    /* Auto-Negotiation Advertisement Register */
    phy->regs[MII_ANAR] = MII_ANAR_SELECTOR | MII_ANAR_10BASET_HD;
    
    if (phy->caps.has_full_duplex) {
        phy->regs[MII_ANAR] |= MII_ANAR_10BASET_FD;
    }
    
    if (phy->caps.has_100mbit) {
        phy->regs[MII_ANAR] |= MII_ANAR_100BASETX_HD;
        if (phy->caps.has_full_duplex) {
            phy->regs[MII_ANAR] |= MII_ANAR_100BASETX_FD;
        }
    }
    
    /* Auto-Negotiation Link Partner Ability Register */
    phy->regs[MII_ANLPAR] = 0x0000;
    
    /* Auto-Negotiation Expansion Register */
    phy->regs[MII_ANER] = 0x0000;
    
    /* Extended Status Register */
    phy->regs[MII_EXTSTAT] = 0x0000;
    
    if (c->caps.has_vlan_support) {
        phy->regs[MII_EXTSTAT] |= MII_ESTATUS_1000BASET_HD;
        if (phy->caps.has_full_duplex) {
            phy->regs[MII_EXTSTAT] |= MII_ESTATUS_1000BASET_FD;
        }
    }
}

/* Initialize MII management interface */
static void mii_init_interface(EL3Core *c)
{
    MIIMgmtInterface *mii = &c->mii;
    
    mii->state = MII_STATE_IDLE;
    mii->phy_addr = 0;
    mii->reg_addr = 0;
    mii->data = 0;
    mii->bit_count = 0;
    mii->operation = MII_OP_READ;
    mii->operation_time_ns = 100000; /* 100 microseconds for MII operations */
    
    if (mii->timer) {
        timer_free(mii->timer);
    }
    mii->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, mii_operation_complete, c);
}

/* Reset PHY to power-on defaults */
void el3_phy_reset(EL3Core *c)
{
    phy_init_capabilities(c);
    phy_init_registers(c);
    mii_init_interface(c);
    
    /* Initially link is down */
    c->phy.link_up = false;
    c->phy.link_up_time_ns = 0;
    
    /* Update NET_DIAG register */
    c->windows[4][W4_NET_DIAG >> 1] &= ~NET_DIAG_LINK_BEAT;
}

/* Handle PHY control register writes */
static void phy_write_control(EL3Core *c, uint16_t value)
{
    PHYState *phy = &c->phy;
    uint16_t old_value = phy->regs[MII_BMCR];
    
    /* Handle reset bit */
    if (value & MII_BMCR_RESET) {
        phy_init_registers(c);
        phy->regs[MII_BMCR] &= ~MII_BMCR_RESET; /* Self-clearing */
        return;
    }
    
    /* Handle loopback */
    if ((value & MII_BMCR_LOOPBACK) != (old_value & MII_BMCR_LOOPBACK)) {
        if (value & MII_BMCR_LOOPBACK) {
            /* Enable loopback */
            phy->regs[MII_BMCR] |= MII_BMCR_LOOPBACK;
        } else {
            /* Disable loopback */
            phy->regs[MII_BMCR] &= ~MII_BMCR_LOOPBACK;
        }
    }
    
    /* Handle speed selection */
    if (phy->caps.has_100mbit) {
        bool new_100mbit = (value & MII_BMCR_SPEED100) != 0;
        bool old_100mbit = (old_value & MII_BMCR_SPEED100) != 0;
        
        if (new_100mbit != old_100mbit) {
            if (new_100mbit) {
                phy->regs[MII_BMCR] |= MII_BMCR_SPEED100;
            } else {
                phy->regs[MII_BMCR] &= ~MII_BMCR_SPEED100;
            }
            /* Speed change affects link status */
            phy->link_up = false;
            phy_update_link_status(c);
        }
    }
    
    /* Handle duplex mode */
    if (phy->caps.has_full_duplex) {
        bool new_fd = (value & MII_BMCR_FD) != 0;
        bool old_fd = (old_value & MII_BMCR_FD) != 0;
        
        if (new_fd != old_fd) {
            if (new_fd) {
                phy->regs[MII_BMCR] |= MII_BMCR_FD;
            } else {
                phy->regs[MII_BMCR] &= ~MII_BMCR_FD;
            }
        }
    }
    
    /* Handle auto-negotiation */
    bool new_aneg = (value & MII_BMCR_AUTOEN) != 0;
    bool old_aneg = (old_value & MII_BMCR_AUTOEN) != 0;
    
    if (new_aneg && !old_aneg) {
        /* Enable auto-negotiation */
        phy->regs[MII_BMCR] |= MII_BMCR_AUTOEN;
        phy_start_autonegotiation(c);
    } else if (!new_aneg && old_aneg) {
        /* Disable auto-negotiation */
        phy->regs[MII_BMCR] &= ~MII_BMCR_AUTOEN;
        phy->regs[MII_BMSR] &= ~MII_BMSR_AN_COMP;
    }
    
    /* Handle auto-negotiation restart */
    if (value & MII_BMCR_ANRESTART) {
        phy->regs[MII_BMCR] |= MII_BMCR_ANRESTART;
        phy_start_autonegotiation(c);
        phy->regs[MII_BMCR] &= ~MII_BMCR_ANRESTART; /* Self-clearing */
    }
    
    /* Preserve other bits */
    phy->regs[MII_BMCR] = (phy->regs[MII_BMCR] & ~(MII_BMCR_LOOPBACK | 
                                                  MII_BMCR_SPEED100 |
                                                  MII_BMCR_AUTOEN |
                                                  MII_BMCR_FD)) |
                         (value & (MII_BMCR_LOOPBACK | 
                                   MII_BMCR_SPEED100 |
                                   MII_BMCR_AUTOEN |
                                   MII_BMCR_FD));
}

/* Handle PHY status register reads */
static uint16_t phy_read_status(EL3Core *c)
{
    PHYState *phy = &c->phy;
    uint16_t value = phy->regs[MII_BMSR];
    
    /* Clear latched bits on read */
    phy->regs[MII_BMSR] &= ~(MII_BMSR_JABBER | MII_BMSR_RFAULT);
    
    return value;
}

/* Start auto-negotiation process */
static void phy_start_autonegotiation(EL3Core *c)
{
    PHYState *phy = &c->phy;
    
    /* Set auto-negotiation in progress */
    phy->regs[MII_BMSR] &= ~MII_BMSR_AN_COMP;
    
    /* Schedule completion after realistic delay (500ms) */
    timer_mod_ns(c->mii.timer, 
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 500000000);
}

/* Complete auto-negotiation */
static void phy_complete_autonegotiation(EL3Core *c)
{
    PHYState *phy = &c->phy;
    
    /* Set negotiation complete */
    phy->regs[MII_BMSR] |= MII_BMSR_AN_COMP;
    
    /* Set link partner abilities based on our advertisement */
    phy->regs[MII_ANLPAR] = phy->regs[MII_ANAR] | MII_ANLPAR_ACK;
    
    /* Bring link up */
    phy->link_up = true;
    phy->link_up_time_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    
    phy_update_link_status(c);
    
    TRACE_PHY_AUTONEG_COMPLETE();
}

/* Update link status in core registers */
static void phy_update_link_status(EL3Core *c)
{
    PHYState *phy = &c->phy;
    
    if (phy->link_up) {
        c->windows[4][W4_NET_DIAG >> 1] |= NET_DIAG_LINK_BEAT;
    } else {
        c->windows[4][W4_NET_DIAG >> 1] &= ~NET_DIAG_LINK_BEAT;
    }
    
    /* Update core link status */
    el3_core_set_link_status(qemu_get_queue(c->nic));
}

/* MII operation completion callback */
static void mii_operation_complete(void *opaque)
{
    EL3Core *c = opaque;
    MIIMgmtInterface *mii = &c->mii;
    PHYState *phy = &c->phy;
    
    switch (mii->state) {
    case MII_STATE_READ:
        if (mii->phy_addr == 0 && mii->reg_addr < 32) {
            mii->data = phy->regs[mii->reg_addr];
        } else {
            mii->data = 0xFFFF; /* Invalid PHY or register */
        }
        TRACE_MII_OP_COMPLETE("READ", mii->phy_addr, mii->reg_addr, mii->data);
        break;
        
    case MII_STATE_WRITE:
        if (mii->phy_addr == 0 && mii->reg_addr < 32) {
            switch (mii->reg_addr) {
            case MII_BMCR:
                phy_write_control(c, mii->data);
                break;
            default:
                phy->regs[mii->reg_addr] = mii->data;
                break;
            }
        }
        TRACE_MII_OP_COMPLETE("WRITE", mii->phy_addr, mii->reg_addr, mii->data);
        break;
        
    default:
        break;
    }
    
    mii->state = MII_STATE_IDLE;
}

/* Start MII management operation */
void el3_mii_start_operation(EL3Core *c, uint8_t phy_addr, uint8_t reg_addr, 
                             MIIOperation operation, uint16_t data)
{
    MIIMgmtInterface *mii = &c->miI;
    
    if (!c->caps.has_mii) {
        return;
    }
    
    /* Validate PHY address */
    if (phy_addr > MII_MAX_PHY_ADDR) {
        return;
    }
    
    /* Validate register address */
    if (reg_addr > MII_MAX_REG_ADDR) {
        return;
    }
    
    mii->phy_addr = phy_addr;
    mii->reg_addr = reg_addr;
    mii->operation = operation;
    mii->data = data;
    
    TRACE_MII_OP_START(operation == MII_OP_READ ? "READ" : "WRITE", 
                       phy_addr, reg_addr);
    
    switch (operation) {
    case MII_OP_READ:
        mii->state = MII_STATE_READ;
        break;
    case MII_OP_WRITE:
        mii->state = MII_STATE_WRITE;
        break;
    }
    
    /* Schedule operation completion */
    timer_mod_ns(mii->timer, 
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + mii->operation_time_ns);
}

/* Read MII management data */
uint16_t el3_mii_read_data(EL3Core *c)
{
    MIIMgmtInterface *mii = &c->mii;
    
    if (!c->caps.has_mii) {
        return 0xFFFF;
    }
    
    if (mii->state == MII_STATE_READ) {
        return mii->data;
    }
    
    return 0xFFFF;
}

/* Check if MII operation is in progress */
bool el3_mii_is_busy(EL3Core *c)
{
    MIIMgmtInterface *mii = &c->mii;
    
    if (!c->caps.has_mii) {
        return false;
    }
    
    return (mii->state != MII_STATE_IDLE);
}

/* Set PHY link status */
void el3_phy_set_link(EL3Core *c, bool link_up)
{
    PHYState *phy = &c->phy;
    
    if (!c->caps.has_mii) {
        return;
    }
    
    if (phy->link_up != link_up) {
        phy->link_up = link_up;
        phy->link_up_time_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        
        if (link_up) {
            /* If auto-negotiation is enabled, restart it when link comes up */
            if (phy->regs[MII_BMCR] & MII_BMCR_AUTOEN) {
                phy_start_autonegotiation(c);
            }
        } else {
            /* Clear auto-negotiation complete when link goes down */
            phy->regs[MII_BMSR] &= ~MII_BMSR_AN_COMP;
        }
        
        phy_update_link_status(c);
        TRACE_PHY_LINK_CHANGE(link_up);
    }
}

/* Get current PHY link status */
bool el3_phy_get_link(EL3Core *c)
{
    if (!c->caps.has_mii) {
        return true; /* Assume link is always up for non-MII models */
    }
    
    return c->phy.link_up;
}

/* Get PHY speed (in Mbps) */
int el3_phy_get_speed(EL3Core *c)
{
    if (!c->caps.has_mii) {
        return 10; /* Default to 10 Mbps for non-MII models */
    }
    
    PHYState *phy = &c->phy;
    
    /* If auto-negotiation is complete, use negotiated speed */
    if (phy->regs[MII_BMSR] & MII_BMSR_AN_COMP) {
        /* Check link partner abilities */
        if (phy->caps.has_100mbit && 
            (phy->regs[MII_ANLPAR] & MII_ANLPAR_TX)) {
            return 100;
        }
        return 10;
    }
    
    /* Otherwise use forced speed */
    if (phy->caps.has_100mbit && 
        (phy->regs[MII_BMCR] & MII_BMCR_SPEED100)) {
        return 100;
    }
    return 10;
}

/* Get PHY duplex mode */
bool el3_phy_get_duplex(EL3Core *c)
{
    if (!c->caps.has_mii) {
        return false; /* Default to half duplex for non-MII models */
    }
    
    PHYState *phy = &c->phy;
    
    /* If auto-negotiation is complete, use negotiated duplex */
    if (phy->regs[MII_BMSR] & MII_BMSR_AN_COMP) {
        if (phy->caps.has_full_duplex) {
            if ((phy->regs[MII_BMCR] & MII_BMCR_SPEED100) &&
                (phy->regs[MII_ANLPAR] & MII_ANLPAR_TXFD)) {
                return true; /* 100 Mbps full duplex */
            }
            if (!(phy->regs[MII_BMCR] & MII_BMCR_SPEED100) &&
                (phy->regs[MII_ANLPAR] & MII_ANLPAR_10FD)) {
                return true; /* 10 Mbps full duplex */
            }
        }
        return false; /* Half duplex */
    }
    
    /* Otherwise use forced duplex */
    if (phy->caps.has_full_duplex) {
        return (phy->regs[MII_BMCR] & MII_BMCR_FD) != 0;
    }
    return false; /* Half duplex */
}

/* Handle MII register reads */
uint32_t el3_mii_register_read(EL3Core *c, unsigned reg)
{
    if (!c->caps.has_mii) {
        return 0xFFFF;
    }
    
    uint32_t value = 0;
    
    switch (reg) {
    case 0: /* MII Management Control/Status */
        /* Bit 0: MII Busy */
        /* Bits 15-8: PHY Address */
        /* Bits 7-3: Register Address */
        /* Bits 2-1: Operation (00=read, 01=write) */
        value = el3_mii_is_busy(c) ? 0x0001 : 0x0000;
        value |= (c->mii.phy_addr << 8);
        value |= (c->mii.reg_addr << 3);
        if (c->mii.operation == MII_OP_WRITE) {
            value |= 0x0002;
        }
        break;
        
    case 2: /* MII Management Read Data */
        value = el3_mii_read_data(c);
        break;
        
    case 4: /* MII Management Write Data */
        value = c->mii.data;
        break;
        
    default:
        value = 0xFFFF;
        break;
    }
    
    TRACE_MII_READ(0, reg, value);
    return value;
}

/* Handle MII register writes */
void el3_mii_register_write(EL3Core *c, unsigned reg, uint32_t value)
{
    if (!c->caps.has_mii) {
        return;
    }
    
    TRACE_MII_WRITE(0, reg, value);
    
    switch (reg) {
    case 0: /* MII Management Control/Status */
        /* Extract fields */
        uint8_t phy_addr = (value >> 8) & 0x1F;
        uint8_t reg_addr = (value >> 3) & 0x1F;
        MIIOperation operation = (value & 0x0002) ? MII_OP_WRITE : MII_OP_READ;
        
        /* Start operation */
        el3_mii_start_operation(c, phy_addr, reg_addr, operation, c->mii.data);
        break;
        
    case 4: /* MII Management Write Data */
        c->mii.data = value & 0xFFFF;
        break;
        
    default:
        break;
    }
}

/* VMState for PHY state */
const VMStateDescription vmstate_el3_phy = {
    .name = "el3_phy",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT16_ARRAY(regs, PHYState, 32),
        VMSTATE_BOOL(link_up, PHYState),
        VMSTATE_UINT64(link_up_time_ns, PHYState),
        VMSTATE_BOOL(caps.has_10mbit, PHYState),
        VMSTATE_BOOL(caps.has_100mbit, PHYState),
        VMSTATE_BOOL(caps.has_full_duplex, PHYState),
        VMSTATE_BOOL(caps.has_auto_negotiation, PHYState),
        VMSTATE_END_OF_LIST()
    }
};

/* VMState for MII management interface */
const VMStateDescription vmstate_el3_mii = {
    .name = "el3_mii",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(state, MIIMgmtInterface),
        VMSTATE_UINT8(phy_addr, MIIMgmtInterface),
        VMSTATE_UINT8(reg_addr, MIIMgmtInterface),
        VMSTATE_UINT16(data, MIIMgmtInterface),
        VMSTATE_INT32(bit_count, MIIMgmtInterface),
        VMSTATE_UINT32(operation, MIIMgmtInterface),
        VMSTATE_UINT64(operation_time_ns, MIIMgmtInterface),
        VMSTATE_END_OF_LIST()
    }
};

/* Integration with core reset */
void el3_core_reset_with_phy(EL3Core *c)
{
    el3_core_reset(c);
    el3_phy_reset(c);
}

/* Integration with core initialization */
void el3_core_init_with_phy(EL3Core *c, int model, const struct EL3VariantOps *ops)
{
    el3_core_init(c, model, ops);
    
    /* Initialize PHY if supported */
    if (c->caps.has_mii) {
        el3_phy_reset(c);
    }
}