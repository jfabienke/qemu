#include "hw/net/el3_core.h"

/* PHY register defaults for DP83840 */
static const uint16_t phy_defaults_905b[] = {
    [MII_BMCR] = 0x2100,      /* Normal operation, 10Mbps, HDX */
    [MII_BMSR] = 0x7809,      /* 100BASE-TX FD/HD, 10BASE-T FD/HD, no Jabber */
    [MII_PHYIDR1] = DP83840_PHYID1,
    [MII_PHYIDR2] = DP83840_PHYID2,
    [MII_ANAR] = 0x01E1,      /* Advertise 10/100 HDX/FDX */
    [MII_ANLPAR] = 0x0000,    /* Link partner ability - initially zero */
    [MII_ANER] = 0x0000,      /* Expansion register - no next page */
};

static const uint16_t phy_defaults_905c[] = {
    [MII_BMCR] = 0x2100,      /* Normal operation, 10Mbps, HDX */
    [MII_BMSR] = 0x7809,      /* 100BASE-TX FD/HD, 10BASE-T FD/HD, no Jabber */
    [MII_PHYIDR1] = DP83840_PHYID1,
    [MII_PHYIDR2] = DP83840_PHYID2,
    [MII_ANAR] = 0x05E1,      /* Advertise 10/100 HDX/FDX + Pause */
    [MII_ANLPAR] = 0x0000,    /* Link partner ability - initially zero */
    [MII_ANER] = 0x0000,      /* Expansion register - no next page */
};

void el3_phy_init(EL3Core *c)
{
    const uint16_t *defaults;
    int i;

    if (c->model == MODEL_3C905C) {
        defaults = phy_defaults_905c;
        c->phy_id = (DP83840_PHYID1 << 16) | DP83840_PHYID2;
    } else {
        defaults = phy_defaults_905b;
        c->phy_id = (DP83840_PHYID1 << 16) | DP83840_PHYID2;
    }

    for (i = 0; i < 32; i++) {
        c->phy_regs[i] = defaults[i];
    }

    c->autoneg_complete = false;
    c->link_partner_adv = 0;
    c->autoneg_enabled = false;
    c->link_speed = 10;
    c->full_duplex = false;
    c->link_up = true;

    c->media_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, SCALE_MS, el3_media_timer_cb, c);
}

uint16_t el3_phy_read(EL3Core *c, uint8_t reg)
{
    uint16_t val = c->phy_regs[reg];

    switch (reg) {
    case MII_BMSR:
        /* Clear latch-low bits after read */
        c->phy_regs[MII_BMSR] |= BMSR_LINK_STAT;
        if (c->autoneg_complete) {
            c->phy_regs[MII_BMSR] |= BMSR_AN_COMPLETE;
        }
        break;
    case MII_ANER:
        /* Always report no next page */
        val = 0x0000;
        break;
    }

    return val;
}

void el3_phy_write(EL3Core *c, uint8_t reg, uint16_t val)
{
    uint16_t old_val = c->phy_regs[reg];

    switch (reg) {
    case MII_BMCR:
        if (val & BMCR_RESET) {
            /* Reset PHY - restore defaults */
            const uint16_t *defaults = (c->model == MODEL_3C905C) ? 
                phy_defaults_905c : phy_defaults_905b;
            memcpy(c->phy_regs, defaults, sizeof(c->phy_regs));
            return;
        }

        if (val & BMCR_RESTART) {
            /* Restart autonegotiation */
            c->autoneg_complete = false;
            c->phy_regs[MII_BMSR] &= ~BMSR_AN_COMPLETE;
            timer_mod(c->media_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 500 * SCALE_MS);
        }

        if ((val ^ old_val) & (BMCR_SPEED | BMCR_DUPLEX)) {
            /* Speed or duplex changed - update MAC settings */
            c->link_speed = (val & BMCR_SPEED) ? 100 : 10;
            c->full_duplex = (val & BMCR_DUPLEX) ? true : false;
            el3_core_set_link_status(c->nic->ncs);
        }

        /* Clear self-clearing bits */
        val &= ~(BMCR_RESTART | BMCR_RESET);
        break;

    case MII_ANAR:
        /* Update advertised capabilities */
        val |= ADVERTISE_CSMA;  /* Always set CSMA */
        if (c->model != MODEL_3C905C) {
            val &= ~ADVERTISE_PAUSE;  /* 905B doesn't support pause */
        }
        break;
    }

    c->phy_regs[reg] = val;
}

void el3_autoneg_complete(EL3Core *c)
{
    uint16_t link_adv = 0;

    /* Simulate link partner capabilities */
    if (c->model == MODEL_3C905C) {
        link_adv = ADVERTISE_100FULL | ADVERTISE_100HALF | 
                   ADVERTISE_10FULL | ADVERTISE_10HALF | ADVERTISE_PAUSE;
    } else {
        link_adv = ADVERTISE_100FULL | ADVERTISE_100HALF | 
                   ADVERTISE_10FULL | ADVERTISE_10HALF;
    }

    c->link_partner_adv = link_adv;
    c->phy_regs[MII_ANLPAR] = link_adv | ADVERTISE_LPACK;
    c->autoneg_complete = true;
    c->phy_regs[MII_BMSR] |= BMSR_AN_COMPLETE;

    /* Determine link speed and duplex from common capabilities */
    if ((c->phy_regs[MII_ANAR] & link_adv) & ADVERTISE_100FULL) {
        c->link_speed = 100;
        c->full_duplex = true;
    } else if ((c->phy_regs[MII_ANAR] & link_adv) & ADVERTISE_100HALF) {
        c->link_speed = 100;
        c->full_duplex = false;
    } else if ((c->phy_regs[MII_ANAR] & link_adv) & ADVERTISE_10FULL) {
        c->link_speed = 10;
        c->full_duplex = true;
    } else {
        c->link_speed = 10;
        c->full_duplex = false;
    }

    el3_core_set_link_status(c->nic->ncs);
}

void el3_media_timer_cb(void *opaque)
{
    EL3Core *c = opaque;
    el3_autoneg_complete(c);
}

void el3_mii_command(EL3Core *c, uint16_t cmd)
{
    uint8_t reg = (cmd >> 8) & 0x1F;
    uint16_t val = cmd & 0xFFFF;

    if (cmd & 0x8000) {
        /* Write operation */
        el3_phy_write(c, reg, val);
    } else {
        /* Read operation - handled in el3_core_register_read */
        c->phy_regs[MII_BMCR] &= ~BMCR_RESTART;  /* Clear restart bit after read */
    }
}

uint16_t el3_mii_data_read(EL3Core *c)
{
    uint8_t reg = (c->windows[4][W4_PHYSICAL_MGMT] >> 8) & 0x1F;
    return el3_phy_read(c, reg);
}