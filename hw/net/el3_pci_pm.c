#include "qemu/osdep.h"
#include "hw/pci/pci.h"
#include "hw/net/el3_core.h"

/* PCI PM Capability Structure */
typedef enum {
    PCI_PM_D0_ACTIVE = 0,
    PCI_PM_D1_STANDBY = 1,
    PCI_PM_D2_SUSPEND = 2,
    PCI_PM_D3_HOT = 3,
    PCI_PM_D3_COLD = 4
} pci_pm_state_t;

/* Wake-on-LAN Control Register bits (Window 7, offset 0x0C) */
#define WOL_CTL_MAGIC_PKT_EN    0x0001
#define WOL_CTL_LINK_CHANGE_EN  0x0002
#define WOL_CTL_ARP_REQ_EN      0x0004
#define WOL_CTL_DIR_PKT_EN      0x0008
#define WOL_CTL_BROADCAST_EN    0x0010
#define WOL_CTL_MULTICAST_EN    0x0020
#define WOL_CTL_PATTERN_EN      0x0040
#define WOL_CTL_PME_EN          0x0080

/* PME Status/Control bits */
#define PME_STATUS_ASSERTED     0x8000
#define PME_CTRL_ENABLE         0x0100

/* WoL Pattern structure */
typedef struct {
    uint8_t pattern[128];
    uint8_t mask[16];       /* 128 bits = 16 bytes mask */
    uint8_t offset;
    uint32_t crc;           /* CRC32 validation */
    bool valid;
} wol_pattern_t;

/* PCI PM Capability structure */
typedef struct {
    uint8_t cap_id;         /* Capability ID (0x01 for PM) */
    uint8_t next_ptr;       /* Pointer to next capability */
    uint16_t pm_cap;        /* PM Capabilities register */
    uint16_t pmcsr;         /* PM Control/Status register */
    uint8_t pmcsr_mask;     /* PMCSR mask */
    uint8_t pmdc;           /* PM Data Control register */
    pci_pm_state_t state;   /* Current power state */
    bool pme_enabled;       /* PME enable status */
    bool pme_asserted;      /* PME signal status */
    wol_pattern_t patterns[4]; /* Wake patterns */
    uint16_t wol_ctl;       /* Wake-on-LAN control register */
} EL3PCIPMState;

/* CRC32 calculation for pattern validation */
static uint32_t crc32_calculate(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

/* Initialize PCI Power Management capability */
void el3_pci_pm_init(EL3Core *c, PCIDevice *pci_dev) {
    EL3PCIPMState *pm = g_malloc0(sizeof(EL3PCIPMState));
    
    /* Set up PM capability */
    pm->cap_id = PCI_CAP_ID_PM;
    pm->next_ptr = 0x00;
    pm->pm_cap = (PCI_PM_D3_COLD << 8) | 
                 (PCI_PM_D3_HOT << 6) | 
                 (PCI_PM_D2 << 4) | 
                 (PCI_PM_D1 << 2) | 
                 PCI_PM_D0_ACTIVE;
    
    /* Initialize PMCSR */
    pm->pmcsr = PCI_PM_D0_ACTIVE;
    pm->pmcsr_mask = 0x0000;
    pm->pmdc = 0x00;
    
    /* Initialize power state */
    pm->state = PCI_PM_D0_ACTIVE;
    pm->pme_enabled = false;
    pm->pme_asserted = false;
    
    /* Initialize WoL control register */
    pm->wol_ctl = 0x0000;
    
    /* Store PM state in device */
    pci_set_capability_data(pci_dev, PCI_CAP_ID_PM, (uint8_t *)pm, sizeof(EL3PCIPMState));
}

/* Set PCI power state */
void el3_pci_pm_set_state(EL3Core *c, pci_pm_state_t new_state) {
    PCIDevice *pci_dev = PCI_DEVICE(c);
    EL3PCIPMState *pm = (EL3PCIPMState *)pci_get_capability_data(pci_dev, PCI_CAP_ID_PM);
    
    if (!pm) {
        return;
    }
    
    /* Handle state transitions */
    switch (new_state) {
    case PCI_PM_D0_ACTIVE:
        /* Wake up from any sleep state */
        pm->state = PCI_PM_D0_ACTIVE;
        pm->pmcsr = (pm->pmcsr & ~0x0003) | PCI_PM_D0_ACTIVE;
        break;
        
    case PCI_PM_D1_STANDBY:
        pm->state = PCI_PM_D1_STANDBY;
        pm->pmcsr = (pm->pmcsr & ~0x0003) | PCI_PM_D1_STANDBY;
        break;
        
    case PCI_PM_D2_SUSPEND:
        pm->state = PCI_PM_D2_SUSPEND;
        pm->pmcsr = (pm->pmcsr & ~0x0003) | PCI_PM_D2_SUSPEND;
        break;
        
    case PCI_PM_D3_HOT:
        pm->state = PCI_PM_D3_HOT;
        pm->pmcsr = (pm->pmcsr & ~0x0003) | PCI_PM_D3_HOT;
        break;
        
    case PCI_PM_D3_COLD:
        pm->state = PCI_PM_D3_COLD;
        pm->pmcsr = (pm->pmcsr & ~0x0003) | PCI_PM_D3_COLD;
        break;
    }
}

/* Get current PCI power state */
pci_pm_state_t el3_pci_pm_get_state(EL3Core *c) {
    PCIDevice *pci_dev = PCI_DEVICE(c);
    EL3PCIPMState *pm = (EL3PCIPMState *)pci_get_capability_data(pci_dev, PCI_CAP_ID_PM);
    
    if (!pm) {
        return PCI_PM_D0_ACTIVE;
    }
    
    return pm->state;
}

/* Check if packet should trigger wake */
bool el3_wol_packet_check(EL3Core *c, const uint8_t *buf, size_t len) {
    PCIDevice *pci_dev = PCI_DEVICE(c);
    EL3PCIPMState *pm = (EL3PCIPMState *)pci_get_capability_data(pci_dev, PCI_CAP_ID_PM);
    
    if (!pm || pm->state == PCI_PM_D0_ACTIVE) {
        return false;
    }
    
    /* Check Magic Packet */
    if ((pm->wol_ctl & WOL_CTL_MAGIC_PKT_EN) && el3_wol_magic_packet_detect(c, buf, len)) {
        return true;
    }
    
    /* Check pattern matching */
    if ((pm->wol_ctl & WOL_CTL_PATTERN_EN) && el3_wol_pattern_match(c, buf, len)) {
        return true;
    }
    
    /* Check directed packet */
    if ((pm->wol_ctl & WOL_CTL_DIR_PKT_EN) && memcmp(buf, c->conf.macaddr.a, 6) == 0) {
        return true;
    }
    
    /* Check broadcast */
    if ((pm->wol_ctl & WOL_CTL_BROADCAST_EN) && buf[0] == 0xFF && buf[1] == 0xFF) {
        return true;
    }
    
    /* Check multicast */
    if ((pm->wol_ctl & WOL_CTL_MULTICAST_EN) && (buf[0] & 0x01)) {
        return true;
    }
    
    return false;
}

/* Detect Magic Packet (6 bytes 0xFF followed by 16 repetitions of MAC address) */
bool el3_wol_magic_packet_detect(EL3Core *c, const uint8_t *buf, size_t len) {
    if (len < 102) { /* 6 + 16*6 = 102 bytes minimum */
        return false;
    }
    
    /* Check for 6 bytes of 0xFF */
    if (buf[0] != 0xFF || buf[1] != 0xFF || buf[2] != 0xFF ||
        buf[3] != 0xFF || buf[4] != 0xFF || buf[5] != 0xFF) {
        return false;
    }
    
    /* Check for 16 repetitions of MAC address */
    for (int i = 0; i < 16; i++) {
        if (memcmp(&buf[6 + i * 6], c->conf.macaddr.a, 6) != 0) {
            return false;
        }
    }
    
    return true;
}

/* Match packet against stored wake patterns */
bool el3_wol_pattern_match(EL3Core *c, const uint8_t *buf, size_t len) {
    PCIDevice *pci_dev = PCI_DEVICE(c);
    EL3PCIPMState *pm = (EL3PCIPMState *)pci_get_capability_data(pci_dev, PCI_CAP_ID_PM);
    
    if (!pm) {
        return false;
    }
    
    for (int i = 0; i < 4; i++) {
        if (!pm->patterns[i].valid) {
            continue;
        }
        
        /* Check if pattern fits in packet */
        if (pm->patterns[i].offset + 128 > len) {
            continue;
        }
        
        /* Validate CRC */
        uint32_t calc_crc = crc32_calculate(&buf[pm->patterns[i].offset], 128);
        if (calc_crc != pm->patterns[i].crc) {
            continue;
        }
        
        /* Check pattern with mask */
        bool match = true;
        for (int j = 0; j < 128; j++) {
            uint8_t mask_byte = pm->patterns[i].mask[j / 8];
            uint8_t mask_bit = (mask_byte >> (j % 8)) & 1;
            
            if (mask_bit && (buf[pm->patterns[i].offset + j] != pm->patterns[i].pattern[j])) {
                match = false;
                break;
            }
        }
        
        if (match) {
            return true;
        }
    }
    
    return false;
}

/* Assert PME signal */
void el3_pme_assert(EL3Core *c) {
    PCIDevice *pci_dev = PCI_DEVICE(c);
    EL3PCIPMState *pm = (EL3PCIPMState *)pci_get_capability_data(pci_dev, PCI_CAP_ID_PM);
    
    if (!pm || !pm->pme_enabled) {
        return;
    }
    
    pm->pme_asserted = true;
    pm->pmcsr |= PME_STATUS_ASSERTED;
    
    /* Assert PME interrupt */
    pci_set_irq(pci_dev, 1);
}

/* Clear PME status */
void el3_pme_clear(EL3Core *c) {
    PCIDevice *pci_dev = PCI_DEVICE(c);
    EL3PCIPMState *pm = (EL3PCIPMState *)pci_get_capability_data(pci_dev, PCI_CAP_ID_PM);
    
    if (!pm) {
        return;
    }
    
    pm->pme_asserted = false;
    pm->pmcsr &= ~PME_STATUS_ASSERTED;
    
    /* Clear PME interrupt */
    pci_set_irq(pci_dev, 0);
}

/* Handle PMCSR register read */
uint16_t el3_pmcsr_read(EL3Core *c) {
    PCIDevice *pci_dev = PCI_DEVICE(c);
    EL3PCIPMState *pm = (EL3PCIPMState *)pci_get_capability_data(pci_dev, PCI_CAP_ID_PM);
    
    if (!pm) {
        return 0;
    }
    
    return pm->pmcsr;
}

/* Handle PMCSR register write */
void el3_pmcsr_write(EL3Core *c, uint16_t val) {
    PCIDevice *pci_dev = PCI_DEVICE(c);
    EL3PCIPMState *pm = (EL3PCIPMState *)pci_get_capability_data(pci_dev, PCI_CAP_ID_PM);
    
    if (!pm) {
        return;
    }
    
    /* Handle power state transition */
    pci_pm_state_t new_state = val & 0x0003;
    if (new_state != pm->state) {
        el3_pci_pm_set_state(c, new_state);
    }
    
    /* Handle PME enable */
    if (val & PME_CTRL_ENABLE) {
        pm->pme_enabled = true;
    } else {
        pm->pme_enabled = false;
        el3_pme_clear(c);
    }
    
    /* Update PMCSR but preserve read-only bits */
    pm->pmcsr = (pm->pmcsr & 0xFF00) | (val & 0x00FF);
}

/* Handle WoL control register read */
uint16_t el3_wol_ctl_read(EL3Core *c) {
    PCIDevice *pci_dev = PCI_DEVICE(c);
    EL3PCIPMState *pm = (EL3PCIPMState *)pci_get_capability_data(pci_dev, PCI_CAP_ID_PM);
    
    if (!pm) {
        return 0;
    }
    
    return pm->wol_ctl;
}

/* Handle WoL control register write */
void el3_wol_ctl_write(EL3Core *c, uint16_t val) {
    PCIDevice *pci_dev = PCI_DEVICE(c);
    EL3PCIPMState *pm = (EL3PCIPMState *)pci_get_capability_data(pci_dev, PCI_CAP_ID_PM);
    
    if (!pm) {
        return;
    }
    
    pm->wol_ctl = val;
}

/* Handle pattern register access */
void el3_wol_pattern_write(EL3Core *c, uint8_t pattern_idx, uint8_t offset, 
                          const uint8_t *data, size_t len) {
    PCIDevice *pci_dev = PCI_DEVICE(c);
    EL3PCIPMState *pm = (EL3PCIPMState *)pci_get_capability_data(pci_dev, PCI_CAP_ID_PM);
    
    if (!pm || pattern_idx >= 4 || offset >= 128 || len > 128) {
        return;
    }
    
    memcpy(&pm->patterns[pattern_idx].pattern[offset], data, len);
    
    /* Recalculate CRC if this makes the pattern complete */
    if (offset + len >= 128) {
        pm->patterns[pattern_idx].crc = crc32_calculate(pm->patterns[pattern_idx].pattern, 128);
        pm->patterns[pattern_idx].valid = true;
    }
}

/* Handle pattern mask register access */
void el3_wol_pattern_mask_write(EL3Core *c, uint8_t pattern_idx, uint8_t offset, 
                               const uint8_t *mask, size_t len) {
    PCIDevice *pci_dev = PCI_DEVICE(c);
    EL3PCIPMState *pm = (EL3PCIPMState *)pci_get_capability_data(pci_dev, PCI_CAP_ID_PM);
    
    if (!pm || pattern_idx >= 4 || offset >= 16 || len > 16) {
        return;
    }
    
    memcpy(&pm->patterns[pattern_idx].mask[offset], mask, len);
}

/* Set pattern offset */
void el3_wol_pattern_offset_write(EL3Core *c, uint8_t pattern_idx, uint8_t offset) {
    PCIDevice *pci_dev = PCI_DEVICE(c);
    EL3PCIPMState *pm = (EL3PCIPMState *)pci_get_capability_data(pci_dev, PCI_CAP_ID_PM);
    
    if (!pm || pattern_idx >= 4) {
        return;
    }
    
    pm->patterns[pattern_idx].offset = offset;
}