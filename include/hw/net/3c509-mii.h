#ifndef HW_NET_3C509_MII_H
#define HW_NET_3C509_MII_H

#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "migration/vmstate.h"

/* Forward declarations to avoid circular dependency */
typedef struct EL3Core EL3Core;
struct EL3VariantOps;

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

/* Function prototypes for MII operations */
void el3_mii_start_operation(EL3Core *c, uint8_t phy_addr, uint8_t reg_addr,
                             MIIOperation operation, uint16_t data);
uint16_t el3_mii_read_data(EL3Core *c);
bool el3_mii_is_busy(EL3Core *c);

/* PHY management functions */
void el3_phy_reset(EL3Core *c);
void el3_phy_set_link(EL3Core *c, bool link_up);
bool el3_phy_get_link(EL3Core *c);
int el3_phy_get_speed(EL3Core *c);
bool el3_phy_get_duplex(EL3Core *c);

/* MII register access functions */
uint32_t el3_mii_register_read(EL3Core *c, unsigned reg);
void el3_mii_register_write(EL3Core *c, unsigned reg, uint32_t value);

/* Integration hooks */
void el3_core_reset_with_phy(EL3Core *c);
void el3_core_init_with_phy(EL3Core *c, int model, const struct EL3VariantOps *ops);

/* VMState descriptors for migration */
extern const VMStateDescription vmstate_el3_phy;
extern const VMStateDescription vmstate_el3_mii;

#endif /* HW_NET_3C509_MII_H */