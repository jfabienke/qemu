#ifndef HW_NET_EL3_CORE_H
#define HW_NET_EL3_CORE_H

#include "qemu/osdep.h"
#include "net/net.h"
#include "hw/irq.h"
#include "qemu/timer.h"
#include "system/memory.h"
#include "exec/memattrs.h"
#include "system/address-spaces.h"
#include "qemu/main-loop.h"
#include "qemu/fifo8.h"
#include "migration/vmstate.h"

/* Model IDs */
typedef enum {
    MODEL_3C509,
    MODEL_3C509B,
    MODEL_3C515,
    MODEL_3C590,    /* Vortex */
    MODEL_3C595,    /* Vortex */
    MODEL_3C900,    /* Boomerang */
    MODEL_3C905,    /* Boomerang */
    MODEL_3C905B,   /* Cyclone */
    MODEL_3C905C,   /* Tornado */
} EL3Model;

/* Maximum windows and registers */
#define EL3_MAX_WINDOWS 8  /* Hardware has windows 0-7 */

/* Ethernet frame size constants */
#define ETH_MIN_DATA_NOFCS    60    /* Minimum data without FCS */
#define EL3_TX_SANITY_MAX     9216  /* Emulator defensive cap (not hardware limit) */
#define ETH_FCS_LEN           4     /* Frame Check Sequence length */
#define EL3_WINDOW_SIZE 16

/* FIFO sizes for 3C509B */
#define TX_FIFO_SIZE       2048  /* TX FIFO capacity in bytes */
#define RX_FIFO_SIZE       8192  /* RX FIFO capacity (grown from 4096 to hold a FDDI-sized frame) */

/* MacControl (Window 3, offset 6): allowLargePackets is bit 6 -- when set, the oversizedFrame
 * error fires at >=4491 B (excl FCS) instead of >=1515 B, i.e. FDDI-sized RX is accepted.
 * (3Com PCI/EISA Bus-Master ref p4-26/4-27.) */
#define W3_MAC_CONTROL        0x06
#define MAC_CTRL_ALLOW_LARGE  0x0040
#define EL3_LARGE_RX_MAX      4490  /* max accepted RX frame (excl FCS) when allowLargePackets=1 */

/* ISA DMA constraints */
#define ISA_DMA_MAX_ADDR    0xFFFFFF  /* 24-bit ISA address space */
#define ISA_DMA_BOUNDARY   0x10000    /* 64KB boundary for ISA DMA */

/* Window offsets */
/* Window 0 - EEPROM */
#define W0_EEPROM_CMD      0x0A
#define W0_EEPROM_DATA     0x0C

/* EEPROM Microwire (93C46) control bits for W0_EEPROM_CMD */
#define EEPROM_SK          0x01  /* Serial clock */
#define EEPROM_CS          0x02  /* Chip select */
#define EEPROM_DI          0x04  /* Data in (to EEPROM) */
#define EEPROM_DO          0x08  /* Data out (from EEPROM) - read only */
#define EEPROM_BUSY        0x8000  /* EEPROM busy flag */

/* 93C46 EEPROM opcodes (after start bit) */
#define EEPROM_OP_READ     0x02  /* 10b - Read */
#define EEPROM_OP_WRITE    0x01  /* 01b - Write */
#define EEPROM_OP_ERASE    0x03  /* 11b - Erase */
#define EEPROM_OP_EWEN     0x00  /* 00 11xxxx - Erase/Write Enable */
#define EEPROM_OP_EWDS     0x00  /* 00 00xxxx - Erase/Write Disable */
#define EEPROM_OP_ERAL     0x00  /* 00 10xxxx - Erase All */
#define EEPROM_OP_WRAL     0x00  /* 00 01xxxx - Write All */

/* Window 1 - Operating Status */
#define W1_TX_RX_FIFO      0x00  /* TX FIFO (write) / RX FIFO (read) */
#define W1_RX_STATUS       0x08  /* RX packet status */
#define W1_TIMER           0x0A  /* Timer */
#define W1_TX_STATUS       0x0B  /* TX completion status */
#define W1_TX_FREE         0x0C  /* Free bytes in TX FIFO */

/* RX Status Register bits (Window 1, offset 0x08) */
#define RX_STATUS_INCOMPLETE  0x8000  /* Bit 15: Packet not fully received */
#define RX_STATUS_ERROR       0x4000  /* Bit 14: Packet has error */
#define RX_STATUS_ERR_MASK    0x3800  /* Bits 13-11: Error code */
#define RX_STATUS_ERR_SHIFT   11      /* Shift to get error code */
#define RX_STATUS_LENGTH      0x07FF  /* Bits 10-0: Packet length */

/* RX Error codes (bits 13-11 when error bit is set) */
#define RX_ERR_OVERRUN        0x0000  /* 000: RX FIFO overrun */
#define RX_ERR_OVERSIZE       0x0800  /* 001: Frame > 1518 bytes */
#define RX_ERR_ALIGNMENT      0x1000  /* 010: Alignment/framing error */
#define RX_ERR_RUNT           0x1800  /* 011: Frame < 64 bytes */
#define RX_ERR_DRIBBLE        0x2000  /* 100: Dribble bits after frame */
#define RX_ERR_CRC            0x2800  /* 101: CRC error */

/* RX FIFO constants */
#define RX_FIFO_SIZE          8192    /* grown to hold a FDDI-sized frame (was 4096) */
#define RX_HEADER_SIZE        4       /* 2 words: status + count */
/* Note: Compare error codes directly against (rx_status & RX_STATUS_ERR_MASK) */

/* 3C59x TX Descriptor (DownList) Status bits */
#define TXD_LENGTH_MASK      0x00001FFF  /* Bits 0-12: Packet length */
#define TXD_CRC_DISABLE      0x00002000  /* Bit 13: Disable CRC generation */
#define TXD_COMPLETE         0x00008000  /* Bit 15: TX complete */
#define TXD_DN_COMPLETE      0x00010000  /* Bit 16: Download complete to FIFO */
#define TXD_UNDERRUN         0x00020000  /* Bit 17: TX underrun */
#define TXD_MAX_COLL         0x00040000  /* Bit 18: Max collisions */
#define TXD_LATE_COLL        0x00080000  /* Bit 19: Late collision */
#define TXD_CARRIER_LOST     0x00100000  /* Bit 20: Carrier sense lost */
#define TXD_JABBER           0x00200000  /* Bit 21: Jabber error */
/* Note: Checksum offload bits (25-27) are from 3CR990 Typhoon, NOT 3C905B/C */
/* #define TXD_ADD_IPCHKSUM  0x02000000 */  /* Not on 3C905B/C */
/* #define TXD_ADD_TCPCHKSUM 0x04000000 */  /* Not on 3C905B/C */
/* #define TXD_ADD_UDPCHKSUM 0x08000000 */  /* Not on 3C905B/C */
#define TXD_INTR_UPLOADED    0x80000000  /* Bit 31: Interrupt on upload to FIFO */

/* 3C59x TX Status Register bits (8-bit, separate from descriptor) */
#define TX_STATUS_COMPLETE   0x01  /* Transmission complete */
#define TX_STATUS_UNDERRUN   0x04  /* TX FIFO underrun */
#define TX_STATUS_MAX_COLL   0x08  /* Excessive collisions (16) */
#define TX_STATUS_JABBER     0x10  /* Jabber timeout */
#define TX_STATUS_UNDERRUN2  0x20  /* Transmit underrun */

/* 3C59x RX Descriptor (UpList) Status bits
 * Note: the Vortex/Boomerang/Cyclone/Tornado silicon DOES support FDDI-sized large packets
 * (<= 4494 bytes incl FCS) via allowLargePackets in MacControl -- it is gated off in software
 * (like Linux 3c59x, limited by its 4K skbuff allocation; see Becker's 3c59x.c header and 3Com's
 * PCI/EISA Bus-Master Driver Tech Ref p2-2). What the 3C905B/C lack vs the later 3CR990 Typhoon is
 * hardware checksum validation, VLAN tag extraction, and true 9KB jumbo frames.
 */
#define RXD_LENGTH_MASK      0x00001FFF  /* Bits 0-12: Packet length */
#define RXD_ERROR            0x00004000  /* Bit 14: Error occurred */
#define RXD_COMPLETE         0x00008000  /* Bit 15: RX complete */
#define RXD_OVERRUN          0x00010000  /* Bit 16: RX FIFO overrun */
#define RXD_RUNT             0x00020000  /* Bit 17: Runt frame < 64 bytes */
#define RXD_ALIGN_ERROR      0x00040000  /* Bit 18: Alignment error */
#define RXD_CRC_ERROR        0x00080000  /* Bit 19: CRC error */
#define RXD_OVERSIZE         0x00100000  /* Bit 20: Oversize frame > 1518 */
#define RXD_DRIBBLE          0x00200000  /* Bit 21: Dribble bits */
#define RXD_IP_CHKSUM_ERR    0x02000000  /* Bit 25: IP checksum error */
#define RXD_TCP_CHKSUM_ERR   0x04000000  /* Bit 26: TCP checksum error */
#define RXD_UDP_CHKSUM_ERR   0x08000000  /* Bit 27: UDP checksum error */
#define RXD_IP_CHKSUM_VALID  0x20000000  /* Bit 29: IP checksum validated */
#define RXD_TCP_CHKSUM_VALID 0x40000000  /* Bit 30: TCP checksum validated */
#define RXD_UDP_CHKSUM_VALID 0x80000000  /* Bit 31: UDP checksum validated */

/* 3C515 TX DMA Descriptor Status bits (from 86Box) */
#define _3C515_TX_DMA_DESC_COMPLETE    0x00008000  /* TX complete */
#define _3C515_TX_DMA_DESC_ERROR       0x00004000  /* Error occurred */
#define _3C515_TX_DMA_DESC_LAST        0x00002000  /* Last fragment */
#define _3C515_TX_DMA_DESC_FIRST       0x00001000  /* First fragment */
#define _3C515_TX_DMA_DESC_DN_COMPLETE 0x00010000  /* Download complete */
#define _3C515_TX_DMA_DESC_UP_COMPLETE 0x00020000  /* Upload complete */

/* 3C59x RX error extraction */
#define RXD_ERROR_SHIFT      16  /* Shift to get error byte from status */
#define RXD_ERROR_MASK       0xFF /* Mask for error byte */

/* Window 3 - Configuration */
#define W3_INTERNAL_CONFIG 0x00
#define W3_RAM_SIZE_MASK   0xC000
#define W3_RAM_SPLIT_MASK  0x3000

/* Window 4 - Diagnostics */
#define W4_FIFO_DIAG       0x04  /* FIFO diagnostic */
#define W4_NET_DIAG        0x06  /* Network diagnostic */
#define W4_PHYSICAL_MGMT   0x08  /* MII/PHY management register */
#define W4_MEDIA_STATUS    0x0A  /* Media type and status */

/* Window 7 - Bus Master/Product ID */
#define W7_PRODUCT_ID      0x02  /* Product identification (32-bit) */
#define W7_VENDOR_ID       0x00  /* Vendor identification (16-bit) */
#define W7_DEVICE_ID       0x02  /* Device identification (16-bit) */
#define W7_UP_LIST_PTR     0x38  /* Upload list pointer */
#define W7_DOWN_LIST_PTR   0x24  /* Download list pointer */
#define W7_UP_PKT_STATUS   0x30  /* Upload packet status */
#define W7_DOWN_PKT_STATUS 0x20  /* Download packet status */

/* NET_DIAG Register bits (Window 4, offset 0x06) */
#define NET_DIAG_FD_ENABLE      0x8000  /* Bit 15: Full-duplex enable [R/W] */
#define NET_DIAG_UPPER_BYTES_OK 0x2000  /* Bit 13: Upper bytes test passed [RO] */
#define NET_DIAG_TX_OK          0x1000  /* Bit 12: TX test passed [RO] */
#define NET_DIAG_LINK_BEAT      0x0800  /* Bit 11: Link beat detect (10BaseT) [RO] */
#define NET_DIAG_STATS_OK       0x0800  /* Bit 11: Statistics test (diagnostic) [RO] */
#define NET_DIAG_RX_OK          0x0400  /* Bit 10: RX test passed [RO] */
#define NET_DIAG_SQE_OK         0x0200  /* Bit 9: SQE test status (AUI) [RO] */
#define NET_DIAG_INTERNAL_OK    0x0200  /* Bit 9: Internal test OK [RO] */
#define NET_DIAG_EXTERNAL_WRAP  0x0100  /* Bit 8: External loopback enabled [RO] */
#define NET_DIAG_FIFO_OK        0x0080  /* Bit 7: FIFO test passed [RO] */
#define NET_DIAG_STATS_ENABLE   0x0040  /* Bit 6: Enable extended statistics [R/W] */
#define NET_DIAG_INTERNAL_LB    0x0020  /* Bit 5: Internal loopback mode [R/W] */
#define NET_DIAG_REV_MASK       0x0003  /* Bits 1-0: Hardware revision [RO] */

/* Window 6 - Statistics */
#define W6_TX_BYTES_OK     0x0C  /* Total TX bytes */
#define W6_RX_BYTES_OK     0x0A  /* Total RX bytes */

/* TX Status bits (8-bit register, Window 1 offset 0x0B) */
#define TX_STAT_COMPLETE   0x01  /* Bit 0: TX complete */
#define TX_STAT_RESERVED1  0x02  /* Bit 1: Reserved */
#define TX_STAT_UNDERRUN   0x04  /* Bit 2: TX underrun */
#define TX_STAT_MAX_COLL   0x08  /* Bit 3: Max collisions (never set in QEMU) */
#define TX_STAT_JABBER     0x10  /* Bit 4: Jabber timeout */
#define TX_STAT_RESERVED5  0x20  /* Bit 5: Reserved */
#define TX_STAT_RESERVED6  0x40  /* Bit 6: Reserved */
#define TX_STAT_RESERVED7  0x80  /* Bit 7: Reserved */

/* TX Status special values recognized by drivers */
#define TX_STATUS_NORMAL_ERROR    0x88  /* 16 collisions, drivers often ignore */
#define TX_STATUS_DUPLEX_MISMATCH 0x82  /* Duplex mismatch signature */
#define TX_STATUS_NEED_RESET      0x30  /* Jabber/underrun, requires TX reset */
#define TX_STATUS_ABORT_MASK      0x38  /* Bits indicating aborted TX */
#define TX_STATUS_FIFO_ERROR      0x14  /* FIFO error mask (3c59x) */
#define TX_STATUS_ANY_ERROR       0x3C  /* Any error mask (3c509) */

/* RX Status driver check masks */
#define RX_STATUS_DRIVER_ERROR    0x4000  /* Bit 14: Error flag drivers check */
#define RX_STATUS_3C59X_LENGTH    0x1FFF  /* Bits 12-0: Length mask for 3c59x */

/* Command codes (bits 15-11 shifted) */
#define CMD_GLOBAL_RESET   0x0000  /* 0x00 << 11 */
#define CMD_SELECT_WINDOW  0x0800  /* 0x01 << 11 */
#define CMD_START_COAX     0x1000  /* 0x02 << 11 */
#define CMD_RX_DISABLE     0x1800  /* 0x03 << 11 */
#define CMD_RX_ENABLE      0x2000  /* 0x04 << 11 */
#define CMD_RX_RESET       0x2800  /* 0x05 << 11 */
#define CMD_RX_DISCARD     0x4000  /* 0x08 << 11 */
#define CMD_TX_ENABLE      0x4800  /* 0x09 << 11 */
#define CMD_TX_DISABLE     0x5000  /* 0x0A << 11 */
#define CMD_TX_RESET       0x5800  /* 0x0B << 11 */
#define CMD_REQUEST_INTR   0x6000  /* 0x0C << 11 - ADDED: force/test interrupt */
#define CMD_ACK_INTR       0x6800  /* 0x0D << 11 */
#define CMD_SET_INTR_ENB   0x7000  /* 0x0E << 11 */
#define CMD_SET_STATUS_ENB 0x7800  /* 0x0F << 11 */
#define CMD_SET_RX_FILTER  0x8000  /* 0x10 << 11 */
#define CMD_SET_RX_EARLY_THRESH  0x8800  /* 0x11 << 11 - ADDED: missing for 3C515/vortex drivers */
#define CMD_SET_TX_AVAIL   0x9000  /* 0x12 << 11 - Set TX available space */
#define CMD_SET_TX_START_THRESH  0x9800  /* 0x13 << 11 - Set TX start threshold */
#define CMD_TX_START       0xC000  /* 0x18 << 11 - Start TX with length param */
#define CMD_START_DMA_UP   0xA000  /* 0x14 << 11, param=0 - Start RX DMA (3C515) */
#define CMD_START_DMA_DOWN 0xA000  /* 0x14 << 11, param=1 - Start TX DMA (3C515) */
#define CMD_STATS_ENABLE   0xA800  /* 0x15 << 11 */
#define CMD_STATS_DISABLE  0xB000  /* 0x16 << 11 */
#define CMD_STOP_COAX      0xB800  /* 0x17 << 11 */

/* 3C515 DMA stall commands - use CMD_UP_STALL base with parameter */
#define CMD_UP_STALL       0x3000  /* 0x06 << 11, param=0 - Stall RX DMA */
#define CMD_UP_UNSTALL     0x3000  /* 0x06 << 11, param=1 - Unstall RX DMA */
#define CMD_DOWN_STALL     0x3000  /* 0x06 << 11, param=2 - Stall TX DMA */
#define CMD_DOWN_UNSTALL   0x3000  /* 0x06 << 11, param=3 - Unstall TX DMA */

/* RX Filter command bits (parameter to CMD_SET_RX_FILTER) */
/* Hardware SetRxFilter bits -- must match the 3C509/Vortex command wire format that drivers
 * send (individual=0x01, multicast=0x02, broadcast=0x04, promiscuous=0x08). */
#define RX_FILTER_INDIVIDUAL  0x0001  /* Accept unicast to station MAC */
#define RX_FILTER_MULTICAST   0x0002  /* Accept multicast (3C509: all; later: via hash) */
#define RX_FILTER_BROADCAST   0x0004  /* Accept broadcast packets */
#define RX_FILTER_PROMISCUOUS 0x0008  /* Accept all packets (promiscuous mode) */
/* Emulator-internal pseudo-flags (not part of the hardware SetRxFilter bitfield) */
#define RX_FILTER_ALLMULTI    0x0100  /* Accept all multicast */
#define RX_FILTER_ACCEPT_ERROR 0x0200  /* Accept packets with CRC/alignment errors */

/* DMA UpStatus/DownStatus bits (3C59x) */
#define UP_PKT_STATUS      0x0001  /* Packet status available */
#define UP_NOBUF           0x0002  /* No buffer available */
#define UP_ERROR           0x0004  /* Error occurred */
#define UP_OVERRUN         0x0008  /* RX overrun */
#define UP_COMPLETE        0x8000  /* Upload complete */

#define DOWN_COMPLETE      0x8000  /* Download complete */


/* Status bits (read from 0x0E) */
#define STAT_INT_LATCH     0x0001  /* Bit 0: Interrupt occurred */
#define STAT_ADAPTER_FAIL  0x0002  /* Bit 1: Hardware failure/RX overrun (509B) */
#define STAT_TX_COMPLETE   0x0004  /* Bit 2: Transmission completed */
#define STAT_TX_AVAILABLE  0x0008  /* Bit 3: TX FIFO has space */
#define STAT_RX_COMPLETE   0x0010  /* Bit 4: Packet received */
#define STAT_RX_EARLY      0x0020  /* Bit 5: Early RX (unused) */
#define STAT_INT_REQ       0x0040  /* Bit 6: Interrupt requested */
#define STAT_STATS_FULL    0x0080  /* Bit 7: Statistics updated */
#define STAT_DMA_DONE      0x0100  /* Bit 8: DMA Done (59x) */
#define STAT_DOWN_COMPLETE 0x0200  /* Bit 9: TX download complete (59x) */
#define STAT_UP_COMPLETE   0x0400  /* Bit 10: RX upload complete (59x) */
#define STAT_DMA_IN_PROG   0x0800  /* Bit 11: DMA in progress (59x) */
#define STAT_CMD_IN_PROG   0x1000  /* Bit 12: Command in progress */

/* Model capabilities */
typedef struct {
    bool has_mii;
    bool has_bus_master;
    bool has_full_duplex;
    bool is_100mbit;
    bool has_vlan_support;     /* Supports 802.1Q VLAN frames (1522 bytes) */
    uint16_t ram_bytes;
    uint16_t tx_fifo_bytes;
    uint16_t rx_fifo_bytes;
    uint8_t max_windows;
} EL3Caps;

/* Forward declaration */
typedef struct EL3Core EL3Core;

/* Variant operations interface - separates bus plumbing from networking logic */
typedef struct EL3VariantOps {
    /* IRQ management */
    void (*irq_set)(EL3Core *c, bool level);
    
    /* Variant-specific register access (return MEMTX_OK if handled) */
    MemTxResult (*reg_read)(EL3Core *c, unsigned win, unsigned off, 
                           unsigned size, uint64_t *data);
    MemTxResult (*reg_write)(EL3Core *c, unsigned win, unsigned off,
                            unsigned size, uint64_t data);
    
    /* TX path: variant provides data to core */
    void (*tx_on_core_sent)(EL3Core *c, uint32_t status_flags);
    void (*tx_kick)(EL3Core *c);
    
    /* RX path: core asks variant to place frames */
    ssize_t (*rx_place_frame)(EL3Core *c, const uint8_t *buf, size_t len,
                             uint32_t rx_status, uint32_t rx_len);
    bool (*rx_has_space)(EL3Core *c, size_t len);
    
    /* General kick for resource updates */
    void (*kick)(EL3Core *c, uint32_t what);
    
    /* Capabilities */
    uint8_t fifo_width_mask;  /* Supported access sizes: bit 0=8, bit 1=16, bit 2=32 */
    bool has_dma;
} EL3VariantOps;

/* Kick types */
#define EL3_KICK_TX    0x0001
#define EL3_KICK_RX    0x0002

/* Window side effect structure */
typedef struct {
    uint8_t window;
    uint16_t offset;
    uint16_t mask;
    void (*handler)(void *s, uint16_t old_val, uint16_t new_val);
} WindowSideEffect;

/* RX packet descriptor for FIFO management */
#define RX_MAX_PACKETS 128  /* Can hold ~68 min-size packets in 4KB */

typedef struct {
    uint16_t status;        /* RX status word */
    uint16_t length;        /* Packet length */
    uint16_t bytes_read;    /* Bytes already read via data port */
    uint16_t data_offset;   /* Offset in rx_data buffer */
    bool complete;          /* Packet fully received */
} RXPacketDesc;

/* PCI DMA descriptor structures (3C59x) */
#define EL3_DMA_RING_SIZE 64

/* Download descriptor (TX) */
typedef struct {
    uint32_t next_desc;     /* Physical address of next descriptor */
    uint32_t status;        /* Frame status and length */
    uint32_t addr;          /* Physical address of data buffer */
    uint32_t length;        /* Length of this fragment */
} EL3DownDesc;

/* Upload descriptor (RX) */
typedef struct {
    uint32_t next_desc;     /* Physical address of next descriptor */
    uint32_t status;        /* Frame status and length */
    uint32_t addr;          /* Physical address of data buffer */
    uint32_t length;        /* Length of this fragment */
} EL3UpDesc;

/* DMA descriptor flags */
#define EL3_DESC_DMA_DONE    0x00010000  /* DMA transfer complete */
#define EL3_DESC_DMA_INDICATE 0x00008000 /* Generate interrupt when done */
#define EL3_DESC_DOWN_COMPLETE 0x00004000 /* Download complete */
#define EL3_DESC_UP_COMPLETE   0x00004000 /* Upload complete */
#define EL3_DESC_UP_ERROR      0x00002000 /* Upload error */
#define EL3_DESC_DOWN_ERROR    0x00002000 /* Download error */
#define EL3_DESC_LENGTH_MASK   0x00001FFF /* Length field mask */

/* ISA DMA descriptor structure (3C515) */
typedef struct {
    uint32_t next;          /* Physical address of next descriptor (24-bit) */
    uint32_t status;        /* Frame status and length */
    uint32_t addr;          /* Physical address of data buffer (24-bit) */
    uint32_t length;        /* Length of this fragment */
} EL3ISADmaDesc;

/* Actual 3C905B/C Hardware Features:
 * - MII/PHY management interface (100BASE-TX PHY)
 * - Wake-on-LAN (WoL) support
 * - 802.3x PAUSE flow control
 * - MAC address filtering (multicast hash)
 * - Early RX/TX thresholds
 * - DMA descriptor rings (no checksum offload)
 * - Power management (ACPI D0-D3 states)
 */

/* MII/PHY register definitions from 86Box */
#define MII_BMCR      0x00
#define MII_BMSR      0x01
#define MII_PHYIDR1   0x02
#define MII_PHYIDR2   0x03
#define MII_ANAR      0x04
#define MII_ANLPAR    0x05
#define MII_ANER      0x06
#define MII_100TX_PHY 0x10
#define MII_100TX_CSR 0x11
#define MII_100TX_ISR 0x12
#define MII_100TX_ESR 0x13
#define MII_100TX_PCR 0x19
#define MII_100TX_PSR 0x1A
#define MII_100TX_PIR 0x1B
#define MII_100TX_POR 0x1C
#define MII_100TX_LCR 0x1D
#define MII_100TX_TCR 0x1E
#define MII_100TX_TECH 0x1F

/* BMCR bits */
#define BMCR_RESET     0x8000
#define BMCR_LOOPBACK  0x4000
#define BMCR_SPEED     0x2000
#define BMCR_ANE       0x1000
#define BMCR_PWDN      0x0800
#define BMCR_ISOLATE   0x0400
#define BMCR_RESTART   0x0200
#define BMCR_DUPLEX    0x0100
#define BMCR_COLTEST   0x0080

/* BMSR bits */
#define BMSR_100T4     0x8000
#define BMSR_100TX_FD  0x4000
#define BMSR_100TX_HD  0x2000
#define BMSR_10T_FD    0x1000
#define BMSR_10T_HD    0x0800
#define BMSR_AN_COMPLETE 0x0020
#define BMSR_REMOTE_FAULT 0x0010
#define BMSR_AN_ABLE   0x0008
#define BMSR_LINK_STAT 0x0004
#define BMSR_JABBER    0x0002
#define BMSR_EXT_CAP   0x0001

/* ANAR and ANLPAR bits */
#define ADVERTISE_SLCT    0x001F
#define ADVERTISE_CSMA    0x0001
#define ADVERTISE_10HALF  0x0020
#define ADVERTISE_10FULL  0x0040
#define ADVERTISE_100HALF 0x0080
#define ADVERTISE_100FULL 0x0100
#define ADVERTISE_100BASET4 0x0200
#define ADVERTISE_PAUSE   0x0400
#define ADVERTISE_REMOTE_FAULT 0x2000
#define ADVERTISE_LPACK   0x4000
#define ADVERTISE_NP      0x8000

/* DP83840 PHY ID constants */
#define DP83840_PHYID1 0x2000
#define DP83840_PHYID2 0x5C00

/* DMA engine state */
typedef enum {
    EL3_DMA_IDLE = 0,
    EL3_DMA_FETCHING_DESC,
    EL3_DMA_TRANSFERRING,
    EL3_DMA_UPDATING_DESC
} EL3DMAState;

/* DMA ring buffer management */
typedef struct EL3DMAEngine {
    hwaddr base_addr;       /* Physical base address of descriptor ring */
    uint32_t current_desc;  /* Current descriptor index */
    uint32_t ring_size;     /* Number of descriptors in ring */
    EL3DMAState state;      /* Current DMA state */
    bool enabled;           /* DMA engine enabled */
    
    /* Current operation */
    EL3DownDesc down_desc;  /* Current download descriptor */
    EL3UpDesc up_desc;      /* Current upload descriptor */
    uint32_t transfer_len;  /* Bytes remaining in current transfer */
    hwaddr buffer_addr;     /* Current buffer address */
    
    /* Bottom Half handlers for async DMA */
    QEMUBH *tx_bh;          /* TX (download) bottom half */
    QEMUBH *rx_bh;          /* RX (upload) bottom half */
    
    /* Context for BH operations */
    AddressSpace *as;       /* Address space for DMA operations */
    void *opaque;           /* Context pointer (e.g., device state) */
    
    /* Pending frame data for RX */
    uint8_t *pending_frame; /* Buffer holding pending RX frame */
    size_t pending_len;     /* Length of pending frame */
    uint32_t pending_status; /* RX status for pending frame */
    
    /* Scheduling state for synchronization */
    int tx_scheduled;       /* TX BH scheduled (0/1 for atomics) */
    int rx_scheduled;       /* RX BH scheduled (0/1 for atomics) */
    
    /* AioContext for BH operations */
    AioContext *ctx;        /* Current AioContext */
} EL3DMAEngine;

/* ISA ID port states */
typedef enum {
    ID_IDLE = 0,           /* Waiting for unlock sequence */
    ID_UNLOCKING,          /* Processing 32-byte unlock sequence */ 
    ID_ISOLATION,          /* Ready for tag/select commands */
    ID_SELECTED,           /* This card is selected */
    ID_CONFIG              /* Card is activated and configured */
} IDPortState;

/* ISA ID sequence state */
typedef struct {
    IDPortState state;      /* Current state of ID port FSM */
    uint8_t unlock_pos;     /* Position in 32-byte unlock sequence */
    uint8_t board_tag;      /* Board tag for multi-card selection */
    uint8_t selected_tag;   /* Currently selected tag */
    uint32_t product_id;    /* 32-bit EISA product ID */
    uint8_t bit_pos;        /* Bit position for product ID read */
    uint16_t id_port;       /* ID port (0x100-0x10F) */
    
    /* EEPROM streaming via ID port */
    uint16_t eeprom_data_register;  /* Last EEPROM word read for contention test */
    uint8_t eeprom_stream_addr;     /* Current EEPROM word being streamed */
    uint8_t eeprom_stream_bits;     /* Bits remaining in current stream (0=not streaming) */
    uint16_t eeprom_stream_data;    /* Current word being streamed */
    uint8_t tag_register;           /* SET_TAG value (0=respond, non-zero=silent) */
} EL3IDState;

/* EEPROM state machine */
typedef enum {
    EEPROM_STATE_IDLE = 0,
    EEPROM_STATE_BUSY
} EEPROMState;

/* Core state for all 3Com NICs */
struct EL3Core {
    /* Variant operations */
    const EL3VariantOps *ops;
    
    /* Model and capabilities */
    EL3Model model;
    EL3Caps caps;
    
    /* Windows and registers */
    uint16_t windows[EL3_MAX_WINDOWS][EL3_WINDOW_SIZE];
    uint8_t current_window;
    
    /* EEPROM state */
    uint16_t eeprom[64];
    EEPROMState eeprom_state;
    uint8_t eeprom_addr;
    uint16_t eeprom_data;
    int64_t eeprom_due_ns;      /* Timer due time for migration */
    uint64_t eeprom_latency_ns; /* 162us default */
    QEMUTimer *eeprom_timer;
    
    /* Command timer */
    QEMUTimer *cmd_timer;
    
    /* Status and interrupts */
    uint16_t command;
    uint16_t status;           /* Live status register */
    uint16_t int_status;       /* Latched interrupt events */
    uint16_t int_mask;         /* Interrupt enable mask */
    uint16_t intr_enb;         /* Interrupt enable mask (0x0E command) */
    uint16_t status_enb;       /* Status enable mask (0x0F command) */
    
    /* Enable states */
    bool rx_enabled;           /* RX path enabled */
    bool tx_enabled;           /* TX path enabled */
    
    /* Thresholds and filters */
    uint16_t tx_avail_thresh;  /* TX available interrupt threshold */
    uint16_t tx_start_thresh;  /* TX start threshold */
    uint16_t rx_early_thresh;  /* RX early interrupt threshold */
    uint32_t rx_filter;        /* RX filter configuration */
    
    /* Network */
    NICState *nic;
    NICConf conf;
    
    /* Side effects */
    const WindowSideEffect *side_effects;
    int num_side_effects;
    
    /* ISA ID sequence (3C509 only) */
    EL3IDState id_state;
    
    /* Statistics counters */
    bool stats_enabled;  /* Statistics collection enabled */
    bool stats_frozen;   /* Statistics frozen for atomic read (StatsEnable) */
    struct {
        uint8_t tx_carrier_errors;
        uint8_t tx_heartbeat_errors;
        uint8_t tx_mult_collisions;
        uint8_t tx_single_collisions;
        uint8_t tx_late_collisions;
        uint8_t rx_overruns;
        uint8_t tx_frames_ok;      /* 8-bit counter (per 3C509B docs) */
        uint8_t rx_frames_ok;
        uint8_t tx_deferrals;
        uint16_t rx_bytes_ok;
        uint16_t tx_bytes_ok;
        
        /* 3Com-specific TX error counters */
        uint8_t tx_underruns;      /* TX FIFO underrun count */
        uint8_t tx_oversize;       /* Oversize frame count (if enforced) */
    } stats;
    
    /* Frozen snapshot of statistics for atomic reads */
    struct {
        uint8_t tx_carrier_errors;
        uint8_t tx_heartbeat_errors;
        uint8_t tx_mult_collisions;
        uint8_t tx_single_collisions;
        uint8_t tx_late_collisions;
        uint8_t rx_overruns;
        uint8_t tx_frames_ok;
        uint8_t rx_frames_ok;
        uint8_t tx_deferrals;
        uint16_t rx_bytes_ok;
        uint16_t tx_bytes_ok;
        uint8_t tx_underruns;
        uint8_t tx_oversize;
    } stats_snapshot;
    
    /* RX data buffer - holds only packet data, no headers. Sized to one FDDI-sized frame. */
    uint8_t rx_data[8192];
    uint16_t rx_data_write_ptr;  /* Write position in data buffer */
    uint16_t rx_data_used;       /* Bytes currently in data buffer */
    
    /* RX packet descriptor queue */
    RXPacketDesc rx_packets[RX_MAX_PACKETS];
    uint8_t rx_packet_head;      /* Index of first packet */
    uint8_t rx_packet_tail;      /* Index of next free slot */
    uint8_t rx_packet_count;     /* Number of packets in queue */
    
    /* TX FIFO for 3C509B (4KB) */
    uint8_t tx_fifo[4096];
    uint16_t tx_fifo_write_ptr;  /* Write position in FIFO */
    uint16_t tx_fifo_read_ptr;   /* Read position in FIFO */
    uint16_t tx_fifo_used;        /* Bytes currently in FIFO */
    
    /* TX state management */
    uint16_t current_tx_len;     /* Frame length from the PIO TX preamble */
    uint16_t current_tx_written; /* Bytes written so far */
    uint8_t tx_preamble_pos;     /* 3c509 PIO TX preamble byte counter (0..4) */
    bool tx_in_progress;         /* TX operation in progress */
    uint16_t tx_status;           /* TX status register (W1C) */
    QEMUBH *tx_bh;                /* Bottom half for async TX */
    bool internal_loopback;      /* Internal loopback mode */
    bool tx_blocked;             /* TX backpressure state */

    /* Hardware-delay timing model (opt-in via the 'realtiming' property). When off, TX/EEPROM/
     * commands complete instantly (fast). When on, they take modeled time off the virtual
     * clock -- run the guest with -icount so its busy-waits line up with these.
     *
     * icount fidelity limits (QEMU has no per-access bus-latency API, and MAX_ICOUNT_SHIFT=10):
     *   - One time-per-instruction rate: icount cannot model a fast CPU with slow (wait-stated)
     *     ISA I/O. shift=10 (~1 MIPS, ~1 us/insn) happens to make ISA I/O realistic AND models
     *     ~286 speed -- the sweet spot. At faster shifts the CPU is faster but ISA I/O is also
     *     "faster", so a guest's blind io_delay (which counts on ~1 us/access) under-waits.
     *   - Consequence: at shift<9 the driver's ~162 us EEPROM read-wait may under-run the busy
     *     window and read the placeholder. The TX wire-time model is unaffected (pure virtual
     *     clock). Use shift=10 for EEPROM-correct realtiming runs. */
    bool realtiming;             /* model hardware delays (else instant) */
    uint32_t tx_ns_per_byte;     /* wire time per byte (10BaseT = 800 ns) */
    int64_t tx_drain_deadline_ns;/* virtual-clock time when the TX FIFO finishes draining */
    QEMUTimer *tx_timer;         /* fires at the drain deadline -> TxComplete + IRQ */
    /* Realtiming bus-master DOWN completions: the descriptor DN_COMPLETE write-back is PACED to
     * the modeled transfer deadline (real HW sets it when the DMA actually finishes). Writing it
     * synchronously at StartDmaDown let descriptor-polling drivers retire + re-kick at CPU speed,
     * bypassing the wire/bus pacing entirely (a 10 Mbit link measured 19.8 Mbit). Each pending
     * entry completes (write-back + TxComplete latch) in el3_tx_drain_timer_cb at its own due
     * time. Not in vmstate: an in-flight paced completion lost across snapshot just means the
     * driver re-kicks -- acceptable for this test device. */
#define EL3_DN_PEND_MAX 8
    struct {
        uint64_t addr;           /* descriptor guest-phys address */
        uint32_t status;         /* status dword to write back (DN_COMPLETE set) */
        int64_t due_ns;          /* this transfer's accumulated wire/bus deadline */
    } dn_pend[EL3_DN_PEND_MAX];
    int dn_pend_n;
    /* Streaming TX: the FIFO drains at wire rate while the driver fills it, so TxFree falls as
     * bytes are written and rises as they drain. This lets interrupt-driven chunked drivers
     * (e.g. 3Com's 3C5X9PD, which writes the frame in [0x2fc]-byte slices) pace correctly and
     * stops TxAvailable from firing spuriously mid-frame. */
    uint32_t tx_occupancy;       /* bytes still "in the FIFO / on the wire" (drains over time) */
    int64_t tx_drain_last_ns;    /* virtual-clock time tx_occupancy was last advanced */
    bool tx_wire_active;          /* card has started clocking the current burst onto the wire
                                   * (occupancy reached tx_start_thresh) -- Parallel Tasking early-start */
    bool tx_avail_armed;          /* TX Available is ONE-SHOT (3c5x9b tech ref): SetTxAvailableThreshold
                                   * arms it; it fires once when free >= threshold, then disarms until
                                   * the driver reissues the command. Not a free-running level/edge. */
    /* (cmd_timer already declared above -- reused to clear STAT_CMD_IN_PROG) */

    /* Parallel Tasking instrumentation -- measures the host/wire overlap and CPU cost so the
     * advantage of 3Com's async/early-start TX vs the synchronous drivers is quantifiable.
     * Dumped to stderr at exit (see el3_pt_dump). All times are QEMU_CLOCK_VIRTUAL ns. */
    struct {
        uint64_t tx_frames;          /* frames handed to the wire */
        uint64_t tx_bytes;           /* frame bytes (payload, pre-pad) */
        uint64_t txfree_polls;       /* W1_TX_FREE reads -- busy-wait proxy (sync drivers spin here) */
        uint64_t fifo_writes;        /* TX FIFO write accesses (port OUTs) */
        uint64_t tx_avail_irqs;      /* TxAvailable interrupt assertions (rising edge) */
        uint64_t tx_complete_irqs;   /* TxComplete interrupt assertions */
        uint64_t rx_complete_irqs;   /* RxComplete interrupt assertions */
        int64_t  fill_ns_total;      /* sum of per-frame fill spans (first->last FIFO byte) */
        int64_t  wire_ns_total;      /* sum of per-frame wire-drain times */
        int64_t  overlap_ns_total;   /* sum of per-frame host/wire overlap (Parallel Tasking gain) */
        int64_t  frame_first_write_ns; /* in-progress: first FIFO write of the current frame */
        int64_t  frame_wire_start_ns;  /* in-progress: when the wire started (early-start) */
        bool     pt_active;          /* any TX seen -> dump at exit */
    } pt;
    
    /* AioContext for BH operations */
    AioContext *ctx;             /* Current AioContext */
    
    /* Multicast hash table (raw bytes for endian safety) */
    uint8_t mcast_hash[8];        /* 64-bit hash table as bytes */
    
    /* DMA status registers (3C59x) */
    uint16_t up_status;           /* UpList status */
    uint16_t down_status;         /* DownList status */
    
    /* 3C515 DMA engine state */
    AddressSpace *dma_as;         /* Address space for bus-master DMA (set by the device) */
    uint32_t up_list_ptr;         /* Upload list pointer */
    uint32_t down_list_ptr;       /* Download list pointer */
    bool up_stalled;              /* Upload stalled */
    bool down_stalled;            /* Download stalled */
    bool rx_dma_armed;            /* StartDmaUp armed a single-transfer RX-DMA into up_list_ptr */
    
    /* 3C515 DMA pacing */
    QEMUTimer *dma_timer;         /* Timer for DMA pacing */
    QEMUBH *dma_bh;               /* Bottom half for DMA operations */
    uint32_t dma_budget_bytes;    /* Bytes remaining in current budget */
    uint64_t dma_budget_ns;       /* Nanoseconds remaining in current budget */
    uint32_t dma_rate_bps;        /* DMA rate in bytes per second */
    
    /* PIO fallback support */
    bool bus_master_enabled;      /* Bus master mode enabled */
    
    /* IRQ level tracking */
    bool irq_level;              /* Current IRQ level for level-triggered interrupts */
    
    /* MII/PHY support */
    uint16_t phy_regs[32];        /* PHY registers */
    uint32_t phy_id;              /* PHY identifier */
    bool autoneg_complete;        /* Auto-negotiation complete */
    uint16_t link_partner_adv;    /* Link partner advertisement */
    bool autoneg_enabled;         /* Auto-negotiation enabled */
    int link_speed;               /* Link speed (10 or 100) */
    bool full_duplex;             /* Full duplex mode */
    bool link_up;                 /* Link status */
    QEMUTimer *media_timer;       /* Media status timer */
};

/* Core functions */
void el3_core_init(EL3Core *c, EL3Model model, const EL3VariantOps *ops);
void el3_core_reset(EL3Core *c);

/* Register access dispatch (calls core or variant ops) */
uint32_t el3_core_register_read(EL3Core *c, unsigned win, unsigned off, unsigned size);
void el3_core_register_write(EL3Core *c, unsigned win, unsigned off, uint32_t val, unsigned size);

/* Single-transfer bus-master TX DMA (3C515): process one download descriptor at desc_addr. */
void el3_core_dma_tx_single(EL3Core *c, AddressSpace *as, hwaddr desc_addr);

/* Legacy compatibility wrappers */
uint32_t el3_core_read(EL3Core *c, hwaddr addr, unsigned size);
void el3_core_write(EL3Core *c, hwaddr addr, uint32_t val, unsigned size);

/* Networking functions */
bool el3_core_can_receive(NetClientState *nc);
ssize_t el3_core_receive(NetClientState *nc, const uint8_t *buf, size_t size);
void el3_core_set_link_status(NetClientState *nc);
void el3_core_tx_submit(EL3Core *c, const uint8_t *buf, size_t len);
void el3_update_irq(EL3Core *c);
void el3_update_tx_available(EL3Core *c);

/* Window management */
void el3_select_window(EL3Core *c, uint8_t window);

/* EEPROM functions */
void el3_eeprom_init_3c509(EL3Core *c);
void el3_eeprom_init_3c59x(EL3Core *c);
uint16_t el3_eeprom_read(EL3Core *c);
void el3_eeprom_cmd(EL3Core *c, uint16_t cmd);

/* Helper functions */
uint16_t el3_get_tx_free(EL3Core *c);
uint16_t el3_get_rx_free(EL3Core *c);

/* Core state management */
uint8_t el3_core_get_window(EL3Core *c);
bool el3_core_is_rx_enabled(EL3Core *c);
bool el3_core_is_tx_enabled(EL3Core *c);
uint16_t el3_core_get_status(EL3Core *c);
void el3_core_set_status(EL3Core *c, uint16_t bits);
void el3_core_clear_status(EL3Core *c, uint16_t bits);
uint16_t el3_core_get_int_status(EL3Core *c);
void el3_core_set_int_status(EL3Core *c, uint16_t bits);
void el3_core_clear_int_status(EL3Core *c, uint16_t bits);
bool el3_core_should_interrupt(EL3Core *c);
void el3_core_change_aio_ctx(EL3Core *c, AioContext *ctx);

/* Network interface management */
bool el3_core_init_nic(EL3Core *c, DeviceState *dev, Error **errp);
void el3_core_post_realize(EL3Core *c);
void el3_core_unrealize(EL3Core *c);

/* TX/RX management */
uint32_t el3_core_get_pending_tx_len(EL3Core *c);
bool el3_core_has_tx_space(EL3Core *c, size_t len);
void el3_core_tx_complete(EL3Core *c, uint32_t status);
void el3_core_rx_notify(EL3Core *c, size_t len);

/* Statistics management */
void el3_core_update_stats(EL3Core *c, bool tx, bool ok, size_t bytes);
void el3_core_clear_stats(EL3Core *c);

/* Command processing */
void el3_core_process_command(EL3Core *c, uint16_t cmd);

/* RX filter functions */
void el3_set_rx_filter(EL3Core *c, uint16_t filter);
bool el3_accept_frame(EL3Core *c, const uint8_t *buf, size_t len);

/* Multi-size FIFO operations */
typedef struct {
    Fifo8 fifo;             /* Underlying byte FIFO */
    uint32_t capacity;      /* FIFO capacity in bytes */
    uint8_t width_mask;     /* Supported access widths (bit 0=8, bit 1=16, bit 2=32) */
} EL3MultiSizeFifo;

void el3_fifo_init(EL3MultiSizeFifo *mf, uint32_t capacity, uint8_t width_mask);
void el3_fifo_reset(EL3MultiSizeFifo *mf);
void el3_fifo_cleanup(EL3MultiSizeFifo *mf);

/* Multi-size FIFO operations */
uint32_t el3_fifo_read(EL3MultiSizeFifo *mf, unsigned size);
void el3_fifo_write(EL3MultiSizeFifo *mf, uint32_t data, unsigned size);
bool el3_fifo_push_packet(EL3MultiSizeFifo *mf, const uint8_t *data, size_t len);
size_t el3_fifo_pop_packet(EL3MultiSizeFifo *mf, uint8_t *buf, size_t max_len);
uint32_t el3_fifo_used(EL3MultiSizeFifo *mf);
uint32_t el3_fifo_free(EL3MultiSizeFifo *mf);
bool el3_fifo_is_empty(EL3MultiSizeFifo *mf);
bool el3_fifo_is_full(EL3MultiSizeFifo *mf);

/* PCI DMA functions (3C59x) */
void el3_dma_init(EL3DMAEngine *dma, AddressSpace *as, void *opaque, AioContext *ctx);
void el3_dma_cleanup(EL3DMAEngine *dma);
void el3_dma_change_aio_ctx(EL3DMAEngine *dma, AioContext *ctx);
void el3_dma_reset(EL3DMAEngine *dma);
void el3_dma_set_ring_base(EL3DMAEngine *dma, hwaddr addr, uint32_t size);
bool el3_dma_start_download(EL3DMAEngine *dma);
bool el3_dma_start_upload(EL3DMAEngine *dma);
void el3_dma_kick(EL3DMAEngine *dma, bool is_tx);
bool el3_dma_is_idle(EL3DMAEngine *dma);
uint32_t el3_dma_get_status(EL3DMAEngine *dma);

/* DMA packet submission */
void el3_dma_tx_submit(EL3DMAEngine *dma, const uint8_t *data, size_t len);
void el3_dma_rx_submit(EL3DMAEngine *dma, const uint8_t *data, size_t len, uint32_t status);

/* BH handlers (internal) */
void el3_dma_tx_bh(void *opaque);
void el3_dma_rx_bh(void *opaque);

/* ISA ID sequence functions (3C509) */
void el3_id_port_write(EL3Core *c, uint8_t val);
uint8_t el3_id_port_read(EL3Core *c);
void el3_id_sequence_reset(EL3Core *c);

/* MII/PHY functions */
void el3_phy_init(EL3Core *c);
uint16_t el3_phy_read(EL3Core *c, uint8_t reg);
void el3_phy_write(EL3Core *c, uint8_t reg, uint16_t val);
void el3_autoneg_complete(EL3Core *c);
void el3_media_timer_cb(void *opaque);
void el3_mii_command(EL3Core *c, uint16_t cmd);
uint16_t el3_mii_data_read(EL3Core *c);

/* VMState descriptors for migration */
extern const VMStateDescription vmstate_el3_core;
extern const VMStateDescription vmstate_el3_dma_engine;
extern const VMStateDescription vmstate_el3_multisize_fifo;
extern const VMStateDescription vmstate_el3_id_state;

#endif /* HW_NET_EL3_CORE_H */