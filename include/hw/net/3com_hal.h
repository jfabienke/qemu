/**
 * Hardware Abstraction Layer (HAL) Interface
 * 
 * This interface defines the abstraction between the packet driver and the
 * underlying 3Com NIC hardware. It provides a clean separation that will
 * facilitate both driver development and QEMU emulation.
 */

#ifndef HAL_INTERFACE_H
#define HAL_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>

/* Forward declarations */
struct hal_device;
struct packet_buffer;
struct dma_descriptor;

/* HAL Error Codes */
typedef enum {
    HAL_SUCCESS = 0,
    HAL_ERROR_NOT_FOUND = -1,
    HAL_ERROR_INVALID_PARAM = -2,
    HAL_ERROR_NO_MEMORY = -3,
    HAL_ERROR_TIMEOUT = -4,
    HAL_ERROR_NO_LINK = -5,
    HAL_ERROR_DMA_FAILURE = -6,
    HAL_ERROR_BUSY = -7,
    HAL_ERROR_NOT_SUPPORTED = -8
} hal_error_t;

/* NIC Model Types */
typedef enum {
    NIC_MODEL_UNKNOWN = 0,
    NIC_MODEL_3C509B,
    NIC_MODEL_3C515TX,
    NIC_MODEL_3C509B_COMBO,
    NIC_MODEL_3C515TX_ISA
} nic_model_t;

/* Media Types */
typedef enum {
    MEDIA_TYPE_NONE = 0,
    MEDIA_TYPE_10BASE_T,
    MEDIA_TYPE_10BASE_2,
    MEDIA_TYPE_10BASE_5,
    MEDIA_TYPE_100BASE_TX,
    MEDIA_TYPE_100BASE_FX,
    MEDIA_TYPE_AUTO
} media_type_t;

/* Link State */
typedef enum {
    LINK_STATE_DOWN = 0,
    LINK_STATE_UP,
    LINK_STATE_NEGOTIATING
} link_state_t;

/* Interrupt Status Flags */
#define HAL_INT_RX_COMPLETE     0x0001
#define HAL_INT_TX_COMPLETE     0x0002
#define HAL_INT_ADAPTER_FAIL    0x0004
#define HAL_INT_TX_AVAILABLE    0x0008
#define HAL_INT_RX_EARLY        0x0010
#define HAL_INT_STATS_FULL      0x0020
#define HAL_INT_DMA_DONE        0x0040
#define HAL_INT_DOWN_COMPLETE   0x0080
#define HAL_INT_UP_COMPLETE     0x0100
#define HAL_INT_CMD_COMPLETE    0x0200

/* RX Filter Modes */
#define HAL_RX_FILTER_INDIVIDUAL   0x01
#define HAL_RX_FILTER_MULTICAST    0x02
#define HAL_RX_FILTER_BROADCAST    0x04
#define HAL_RX_FILTER_PROMISCUOUS  0x08
#define HAL_RX_FILTER_ALL_MULTI    0x10

/* Device Capabilities */
typedef struct {
    bool has_bus_master;
    bool has_mii;
    bool has_auto_negotiation;
    bool has_full_duplex;
    bool has_100mbps;
    uint16_t tx_fifo_size;
    uint16_t rx_fifo_size;
    uint8_t num_windows;
    uint8_t eeprom_size;
} hal_capabilities_t;

/* Device Statistics */
typedef struct {
    uint32_t tx_packets;
    uint32_t rx_packets;
    uint32_t tx_bytes;
    uint32_t rx_bytes;
    uint32_t tx_errors;
    uint32_t rx_errors;
    uint32_t tx_dropped;
    uint32_t rx_dropped;
    uint16_t tx_collisions;
    uint16_t rx_overruns;
    uint16_t tx_underruns;
    uint16_t rx_crc_errors;
    uint16_t rx_frame_errors;
} hal_statistics_t;

/* Device Configuration */
typedef struct {
    uint16_t io_base;
    uint8_t irq;
    uint8_t mac_address[6];
    media_type_t media_type;
    bool enable_bus_master;
    bool enable_full_duplex;
    uint16_t tx_threshold;
    uint16_t rx_filter;
} hal_config_t;

/* HAL Device Operations */
typedef struct hal_operations {
    /* Initialization and Detection */
    hal_error_t (*probe)(uint16_t io_base, nic_model_t *model);
    hal_error_t (*init)(struct hal_device *dev, const hal_config_t *config);
    hal_error_t (*reset)(struct hal_device *dev);
    hal_error_t (*shutdown)(struct hal_device *dev);
    
    /* Configuration */
    hal_error_t (*set_mac_address)(struct hal_device *dev, const uint8_t mac[6]);
    hal_error_t (*get_mac_address)(struct hal_device *dev, uint8_t mac[6]);
    hal_error_t (*set_media_type)(struct hal_device *dev, media_type_t type);
    hal_error_t (*set_rx_filter)(struct hal_device *dev, uint16_t filter);
    
    /* Link Management */
    hal_error_t (*get_link_state)(struct hal_device *dev, link_state_t *state);
    hal_error_t (*get_link_speed)(struct hal_device *dev, uint16_t *speed);
    hal_error_t (*auto_negotiate)(struct hal_device *dev);
    
    /* Packet Operations */
    hal_error_t (*transmit)(struct hal_device *dev, const uint8_t *data, uint16_t length);
    hal_error_t (*receive)(struct hal_device *dev, uint8_t *buffer, uint16_t *length);
    hal_error_t (*can_transmit)(struct hal_device *dev, bool *ready);
    hal_error_t (*has_packet)(struct hal_device *dev, bool *available);
    
    /* Interrupt Handling */
    hal_error_t (*enable_interrupts)(struct hal_device *dev, uint16_t mask);
    hal_error_t (*disable_interrupts)(struct hal_device *dev, uint16_t mask);
    hal_error_t (*get_interrupt_status)(struct hal_device *dev, uint16_t *status);
    hal_error_t (*acknowledge_interrupt)(struct hal_device *dev, uint16_t status);
    
    /* Statistics */
    hal_error_t (*get_statistics)(struct hal_device *dev, hal_statistics_t *stats);
    hal_error_t (*clear_statistics)(struct hal_device *dev);
    
    /* EEPROM Access */
    hal_error_t (*read_eeprom)(struct hal_device *dev, uint8_t offset, uint16_t *value);
    hal_error_t (*write_eeprom)(struct hal_device *dev, uint8_t offset, uint16_t value);
    
    /* DMA Operations (3C515 only) */
    hal_error_t (*setup_dma_rx)(struct hal_device *dev, struct dma_descriptor *ring, uint16_t count);
    hal_error_t (*setup_dma_tx)(struct hal_device *dev, struct dma_descriptor *ring, uint16_t count);
    hal_error_t (*start_dma_rx)(struct hal_device *dev);
    hal_error_t (*start_dma_tx)(struct hal_device *dev);
    hal_error_t (*stop_dma_rx)(struct hal_device *dev);
    hal_error_t (*stop_dma_tx)(struct hal_device *dev);
    
    /* Diagnostics */
    hal_error_t (*run_self_test)(struct hal_device *dev, uint16_t *result);
    hal_error_t (*get_register_dump)(struct hal_device *dev, uint16_t *regs, uint16_t count);
    hal_error_t (*loopback_test)(struct hal_device *dev, bool enable);
} hal_operations_t;

/* HAL Device Structure */
struct hal_device {
    /* Device Information */
    nic_model_t model;
    uint16_t io_base;
    uint8_t irq;
    uint8_t mac_address[6];
    
    /* Current State */
    uint8_t current_window;
    link_state_t link_state;
    media_type_t media_type;
    uint16_t link_speed;
    bool is_initialized;
    bool interrupts_enabled;
    
    /* Capabilities */
    hal_capabilities_t capabilities;
    
    /* Operations */
    const hal_operations_t *ops;
    
    /* Private Data */
    void *priv;
};

/* Global HAL Functions */

/**
 * Initialize the HAL subsystem
 */
hal_error_t hal_init(void);

/**
 * Shutdown the HAL subsystem
 */
hal_error_t hal_shutdown(void);

/**
 * Detect all available NICs
 */
hal_error_t hal_detect_devices(struct hal_device **devices, uint8_t *count);

/**
 * Create a HAL device for a specific NIC
 */
hal_error_t hal_create_device(uint16_t io_base, uint8_t irq, struct hal_device **dev);

/**
 * Destroy a HAL device
 */
hal_error_t hal_destroy_device(struct hal_device *dev);

/**
 * Register model-specific operations
 */
hal_error_t hal_register_operations(nic_model_t model, const hal_operations_t *ops);

/* Utility Functions */

/**
 * Convert model to string
 */
const char* hal_model_to_string(nic_model_t model);

/**
 * Convert media type to string
 */
const char* hal_media_to_string(media_type_t media);

/**
 * Convert error code to string
 */
const char* hal_error_to_string(hal_error_t error);

/* Register Access Macros (for implementation) */

#define HAL_WRITE8(base, offset, value)   outb((base) + (offset), (value))
#define HAL_WRITE16(base, offset, value)  outw((base) + (offset), (value))
#define HAL_WRITE32(base, offset, value)  outl((base) + (offset), (value))

#define HAL_READ8(base, offset)           inb((base) + (offset))
#define HAL_READ16(base, offset)          inw((base) + (offset))
#define HAL_READ32(base, offset)          inl((base) + (offset))

/* Window Selection Helper */
#define HAL_SELECT_WINDOW(base, window)   HAL_WRITE16(base, 0x0E, 0x0800 | (window))

/* Command Helper */
#define HAL_ISSUE_COMMAND(base, cmd)      HAL_WRITE16(base, 0x0E, (cmd))

/* Timing Helpers */
#define HAL_DELAY_US(us)    delay_us(us)
#define HAL_DELAY_MS(ms)    delay_ms(ms)

/* DMA Descriptor Structure (for 3C515) */
struct dma_descriptor {
    uint32_t next;      /* Physical address of next descriptor */
    uint32_t status;    /* Status and control bits */
    uint32_t addr;      /* Physical buffer address */
    uint32_t length;    /* Buffer length and flags */
};

/* DMA Descriptor Flags */
#define DMA_DESC_COMPLETE   0x8000
#define DMA_DESC_ERROR      0x4000
#define DMA_DESC_LAST       0x2000
#define DMA_DESC_FIRST      0x1000
#define DMA_DESC_DN_COMPLETE 0x10000
#define DMA_DESC_UP_COMPLETE 0x20000

/* Packet Buffer Structure */
struct packet_buffer {
    uint8_t *data;
    uint16_t length;
    uint16_t capacity;
    uint32_t physical_addr;  /* For DMA */
};

/* Debug Support */
#ifdef DEBUG
#define HAL_DEBUG(fmt, ...) printf("HAL: " fmt "\n", ##__VA_ARGS__)
#else
#define HAL_DEBUG(fmt, ...)
#endif

/* QEMU Detection Helper */
bool hal_is_qemu(void);

/* Performance Monitoring */
typedef struct {
    uint32_t io_reads;
    uint32_t io_writes;
    uint32_t interrupts;
    uint32_t dma_transfers;
    uint32_t packets_sent;
    uint32_t packets_received;
} hal_perf_counters_t;

hal_error_t hal_get_perf_counters(struct hal_device *dev, hal_perf_counters_t *counters);
hal_error_t hal_reset_perf_counters(struct hal_device *dev);

#endif /* HAL_INTERFACE_H */