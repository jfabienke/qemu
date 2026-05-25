#include "hw/net/el3_core.h"

/* CRC-16 constants */
#define CRC16_POLY 0x8005
#define CRC16_INIT 0xFFFF

/* EEPROM command opcodes */
#define EEPROM_CMD_READ  0x2
#define EEPROM_CMD_WRITE 0x1
#define EEPROM_CMD_ERASE 0x3
#define EEPROM_CMD_EWEN  0x0
#define EEPROM_CMD_EWDS  0x0
#define EEPROM_CMD_ERAL  0x0
#define EEPROM_CMD_WRAL  0x0

/* EEPROM command masks */
#define EEPROM_CMD_MASK  0x3
#define EEPROM_ADDR_MASK 0x3F
#define EEPROM_DATA_MASK 0xFFFF

/* EEPROM timing */
#define EEPROM_WRITE_DELAY_NS 10000000 /* 10ms */
#define EEPROM_ERASE_DELAY_NS 10000000 /* 10ms */

/* EEPROM state machine */
typedef enum {
    EEPROM_STATE_IDLE,
    EEPROM_STATE_READING,
    EEPROM_STATE_WRITING,
    EEPROM_STATE_ERASING,
    EEPROM_STATE_EWEN,
    EEPROM_STATE_EWDS,
    EEPROM_STATE_ERAL,
    EEPROM_STATE_WRAL
} EEPROMState;

static uint16_t crc16(uint16_t crc, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ CRC16_POLY;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static void el3_eeprom_update_crc(EL3Core *c)
{
    uint16_t crc = CRC16_INIT;
    uint8_t data[63 * 2];

    for (int i = 0; i < 63; i++) {
        data[i * 2] = c->eeprom[i] & 0xFF;
        data[i * 2 + 1] = (c->eeprom[i] >> 8) & 0xFF;
    }

    crc = crc16(crc, data, sizeof(data));
    c->eeprom[63] = crc;
}

void el3_eeprom_init_3c509(EL3Core *c)
{
    /* Initialize EEPROM with default values */
    memset(c->eeprom, 0xFF, sizeof(c->eeprom));

    /* Set MAC address */
    c->eeprom[0x0A] = c->conf.macaddr.a[1] | (c->conf.macaddr.a[0] << 8);
    c->eeprom[0x0B] = c->conf.macaddr.a[3] | (c->conf.macaddr.a[2] << 8);
    c->eeprom[0x0C] = c->conf.macaddr.a[5] | (c->conf.macaddr.a[4] << 8);

    /* Update CRC */
    el3_eeprom_update_crc(c);
}

void el3_eeprom_init_3c59x(EL3Core *c)
{
    el3_eeprom_init_3c509(c);
}

uint16_t el3_eeprom_read(EL3Core *c)
{
    uint16_t val = 0;

    if (c->eeprom_state == EEPROM_BUSY) {
        val |= EEPROM_BUSY;
    } else {
        val |= (c->eeprom[c->eeprom_addr] >> 8) & 0xFF;
        val |= (c->eeprom[c->eeprom_addr] & 0xFF) << 8;
    }

    return val;
}

static void el3_eeprom_write_word(EL3Core *c, uint8_t addr, uint16_t data)
{
    if (addr < 64) {
        c->eeprom[addr] = data;
        if (addr != 63) {
            el3_eeprom_update_crc(c);
        }
    }
}

static void el3_eeprom_erase_word(EL3Core *c, uint8_t addr)
{
    if (addr < 64) {
        c->eeprom[addr] = 0xFFFF;
        if (addr != 63) {
            el3_eeprom_update_crc(c);
        }
    }
}

static void el3_eeprom_erase_all(EL3Core *c)
{
    for (int i = 0; i < 64; i++) {
        c->eeprom[i] = 0xFFFF;
    }
    el3_eeprom_update_crc(c);
}

static void el3_eeprom_write_all(EL3Core *c, uint16_t data)
{
    for (int i = 0; i < 64; i++) {
        c->eeprom[i] = data;
    }
    el3_eeprom_update_crc(c);
}

static void el3_eeprom_timer_cb(void *opaque)
{
    EL3Core *c = opaque;

    switch (c->eeprom_state) {
    case EEPROM_STATE_WRITING:
        el3_eeprom_write_word(c, c->eeprom_addr, c->eeprom_data);
        c->eeprom_state = EEPROM_STATE_IDLE;
        break;
    case EEPROM_STATE_ERASING:
        el3_eeprom_erase_word(c, c->eeprom_addr);
        c->eeprom_state = EEPROM_STATE_IDLE;
        break;
    case EEPROM_STATE_ERAL:
        el3_eeprom_erase_all(c);
        c->eeprom_state = EEPROM_STATE_IDLE;
        break;
    case EEPROM_STATE_WRAL:
        el3_eeprom_write_all(c, c->eeprom_data);
        c->eeprom_state = EEPROM_STATE_IDLE;
        break;
    default:
        c->eeprom_state = EEPROM_STATE_IDLE;
        break;
    }
}

void el3_eeprom_cmd(EL3Core *c, uint16_t cmd)
{
    uint8_t opcode = (cmd >> 13) & EEPROM_CMD_MASK;
    uint8_t addr = (cmd >> 7) & EEPROM_ADDR_MASK;
    uint16_t data = cmd & EEPROM_DATA_MASK;

    switch (opcode) {
    case EEPROM_CMD_READ:
        c->eeprom_addr = addr;
        c->eeprom_state = EEPROM_STATE_IDLE;
        break;
    case EEPROM_CMD_WRITE:
        if (c->eeprom_state == EEPROM_STATE_IDLE) {
            c->eeprom_addr = addr;
            c->eeprom_data = data;
            c->eeprom_state = EEPROM_STATE_WRITING;
            timer_mod(c->eeprom_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + EEPROM_WRITE_DELAY_NS);
        }
        break;
    case EEPROM_CMD_ERASE:
        if (c->eeprom_state == EEPROM_STATE_IDLE) {
            c->eeprom_addr = addr;
            c->eeprom_state = EEPROM_STATE_ERASING;
            timer_mod(c->eeprom_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + EEPROM_ERASE_DELAY_NS);
        }
        break;
    case EEPROM_CMD_EWEN:
        if ((cmd >> 11) & 0x3 == 0x3) {
            c->eeprom_state = EEPROM_STATE_EWEN;
        }
        break;
    case EEPROM_CMD_EWDS:
        if ((cmd >> 11) & 0x3 == 0x0) {
            c->eeprom_state = EEPROM_STATE_EWDS;
        }
        break;
    case EEPROM_CMD_ERAL:
        if (c->eeprom_state == EEPROM_STATE_IDLE) {
            c->eeprom_state = EEPROM_STATE_ERAL;
            timer_mod(c->eeprom_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + EEPROM_ERASE_DELAY_NS);
        }
        break;
    case EEPROM_CMD_WRAL:
        if (c->eeprom_state == EEPROM_STATE_IDLE) {
            c->eeprom_data = data;
            c->eeprom_state = EEPROM_STATE_WRAL;
            timer_mod(c->eeprom_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + EEPROM_WRITE_DELAY_NS);
        }
        break;
    }
}