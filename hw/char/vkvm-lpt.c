/*
 * QEMU ISA LPT fake peripheral for vintage-kvm.
 *
 * This is a source drop-in for a QEMU tree, not a standalone program. It
 * models the PC-side LPT register file enough for the DOS Stage 1 transport
 * code to run packet stress tests against a fake Pico peer:
 *
 *   base+0      SPP data
 *   base+1      SPP status
 *   base+2      SPP control
 *   base+4      EPP data
 *   base+0x400  ECP DFIFO
 *   base+0x402  ECP ECR
 *
 * The model is intentionally functional, not cycle accurate. It validates the
 * DOS register choreography and packet framing; real throughput still needs
 * hardware.
 */

#include "qemu/osdep.h"
#include "hw/isa/isa.h"
#include "hw/qdev-properties.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define TYPE_VKVM_LPT "vkvm-lpt"
OBJECT_DECLARE_SIMPLE_TYPE(VkvmLptState, VKVM_LPT)

#define LPT_DATA       0
#define LPT_STATUS     1
#define LPT_CONTROL    2
#define LPT_EPP_DATA   4
#define LPT_ECP_DFIFO  0
#define LPT_ECR        2

#define CTRL_INIT      0x04
#define CTRL_BASE      0x0c
#define CTRL_NEG_REQ   0x06
#define CTRL_NEG_STRB  0x07
#define CTRL_NEG_ACK   0x04
#define CTRL_DIR_INPUT 0x20

#define STAT_SELECT    0x10
#define STAT_NACK      0x40
#define STAT_PHASE     0x80

#define ECR_EMPTY      0x01
#define ECR_FULL       0x02
#define ECR_DMA_EN     0x08
#define ECR_MODE_MASK  0xe0
#define ECR_MODE_ECP   0x60
#define ECR_MODE_EPP   0x80

#define XFLAG_BYTE     0x01
#define XFLAG_ECP      0x14
#define XFLAG_EPP      0x40

#define SOH            0x01
#define ETX            0x03
#define HEADER_LEN     5
#define TRAILER_LEN    3
#define OVERHEAD       (HEADER_LEN + TRAILER_LEN)
#define MAX_PAYLOAD    (256 - OVERHEAD)
#define MAX_PACKET     256

#define CMD_CAP_REQ    0x00
#define CMD_CAP_RSP    0x0f
#define CMD_CAP_ACK    0x0e
#define CMD_PING       0x10
#define CMD_PONG       0x11
#define CMD_SEND_BLOCK 0x20
#define CMD_RECV_BLOCK 0x21

#define CMD_STAGE0_PROBE0 0x50
#define CMD_STAGE0_PROBE1 0x31
#define CMD_STAGE0_PROBE2 0x32
#define CMD_STAGE0_PROBE3 0x38
#define CMD_STAGE0_GET_META 0x81
#define CMD_STAGE0_GET_BLOCK 0x82
#define CMD_STAGE0_ACK 0x06
#define CMD_STAGE0_NAK 0x15

#define STAGE0_BLOCK_SIZE 64

enum {
    STAGE0_RAW_IDLE = 0,
    STAGE0_RAW_BLOCK_LO = 1,
    STAGE0_RAW_BLOCK_HI = 2,
};

enum {
    VKVM_MODE_NONE = 0,
    VKVM_MODE_SPP = 1,
    VKVM_MODE_BYTE = 2,
    VKVM_MODE_EPP = 3,
    VKVM_MODE_ECP = 4,
};

#define INBOUND_CAP 131072
#define REVERSE_CAP 131072
#define ECP_FIFO_DEPTH 16

enum {
    VKVM_FAULT_NONE = 0,
    VKVM_FAULT_DMA_DISABLED = 1,
    VKVM_FAULT_ECP_EMPTY_STUCK = 2,
    VKVM_FAULT_ECP_FULL_STUCK = 3,
    VKVM_FAULT_CORRUPT_PONG = 4,
    VKVM_FAULT_DMA_SHORT = 5,
};

struct VkvmLptState {
    ISADevice parent_obj;

    MemoryRegion base_io;
    MemoryRegion ecp_io;

    uint32_t iobase;
    int32_t dma;
    uint8_t max_mode;
    uint8_t fault;
    bool trace;

    uint8_t data;
    uint8_t control;
    uint8_t ecr;

    uint8_t pending_xflag;
    uint8_t active_mode;

    uint8_t inbound[INBOUND_CAP];
    size_t inbound_len;
    uint8_t stage0_probe_pos;
    uint8_t stage0_raw_state;
    uint16_t stage0_block_no;

    uint8_t reverse[REVERSE_CAP];
    size_t reverse_head;
    size_t reverse_len;

    uint8_t nibble_phase;
    uint8_t reverse_half;
    uint8_t status_reads;

    bool byte_ack;
    uint8_t byte_data;

    uint8_t ecp_rx_fifo[ECP_FIFO_DEPTH];
    size_t ecp_rx_head;
    size_t ecp_rx_len;
    IsaDma *isa_dma;
    bool dma_dreq;
    bool dma_in_handler;
    uint64_t dma_tx_bytes;
    uint64_t dma_rx_bytes;

    uint8_t next_seq;
    uint64_t packets_in;
    uint64_t packets_out;
    uint64_t errors;
};

static const uint8_t stage2_stub[] = {
    0xb8, 0x00, 0x4c, 0xcd, 0x21, /* mov ax,4c00h; int 21h */
};

static const uint8_t stage0_stage1_stub[] = {
    0x0e, 0x1f, 0xba, 0x0e, 0x08, 0xb4, 0x09, 0xcd,
    0x21, 0xb8, 0x00, 0x4c, 0xcd, 0x21, 0x53, 0x54,
    0x41, 0x47, 0x45, 0x30, 0x5f, 0x48, 0x41, 0x4e,
    0x44, 0x4f, 0x46, 0x46, 0x5f, 0x4f, 0x4b, 0x0d,
    0x0a, 0x24,
};

#define vkvm_trace(s, fmt, ...)                                      \
    do {                                                             \
        if ((s)->trace) {                                            \
            qemu_log("vkvm-lpt: " fmt "\n", ## __VA_ARGS__);        \
        }                                                            \
    } while (0)

static uint16_t vkvm_crc16(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xffff;

    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i] << 8;
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static uint32_t vkvm_crc32(const uint8_t *buf, size_t len)
{
    uint32_t crc = 0xffffffffu;

    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xedb88320u;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc ^ 0xffffffffu;
}

static bool vkvm_push_reverse(VkvmLptState *s, uint8_t byte)
{
    size_t idx;
    bool was_empty = s->reverse_len == 0;

    if (s->reverse_len >= REVERSE_CAP) {
        s->errors++;
        return false;
    }

    idx = (s->reverse_head + s->reverse_len) % REVERSE_CAP;
    s->reverse[idx] = byte;
    s->reverse_len++;
    if (was_empty && s->reverse_half == 0 && s->status_reads == 0) {
        s->nibble_phase ^= STAT_PHASE;
    }
    return true;
}

static bool vkvm_ecp_dma_enabled(VkvmLptState *s)
{
    return s->isa_dma &&
           s->dma >= 0 &&
           s->fault != VKVM_FAULT_DMA_DISABLED &&
           (s->ecr & ECR_MODE_MASK) == ECR_MODE_ECP &&
           (s->ecr & ECR_DMA_EN) != 0;
}

static void vkvm_accept_byte(VkvmLptState *s, uint8_t byte);
static void vkvm_ecp_dma_update(VkvmLptState *s);

static void vkvm_push_le16(VkvmLptState *s, uint16_t value)
{
    vkvm_push_reverse(s, value & 0xff);
    vkvm_push_reverse(s, value >> 8);
}

static bool vkvm_pop_reverse(VkvmLptState *s, uint8_t *byte)
{
    if (s->reverse_len == 0) {
        return false;
    }

    *byte = s->reverse[s->reverse_head];
    s->reverse_head = (s->reverse_head + 1) % REVERSE_CAP;
    s->reverse_len--;
    return true;
}

static void vkvm_ecp_rx_clear(VkvmLptState *s)
{
    s->ecp_rx_head = 0;
    s->ecp_rx_len = 0;
}

static void vkvm_ecp_rx_fill(VkvmLptState *s)
{
    while (s->ecp_rx_len < ECP_FIFO_DEPTH && s->reverse_len > 0) {
        size_t idx = (s->ecp_rx_head + s->ecp_rx_len) % ECP_FIFO_DEPTH;

        if (!vkvm_pop_reverse(s, &s->ecp_rx_fifo[idx])) {
            break;
        }
        s->ecp_rx_len++;
    }
}

static bool vkvm_ecp_rx_pop(VkvmLptState *s, uint8_t *byte)
{
    vkvm_ecp_rx_fill(s);
    if (s->ecp_rx_len == 0) {
        return false;
    }

    *byte = s->ecp_rx_fifo[s->ecp_rx_head];
    s->ecp_rx_head = (s->ecp_rx_head + 1) % ECP_FIFO_DEPTH;
    s->ecp_rx_len--;
    vkvm_ecp_rx_fill(s);
    vkvm_ecp_dma_update(s);
    return true;
}

static bool vkvm_peek_reverse_nibble(VkvmLptState *s, uint8_t *nibble)
{
    uint8_t byte;

    if (s->reverse_len == 0) {
        return false;
    }

    byte = s->reverse[s->reverse_head];
    if (s->reverse_half) {
        *nibble = byte >> 4;
    } else {
        *nibble = byte & 0x0f;
    }
    return true;
}

static void vkvm_advance_reverse_nibble(VkvmLptState *s)
{
    uint8_t ignored;

    if (s->reverse_half && s->reverse_len == 1) {
        s->reverse_half = 0;
        vkvm_pop_reverse(s, &ignored);
        return;
    }

    s->nibble_phase ^= STAT_PHASE;
    s->reverse_half ^= 1;
    if (s->reverse_half == 0) {
        vkvm_pop_reverse(s, &ignored);
    }
}

static bool vkvm_encode_packet(uint8_t cmd, uint8_t seq, const uint8_t *payload,
                               size_t payload_len, uint8_t *out,
                               size_t *out_len)
{
    uint16_t crc;
    size_t total = payload_len + OVERHEAD;

    if (payload_len > MAX_PAYLOAD || total > MAX_PACKET) {
        return false;
    }

    out[0] = SOH;
    out[1] = cmd;
    out[2] = seq;
    out[3] = payload_len >> 8;
    out[4] = payload_len & 0xff;
    memcpy(&out[5], payload, payload_len);

    crc = vkvm_crc16(&out[1], 4 + payload_len);
    out[5 + payload_len] = crc >> 8;
    out[6 + payload_len] = crc & 0xff;
    out[7 + payload_len] = ETX;
    *out_len = total;
    return true;
}

static bool vkvm_queue_packet(VkvmLptState *s, uint8_t cmd,
                              const uint8_t *payload, size_t payload_len)
{
    uint8_t pkt[MAX_PACKET];
    uint8_t corrupt_payload[MAX_PAYLOAD];
    size_t len;

    if (s->fault == VKVM_FAULT_CORRUPT_PONG &&
        cmd == CMD_PONG && payload_len > 0) {
        memcpy(corrupt_payload, payload, payload_len);
        corrupt_payload[0] ^= 0xff;
        payload = corrupt_payload;
    }

    if (!vkvm_encode_packet(cmd, s->next_seq++, payload, payload_len,
                            pkt, &len)) {
        s->errors++;
        return false;
    }

    for (size_t i = 0; i < len; i++) {
        if (!vkvm_push_reverse(s, pkt[i])) {
            return false;
        }
    }

    s->packets_out++;
    vkvm_ecp_dma_update(s);
    return true;
}

static void vkvm_queue_cap_rsp(VkvmLptState *s)
{
    uint8_t payload[36] = {0};
    uint32_t size = sizeof(stage2_stub);
    uint32_t crc = vkvm_crc32(stage2_stub, sizeof(stage2_stub));

    payload[0] = 1;                    /* version_major */
    payload[1] = 0;                    /* version_minor */
    payload[23] = s->active_mode;      /* active_parallel_mode */

    payload[28] = size >> 24;
    payload[29] = size >> 16;
    payload[30] = size >> 8;
    payload[31] = size;

    payload[32] = crc >> 24;
    payload[33] = crc >> 16;
    payload[34] = crc >> 8;
    payload[35] = crc;

    vkvm_queue_packet(s, CMD_CAP_RSP, payload, sizeof(payload));
}

static void vkvm_queue_block(VkvmLptState *s, const uint8_t *payload,
                             size_t payload_len)
{
    uint8_t rsp[4 + 1 + 64] = {0};
    uint32_t block;

    if (payload_len < 4) {
        s->errors++;
        return;
    }

    block = ((uint32_t)payload[0] << 24) |
            ((uint32_t)payload[1] << 16) |
            ((uint32_t)payload[2] << 8) |
            payload[3];

    rsp[0] = payload[0];
    rsp[1] = payload[1];
    rsp[2] = payload[2];
    rsp[3] = payload[3];

    if (block == 0) {
        rsp[4] = sizeof(stage2_stub);
        memcpy(&rsp[5], stage2_stub, sizeof(stage2_stub));
        vkvm_queue_packet(s, CMD_RECV_BLOCK, rsp, 5 + sizeof(stage2_stub));
    } else {
        rsp[4] = 0;
        vkvm_queue_packet(s, CMD_RECV_BLOCK, rsp, 5);
    }
}

static uint16_t vkvm_sum16(const uint8_t *buf, size_t len)
{
    uint16_t sum = 0;

    for (size_t i = 0; i < len; i++) {
        sum += buf[i];
    }

    return sum;
}

static void vkvm_queue_stage0_meta(VkvmLptState *s)
{
    uint16_t size = sizeof(stage0_stage1_stub);
    uint16_t blocks = (size + STAGE0_BLOCK_SIZE - 1) / STAGE0_BLOCK_SIZE;
    uint16_t sum = vkvm_sum16(stage0_stage1_stub, size);

    vkvm_trace(s, "stage0 meta size=%u blocks=%u sum=0x%04x",
               size, blocks, sum);
    vkvm_push_le16(s, size);
    vkvm_push_le16(s, blocks);
    vkvm_push_le16(s, sum);
}

static void vkvm_queue_stage0_block(VkvmLptState *s, uint16_t block)
{
    size_t offset = (size_t)block * STAGE0_BLOCK_SIZE;
    size_t remaining;
    size_t len;
    uint16_t crc;

    if (offset >= sizeof(stage0_stage1_stub)) {
        vkvm_push_reverse(s, 0);
        vkvm_push_le16(s, vkvm_crc16(NULL, 0));
        return;
    }

    remaining = sizeof(stage0_stage1_stub) - offset;
    len = MIN(remaining, (size_t)STAGE0_BLOCK_SIZE);
    crc = vkvm_crc16(&stage0_stage1_stub[offset], len);

    vkvm_trace(s, "stage0 block=%u len=%zu crc=0x%04x", block, len, crc);
    vkvm_push_reverse(s, len);
    for (size_t i = 0; i < len; i++) {
        vkvm_push_reverse(s, stage0_stage1_stub[offset + i]);
    }
    vkvm_push_le16(s, crc);
}

static bool vkvm_stage0_probe_byte(VkvmLptState *s, uint8_t byte)
{
    static const uint8_t probe[] = {
        CMD_STAGE0_PROBE0, CMD_STAGE0_PROBE1,
        CMD_STAGE0_PROBE2, CMD_STAGE0_PROBE3,
    };

    if (byte == probe[s->stage0_probe_pos]) {
        s->stage0_probe_pos++;
        if (s->stage0_probe_pos == sizeof(probe)) {
            s->stage0_probe_pos = 0;
            vkvm_trace(s, "stage0 probe ok");
            vkvm_push_reverse(s, 'O');
            vkvm_push_reverse(s, 'K');
        }
        return true;
    }

    s->stage0_probe_pos = byte == CMD_STAGE0_PROBE0 ? 1 : 0;
    return s->stage0_probe_pos != 0;
}

static void vkvm_accept_stage0_byte(VkvmLptState *s, uint8_t byte)
{
    switch (s->stage0_raw_state) {
    case STAGE0_RAW_BLOCK_LO:
        s->stage0_block_no = byte;
        s->stage0_raw_state = STAGE0_RAW_BLOCK_HI;
        return;
    case STAGE0_RAW_BLOCK_HI:
        s->stage0_block_no |= (uint16_t)byte << 8;
        s->stage0_raw_state = STAGE0_RAW_IDLE;
        vkvm_queue_stage0_block(s, s->stage0_block_no);
        return;
    default:
        break;
    }

    if (vkvm_stage0_probe_byte(s, byte)) {
        return;
    }

    switch (byte) {
    case CMD_STAGE0_GET_META:
        vkvm_queue_stage0_meta(s);
        break;
    case CMD_STAGE0_GET_BLOCK:
        s->stage0_raw_state = STAGE0_RAW_BLOCK_LO;
        s->stage0_block_no = 0;
        break;
    case CMD_STAGE0_ACK:
    case CMD_STAGE0_NAK:
        break;
    default:
        s->errors++;
        break;
    }
}

static void vkvm_ecp_dma_set_dreq(VkvmLptState *s, bool hold)
{
    IsaDmaClass *k;

    if (!s->isa_dma || s->dma < 0 || s->dma_in_handler ||
        s->dma_dreq == hold) {
        return;
    }

    k = ISADMA_GET_CLASS(s->isa_dma);
    if (hold) {
        k->hold_DREQ(s->isa_dma, s->dma);
    } else {
        k->release_DREQ(s->isa_dma, s->dma);
    }
    s->dma_dreq = hold;
}

static void vkvm_ecp_dma_update(VkvmLptState *s)
{
    if (!vkvm_ecp_dma_enabled(s)) {
        vkvm_ecp_dma_set_dreq(s, false);
        return;
    }

    if (s->control & CTRL_DIR_INPUT) {
        vkvm_ecp_rx_fill(s);
        vkvm_ecp_dma_set_dreq(s, s->fault != VKVM_FAULT_ECP_EMPTY_STUCK &&
                              s->ecp_rx_len > 0);
    } else {
        vkvm_ecp_dma_set_dreq(s, true);
    }
}

static int vkvm_ecp_dma_transfer(void *opaque, int nchan, int dma_pos,
                                 int dma_len)
{
    VkvmLptState *s = opaque;
    IsaDmaClass *k;
    uint8_t buf[64];
    int pos = dma_pos;

    if (!vkvm_ecp_dma_enabled(s)) {
        return pos;
    }

    k = ISADMA_GET_CLASS(s->isa_dma);
    s->dma_in_handler = true;

    if (s->control & CTRL_DIR_INPUT) {
        while (pos < dma_len) {
            int got = 0;
            int chunk = MIN(dma_len - pos, (int)sizeof(buf));

            if (s->fault == VKVM_FAULT_DMA_SHORT && pos >= dma_len / 2) {
                break;
            }
            while (got < chunk && vkvm_ecp_rx_pop(s, &buf[got])) {
                got++;
            }
            if (got == 0) {
                break;
            }
            k->write_memory(s->isa_dma, nchan, buf, pos, got);
            pos += got;
            s->dma_rx_bytes += got;
        }
    } else {
        while (pos < dma_len) {
            int chunk = MIN(dma_len - pos, (int)sizeof(buf));
            int copied = k->read_memory(s->isa_dma, nchan, buf, pos, chunk);

            if (s->fault == VKVM_FAULT_DMA_SHORT && pos >= dma_len / 2) {
                break;
            }
            if (copied <= 0) {
                break;
            }
            for (int i = 0; i < copied; i++) {
                vkvm_accept_byte(s, buf[i]);
            }
            pos += copied;
            s->dma_tx_bytes += copied;
        }
    }

    s->dma_in_handler = false;
    if (pos >= dma_len) {
        vkvm_ecp_dma_set_dreq(s, false);
    } else if (s->fault == VKVM_FAULT_DMA_SHORT) {
        vkvm_ecp_dma_set_dreq(s, false);
    } else {
        vkvm_ecp_dma_update(s);
    }
    return pos;
}

static bool vkvm_decode_header(VkvmLptState *s, size_t *total)
{
    uint16_t payload_len;

    if (s->inbound_len < HEADER_LEN) {
        return false;
    }
    if (s->inbound[0] != SOH) {
        s->errors++;
        s->inbound_len = 0;
        return false;
    }

    payload_len = ((uint16_t)s->inbound[3] << 8) | s->inbound[4];
    if (payload_len > MAX_PAYLOAD) {
        s->errors++;
        s->inbound_len = 0;
        return false;
    }

    *total = payload_len + OVERHEAD;
    return s->inbound_len >= *total;
}

static void vkvm_process_packets(VkvmLptState *s)
{
    size_t total;

    while (vkvm_decode_header(s, &total)) {
        uint8_t cmd = s->inbound[1];
        uint16_t payload_len = ((uint16_t)s->inbound[3] << 8) | s->inbound[4];
        const uint8_t *payload = &s->inbound[5];
        uint16_t expected_crc = ((uint16_t)s->inbound[5 + payload_len] << 8) |
                                s->inbound[6 + payload_len];
        uint16_t actual_crc = vkvm_crc16(&s->inbound[1], 4 + payload_len);

        if (s->inbound[7 + payload_len] != ETX || expected_crc != actual_crc) {
            s->errors++;
            s->inbound_len = 0;
            return;
        }

        s->packets_in++;
        vkvm_trace(s, "packet in cmd=0x%02x payload=%u", cmd, payload_len);

        switch (cmd) {
        case CMD_CAP_REQ:
            vkvm_queue_cap_rsp(s);
            break;
        case CMD_CAP_ACK:
            break;
        case CMD_PING:
            vkvm_queue_packet(s, CMD_PONG, payload, payload_len);
            break;
        case CMD_SEND_BLOCK:
            vkvm_queue_block(s, payload, payload_len);
            break;
        default:
            break;
        }

        if (s->inbound_len > total) {
            memmove(s->inbound, &s->inbound[total], s->inbound_len - total);
        }
        s->inbound_len -= total;
    }
}

static void vkvm_accept_byte(VkvmLptState *s, uint8_t byte)
{
    if (s->inbound_len == 0 && byte != SOH) {
        vkvm_accept_stage0_byte(s, byte);
        return;
    }

    if (s->inbound_len >= INBOUND_CAP) {
        s->errors++;
        s->inbound_len = 0;
        return;
    }

    s->inbound[s->inbound_len++] = byte;
    vkvm_process_packets(s);
}

static uint8_t vkvm_mode_from_xflag(uint8_t xflag)
{
    switch (xflag) {
    case XFLAG_ECP:
        return VKVM_MODE_ECP;
    case XFLAG_EPP:
        return VKVM_MODE_EPP;
    case XFLAG_BYTE:
        return VKVM_MODE_BYTE;
    default:
        return VKVM_MODE_SPP;
    }
}

static bool vkvm_accepts_xflag(VkvmLptState *s)
{
    return vkvm_mode_from_xflag(s->pending_xflag) <= s->max_mode;
}

static uint8_t vkvm_status_read(VkvmLptState *s)
{
    uint8_t status = STAT_NACK | STAT_SELECT;
    uint8_t nibble;

    if (s->control == CTRL_NEG_REQ || s->control == CTRL_NEG_STRB) {
        status &= ~STAT_NACK;
        if (vkvm_accepts_xflag(s)) {
            status |= STAT_SELECT;
        } else {
            status &= ~STAT_SELECT;
        }
        return status;
    }

    if (s->byte_ack) {
        return status & ~STAT_NACK;
    }

    if (s->control & CTRL_DIR_INPUT) {
        return status;
    }

    if (vkvm_peek_reverse_nibble(s, &nibble)) {
        status = s->nibble_phase | ((nibble & 0x0f) << 3);
        /*
         * DOS reads each candidate phase twice for debounce. Advance after the
         * second read so the next lpt_recv_nibble call observes a fresh phase.
         */
        if (++s->status_reads >= 2) {
            s->status_reads = 0;
            vkvm_advance_reverse_nibble(s);
        }
        return status;
    }

    return s->nibble_phase;
}

static void vkvm_control_write(VkvmLptState *s, uint8_t value)
{
    bool prev_init_high = (s->control & CTRL_INIT) != 0;
    bool next_init_low = (value & CTRL_INIT) == 0;

    s->control = value;
    if ((value & CTRL_DIR_INPUT) == 0) {
        vkvm_ecp_rx_clear(s);
    }
    vkvm_ecp_dma_update(s);

    if (value == CTRL_NEG_ACK) {
        s->active_mode = vkvm_mode_from_xflag(s->pending_xflag);
        vkvm_trace(s, "negotiated mode=%u", s->active_mode);
        return;
    }

    if ((value & CTRL_DIR_INPUT) && (value & 0x02) && s->reverse_len > 0) {
        vkvm_pop_reverse(s, &s->byte_data);
        s->byte_ack = true;
        return;
    }

    if ((value & CTRL_DIR_INPUT) && !(value & 0x02)) {
        s->byte_ack = false;
        return;
    }

    if (prev_init_high && next_init_low) {
        vkvm_accept_byte(s, s->data);
    }
}

static uint64_t vkvm_base_read(void *opaque, hwaddr addr, unsigned size)
{
    VkvmLptState *s = opaque;
    uint8_t byte = 0xff;

    switch (addr) {
    case LPT_DATA:
        byte = s->byte_ack ? s->byte_data : s->data;
        break;
    case LPT_STATUS:
        byte = vkvm_status_read(s);
        break;
    case LPT_CONTROL:
        byte = s->control;
        break;
    case LPT_EPP_DATA:
        if (!vkvm_pop_reverse(s, &byte)) {
            byte = 0xff;
            s->errors++;
        }
        break;
    default:
        break;
    }

    return byte;
}

static void vkvm_base_write(void *opaque, hwaddr addr, uint64_t val,
                            unsigned size)
{
    VkvmLptState *s = opaque;
    uint8_t byte = val;

    switch (addr) {
    case LPT_DATA:
        s->data = byte;
        s->pending_xflag = byte;
        break;
    case LPT_CONTROL:
        vkvm_control_write(s, byte);
        break;
    case LPT_EPP_DATA:
        vkvm_accept_byte(s, byte);
        break;
    default:
        break;
    }
}

static uint64_t vkvm_ecp_read(void *opaque, hwaddr addr, unsigned size)
{
    VkvmLptState *s = opaque;
    uint8_t byte = 0xff;

    switch (addr) {
    case LPT_ECP_DFIFO:
        if ((s->ecr & ECR_MODE_MASK) != ECR_MODE_ECP ||
            (s->control & CTRL_DIR_INPUT) == 0 ||
            !vkvm_ecp_rx_pop(s, &byte)) {
            s->errors++;
        }
        return byte;
    case LPT_ECR:
        if ((s->ecr & ECR_MODE_MASK) != ECR_MODE_ECP) {
            return s->ecr;
        }
        byte = s->ecr & ~(ECR_EMPTY | ECR_FULL);
        if (s->control & CTRL_DIR_INPUT) {
            vkvm_ecp_rx_fill(s);
            if (s->ecp_rx_len == 0) {
                byte |= ECR_EMPTY;
            }
            if (s->ecp_rx_len >= ECP_FIFO_DEPTH) {
                byte |= ECR_FULL;
            }
            if (s->fault == VKVM_FAULT_ECP_EMPTY_STUCK) {
                byte |= ECR_EMPTY;
                byte &= ~ECR_FULL;
            }
            if (s->fault == VKVM_FAULT_ECP_FULL_STUCK) {
                byte |= ECR_FULL;
                byte &= ~ECR_EMPTY;
            }
        } else {
            byte |= ECR_EMPTY;
            if (s->fault == VKVM_FAULT_ECP_FULL_STUCK) {
                byte |= ECR_FULL;
            }
        }
        return byte;
    default:
        return 0xff;
    }
}

static void vkvm_ecp_write(void *opaque, hwaddr addr, uint64_t val,
                           unsigned size)
{
    VkvmLptState *s = opaque;
    uint8_t byte = val;

    switch (addr) {
    case LPT_ECP_DFIFO:
        vkvm_accept_byte(s, byte);
        vkvm_ecp_dma_update(s);
        break;
    case LPT_ECR:
        if ((s->ecr & ECR_MODE_MASK) != (byte & ECR_MODE_MASK)) {
            vkvm_ecp_rx_clear(s);
        }
        s->ecr = byte;
        vkvm_ecp_dma_update(s);
        break;
    default:
        break;
    }
}

static const MemoryRegionOps vkvm_base_ops = {
    .read = vkvm_base_read,
    .write = vkvm_base_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

static const MemoryRegionOps vkvm_ecp_ops = {
    .read = vkvm_ecp_read,
    .write = vkvm_ecp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

static void vkvm_lpt_realizefn(DeviceState *dev, Error **errp)
{
    ISADevice *isa = ISA_DEVICE(dev);
    VkvmLptState *s = VKVM_LPT(dev);
    ISABus *bus = isa_bus_from_device(isa);
    IsaDmaClass *k;

    if (s->max_mode < VKVM_MODE_SPP || s->max_mode > VKVM_MODE_ECP) {
        error_setg(errp, "max-mode must be 1(SPP)..4(ECP)");
        return;
    }
    if (s->dma < -1 || s->dma > 3) {
        error_setg(errp, "dma must be -1(disabled) or 0..3");
        return;
    }
    if (s->fault > VKVM_FAULT_DMA_SHORT) {
        error_setg(errp, "fault must be 0(none)..5(dma-short)");
        return;
    }

    if (s->dma >= 0) {
        s->isa_dma = isa_bus_get_dma(bus, s->dma);
        if (!s->isa_dma) {
            error_setg(errp, "ISA bus does not provide DMA channel %d", s->dma);
            return;
        }
        k = ISADMA_GET_CLASS(s->isa_dma);
        k->register_channel(s->isa_dma, s->dma, vkvm_ecp_dma_transfer, s);
    }

    s->control = CTRL_BASE;
    s->active_mode = VKVM_MODE_SPP;
    s->ecr = ECR_EMPTY;

    memory_region_init_io(&s->base_io, OBJECT(s), &vkvm_base_ops, s,
                          "vkvm-lpt-base", 8);
    isa_register_ioport(isa, &s->base_io, s->iobase);

    memory_region_init_io(&s->ecp_io, OBJECT(s), &vkvm_ecp_ops, s,
                          "vkvm-lpt-ecp", 3);
    isa_register_ioport(isa, &s->ecp_io, s->iobase + 0x400);

    vkvm_trace(s, "realized iobase=0x%x max_mode=%u", s->iobase, s->max_mode);
}

static const Property vkvm_lpt_properties[] = {
    DEFINE_PROP_UINT32("iobase", VkvmLptState, iobase, 0x378),
    DEFINE_PROP_INT32("dma", VkvmLptState, dma, 3),
    DEFINE_PROP_UINT8("max-mode", VkvmLptState, max_mode, VKVM_MODE_ECP),
    DEFINE_PROP_UINT8("fault", VkvmLptState, fault, VKVM_FAULT_NONE),
    DEFINE_PROP_BOOL("trace", VkvmLptState, trace, false),
};

static void vkvm_lpt_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = vkvm_lpt_realizefn;
    device_class_set_props(dc, vkvm_lpt_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo vkvm_lpt_info = {
    .name = TYPE_VKVM_LPT,
    .parent = TYPE_ISA_DEVICE,
    .instance_size = sizeof(VkvmLptState),
    .class_init = vkvm_lpt_class_init,
};

static void vkvm_lpt_register_types(void)
{
    type_register_static(&vkvm_lpt_info);
}

type_init(vkvm_lpt_register_types)
