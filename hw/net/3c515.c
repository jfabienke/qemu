/*
 * QEMU 3Com 3C515 Corkscrew emulation
 *
 * Copyright (c) 2024 QEMU contributors
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 * The 3C515 is an ISA PnP device that uses standard ISA enumeration
 * and configuration through iobase/irq properties, unlike the 3C509
 * which requires custom ID port-based enumeration at a fixed address.
 */

#include "qemu/osdep.h"
#include "hw/isa/isa.h"
#include "hw/core/qdev-properties.h"
#include "net/net.h"
#include "hw/net/el3_core.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/error-report.h"

#define TYPE_ISA_3C515 "3c515"
OBJECT_DECLARE_SIMPLE_TYPE(ISA3C515State, ISA_3C515)

#define ISA_3C515_IO_SIZE       0x20
/* The 3C515 mirrors the Window-0 EEPROM at the +0x2000 ISA alias (cmd 0x200A/data
 * 0x200C). Map it as a separate small region so we don't span the standard ISA
 * ports (COM/floppy/etc.) between 0x20 and 0x2000. */
#define ISA_3C515_EEPROM_ALIAS  0x2000
#define ISA_3C515_EEPROM_SIZE   0x10
/* Bus-master DMA register block. On the real Corkscrew (and the 3c59x family) the master
 * registers sit right after the windowed block at base+0x20..0x3F: DownListPtr = base+0x24
 * (region offset 0x04), UpListPtr = base+0x38 (region offset 0x18). NOT base+0x400 -- that
 * port (0x700 with iobase 0x300) gets shadowed by the PCI host bridge I/O window after BIOS
 * setup, so writes there never reach this region. */
#define ISA_3C515_DMA_BASE      0x20
#define ISA_3C515_DMA_SIZE      0x20
#define ISA_3C515_DEFAULT_IOBASE 0x300
#define ISA_3C515_DEFAULT_IRQ   10
/* Practical 16-bit ISA bus-master throughput (bytes/s). A 0-wait-state 16-bit ISA cycle on a
 * Pentium/late-486 chipset is 2 BCLK @ 8.33 MHz = 2 bytes/240ns = 8.33 MB/s peak; minus
 * DRAM-refresh steal + bus-master arbitration/turnaround, ~6 MB/s (~48 Mbit) sustained -- which
 * matches the ~20-40 Mbit the real 3C515 reached (never its marketed 100, ISA being the wall).
 * Slower/wait-state chipsets do less; tunable via the dma_rate property (and the auto-tune
 * milestone, design S7, would measure it per-board). Used to pace el3_core_dma_tx_single. */
#define ISA_3C515_DEFAULT_DMA_RATE (6 * 1024 * 1024)  /* 6 MB/s ~= 48 Mbit (0WS 16-bit ISA) */
#define ISA_3C515_ROM_SIZE       0x4000  /* 16KB ROM */

typedef struct ISA3C515State {
    ISADevice parent_obj;
    EL3Core core;
    MemoryRegion io;
    MemoryRegion rom_io;     /* EEPROM +0x2000 alias region (formerly ROM) */
    MemoryRegion dma_io;     /* bus-master DMA register block at base+0x400 */
    
    uint16_t iobase;
    uint8_t irq;
    uint8_t dmachan;     /* ISA DMA channel the bus master cascades through (DCR bits 0-2) */
    bool autoselect;     /* InternalConfig autoselect bit (EEPROM "driver, pick the media") */
    uint16_t linkspeed;  /* Mbit/s for the realtiming wire model (10 or 100) */
    qemu_irq irq_line;  /* Actual IRQ line */
    bool pnp_enable;
    /* ISA Plug and Play (pnp=on): the card powers up inactive and must be isolated + activated */
    MemoryRegion pnp_addr_io, pnp_write_io, pnp_rd_io;
    MemoryRegion pnp_chk_io;  /* I/O range check: answers [pnp_iobase, +0x20) with 0x55/0xAA */
    uint8_t pnp_iochk;       /* register 0x31: bit1 enable, bit0 0x55 (else 0xAA) */
    uint8_t pnp_state;       /* PNP_WFK / PNP_SLEEP / PNP_ISOL / PNP_CFG */
    uint8_t pnp_keypos;      /* initiation-key bytes matched so far */
    uint8_t pnp_addr;        /* last register index written to 0x279 */
    uint16_t pnp_rdp;        /* read-data port, 0 = not set */
    uint8_t pnp_csn, pnp_ldn, pnp_active, pnp_irq, pnp_irqtype, pnp_dma;
    uint16_t pnp_iobase;
    uint16_t pnp_bit;        /* isolation: next serial-id bit (0..71) */
    uint8_t pnp_half;        /* isolation: 0 = next read is the first of the pair */
    uint16_t pnp_resptr;     /* resource-data read pointer (serial id, then pnp_res) */
    uint8_t pnp_serial[9];
    uint32_t dma_rate_bps;
    char *romfile;
} ISA3C515State;

/* I/O read handler */
static uint64_t isa_3c515_io_read(void *opaque, hwaddr addr, unsigned size)
{
    ISA3C515State *s = opaque;
    return el3_core_read(&s->core, addr, size);
}

/* I/O write handler */
static void isa_3c515_io_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    ISA3C515State *s = opaque;
    el3_core_write(&s->core, addr, val, size);
}

/* EEPROM alias at iobase+0x2000: forward to el3_core with the +0x2000 offset so the
 * core's 3C515 normalization routes 0x200A/0x200C to the Window-0 EEPROM cmd/data. */
/* The +0x2000 alias registers are window-independent on real silicon (Linux 3c515 probes them
 * without selecting a window): +0x2000 DCR (bits 0-2 = the ISA DMA channel the bus master uses),
 * +0x2002 resource config (bits 4-8 = the I/O base's bits 4-8, bits 0-3 = the IRQ), +0x200A/+0x200C
 * the Window-0 EEPROM command/data. */
static uint64_t isa_3c515_eeprom_read(void *opaque, hwaddr addr, unsigned size)
{
    ISA3C515State *s = opaque;
    if (addr == 0x00) {
        return s->dmachan & 7;
    }
    if (addr == 0x02) {
        return (uint64_t)((s->iobase & 0x1F0) | (s->irq & 0x0F));
    }
    return el3_core_register_read(&s->core, 0, ISA_3C515_EEPROM_ALIAS + addr, size);
}

static void isa_3c515_eeprom_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    ISA3C515State *s = opaque;
    el3_core_register_write(&s->core, 0, ISA_3C515_EEPROM_ALIAS + addr, val, size);
}

/* ===========================================================================
 * ISA Plug and Play (pnp=on). The ISA PnP spec's card side, enough for a real isolator (our driver's
 * CFG_PNP build, isapnptools' pnpdump/isapnp): Wait-for-Key -> the 32-byte LFSR initiation key on
 * 0x279 -> Sleep; Wake[0] -> Isolation, where Set-RD_DATA is accepted (ONLY there -- a driver that
 * sets the read port before Wake[0] gets no isolation data) and the 72-bit serial identifier streams
 * out as 0x55/0xAA pairs; a CSN write -> Config, with Wake[CSN], the resource data (serial id, then the
 * tags below), and logical device 0's activate/I-O/IRQ/DMA registers. The card's I/O decode is off
 * until activated, then maps at the assigned base (and IRQ). Timing is not modelled.
 * =========================================================================== */
enum { PNP_WFK, PNP_SLEEP, PNP_ISOL, PNP_CFG };
#define PNP_ADDR_PORT   0x279
#define PNP_WRITE_PORT  0xA79

/* Resource data after the serial id, as a real 3C515 reports it (pnpdump on an IBM PC/AT, 2026-09-26):
 * PnP 1.0 / vendor 1.0, "3Com Fast EtherLink ISA", logical device TCM5051, IRQ 3/5/7/9/10/11/12/15,
 * 16-bit bus-master DMA 3/5/6/7, one I/O range 0x280-0x3E0 (align/len 0x20, 10-bit decode). */
static uint8_t pnp_res[] = {           /* last byte: the end-tag checksum, set in pnp_realize */
    0x0A, 0x10, 0x10,                              /* PnP 1.0, vendor version 1.0 */
    0x82, 0x17, 0x00, '3','C','o','m',' ','F','a','s','t',' ',
    'E','t','h','e','r','L','i','n','k',' ','I','S','A',   /* the real card's ANSI string */
    0x15, 0x50, 0x6D, 0x50, 0x51, 0x02,            /* logical device TCM5051, I/O range check */
    0x22, 0xA8, 0x9E,                              /* IRQ 3,5,7,9,10,11,12,15 */
    0x2A, 0xE8, 0x06,                              /* DMA 3,5,6,7: 16-bit only, bus master */
    0x47, 0x00, 0x80, 0x02, 0xE0, 0x03, 0x20, 0x20, /* I/O 0x280-0x3E0, align/len 0x20, 10-bit decode */
    0x79, 0x00,
};

static uint8_t pnp_key_byte(int i)
{
    uint8_t code = 0x6A;
    while (i-- > 0) {
        code = (uint8_t)((code >> 1) | ((((code & 1) ^ ((code >> 1) & 1))) << 7));
    }
    return code;
}

static void pnp_decode(ISA3C515State *s, bool on)
{
    if (on) {
        memory_region_set_address(&s->io, s->iobase);
        memory_region_set_address(&s->rom_io, s->iobase + ISA_3C515_EEPROM_ALIAS);
        memory_region_set_address(&s->dma_io, s->iobase + ISA_3C515_DMA_BASE);
    }
    memory_region_set_enabled(&s->io, on);
    memory_region_set_enabled(&s->rom_io, on);
    memory_region_set_enabled(&s->dma_io, on);
}

/* I/O range check (0x31): while enabled and the device inactive, the card answers its assigned range
 * with the pattern. Mapped BELOW other devices, so another card decoding those ports wins the read and
 * the pattern comes back wrong -- as a conflicting device corrupts it on a real bus. */
static void pnp_iochk_update(ISA3C515State *s)
{
    bool on = (s->pnp_iochk & 2) && !s->pnp_active;
    if (on) {
        memory_region_set_address(&s->pnp_chk_io, s->pnp_iobase);
    }
    memory_region_set_enabled(&s->pnp_chk_io, on);
}

static uint64_t pnp_chk_read(void *opaque, hwaddr addr, unsigned size)
{
    ISA3C515State *s = opaque;
    return (s->pnp_iochk & 1) ? 0x55 : 0xAA;
}

static void pnp_reset(ISA3C515State *s)
{
    s->pnp_state = PNP_WFK;
    s->pnp_keypos = 0;
    s->pnp_rdp = 0;
    memory_region_set_enabled(&s->pnp_rd_io, false);
    s->pnp_csn = 0;
    s->pnp_ldn = 0;
    s->pnp_active = 0;
    s->pnp_iobase = 0;
    s->pnp_irq = 0;
    s->pnp_irqtype = 2;
    s->pnp_dma = 4;
    s->pnp_iochk = 0;
    pnp_iochk_update(s);
    pnp_decode(s, false);
}

static void pnp_addr_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    ISA3C515State *s = opaque;
    uint8_t v = (uint8_t)val;
    if (s->pnp_state == PNP_WFK) {
        if (v == pnp_key_byte(s->pnp_keypos)) {
            if (++s->pnp_keypos == 32) {
                s->pnp_state = PNP_SLEEP;
                s->pnp_keypos = 0;
            }
        } else {
            s->pnp_keypos = (v == pnp_key_byte(0)) ? 1 : 0;
        }
        return;
    }
    s->pnp_addr = v;
}

static void pnp_write_data(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    ISA3C515State *s = opaque;
    uint8_t v = (uint8_t)val;
    if (s->pnp_state == PNP_WFK) {
        return;
    }
    switch (s->pnp_addr) {
    case 0x00:                                   /* Set RD_DATA port: Isolation state only */
        if (s->pnp_state == PNP_ISOL) {
            s->pnp_rdp = (uint16_t)((v << 2) | 3);
            memory_region_set_address(&s->pnp_rd_io, s->pnp_rdp);
            memory_region_set_enabled(&s->pnp_rd_io, true);
        }
        break;
    case 0x02:                                   /* Config Control */
        if (v & 0x01) {                          /* reset: logical devices to power-up (inactive) */
            s->pnp_active = 0;
            pnp_decode(s, false);
        }
        if (v & 0x04) {
            s->pnp_csn = 0;
        }
        if (v & 0x02) {                          /* -> Wait for Key */
            s->pnp_state = PNP_WFK;
            s->pnp_keypos = 0;
        }
        break;
    case 0x03:                                   /* Wake[CSN] */
        if (v == s->pnp_csn) {
            s->pnp_state = (v == 0) ? PNP_ISOL : PNP_CFG;
            s->pnp_bit = 0;
            s->pnp_half = 0;
            s->pnp_resptr = 0;
        } else {
            s->pnp_state = PNP_SLEEP;
        }
        break;
    case 0x06:                                   /* CSN: the isolated card leaves Isolation */
        if (s->pnp_state == PNP_ISOL) {
            s->pnp_csn = v;
            s->pnp_state = PNP_CFG;
        }
        break;
    default:
        if (s->pnp_state != PNP_CFG) {
            break;
        }
        switch (s->pnp_addr) {
        case 0x07: s->pnp_ldn = v; break;
        case 0x30:
            s->pnp_active = v & 1;
            if (s->pnp_active && s->pnp_iobase) {
                s->iobase = s->pnp_iobase;
                if (s->pnp_irq && s->pnp_irq < 16) {
                    s->irq = s->pnp_irq;
                    s->irq_line = isa_get_irq(ISA_DEVICE(s), s->irq);
                }
                if (s->pnp_dma < 8) {
                    s->dmachan = s->pnp_dma;
                }
            }
            pnp_decode(s, s->pnp_active && s->pnp_iobase);
            pnp_iochk_update(s);
            break;
        case 0x31: s->pnp_iochk = v & 3; pnp_iochk_update(s); break;
        case 0x60: s->pnp_iobase = (uint16_t)((s->pnp_iobase & 0x00FF) | (v << 8)); pnp_iochk_update(s); break;
        case 0x61: s->pnp_iobase = (uint16_t)((s->pnp_iobase & 0xFF00) | v); pnp_iochk_update(s); break;
        case 0x70: s->pnp_irq = v & 0x0F; break;
        case 0x71: s->pnp_irqtype = v; break;
        case 0x74: s->pnp_dma = v & 7; break;
        }
        break;
    }
}

static uint64_t pnp_read_data(void *opaque, hwaddr addr, unsigned size)
{
    ISA3C515State *s = opaque;
    if (s->pnp_state == PNP_ISOL && s->pnp_addr == 0x01) {
        uint8_t val = 0xFF;                      /* a 0 bit: the card doesn't drive the bus */
        if (s->pnp_bit < 72) {
            int bit = (s->pnp_serial[s->pnp_bit >> 3] >> (s->pnp_bit & 7)) & 1;
            if (bit) {
                val = s->pnp_half ? 0xAA : 0x55;
            }
            if (s->pnp_half) {
                s->pnp_bit++;
            }
            s->pnp_half ^= 1;
        }
        return val;
    }
    if (s->pnp_state != PNP_CFG) {
        return 0xFF;
    }
    switch (s->pnp_addr) {
    case 0x04: {                                 /* resource data: serial id, then the tags */
        uint16_t p = s->pnp_resptr++;
        if (p < 9) {
            return s->pnp_serial[p];
        }
        p -= 9;
        return (p < sizeof(pnp_res)) ? pnp_res[p] : 0;
    }
    case 0x05: return 1;                         /* status: resource data ready */
    case 0x06: return s->pnp_csn;
    case 0x07: return s->pnp_ldn;
    case 0x30: return s->pnp_active;
    case 0x31: return s->pnp_iochk;
    case 0x60: return s->pnp_iobase >> 8;
    case 0x61: return s->pnp_iobase & 0xFF;
    case 0x70: return s->pnp_irq;
    case 0x71: return s->pnp_irqtype;
    case 0x74: return s->pnp_dma;
    case 0x75: return 4;
    default:   return 0;
    }
}

static uint64_t pnp_addr_read(void *opaque, hwaddr addr, unsigned size)
{
    return 0xFF;                                 /* write-only (LPT2 status shares 0x279) */
}

static const MemoryRegionOps pnp_addr_ops = {
    .read = pnp_addr_read, .write = pnp_addr_write,
    .endianness = DEVICE_LITTLE_ENDIAN, .impl = { .min_access_size = 1, .max_access_size = 1 },
};
static const MemoryRegionOps pnp_write_ops = {
    .read = pnp_addr_read, .write = pnp_write_data,
    .endianness = DEVICE_LITTLE_ENDIAN, .impl = { .min_access_size = 1, .max_access_size = 1 },
};
static const MemoryRegionOps pnp_chk_ops = {
    .read = pnp_chk_read, .write = NULL,
    .endianness = DEVICE_LITTLE_ENDIAN, .impl = { .min_access_size = 1, .max_access_size = 1 },
};
static const MemoryRegionOps pnp_rd_ops = {
    .read = pnp_read_data, .write = NULL,
    .endianness = DEVICE_LITTLE_ENDIAN, .impl = { .min_access_size = 1, .max_access_size = 1 },
};

static void pnp_realize(ISA3C515State *s, ISADevice *isa)
{
    MemoryRegion *io = isa_address_space_io(isa);
    uint8_t lfsr = 0x6A;
    int i, j;
    /* serial identifier: vendor TCM (0x50 0x6D), product 5051, serial 0x78563412, then the checksum
     * over those 64 bits (the isolation LFSR, seed 0x6A) */
    static const uint8_t id[8] = { 0x50, 0x6D, 0x50, 0x51, 0x12, 0x34, 0x56, 0x78 };
    for (i = 0; i < 8; i++) {
        s->pnp_serial[i] = id[i];
        for (j = 0; j < 8; j++) {
            int bit = (id[i] >> j) & 1;
            lfsr = (uint8_t)(((((lfsr ^ (lfsr >> 1)) & 1) ^ bit) << 7) | (lfsr >> 1));
        }
    }
    s->pnp_serial[8] = lfsr;
    {   /* end-tag checksum: the resource bytes (after the serial id) sum to 0 mod 256 */
        uint8_t sum = 0;
        for (i = 0; i < (int)sizeof(pnp_res) - 1; i++) {
            sum += pnp_res[i];
        }
        pnp_res[sizeof(pnp_res) - 1] = (uint8_t)(0x100 - sum);
    }
    memory_region_init_io(&s->pnp_addr_io, OBJECT(s), &pnp_addr_ops, s, "3c515-pnp-addr", 1);
    memory_region_init_io(&s->pnp_write_io, OBJECT(s), &pnp_write_ops, s, "3c515-pnp-write", 1);
    memory_region_init_io(&s->pnp_rd_io, OBJECT(s), &pnp_rd_ops, s, "3c515-pnp-rd", 1);
    memory_region_add_subregion_overlap(io, PNP_ADDR_PORT, &s->pnp_addr_io, 1);
    memory_region_add_subregion_overlap(io, PNP_WRITE_PORT, &s->pnp_write_io, 1);
    memory_region_add_subregion_overlap(io, 0x203, &s->pnp_rd_io, 1);
    memory_region_init_io(&s->pnp_chk_io, OBJECT(s), &pnp_chk_ops, s, "3c515-pnp-iochk", 0x20);
    memory_region_add_subregion_overlap(io, 0x300, &s->pnp_chk_io, -1);
    pnp_reset(s);
}

static const MemoryRegionOps isa_3c515_eeprom_ops = {
    .read = isa_3c515_eeprom_read,
    .write = isa_3c515_eeprom_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 2 },
};

/* Bus-master DMA registers (base+0x400). DownListPtr (0x04)/UpListPtr (0x18) are 32-bit, but
 * a 286 driver writes each as two 16-bit halves -- handle both. The descriptor is processed
 * when the driver issues the StartDmaDown command (handled in el3_process_command). */
static void isa_3c515_dma_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    ISA3C515State *s = opaque;
    switch (addr) {
    case 0x04: /* DownListPtr (low half, or full 32-bit) */
        if (size == 4) {
            s->core.down_list_ptr = (uint32_t)val;
        } else {
            s->core.down_list_ptr = (s->core.down_list_ptr & 0xFFFF0000u) | (val & 0xFFFFu);
        }
        break;
    case 0x06: /* DownListPtr high half */
        s->core.down_list_ptr = (s->core.down_list_ptr & 0x0000FFFFu) | ((val & 0xFFFFu) << 16);
        break;
    case 0x18: /* UpListPtr (low half, or full 32-bit) */
        if (size == 4) {
            s->core.up_list_ptr = (uint32_t)val;
        } else {
            s->core.up_list_ptr = (s->core.up_list_ptr & 0xFFFF0000u) | (val & 0xFFFFu);
        }
        break;
    case 0x1A: /* UpListPtr high half */
        s->core.up_list_ptr = (s->core.up_list_ptr & 0x0000FFFFu) | ((val & 0xFFFFu) << 16);
        break;
    case 0x1C: /* RxCopybreak (MODEL-PRIVATE, not real 3C515 silicon): the guest driver writes its
                * autotuned copybreak threshold T here at CONFIGURE time so the emulator's size-routing
                * (el3_core_dma_rx_single: len <= T -> PIO FIFO, > T -> deep DMA ring) tracks the driver's
                * live decision -- closing the autotune loop. Ignored when a harness pinned a value
                * (rx_copybreak_lock) so a test can still force all-DMA (T=0) or a fixed T. Only 3cpd.exe
                * knows this offset; no real driver writes base+0x3C. */
        if (!s->core.rx_copybreak_lock) {
            s->core.rx_copybreak = (uint16_t)val;
        }
        break;
    default:
        break;
    }
}

static uint64_t isa_3c515_dma_read(void *opaque, hwaddr addr, unsigned size)
{
    ISA3C515State *s = opaque;
    switch (addr) {
    case 0x04: return (size == 4) ? s->core.down_list_ptr : (s->core.down_list_ptr & 0xFFFFu);
    case 0x06: return (s->core.down_list_ptr >> 16) & 0xFFFFu;
    case 0x18: return (size == 4) ? s->core.up_list_ptr : (s->core.up_list_ptr & 0xFFFFu);
    case 0x1A: return (s->core.up_list_ptr >> 16) & 0xFFFFu;
    case 0x1C: return s->core.rx_copybreak;  /* model-private RxCopybreak (read-back; see write) */
    default: return 0;
    }
}

static const MemoryRegionOps isa_3c515_dma_ops = {
    .read = isa_3c515_dma_read,
    .write = isa_3c515_dma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static const MemoryRegionOps isa_3c515_io_ops = {
    .read = isa_3c515_io_read,
    .write = isa_3c515_io_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        /* 32-bit FIFO access (offset 0): 386+ drivers stream the TX/RX FIFO with `rep outsd`/
         * `insd`. el3_core handles size==4; see the note in 3c509.c. */
        .max_access_size = 4,
    },
};

/* Forward declaration */
static void isa_3c515_irq_handler(EL3Core *c, bool level);

/* IRQ handler for ISA variant */
static void isa_3c515_irq_handler(EL3Core *c, bool level)
{
    ISA3C515State *s = container_of(c, ISA3C515State, core);
    qemu_set_irq(s->irq_line, level);
}

/* Variant operations for ISA */
static const EL3VariantOps isa_3c515_ops = {
    .irq_set = isa_3c515_irq_handler,
    .has_dma = true,  /* 3C515 has ISA bus master DMA */
};

/* Network client info */
static NetClientInfo net_3c515_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = el3_core_can_receive,
    .receive = el3_core_receive,
    .link_status_changed = el3_core_set_link_status,
};

/* Reset handler */
static void isa_3c515_reset(DeviceState *dev)
{
    ISA3C515State *s = ISA_3C515(dev);
    el3_core_reset(&s->core);
    if (s->autoselect) {
        s->core.windows[3][1] |= 0x0100;         /* InternalConfig bit 24: autoselect */
    }
    if (s->pnp_enable) {
        pnp_reset(s);
    }
}

/* VMState */
static const VMStateDescription vmstate_3c515 = {
    .name = "3c515",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT(core, ISA3C515State, 1, vmstate_el3_core, EL3Core),
        VMSTATE_UINT16(iobase, ISA3C515State),
        VMSTATE_UINT8(irq, ISA3C515State),
        VMSTATE_BOOL(pnp_enable, ISA3C515State),
        VMSTATE_UINT32(dma_rate_bps, ISA3C515State),
        VMSTATE_END_OF_LIST()
    }
};

/* Device realization */
static void isa_3c515_realize(DeviceState *dev, Error **errp)
{
    ISA3C515State *s = ISA_3C515(dev);
    ISADevice *isa = ISA_DEVICE(dev);
    
    /* Initialize core as 3C515 */
    el3_core_init(&s->core, MODEL_3C515, &isa_3c515_ops);

    /* realtiming wire rate: 100 Mbit -> 80 ns/byte, else 10BaseT -> 800 ns/byte */
    s->core.tx_ns_per_byte = (s->linkspeed >= 100) ? 80 : 800;
    /* InternalConfig xcvrSelect for that link (el3_core_reset re-derives it from tx_ns_per_byte; the
     * core's init-time reset ran before the link speed was known): 4 = 100BASE-TX, 0 = 10BASE-T */
    s->core.windows[3][1] = (uint16_t)((((s->linkspeed >= 100) ? 4 : 0) << 4) | (s->autoselect ? 0x0100 : 0));
    
    /* Setup MAC address */
    qemu_macaddr_default_if_unset(&s->core.conf.macaddr);
    
    /* Create network interface */
    s->core.nic = qemu_new_nic(&net_3c515_info, &s->core.conf,
                                TYPE_ISA_3C515, DEVICE(dev)->id,
                                &dev->mem_reentrancy_guard, &s->core);
    qemu_format_nic_info_str(qemu_get_queue(s->core.nic),
                              s->core.conf.macaddr.a);
    
    /* Initialize EEPROM with MAC address */
    el3_eeprom_init_3c509(&s->core);
    
    /* Setup I/O region */
    memory_region_init_io(&s->io, OBJECT(dev), &isa_3c515_io_ops,
                          s, TYPE_ISA_3C515, ISA_3C515_IO_SIZE);
    isa_register_ioport(isa, &s->io, s->iobase);

    /* EEPROM alias region at iobase+0x2000 (reuses the spare rom_io member). */
    memory_region_init_io(&s->rom_io, OBJECT(dev), &isa_3c515_eeprom_ops,
                          s, "3c515-eeprom", ISA_3C515_EEPROM_SIZE);
    isa_register_ioport(isa, &s->rom_io, s->iobase + ISA_3C515_EEPROM_ALIAS);

    /* Bus-master DMA register block at iobase+0x400; DMA targets guest system memory. */
    s->core.dma_as = &address_space_memory;
    memory_region_init_io(&s->dma_io, OBJECT(dev), &isa_3c515_dma_ops,
                          s, "3c515-dma", ISA_3C515_DMA_SIZE);
    isa_register_ioport(isa, &s->dma_io, s->iobase + ISA_3C515_DMA_BASE);

    /* Setup IRQ */
    s->irq_line = isa_get_irq(isa, s->irq);

    /* pnp=on: the card powers up inactive until ISA PnP isolation + activation (pnp=off: legacy mode,
     * always decoding at iobase) */
    if (s->pnp_enable) {
        pnp_realize(s, isa);
    }
    
    /* Configure DMA pacing */
    s->core.dma_rate_bps = s->dma_rate_bps;
}

/* Properties */
static const Property isa_3c515_properties[] = {
    DEFINE_PROP_UINT16("iobase", ISA3C515State, iobase, ISA_3C515_DEFAULT_IOBASE),
    DEFINE_PROP_UINT8("irq", ISA3C515State, irq, ISA_3C515_DEFAULT_IRQ),
    /* ISA DMA channel reported in the DCR (+0x2000): a real driver puts it in cascade mode so the
     * bus master can take the bus. The emulated bus master doesn't arbitrate through the 8237. */
    DEFINE_PROP_UINT8("dmachan", ISA3C515State, dmachan, 5),
    /* Test-only: the EEPROM's InternalConfig autoselect bit ("driver, pick the media"). */
    DEFINE_PROP_BOOL("autoselect", ISA3C515State, autoselect, false),
    DEFINE_PROP_BOOL("realtiming", ISA3C515State, core.realtiming, false),
    DEFINE_PROP_UINT16("linkspeed", ISA3C515State, linkspeed, 100),
    /* Test-only: emulator-side RX wire-rate frame generator. Value = synthetic frame size in bytes
     * (0 = off). Paced in virtual time at the link rate so RX throughput is measurable under -icount
     * without an external injector. See el3_rx_gen_timer_cb(). */
    DEFINE_PROP_UINT32("rxgen", ISA3C515State, core.rx_gen_size, 0),
    /* Test-only: rxgen L4 mode -> inject valid TCP data segments (L4-RX goodput) vs broadcast frames. */
    DEFINE_PROP_BOOL("rxgenl4", ISA3C515State, core.rx_gen_l4, false),
    /* Test-only: L4-RX sliding-window depth (frames in flight). 1 = stop-and-wait closed loop (default,
     * conservative); >1 lets the guest's ISR coalesce a batch and hides the per-frame ACK round-trip. */
    DEFINE_PROP_UINT32("rxgenwin", ISA3C515State, core.rx_gen_window, 1),
    /* Test-only: corrupt each L4-RX segment's TCP checksum -> validate that the RX verify path drops it. */
    DEFINE_PROP_BOOL("rxgenbad", ISA3C515State, core.rx_gen_bad, false),
    /* RX copybreak threshold (bytes): frames <= this stay on the PIO FIFO path; larger frames are
     * bus-mastered into the conventional/XMS ring (docs/10). 0 = DMA everything. The driver autotunes
     * its own T (copybreak_autotune); set this to match for the in-emulator copybreak test. */
    DEFINE_PROP_UINT16("rxcopybreak", ISA3C515State, core.rx_copybreak, 0),
    /* Pin rxcopybreak to the harness value and ignore the driver's live model-register writes (DMA-base
     * +0x1C). Default false = closed loop: the driver drives T from copybreak_autotune. Set on to force
     * a fixed T / all-DMA (rxcopybreak=0) for ring stress tests. */
    DEFINE_PROP_BOOL("rxcopybreaklock", ISA3C515State, core.rx_copybreak_lock, false),
    /* Test-only: in-emulator TCP-ACK reflector for measuring L4 TX goodput in virtual time. */
    DEFINE_PROP_BOOL("tcpreflect", ISA3C515State, core.tcp_reflect, false),
    /* Test-only: make the reflector ACK data segments (delayed) so a REAL flow-controlled sender
     * (mTCP SpdTest, our tcp_send) keeps its window open. Off for rawblast (which ignores ACKs). */
    DEFINE_PROP_BOOL("reflectack", ISA3C515State, core.reflect_ack, false),
    /* Test-only: in-emulator NVMe/TCP target on port 4420 (deterministic-under-icount storage path). */
    DEFINE_PROP_BOOL("nvmet", ISA3C515State, core.nvmet, false),
    /* ISA Plug and Play mode: the card is inactive at power-on until PnP isolation + activation (a
     * PnP-mode card on a board without a PnP BIOS). Off = legacy mode, always decoding at iobase. */
    DEFINE_PROP_BOOL("pnp", ISA3C515State, pnp_enable, false),
    DEFINE_PROP_UINT32("dma_rate", ISA3C515State, dma_rate_bps, ISA_3C515_DEFAULT_DMA_RATE),
    DEFINE_PROP_STRING("romfile", ISA3C515State, romfile),
    DEFINE_NIC_PROPERTIES(ISA3C515State, core.conf),
};

/* Class initialization */
static void isa_3c515_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    
    dc->realize = isa_3c515_realize;
    dc->legacy_reset = isa_3c515_reset;
    dc->desc = "3Com 3C515-TX ISA PnP Ethernet";
    dc->vmsd = &vmstate_3c515;
    device_class_set_props(dc, isa_3c515_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

/* Type info */
static const TypeInfo isa_3c515_info = {
    .name = TYPE_ISA_3C515,
    .parent = TYPE_ISA_DEVICE,
    .instance_size = sizeof(ISA3C515State),
    .class_init = isa_3c515_class_init,
};

/* Type registration */
static void isa_3c515_register_types(void)
{
    type_register_static(&isa_3c515_info);
}

type_init(isa_3c515_register_types)