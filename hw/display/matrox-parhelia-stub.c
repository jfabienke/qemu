/*
 * Matrox Parhelia stub PCI device for QEMU.
 *
 * A *stub*, not an emulator.  It exists at the Parhelia PCI vendor/device
 * IDs with plausible BARs so a guest video miniport's PCI probe
 * (e.g. NT 4.0's VideoPortGetAccessRanges) succeeds and the driver's
 * adapter-detection / BAR-mapping / init-dispatch path can be exercised
 * end-to-end.  It does NOT emulate the chip: MMIO reads return 0 and
 * writes are dropped.
 *
 * This is "Tier 1" of the stub scoping: unblocks "device exists" + BAR
 * mapping (NT video-miniport Layer-5 entry).  Tier 2 would gate specific
 * MMIO register reads to plausible values; Tier 3 (real CRTC/PLL/
 * framebuffer emulation) is real-silicon territory and intentionally not
 * attempted here.
 *
 * Instantiate with:  -device matrox-parhelia-stub
 *
 * SPDX-License-Identifier: MIT
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/pci_ids.h"
#include "qapi/error.h"
#include "qom/object.h"

#define TYPE_MATROX_PARHELIA_STUB "matrox-parhelia-stub"
OBJECT_DECLARE_SIMPLE_TYPE(MatroxParheliaStubState, MATROX_PARHELIA_STUB)

#define PARHELIA_PCI_VENDOR    0x102B   /* Matrox */
#define PARHELIA_PCI_DEVICE    0x0527   /* Parhelia 128/256 (rev C) */
#define PARHELIA_PCI_REVISION  0x01

/*
 * BAR layout the miniport expects, in this order:
 *   BAR0 = framebuffer aperture (sized by VRAM on real silicon)
 *   BAR1 = MMIO register window (16 KB on real Parhelia)
 * The aperture size is not validated by the Tier-1 path; 128 MiB is a
 * plausible real-card value and is allocated lazily by QEMU.
 */
#define PARHELIA_FB_BAR_SIZE   (128 * MiB)
#define PARHELIA_MMIO_BAR_SIZE (16 * KiB)

struct MatroxParheliaStubState {
    PCIDevice parent_obj;
    MemoryRegion fb;     /* BAR0 — framebuffer aperture */
    MemoryRegion mmio;   /* BAR1 — register window */
};

static uint64_t parhelia_stub_mmio_read(void *opaque, hwaddr addr,
                                        unsigned size)
{
    qemu_log_mask(LOG_UNIMP,
                  "matrox-parhelia-stub: MMIO read @0x%" HWADDR_PRIx
                  " size %u -> 0 (stub)\n", addr, size);
    return 0;
}

static void parhelia_stub_mmio_write(void *opaque, hwaddr addr,
                                     uint64_t val, unsigned size)
{
    qemu_log_mask(LOG_UNIMP,
                  "matrox-parhelia-stub: MMIO write @0x%" HWADDR_PRIx
                  " size %u val 0x%" PRIx64 " (dropped)\n",
                  addr, size, val);
}

static const MemoryRegionOps parhelia_stub_mmio_ops = {
    .read = parhelia_stub_mmio_read,
    .write = parhelia_stub_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 1, .max_access_size = 4 },
};

static void parhelia_stub_realize(PCIDevice *pdev, Error **errp)
{
    MatroxParheliaStubState *s = MATROX_PARHELIA_STUB(pdev);

    /* Legacy INTA: realistic shape, but not wired to anything (stub). */
    pci_config_set_interrupt_pin(pdev->config, 1);

    /* BAR0: framebuffer aperture — plain RAM so the guest can map it. */
    memory_region_init_ram(&s->fb, OBJECT(s), "parhelia-stub-fb",
                           PARHELIA_FB_BAR_SIZE, &error_fatal);
    pci_register_bar(pdev, 0,
                     PCI_BASE_ADDRESS_SPACE_MEMORY |
                     PCI_BASE_ADDRESS_MEM_PREFETCH, &s->fb);

    /* BAR1: MMIO register window — reads 0, writes dropped (Tier 1). */
    memory_region_init_io(&s->mmio, OBJECT(s), &parhelia_stub_mmio_ops, s,
                          "parhelia-stub-mmio", PARHELIA_MMIO_BAR_SIZE);
    pci_register_bar(pdev, 1, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->mmio);
}

static void parhelia_stub_class_init(ObjectClass *class, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(class);

    k->realize   = parhelia_stub_realize;
    k->vendor_id = PARHELIA_PCI_VENDOR;
    k->device_id = PARHELIA_PCI_DEVICE;
    k->revision  = PARHELIA_PCI_REVISION;
    /*
     * Real Parhelia is class 0x0300 (VGA).  We advertise DISPLAY_OTHER so
     * the stub doesn't contend with the guest's primary VGA for legacy
     * VGA I/O arbitration.  Detection is by vendor/device ID, not class,
     * so this does not affect the miniport's match.
     */
    k->class_id  = PCI_CLASS_DISPLAY_OTHER;
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
    dc->desc = "Matrox Parhelia stub (PCI presence + BARs only)";
}

static const TypeInfo parhelia_stub_types[] = {
    {
        .name          = TYPE_MATROX_PARHELIA_STUB,
        .parent        = TYPE_PCI_DEVICE,
        .instance_size = sizeof(MatroxParheliaStubState),
        .class_init    = parhelia_stub_class_init,
        .interfaces    = (const InterfaceInfo[]) {
            { INTERFACE_CONVENTIONAL_PCI_DEVICE },
            { },
        },
    }
};

DEFINE_TYPES(parhelia_stub_types)
