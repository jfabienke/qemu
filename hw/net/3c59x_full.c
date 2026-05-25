/*
 * QEMU 3Com 3C59x Vortex/Boomerang emulation (stub)
 *
 * Copyright (c) 2024 QEMU contributors
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "hw/pci/pci.h"
#include "hw/net/el3_core.h"
#include "qemu/module.h"

/* TODO: Implement 3C59x PCI device with Window 7 DMA rings */

static void el3_59x_register_types(void)
{
    /* TODO: Register 3C59x type */
}

type_init(el3_59x_register_types)
