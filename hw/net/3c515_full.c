/*
 * QEMU 3Com 3C515 Corkscrew emulation (stub)
 *
 * Copyright (c) 2024 QEMU contributors
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "hw/isa/isa.h"
#include "hw/net/el3_core.h"
#include "qemu/module.h"

/* TODO: Implement 3C515 ISA bus master device with DMA pacing */

static void el3_515_register_types(void)
{
    /* TODO: Register 3C515 type */
}

type_init(el3_515_register_types)
