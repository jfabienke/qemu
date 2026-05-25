#ifndef HW_NET_EL3_DESC_H
#define HW_NET_EL3_DESC_H
#include "qemu/osdep.h"
typedef struct QEMU_PACKED EL3BoomDesc { uint32_t next, status, addr, length; } EL3BoomDesc;
QEMU_BUILD_BUG_ON(sizeof(EL3BoomDesc)!=16);
#endif
