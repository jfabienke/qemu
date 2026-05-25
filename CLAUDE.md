# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

QEMU requires out-of-tree builds. Always create a separate build directory:

```bash
# Standard build
mkdir build && cd build
../configure
make -j$(nproc)

# Build with specific targets
../configure --target-list=x86_64-softmmu,aarch64-softmmu
make -j$(nproc)

# Clean build
make clean

# Full rebuild
rm -rf build && mkdir build && cd build
```

## Testing Commands

```bash
# Run unit tests
make check-unit

# Run all tests (comprehensive but slow)
make check

# Run specific test suite with meson
meson test --suite qemu:block

# Run functional Python tests
make check-functional

# Run a single test file
./tests/unit/test-crypto-hash

# Style checking before committing
./scripts/checkpatch.pl <patch-file>
```

## Code Architecture

QEMU is a machine emulator with modular architecture:

**Core Components:**
- `/target/` - CPU architecture emulation (x86, ARM, RISC-V, etc.). Each target has instruction decoding and translation.
- `/tcg/` - Tiny Code Generator that translates target instructions to host instructions
- `/hw/` - Hardware device emulation organized by category (net/, block/, display/, etc.) and architecture
- `/accel/` - Acceleration backends (TCG software emulation, KVM hardware virtualization, HVF, etc.)

**Key Subsystems:**
- Block layer (`/block/`) - Storage backends and disk image formats
- Migration (`/migration/`) - Live migration and savevm functionality  
- QOM (`/qom/`) - QEMU Object Model for device/object management
- Memory API (`/include/exec/memory.h`) - Guest memory management
- Main loop (`/util/main-loop.c`) - Event-driven architecture

**Device Model:**
- All devices inherit from QOM DeviceState
- Devices register memory regions, I/O ports, and IRQs
- Bus/device hierarchy (PCI, USB, etc.)

## Development Guidelines

**Style Requirements:**
- 4 spaces indentation (no tabs except Makefiles)
- 80 character line limit (up to 85 for readability)
- Check patches with: `./scripts/checkpatch.pl`
- Follow `/docs/devel/style.rst`

**Adding New Devices:**
1. Create device in appropriate `/hw/` subdirectory
2. Register with QOM type system
3. Add to relevant `meson.build`
4. Update MAINTAINERS if creating new subsystem

**Meson Build System:**
- All build configuration in `meson.build` files
- Use `config_host` for host configuration
- Use `config_target` for target-specific builds
- Dependencies managed through `meson.options`

## Common Development Tasks

```bash
# Debug build
../configure --enable-debug
make

# Build only specific target
../configure --target-list=x86_64-softmmu
make

# Run QEMU with debugging
./build/qemu-system-x86_64 -d cpu,exec -D /tmp/qemu.log

# Generate compile_commands.json for LSP
meson compile -C build compile_commands.json

# Run with gdb
gdb --args ./build/qemu-system-x86_64 [qemu options]
```

## Important Files and Locations

- `/configure` - Main configuration script
- `/meson.build` - Root build configuration
- `/hw/*/meson.build` - Device build configurations
- `/MAINTAINERS` - Subsystem ownership
- `/docs/devel/` - Developer documentation
- `/scripts/checkpatch.pl` - Style checker
- `/tests/` - All test suites