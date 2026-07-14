#!/bin/bash
# Pre-release memory sanity check — manual run, no flashing required.
#
# For each board, estimates free SRAM heap right after ESPectrum::setup()
# finishes and ESPectrum::loop() starts, WITHOUT running any code:
#
#   free_at_static   = TOTAL_SRAM - (static .data/.bss/.ram_* footprint, read
#                       from the built .elf via `size -A`)
#   projected_free    = free_at_static - FIXED_BOOT_MALLOCS
#
# FIXED_BOOT_MALLOCS are the allocations that happen unconditionally during
# setup() with the *default* NVS config (see BOARD BUDGETS below for the
# line-item sources). This is a deterministic estimate, not a measurement:
# it does NOT include user-enabled optional features (DivMMC, IDE, MB-02+,
# General Sound, MIDI wavetable, ZiFi, Gigascreen, ULA+ 16-col...) since
# those are off by default and depend on what's saved in storage.nvs. If
# you've enabled any of those on the target board, subtract their cost
# manually — see OPTIONAL FEATURE COSTS at the bottom of the output.
#
# Cross-checked against the on-hardware figures in CLAUDE.md (ZERO: "~148KB
# free heap at boot, ~5KB after framebuffer") — this script's model lands in
# the same ballpark (single-digit-KB on ZERO) but is not exact; treat WARN/FAIL
# as "go re-verify on real hardware", not as ground truth.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# --- Pico SDK toolchain bootstrap (same as draft-release.sh) ---
if ! command -v arm-none-eabi-size >/dev/null 2>&1; then
    TC_VER=$(grep -oP 'set\(toolchainVersion \K[^)]+' CMakeLists.txt)
    TC_DIR="$HOME/.pico-sdk/toolchain/$TC_VER"
    if [ -d "$TC_DIR/bin" ]; then
        export PATH="$TC_DIR/bin:$PATH"
    else
        LATEST_TC=$(ls -d "$HOME/.pico-sdk/toolchain"/*/bin 2>/dev/null | sort -V | tail -1)
        [ -n "$LATEST_TC" ] && export PATH="$LATEST_TC:$PATH"
    fi
fi
if ! command -v arm-none-eabi-size >/dev/null 2>&1; then
    echo "Error: arm-none-eabi-size not found (no Pico SDK toolchain on PATH)"
    exit 1
fi

VER=$(grep -oP 'set \(PORT_VERSION "\K[0-9.]+' CMakeLists.txt)
RAM_BASE=536870912  # 0x20000000

# --- Board table: name | elf glob | total SRAM bytes | MCU family ---
# RP2040 total SRAM: 264KB (270336B). RP2350/RP2350B: 520KB (532480B) — same
# figure for the A and B package, they differ in GPIO count/pins, not SRAM.
BOARDS=(
    "ZERO|build-ZERO/bin/MinSizeRel/z0p1-spec-VGA-HDMI-*.elf|270336|RP2040"
    "MURM_P1|build-MURM_P1/bin/MinSizeRel/m1p1-spec-VGA-HDMI-*.elf|270336|RP2040"
    "MURM_P2|build-MURM_P2/bin/MinSizeRel/m1p2-spec-VGA-HDMI-*.elf|532480|RP2350"
    "MURM2_P2|build-MURM2_P2/bin/MinSizeRel/m2p2-spec-VGA-HDMI-*.elf|532480|RP2350"
    "PICO_PC|build-PICO_PC/bin/MinSizeRel/PCp2-spec-VGA-HDMI-*.elf|532480|RP2350"
    "PICO_DV|build-PICO_DV/bin/MinSizeRel/DVp2-spec-VGA-HDMI-*.elf|532480|RP2350"
    "ZERO2|build-ZERO2/bin/MinSizeRel/z0p2-spec-VGA-HDMI-*.elf|532480|RP2350"
)

# --- Fixed boot-time heap mallocs, default config, per MCU family ---
# RP2040 (ZERO / MURM_P1):
#   MemESP pages 1,2,3: 3x `new unsigned char[16384]`  (ESPectrum.cpp ~798-841)  = 49152
#   Framebuffer, default 320x240 landscape (Video.cpp / DMABufferDescriptor.h)  = 78084
#   render_core (core1, concurrent): vga_alloc_buffers + lines_pattern_data     ~ 8500
FIXED_BOOT_RP2040=$((49152 + 78084 + 8500))

# RP2350 (all others):
#   MemESP pages 1,2,3 are static (0 heap) on RP2350, unlike RP2040
#   WD1793 track buffer: `malloc(DISK_TRACK_BUF_SZ)`, unconditional            = 12800
#   Framebuffer, default 320x240 landscape                                    = 78084
#   TurboSubsys (AY chip1): default ON on RP2350 (turbosound=3)               ~ 1300
#   render_core (core1, concurrent): RP2350-HDMI path is static/0-heap, but
#   VGA-selected boards still allocate ~8-9KB via vga_alloc_buffers           ~ 8500
FIXED_BOOT_RP2350=$((12800 + 78084 + 1300 + 8500))

# --- Thresholds (tune as needed) ---
WARN_BELOW=16384
FAIL_BELOW=4096

sram_static_bytes() {
    arm-none-eabi-size -A "$1" 2>/dev/null | awk -v ram="$RAM_BASE" '
        NR>2 && $1 !~ /^\.debug/ && $1 !~ /^\.ARM\.attr/ && $1 !~ /^\.comment/ && NF>=3 {
            sz=$2+0; ad=$3+0
            if (ad >= ram) { total += sz }
        }
        END { printf "%d", total+0 }
    '
}

printf "pico-spec pre-release memory check — v%s\n\n" "$VER"
printf "%-10s %-8s %10s %10s %10s %10s  %s\n" \
    "BOARD" "MCU" "STATIC" "FREE@STAT" "FIXEDBOOT" "PROJECTED" "STATUS"

EXIT_CODE=0
MISSING=()

for entry in "${BOARDS[@]}"; do
    IFS='|' read -r NAME GLOB TOTAL_SRAM MCU <<< "$entry"

    ELF=$(ls -t $GLOB 2>/dev/null | head -1)
    if [ -z "$ELF" ]; then
        printf "%-10s %-8s %10s %10s %10s %10s  %s\n" "$NAME" "$MCU" "-" "-" "-" "-" "NOT BUILT"
        MISSING+=("$NAME")
        continue
    fi

    STATIC=$(sram_static_bytes "$ELF")
    FREE_AT_STATIC=$((TOTAL_SRAM - STATIC))

    if [ "$MCU" = "RP2040" ]; then
        FIXED_BOOT=$FIXED_BOOT_RP2040
    else
        FIXED_BOOT=$FIXED_BOOT_RP2350
    fi

    PROJECTED=$((FREE_AT_STATIC - FIXED_BOOT))

    STATUS="OK"
    if [ "$PROJECTED" -lt "$FAIL_BELOW" ]; then
        STATUS="FAIL"
        EXIT_CODE=1
    elif [ "$PROJECTED" -lt "$WARN_BELOW" ]; then
        STATUS="WARN"
    fi

    BASENAME=$(basename "$ELF")
    if [[ "$BASENAME" != *"$VER"* ]]; then
        STATUS="$STATUS (stale: $BASENAME)"
    fi

    printf "%-10s %-8s %10d %10d %10d %10d  %s\n" \
        "$NAME" "$MCU" "$STATIC" "$FREE_AT_STATIC" "$FIXED_BOOT" "$PROJECTED" "$STATUS"
done

echo ""
echo "STATIC     = .data+.bss+.ram_*+stack sections from the built .elf (arm-none-eabi-size -A)"
echo "FREE@STAT  = TOTAL_SRAM - STATIC (heap available before any malloc() runs)"
echo "FIXEDBOOT  = deterministic mallocs during setup() with default config (see script header)"
echo "PROJECTED  = estimated free heap once ESPectrum::loop() starts"
echo ""
echo "NOT included above (off by default, subtract manually if enabled on target board):"
echo "  DivMMC ~1.2-25KB | IDE ~2.3KB+ | MB-02+ ~21KB | General Sound 32KB (PSRAM, ~0 SRAM cost)"
echo "  MIDI GM.DLS (PSRAM/flash, ~0 SRAM cost) | Gigascreen ~52KB (PSRAM-tiered on RP2350)"
echo "  ALF cart 16KB | remembered disk/tape mounts ~2KB each"

if [ ${#MISSING[@]} -gt 0 ]; then
    echo ""
    echo "Not built (run ./build_all.sh ${MISSING[*]}): ${MISSING[*]}"
fi

exit $EXIT_CODE
