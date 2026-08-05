/* Auxiliary linker script — AUGMENTS the SDK's memmap_default.ld (it is not a
 * replacement: the INSERT command at the bottom makes ld merge this section
 * into the script it was given rather than override it).
 *
 * Why: the SDK's flash .text statement is
 *     *(EXCLUDE_FILE(*libgcc.a: *libc.a:*lib_a-mem*.o *libm.a:) .text*)
 * i.e. ALL of libgcc is deliberately kept out of flash and lands in the RAM
 * catch-all of .data — that is right for the hot helpers (dividers, _ashldi3,
 * _thumb1_case_*, memcpy/memset), but it also drags in the ARM EH unwinder,
 * which is ~4 KB of code that only ever runs on a C++ throw. This firmware
 * never throws (no try/catch/throw anywhere); the unwinder is linked in solely
 * because libstdc++'s __throw_* paths reference it. On RP2040 that is ~4 KB of
 * the ~5 KB of heap left after the framebuffer is allocated, so it matters.
 *
 * Placing the output section BEFORE .data is what does the work: an input
 * section goes to the first statement that matches it in script order, so these
 * three objects are claimed for flash before the .data catch-all sees them.
 *
 * RP2350 boards get the same treatment directly in rp2350-memmap.ld (measured
 * -4096 B of SRAM there), which is the project's own script; RP2040 boards use
 * the SDK script unmodified, hence this fragment.
 */
SECTIONS
{
    /* No `> FLASH`: the MEMORY block comes from a later script fragment
     * (pico_flash_region.ld), so naming the region here would warn about an
     * undeclared/redeclared region. Without it the section simply takes the
     * current location counter, which at this insertion point is the end of the
     * flash contents — exactly where it belongs, and still inside
     * __flash_binary_start..__flash_binary_end (verified in the .map). */
    .unwind_flash : {
        *libgcc.a:unwind-arm.o(.text* .rodata*)
        *libgcc.a:pr-support.o(.text* .rodata*)
        *libgcc.a:libunwind.o(.text* .rodata*)
    }
} INSERT BEFORE .data
