/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
 extern "C" {
#endif

//--------------------------------------------------------------------+
// Board Specific Configuration
//--------------------------------------------------------------------+

#if CFG_TUSB_MCU == OPT_MCU_RP2040
// change to 1 if using pico-pio-usb as host controller for raspberry rp2040
#define CFG_TUH_RPI_PIO_USB   0
#define BOARD_TUH_RHPORT      CFG_TUH_RPI_PIO_USB
#endif

// RHPort number used for host can be defined by board.mk, default to port 0
#ifndef BOARD_TUH_RHPORT
#define BOARD_TUH_RHPORT      0
#endif

// RHPort max operational speed can defined by board.mk
#ifndef BOARD_TUH_MAX_SPEED
#define BOARD_TUH_MAX_SPEED   OPT_MODE_DEFAULT_SPEED
#endif

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------

// defined by compiler flags for flexibility
#ifndef CFG_TUSB_MCU
#error CFG_TUSB_MCU must be defined
#endif

#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS           OPT_OS_NONE
#endif

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG        0
#endif

// Enable Host stack
#define CFG_TUH_ENABLED       1

// Default is max speed that hardware controller could support with on-chip PHY
#define CFG_TUH_MAX_SPEED     BOARD_TUH_MAX_SPEED

/* USB DMA on some MCUs can only access a specific SRAM region with restriction on alignment.
 * Tinyusb use follows macros to declare transferring memory so that they can be put
 * into those specific section.
 * e.g
 * - CFG_TUSB_MEM SECTION : __attribute__ (( section(".usb_ram") ))
 * - CFG_TUSB_MEM_ALIGN   : __attribute__ ((aligned(4)))
 */
#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN          __attribute__ ((aligned(4)))
#endif

//--------------------------------------------------------------------
// CONFIGURATION
//--------------------------------------------------------------------

// Size of buffer to hold descriptors and other data used for enumeration.
// 512 instead of TinyUSB's stock 1024: this is permanent .bss on every board and
// RP2040 has only ~12 KB of heap left after the framebuffer. The hard requirement
// is the configuration descriptor's wTotalLength — usbh.c aborts enumeration
// (TU_ASSERT, so the device just never mounts) if it does not fit. Keyboards/mice
// are <100 B, hubs ~25 B, and the largest thing we support is a DS4/DS5 pad with
// its audio interfaces (~350 B), so 512 keeps real headroom. Symptom if some
// exotic composite device ever exceeds it: the device is absent from OSD →
// "HID devices" while g_tusb_assert_count (ZiFi console line) ticks up — raise
// this back to 1024 in that case.
#define CFG_TUH_ENUMERATION_BUFSIZE 512

// A failed TU_ASSERT executes a bkpt instruction whenever a debug probe is
// attached, freezing the session on every RECOVERABLE assert (e.g. cdc_host's
// get_itf(TUSB_INDEX_INVALID) while a dongle re-enumerates). Route TinyUSB's
// breakpoint to a counting no-op instead — g_tusb_assert_count in main.cpp.
#define CFG_TUSB_DEBUG_BREAKPOINT picospec_tusb_assert_hook

#define CFG_TUH_XINPUT                 1 //
#define CFG_TUH_HUB                 1 // number of supported hubs
// CDC host: one serial adapter at a time (the ESP-01 bridge). The vendor serial
// sub-drivers let a CH340/CP2102/FTDI USB-UART dongle carry the ESP-01 over the USB
// host port (through the hub, alongside the keyboard) instead of GPIO. RP2350 only —
// the RP2040 boards (ZERO/MURM) don't run ZiFi and are SRAM-tight, so keep CDC off.
// IMPORTANT: gate on PICO_RP2350 (an SDK -D on the build), NOT CFG_TUSB_MCU — TinyUSB
// reports OPT_MCU_RP2040 for BOTH RP2040 and RP2350, so it can't distinguish them.
#if PICO_RP2350
#define CFG_TUH_CDC                 1
// Non-standard USB-serial chips. CH340C = CH34x (the documented dongle); CP210x/FTDI
// come free and cover other common adapters. Stock CDC-ACM is always on.
#define CFG_TUH_CDC_CH34X           1
#define CFG_TUH_CDC_CP210X          1
#define CFG_TUH_CDC_FTDI            1
// Per-interface FIFOs. NOTE: TinyUSB's cdc_host sizes BOTH the rx and tx FIFOs from
// CFG_TUH_CDC_TX_BUFSIZE (rx_ff_buf[CFG_TUH_CDC_TX_BUFSIZE]). This FIFO is the only
// cushion for bytes the ESP keeps sending while tuh_task() is stalled (SD write /
// mbedTLS work) — unlike the UART path there's no IRQ-context drain upstream of it:
// once it fills, the IN endpoint stops being re-armed and the CH340's ~256 B
// internals overflow SILENTLY (tu_edpt_stream_read_xfer requires ≥64 B of FIFO room
// to re-arm). The FIFO only cushions tuh_task() STALLS — it cannot fix a wire-rate
// deficit, but with the vendored TinyUSB 0.21 HCD (external/tinyusb, ~0.9 MB/s
// bulk drain) there is none: the full menu rate 921600 (~92 KB/s) fits with
// headroom (ZIFI_CDC_MAX_BAUD in ZiFi.cpp; under the old <=0.20 driver's ~64 KB/s
// drain the ceiling was 460800). Sized for 921600 (applied via AT+UART_CUR +
// tuh_cdc_set_baudrate): 8 KB tolerates ~89 ms of stall — enough for the TLS
// handshake compute gaps and (with Ftp.cpp's 4 KB write slicing) SD writes; 4 KB
// (~44 ms) still lost bytes at 460800 in hw testing. MURM1_P2 (the board-define
// fallback) is SRAM-tight — Profi leaves ~10 KB heap and this BSS is spent even
// with ZiFi off — so it keeps 2 KB (~22 ms): practical ceiling there is 230400.
#if defined(MURM2) || defined(PICO_PC) || defined(PICO_DV) || defined(ZERO2)
#define CFG_TUH_CDC_RX_BUFSIZE      8192
#define CFG_TUH_CDC_TX_BUFSIZE      8192
#else
#define CFG_TUH_CDC_RX_BUFSIZE      2048
#define CFG_TUH_CDC_TX_BUFSIZE      2048
#endif
// CFG_TUH_CDC_RX_EPSIZE stays at the default 64 (one packet per armed transfer).
// 512 was tried to let bursts chain through the double-buffered EPX without
// tuh_task — it did move data, but the CH340's constant SHORT packets through the
// ping-pong buffers delivered CORRUPTED bytes (hw 2026-07-06: MRF page rendered
// as garbage, rx counters clean). Multi-packet RX is only safe for full-packet
// sources (MSC); serial dongles must stay single-packet. Burst survival is
// handled by cdcPump()'s three call sites instead (see ZiFi.cpp).
#else
#define CFG_TUH_CDC                 0
#endif
// HID interface slots. Composite devices (kbd + consumer keys, pads with extra
// interfaces) eat more than one each, so this cannot be cut to the device count —
// but every slot costs ~200 B of permanent .bss across four arrays
// (_hidh_epbuf 72 + hid_snap 44 + hid_info 22 + _hidh_itf 14), which is why it is
// 6 and not 8: keyboard (1-2) + mouse (1) + two pads (1-2 each) still fits, and
// RP2040 needs the KB. A 7th interface is simply not serviced (tuh_hid_mount
// asserts and that interface is ignored) — bump this if a real device needs it.
#define CFG_TUH_HID                 6
// USB mass-storage host (flash sticks in the file manager, FatFs volume "USB:").
// RP2350 only — same SRAM reasoning as CDC above; RP2040 boards stay MSC-free.
#if PICO_RP2350
#define CFG_TUH_MSC                 1
#else
#define CFG_TUH_MSC                 0
#endif
#define CFG_TUH_VENDOR              0

// max device support (excluding hub device)
#if PICO_RP2350
#define CFG_TUH_DEVICE_MAX          6 // hub + keyboard + mouse + 2 gamepads + MSC stick
#else
#define CFG_TUH_DEVICE_MAX          5 // hub + keyboard + mouse + 2 gamepads
#endif

//------------- HID -------------//
// EPIN must stay 64 — the IN transfer is armed for the device's endpoint max
// packet size, which is 64 for full-speed interrupt endpoints.
#define CFG_TUH_HID_EPIN_BUFSIZE    64
// EPOUT is only ever touched by tuh_hid_send_report() (rumble / LED output
// reports over the interrupt OUT endpoint), which this firmware never calls:
// keyboard LEDs go through the control pipe and XInput pads use xinput_host's own
// epout_buf. Keep a token 8 bytes instead of 64 per slot. If HID output reports
// are ever added, raise this to the largest report used — send_report() refuses
// anything longer (len > CFG_TUH_HID_EPOUT_BUFSIZE returns false).
#define CFG_TUH_HID_EPOUT_BUFSIZE   8

//------------- CDC -------------//

// Set Line Control state on enumeration/mounted:
// DTR ( bit 0), RTS (bit 1)
#define CFG_TUH_CDC_LINE_CONTROL_ON_ENUM    0x03

// Set Line Coding on enumeration/mounted, value for cdc_line_coding_t
// bit rate = 115200, 1 stop bit, no parity, 8 bit data width
#define CFG_TUH_CDC_LINE_CODING_ON_ENUM   { 115200, CDC_LINE_CONDING_STOP_BITS_1, CDC_LINE_CODING_PARITY_NONE, 8 }


#ifdef __cplusplus
 }
#endif

#endif /* _TUSB_CONFIG_H_ */
