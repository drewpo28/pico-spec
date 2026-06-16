# pico-spec Project Memory

## SAA1099 Emulation Key Findings

### Current implementation
Based on **stripwax/SAASound** (Dave Hooper) — https://github.com/stripwax/SAASound
Verified against real SAA1099P hardware.
Flattened from CSAAFreq, CSAANoise, CSAAEnv, CSAAAmp, CSAADevice into a single class.

### Mixing model (stripwax CSAAAmp)

- `intermediate` per channel: 0 (silent), 1 or 2 based on tone/noise mix_mode
  - mix_mode=0: `intermediate=0` (both off — DC/buzz)
  - mix_mode=1 (tone only): `intermediate = tone_level * 2` (0 or 2)
  - mix_mode=2 (noise only): `intermediate = noise_level * 2` (0 or 2)
  - mix_mode=3 (tone+noise): `intermediate = tone_level * (2 - noise_level)`
- **Non-envelope channels (ch0/1/3/4)**: `output += amp * intermediate * 16`
  - tone=1 → intermediate=2 → loud (no inversion)
- **Envelope channels (ch2/ch5)**: `output += pdm_x4[amp/2][env_level] * (2 - intermediate)`
  - Uses PDM effective amplitude table (models analog behavior of real chip)
  - `(2 - intermediate)`: when intermediate=2 (tone=1) → factor=0 → silence; when 0 → factor=2 → full envelope
  - **DC/buzz mode**: mix_mode=0 → intermediate=0 → `pdm_x4[amp/2][env_level] * 2` — pure envelope output
  - Right-channel: `env_right_level = 15 - env_level` when invert_right set
- **Envelope applies ONLY to ch2 (env0) and ch5 (env1)**

### Envelope clock

- **Internal clock**: triggered on ch1 half-cycle (env0) / ch4 half-cycle (env1)
- **External clock**: `selectRegister(0x18/0x19)` triggers one tick — only when reg 24/25 is addressed
- Envelope parameter changes are buffered and applied at natural phase boundaries (CSAAEnv::Tick logic)

### Tone frequency

- Period = `max(511 - freq_offset, 1)` — confirmed by FPGA and SAA1099Tracker
- Counter: `counter += (1 << octave)` per sample; flip level when `counter >= period`
- Philips quirk: offset buffered and deferred when octave written in same cycle

### Envelope

- 8 shapes (index 0-7): zero, max, single/repetitive decay, single/repetitive triangle, single/repetitive attack
- Two resolutions: 4-bit (step by 1, 16 steps) and 3-bit (step by 2, 8 effective steps)
- Phase-based with 1 or 2 phases per shape; looping flag per shape

### Noise

- 18-bit Galois LFSR: `rand = (rand >> 1) ^ 0x20400` when bit0=1, else `rand >>= 1`
- Noise source 3: triggered by ch0 half-cycle (noise[0]) / ch3 half-cycle (noise[1])

### Key reference implementations

- **stripwax/SAASound**: current implementation basis, verified against real SAA1099P
- SCPlayer (Deltafire): uses stripwax as submodule
- MAME: applies envelope to all channels (differs from stripwax)
- FPGA sorgelig/SAMCoupe_MIST: applies envelope to all channels
- SAA1099Tracker (mborik): applies envelope to all channels
- UnrealSpeccy: different mixing model — `vol_table * env * 2`, subtractive noise — NOT used

### Build

- `cmake --build build` from project root
- SAASound.cpp compiled with `-O3 -ffast-math -funroll-loops`

## RP2040 Memory Constraints (ZERO target)

### Key facts

- RP2040: 256KB SRAM total, no PSRAM
- BSS/data ~109KB → free heap ~148KB at boot
- Framebuffer: 240×320 ≈ 77KB (allocated in VIDEO::Init)
- After framebuffer: only ~5KB free heap remains
- `MEM_REMAIN` = 6×16KB = 96KB — reserve for framebuffer

### Rules for RP2040 development

- **ALWAYS test new features on ZERO** — any static array or large struct can break it
- **No heap-heavy operations after VIDEO::Init** — only ~5KB free
  - `vector<string>` parsing of storage.nvs caused OOM in `Config::loadDiskMounts()` (fixed: parse line-by-line)
- **Guard large RAM features with `#if !PICO_RP2040`**:
  - ULA+ (`AluBytesUlaPlus[16][256]` = 16KB) — disabled for RP2040
  - Gigascreen — already guarded
- **New static buffers**: consider `#if !PICO_RP2040` or make conditional
- **`Config::load()` is safe** (runs at 148KB free), `Config::loadDiskMounts()` runs at ~5KB free

## GPIO Map (all boards)

### Classification

- **FIXED** — hardwired on PCB, cannot change (display, SD card, onboard PSRAM, LED)
- **REASSIGNABLE** — currently used but can be remapped/disabled in software (NESPAD, keyboard, MIDI, audio, WAV-input)
- **FREE** — not used by any peripheral

### MIDI UART TX constraint (RP2350)

On RP2350, UART TX available via two funcsel:
- funcsel 2 (`GPIO_FUNC_UART`): GPIO 0→UART0, 4→UART1, 8→UART0, 12→UART1, 16→UART0, 20→UART1, 24→UART0, 28→UART1
- funcsel 11 (`GPIO_FUNC_UART_AUX`): GPIO 2→UART0, 6→UART1, 10→UART0, 14→UART1, 18→UART0, 22→UART1, 26→UART1, 30→UART0
- Odd GPIO = RX only, cannot be TX
- Code auto-selects: `(pin/4)%2 → 0=UART0, 1=UART1`, funcsel via `(gpio & 0x2) ? UART_AUX : UART`
- RP2040: no MIDI support (`PICO_RP2040=1`)

### MURM2 (RP2350A, GPIO 0-29) — FIXED=15, REASSIGNABLE=9, FREE=5

| GPIO | Function | Cat | Notes |
|------|----------|-----|-------|
| 0 | — | FREE | |
| 1 | — | FREE | |
| 2 | KBD_CLOCK | REASSIGN | PS/2 keyboard |
| 3 | KBD_DATA | REASSIGN | PS/2 keyboard |
| 4 | SD MISO | FIXED | SPI0 PCB |
| 5 | SD CS | FIXED | SPI0 PCB |
| 6 | SD SCK | FIXED | SPI0 PCB |
| 7 | SD MOSI | FIXED | SPI0 PCB |
| 8 | BUTTER_PSRAM | FIXED | RP2350 onboard PSRAM XIP CS1 |
| 9 | Audio DATA/BEEPER/LATCH_595 | REASSIGN | Triple-alias (I2S/PWM/595) |
| 10 | Audio BCK/PWM0/CLK_595 | REASSIGN | Triple-alias |
| 11 | Audio LCK/PWM1/DATA_595 | REASSIGN | Triple-alias |
| 12-19 | VGA/HDMI (8 pins) | FIXED | Display base=12; also PSRAM SPI on 18-19 |
| 20 | PSRAM_MOSI / NES_CLK | FIXED | PSRAM priority; **NESPAD conflict!** |
| 21 | PSRAM_MISO / NES_LAT | FIXED | PSRAM priority; **NESPAD conflict!** |
| 22 | MIDI_TX / LOAD_WAV_PIO | REASSIGN | Mutually exclusive. UART1 TX funcsel 11 — works |
| 23 | — | FREE | |
| 24 | — | FREE | |
| 25 | LED | FIXED | |
| 26 | NES_DATA | REASSIGN | NESPAD joy1 |
| 27 | NES_DATA+1 (implicit) | REASSIGN | NESPAD joy2 (PIO reads DATA+1) |
| 28 | — | FREE | |
| 29 | CLK_AY_PIN2 | REASSIGN | AY clock out |

### PICO_PC (RP2350A, GPIO 0-29) — FIXED=13, REASSIGNABLE=10, FREE=6

| GPIO | Function | Cat | Notes |
|------|----------|-----|-------|
| 0 | KBD_CLOCK | REASSIGN | |
| 1 | KBD_DATA | REASSIGN | |
| 2 | — | FREE | QWST1 connector |
| 3 | — | FREE | |
| 4 | SD MISO | FIXED | SPI0 PCB |
| 5 | NES_CLK / LOAD_WAV_PIO | REASSIGN | Shared |
| 6 | SD SCK | FIXED | SPI0 PCB |
| 7 | SD MOSI | FIXED | SPI0 PCB |
| 8 | BUTTER_PSRAM | FIXED | RP2350 onboard PSRAM XIP CS1 |
| 9 | NES_LAT | REASSIGN | |
| 10 | — | FREE | |
| 11 | — | FREE | |
| 12-19 | VGA/HDMI (8 pins) | FIXED | Display base=12 |
| 20 | NES_DATA | REASSIGN | UXT1-3 |
| 21 | NES_DATA2 | REASSIGN | UXT1-4 |
| 22 | SD CS | FIXED | SPI0 PCB |
| 23 | — | FREE | |
| 24 | — | FREE | |
| 25 | LED | FIXED | |
| 26 | BEEPER/LATCH_595/MIDI_TX | REASSIGN | UART1 TX funcsel 11 — works |
| 27 | PWM0/CLK_595 | REASSIGN | Audio PWM right |
| 28 | PWM1/DATA_595 | REASSIGN | Audio PWM left |
| 29 | CLK_AY_PIN2 | REASSIGN | AY clock out |

### PICO_DV (RP2350, GPIO 0-29 + 47) — FIXED=14, REASSIGNABLE=8, FREE=9

| GPIO | Function | Cat | Notes |
|------|----------|-----|-------|
| 0 | ZiFi UART0 TX | REASSIGN | freed from KBD; UART0 TX funcsel 2 |
| 1 | ZiFi UART0 RX | REASSIGN | freed from KBD; UART0 RX funcsel 2 |
| 2 | — | FREE | |
| 3 | — | FREE | |
| 4 | — | FREE | |
| 5 | SD SCK | FIXED | SPI1 PCB |
| 6-13 | VGA/HDMI (8 pins) | FIXED | Display base=6; NES_CLK=8, NES_LAT=9 conflict! |
| 14 | KBD_CLOCK | REASSIGN | moved from GP0 for ZiFi |
| 15 | KBD_DATA | REASSIGN | moved from GP1 for ZiFi |
| 16 | — | FREE | |
| 17 | — | FREE | |
| 18 | SD MOSI | FIXED | SPI1 PCB |
| 19 | SD MISO | FIXED | SPI1 PCB |
| 20 | LOAD_WAV_PIO / NES_DATA | REASSIGN | No USE_NESPAD |
| 21 | MIDI_TX / NES_DATA2 | REASSIGN | **BUG: GPIO 21 odd = UART RX, not TX! MIDI broken** |
| 22 | SD CS | FIXED | SPI1 PCB |
| 23 | — | FREE | |
| 24 | — | FREE | |
| 25 | LED | FIXED | |
| 26 | Audio DATA/PWM0/LATCH_595 | REASSIGN | |
| 27 | Audio BCK/PWM1/CLK_595 | REASSIGN | |
| 28 | Audio LCK/BEEPER/DATA_595 | REASSIGN | |
| 29 | CLK_AY_PIN2 | REASSIGN | |
| 47 | BUTTER_PSRAM | FIXED | RP2350B onboard PSRAM |

### ZERO (RP2040, GPIO 0-28) — FIXED=16, REASSIGNABLE=8, FREE=5

| GPIO | Function | Cat | Notes |
|------|----------|-----|-------|
| 0 | KBD_CLOCK | REASSIGN | |
| 1 | KBD_DATA | REASSIGN | |
| 2-5 | PSRAM SPI (CS/SCK/MOSI/MISO) | FIXED | Onboard PSRAM |
| 6 | — | FREE | |
| 7 | CLK_AY_PIN2 / NES_CLK | REASSIGN | AY + NESPAD share (conflict) |
| 8 | NES_LAT | REASSIGN | |
| 9 | NES_DATA (DATA2=9) | REASSIGN | Single joystick |
| 10 | Audio DATA/PWM0/LATCH_595 | REASSIGN | |
| 11 | Audio BCK/PWM1/CLK_595 | REASSIGN | |
| 12 | Audio LCK/BEEPER/DATA_595 | REASSIGN | |
| 13 | — | FREE | |
| 14 | — | FREE | |
| 15 | — | FREE | |
| 16 | — | FREE | |
| 17 | LOAD_WAV_PIO | REASSIGN | No MIDI (RP2040) |
| 18-21 | SD SPI (SCK/MOSI/MISO/CS) | FIXED | SPI0 PCB |
| 22-28 | VGA/HDMI (8 pins from 22) | FIXED | Includes LED=25, SMPS=23, VBUS=24 |

### ZERO2 (RP2350B, GPIO 0-47) — FIXED=14, REASSIGNABLE=12, FREE=22

| GPIO | Function | Cat | Notes |
|------|----------|-----|-------|
| 0-1 | — | FREE | PSRAM disabled |
| 2 | PCM5122_I2C_SDA | REASSIGN | DAC control (if attached) |
| 3 | PCM5122_I2C_SCL | REASSIGN | |
| 4-6 | — | FREE | NESPAD disabled |
| 7 | CLK_AY_PIN2 | REASSIGN | AY clock out |
| 8-9 | — | FREE | |
| 10 | Audio DATA/PWM0/LATCH_595 | REASSIGN | |
| 11 | Audio BCK/PWM1/CLK_595 | REASSIGN | |
| 12 | Audio LCK/BEEPER/DATA_595 | REASSIGN | |
| 13 | — | FREE | |
| 14 | KBD_CLOCK | REASSIGN | Moved from 2/3 for PCM5122 I2C |
| 15 | KBD_DATA | REASSIGN | |
| 16 | — | FREE | |
| 17 | LOAD_WAV_PIO | REASSIGN | WAV loader |
| 18 | PCM5122_I2S_BCK | REASSIGN | DAC bit clock |
| 19 | PCM5122_I2S_LCK | REASSIGN | DAC LR clock |
| 20 | — | FREE | Good MIDI candidate (UART1 TX) |
| 21 | PCM5122_I2S_DATA | REASSIGN | DAC data |
| 22 | MIDI_TX | REASSIGN | UART1 TX funcsel 11 — works |
| 23-29 | — | FREE | |
| 30 | SD SCK | FIXED | SPI1 Waveshare board |
| 31 | SD MOSI | FIXED | |
| 32-39 | VGA/HDMI (8 pins) | FIXED | Display base=32 |
| 40 | SD MISO | FIXED | |
| 41-42 | — | FREE | |
| 43 | SD CS | FIXED | |
| 44-46 | — | FREE | |
| 47 | BUTTER_PSRAM | FIXED | RP2350B onboard PSRAM |

### MURM (RP2040, GPIO 0-28) — FIXED=18, REASSIGNABLE=10, FREE=0

| GPIO | Function | Cat | Notes |
|------|----------|-----|-------|
| 0 | KBD_CLOCK / DBG_UART0_TX | REASSIGN | DBG_UART=ON → UART0_TX (J6 header), KBD moves to GP16/17 |
| 1 | KBD_DATA / DBG_UART0_RX | REASSIGN | DBG_UART=ON → UART0_RX |
| 2-5 | SD SPI (SCK/MOSI/MISO/CS) | FIXED | SPI0 PCB |
| 6-13 | VGA/HDMI (8 pins) | FIXED | Display base=6 |
| 14 | NES_CLK | REASSIGN | DBG_UART=ON → NESPAD disabled |
| 15 | NES_LAT | REASSIGN | DBG_UART=ON → NESPAD disabled |
| 16 | NES_DATA / KBD_CLOCK (DBG_UART=ON) | REASSIGN | when DBG_UART=ON: KBD moves here |
| 17 | NES_DATA+1 / KBD_DATA (DBG_UART=ON) | REASSIGN | when DBG_UART=ON: KBD moves here |
| 18-21 | PSRAM SPI (CS/SCK/MOSI/MISO) | FIXED | Onboard PSRAM; BUTTER=19 |
| 22 | MIDI_TX / LOAD_WAV_PIO | REASSIGN | RP2040: WAV only. RP2350(MURM_P2): UART1 TX works |
| 23 | — | FIXED(RP2040) | SMPS power on standard Pico |
| 24 | — | FIXED(RP2040) | VBUS sense on standard Pico |
| 25 | LED | FIXED | |
| 26 | Audio DATA/PWM0/LATCH_595 | REASSIGN | |
| 27 | Audio BCK/PWM1/CLK_595 | REASSIGN | |
| 28 | Audio LCK/BEEPER/DATA_595 | REASSIGN | |
| 29 | CLK_AY_PIN2 | REASSIGN | RP2040: GPIO29=ADC3/VSYS |

### Summary

| Board | MCU | GPIO | FIXED | REASSIGN | FREE | MIDI | NESPAD |
|-------|-----|------|-------|----------|------|------|--------|
| MURM2 | RP2350A | 0-29 | 15 | 9 | 5 | OK (GPIO 22) | **Conflict with PSRAM** (CLK=20, LAT=21) |
| PICO_PC | RP2350A | 0-29 | 13 | 10 | 6 | OK (GPIO 26) | OK |
| PICO_DV | RP2350 | 0-29,47 | 14 | 8 | 9 | **BUG** (GPIO 21=RX!) | Conflict with display (8,9) |
| ZERO | RP2040 | 0-28 | 16 | 8 | 5 | N/A (RP2040) | OK |
| ZERO2 | RP2350B | 0-47 | 14 | 12 | 22 | OK (GPIO 22) | Disabled |
| MURM | RP2040 | 0-28 | 18 | 10 | 0 | N/A (RP2040) | OK |

### Known bugs and conflicts

1. **PICO_DV MIDI_TX_PIN=21** — odd GPIO, hardware UART1 RX not TX. MIDI broken. Fix: move to GPIO 20
2. **MURM2 NESPAD vs PSRAM** — NES_CLK=20, NES_LAT=21 overlap PSRAM_MOSI=20, PSRAM_MISO=21. Cannot coexist
3. **PICO_DV NESPAD vs Display** — NES_CLK=8, NES_LAT=9 inside display range (6-13). USE_NESPAD correctly not set
4. **MURM2/MURM MIDI_TX=LOAD_WAV_PIO=22** — mutually exclusive features on same pin. Handled in code (warning in messages.h)

## ZiFi NIC — two host interfaces (both bridge to one ESP UART)

Port low byte `0xEF`; high address byte = register. Gated by `Config::zifi_enabled`.
- **ZIFI-API FIFO** (`#00EF`..`#C7EF`): hi ≤ 0xC7. `ZiFi::read/write`. DR data + ZIFR/ZOFR/IMR/CR. High-level FIFO interface.
- **16550 UART window** (`#F8EF`..`#FFEF`): hi ≥ 0xF8. `ZiFi::uart16550Read/Write`. reg = hi&7: 0=RBR/THR(or DLL if DLAB), 1=IER/DLM, 2=IIR/FCR, 3=LCR, 4=MCR, 5=LSR, 6=MSR, 7=SCR. THR/RBR bridge to the SAME `zifi_in_buf`/`zifi_out_buf` as the API. LSR=`0x60 | (rx?1:0)`; baud fixed 115200 8N1 (divisor latches stored, ignored).
- Most real ZiFi software (e.g. `debug/NET/MRF.TRD` terminal, drivers ZW-64/ZW-64-SC/GZ-80) uses the **16550 window**, not the API. Verified by disasm: `LD B,#Fx; LD C,#EF; OUT (C),A` + `IN A,(#FDEF)` LSR poll. App sends its own AT commands over the bridge.
- Wired in `Ports::input`/`Ports::output` after the API check.

### UART TX/RX pins — runtime, per-board (`src/BoardPins.*`)
- **No compile-time pin define** anymore (old `-DZIFI_TX_PIN` removed). `ZiFi::init()` reads `Config::zifi_tx_pin`/`zifi_rx_pin` and resolves via `BoardPins::resolveZifiPins()` + `uartInstanceForTx()` (authoritative RP2350 pinmux from `rp2350[ab]_interface_pins.json` — the old `(pin/4)%2` heuristic was WRONG for GPIO 8/10/24/26). UART instance/funcsel chosen at runtime; `g_uart`/`g_uart_irq` statics.
- Config sentinels: `0xFE` = board default, `0xFF` = OFF (no UART, FIFO-only), else explicit TX (RX = odd partner). Stored in NVS (`zifi_tx_pin`/`zifi_rx_pin`).
- **Per-board candidate pairs** + defaults live in `BoardPins.cpp` (`#if PICO_DV/MURM2/PICO_PC/ZERO2/#else MURM1_P2`). Defaults: PICO_DV 0/1, MURM2 20/21, PICO_PC 20/21, ZERO2 24/25, MURM1_P2 16/17.
- **Picker**: Network → first row `GPIO x/y` (or `GPIO Off`, `(def)` suffix when unset) → submenu listing `Off` + each board pair with a note (what it displaces, e.g. "off: NESPAD"). On select: save + `ZiFi::deinit()/init()` if NIC on. `BoardPins` is the reusable home for board pin-maps (extend for MIDI etc.).
- **Yield-at-boot**: when a chosen pair shares pins with a peripheral (non-empty note), ZiFi has priority — at boot each conflicting peripheral calls `BoardPins::zifiOwnsPin(pin)` and **skips its own init** if ZiFi owns it: NESPAD (`main.cpp`, moved after `Config::load`, gated by `nespad_active`), MIDI (`ESPectrum.cpp` `Midi::enabled=0`), WAV (`pwm_audio.cpp` skip `inInit`), PCM5122 (`pwm_audio.cpp` skip I2S), AY-clock (`PinSerialData_595.c` via `extern "C" board_zifi_owns_pin`). The displaced peripheral only releases pins at boot, so selecting a conflicting pair **or** enabling the NIC with a conflicting default prompts `OSD_DLG_APPLYREBOOT` (`BoardPins::zifiActiveNote()` non-empty). Defaults that conflict by design: MURM2/PICO_PC 20/21 = NESPAD, MURM1_P2 16/17 = NESPAD.

## RTC / Time (Pentagon Mr Gluk TimeKeeper)

- `src/RTC.*` — MC146818 emulation (RP2350-only). Ports (Pentagon/Profi):
  - `OUT (#DFF7), reg` — latch register index (confirmed via `OUT (C),H` at Gluk ROM 0x11BA)
  - `OUT (#BFF7), data` / `IN A,(#BFF7)` — data register (runtime-unpacked, not in static ROM)
  - Wired in `Ports::input`/`Ports::output`; responds on `isPentagon||isProfi` (NOT gated on EFF7 bit7 CMOS, for robustness — those ports are RTC-specific on these machines)
- Reg B=0x02 (24h, BCD — what Gluk expects); Reg D bit7 VRT=1 (battery valid); Reg A UIP=0 always. Clock regs 0x00-0x09 computed live from `base_secs + elapsed_ms` (no per-register tick); guest writes to time regs ignored.
- Time source: SNTP via ZiFi ESP — `ZiFiAT::syncTime(tz, out)` sends `AT+CIPSNTPCFG=1,tz,"pool.ntp.org"` then polls `AT+CIPSNTPTIME?` (parses `+CIPSNTPTIME:Www Mmm dd hh:mm:ss yyyy`, accepts year≥2020).
- Trigger: **manual** — Network menu → "Sync time (SNTP)". Timezone via Network → "Time zone" (UTC−12..+14 list → `Config::wifi_tz`, saved to wifi.cfg key `tz`).
- **Network menu** (RP2350, built dynamically): row 1 = `WiFi On <ssid> <ip>` / `WiFi Off` (live status, padded to fixed 32 width so geometry stays stable) then `Sync time (SNTP)` / `Time zone >` / `ZiFi NIC >`. Selecting the **WiFi** row is the all-in-one action — connected: SSID+IP + disconnect (msgDialog); not connected: `AT+CWLAP` scan → pick SSID (dynamic menuRun list) → password (`wifiAskPassword` box over `OSD::inlineTextEdit`) → connect → saves SSID/pass to wifi.cfg. Status is cached (`getStatus` is blocking) and refreshed on menu entry + after connect/disconnect/NIC-toggle. Connect/Disconnect/Reload items removed.
- **wifi.cfg** lives in `CONFIG_DIR` (`/.config/pico-spec/wifi.cfg`); legacy `/wifi.cfg` still read as fallback. `Config::saveWifiConfig()` writes ssid/pass/tz/autoconnect; `ZiFiAT::scan()` parses `+CWLAP`.
- **Auto-sync on boot**: when `ZiFi::enabled && Config::rtc_enabled && wifi_ssid` set, `ESPectrum::loop` kicks off `ZiFiAT::autoSyncBegin()` ~4 s in, then `autoSyncPoll()` each tick. Non-blocking background state machine (CWMODE→CWJAP→CIPSNTPCFG→poll CIPSNTPTIME?, ~15 retries) — **no OSD, never freezes** audio/video; writes straight into RTC, silent on failure. Manual menu sync still uses the blocking `syncTime()`.
- **"NO CMOS" fix (hw-confirmed)**: Gluk treats CMOS valid only when NVRAM **reg 0x11 == 0xAA** (unpacked-RAM check at 0x6049 `CP 0xAA / JR NZ`); reg 0x12 == 0x47 (`'G'`) gates loading the 27-byte config (regs 0x13–0x2D → RAM 0x63A1). No checksum. Gluk's auto-path writes a bogus 0x55 and never self-validates (real signature written only on menu-save). `RTC::init()` seeds `regs[0x11] = 0xAA` after `loadNVRAM()` so the clock works out of the box; Gluk then reads time regs 0x00–0x09.
- NVRAM (0x0E–0x3F + reg B) persisted to `CONFIG_DIR/cmos.nvr` (battery emulation): `loadNVRAM()` at init, dirty-flushed from main loop via `RTC::flushNVRAM()`.
- `RTC_PORT_TRACE` CMake option (default OFF) logs every `..F7` IN/OUT for debugging.
- **Toggle**: Options → Other → "RTC + NVRAM" (Yes/No → `Config::rtc_enabled`, default on, NVS-persisted). RP2350-only: the menu row is appended at runtime under `#if !PICO_RP2040`; when off, the `#DFF7`/`#BFF7` port handlers are bypassed (Gluk shows "NO CMOS").

## Tools

- `tools/z80disasm.py` — Z80 disassembler (pure Python3, no deps)
  - TAP files: `python3 tools/z80disasm.py input.tap` (auto-parses headers/blocks)
  - Raw binaries: `python3 tools/z80disasm.py code.bin --org 0x8000`
  - API: `from tools.z80disasm import disasm_bytes, disasm_bytes_text`
  - Supports all Z80 prefixes: CB, DD, FD, ED, DD CB, FD CB (including undocumented)
- [profi2png VGA detection fix](memory/profi2png_vga_detection.md) — max_byte>15 heuristic always fires for VGA std-mode (0xC0+ sync bits); fix: also require min_byte<0xC0.

## Test Files

- `FPGA48all.tap` — SAA1099 test program for ZX Spectrum 128K
  - Disassembly: `FPGA48all_disasm.txt`
  - Loader at 0x5E00, screen at 0x4000, main code at 0x6200
  - Main code starts with `CALL 0x817E` (IM 2 setup), uses SAA1099 port 0x01FE
