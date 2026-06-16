# Network

> Menu path: **Network** · Boards: **RP2350 only** (RP2040 unsupported) · Requires: an **ESP-01S** (ESP8266) module with stock AT firmware + SD card

## What it is

Networking via an **ESP-01S** module on the UART. The ESP runs the **stock Espressif
AT firmware** — no reflashing needed. The RP2350 side implements:

- **ZiFi NIC** — a network interface for ZX-Spectrum software (port `#EF`, 16550-UART
  window). Compatible with the **MRF** terminal/drivers (see [Software](#software)).
- **WiFi** — join an access point (scan, password, autoconnect).
- **SNTP time sync** → built-in clock (RTC / Mr Gluk TimeKeeper).
- **File transfer** — an **FTP / SFTP / SSH** client right in the OSD: browse remote
  directories, download/upload files to SD, copy folders, delete. SSH crypto
  (curve25519, AES-CTR, HMAC-SHA256) runs on the RP2350 via mbedTLS;
  the ESP is just a TCP bridge.

## Menu structure

```
Network
├── ESP01 ▸
│   ├── GPIO ▸            choose the UART pin pair (TX/RX)
│   ├── Baud ▸            115200 / 230400 / 460800 / 921600
│   ├── Time zone ▸       UTC−12 … UTC+14
│   └── Sync time (SNTP)  one-shot clock sync
├── WiFi <status>         connect / disconnect (shows SSID + IP)
├── File transfer ▸       FTP / SFTP client (see below)
└── ZiFi NIC ▸            Off / On — enable the network interface
```

## Options

| Item | Values | Default | Description |
|------|--------|---------|-------------|
| ESP01 › GPIO | board TX/RX pairs | board-specific | UART pins to the ESP. Conflicting pairs are tagged (e.g. `off: NESPAD`) — changing one needs a reboot |
| ESP01 › Baud | 115200 / 230400 / 460800 / 921600 | 115200 | UART speed. Higher = faster transfers (~8× at 921600). Applied live via `AT+UART_CUR` |
| ESP01 › Time zone | UTC−12 … UTC+14 | UTC+0 | Timezone for SNTP |
| ESP01 › Sync time | — | — | One-shot time sync (needs WiFi) |
| WiFi | — | — | Disconnected → scan + password; connected → show SSID/IP, disconnect |
| ZiFi NIC | Off / On | Off | Enables the network interface. Stored in NVS |

> On speed: **921600** is the fastest, but with no hardware flow control (the ESP-01S
> doesn't expose RTS/CTS) marginal wiring can drop the odd byte — under SFTP that shows
> as a `MAC mismatch` and a dropped session (FTP wouldn't notice). If you hit it, pick
> **460800** or lower.

## File transfer (FTP / SFTP)

**Network → File transfer** → pick the protocol (FTP / SFTP) → enter host, user, port
and password. The password is shown as **asterisks**; **TAB** reveals/hides it. For
SFTP, the host key fingerprint (**SHA-256**) is shown on first connect — trust is saved
to `known_hosts` on SD (TOFU); a changed key blocks the connection (MITM guard).

Remote file browser (RAM-bounded — the listing is indexed on SD, like the SD-card
browser):

| Key | Action |
|-----|--------|
| ↑ ↓ / PgUp PgDn / Home End | navigate |
| Enter (on a folder) | enter the directory |
| Enter (on a file) | download → pick the SD destination folder |
| **F5** | copy file/folder (folder = recursive) → pick the SD folder |
| **F8 / Del** | delete file/folder (with confirmation) |
| `[Upload file here]` | upload an SD file into the current remote directory |
| `..` | go up one level |
| Esc | exit |

The last upload/download folders are remembered (in `wifi.cfg`). Downloads default to
`/spec`.

## ESP-01S wiring

Just **4 wires**. Both sides are **3.3 V** logic — no level shifter needed.
**TX and RX cross over.**

```
   ESP-01S                       RP2350 (pins — see table below)
   TX   ───────────────────────►  RX    (zifi_rx, odd pin = tx+1)
   RX   ◄───────────────────────  TX    (zifi_tx, even pin)
   GND  ───── GND
   VCC  ───── 3V3                 (stable 3.3 V)

   EN/CH_PD, RST, GPIO0, GPIO2 — leave unconnected
```

> **Power.** The ESP-01S draws bursts up to ~300 mA on WiFi TX. Feed it from a solid
> 3.3 V rail and add a **100–470 µF** cap at VCC — otherwise expect brownout resets.
> The Pico's own 3V3 may be marginal.

### Default pins per board

Change the pair in **Network → ESP01 → GPIO**. TX is the even pin, RX its odd neighbour.

| Board | TX / RX (default) | UART | Note |
|-------|-------------------|------|------|
| **PICO_DV** | **0 / 1** | UART0 | dedicated ZiFi header |
| **MURM2** | **20 / 21** | UART1 | takes NESPAD pins |
| **PICO_PC** | **20 / 21** | UART1 | takes NESPAD pins; alt `2/3` (QWST1), `10/11` |
| **ZERO2** | **24 / 25** | UART1 | free |
| **MURM-1 (RP2350)** | **16 / 17** | UART0 | takes NESPAD pins |

Full board GPIO maps: [Boards & pinout](../hardware/boards.md).

## Software

No Spectrum-side software is needed for File transfer — it's all done from the OSD.
For the **ZiFi NIC** (networking inside ZX-Spectrum programs) use the **MRF**
terminal/drivers:

- **MRF (Moon Rabbit Firmware / terminal)**: <https://zxart.ee/eng/software/prikladnoe-po/mrf/tabs:releases/>

## Links

- ESP-01S / AT commands (Espressif AT): <https://docs.espressif.com/projects/esp-at/en/latest/>
- Developer notes on ZiFi/networking: [../../../CLAUDE.md](../../../CLAUDE.md)
