# Network

> Menu path: **Network** · Boards: **RP2350 only** (RP2040 unsupported) · Requires: an **ESP-01S** (ESP8266) module with stock AT firmware + SD card

## What it is

Networking via an **ESP-01S** module on the UART. The ESP runs the **stock Espressif
AT firmware** — no reflashing needed. The RP2350 side implements:

- **ZiFi NIC** — a network interface for ZX-Spectrum software (port `#EF`, 16550-UART
  window). Compatible with the **MRF** terminal/drivers (see [Software](#software)).
- **WiFi** — join an access point (scan, password). Credentials are saved and
  reconnected automatically on the next boot.
- **SNTP time sync** → built-in clock (RTC / Mr Gluk TimeKeeper) — runs
  automatically right after a WiFi connect and at boot, or on demand. Needs the
  RTC enabled (**Options → Other → RTC + NVRAM**).
- **File transfer** — an **FTP / SFTP / SSH** client right in the OSD: browse remote
  directories, download/upload files to SD, copy folders, delete. SSH crypto
  (curve25519, AES-CTR, HMAC-SHA256) runs on the RP2350 via mbedTLS;
  the ESP is just a TCP bridge.
- **FTP server** — share the SD card to the LAN. A PC connects to the device, browses
  the SD and uploads/downloads files (the reverse of File transfer). A live log
  terminal shows activity; emulation is paused until you press **ESC**. Anonymous,
  active mode only.

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
├── FTP Server ▸          share the SD card to the LAN (see below)
└── ZiFi NIC ▸            Off / On — enable the network interface
```

## Options

| Item | Values | Default | Description |
|------|--------|---------|-------------|
| ESP01 › GPIO | board TX/RX pairs | board-specific | UART pins to the ESP. Conflicting pairs are tagged (e.g. `off: NESPAD`) — changing one needs a reboot |
| ESP01 › Baud | 115200 / 230400 / 460800 / 921600 | 115200 | UART speed. Higher = faster transfers (~8× at 921600). Applied live via `AT+UART_CUR` |
| ESP01 › Time zone | UTC−12 … UTC+14 | UTC+0 | Timezone for SNTP |
| ESP01 › Sync time | — | — | On-demand SNTP sync (needs WiFi + RTC enabled). Also auto-runs on connect/boot |
| WiFi | — | — | Disconnected → scan + password → connect (+ auto time-sync); connected → show SSID/IP, disconnect |
| FTP Server | — | — | Share the SD card to the LAN over FTP (anonymous, active mode). Emulation pauses; **ESC** stops |
| ZiFi NIC | Off / On | Off | Enables the network interface. Stored in NVS |

> On speed: **921600** is the fastest, but with no hardware flow control (the ESP-01S
> doesn't expose RTS/CTS) marginal wiring can drop the odd byte — under SFTP that shows
> as a `MAC mismatch` and a dropped session (FTP wouldn't notice). If you hit it, pick
> **460800** or lower.

## File transfer (FTP / SFTP)

Requires WiFi connected (the **ZiFi NIC** toggle is independent — File transfer works
whether it's on or off).

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

## FTP server (sharing the SD card)

The reverse of File transfer: the device becomes an **FTP server** that shares the
**whole SD card** to the LAN, so a PC can browse it and upload/download files.
Requires WiFi connected.

**Network → FTP Server** starts a server on **port 21** and opens a live **log
terminal** showing the connection details (`ftp://<ip>:21`, user `anonymous`) and each
command/response. The ZX-Spectrum emulation is **paused** the whole time; press
**ESC** to stop the server and resume.

- **Anonymous** — any username/password is accepted (e.g. `anonymous` with an empty
  password). Full read/write access to the SD.
- **Active mode only.** ESP-AT firmware exposes a single listening port, so the data
  connection is opened **outbound** to the client. Set your FTP client to **active**
  (not passive) mode. On a LAN this just works; behind NAT/firewall it may not.
- **One client at a time.** Binary transfers only (`TYPE I`).
- Supported commands: list (`LIST`/`NLST`), download (`RETR`), upload (`STOR`/`APPE`),
  delete (`DELE`), make/remove directory (`MKD`/`RMD`), rename (`RNFR`/`RNTO`),
  `SIZE`, `CWD`/`CDUP`, `PWD`.

Command-line `ftp` example:

```
ftp> open <device-ip> 21
Name: anonymous
Password:           (just press Enter)
ftp> binary
ftp> ls
ftp> get game.tap
ftp> put snapshot.z80
ftp> bye
```

> **FileZilla / GUI clients:** set *Active* mode (Site Manager → Transfer Settings →
> Active) and *Anonymous* logon — passive mode will fail to open the data connection.

No persistent settings: the FTP server is a transient action with no config of its own.

## ESP-01S wiring

Just **4 wires** — power (VCC, GND) and a serial pair (TX, RX). Both sides are
**3.3 V** logic, so **no level shifter** is needed.

**The serial pair crosses over:** each side's **TX → the other side's RX**.

The diagram below uses the **PICO_DV** default pins (UART0: GPIO 0 = TX, GPIO 1 = RX).
For other boards only the GPIO numbers change — see the table below.

```
        ESP-01S                              RP2350  (example: PICO_DV)
   ┌───────────────────┐
   │  ((( antenna )))  │
   │   ┌──────────┐    │
   │   │ ESP8266  │    │
   │   └──────────┘    │
   │              TX ●─┼──────────────────●  GPIO 1   ( Pico RX )
   │              RX ●─┼──────────────────●  GPIO 0   ( Pico TX )
   │             GND ●─┼──────────────────●  GND
   │             VCC ●─┼──────────────────●  3V3   ( solid 3.3 V — see note )
   │                   │
   └───────────────────┘
   EN / RST / GPIO0 / GPIO2 on the ESP — leave unconnected

   The crossover is baked into the labels:  ESP TX → Pico RX,  ESP RX → Pico TX.
```

Connection list (PICO_DV):

| ESP-01S |   | RP2350 (PICO_DV) | meaning |
|---------|---|------------------|---------|
| **TX**  | ──► | **GPIO 1** | ESP TX → Pico **RX** |
| **RX**  | ◄── | **GPIO 0** | ESP RX ← Pico **TX** |
| **GND** | ─── | **GND** | common ground |
| **VCC** | ─── | **3V3** | 3.3 V power |

> **The crossover rule never changes** (only the GPIO numbers do): **Pico TX (even
> pin) → ESP RX**, and **Pico RX (odd pin) → ESP TX**. Connecting TX→TX / RX→RX is the
> #1 reason "nothing happens".

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

## Files on SD

| Path | Purpose |
|------|---------|
| `/.config/pico-spec/wifi.cfg` | WiFi + network settings (keys below). Legacy `/wifi.cfg` is still read as a fallback |
| `/.config/pico-spec/known_hosts` | SFTP host-key trust (TOFU): one `host base64(SHA-256)` line per server |
| `/spec` | Default download / copy destination |

`wifi.cfg` keys (`key=value`, one per line): `ssid`, `pass` (WiFi password), `tz`,
`autoconnect`, `baud`, `net_host`, `net_user`, `net_port`, `net_proto` (0=FTP, 1=SFTP),
`net_dl` / `net_ul` (last download/upload folders). The FTP/SFTP **server** password is
**not** stored — it's re-prompted each session.

## Links

- ESP-01S / AT commands (Espressif AT): <https://docs.espressif.com/projects/esp-at/en/latest/>
- Developer notes on ZiFi/networking: [../../../CLAUDE.md](../../../CLAUDE.md)
