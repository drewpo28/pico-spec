# pico-spec Documentation (EN)

> [Русская версия](../ru/README.md) · [Home](../README.md)

The table of contents mirrors the emulator's main on-screen (OSD) menu. Find a
feature the same way you see it on screen.

> ⚠️ The English pages are stubs — content is authored first in [`ru/`](../ru/README.md)
> and translated here.

## Main menu

| Menu item | Description |
|-----------|-------------|
| 💾 [Storage](menu/01-storage.md) | TAP/SNA/DSK, TR-DOS, esxDOS, IDE/HDD, MB-02+ |
| 🔊 [Audio](menu/02-audio.md) | AY, SAA1099, TurboSound, Covox, SounDrive, MIDI, General Sound |
| 📺 [Video](menu/03-video.md) | VGA/HDMI modes, scanlines, ULA+, Timex, Pentagon 16c, Profi DS80 |
| 🖥️ [Machine](menu/04-machine.md) | 48K / 128K / +2 / Pentagon / Profi / Byte / ALF |
| ♻️ [Reset](menu/05-reset.md) | Reset and boot modes |
| ⚙️ [Options](menu/06-options.md) | Default machine/ROM, joystick, language, update |
| 🐞 [Debug](menu/07-debug.md) | Debugger, dumps, tracing |
| 🔧 [Hardware](menu/08-hardware.md) | Chip/Board info, HID, Speed Test, Overclock |
| 🌐 [Network](menu/10-network.md) | ESP-01S: WiFi, SNTP, ZiFi NIC, FTP/SFTP/SSH (RP2350) |
| 📦 [Other](menu/09-other.md) | Volume, ZX Keyboard, Help, About, TFT |

## Cross-cutting topics

| Topic | Description |
|-------|-------------|
| [Boards & pinout](hardware/boards.md) | GPIO maps for all supported boards |
| [MIDI](hardware/midi.md) | MIDI output wiring (RP2350) |
| [PCM5122 (I2S DAC)](hardware/audio-dac-pcm5122.md) | Audio DAC for Waveshare PiZero boards |

## Test software

- [Test image catalog](images.md)

## For authors

- [Page template](_template.md)
