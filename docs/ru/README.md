# Документация pico-spec (RU)

> [English version](../en/README.md) · [Главная](../README.md)

Оглавление повторяет структуру главного меню эмулятора (вызывается в OSD).
Найди нужный пункт так же, как видишь его на экране.

## Главное меню

| Пункт меню | Описание |
|------------|----------|
| 💾 [Storage](menu/01-storage.md) | TAP/SNA/DSK, TR-DOS, esxDOS, IDE/HDD, MB-02+ |
| 🔊 [Audio](menu/02-audio.md) | AY, SAA1099, TurboSound, Covox, SounDrive, MIDI, General Sound |
| 📺 [Video](menu/03-video.md) | Режимы VGA/HDMI, scanlines, ULA+, Timex, Pentagon 16c, Profi DS80 |
| 🖥️ [Machine](menu/04-machine.md) | 48K / 128K / +2 / Pentagon / Profi / Byte / ALF |
| ♻️ [Reset](menu/05-reset.md) | Сброс и режимы запуска |
| ⚙️ [Options](menu/06-options.md) | Модель/ROM по умолчанию, джойстик, язык, обновление |
| 🐞 [Debug](menu/07-debug.md) | Отладчик, дампы, трассировка |
| 🔧 [Hardware](menu/08-hardware.md) | Инфо о чипе/плате, HID, Speed Test, Overclock |
| 🌐 [Network](menu/10-network.md) | ESP-01S: WiFi, SNTP, ZiFi NIC, FTP/SFTP/SSH (RP2350) |
| 📦 [Прочее](menu/09-other.md) | Volume, ZX Keyboard, Help, About, TFT |

## Сквозные темы

| Тема | Описание |
|------|----------|
| [Платы и распиновка](hardware/boards.md) | GPIO-карты всех поддерживаемых плат |
| [MIDI](hardware/midi.md) | Подключение MIDI-выхода (RP2350) |
| [PCM5122 (I2S DAC)](hardware/audio-dac-pcm5122.md) | Аудио-ЦАП для плат Waveshare PiZero |

## Тестовое ПО

- [Каталог тестовых образов](images.md) — какой образ что проверяет и где его взять.

## Для авторов

- [Шаблон страницы](_template.md) — копируй его при добавлении новой фичи.
