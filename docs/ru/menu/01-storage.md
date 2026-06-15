# Storage — Хранилище

> Путь в меню: **Storage** · Платы: все (MB-02+, Z-Controller, IDE/HDD — только RP2350) · Требует: SD-карта

Работа с носителями: лента, дисководы (TR-DOS), esxDOS, MB-02+, IDE/HDD и
снимки памяти (snapshots). Состав пунктов зависит от платы и наличия SD-карты.

## Опции меню

| Пункт | Описание |
|-------|----------|
| Tape | Лента: загрузка `.tap` / `.tzx` / `.pzx`, реальный вход с ленты (audio-in), режим плеера. Поддержка flashload и турбо-загрузчиков. |
| Betadisk | TR-DOS: дисководы A–D, образы `.trd` / `.scl` / `.fdi` / `.udi` / `.td0` / `.pro` / `.mbd`. Выбор ROM TR-DOS, Fast Mode, Sound & LED. Работает с **любой** моделью (gated `Config::betadisk`). |
| esxDOS | esxDOS + DivMMC/DivIDE/DivSD: образ карты, NMI «Magic Button». |
| MB-02+ *(RP2350)* | Контроллер MB-02+ (BS-DOS): FDC + DMA + SRAM-пейджинг, RAM-диск 512K. |
| Z-Controller *(RP2350)* | Z-Controller (SD-карта для Z-Player и т.п.), двойная схема /CS. |
| IDE/HDD *(RP2350)* | Эмуляция IDE-диска (NEMO/PROFI): образы `.hdf` / `.hdd` / Fixed VHD, создание пустого образа. |
| Snapshot | Снимки памяти `.sna` / `.z80` / `.sp`: загрузка и сохранение состояния. |

## Подсистемы подробнее

- **Форматы дисков** — UDI, FDI, TD0 (Teledisk), PRO, MBD, TRD, SCL. Часть
  копи-защищённых FDI требует доводки (см. заметки разработчика).
- **TR-DOS** — не привязан к Pentagon: работает на 48K/128K/+2/Pentagon/Profi,
  включается флагом Betadisk; вход в TR-DOS обрабатывает `Z80::check_trdos`.
- **IDE/HDD** — модуль `src/IDE.*`, схемы NEMO и PROFI; ATAPI/CD (.iso) пока нет.
- **MB-02+** — boot EPROM, FDC, DMA, SRAM-пейджинг функциональны.

## Схема / подключение

Распиновка SD-карты (SPI) для каждой платы — в [Платы и распиновка](../hardware/boards.md).

## Как проверить

| Образ | Фича | Что проверяет | Ожидаемый результат |
|-------|------|---------------|---------------------|
| `48.udi` | Betadisk/UDI | Чтение UDI | Загрузка (нужен write-enabled диск) |
| TR-DOS `.trd` | Betadisk | Базовый TR-DOS | Каталог читается, программы стартуют |
| `.td0` (CP/M) | TD0 | Распаковка LZH | Q_DOS / CMP01 / Dos5 грузятся |

Полный список — в [каталоге образов](../images.md).

## Ссылки

- TR-DOS / Betadisk: <https://en.wikipedia.org/wiki/Beta_Disk_Interface>
- esxDOS: <http://www.esxdos.org/>
- MB-02+: <https://velesoft.speccy.cz/mb02-cz.htm>
- Формат TD0 (Teledisk): <http://dunfield.classiccmp.org/img47321/teledisk.htm>
