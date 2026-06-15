<!--
FEATURE PAGE TEMPLATE.
Copy this file, rename it, fill it in. Delete this comment.
Rules:
  - 1 page = 1 top-level menu item (describe its submenu via the options table).
  - The blockquote header is mandatory: menu path / supported boards / requirements.
  - Do NOT commit test images — link them + say what they verify (see images.md).
  - Put pictures under docs/assets/, reference them with a relative path.
-->

# Feature name (as shown in the menu)

> Menu path: **Section › Item** · Boards: all / RP2350 / Waveshare … · Requires: PSRAM / SD …

## Overview

Short description: what it emulates/does, why, which original hardware/chips.

## Menu options

| Item | Values | Default | Description |
|------|--------|---------|-------------|
| AY enabled | On / Off | On | Enables AY-3-8912 emulation |

## Wiring / schematic

If the feature needs a physical connection (audio, MIDI, joystick, DAC):

![Schematic](../assets/schematics/name.png)

Otherwise, remove this section.

## How to test

| Image | Where to get | What it verifies | Expected result |
|-------|--------------|------------------|-----------------|
| `Example.tap` | [link](https://…) | AY registers | 3-channel music plays |

## Links

- Upstream project: <https://…>
- Chip / format documentation: <https://…>
