# Notice

cascade-rs is a derivative work of CASCADE (Car Scanner Device Emulator),
(C) Copyright 2014 Ulrich Hecht, https://github.com/uli/cascade,
and is distributed under the CASCADE Public License 1.0 (see `LICENSE`).

Changes relative to the original: rewritten in Rust. Behavioural
differences from upstream are listed here as they are introduced.

- ROM data-window bound: upstream accepts a bank at `phys` only when
  `phys <= rom_size - 0xc000`; this port uses `phys + 0x4000 <= rom_size`
  (see `oracle/patches/0002-rom-data-window-bound.patch`).

No ROM packs, vendor libraries or other proprietary material are included.
