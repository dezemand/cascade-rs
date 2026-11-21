//! Hi-Scan keypad matrix (port of `keypad.cpp`). Four active-low lines,
//! selected through I/O 0x202 and read at 0x200.

/// A key, encoded as upstream does: line in bits 8..9, bit mask in 0..7.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub struct Key(pub u16);

impl Key {
    pub const F1: Key = Key(0x301);
    pub const F2: Key = Key(0x302);
    pub const F3: Key = Key(0x304);
    pub const F4: Key = Key(0x308);
    pub const F5: Key = Key(0x310);
    pub const F6: Key = Key(0x320);
    pub const ESCAPE: Key = Key(0x340);
    pub const N1: Key = Key(0x201);
    pub const N2: Key = Key(0x202);
    pub const N3: Key = Key(0x204);
    pub const N4: Key = Key(0x208);
    pub const N5: Key = Key(0x210);
    pub const N6: Key = Key(0x220);
    pub const N7: Key = Key(0x240);
    pub const N8: Key = Key(0x101);
    pub const N9: Key = Key(0x102);
    pub const N0: Key = Key(0x104);
    pub const UP: Key = Key(0x108);
    pub const DOWN: Key = Key(0x110);
    pub const LEFT: Key = Key(0x120);
    pub const RIGHT: Key = Key(0x140);
    pub const HELP: Key = Key(0x001);
    pub const SHIFT: Key = Key(0x002);
    pub const UNDO: Key = Key(0x004);
    pub const NO: Key = Key(0x008);
    pub const YES: Key = Key(0x010);
    pub const BACKLIGHT: Key = Key(0x020);
    pub const ENTER: Key = Key(0x040);

    /// Parse a key name as used on the command line (`enter`, `f1`, `3`, ...).
    pub fn from_name(s: &str) -> Option<Key> {
        Some(match s.to_ascii_lowercase().as_str() {
            "f1" => Key::F1,
            "f2" => Key::F2,
            "f3" => Key::F3,
            "f4" => Key::F4,
            "f5" => Key::F5,
            "f6" => Key::F6,
            "esc" | "escape" => Key::ESCAPE,
            "1" => Key::N1,
            "2" => Key::N2,
            "3" => Key::N3,
            "4" => Key::N4,
            "5" => Key::N5,
            "6" => Key::N6,
            "7" => Key::N7,
            "8" => Key::N8,
            "9" => Key::N9,
            "0" => Key::N0,
            "up" => Key::UP,
            "down" => Key::DOWN,
            "left" => Key::LEFT,
            "right" => Key::RIGHT,
            "help" => Key::HELP,
            "shift" => Key::SHIFT,
            "undo" => Key::UNDO,
            "no" => Key::NO,
            "yes" => Key::YES,
            "backlight" => Key::BACKLIGHT,
            "enter" | "return" => Key::ENTER,
            _ => return None,
        })
    }
}

#[derive(Clone)]
pub struct Keypad {
    lines: [u8; 4],
}

impl Default for Keypad {
    fn default() -> Self {
        Self { lines: [0x7f; 4] }
    }
}

impl Keypad {
    pub fn set(&mut self, key: Key, down: bool) {
        let line = &mut self.lines[(key.0 >> 8) as usize & 3];
        let mask = (key.0 & 0xff) as u8;
        if down {
            *line &= !mask;
        } else {
            *line |= mask;
        }
    }

    pub fn line(&self, n: usize) -> u8 {
        self.lines[n]
    }
}
