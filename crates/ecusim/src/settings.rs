//! What the cascade-rs apps remember between runs (last packs, dumps and
//! options): one `key=value` file per app in the user's settings folder
//! (`~/Library/Application Support/cascade-rs` on macOS, `$XDG_CONFIG_HOME`
//! or `~/.config/cascade-rs` elsewhere, `%APPDATA%\cascade-rs` on Windows).
//! Paths stay on this computer; nothing is written into the repository.
//! A missing or unreadable file is an empty one.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

/// How many recent entries a list keeps.
pub const RECENT: usize = 5;

#[derive(Clone, Debug, Default)]
pub struct Settings {
    file: Option<PathBuf>,
    map: BTreeMap<String, String>,
}

fn folder() -> Option<PathBuf> {
    let env = |k: &str| {
        std::env::var_os(k)
            .filter(|v| !v.is_empty())
            .map(PathBuf::from)
    };
    let base = if cfg!(target_os = "macos") {
        env("HOME")?.join("Library/Application Support")
    } else if cfg!(windows) {
        env("APPDATA")?
    } else {
        env("XDG_CONFIG_HOME").or_else(|| Some(env("HOME")?.join(".config")))?
    };
    Some(base.join("cascade-rs"))
}

impl Settings {
    /// The settings of `app` (a file name such as `ecusim-gui`).
    pub fn load(app: &str) -> Self {
        Self::at(folder().map(|f| f.join(format!("{app}.conf"))))
    }

    /// Settings kept in `file` (`None`: in memory only).
    pub fn at(file: Option<PathBuf>) -> Self {
        let map = file
            .as_deref()
            .and_then(|f| std::fs::read_to_string(f).ok())
            .unwrap_or_default()
            .lines()
            .filter_map(|l| l.split_once('='))
            .map(|(k, v)| (k.trim().to_string(), v.to_string()))
            .collect();
        Self { file, map }
    }

    pub fn get(&self, key: &str) -> Option<&str> {
        self.map.get(key).map(String::as_str)
    }

    pub fn set(&mut self, key: &str, value: Option<&str>) {
        match value {
            Some(v) => self.map.insert(key.into(), v.replace('\n', " ")),
            None => self.map.remove(key),
        };
    }

    pub fn flag(&self, key: &str, default: bool) -> bool {
        self.get(key).map_or(default, |v| v == "1")
    }

    pub fn set_flag(&mut self, key: &str, on: bool) {
        self.set(key, Some(if on { "1" } else { "0" }));
    }

    /// The recent paths under `key`, newest first, that still exist.
    pub fn recent(&self, key: &str) -> Vec<PathBuf> {
        (0..RECENT)
            .filter_map(|i| self.get(&format!("{key}.{i}")))
            .map(PathBuf::from)
            .filter(|p| p.exists())
            .collect()
    }

    /// Put `path` first in the recent list under `key`.
    pub fn push_recent(&mut self, key: &str, path: &Path) {
        let mut list: Vec<PathBuf> = (0..RECENT)
            .filter_map(|i| self.get(&format!("{key}.{i}")))
            .map(PathBuf::from)
            .filter(|p| p != path)
            .collect();
        list.insert(0, path.to_path_buf());
        list.truncate(RECENT);
        for i in 0..RECENT {
            let v = list.get(i).map(|p| p.to_string_lossy().into_owned());
            self.set(&format!("{key}.{i}"), v.as_deref());
        }
    }

    /// Write the file (errors are ignored: remembering is a convenience).
    pub fn save(&self) {
        let Some(f) = &self.file else {
            return;
        };
        if let Some(dir) = f.parent() {
            let _ = std::fs::create_dir_all(dir);
        }
        let text: String = self.map.iter().map(|(k, v)| format!("{k}={v}\n")).collect();
        let _ = std::fs::write(f, text);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recent_lists_and_round_trip() {
        let dir = std::env::temp_dir().join(format!("cascade-settings-{}", std::process::id()));
        let file = dir.join("t.conf");
        let a = dir.join("a.bin");
        let b = dir.join("b.bin");
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(&a, b"x").unwrap();
        std::fs::write(&b, b"x").unwrap();
        let mut s = Settings::at(Some(file.clone()));
        s.push_recent("rom", &a);
        s.push_recent("rom", &b);
        s.push_recent("rom", &a);
        s.set_flag("eeprom", false);
        s.save();
        let s = Settings::at(Some(file));
        assert_eq!(s.recent("rom"), vec![a.clone(), b]);
        assert!(!s.flag("eeprom", true));
        std::fs::remove_file(&a).unwrap();
        assert_eq!(s.recent("rom").len(), 1, "missing files drop out");
        let _ = std::fs::remove_dir_all(&dir);
    }
}
