//! cascade-gui's command line.

/// Environment: CASCADE_ROM (the pack when none is given), CASCADE_EXROM
/// (--exrom), CASCADE_ECU_ROM (preloads the ECU line without selecting it).
/// Without a pack the setup screen opens.
#[derive(clap::Parser)]
#[command(
    name = "cascade-gui",
    about = "Desktop front end for the cascade-rs Hi-Scan emulator",
    args_override_self = true
)]
pub struct Cli {
    /// The Hi-Scan ROM pack (default: $CASCADE_ROM)
    pub rom: Option<String>,
    /// An extended ROM (troubleshooting pack)
    #[arg(long, value_name = "INFO.dat")]
    pub exrom: Option<String>,
    /// The EEPROM file (default: ROM.eep next to the ROM, as upstream)
    #[arg(long, value_name = "FILE", conflicts_with = "no_eeprom")]
    pub eeprom: Option<String>,
    /// Keep the EEPROM in memory only
    #[arg(long)]
    pub no_eeprom: bool,
    /// Answer from a `kdiag --trace` recording
    #[arg(long, value_name = "KDIAG.jsonl")]
    pub replay: Option<String>,
    /// A SIMK41 engine ECU dump, with the ECU line selected at start
    #[arg(long, value_name = "SIMK41.bin")]
    pub ecu_rom: Option<String>,
}
