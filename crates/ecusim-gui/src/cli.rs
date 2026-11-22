//! ecusim-gui's command line.

use std::path::PathBuf;

/// With neither a server answering nor --rom, a setup screen asks.
#[derive(clap::Parser)]
#[command(
    name = "ecusim-gui",
    about = "The car's controls for an `ecusim` ECU process",
    args_override_self = true
)]
pub struct Cli {
    /// Start `ecusim serve` with this dump if no server answers (stopped on
    /// exit)
    #[arg(long, value_name = "SIMK41.bin")]
    pub rom: Option<String>,
    /// The server's socket (default `$TMPDIR/ecusim.sock`)
    #[arg(long, value_name = "PATH")]
    pub socket: Option<PathBuf>,
}
