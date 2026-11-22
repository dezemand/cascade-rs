//! Open-loop engine model **[assumed]** and the sensor curves.
//!
//! The model follows the controls: the starter turns the engine at about
//! 250 rpm, after half a second of cranking it "starts" and idles, and the
//! throttle raises the speed. It does not depend on the ECU's injection and
//! ignition outputs (not decoded yet). Its MAP, battery and O2 behaviour are
//! plausible textbook values, nothing measured.
//!
//! The sensor curves are measured: each ADC channel was swept through the
//! ROM in the emulator and the value the ROM reports in its `21 01` block
//! read back (non-immobiliser GK 2.0 dump). Channels: 0 MAP, 2 front O2,
//! 3 rear O2, 4 battery, 10 throttle, 11 coolant, 14 intake air.

/// What the user controls.
#[derive(Clone, Debug, PartialEq)]
pub struct Controls {
    pub ignition: bool,
    pub starter: bool,
    /// Accelerator, 0-100 %.
    pub throttle: f64,
    pub coolant_c: f64,
    pub intake_c: f64,
    /// Battery at rest, engine off.
    pub battery_v: f64,
    pub vehicle_kmh: f64,
    /// Idle speed the model settles at.
    pub idle_rpm: f64,
    /// Fixed engine speed, overriding the model (0 = model).
    pub rpm_override: f64,
}

impl Default for Controls {
    fn default() -> Self {
        Self {
            ignition: false,
            starter: false,
            throttle: 0.0,
            coolant_c: 20.0,
            intake_c: 20.0,
            battery_v: 12.6,
            vehicle_kmh: 0.0,
            idle_rpm: 800.0,
            rpm_override: 0.0,
        }
    }
}

impl Controls {
    /// Set one control from text (`SET name value`).
    pub fn set(&mut self, name: &str, value: &str) -> Result<(), String> {
        let on = || match value {
            "on" | "1" | "true" => Ok(true),
            "off" | "0" | "false" => Ok(false),
            v => Err(format!("{name}: expected on/off, got {v:?}")),
        };
        let num = |lo: f64, hi: f64| -> Result<f64, String> {
            let v: f64 = value.parse().map_err(|e| format!("{name}: {e}"))?;
            if (lo..=hi).contains(&v) {
                Ok(v)
            } else {
                Err(format!("{name}: {v} is outside {lo}..{hi}"))
            }
        };
        match name {
            "ignition" => self.ignition = on()?,
            "starter" => self.starter = on()?,
            "throttle" => self.throttle = num(0.0, 100.0)?,
            "coolant" => self.coolant_c = num(-15.0, 135.0)?,
            "intake" => self.intake_c = num(-15.0, 135.0)?,
            "battery" => self.battery_v = num(0.0, 24.0)?,
            "speed" => self.vehicle_kmh = num(0.0, 250.0)?,
            "idle" => self.idle_rpm = num(500.0, 2000.0)?,
            "rpm" => self.rpm_override = num(0.0, 7000.0)?,
            n => return Err(format!("unknown control {n:?}")),
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Engine {
    Off,
    Cranking,
    Running,
}

impl Engine {
    pub fn name(self) -> &'static str {
        match self {
            Engine::Off => "off",
            Engine::Cranking => "cranking",
            Engine::Running => "running",
        }
    }
}

/// The model's state and outputs.
#[derive(Clone, Debug)]
pub struct Plant {
    pub engine: Engine,
    pub rpm: f64,
    cranked_s: f64,
    t: f64,
}

/// Sensor values the ECU sees, as engineering values and ADC counts.
#[derive(Clone, Debug, PartialEq)]
pub struct Sensors {
    pub battery_v: f64,
    pub map_kpa: f64,
    pub o2_front_mv: f64,
    pub o2_rear_mv: f64,
    pub throttle_deg: f64,
    /// (ADC channel, 10-bit value)
    pub adc: Vec<(usize, u16)>,
}

impl Default for Plant {
    fn default() -> Self {
        Self {
            engine: Engine::Off,
            rpm: 0.0,
            cranked_s: 0.0,
            t: 0.0,
        }
    }
}

/// Cranking speed.
pub const CRANK_RPM: f64 = 250.0;
/// Cranking time after which releasing the starter leaves it running.
pub const START_S: f64 = 0.5;

impl Plant {
    /// Advance by `dt` seconds.
    pub fn step(&mut self, c: &Controls, dt: f64) {
        self.t += dt;
        self.engine = match (self.engine, c.ignition, c.starter) {
            (_, false, _) => Engine::Off,
            (Engine::Running, true, _) => Engine::Running,
            (_, true, true) => Engine::Cranking,
            (Engine::Cranking, true, false) if self.cranked_s >= START_S => Engine::Running,
            (_, true, false) => Engine::Off,
        };
        if self.engine == Engine::Cranking {
            self.cranked_s += dt;
        } else {
            self.cranked_s = 0.0;
        }
        let target = match self.engine {
            Engine::Off => 0.0,
            Engine::Cranking => CRANK_RPM,
            Engine::Running if c.rpm_override > 0.0 => c.rpm_override,
            Engine::Running => {
                // a little faster when cold, then the throttle on top
                let cold = ((40.0 - c.coolant_c) / 60.0).clamp(0.0, 1.0) * 400.0;
                c.idle_rpm + cold + c.throttle / 100.0 * (6500.0 - c.idle_rpm)
            }
        };
        // first-order response: 0.3 s up, 0.6 s down
        let tau = if target > self.rpm { 0.3 } else { 0.6 };
        self.rpm += (target - self.rpm) * (dt / tau).min(1.0);
        if target == 0.0 && self.rpm < 30.0 {
            self.rpm = 0.0;
        }
    }

    /// Sensor values for the controls and the current state.
    pub fn sensors(&self, c: &Controls) -> Sensors {
        let running = self.engine == Engine::Running;
        let battery_v = match self.engine {
            _ if !c.ignition => c.battery_v,
            Engine::Cranking => c.battery_v - 2.0,
            Engine::Running => (c.battery_v + 1.5).max(13.8),
            Engine::Off => c.battery_v - 0.2,
        };
        let map_kpa = match self.engine {
            Engine::Off => 101.0,
            Engine::Cranking => 90.0,
            Engine::Running => 32.0 + c.throttle * 0.69,
        };
        let warm = c.coolant_c > 40.0;
        let o2_front_mv = if running && warm {
            // closed loop switching about once a second
            if ((self.t * 2.0) as u64).is_multiple_of(2) {
                120.0
            } else {
                780.0
            }
        } else {
            450.0
        };
        let o2_rear_mv = if running && warm { 650.0 } else { 450.0 };
        let throttle_deg = c.throttle / 100.0 * 82.0;
        let adc = vec![
            (0, adc_map(map_kpa)),
            (2, adc_o2(o2_front_mv)),
            (3, adc_o2(o2_rear_mv)),
            (4, adc_battery(battery_v)),
            (10, adc_throttle(throttle_deg)),
            (11, adc_ntc(c.coolant_c)),
            (14, adc_ntc(c.intake_c)),
        ];
        Sensors {
            battery_v,
            map_kpa,
            o2_front_mv,
            o2_rear_mv,
            throttle_deg,
            adc,
        }
    }
}

fn clamp10(v: f64) -> u16 {
    v.round().clamp(0.0, 1023.0) as u16
}

/// Battery: the ROM reports 1.625 V per 64 counts (0.0254 V/count).
pub fn adc_battery(v: f64) -> u16 {
    clamp10(v / (1.625 / 64.0))
}

/// MAP: the ROM's pressure word (0.00120317 psi per bit) rises 14.555 per
/// count from 1876 at count 128 (linear between counts 128 and 896).
pub fn adc_map(kpa: f64) -> u16 {
    let raw = kpa / 6.894_76 / 0.001_203_17;
    clamp10(128.0 + (raw - 1876.0) / 14.555)
}

/// Throttle: 0° up to count 256, then 0.1171875°/count (82.5° at 960).
pub fn adc_throttle(deg: f64) -> u16 {
    clamp10(256.0 + deg.clamp(0.0, 82.5) / 0.117_187_5)
}

/// O2 sensors: 4.878 mV per count.
pub fn adc_o2(mv: f64) -> u16 {
    clamp10(mv / (5000.0 / 1025.0))
}

/// Coolant and intake air (NTC; the two channels read the same within a
/// degree): count -> °C as the ROM reports it.
const NTC: [(u16, f64); 15] = [
    (64, 138.0),
    (128, 118.5),
    (192, 99.75),
    (256, 85.5),
    (320, 75.0),
    (384, 65.25),
    (448, 57.75),
    (512, 49.5),
    (576, 42.0),
    (640, 34.5),
    (704, 27.75),
    (768, 20.25),
    (832, 10.5),
    (896, 0.0),
    (960, -15.0),
];

pub fn adc_ntc(c: f64) -> u16 {
    let c = c.clamp(NTC[14].1, NTC[0].1);
    for w in NTC.windows(2) {
        let ((a0, t0), (a1, t1)) = (w[0], w[1]);
        if c <= t0 && c >= t1 {
            let f = (t0 - c) / (t0 - t1);
            return clamp10(a0 as f64 + f * (a1 - a0) as f64);
        }
    }
    NTC[14].0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn curves_hit_the_measured_points() {
        assert_eq!(adc_battery(13.0), 512);
        assert_eq!(adc_ntc(49.5), 512);
        assert_eq!(adc_ntc(20.25), 768);
        assert_eq!(adc_throttle(30.0), 512);
        // 7465 * 0.00120317 psi = 61.9 kPa at count 512
        assert_eq!(adc_map(7465.0 * 0.001_203_17 * 6.894_76), 512);
    }

    #[test]
    fn start_sequence() {
        let mut p = Plant::default();
        let mut c = Controls {
            ignition: true,
            ..Default::default()
        };
        p.step(&c, 0.1);
        assert_eq!(p.engine, Engine::Off);
        c.starter = true;
        for _ in 0..10 {
            p.step(&c, 0.1);
        }
        assert_eq!(p.engine, Engine::Cranking);
        assert!((p.rpm - CRANK_RPM).abs() < 20.0);
        c.starter = false;
        for _ in 0..30 {
            p.step(&c, 0.1);
        }
        assert_eq!(p.engine, Engine::Running);
        assert!(p.rpm > 900.0, "cold idle is raised: {}", p.rpm);
        c.ignition = false;
        for _ in 0..50 {
            p.step(&c, 0.1);
        }
        assert_eq!((p.engine, p.rpm), (Engine::Off, 0.0));
    }

    #[test]
    fn short_crank_does_not_start() {
        let mut p = Plant::default();
        let mut c = Controls {
            ignition: true,
            starter: true,
            ..Default::default()
        };
        p.step(&c, 0.2);
        c.starter = false;
        p.step(&c, 0.1);
        assert_eq!(p.engine, Engine::Off);
    }
}
