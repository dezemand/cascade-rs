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
//! 3 rear O2, 4 battery, 8 EVAP pressure, 10 throttle, 11 coolant, 12 a
//! wheel-speed signal line, 14 intake air. [`Fault`]s override them.

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
    /// The ECU's battery supply (KL30). Off wipes its memory, fault codes
    /// included; the ignition only switches the engine and the K-line.
    pub battery_connected: bool,
    /// Injected faults (wiring faults on sensors).
    pub faults: std::collections::BTreeSet<Fault>,
    /// Stop the engine with the key left on (`SET engine stop`); taken by the
    /// next model step.
    pub stop_request: bool,
    /// Which ECU family's sensor wiring to drive (set from the dump).
    pub family: Family,
    /// Faults that come and go (a loose connector).
    pub intermittent: std::collections::BTreeSet<Fault>,
    /// Time runs this much faster than the wall clock (1-10) while no
    /// tester is connected.
    pub time_factor: f64,
    /// Engine model: closed loop on the ECU's own fuel, spark and idle air
    /// when the dump supports it (`auto`), or always open loop.
    pub open_loop_only: bool,
}

/// ECU families and how their sensors are wired (ADC channels; found with
/// `tools/fault_sweep.py`, emulator-derived).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Family {
    /// SIMK41 (256 KB): MAP on 0, front O2 on 2.
    #[default]
    Simk41,
    /// SIMK43 (512 KB, CVVT): MAF on 0, oil temperature on 6, front O2 on 7.
    Simk43,
}

impl Family {
    pub fn for_rom(rom: &[u8]) -> Self {
        if rom.len() > 256 * 1024 {
            Family::Simk43
        } else {
            Family::Simk41
        }
    }

    fn front_o2(self) -> usize {
        match self {
            Family::Simk41 => 2,
            Family::Simk43 => 7,
        }
    }
}

/// A wiring fault the ECU can detect. The codes are what the
/// non-immobiliser GK 2.0 dump stores for each (emulator-derived, read with
/// `18 00 00 00` after 30 s with the engine idling).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Fault {
    MapShort,
    MapOpen,
    CoolantShort,
    CoolantOpen,
    ThrottleShort,
    ThrottleOpen,
    O2FrontOpen,
    O2RearOpen,
    EvapShort,
    EvapOpen,
    BatteryLow,
    WheelSpeedCircuit,
    CamMissing,
    AbsWheelSensorFl,
    AbsPumpMotor,
    SrsDriverAirbag,
    SrsPassengerAirbag,
    OilTempShort,
    OilTempOpen,
    /// The automatic gearbox computer stops sending on CAN.
    CanGearboxSilent,
    /// The ESP stops sending on CAN.
    CanEspSilent,
    /// No other node on the engine CAN bus (nothing acknowledges).
    CanBusDead,
}

impl Fault {
    pub const ALL: [Fault; 22] = [
        Fault::MapShort,
        Fault::MapOpen,
        Fault::CoolantShort,
        Fault::CoolantOpen,
        Fault::ThrottleShort,
        Fault::ThrottleOpen,
        Fault::O2FrontOpen,
        Fault::O2RearOpen,
        Fault::EvapShort,
        Fault::EvapOpen,
        Fault::BatteryLow,
        Fault::WheelSpeedCircuit,
        Fault::CamMissing,
        Fault::AbsWheelSensorFl,
        Fault::AbsPumpMotor,
        Fault::SrsDriverAirbag,
        Fault::SrsPassengerAirbag,
        Fault::OilTempShort,
        Fault::OilTempOpen,
        Fault::CanGearboxSilent,
        Fault::CanEspSilent,
        Fault::CanBusDead,
    ];

    /// Name used on the socket (`SET fault.NAME on`).
    pub fn name(self) -> &'static str {
        match self {
            Fault::MapShort => "map_short",
            Fault::MapOpen => "map_open",
            Fault::CoolantShort => "coolant_short",
            Fault::CoolantOpen => "coolant_open",
            Fault::ThrottleShort => "throttle_short",
            Fault::ThrottleOpen => "throttle_open",
            Fault::O2FrontOpen => "o2_front_open",
            Fault::O2RearOpen => "o2_rear_open",
            Fault::EvapShort => "evap_short",
            Fault::EvapOpen => "evap_open",
            Fault::BatteryLow => "battery_low",
            Fault::WheelSpeedCircuit => "wheel_speed_circuit",
            Fault::CamMissing => "cam_missing",
            Fault::CanGearboxSilent => "can_gearbox",
            Fault::CanEspSilent => "can_esp",
            Fault::CanBusDead => "can_bus",
            Fault::AbsWheelSensorFl => "abs_wheel_sensor_fl",
            Fault::AbsPumpMotor => "abs_pump_motor",
            Fault::SrsDriverAirbag => "srs_driver_airbag",
            Fault::SrsPassengerAirbag => "srs_passenger_airbag",
            Fault::OilTempShort => "oil_temp_short",
            Fault::OilTempOpen => "oil_temp_open",
        }
    }

    pub fn from_name(n: &str) -> Option<Fault> {
        Fault::ALL.into_iter().find(|f| f.name() == n)
    }

    /// What is wrong, for people.
    pub fn label(self) -> &'static str {
        match self {
            Fault::MapShort => "MAP/MAF sensor shorted to ground",
            Fault::MapOpen => "MAP/MAF sensor open circuit",
            Fault::CoolantShort => "Coolant sensor shorted",
            Fault::CoolantOpen => "Coolant sensor open circuit",
            Fault::ThrottleShort => "Throttle sensor shorted",
            Fault::ThrottleOpen => "Throttle sensor open circuit",
            Fault::O2FrontOpen => "Front O2 sensor open circuit",
            Fault::O2RearOpen => "Rear O2 sensor open circuit",
            Fault::EvapShort => "EVAP pressure sensor shorted",
            Fault::EvapOpen => "EVAP pressure sensor open circuit",
            Fault::BatteryLow => "Battery sense line at 0 V",
            Fault::WheelSpeedCircuit => "Wheel speed signal line (ADC 12) open",
            Fault::CamMissing => "Cam sensor signal missing",
            Fault::CanGearboxSilent => "CAN: gearbox computer silent",
            Fault::CanEspSilent => "CAN: ESP silent",
            Fault::CanBusDead => "CAN: no other node on the bus",
            Fault::AbsWheelSensorFl => "ABS: front-left wheel sensor open",
            Fault::AbsPumpMotor => "ABS: pump motor circuit",
            Fault::SrsDriverAirbag => "Airbag: driver bag resistance high",
            Fault::SrsPassengerAirbag => "Airbag: passenger bag resistance high",
            Fault::OilTempShort => "Oil temp sensor shorted (SIMK43)",
            Fault::OilTempOpen => "Oil temp sensor open circuit (SIMK43)",
        }
    }

    /// The code the ECU stores for it (emulator-derived).
    pub fn code(self) -> &'static str {
        match self {
            Fault::MapShort => "P0107",
            Fault::MapOpen => "P0108",
            Fault::CoolantShort => "P0117",
            Fault::CoolantOpen => "P0118",
            Fault::ThrottleShort => "P0122",
            Fault::ThrottleOpen => "P0123",
            Fault::O2FrontOpen => "P0132",
            Fault::O2RearOpen => "P0138",
            Fault::EvapShort => "P0452",
            Fault::EvapOpen => "P0453",
            Fault::BatteryLow => "P0560",
            Fault::WheelSpeedCircuit => "P1502",
            Fault::CamMissing => "P0340",
            // P1602 on some dumps, P0600 on others; manual-gearbox dumps
            // store nothing [verified in the emulator]
            Fault::CanGearboxSilent | Fault::CanBusDead => "P1602/P0600",
            Fault::CanEspSilent => "–",
            Fault::AbsWheelSensorFl => "C1200",
            Fault::AbsPumpMotor => "C2402",
            Fault::SrsDriverAirbag => "B1346",
            Fault::SrsPassengerAirbag => "B1352",
            Fault::OilTempShort => "P0197",
            Fault::OilTempOpen => "P0198",
        }
    }

    /// The ADC override (channel, value) for a wiring fault on a sensor.
    pub fn adc(self, family: Family) -> Option<(usize, u16)> {
        Some(match self {
            Fault::MapShort => (0, 0),
            Fault::MapOpen => (0, 0x3FF),
            Fault::CoolantShort => (11, 0),
            Fault::CoolantOpen => (11, 0x3FF),
            Fault::ThrottleShort => (10, 0),
            Fault::ThrottleOpen => (10, 0x3FF),
            Fault::O2FrontOpen => (family.front_o2(), 0x3FF),
            Fault::OilTempShort => (6, 0),
            Fault::OilTempOpen => (6, 0x3FF),
            Fault::O2RearOpen => (3, 0x3FF),
            Fault::EvapShort => (8, 0),
            Fault::EvapOpen => (8, 0x3FF),
            Fault::BatteryLow => (4, 0),
            Fault::WheelSpeedCircuit => (12, 0x3FF),
            Fault::CamMissing
            | Fault::CanGearboxSilent
            | Fault::CanEspSilent
            | Fault::CanBusDead
            | Fault::AbsWheelSensorFl
            | Fault::AbsPumpMotor
            | Fault::SrsDriverAirbag
            | Fault::SrsPassengerAirbag => return None,
        })
    }
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
            battery_connected: true,
            faults: Default::default(),
            stop_request: false,
            family: Family::Simk41,
            intermittent: Default::default(),
            time_factor: 1.0,
            open_loop_only: false,
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
            "battery_connected" => self.battery_connected = on()?,
            "time" => self.time_factor = num(1.0, 10.0)?,
            "model" => match value {
                "auto" | "closed" => self.open_loop_only = false,
                "open" => self.open_loop_only = true,
                v => return Err(format!("model: auto or open, got {v:?}")),
            },
            "engine" if value == "stop" => self.stop_request = true,
            "engine" => return Err(format!("engine: only `stop`, got {value:?}")),
            n if n.starts_with("fault.") => {
                let f = Fault::from_name(&n[6..]).ok_or_else(|| {
                    let all: Vec<&str> = Fault::ALL.iter().map(|f| f.name()).collect();
                    format!("unknown fault {:?} ({})", &n[6..], all.join(", "))
                })?;
                self.faults.remove(&f);
                self.intermittent.remove(&f);
                if value == "intermittent" {
                    self.intermittent.insert(f);
                } else if on()? {
                    self.faults.insert(f);
                }
            }
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
    /// Closed loop: the last step used the ECU's outputs.
    pub closed_loop: bool,
    /// Closed loop state: manifold pressure (kPa), mixture (lambda), front
    /// O2 voltage (mV), and a fuel cut until this time (engine stop).
    pub map_kpa: f64,
    pub lambda: f64,
    o2_mv: f64,
    cut_until: f64,
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
            closed_loop: false,
            map_kpa: 101.0,
            lambda: 1.0,
            o2_mv: 450.0,
            cut_until: 0.0,
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
        if c.stop_request && !c.starter {
            // stalled with the key on: it stays off until cranked again
            self.engine = Engine::Off;
            self.cranked_s = 0.0;
        }
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

    /// Advance by `dt` seconds on the ECU's own outputs [assumed physics,
    /// calibrated to the GK 2.0 2003 dump: at 32 kPa its injection is 3.07
    /// ms, and torque balances friction at 830 rpm, its target idle, with
    /// its own 27 % idle air]. `fueled`: the ECU fired its injectors in the
    /// last 0.3 s.
    pub fn step_closed(
        &mut self,
        c: &Controls,
        dt: f64,
        o: &crate::ecu_view::Outputs,
        fueled: bool,
    ) {
        self.t += dt;
        self.closed_loop = true;
        // engine stop: no combustion until it is cranked again
        if c.stop_request && !c.starter {
            self.cut_until = f64::INFINITY;
        }
        if c.starter {
            self.cut_until = 0.0;
        }
        // manifold: throttle plus idle air against the engine's pumping
        let a = (c.throttle / 100.0).powf(1.5) * 25.0 + o.idle_air_pct / 100.0 * 1.43 + 0.02;
        let target = 101.0 * a / (a + self.rpm.max(0.0) / 1000.0);
        self.map_kpa += (target - self.map_kpa) * (dt / 0.08).min(1.0);
        // mixture: air from pressure, fuel from the ECU's injection time
        let fuel = o.injection_ms + 0.65;
        let burning = c.ignition && fueled && self.t >= self.cut_until && fuel > 0.7;
        self.lambda = if burning {
            (self.map_kpa / 32.7 * breathing(self.rpm)) / (fuel / 3.83)
        } else {
            9.9
        };
        let eff_mix = (1.0 - ((self.lambda - 1.0) / 0.6).powi(2)).clamp(0.0, 1.0);
        let eff_spark = (1.0 - ((o.advance_deg - 14.0) / 45.0).powi(2)).clamp(0.3, 1.0);
        let torque = if burning && self.rpm > 80.0 {
            1.4 * self.map_kpa / 32.0 * eff_mix * eff_spark
        } else {
            0.0
        };
        // friction, steeper past 6500 rpm
        let friction = if self.rpm > 0.0 {
            0.4 + 0.6 * self.rpm / 830.0 + ((self.rpm - 6500.0).max(0.0) / 200.0).powi(2)
        } else {
            0.0
        };
        self.rpm += 600.0 * (torque - friction) * dt;
        if c.ignition && c.starter && self.rpm < CRANK_RPM {
            self.rpm += (CRANK_RPM - self.rpm) * (dt / 0.2).min(1.0);
        }
        self.rpm = self.rpm.max(0.0);
        self.engine = if burning && self.rpm > 400.0 {
            Engine::Running
        } else if c.ignition && c.starter {
            Engine::Cranking
        } else {
            Engine::Off
        };
        // front O2: rich above 1 V-ish, lean below, a little lag
        let o2 = if !burning {
            450.0
        } else if self.lambda < 1.0 {
            800.0
        } else {
            100.0
        };
        self.o2_mv += (o2 - self.o2_mv) * (dt / 0.06).min(1.0);
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
        let map_kpa = if self.closed_loop {
            self.map_kpa
        } else {
            match self.engine {
                Engine::Off => 101.0,
                Engine::Cranking => 90.0,
                Engine::Running => 32.0 + c.throttle * 0.69,
            }
        };
        let warm = c.coolant_c > 40.0;
        // The SIMK43 sets P0101 when the front O2 switches on its own: its
        // trim learning expects the sensor to follow its fuel corrections
        // (closed loop, not modelled yet), so it gets a steady signal.
        let o2_front_mv = if self.closed_loop {
            self.o2_mv
        } else if running && warm && c.family == Family::Simk41 {
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
        // SIMK43: a MAF sensor instead of MAP (about 1 V standing, rising
        // with load [assumed curve]), oil temperature like the coolant
        let maf_v = if running {
            1.3 + c.throttle / 100.0 * 2.8 + plant_rpm_volts(self.rpm)
        } else {
            1.0
        };
        let first = match c.family {
            Family::Simk41 => vec![(0, adc_map(map_kpa)), (2, adc_o2(o2_front_mv))],
            Family::Simk43 => vec![
                (0, clamp10(maf_v / 5.0 * 1023.0)),
                (6, adc_ntc(c.coolant_c)),
                (7, adc_o2(o2_front_mv)),
            ],
        };
        let adc = vec![
            (3, adc_o2(o2_rear_mv)),
            (4, adc_battery(battery_v)),
            (10, adc_throttle(throttle_deg)),
            (11, adc_ntc(c.coolant_c)),
            (14, adc_ntc(c.intake_c)),
            // EVAP pressure and the wheel-speed signal line: mid-scale is
            // within range for the ECU (nothing else known about them)
            (8, 0x200),
            (12, 0x200),
        ];
        let mut adc: Vec<(usize, u16)> = first.into_iter().chain(adc).collect();
        for f in &c.faults {
            if let Some((ch, v)) = f.adc(c.family)
                && let Some(slot) = adc.iter_mut().find(|(c, _)| *c == ch)
            {
                slot.1 = v;
            }
        }
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

/// Air per cycle at constant manifold pressure, relative to 800 rpm: the
/// ECU's own fuel per cycle at 32.7 kPa across engine speed (GK 2.0 2003
/// dump, warm, trims at zero), so that its steady-state mixture is
/// stoichiometric. Below 700 rpm the ECU is not regulating idle yet.
fn breathing(rpm: f64) -> f64 {
    const PTS: [(f64, f64); 8] = [
        (400.0, 0.92),
        (500.0, 0.89),
        (650.0, 0.86),
        (800.0, 1.0),
        (1200.0, 1.03),
        (2000.0, 1.01),
        (3000.0, 1.04),
        (4500.0, 1.20),
    ];
    if rpm <= PTS[0].0 {
        return PTS[0].1;
    }
    for w in PTS.windows(2) {
        let ((r0, v0), (r1, v1)) = (w[0], w[1]);
        if rpm <= r1 {
            return v0 + (v1 - v0) * (rpm - r0) / (r1 - r0);
        }
    }
    PTS[7].1
}

/// Extra MAF voltage from engine speed (more air at higher rpm) [assumed].
fn plant_rpm_volts(rpm: f64) -> f64 {
    (rpm / 6500.0).clamp(0.0, 1.0) * 0.8
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
    fn faults_override_their_sensor() {
        let mut c = Controls::default();
        c.set("fault.coolant_open", "on").unwrap();
        let s = Plant::default().sensors(&c);
        assert!(s.adc.contains(&(11, 0x3FF)));
        c.set("fault.coolant_open", "off").unwrap();
        assert!(!Plant::default().sensors(&c).adc.contains(&(11, 0x3FF)));
        assert!(c.set("fault.nonsense", "on").is_err());
        for f in Fault::ALL {
            assert_eq!(Fault::from_name(f.name()), Some(f));
        }
    }

    #[test]
    fn engine_stops_with_the_key_on() {
        let mut p = Plant::default();
        let mut c = Controls {
            ignition: true,
            starter: true,
            ..Default::default()
        };
        for _ in 0..10 {
            p.step(&c, 0.1);
        }
        c.starter = false;
        p.step(&c, 0.1);
        assert_eq!(p.engine, Engine::Running);
        c.set("engine", "stop").unwrap();
        p.step(&c, 0.1);
        c.stop_request = false;
        for _ in 0..30 {
            p.step(&c, 0.1);
        }
        assert_eq!((p.engine, p.rpm), (Engine::Off, 0.0));
        assert!(c.ignition, "key still on");
    }

    /// A stand-in ECU: injects for the measured air (stoichiometric) and
    /// holds idle at 830 rpm with its idle air, as the dump does.
    fn ecu(p: &Plant, isc: &mut f64) -> crate::ecu_view::Outputs {
        if p.rpm > 700.0 {
            *isc = (*isc + (830.0 - p.rpm) * 0.0005).clamp(5.0, 60.0);
        }
        crate::ecu_view::Outputs {
            injection_ms: p.map_kpa / 32.7 * breathing(p.rpm) * 3.83 - 0.65,
            advance_deg: 12.0,
            idle_air_pct: *isc,
            target_idle: 830.0,
        }
    }

    #[test]
    fn closed_loop_starts_idles_and_stops() {
        let (mut p, mut isc) = (Plant::default(), 27.0);
        let mut c = Controls {
            ignition: true,
            starter: true,
            ..Default::default()
        };
        let dt = 0.002;
        for i in 0..20_000 {
            if i == 1000 {
                c.starter = false;
            }
            let o = ecu(&p, &mut isc);
            p.step_closed(&c, dt, &o, true);
        }
        assert_eq!(p.engine, Engine::Running);
        assert!((p.rpm - 830.0).abs() < 40.0, "idles near target: {}", p.rpm);
        assert!(
            (p.lambda - 1.0).abs() < 0.05,
            "stoichiometric: {}",
            p.lambda
        );
        // no fuel: it stalls
        for _ in 0..3000 {
            let o = ecu(&p, &mut isc);
            p.step_closed(&c, dt, &o, false);
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
