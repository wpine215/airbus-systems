use std::cmp::Ordering;
use std::f32::consts;
use std::time::Duration;

use uom::si::{
    area::square_meter, f32::*, force::newton, length::foot, length::meter,
    mass_density::kilogram_per_cubic_meter, pressure::pascal, pressure::psi, ratio::percent,
    thermodynamic_temperature::degree_celsius, time::second, velocity::knot, volume::cubic_inch,
    volume::gallon, volume_rate::cubic_meter_per_second, volume_rate::gallon_per_second,
};

use crate::{
    overhead::{NormalAltnPushButton, OnOffPushButton},
    shared::{Engine, UpdateContext},
    visitor::Visitable,
};

// TODO:
// - Priority valve
// - Engine fire shutoff valve
// - Leak measurement valve
// - Roll accumulator
// - PTU Rework
// - RAT pump implementation
// - Connecting electric pumps to electric sources
// - Connecting RAT pump/blue loop to emergency generator
// - Actuators
// - Bleed air sources for reservoir/line anti-cavitation

////////////////////////////////////////////////////////////////////////////////
// DATA & REFERENCES
////////////////////////////////////////////////////////////////////////////////
///
/// On A320, the reservoir level variation can, depending on the system,
/// decrease in flight by about 3.5 l (G RSVR), 4 l (Y RSVR) and 0.5 l (B RSVR)
///
/// Each MLG door open (2 total) uses 0.25 liters each of green hyd fluid
/// Each cargo door open (3 total) uses 0.2 liters each of yellow hyd fluid
///
/// Reservoirs
/// ------------------------------------------
/// Normal Qty:
/// -----------
/// Blue: 6.5L (1.7 US Gal)
/// Yellow: 12.5L (3.3 US Gal)
/// Green: 14.5L (3.8 US Gal)
///
/// Loops
/// ------------------------------------------
/// Max loop volume - green: 1.09985 gallons (double check)
///
///
/// EDP (Eaton PV3-240-10C/D/F (F is neo)):
/// ------------------------------------------
/// 37.5 GPM max (100% N2)
/// 3750 RPM
/// 3000 PSI
/// Displacement: 2.40 in3/rev
///
///
/// Electric Pump (Eaton MPEV3-032-EA2 (neo) MPEV-032-15 (ceo)):
/// ------------------------------------------
/// Uses 115/200 VAC, 400HZ electric motor
/// 8.45 GPM max
/// 7600 RPM at full displacement, 8000 RPM at no displacement
/// 3000 PSI
/// Displacement: 0.263 in3/rev
///
///
/// PTU (Eaton Vickers MPHV3-115-1C):
/// ------------------------------------------
/// 2987 PSI
///
/// Yellow to Green
/// ---------------
/// 34 GPM (130 L/min) from Yellow system
/// 24 GPM (90 L/min) to Green system
/// Maintains constant pressure near 3000PSI in green
///
/// Green to Yellow
/// ---------------
/// 16 GPM (60 L/min) from Green system
/// 13 GPM (50 L/min) to Yellow system
/// Maintains constant pressure near 3000PSI in yellow
///  
///
/// RAT PUMP (Eaton PV3-115):
/// ------------------------------------------
/// Max displacement: 1.15 in3/rev, 18.85 mL/rev
/// Normal speed: 6,600 RPM
/// Max. Ov. Speed: 8,250 RPM
/// Theoretical Flow at normal speed: 32.86 gpm, 124.4 l/m
///
///
/// Equations:
/// ------------------------------------------
/// Flow (Q), gpm:  Q = (in3/rev * rpm) / 231
/// Velocity (V), ft/s: V = (0.3208 * flow rate, gpm) / internal area, sq in
/// Force (F), lbs: F = density * area * velocity^2
/// Pressure (P), PSI: P = force / area
///
///
/// Hydraulic Fluid: EXXON HyJet IV
/// ------------------------------------------
/// Kinematic viscosity at 40C: 10.55 mm^2 s^-1, +/- 20%
/// Density at 25C: 996 kg m^-3
///
/// Hydraulic Line (HP) inner diameter
/// ------------------------------------------
/// Currently unknown. Estimating to be 7.5mm for now?
///
///
/// Actuator Force Simvars
/// -------------------------------------------
/// ACCELERATION BODY X (relative to aircraft, "east/west", Feet per second squared)
/// ACCELERATION BODY Y (relative to aircraft, vertical, Feet per second squared)
/// ACCELERATION BODY Z (relative to aircraft, "north/south", Feet per second squared)
/// ROTATION VELOCITY BODY X (feet per second)
/// ROTATION VELOCITY BODY Y (feet per second)
/// ROTATION VELOCITY BODY Z (feet per second)
/// VELOCITY BODY X (feet per second)
/// VELOCITY BODY Y (feet per second)
/// VELOCITY BODY Z (feet per second)
/// WING FLEX PCT (:1 for left, :2 for right; settable) (percent over 100)
///

////////////////////////////////////////////////////////////////////////////////
// ENUMERATIONS
////////////////////////////////////////////////////////////////////////////////

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ActuatorType {
    Aileron,
    BrakesNormal,
    BrakesAlternate,
    BrakesParking,
    CargoDoor,
    Elevator,
    EmergencyGenerator,
    EngReverser,
    Flaps,
    LandingGearNose,
    LandingGearMain,
    LandingGearDoorNose,
    LandingGearDoorMain,
    NoseWheelSteering,
    Rudder,
    Slat,
    Spoiler,
    Stabilizer,
    YawDamper,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BleedSrcType {
    None,
    Engine1,
    XBleedLine,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LoopColor {
    Blue,
    Green,
    Yellow,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PtuState {
    Off,
    GreenToYellow,
    YellowToGreen,
}

////////////////////////////////////////////////////////////////////////////////
// TRAITS
////////////////////////////////////////////////////////////////////////////////

// Trait common to all hydraulic pumps
pub trait PressureSource {
    fn get_delta_vol(&self) -> Volume;
}

////////////////////////////////////////////////////////////////////////////////
// LOOP DEFINITION - INCLUDES RESERVOIR AND ACCUMULATOR
////////////////////////////////////////////////////////////////////////////////

pub struct HydLoop {
    accumulator_pressure: Pressure,
    accumulator_volume: Volume,
    color: LoopColor,
    connected_to_ptu: bool,
    loop_length: Length,
    loop_pressure: Pressure,
    loop_volume: Volume,
    max_loop_volume: Volume,
    reservoir_volume: Volume,
}

impl HydLoop {
    const ACCUMULATOR_PRE_CHARGE: f32 = 1885.0;
    const ACCUMULATOR_MAX_VOLUME: f32 = 0.241966;
    const ACCUMULATOR_SETPOINT: f32 = 3000.; // PSI accumulator aims to achieve
    const ACCUMULATOR_SETPOINT_PSI_THRESHOLD: f32 = 0.8993; // accumulator volume to achieve setpoint
    const BASELINE_LOAD: f32 = 0.02;
    const HYDRAULIC_FLUID_DENSITY: f32 = 1000.55; // Exxon Hyjet IV, kg/m^3
    const HYDRAULIC_FLUID_KINEMATIC_VISCOSITY: f32 = 0.045; // approximate value for ~20C,  m^2/s
    const HYDRAULIC_FLUID_DYNAMIC_VISCOSITY: f32 = 45.02; // kg / (m * s)
    const PIPE_INNER_DIAMETER: f32 = 0.01; // meters
    const PIPE_CROSS_SECTION_AREA: f32 = 0.0000785; //m^2

    pub fn new(
        color: LoopColor,
        connected_to_ptu: bool,
        loop_length: Length,
        loop_volume: Volume,
        max_loop_volume: Volume,
        reservoir_volume: Volume,
    ) -> HydLoop {
        HydLoop {
            accumulator_pressure: Pressure::new::<psi>(HydLoop::ACCUMULATOR_PRE_CHARGE),
            accumulator_volume: Volume::new::<gallon>(0.),
            color,
            connected_to_ptu,
            loop_length,
            loop_pressure: Pressure::new::<psi>(0.),
            loop_volume,
            max_loop_volume,
            reservoir_volume,
        }
    }

    pub fn get_pressure(&self) -> Pressure {
        self.loop_pressure
    }

    pub fn get_reservoir_volume(&self) -> Volume {
        self.reservoir_volume
    }

    pub fn get_usable_reservoir_fluid(&self, amount: Volume) -> Volume {
        let mut drawn = amount;
        if amount > self.reservoir_volume {
            drawn = self.reservoir_volume;
        }
        drawn
    }

    pub fn update(
        &mut self,
        context: &UpdateContext,
        electric_pumps: Vec<&ElectricPump>,
        engine_driven_pumps: Vec<&EngineDrivenPump>,
        ram_air_pumps: Vec<&RatPump>,
        ptu_connected_loop: Vec<&HydLoop>,
    ) {
        let mut pressure = self.loop_pressure;
        let mut delta_vol = Volume::new::<gallon>(0.);
        let mut pump_flow_rate_sum = VolumeRate::new::<gallon_per_second>(0.);

        for p in engine_driven_pumps {
            delta_vol += p.get_delta_vol();
            pump_flow_rate_sum +=
                p.get_delta_vol() / Time::new::<second>(context.delta.as_secs_f32());
        }
        for p in electric_pumps {
            delta_vol += p.get_delta_vol();
            pump_flow_rate_sum +=
                p.get_delta_vol() / Time::new::<second>(context.delta.as_secs_f32());
        }
        for p in ram_air_pumps {
            delta_vol += p.get_delta_vol();
            pump_flow_rate_sum +=
                p.get_delta_vol() / Time::new::<second>(context.delta.as_secs_f32());
        }

        println!(
            "==> Delta vol from pumps (g): {}",
            delta_vol.get::<gallon>()
        );
        println!(
            "==> Flow rate from pumps (g/s): {}",
            pump_flow_rate_sum.get::<gallon_per_second>()
        );

        // Draw delta_vol from reservoir
        delta_vol = self.reservoir_volume.min(delta_vol);
        self.reservoir_volume -= delta_vol;

        // println!("==> Pressure before initial calculation: {}", pressure.get::<psi>());

        // Pressure supplied by engine/electric/ram-air pumps
        pressure += Pressure::new::<pascal>(
            0.5 * HydLoop::HYDRAULIC_FLUID_DENSITY
                * ((1.0 / HydLoop::PIPE_CROSS_SECTION_AREA)
                    * pump_flow_rate_sum.get::<cubic_meter_per_second>())
                .powf(2.0),
        );

        // println!("==> Pressure after initial calculation: {}", pressure.get::<psi>());

        // If PSI is low, accumulator kicks in and provides flow if available
        if pressure.get::<psi>() < HydLoop::ACCUMULATOR_SETPOINT
            && self.accumulator_volume.get::<gallon>() > 0.
        {
            // TODO: implement accumulator pressure logic (Boyle's law / PV=nRT)
            // TODO: add output pressure to `pressure`
            // TODO: add output volume to `delta_vol` & subtract from `self.accumulator_volume`
        }

        // If PSI is high, accumulator kicks in and receives excess flow if able
        if pressure.get::<psi>() > HydLoop::ACCUMULATOR_SETPOINT
            && self.accumulator_volume.get::<gallon>() < HydLoop::ACCUMULATOR_MAX_VOLUME
        {
            // TODO: implement accumulator pressure logic (Boyle's law / PV=nRT)
            // TODO: subtract input pressure from `pressure`
            // TODO: subtract input volume from `delta_vol` & add to `self.accumulator_volume`
        }

        // TODO: Check if PTU isn't off or failed first, and other valid conditions
        if self.connected_to_ptu && ptu_connected_loop.len() > 0 {
            // Our pressure is >=500 PSI less than other loop, so PTU will act as a pump
            if ptu_connected_loop[0].loop_pressure.get::<psi>() >= pressure.get::<psi>() + 500.0 {}
            // Our pressure is >=500 PSI greater than other loop, so PTU will act as a motor
            if ptu_connected_loop[0].loop_pressure.get::<psi>() <= pressure.get::<psi>() - 500.0 {}
        }

        // If `self.loop_volume` is less than `self.max_loop_volume`, draw from `delta_vol` to fill loop
        if self.loop_volume < self.max_loop_volume {
            let difference = self.max_loop_volume - self.loop_volume;
            let delta_loop_vol = delta_vol.min(difference);
            delta_vol -= delta_loop_vol;
            self.loop_volume += delta_loop_vol;
        }

        // If `pressure` is still low, then draw from `self.loop_volume`
        if pressure.get::<psi>() <= 14.5 && self.loop_volume.get::<gallon>() > 0. {
            let max_delta_loop_vol = VolumeRate::new::<gallon_per_second>(0.5)
                * Time::new::<second>(context.delta.as_secs_f32());
            let delta_loop_vol = self.loop_volume.min(max_delta_loop_vol);
            self.loop_volume -= delta_loop_vol;
            delta_vol += delta_loop_vol;
        }

        // // Pressure drop-off due to pipe length (laminar flow)
        // let second_term = (HydLoop::HYDRAULIC_FLUID_DYNAMIC_VISCOSITY
        //     * pump_flow_rate_sum.get::<cubic_meter_per_second>())
        //     / HydLoop::PIPE_INNER_DIAMETER.powf(4.0);
        // let pressure_loss_per_meter = Pressure::new::<pascal>(128.0 / consts::PI) * second_term;
        // let total_pressure_loss = Pressure::new::<psi>(
        //     pressure_loss_per_meter.get::<psi>() * self.loop_length.get::<meter>(),
        // );
        // pressure -= total_pressure_loss;

        // Second attempt:
        // let pipe_backpressure = Pressure::new::<psi>(350.0 * context.delta.as_secs_f32());
        // let delta_p = pressure.min(pipe_backpressure);
        // pressure -= delta_p;

        // Third attempt:
        let current_flow_rate = VolumeRate::new::<cubic_meter_per_second>(
            HydLoop::PIPE_CROSS_SECTION_AREA
                * ((2.0 * pressure.get::<pascal>()) / HydLoop::HYDRAULIC_FLUID_DENSITY).powf(0.5),
        );
        let pressure_loss =
            Pressure::new::<psi>(0.25 * current_flow_rate.get::<gallon_per_second>().powf(2.5));
        println!(
            "==> Current flow rate: {}",
            current_flow_rate.get::<gallon_per_second>()
        );
        println!(
            "==> Current pressure loss: {} ({}%)",
            pressure_loss.get::<psi>(),
            pressure_loss.get::<psi>() / pressure.get::<psi>() * 100.0
        );
        pressure -= pressure.min(pressure_loss);

        // TODO: implement actuator (landing gear & cargo door) volume usage (both input and output) logic

        // TODO: implement pressure decrement from actuator usage
        // For each actuator, subtract its pressure (force * area) from `pressure`

        // If delta_vol is greater than reservoir volume, we screwed up somewhere
        self.reservoir_volume += delta_vol;

        // Update pressure
        self.loop_pressure = pressure;
    }
}

////////////////////////////////////////////////////////////////////////////////
// PUMP DEFINITION
////////////////////////////////////////////////////////////////////////////////

pub struct Pump {
    max_displacement: Volume,
    reservoir_fluid_used: Volume,
    delta_vol: Volume,
}
impl Pump {
    fn new(max_displacement: Volume) -> Pump {
        Pump {
            max_displacement,
            reservoir_fluid_used: Volume::new::<gallon>(0.),
            delta_vol: Volume::new::<gallon>(0.),
        }
    }

    fn update(&mut self, context: &UpdateContext, line: &HydLoop, rpm: f32) {
        let displacement = Pump::calculate_displacement(line.get_pressure(), self.max_displacement);

        let flow = Pump::calculate_flow(rpm, displacement);
        let delta_vol = flow * Time::new::<second>(context.delta.as_secs_f32());

        let amount_drawn = line.get_usable_reservoir_fluid(delta_vol);
        self.reservoir_fluid_used = amount_drawn;
        self.delta_vol = delta_vol.min(amount_drawn);
    }

    fn calculate_displacement(pressure: Pressure, max_displacement: Volume) -> Volume {
        let numerator_term = -1. * max_displacement.get::<cubic_inch>();
        let exponent_term = -0.25 * (pressure.get::<psi>() - 2990.0);
        let denominator_term = (1. + consts::E.powf(exponent_term)).powf(0.04);

        Volume::new::<cubic_inch>(
            numerator_term / denominator_term + max_displacement.get::<cubic_inch>(),
        )
    }

    fn calculate_flow(rpm: f32, displacement: Volume) -> VolumeRate {
        VolumeRate::new::<gallon_per_second>(rpm * displacement.get::<cubic_inch>() / 231.0 / 60.0)
    }
}
impl PressureSource for Pump {
    fn get_delta_vol(&self) -> Volume {
        self.delta_vol
    }
}

pub struct ElectricPump {
    active: bool,
    rpm: f32,
    pump: Pump,
}
impl ElectricPump {
    const SPOOLUP_TIME: f32 = 4.0;
    const SPOOLDOWN_TIME: f32 = 8.0;
    const MAX_DISPLACEMENT: f32 = 0.263;

    pub fn new() -> ElectricPump {
        ElectricPump {
            active: false,
            rpm: 0.,
            pump: Pump::new(Volume::new::<cubic_inch>(ElectricPump::MAX_DISPLACEMENT)),
        }
    }

    pub fn start(&mut self) {
        self.active = true;
    }

    pub fn stop(&mut self) {
        self.active = false;
    }

    pub fn update(&mut self, context: &UpdateContext, line: &HydLoop) {
        // Pump startup/shutdown process
        if self.active && self.rpm < 7600.0 {
            self.rpm += 7600.0f32
                .min((7600. / ElectricPump::SPOOLUP_TIME) * (context.delta.as_secs_f32() * 10.));
        } else if !self.active && self.rpm > 0.0 {
            self.rpm -= 7600.0f32
                .min((7600. / ElectricPump::SPOOLDOWN_TIME) * (context.delta.as_secs_f32() * 10.));
        }

        self.pump.update(context, line, self.rpm);
    }
}
impl PressureSource for ElectricPump {
    fn get_delta_vol(&self) -> Volume {
        self.pump.get_delta_vol()
    }
}

pub struct EngineDrivenPump {
    active: bool,
    pump: Pump,
}
impl EngineDrivenPump {
    const LEAP_1A26_MAX_N2_RPM: f32 = 16645.0;
    const MAX_DISPLACEMENT: f32 = 2.4;
    const MAX_RPM: f32 = 4000.;

    pub fn new() -> EngineDrivenPump {
        EngineDrivenPump {
            active: false,
            pump: Pump::new(Volume::new::<cubic_inch>(
                EngineDrivenPump::MAX_DISPLACEMENT,
            )),
        }
    }

    pub fn update(&mut self, context: &UpdateContext, line: &HydLoop, engine: &Engine) {
        let rpm = (1.0f32.min(4.0 * engine.n2.get::<percent>())) * EngineDrivenPump::MAX_RPM;

        self.pump.update(context, line, rpm);
    }
}
impl PressureSource for EngineDrivenPump {
    fn get_delta_vol(&self) -> Volume {
        self.pump.get_delta_vol()
    }
}

pub struct RatPump {
    active: bool,
    pump: Pump,
}
impl RatPump {
    const MAX_DISPLACEMENT: f32 = 1.15;
    const NORMAL_RPM: f32 = 6000.;

    pub fn new() -> RatPump {
        RatPump {
            active: false,
            pump: Pump::new(Volume::new::<cubic_inch>(RatPump::MAX_DISPLACEMENT)),
        }
    }

    pub fn update(&mut self, context: &UpdateContext, line: &HydLoop) {
        self.pump.update(context, line, RatPump::NORMAL_RPM);
    }
}
impl PressureSource for RatPump {
    fn get_delta_vol(&self) -> Volume {
        self.pump.get_delta_vol()
    }
}

////////////////////////////////////////////////////////////////////////////////
// ACTUATOR DEFINITION
////////////////////////////////////////////////////////////////////////////////

// pub struct Actuator {
//     a_type: ActuatorType,
//     active: bool,
//     affected_by_gravity: bool,
//     area: Area,
//     line: HydLoop,
//     neutral_is_zero: bool,
//     stall_load: Force,
//     volume_used_at_max_deflection: Volume,
// }

// // TODO
// impl Actuator {
//     pub fn new(a_type: ActuatorType, line: HydLoop) -> Actuator {
//         Actuator { a_type, line }
//     }
// }

////////////////////////////////////////////////////////////////////////////////
// BLEED AIR SRC DEFINITION
////////////////////////////////////////////////////////////////////////////////

pub struct BleedAir {
    b_type: BleedSrcType,
}

impl BleedAir {
    pub fn new(b_type: BleedSrcType) -> BleedAir {
        BleedAir { b_type }
    }
}

////////////////////////////////////////////////////////////////////////////////
// TESTS
////////////////////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn green_loop_edp_simulation() {
        let mut edp1 = engine_driven_pump();
        let mut green_loop = hydraulic_loop(LoopColor::Green);
        edp1.active = true;

        let init_n2 = Ratio::new::<percent>(0.5);
        let mut engine1 = engine(init_n2);
        let ct = context(Duration::from_millis(50));
        for x in 0..60 {
            if x == 200 {
                engine1.n2 = Ratio::new::<percent>(0.0);
            }
            println!("Iteration {}", x);
            edp1.update(&ct, &green_loop, &engine1);
            green_loop.update(&ct, Vec::new(), vec![&edp1], Vec::new(), Vec::new());
            if x % 1 == 0 {
                // println!("Iteration {}", x);
                println!("-------------------------------------------");
                println!("---PSI: {}", green_loop.loop_pressure.get::<psi>());
                println!(
                    "--------Reservoir Volume (g): {}",
                    green_loop.reservoir_volume.get::<gallon>()
                );
                println!(
                    "--------Loop Volume (g): {}",
                    green_loop.loop_volume.get::<gallon>()
                );
                println!(
                    "--------Acc Volume (g): {}",
                    green_loop.accumulator_volume.get::<gallon>()
                );
            }
        }

        assert!(true)
    }

    #[test]
    fn yellow_loop_epump_simulation() {
        let mut epump = electric_pump();
        let mut yellow_loop = hydraulic_loop(LoopColor::Yellow);
        epump.active = true;

        let ct = context(Duration::from_millis(50));
        for x in 0..800 {
            if x == 400 {
                epump.active = false;
            }
            epump.update(&ct, &yellow_loop);
            yellow_loop.update(&ct, vec![&epump], Vec::new(), Vec::new(), Vec::new());
            if x % 20 == 0 {
                println!("Iteration {}", x);
                println!("-------------------------------------------");
                println!("---PSI: {}", yellow_loop.loop_pressure.get::<psi>());
                println!("---RPM: {}", epump.rpm);
                println!(
                    "--------Reservoir Volume (g): {}",
                    yellow_loop.reservoir_volume.get::<gallon>()
                );
                println!(
                    "--------Loop Volume (g): {}",
                    yellow_loop.loop_volume.get::<gallon>()
                );
                println!(
                    "--------Acc Volume (g): {}",
                    yellow_loop.accumulator_volume.get::<gallon>()
                );
            }
        }

        assert!(true)
    }

    fn hydraulic_loop(loop_color: LoopColor) -> HydLoop {
        HydLoop::new(
            loop_color,
            false,
            Length::new::<meter>(10.),
            Volume::new::<gallon>(1.),
            Volume::new::<gallon>(1.09985),
            Volume::new::<gallon>(3.7),
        )
    }

    fn electric_pump() -> ElectricPump {
        ElectricPump::new()
    }

    fn engine_driven_pump() -> EngineDrivenPump {
        EngineDrivenPump::new()
    }

    fn engine(n2: Ratio) -> Engine {
        let mut engine = Engine::new();
        engine.n2 = n2;

        engine
    }

    fn context(delta_time: Duration) -> UpdateContext {
        UpdateContext::new(
            delta_time,
            Velocity::new::<knot>(250.),
            Length::new::<foot>(5000.),
            ThermodynamicTemperature::new::<degree_celsius>(25.0),
        )
    }

    #[cfg(test)]
    mod loop_tests {}

    #[cfg(test)]
    mod epump_tests {}

    #[cfg(test)]
    mod edp_tests {
        use super::*;
        use uom::si::ratio::percent;

        #[test]
        fn starts_inactive() {
            assert!(engine_driven_pump().active == false);
        }

        #[test]
        fn max_flow_under_2500_psi_after_100ms() {
            let n2 = Ratio::new::<percent>(0.6);
            let pressure = Pressure::new::<psi>(2000.);
            let time = Duration::from_millis(100);
            let displacement = Volume::new::<cubic_inch>(EngineDrivenPump::MAX_DISPLACEMENT);
            assert!(delta_vol_equality_check(n2, displacement, pressure, time))
        }

        #[test]
        fn zero_flow_above_3000_psi_after_25ms() {
            let n2 = Ratio::new::<percent>(0.6);
            let pressure = Pressure::new::<psi>(3100.);
            let time = Duration::from_millis(25);
            let displacement = Volume::new::<cubic_inch>(0.);
            assert!(delta_vol_equality_check(n2, displacement, pressure, time))
        }

        fn delta_vol_equality_check(
            n2: Ratio,
            displacement: Volume,
            pressure: Pressure,
            time: Duration,
        ) -> bool {
            let actual = get_edp_actual_delta_vol_when(n2, pressure, time);
            let predicted = get_edp_predicted_delta_vol_when(n2, displacement, time);
            println!("Actual: {}", actual.get::<gallon>());
            println!("Predicted: {}", predicted.get::<gallon>());
            actual == predicted
        }

        fn get_edp_actual_delta_vol_when(n2: Ratio, pressure: Pressure, time: Duration) -> Volume {
            let eng = engine(n2);
            let mut edp = engine_driven_pump();
            let mut line = hydraulic_loop(LoopColor::Green);
            line.loop_pressure = pressure;
            edp.update(&context(time), &line, &eng);
            edp.get_delta_vol()
        }

        fn get_edp_predicted_delta_vol_when(
            n2: Ratio,
            displacement: Volume,
            time: Duration,
        ) -> Volume {
            let edp_rpm = (1.0f32.min(4.0 * n2.get::<percent>())) * EngineDrivenPump::MAX_RPM;
            let expected_flow = Pump::calculate_flow(edp_rpm, displacement);
            expected_flow * Time::new::<second>(time.as_secs_f32())
        }
    }
}
