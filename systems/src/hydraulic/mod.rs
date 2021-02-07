use std::{borrow::Borrow, cmp::Ordering};
use std::f64::consts;
use std::time::Duration;

//use uom::{si::{area::square_meter, f64::*, force::newton, length::foot, length::meter, mass_density::kilogram_per_cubic_meter, pressure::atmosphere, pressure::pascal, pressure::psi, ratio::percent, thermodynamic_temperature::{self, degree_celsius}, time::second, velocity::knot, volume::cubic_inch, volume::gallon, volume::liter, volume_rate::cubic_meter_per_second, volume_rate::{VolumeRate, gallon_per_second}}, typenum::private::IsLessOrEqualPrivate};
//use uom::si::f64::*;
use uom::{si::{area::square_meter, f64::*, force::newton, length::foot, length::meter, mass_density::kilogram_per_cubic_meter, pressure::atmosphere, pressure::pascal, pressure::psi, ratio::percent, thermodynamic_temperature::{self, degree_celsius}, time::second, velocity::knot, volume::cubic_inch, volume::gallon, volume::liter, volume_rate::cubic_meter_per_second, volume_rate::gallon_per_second}, typenum::private::IsLessOrEqualPrivate};

use crate::{
    overhead::{NormalAltnPushButton, OnOffPushButton},
    shared::Engine,
    simulator::UpdateContext,
};

//Interpolate values_map_y at point value_at_point in breakpoints break_points_x
fn interpolation(xs: &[f64], ys: &[f64], intermediate_x: f64) -> f64 {
    debug_assert!(xs.len() == ys.len());
    debug_assert!(xs.len() >= 2);
    debug_assert!(ys.len() >= 2);
    // The function also assumes xs are ordered from small to large. Consider adding a debug_assert! for that as well.
   
    if intermediate_x <= xs[0] {
        *ys.first().unwrap()
    } else if intermediate_x >= xs[xs.len()-1] {
        *ys.last().unwrap()
    } else {
        let mut idx:usize =1;
        println!("---INTERP {}", idx);
        while idx < xs.len()-1 {
            if intermediate_x < xs[idx] {
               break;
               println!("---INTERP BREAK idx{}", idx);
            }
            idx += 1;
        }
        println!("---INTERP FINAL IDX {}", idx);
        println!("---INTERP FINAL VAL {}", ys[idx-1] + (intermediate_x - xs[idx-1]) / (xs[idx] - xs[idx-1]) * (ys[idx] - ys[idx-1]));
        ys[idx-1] + (intermediate_x - xs[idx-1]) / (xs[idx] - xs[idx-1]) * (ys[idx] - ys[idx-1])
        
    }
}

// TODO:
// - Priority valve
// - Engine fire shutoff valve
// - Leak measurement valve
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
/// PressureDelta = VolumeDelta / Total_uncompressed_volume * Fluid_Bulk_Modulus
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
// Max gives maximum available volume at that time as if it is a variable displacement
// pump it can be adjusted by pump regulation
// Min will give minimum volume that will be outputed no matter what. example if there is a minimal displacement or 
// a fixed displacement (ie. elec pump)
pub trait PressureSource {
    fn get_delta_vol_max(&self) -> Volume;
    fn get_delta_vol_min(&self) -> Volume;
}

////////////////////////////////////////////////////////////////////////////////
// LOOP DEFINITION - INCLUDES RESERVOIR AND ACCUMULATOR
////////////////////////////////////////////////////////////////////////////////

//Implements fluid structure.
//TODO update method that can update physic constants from given temperature
//This would change pressure response to volume
pub struct HydFluid {
    //temp : thermodynamic_temperature,
    current_bulk : Pressure,
}

impl HydFluid {
    pub fn new ( bulk : Pressure) -> HydFluid {
        HydFluid{
            //temp:temp,
            current_bulk:bulk,
        }
    }

    pub fn get_bulk_mod (&self) -> Pressure {
        return self.current_bulk;
    }
}

//Power Transfer Unit
//TODO enhance simulation with RPM and variable displacement on one side?
pub struct Ptu {
    isEnabled : bool,
    isActiveRight : bool,
    isActiveLeft : bool,
    flow_to_right : VolumeRate,
    flow_to_left : VolumeRate,
}

impl Ptu {
    pub fn update(&mut self,loopLeft : &HydLoop, loopRight: &HydLoop){
        if self.isEnabled {
            let deltaP=loopLeft.get_pressure() - loopRight.get_pressure();
            
            //TODO: use maped characteristics for PTU? 
            //TODO Use variable displacement available on one side?
            //TODO Handle RPM of ptu so transient are bit slower?
            //TODO Handle it as a min/max flow producer using PressureSource trait?
            if self.isActiveLeft || deltaP.get::<psi>()  > 500.0 {//Left sends flow to right
                let vr = 34.0f64.min(loopLeft.loop_pressure.get::<psi>() * 0.01133) / 60.0;
                self.flow_to_left= VolumeRate::new::<gallon_per_second>(-vr);
                self.flow_to_right= VolumeRate::new::<gallon_per_second>(vr * 0.7059);
                //right uses vr , gives to left vr * 0.7059
                self.isActiveLeft=true;
            } else if self.isActiveRight || deltaP.get::<psi>()  < -500.0 {//Right sends flow to left
                let vr = 16.0f64.min(loopRight.loop_pressure.get::<psi>() * 0.005333) / 60.0;
                self.flow_to_left = VolumeRate::new::<gallon_per_second>(vr * 0.8125);
                self.flow_to_right= VolumeRate::new::<gallon_per_second>(-vr);
                //left uses vr, gives vr * 0.8125 to right
                self.isActiveRight=true;
            }
            
            if self.isActiveRight && loopLeft.loop_pressure.get::<psi>()  > 2950.0 || self.isActiveLeft && loopRight.loop_pressure.get::<psi>() > 2950.0 {
                self.flow_to_left=VolumeRate::new::<gallon_per_second>(0.0);
                self.flow_to_right=VolumeRate::new::<gallon_per_second>(0.0);
                self.isActiveRight=false;
                self.isActiveLeft=false;
            }       
        }       
    }

    pub fn enabling (&mut self , enable_flag:bool){
        self.isEnabled = enable_flag;
    }
}

pub struct HydLoop {
    fluid: HydFluid,
    accumulator_gas_pressure: Pressure,
    accumulator_gas_volume: Volume,
    accumulator_fluid_volume: Volume,
    accumulator_press_breakpoints:[f64; 9] ,
    accumulator_flow_carac:[f64; 9] ,
    color: LoopColor,
    connected_to_ptu: bool,
    loop_pressure: Pressure,
    loop_volume: Volume,
    max_loop_volume: Volume,
    high_pressure_volume : Volume,
    ptu_active: bool,
    reservoir_volume: Volume,
    current_delta_vol: Volume,
    current_flow: VolumeRate,
}

impl HydLoop {
    const ACCUMULATOR_GAS_PRE_CHARGE: f64 =1885.0; // Nitrogen PSI
    const ACCUMULATOR_MAX_VOLUME: f64  =0.264; // in gallons
    const HYDRAULIC_FLUID_DENSITY: f64 = 1000.55; // Exxon Hyjet IV, kg/m^3
    const ACCUMULATOR_PRESS_BREAKPTS: [f64; 9] = [
        0.0 ,5.0 , 10.0 ,50.0 ,100.0 ,200.0 ,500.0 ,1000.0 , 10000.0
    ];
    const ACCUMULATOR_FLOW_CARAC: [f64; 9] = [
        0.0,0.005, 0.008, 0.01, 0.02, 0.08,  0.15,   0.35 ,   0.5
    ];

    pub fn new(
        color: LoopColor,
        connected_to_ptu: bool,
        loop_volume: Volume,
        max_loop_volume: Volume,
        high_pressure_volume: Volume,
        reservoir_volume: Volume,
        fluid:HydFluid,
    ) -> HydLoop {
        HydLoop {
            accumulator_gas_pressure: Pressure::new::<psi>(HydLoop::ACCUMULATOR_GAS_PRE_CHARGE),
            accumulator_gas_volume: Volume::new::<gallon>(HydLoop::ACCUMULATOR_MAX_VOLUME),
            accumulator_fluid_volume: Volume::new::<gallon>(0.),
            color,
            connected_to_ptu,
            loop_pressure: Pressure::new::<psi>(1.),
            loop_volume,
            max_loop_volume,
            high_pressure_volume,
            ptu_active: false,
            reservoir_volume,
            fluid,
            current_delta_vol: Volume::new::<gallon>(0.),
            current_flow: VolumeRate::new::<gallon_per_second>(0.),
            accumulator_press_breakpoints:HydLoop::ACCUMULATOR_PRESS_BREAKPTS,
            accumulator_flow_carac:HydLoop::ACCUMULATOR_FLOW_CARAC,
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

    //Method to update pressure of a loop. The more delta volume is added, the more pressure rises
    //Directly from bulk modulus equation
    pub fn delta_pressure_from_delta_volume(&self, delta_vol: Volume) -> Pressure {
            return delta_vol / self.high_pressure_volume * self.fluid.get_bulk_mod();      
    }
    
    //Gives the exact volume of fluid needed to get to any target_press pressure
    pub fn vol_to_target(&self,target_press : Pressure) -> Volume {
        (target_press-self.loop_pressure) * (self.high_pressure_volume) / self.fluid.get_bulk_mod()
    }


    pub fn update(
        &mut self,
        delta_time : &Duration,
        context: &UpdateContext,
        electric_pumps: Vec<&ElectricPump>,
        engine_driven_pumps: Vec<&EngineDrivenPump>,
        ram_air_pumps: Vec<&RatPump>,
        ptu_connected_loop: Vec<&HydLoop>,
    ) {
        let mut pressure = self.loop_pressure;
        let mut delta_vol_max = Volume::new::<gallon>(0.);
        let mut delta_vol_min = Volume::new::<gallon>(0.);
        let mut reservoir_return =Volume::new::<gallon>(0.);
        let mut delta_vol = Volume::new::<gallon>(0.);
      
        for p in engine_driven_pumps {
            delta_vol_max += p.get_delta_vol_max();
            delta_vol_min += p.get_delta_vol_min();
        }
        for p in electric_pumps {
            delta_vol_max += p.get_delta_vol_max();
            delta_vol_min += p.get_delta_vol_min(); 
        }
        for p in ram_air_pumps {
            delta_vol_max += p.get_delta_vol_max();
            delta_vol_min += p.get_delta_vol_min();
        }
        println!("----------START------");
        println!("---Current Press {}", pressure.get::<psi>());
        println!("---DELTA volMax {}", delta_vol_max.get::<gallon>());
        //Static leaks
        //TODO: separate static leaks per zone of high pressure or actuator
        let static_leaks_vol = Volume::new::<gallon>(0.08 * delta_time.as_secs_f64() * self.loop_pressure.get::<psi>() / 3000.0);
        println!("---Leaks vol {}", static_leaks_vol.get::<gallon>());
        // Draw delta_vol from reservoir
        delta_vol -= static_leaks_vol;
        reservoir_return += static_leaks_vol;

        //TODO PTU
        //END PTU

        //Priming the loop if not filled in
        if self.loop_volume < self.max_loop_volume { //} %TODO what to do if we are back under max volume and unprime the loop?
            let difference =  self.max_loop_volume  - self.loop_volume;
            println!("---Priming diff {}", difference.get::<gallon>());
            let availableFluidVol=self.reservoir_volume.min(delta_vol_max);
            let delta_loop_vol = availableFluidVol.min(difference);
            delta_vol_max -= delta_loop_vol;//%TODO check if we cross the deltaVolMin?
            self.loop_volume+= delta_loop_vol;
            self.reservoir_volume -= delta_loop_vol;
            println!("---Priming vol {} / {}", self.loop_volume.get::<gallon>(),self.max_loop_volume.get::<gallon>());
        } else {
            println!("---Primed {}", self.loop_volume.get::<gallon>());
        }
        //end priming


        //ACCUMULATOR
        let accumulatorDeltaPress = self.accumulator_gas_pressure - self.loop_pressure;
        let flowVariation = VolumeRate::new::<gallon_per_second>(interpolation(&self.accumulator_press_breakpoints,&self.accumulator_flow_carac,accumulatorDeltaPress.get::<psi>().abs())); 

        //TODO HANDLE OR CHECK IF RESERVOIR AVAILABILITY is OK
        //TODO check if accumulator can be used as a min/max flow producer to
        //avoid it being a consumer that might unsettle pressure
        if  accumulatorDeltaPress.get::<psi>() > 0.0  {
            let volumeFromAcc = self.accumulator_fluid_volume.min(flowVariation * Time::new::<second>(delta_time.as_secs_f64()));
            self.accumulator_fluid_volume -= volumeFromAcc;
            self.accumulator_gas_volume += volumeFromAcc;  
            delta_vol += volumeFromAcc;
        } else {
            let volumeToAcc = delta_vol.max(Volume::new::<gallon>(0.0)).max(flowVariation * Time::new::<second>(delta_time.as_secs_f64()));
            self.accumulator_fluid_volume += volumeToAcc;
            self.accumulator_gas_volume -= volumeToAcc;
            delta_vol -= volumeToAcc;
        }
    
        self.accumulator_gas_pressure = (Pressure::new::<psi>(HydLoop::ACCUMULATOR_GAS_PRE_CHARGE) * Volume::new::<gallon>(HydLoop::ACCUMULATOR_MAX_VOLUME)) / (Volume::new::<gallon>(HydLoop::ACCUMULATOR_MAX_VOLUME) - self.accumulator_fluid_volume);     
        //END ACCUMULATOR



        //Actuators
        let used_fluidQty= Volume::new::<gallon>(0.); // %%total fluid used
        //foreach actuator
            //used_fluidQty =used_fluidQty+aileron.volumeToActuatorAccumulated*264.172; %264.172 is m^3 to gallons
            //reservoirReturn=reservoirReturn+aileron.volumeToResAccumulated*264.172;
            //actuator.resetVolumes()
            //actuator.set_available_pressure(self.loop_pressure)
         //end foreach
        //end actuator

        delta_vol -= used_fluidQty;
        
        
        //How much we need to reach target of 3000?
        let mut volume_needed_to_reach_pressure_target = self.vol_to_target(Pressure::new::<psi>(3000.0));
        println!("---needed {}", volume_needed_to_reach_pressure_target.get::<gallon>());
        //Actually we need this PLUS what is used by consumers.
        volume_needed_to_reach_pressure_target -= delta_vol;
        println!("---neededFinal {}", volume_needed_to_reach_pressure_target.get::<gallon>());
 
        //Now computing what we will actually use from flow providers limited by
        //their min and max flows and reservoir availability
        let actual_volume_added_to_pressurise = self.reservoir_volume.min(delta_vol_min.max(delta_vol_max.min(volume_needed_to_reach_pressure_target)));     
        println!("---actual vol added {}", actual_volume_added_to_pressurise.get::<gallon>());
        delta_vol+=actual_volume_added_to_pressurise;
        println!("---final delta vol {}", delta_vol.get::<gallon>());

        //Loop Pressure update From Bulk modulus
        let pressDelta = self.delta_pressure_from_delta_volume(delta_vol);
        println!("---Press delta {}", pressDelta.get::<psi>());
        self.loop_pressure += pressDelta;
        println!("---Final press {}", self.loop_pressure.get::<psi>());


        //Update reservoir
        self.reservoir_volume -= actual_volume_added_to_pressurise; //%limit to 0 min? for case of negative added?
        self.reservoir_volume += reservoir_return;
        println!("---Reservoir vol {}", self.reservoir_volume.get::<gallon>());
        //Update Volumes
        self.loop_volume += delta_vol;
        println!("---Total vol {} / {}", self.loop_volume.get::<gallon>(),self.max_loop_volume.get::<gallon>());

        self.current_delta_vol=delta_vol;
        self.current_flow=delta_vol / Time::new::<second>(delta_time.as_secs_f64());
        println!("---Final flow {}", self.current_flow.get::<gallon_per_second>());
        println!("---------END-------");
    }
}

////////////////////////////////////////////////////////////////////////////////
// PUMP DEFINITION
////////////////////////////////////////////////////////////////////////////////

pub struct Pump {
    //max_displacement: Volume,
    //reservoir_fluid_used: Volume,
    delta_vol_max: Volume,
    delta_vol_min: Volume,
    pressBreakpoints:[f64; 9] ,
    displacementCarac:[f64; 9] ,
}
impl Pump {
    fn new(pressBreakpoints:[f64; 9],displacementCarac:[f64; 9]) -> Pump {
        Pump {
            delta_vol_max: Volume::new::<gallon>(0.),
            delta_vol_min: Volume::new::<gallon>(0.),
            pressBreakpoints:pressBreakpoints,
            displacementCarac:displacementCarac,
        }
    }

    fn update(&mut self, delta_time: &Duration,context: &UpdateContext, line: &HydLoop, rpm: f64) {
        let displacement = self.calculate_displacement(line.get_pressure());

        let flow = Pump::calculate_flow(rpm, displacement);       
        
        self.delta_vol_max=flow * Time::new::<second>(delta_time.as_secs_f64());
        self.delta_vol_min=Volume::new::<gallon>(0.0); 
    }

    fn calculate_displacement(&self , pressure: Pressure) -> Volume {
        Volume::new::<cubic_inch>(interpolation(&self.pressBreakpoints,&self.displacementCarac,pressure.get::<psi>()))
    }

    fn calculate_flow(rpm: f64, displacement: Volume) -> VolumeRate {
        VolumeRate::new::<gallon_per_second>(rpm * displacement.get::<cubic_inch>() / 231.0 / 60.0)
    }
}
impl PressureSource for Pump {
    fn get_delta_vol_max(&self) -> Volume {
        self.delta_vol_max
    }

    fn get_delta_vol_min(&self) -> Volume {
        self.delta_vol_min
    }
}

pub struct ElectricPump {
    active: bool,
    rpm: f64,
    pump: Pump,
}
impl ElectricPump {
    const SPOOLUP_TIME: f64 = 4.0;
    const SPOOLDOWN_TIME: f64 = 8.0;
    const DISPLACEMENT_BREAKPTS: [f64; 9] = [
        0.0, 500.0, 1000.0, 1500.0, 2800.0, 2900.0, 3000.0, 3050.0, 3500.0,
    ];
    const DISPLACEMENT_MAP: [f64; 9] = [
        0.263,0.263,0.263,  0.263 , 0.263,  0.263 , 0.163,  0.0 ,   0.0 
    ];

    pub fn new() -> ElectricPump {
        ElectricPump {
            active: false,
            rpm: 0.,
            pump: Pump::new(ElectricPump::DISPLACEMENT_BREAKPTS,ElectricPump::DISPLACEMENT_MAP),
        }
    }

    pub fn start(&mut self) {
        self.active = true;
    }

    pub fn stop(&mut self) {
        self.active = false;
    }

    pub fn update(&mut self,delta_time: &Duration, context: &UpdateContext, line: &HydLoop) {
        // Pump startup/shutdown process
        if self.active && self.rpm < 7600.0 {
            self.rpm += 7600.0f64
                .min((7600. / ElectricPump::SPOOLUP_TIME) * (delta_time.as_secs_f64() * 10.));
        } else if !self.active && self.rpm > 0.0 {
            self.rpm -= 7600.0f64
                .min((7600. / ElectricPump::SPOOLDOWN_TIME) * (delta_time.as_secs_f64() * 10.));
        }

        self.pump.update(delta_time, context, line, self.rpm);
    }
}
impl PressureSource for ElectricPump {
    fn get_delta_vol_max(&self) -> Volume {
        self.pump.get_delta_vol_max()
    }
    fn get_delta_vol_min(&self) -> Volume {
        self.pump.get_delta_vol_min()
    }
}

pub struct EngineDrivenPump {
    active: bool,
    pump: Pump,
}
impl EngineDrivenPump {
    const LEAP_1A26_MAX_N2_RPM: f64 = 16645.0;
    const DISPLACEMENT_BREAKPTS: [f64; 9] = [
        0.0, 500.0, 1000.0, 1500.0, 2800.0, 2900.0, 3000.0, 3050.0, 3500.0,
    ];
    const DISPLACEMENT_MAP: [f64; 9] = [
        2.4 ,2.4,   2.4,    2.4 ,   2.4,    2.4 ,   2.0,    0.0 ,   0.0 ];
    const MAX_RPM: f64 = 4000.;

    pub fn new() -> EngineDrivenPump {
        EngineDrivenPump {
            active: false,
            pump: Pump::new(EngineDrivenPump::DISPLACEMENT_BREAKPTS,
                EngineDrivenPump::DISPLACEMENT_MAP,
            ),
        }
    }

    pub fn update(&mut self, delta_time : &Duration,context: &UpdateContext, line: &HydLoop, engine: &Engine) {
        let rpm = (1.0f64.min(4.0 * engine.n2.get::<percent>())) * EngineDrivenPump::MAX_RPM;

        self.pump.update(delta_time,context, line, rpm);
    }
}
impl PressureSource for EngineDrivenPump {
    fn get_delta_vol_min(&self) -> Volume {
        self.pump.get_delta_vol_min()
    }
    fn get_delta_vol_max(&self) -> Volume {
        self.pump.get_delta_vol_max()
    }
}

pub struct RatPump {
    active: bool,
    pump: Pump,
}
impl RatPump {
    const DISPLACEMENT_BREAKPTS: [f64; 9] = [
        0.0, 500.0, 1000.0, 1500.0, 2800.0, 2900.0, 3000.0, 3050.0, 3500.0,
    ];
    const DISPLACEMENT_MAP: [f64; 9] = [
        1.15 , 1.15,  1.15,  1.15 , 1.15,  1.15 , 0.9, 0.0 ,0.0
    ];
    
    const NORMAL_RPM: f64 = 6000.;

    pub fn new() -> RatPump {
        RatPump {
            active: false,
            pump: Pump::new(RatPump::DISPLACEMENT_BREAKPTS,RatPump::DISPLACEMENT_MAP),
        }
    }

    pub fn update(&mut self, delta_time: &Duration,context: &UpdateContext, line: &HydLoop) {
        self.pump.update(delta_time, context, line, RatPump::NORMAL_RPM);
    }
}
impl PressureSource for RatPump {
    fn get_delta_vol_max(&self) -> Volume {
        self.pump.get_delta_vol_max()
    }

    fn get_delta_vol_min(&self) -> Volume {
        self.pump.get_delta_vol_min()
    }
}

////////////////////////////////////////////////////////////////////////////////
// ACTUATOR DEFINITION
////////////////////////////////////////////////////////////////////////////////

pub struct Actuator {
    a_type: ActuatorType,
    active: bool,
    affected_by_gravity: bool,
    area: Area,
    line: HydLoop,
    neutral_is_zero: bool,
    stall_load: Force,
    volume_used_at_max_deflection: Volume,
}

// TODO
impl Actuator {
    pub fn new(a_type: ActuatorType, line: HydLoop) -> Actuator {
        Actuator { 
            a_type,
            active: false,
            affected_by_gravity: false,
            area: Area::new::<square_meter>(5.0),
            line,
            neutral_is_zero: true,
            stall_load: Force::new::<newton>(47000.),
            volume_used_at_max_deflection: Volume::new::<gallon>(0.),
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
// TESTS
////////////////////////////////////////////////////////////////////////////////


use plotlib::page::Page;
use plotlib::repr::Plot;
use plotlib::view::ContinuousView;
use plotlib::style::{PointMarker, PointStyle, LineStyle};

extern crate rustplotlib;
use rustplotlib::Figure;


fn make_figure<'a>(h: &'a History) -> Figure<'a> {
    use rustplotlib::{Axes2D, Line2D};
  
    let mut allAxis: Vec<Option<Axes2D>> = Vec::new();

    let mut idx=0;
    for curData in &h.dataVector {
        let mut currAxis = Axes2D::new()
            .add(Line2D::new(h.nameVector[idx].as_str())
            .data(&h.timeVector, &curData)
            .color("blue")
            //.marker("x")
            //.linestyle("--")
            .linewidth(1.0))
            .xlabel("Time [sec]")
            .ylabel(h.nameVector[idx].as_str())
            .legend("best")
            .xlim(0.0, *h.timeVector.last().unwrap());
            //.ylim(-2.0, 2.0);

            currAxis=currAxis.grid(true);    
        idx=idx+1;
        allAxis.push(Some(currAxis));
    }
  
    Figure::new()
      .subplots(allAxis.len() as u32, 1, allAxis)
  }

//History class to record a simulation
pub struct History {
    timeVector: Vec<f64>, //Simulation time starting from 0
    nameVector: Vec<String>, //Name of each var saved
    dataVector: Vec<Vec<f64>>, //Vector data for each var saved
    dataSize: usize,
}

impl History {
    pub fn new(names: Vec<String> ) -> History {          
        History {
            timeVector: Vec::new(),
            nameVector: names.clone(),
            dataVector: Vec::new(),
            dataSize: names.len(),
        }
    }

    //Sets initialisation values of each data before first step
    pub fn init(&mut self,startTime:f64, values: Vec<f64>) {
        self.timeVector.push(startTime);
        for idx in 0..(values.len()) {
            self.dataVector.push(vec![values[idx]]);
        }
    }

    //Updates all values and time vector
    pub fn update(&mut self,deltaTime :f64, values: Vec<f64>) {
        self.timeVector.push(self.timeVector.last().unwrap() + deltaTime);
        self.pushData(values);
    }

    pub fn pushData(&mut self,values: Vec<f64>){
        for idx in 0..values.len() {
            self.dataVector[idx].push(values[idx]);
        }
    }

    //Builds a graph using rust crate plotlib
    pub fn show(self){
        
        let mut v = ContinuousView::new()
        .x_range(0.0, *self.timeVector.last().unwrap())
        .y_range(0.0, 3500.0)
        .x_label("Time (s)")
        .y_label("Value");

        for curData in self.dataVector {
            //Here build the 2 by Xsamples vector
            let mut newVector: Vec<(f64,f64)> = Vec::new();
            for sampleIdx in 0..self.timeVector.len(){
                newVector.push( (self.timeVector[sampleIdx] , curData[sampleIdx]) );
            }

            // We create our scatter plot from the data
            let s1: Plot = Plot::new(newVector).line_style(
                LineStyle::new()
                    .colour("#DD3355"),
            );
            
            v=v.add(s1);
        }
           

        // A page with a single view is then saved to an SVG file
        Page::single(&v).save("scatter.svg").unwrap();

    }
    
    //builds a graph using matplotlib python backend. PYTHON REQUIRED AS WELL AS MATPLOTLIB PACKAGE
    pub fn showMatplotlib(&self,figure_title : &str){
        let fig = make_figure(&self);

        use rustplotlib::Backend;
        use rustplotlib::backend::Matplotlib;
        let mut mpl = Matplotlib::new().unwrap();
        mpl.set_style("ggplot").unwrap();
        
        fig.apply(&mut mpl).unwrap();
        
        //mpl.savefig("simple.png").unwrap();
        mpl.savefig(figure_title);
        //mpl.dump_pickle("simple.fig.pickle").unwrap();
        mpl.wait().unwrap();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn green_loop_edp_simulation() {
        let green_loop_var_names = vec!["Loop Pressure".to_string(), "Loop Volume".to_string(), "Loop Reservoir".to_string(), "Loop Flow".to_string()];
        let mut greenLoopHistory = History::new(green_loop_var_names);
        
        let edp1_var_names = vec!["Delta Vol Max".to_string(), "n2 ratio".to_string()];
        let mut edp1_History = History::new(edp1_var_names);

        let mut edp1 = engine_driven_pump();
        let mut green_loop = hydraulic_loop(LoopColor::Green);
        edp1.active = true;

        let init_n2 = Ratio::new::<percent>(0.5);
        let mut engine1 = engine(init_n2);
        let ct = context(Duration::from_millis(100));

        //let initValues = vec![green_loop.loop_pressure.get::<psi>(), green_loop.accumulator_gas_pressure.get::<psi>()];     
        greenLoopHistory.init(0.0,vec![green_loop.loop_pressure.get::<psi>(), green_loop.loop_volume.get::<gallon>(),green_loop.reservoir_volume.get::<gallon>(),green_loop.current_flow.get::<gallon_per_second>()]);
        edp1_History.init(0.0,vec![edp1.get_delta_vol_max().get::<liter>(), engine1.n2.get::<percent>() as f64]);

        for x in 0..400 {
            if x == 200 {
                engine1.n2 = Ratio::new::<percent>(0.0);
                //green_loop.loop_pressure = Pressure::new::<psi>(1500.);
            }
            // println!("Iteration {}", x);
            // println!("-------------------------------------------");
            edp1.update(&ct.delta,&ct, &green_loop, &engine1);
            green_loop.update(&ct.delta,&ct, Vec::new(), vec![&edp1], Vec::new(), Vec::new());
            if x % 20 == 0 {
                println!("Iteration {}", x);
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
                // println!(
                //     "--------Acc Fluid Volume (L): {}",
                //     green_loop.accumulator_fluid_volume.get::<liter>()
                // );
                // println!(
                //     "--------Acc Gas Volume (L): {}",
                //     green_loop.accumulator_gas_volume.get::<liter>()
                // );
                println!(
                    "--------Acc Gas Pressure (psi): {}",
                    green_loop.accumulator_gas_pressure.get::<psi>()
                );
            }

            greenLoopHistory.update(ct.delta.as_secs_f64(), vec![green_loop.loop_pressure.get::<psi>(), green_loop.loop_volume.get::<gallon>(),green_loop.reservoir_volume.get::<gallon>(),green_loop.current_flow.get::<gallon_per_second>()]);
            edp1_History.update(ct.delta.as_secs_f64(),vec![edp1.get_delta_vol_max().get::<liter>(), engine1.n2.get::<percent>() as f64]);

        }
        assert!(true);

        greenLoopHistory.showMatplotlib("Green Loop Pressures");     
        edp1_History.showMatplotlib("EDP1 data") ;
    }

    #[test]
    fn yellow_loop_epump_simulation() {
        let mut epump = electric_pump();
        let mut yellow_loop = hydraulic_loop(LoopColor::Yellow);
        epump.active = true;

        let ct = context(Duration::from_millis(100));
        for x in 0..800 {
            if x == 400 {
                epump.active = false;
            }
            epump.update(&ct.delta,&ct, &yellow_loop);
            yellow_loop.update(&ct.delta,&ct, vec![&epump], Vec::new(), Vec::new(), Vec::new());
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
                    yellow_loop.accumulator_gas_volume.get::<gallon>()
                );
            }
        }

        assert!(true)
    }

    fn hydraulic_loop(loop_color: LoopColor) -> HydLoop {
        HydLoop::new(
            loop_color,
            false,
            Volume::new::<gallon>(26.00),
            Volume::new::<gallon>(26.41),
            Volume::new::<gallon>(10.0),
            Volume::new::<gallon>(3.83),
            HydFluid::new(Pressure::new::<pascal>(1450000000.0))
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
            let displacement = Volume::new::<cubic_inch>(EngineDrivenPump::DISPLACEMENT_MAP.iter().cloned().fold(-1./0. /* -inf */, f64::max));
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
            let mut context = context((time));
            line.loop_pressure = pressure;
            edp.update(&time,&context, &line, &eng);
            edp.get_delta_vol_max()
        }

        fn get_edp_predicted_delta_vol_when(
            n2: Ratio,
            displacement: Volume,
            time: Duration,
        ) -> Volume {
            let edp_rpm = (1.0f64.min(4.0 * n2.get::<percent>())) * EngineDrivenPump::MAX_RPM;
            let expected_flow = Pump::calculate_flow(edp_rpm, displacement);
            expected_flow * Time::new::<second>(time.as_secs_f64())
        }
    }
}
