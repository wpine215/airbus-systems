use std::time::Duration;
use uom::si::{
    area::square_meter, f64::*, force::newton, length::foot, length::meter,
    mass_density::kilogram_per_cubic_meter, pressure::atmosphere, pressure::pascal, pressure::psi,
    ratio::percent, thermodynamic_temperature::degree_celsius, time::second, velocity::knot,
    volume::cubic_inch, volume::gallon, volume::liter, volume_rate::cubic_meter_per_second,
    volume_rate::gallon_per_second,
};
use crate::{hydraulic::{Actuator, ElectricPump, EngineDrivenPump, HydFluid, HydLoop, LoopColor, Pump, RatPump}, overhead::{AutoOffPushButton, NormalAltnPushButton, OnOffPushButton}, shared::{DelayedTrueLogicGate, Engine}, simulator::UpdateContext};

pub struct A320Hydraulic {
    blue_loop: HydLoop,
    green_loop: HydLoop,
    yellow_loop: HydLoop,
    engine_driven_pump_1: EngineDrivenPump,
    engine_driven_pump_2: EngineDrivenPump,
    blue_electric_pump: ElectricPump,
    yellow_electric_pump: ElectricPump,
    // Until hydraulic is implemented, we'll fake it with this boolean.
    // blue_pressurised: bool,
}

impl A320Hydraulic {
    const MIN_PRESS_PRESSURISED : f64 = 100.0;

    pub fn new() -> A320Hydraulic {
        A320Hydraulic {
            
            blue_loop: HydLoop::new(
                LoopColor::Blue,
                false,
                false,
                Volume::new::<gallon>(1.5),
                Volume::new::<gallon>(1.6),
                Volume::new::<gallon>(1.6),
                Volume::new::<gallon>(1.5),
                HydFluid::new(Pressure::new::<pascal>(1450000000.0))
            ),
            green_loop: HydLoop::new(
                LoopColor::Green,
                true,
                false,
                Volume::new::<gallon>(10.2),
                Volume::new::<gallon>(10.2),
                Volume::new::<gallon>(8.0),
                Volume::new::<gallon>(3.3),
                HydFluid::new(Pressure::new::<pascal>(1450000000.0))
            ),
            yellow_loop: HydLoop::new(
                LoopColor::Blue,
                false,
                true,
                Volume::new::<gallon>(26.00),
                Volume::new::<gallon>(26.41),
                Volume::new::<gallon>(10.0),
                Volume::new::<gallon>(3.83),
                HydFluid::new(Pressure::new::<pascal>(1450000000.0))
            ),
            engine_driven_pump_1: EngineDrivenPump::new(),
            engine_driven_pump_2: EngineDrivenPump::new(),
            blue_electric_pump: ElectricPump::new(),
            yellow_electric_pump: ElectricPump::new(),
        }
    }

    pub fn is_blue_pressurised(&self) -> bool {
        self.blue_loop.get_pressure().get::<psi>() >= A320Hydraulic::MIN_PRESS_PRESSURISED
    }

    pub fn is_green_pressurised(&self) -> bool {
        self.green_loop.get_pressure().get::<psi>() >= A320Hydraulic::MIN_PRESS_PRESSURISED
    }

    pub fn is_yellow_pressurised(&self) -> bool {
        self.yellow_loop.get_pressure().get::<psi>() >= A320Hydraulic::MIN_PRESS_PRESSURISED
    }

    pub fn update(&mut self, _: &UpdateContext) {
        
    }
}

pub struct A320HydraulicOverheadPanel {
}

impl A320HydraulicOverheadPanel {
    pub fn new() -> A320HydraulicOverheadPanel {
        A320HydraulicOverheadPanel {

        }
    }

    pub fn update(&mut self, context: &UpdateContext) {
    }
}
