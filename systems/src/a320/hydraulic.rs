use std::time::{Duration, Instant};
use uom::si::{
    area::square_meter, f64::*, force::newton, length::foot, length::meter,
    mass_density::kilogram_per_cubic_meter, pressure::atmosphere, pressure::pascal, pressure::psi,
    ratio::percent, thermodynamic_temperature::degree_celsius, time::second, velocity::knot,
    volume::cubic_inch, volume::gallon, volume::liter, volume_rate::cubic_meter_per_second,
    volume_rate::gallon_per_second,
};
use crate::{hydraulic::{Actuator, ElectricPump, EngineDrivenPump, HydFluid, HydLoop, LoopColor, Pump, RatPump, Ptu}, overhead::{AutoOffPushButton, NormalAltnPushButton, OnOffPushButton}, shared::{DelayedTrueLogicGate, Engine}, simulator::UpdateContext};

pub struct A320Hydraulic {
    blue_loop: HydLoop,
    green_loop: HydLoop,
    yellow_loop: HydLoop,
    engine_driven_pump_1: EngineDrivenPump,
    engine_driven_pump_2: EngineDrivenPump,
    blue_electric_pump: ElectricPump,
    yellow_electric_pump: ElectricPump,
    ptu: Ptu,
    total_sim_time_elapsed: Duration,
    lag_time_accumulator: Duration,
    // Until hydraulic is implemented, we'll fake it with this boolean.
    // blue_pressurised: bool,
}

impl A320Hydraulic {
    const MIN_PRESS_PRESSURISED : f64 = 100.0;
    const HYDRAULIC_SIM_TIME_STEP : u64 = 100; //refresh rate of hydraulic simulation in ms

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
                HydFluid::new(Pressure::new::<pascal>(1450000000.0)),
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
            ptu : Ptu::new(),
            total_sim_time_elapsed: Duration::new(0,0),
            lag_time_accumulator: Duration::new(0,0),
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

    pub fn update(&mut self, ct: &UpdateContext) {

        let min_hyd_loop_timestep = Duration::from_millis(A320Hydraulic::HYDRAULIC_SIM_TIME_STEP); //Hyd Sim rate = 10 Hz

        //time to catch up in our simulation
        self.total_sim_time_elapsed += ct.delta;

        let time_to_catch=self.total_sim_time_elapsed + self.lag_time_accumulator;

        
        //Dummy engine TODO to remove when I know how to get engine state
        let mut engine1 = Engine::new();
        engine1.n2=Ratio::new::<percent>(0.5);
        let mut engine2 = Engine::new();
        engine2.n2=Ratio::new::<percent>(0.5);

        //Number of time steps to do according to required time step
        let numberOfSteps_f64 = time_to_catch.as_secs_f64()/min_hyd_loop_timestep.as_secs_f64();

        if numberOfSteps_f64 < 1.0 {
            //Can't do a full time step
            //we can either do an update with smaller step or wait next iteration
            //Other option is to update only actuator position based on known hydraulic 
            //state to avoid lag of control surfaces if sim runs really fast
            self.lag_time_accumulator=Duration::from_secs_f64(numberOfSteps_f64 * min_hyd_loop_timestep.as_secs_f64()); //Time lag is float part of num of steps * fixed time step to get a result in time
        } else {
            //TRUE UPDATE LOOP HERE
            let num_of_update_loops = numberOfSteps_f64.floor() as u32; //Int part is the actual number of loops to do
            //Rest of floating part goes into accumulator
            self.lag_time_accumulator= Duration::from_secs_f64((numberOfSteps_f64 - (num_of_update_loops as f64))* min_hyd_loop_timestep.as_secs_f64()); //Keep track of time left after all fixed loop are done
           

            for curLoop in  0..num_of_update_loops {
                //UPDATE HYDRAULICS FIXED TIME STEP
                     
                self.ptu.update(&self.green_loop, &self.yellow_loop);
                self.engine_driven_pump_1.update(&min_hyd_loop_timestep,&ct, &self.green_loop, &engine1);
                self.engine_driven_pump_2.update(&min_hyd_loop_timestep,&ct, &self.yellow_loop, &engine2);
                self.yellow_electric_pump.update(&min_hyd_loop_timestep,&ct, &self.yellow_loop);
                self.blue_electric_pump.update(&min_hyd_loop_timestep,&ct, &self.blue_loop);
          

                self.green_loop.update(&min_hyd_loop_timestep,&ct, Vec::new(), vec![&self.engine_driven_pump_1], Vec::new(), vec![&self.ptu]);
                self.yellow_loop.update(&min_hyd_loop_timestep,&ct, vec![&self.yellow_electric_pump], vec![&self.engine_driven_pump_2], Vec::new(), vec![&self.ptu]);
                self.blue_loop.update(&min_hyd_loop_timestep,&ct, vec![&self.blue_electric_pump], Vec::new(), Vec::new(), Vec::new());
            }
          
        }        
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
