use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;

use nalgebra as na;
use serde;
use tokio::sync::RwLock as TRwLock;
use tokio::time::Instant;

use crate::obsw_interface::*;
use crate::pid::PidRegulator;

#[derive(Clone, Debug, PartialEq, serde::Deserialize, serde::Serialize)]
pub struct Controls {
    pub throttle_main: f32,       // Generally influences speed
    pub throttle_split: [f32; 4], // Allows you to steer motors individually
    pub sideways: f32,            // Left/right linear movement
    pub elevation: f32,           // Up-down motion
    pub pitch: f32,               // Rotate forward-backward
    pub roll: f32,                // Roll left/right
    pub yaw: f32,                 // Rotate left/right - change heading
    pub desired_flight_mode: FlightMode,
}

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
pub enum BlimpAction {
    // Even numbers are up-down servers. Odd numbers are left-right ones.
    // Servos corresponding to given motor i are 2i and 2i+1.
    SetServo { servo: u8, location: f32 },
    // Motors layout
    // 0 1
    // 2 3
    SetMotor { motor: u8, speed: f32 },
    SendMsg(Box<MessageB2G>), // This has to be boxed, because otherwise we would have infinitely
                              // sized struct
}

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
pub enum SensorType {
    Barometer,
    MagnetometerHeading,
    AccelerometerX,
    AccelerometerY,
    AccelerometerZ,
    GyroscopeX,
    GyroscopeY,
    GyroscopeZ,
    GPSLatitude,
    GPSLongitude,
    GPSAltitude,
}

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
pub enum BlimpEvent {
    Control(Controls),
    GetMsg(MessageG2B),
    SensorDataF64(SensorType, f64),
}

#[derive(Debug, Clone, PartialEq, Eq, serde::Deserialize, serde::Serialize)]
pub enum FlightMode {
    Manual,   // Throttle -> motors speed; Pitch -> motors pitch; Roll -> motors yaw
    Atti,     // Stabilize heading, control thrust vector
    AltiAtti, // Like Atti, but also stabilize altitude
}

#[derive(Clone, Debug, PartialEq, serde::Deserialize, serde::Serialize)]
pub struct BlimpState {
    flight_mode: FlightMode,
    altitude: Option<f64>,
    desired_altitude: Option<f64>,
    heading: Option<f64>,
    desired_heading: Option<f64>,
    pitch: f64,
    roll: f64,
}

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
pub enum MessageG2B {
    Ping(u32),
    Pong(u32),
    Control(Controls),
}

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
pub enum MessageB2G {
    Ping(u32),
    Pong(u32),
    ForwardAction(BlimpAction),
    ForwardEvent(BlimpEvent),
    BlimpState(BlimpState),
}

pub struct BlimpMainAlgo {
    action_callback: TRwLock<
        Option<
            Arc<
                dyn Fn(BlimpAction) -> Pin<Box<dyn Future<Output = ()> + Send + Sync>>
                    + Send
                    + Sync,
            >,
        >,
    >,

    curr_flight_mode: TRwLock<FlightMode>,
    controls: TRwLock<Controls>,
    altitude: TRwLock<Option<f64>>,
    gps_location: TRwLock<Option<(f64, f64)>>,
    acceleration: TRwLock<Option<(f64, f64, f64)>>,
    heading: TRwLock<Option<f64>>,
    pitch_roll: TRwLock<(f64, f64)>,

    attitude_pid: TRwLock<PidRegulator<f64>>,
    altitude_pid: TRwLock<PidRegulator<f64>>,
    previous_step_time: TRwLock<Instant>,
}

impl BlimpAlgorithm<BlimpEvent, BlimpAction> for BlimpMainAlgo {
    fn handle_event(&self, ev: BlimpEvent) -> Pin<Box<impl Future<Output = ()>>> {
        Box::pin(async move {
            match &ev {
                BlimpEvent::Control(ctrl) => {
                    *self.controls.write().await = ctrl.clone();
                }
                BlimpEvent::SensorDataF64(SensorType::Barometer, press) => {
                    // Compute altitude
                    // See: https://en.wikipedia.org/wiki/Barometric_formula
                    // p = p_b * exp(-g * M * h / R / T)
                    // ln (p / p_b) = -g * M * h / R / T
                    // h = (ln p - ln p_b) * (-R) * T / g / M
                    // h = (ln p_b - ln p) * R * T / g / M
                    // TODO: Stablize and smoothen
                    // TODO: Allow changing base (sea level) pressure and temperature
                    let base_pressure: f64 = 101325.0;
                    let temperature: f64 = 288.15;
                    let const_coef: f64 = 0.0292718; // R / g / M
                    *self.altitude.write().await =
                        Some((base_pressure.ln() - press.ln()) * const_coef * temperature);
                }
                BlimpEvent::SensorDataF64(SensorType::MagnetometerHeading, heading) => {
                    *self.heading.write().await = Some(*heading);
                }
                BlimpEvent::SensorDataF64(SensorType::AccelerometerX, acc_x) => {
                    let mut acc_locked = self.acceleration.write().await;
                    let prev_acc = acc_locked.unwrap_or((0.0, 0.0, 0.0));
                    *acc_locked = Some((*acc_x, prev_acc.1, prev_acc.2));
                }
                BlimpEvent::SensorDataF64(SensorType::AccelerometerY, acc_y) => {
                    let mut acc_locked = self.acceleration.write().await;
                    let prev_acc = acc_locked.unwrap_or((0.0, 0.0, 0.0));
                    *acc_locked = Some((prev_acc.0, *acc_y, prev_acc.2));
                }
                BlimpEvent::SensorDataF64(SensorType::AccelerometerZ, acc_z) => {
                    let mut acc_locked = self.acceleration.write().await;
                    let prev_acc = acc_locked.unwrap_or((0.0, 0.0, 0.0));
                    let acc_new = (prev_acc.0, prev_acc.1, *acc_z);
                    *acc_locked = Some(acc_new);

                    let acc_resultant =
                        (acc_new.0 * acc_new.0 + acc_new.1 * acc_new.1 + acc_new.2 * acc_new.2)
                            .sqrt();

                    let pitch = (-acc_new.1 / acc_resultant).asin();
                    let roll = (-acc_new.1).atan2(acc_new.2);
                    *self.pitch_roll.write().await = (pitch, roll);
                }
                BlimpEvent::SensorDataF64(SensorType::GPSLatitude, latitude) => {
                    let prev_long = self.gps_location.read().await.unwrap_or((0.0, 0.0)).1;
                    *self.gps_location.write().await = Some((*latitude, prev_long));
                }
                BlimpEvent::SensorDataF64(SensorType::GPSLongitude, longitude) => {
                    let prev_lat = self.gps_location.read().await.unwrap_or((0.0, 0.0)).0;
                    *self.gps_location.write().await = Some((prev_lat, *longitude));
                }
                BlimpEvent::GetMsg(msg) => match msg {
                    MessageG2B::Ping(id) => {
                        self.perform_action(BlimpAction::SendMsg(Box::new(MessageB2G::Pong(*id))))
                            .await;
                    }
                    MessageG2B::Pong(_id) => {}
                    MessageG2B::Control(ctrl) => {
                        self.handle_event(BlimpEvent::Control(ctrl.clone())).await;
                    }
                },
                _ => {}
            }
            if matches!(&ev, BlimpEvent::SensorDataF64(..)) {
                self.perform_action(BlimpAction::SendMsg(Box::new(MessageB2G::ForwardEvent(
                    ev.clone(),
                ))))
                .await;
            }
        })
    }

    fn set_action_callback(
        &mut self,
        callback: Arc<
            dyn Fn(BlimpAction) -> Pin<Box<dyn Future<Output = ()> + Send + Sync>> + Send + Sync,
        >,
    ) -> Pin<Box<impl Future<Output = ()>>> {
        Box::pin(async move {
            *self.action_callback.write().await = Some(callback);
        })
    }
}

impl BlimpMainAlgo {
    pub fn new() -> Self {
        Self {
            action_callback: TRwLock::new(None),

            curr_flight_mode: TRwLock::new(FlightMode::Manual),
            controls: TRwLock::new(Controls {
                throttle_main: 0.0,
                throttle_split: [0.0, 0.0, 0.0, 0.0],
                sideways: 0.0,
                elevation: 0.0,
                pitch: 0.0,
                roll: 0.0,
                yaw: 0.0,
                desired_flight_mode: FlightMode::Manual,
            }),
            altitude: TRwLock::new(None),
            gps_location: TRwLock::new(None),
            acceleration: TRwLock::new(None),
            heading: TRwLock::new(None),
            pitch_roll: TRwLock::new((0.0, 0.0)),

            attitude_pid: TRwLock::new(PidRegulator::new(0.0, 1.0, 0.15, 0.05)),
            altitude_pid: TRwLock::new(PidRegulator::new(0.0, 1.0, 0.15, 0.05)),
            previous_step_time: TRwLock::new(Instant::now()),
        }
    }

    pub async fn step(&self) {
        let mut curr_flight_mode = self.curr_flight_mode.write().await;
        let controls = self.controls.read().await;
        if *curr_flight_mode != controls.desired_flight_mode {
            match controls.desired_flight_mode {
                FlightMode::Manual => {}
                FlightMode::Atti => {
                    self.attitude_pid.write().await.setpoint =
                        (*self.heading.read().await).unwrap_or(0.0);
                }
                FlightMode::AltiAtti => {
                    self.attitude_pid.write().await.setpoint =
                        (*self.heading.read().await).unwrap_or(0.0);
                    self.altitude_pid.write().await.setpoint =
                        (*self.altitude.read().await).unwrap_or(0.0);
                }
            }
        }
        *curr_flight_mode = controls.desired_flight_mode.clone();

        let curr_flight_mode = curr_flight_mode.downgrade();
        match *curr_flight_mode {
            FlightMode::Manual => {
                for i in 0..(4 as u8) {
                    let speed: f32 = controls.throttle_main
                        + controls.throttle_split[i as usize]
                        + (if i % 2 == 0 { 1.0 } else { -1.0 }) * controls.yaw;
                    //Motor
                    self.perform_action(BlimpAction::SetMotor { motor: i, speed })
                        .await;
                    // Up-down servo
                    self.perform_action(BlimpAction::SetServo {
                        servo: 2 * i,
                        location: controls.elevation * 90.0,
                    })
                    .await;
                    //Sideways servo
                    self.perform_action(BlimpAction::SetServo {
                        servo: 2 * i + 1,
                        location: controls.yaw * 90.0,
                    })
                    .await;
                }
            }
            FlightMode::Atti | FlightMode::AltiAtti => {
                let previous_step_time = self.previous_step_time.read().await;
                let delta_time = (tokio::time::Instant::now() - *previous_step_time).as_secs_f64();

                let mut attitude_pid = self.attitude_pid.write().await;
                let heading = self.heading.read().await;
                attitude_pid.setpoint += controls.yaw as f64 * delta_time;
                let attitude_pid_result = if let Some(heading) = *heading {
                    Some(attitude_pid.update(heading.clone(), delta_time))
                } else {
                    None
                };

                let mut altitude_pid = self.altitude_pid.write().await;
                let altitude = self.altitude.read().await;
                let altitude_pid_result = if *curr_flight_mode == FlightMode::AltiAtti {
                    altitude_pid.setpoint += controls.elevation as f64 * delta_time;
                    if let Some(altitude) = *altitude {
                        Some(altitude_pid.update(altitude.clone(), delta_time))
                    } else {
                        None
                    }
                } else {
                    None
                };

                let mut mdfv = na::Vector3::<f64>::zeros();
                let mut lrfvs = Vec::<na::Vector3<f64>>::new();
                for _ in 0..4 {
                    lrfvs.push(na::Vector3::<f64>::zeros());
                }

                mdfv.x += controls.sideways as f64;
                mdfv.y += controls.throttle_main as f64;
                mdfv.z += if let Some(altitude_pid_result) = altitude_pid_result {
                    altitude_pid_result
                } else {
                    controls.elevation as f64
                };

                self.vectored_thrust(mdfv, &lrfvs).await;
            }
        }

        let pitch_roll_locked = self.pitch_roll.read().await;
        self.perform_action(BlimpAction::SendMsg(Box::new(MessageB2G::BlimpState(
            BlimpState {
                flight_mode: curr_flight_mode.clone(),
                altitude: *self.altitude.read().await,
                desired_altitude: if *curr_flight_mode == FlightMode::AltiAtti {
                    Some(self.altitude_pid.read().await.setpoint)
                } else {
                    None
                },
                heading: *self.heading.read().await,
                desired_heading: if *curr_flight_mode == FlightMode::Atti
                    || *curr_flight_mode == FlightMode::AltiAtti
                {
                    Some(self.attitude_pid.read().await.setpoint)
                } else {
                    None
                },
                pitch: pitch_roll_locked.0,
                roll: pitch_roll_locked.1,
            },
        ))))
        .await;

        *self.previous_step_time.write().await = tokio::time::Instant::now();
    }

    async fn perform_action(&self, action: BlimpAction) {
        // action_callback.read().await(action.clone()).await;
        if let Some(ac) = &*self.action_callback.read().await {
            ac(action.clone()).await;
        }

        // Some actions should be forwarded
        if matches!(
            action,
            BlimpAction::SetMotor { .. } | BlimpAction::SetServo { .. }
        ) {
            if let Some(ac) = &*self.action_callback.read().await {
                ac(BlimpAction::SendMsg(Box::new(MessageB2G::ForwardAction(
                    action,
                ))))
                .await;
            }
        }
    }

    // MDFV - main desired force vector
    // LRFVs - local rotating force vectors
    // LFVs - local force vectors = MDFV + LRFV
    async fn vectored_thrust(&self, mdfv: na::Vector3<f64>, lrfvs: &[na::Vector3<f64>]) {
        let lfvs = lrfvs
            .iter()
            .map(|x| x + mdfv)
            .collect::<Vec<na::Vector3<f64>>>();
        for i in 0..lrfvs.len() {
            // Up-down servo
            let lfv_x = lfvs[i].x;
            let lfv_y = lfvs[i].y;
            let lfv_z = lfvs[i].z;
            let lfv_hor = (f64::powf(lfv_x, 2.0) + f64::powf(lfv_y, 2.0)).sqrt();
            let lfv_magn =
                (f64::powf(lfv_x, 2.0) + f64::powf(lfv_y, 2.0) + f64::powf(lfv_z, 2.0)).sqrt();

            self.perform_action(BlimpAction::SetServo {
                servo: 2 * i as u8,
                location: (f64::atan2(lfv_z, lfv_hor) * 180.0 / std::f64::consts::PI) as f32,
            })
            .await;
            // Sideways servo
            self.perform_action(BlimpAction::SetServo {
                servo: (2 * i + 1) as u8,
                location: (f64::atan2(lfv_y, lfv_x) * 180.0 / std::f64::consts::PI) as f32,
            })
            .await;

            // Motor
            self.perform_action(BlimpAction::SetMotor {
                motor: i as u8,
                speed: lfv_magn as f32,
            })
            .await;
        }
    }
}
