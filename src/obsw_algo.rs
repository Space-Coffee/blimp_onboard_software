use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;

use nalgebra as na;
use serde;
use tokio;
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
    pub motors_toggles: [bool; 4],
    pub motors_reverse: [bool; 4],
    pub nav_lights: bool,
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
    // This has to be boxed, because otherwise we would have infinitely sized struct
    SendMsg(Box<MessageB2G>),
    // Positive number is blink frequency; zero is solid light; negative is off
    NavLights(f32),
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
    altitude: f64,
    desired_altitude: Option<f64>,
    heading: f64,
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

pub struct BlimpInnerState {
    curr_flight_mode: FlightMode,
    controls: Controls,
    altitude: f64,
    gps_location: Option<(f64, f64)>,
    acceleration: Option<(f64, f64, f64)>,
    heading: f64,
    pub pitch_roll: (f64, f64),
}

struct BlimpPids {
    attitude_pid: PidRegulator<f64>,
    altitude_pid: PidRegulator<f64>,
    pitch_pid: PidRegulator<f64>,
    roll_pid: PidRegulator<f64>,
    previous_step_time: Instant,
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

    pub inner_state: TRwLock<BlimpInnerState>,
    pids: TRwLock<BlimpPids>,
}

impl BlimpAlgorithm<BlimpEvent, BlimpAction> for BlimpMainAlgo {
    fn handle_event(
        self: Arc<Self>,
        ev: BlimpEvent,
    ) -> Pin<Box<dyn Future<Output = ()> + Send + Sync>> {
        Box::pin(async move {
            let mut inner_state = self.inner_state.write().await;
            match &ev {
                BlimpEvent::Control(ctrl) => {
                    inner_state.controls = ctrl.clone();
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
                    let const_coef: f64 = 29.2718; // R / g / M
                    inner_state.altitude =
                        (base_pressure.ln() - press.ln()) * const_coef * temperature;
                }
                BlimpEvent::SensorDataF64(SensorType::MagnetometerHeading, heading) => {
                    inner_state.heading = *heading;
                }
                BlimpEvent::SensorDataF64(SensorType::AccelerometerX, acc_x) => {
                    let prev_acc = inner_state.acceleration.unwrap_or((0.0, 0.0, 0.0));
                    inner_state.acceleration = Some((*acc_x, prev_acc.1, prev_acc.2));
                }
                BlimpEvent::SensorDataF64(SensorType::AccelerometerY, acc_y) => {
                    let prev_acc = inner_state.acceleration.unwrap_or((0.0, 0.0, 0.0));
                    inner_state.acceleration = Some((prev_acc.0, *acc_y, prev_acc.2));
                }
                BlimpEvent::SensorDataF64(SensorType::AccelerometerZ, acc_z) => {
                    let prev_acc = inner_state.acceleration.unwrap_or((0.0, 0.0, 0.0));
                    let acc_new = (prev_acc.0, prev_acc.1, *acc_z);
                    inner_state.acceleration = Some(acc_new);

                    // See: https://mwrona.com/posts/accel-roll-pitch/
                    let acc_resultant =
                        (acc_new.0 * acc_new.0 + acc_new.1 * acc_new.1 + acc_new.2 * acc_new.2)
                            .sqrt();

                    let pitch = (acc_new.1 / acc_resultant).asin();
                    let roll = (-acc_new.0).atan2(acc_new.2);
                    inner_state.pitch_roll = (pitch, roll);
                }
                BlimpEvent::SensorDataF64(SensorType::GPSLatitude, latitude) => {
                    let prev_long = inner_state.gps_location.unwrap_or((0.0, 0.0)).1;
                    inner_state.gps_location = Some((*latitude, prev_long));
                }
                BlimpEvent::SensorDataF64(SensorType::GPSLongitude, longitude) => {
                    let prev_lat = inner_state.gps_location.unwrap_or((0.0, 0.0)).0;
                    inner_state.gps_location = Some((prev_lat, *longitude));
                }
                BlimpEvent::GetMsg(msg) => match msg {
                    MessageG2B::Ping(id) => {
                        self.perform_action(BlimpAction::SendMsg(Box::new(MessageB2G::Pong(*id))))
                            .await;
                    }
                    MessageG2B::Pong(_id) => {}
                    MessageG2B::Control(ctrl) => {
                        tokio::spawn(self.clone().handle_event(BlimpEvent::Control(ctrl.clone())));
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
        &self,
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

            inner_state: TRwLock::new(BlimpInnerState {
                curr_flight_mode: FlightMode::Manual,
                controls: Controls {
                    throttle_main: 0.0,
                    throttle_split: [0.0, 0.0, 0.0, 0.0],
                    sideways: 0.0,
                    elevation: 0.0,
                    pitch: 0.0,
                    roll: 0.0,
                    yaw: 0.0,
                    desired_flight_mode: FlightMode::Manual,
                    motors_toggles: [true; 4],
                    motors_reverse: [false; 4],
                    nav_lights: false,
                },
                altitude: 0.0,
                gps_location: None,
                acceleration: None,
                heading: 0.0,
                pitch_roll: (0.0, 0.0),
            }),

            pids: TRwLock::new(BlimpPids {
                attitude_pid: PidRegulator::new(0.0, 1.0, 0.15, 2.0, Some(std::f64::consts::PI)),
                altitude_pid: PidRegulator::new(0.0, 1.0, 0.15, 2.0, None),
                pitch_pid: PidRegulator::new(0.0, 1.0, 0.15, 2.0, Some(std::f64::consts::PI)),
                roll_pid: PidRegulator::new(0.0, 1.0, 0.15, 2.0, Some(std::f64::consts::PI)),
                previous_step_time: Instant::now(),
            }),
        }
    }

    pub async fn step(&self) {
        let mut inner_state = self.inner_state.write().await;
        if inner_state.curr_flight_mode != inner_state.controls.desired_flight_mode {
            match inner_state.controls.desired_flight_mode {
                FlightMode::Manual => {}
                FlightMode::Atti => {
                    self.pids.write().await.attitude_pid.setpoint = inner_state.heading;
                }
                FlightMode::AltiAtti => {
                    let mut pids = self.pids.write().await;
                    pids.attitude_pid.setpoint = inner_state.heading;
                    pids.altitude_pid.setpoint = inner_state.altitude;
                }
            }
        }
        inner_state.curr_flight_mode = inner_state.controls.desired_flight_mode.clone();

        match inner_state.curr_flight_mode {
            FlightMode::Manual => {
                for i in 0..(4 as u8) {
                    if !inner_state.controls.motors_toggles[i as usize] {
                        // self.perform_action(BlimpAction::SetMotor {
                        //     motor: i,
                        //     speed: 0.0,
                        // })
                        // .await;
                        // self.perform_action(BlimpAction::SetServo {
                        //     servo: 2 * i,
                        //     location: 0.0,
                        // })
                        // .await;
                        // self.perform_action(BlimpAction::SetServo {
                        //     servo: 2 * i + 1,
                        //     location: 0.0,
                        // })
                        // .await;

                        continue;
                    }

                    let speed: f32 = (inner_state.controls.throttle_main
                        + inner_state.controls.throttle_split[i as usize])
                        * (if inner_state.controls.motors_reverse[i as usize] {
                            -1.0
                        } else {
                            1.0
                        })
                        + inner_state.controls.yaw * (if i % 2 == 0 { 1.0 } else { -1.0 });
                    //Motor
                    self.perform_action(BlimpAction::SetMotor { motor: i, speed })
                        .await;
                    // Up-down servo
                    self.perform_action(BlimpAction::SetServo {
                        servo: 2 * i,
                        location: (inner_state.controls.elevation
                            * (if i % 2 == 0 { 1.0 } else { -1.0 }))
                        .clamp(-1.0, 1.0)
                            * 90.0,
                    })
                    .await;
                    //Sideways servo
                    self.perform_action(BlimpAction::SetServo {
                        servo: 2 * i + 1,
                        location: (inner_state.controls.roll
                            * (if i % 2 == 0 { -1.0 } else { 1.0 })
                            + (if i % 2 == 0 { -1.0 } else { -1.0 }))
                        .clamp(-1.0, 1.0)
                            * 90.0,
                    })
                    .await;
                }
            }
            FlightMode::Atti | FlightMode::AltiAtti => {
                let mut pids = self.pids.write().await;
                let delta_time =
                    (tokio::time::Instant::now() - pids.previous_step_time).as_secs_f64();

                let heading = inner_state.heading;
                pids.attitude_pid.setpoint += 0.25 * inner_state.controls.yaw as f64 * delta_time;
                let attitude_pid_result =
                    Some(pids.attitude_pid.update(heading.clone(), delta_time));

                let altitude = inner_state.altitude;
                let altitude_pid_result = if inner_state.curr_flight_mode == FlightMode::AltiAtti {
                    pids.altitude_pid.setpoint +=
                        0.5 * inner_state.controls.elevation as f64 * delta_time;
                    Some(pids.altitude_pid.update(altitude.clone(), delta_time))
                } else {
                    None
                };

                pids.pitch_pid.setpoint = 0.0;
                let pitch_result = pids.pitch_pid.update(inner_state.pitch_roll.0, delta_time);

                pids.roll_pid.setpoint = 0.0;
                let roll_result = pids.roll_pid.update(inner_state.pitch_roll.1, delta_time);

                let mut mdfv = na::Vector3::<f64>::zeros();
                let mut lrfvs = Vec::<na::Vector3<f64>>::new();
                for i in 0..4 {
                    lrfvs.push(na::Vector3::<f64>::zeros());
                    lrfvs[i].y +=
                        inner_state.controls.yaw as f64 * (if i % 2 == 0 { 1.0 } else { -1.0 });
                    // lrfvs[i].z += 0.2;
                    // lrfvs[i].y +=
                    //     attitude_pid_result.unwrap_or(0.0) * (if i % 2 == 0 { 1.0 } else { -1.0 });
                    lrfvs[i].z += pitch_result * (if i >= 2 { -1.0 } else { 1.0 });
                    lrfvs[i].z += roll_result * (if i % 2 == 0 { 1.0 } else { -1.0 });
                }

                mdfv.x += inner_state.controls.sideways as f64;
                mdfv.y += inner_state.controls.throttle_main as f64;
                mdfv.z += if let Some(altitude_pid_result) = altitude_pid_result {
                    altitude_pid_result
                } else {
                    inner_state.controls.elevation as f64
                };

                self.vectored_thrust(mdfv, &lrfvs).await;
            }
        }

        self.perform_action(BlimpAction::NavLights(if inner_state.controls.nav_lights {
            0.5
        } else {
            -1.0
        }))
        .await;

        {
            let mut pids = self.pids.write().await;
            self.perform_action(BlimpAction::SendMsg(Box::new(MessageB2G::BlimpState(
                BlimpState {
                    flight_mode: inner_state.curr_flight_mode.clone(),
                    altitude: inner_state.altitude,
                    desired_altitude: if inner_state.curr_flight_mode == FlightMode::AltiAtti {
                        Some(pids.altitude_pid.setpoint)
                    } else {
                        None
                    },
                    heading: inner_state.heading,
                    desired_heading: if inner_state.curr_flight_mode == FlightMode::Atti
                        || inner_state.curr_flight_mode == FlightMode::AltiAtti
                    {
                        Some(pids.attitude_pid.setpoint)
                    } else {
                        None
                    },
                    pitch: inner_state.pitch_roll.0,
                    roll: inner_state.pitch_roll.1,
                },
            ))))
            .await;
            pids.previous_step_time = tokio::time::Instant::now();
        }
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
            let lfv_xz = (f64::powf(lfv_x, 2.0) + f64::powf(lfv_z, 2.0)).sqrt();
            let lfv_yz = (f64::powf(lfv_y, 2.0) + f64::powf(lfv_z, 2.0)).sqrt();
            let lfv_hor = (f64::powf(lfv_x, 2.0) + f64::powf(lfv_y, 2.0)).sqrt();
            let lfv_magn =
                (f64::powf(lfv_x, 2.0) + f64::powf(lfv_y, 2.0) + f64::powf(lfv_z, 2.0)).sqrt();

            let servo_1_angle =
                f64::atan2(lfv_z, lfv_y) * (if i % 2 == 0 { -1.0 } else { 1.0 }) * 180.0
                    / std::f64::consts::PI;
            self.perform_action(BlimpAction::SetServo {
                servo: 2 * i as u8,
                location: servo_1_angle as f32,
            })
            .await;
            // Sideways servo
            self.perform_action(BlimpAction::SetServo {
                servo: (2 * i + 1) as u8,
                location: (f64::atan2(
                    lfv_yz
                        /* * (if servo_1_angle < 0.0 { -1.0 } else { 1.0 }) */
                        * (if i % 2 == 0 { -1.0 } else { -1.0 }),
                    lfv_x,
                ) * 180.0
                    / std::f64::consts::PI) as f32,
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
