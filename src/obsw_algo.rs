use std::pin::Pin;

use postcard;
use serde;
use tokio::sync::RwLock as TRwLock;

use crate::obsw_interface::*;

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
pub struct Controls {
    pub throttle: i32,
    pub elevation: i32,
    pub yaw: i32,
}

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
pub enum BlimpAction {
    SetServo { servo: u8, location: i16 },
    SetMotor { motor: u8, speed: i32 },
    SendMsg(Vec<u8>),
}

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
pub enum SensorType {
    Barometer,
    GPSLatitude,
    GPSLongitude,
    GPSAltitude,
}

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
pub enum BlimpEvent {
    Control(Controls),
    GetMsg(Vec<u8>),
    SensorDataF64(SensorType, f64),
}

#[derive(Debug)]
pub enum FlightMode {
    Manual,            // Throttle -> motors speed; Pitch -> motors pitch; Roll -> motors yaw
    StabilizeAttiAlti, // Maintain altitude and attitude/azimuth
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
}

pub struct BlimpMainAlgo {
    action_callback: TRwLock<
        Option<
            Box<
                dyn Fn(BlimpAction) -> Pin<Box<dyn std::future::Future<Output = ()> + Send + Sync>>
                    + Send,
            >,
        >,
    >,
    curr_flight_mode: TRwLock<FlightMode>,
    controls: TRwLock<Controls>,
    altitude: TRwLock<Option<f64>>,
    gps_location: TRwLock<Option<(f64, f64)>>,
}

impl BlimpAlgorithm<BlimpEvent, BlimpAction> for BlimpMainAlgo {
    fn handle_event(&self, ev: BlimpEvent) -> Pin<Box<impl std::future::Future<Output = ()>>> {
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
                BlimpEvent::SensorDataF64(SensorType::GPSLatitude, latitude) => {
                    let prev_long = self.gps_location.read().await.unwrap_or((0.0, 0.0)).1;
                    *self.gps_location.write().await = Some((*latitude, prev_long));
                }
                BlimpEvent::SensorDataF64(SensorType::GPSLongitude, longitude) => {
                    let prev_lat = self.gps_location.read().await.unwrap_or((0.0, 0.0)).0;
                    *self.gps_location.write().await = Some((prev_lat, *longitude));
                }
                BlimpEvent::GetMsg(msg) => {
                    if let Ok(msg_deserialized) = postcard::from_bytes::<MessageG2B>(&msg) {
                        match msg_deserialized {
                            MessageG2B::Ping(id) => {
                                if let Some(fut) =
                                    self.action_callback.read().await.as_ref().map(|x| async {
                                        self.perform_action(
                                            x.as_ref(),
                                            BlimpAction::SendMsg(
                                                postcard::to_stdvec::<MessageB2G>(
                                                    &MessageB2G::Pong(id),
                                                )
                                                .unwrap(),
                                            ),
                                        )
                                        .await
                                    })
                                {
                                    fut.await;
                                }
                            }
                            MessageG2B::Pong(_id) => {}
                            MessageG2B::Control(ctrl) => {
                                self.handle_event(BlimpEvent::Control(ctrl)).await;
                            }
                        }
                    } else {
                        eprintln!("Error occurred while deseerializing message");
                    }
                }
                _ => {}
            }
            if matches!(&ev, BlimpEvent::SensorDataF64(..)) {
                if let Some(fut) = self.action_callback.read().await.as_ref().map(|x| async {
                    self.perform_action(
                        x,
                        BlimpAction::SendMsg(
                            postcard::to_stdvec::<MessageB2G>(&MessageB2G::ForwardEvent(
                                ev.clone(),
                            ))
                            .unwrap(),
                        ),
                    )
                    .await
                }) {
                    fut.await;
                }
            }
        })
    }

    fn set_action_callback(
        &mut self,
        callback: Box<
            dyn Fn(BlimpAction) -> Pin<Box<dyn std::future::Future<Output = ()> + Send + Sync>>
                + Send,
        >,
    ) {
        //TODO: decide if this should be async too
        *self.action_callback.blocking_write() = Some(callback);
    }
}

impl BlimpMainAlgo {
    pub fn new() -> Self {
        Self {
            action_callback: TRwLock::new(None),
            curr_flight_mode: TRwLock::new(FlightMode::Manual),
            controls: TRwLock::new(Controls {
                throttle: 0,
                elevation: 0,
                yaw: 0,
            }),
            altitude: TRwLock::new(None),
            gps_location: TRwLock::new(None),
        }
    }

    pub async fn step(&mut self) {
        let curr_flight_mode = self.curr_flight_mode.read().await;
        match *curr_flight_mode {
            FlightMode::Manual => {
                if let Some(fut) = self.action_callback.read().await.as_ref().map(|x| {
                    async {
                        for i in 0..4 {
                            let controls = self.controls.read().await;
                            let speed: i32 = controls.throttle
                                + (if i % 2 == 0 { 1 } else { -1 }) * controls.yaw
                                + controls.elevation;
                            //Motor
                            self.perform_action(
                                x.as_ref(),
                                BlimpAction::SetMotor { motor: i, speed },
                            )
                            .await;
                            // Up-down servo
                            self.perform_action(
                                x.as_ref(),
                                BlimpAction::SetServo {
                                    servo: 2 * i,
                                    location: controls.elevation as i16,
                                },
                            )
                            .await;
                            //Sideways servo
                            self.perform_action(
                                x.as_ref(),
                                BlimpAction::SetServo {
                                    servo: 2 * i + 1,
                                    location: controls.yaw as i16,
                                },
                            )
                            .await;
                        }
                    }
                }) {
                    fut.await;
                }
            }
            FlightMode::StabilizeAttiAlti => {}
        }
    }

    async fn perform_action(
        &self,
        action_callback: &dyn Fn(
            BlimpAction,
        )
            -> Pin<Box<dyn std::future::Future<Output = ()> + Send + Sync>>,
        action: BlimpAction,
    ) {
        action_callback(action.clone()).await;

        // Some actions should be forwarded
        if matches!(
            action,
            BlimpAction::SetMotor { .. } | BlimpAction::SetServo { .. }
        ) {
            action_callback(BlimpAction::SendMsg(
                postcard::to_stdvec::<MessageB2G>(&MessageB2G::ForwardAction(action)).unwrap(),
            ))
            .await;
        }
    }
}
