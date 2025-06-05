use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;

use serde;
use tokio::sync::RwLock as TRwLock;

use crate::obsw_interface::*;

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
pub struct Controls {
    pub throttle_main: f32,       // Generally influences speed
    pub throttle_split: [f32; 4], // Allows you to steer motors individually
    pub sideways: f32,            // Left/right linear movement
    pub elevation: f32,           // Up-down motion
    pub pitch: f32,               // Rotate forward-backward
    pub roll: f32,                // Roll left/right
    pub yaw: f32,                 // Rotate left/right - change heading
    pub desired_flight_mode: Option<FlightMode>,
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
    Accelerometer,
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

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub enum FlightMode {
    Manual,   // Throttle -> motors speed; Pitch -> motors pitch; Roll -> motors yaw
    Atti,     // Stabilize heading, control thrust vector
    AltiAtti, // Like Atti, but also stabilize altitude
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
                throttle: 0,
                elevation: 0,
                yaw: 0,
            }),
            altitude: TRwLock::new(None),
            gps_location: TRwLock::new(None),
        }
    }

    pub async fn step(&self) {
        let curr_flight_mode = self.curr_flight_mode.read().await;
        match *curr_flight_mode {
            FlightMode::Manual => {
                for i in 0..4 {
                    let controls = self.controls.read().await;
                    let speed: f32 = controls.throttle_split[i]
                        + (if i % 2 == 0 { 1.0 } else { -1.0 }) * controls.yaw;
                    //Motor
                    self.perform_action(BlimpAction::SetMotor { motor: i, speed })
                        .await;
                    // Up-down servo
                    self.perform_action(BlimpAction::SetServo {
                        servo: 2 * i,
                        location: controls.elevation as i16,
                    })
                    .await;
                    //Sideways servo
                    self.perform_action(BlimpAction::SetServo {
                        servo: 2 * i + 1,
                        location: controls.yaw as i16,
                    })
                    .await;
                }
            }
            FlightMode::AltiAtti => {}
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
}
