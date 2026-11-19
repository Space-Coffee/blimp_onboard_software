use serde;

#[derive(Clone, Debug, serde::Deserialize)]
pub struct BlimpAlgoConfig {
    pub temperature: f64,
    pub base_pressure: f64,
    pub attitude_pid: PidConfig<f64>,
    pub altitude_pid: PidConfig<f64>,
    pub pitch_pid: PidConfig<f64>,
    pub roll_pid: PidConfig<f64>,
}

#[derive(Clone, Debug, serde::Deserialize)]
pub struct PidConfig<T> {
    pub k_p: T,
    pub k_i: T,
    pub k_d: T,
}
