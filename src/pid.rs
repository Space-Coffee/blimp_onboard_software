use num_traits;

pub struct PidRegulator<T> {
    pub setpoint: T,
    pub k_p: T,
    pub k_i: T,
    pub k_d: T,

    deviation_integral: T,
    previous_deviation: T,
}

impl<T> PidRegulator<T>
where
    T: Num,
{
    pub fn update(&mut self, current: T, delta_time: T) -> T {
        let prop = (self.setpoint - current) * self.k_p;

        self.deviation_integral += (current - self.setpoint) * delta_time;
        let inte = self.deviation_integral * self.k_i;

        let current_deviation = current - self.setpoint;
        let deri = (current_deviation - self.previous_deviation) * self.k_d / delta_time;
        self.previous_deviation = current_deviation;

        return prop + inte + deri;
    }
}
