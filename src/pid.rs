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
    T: Clone + num_traits::Num + num_traits::NumAssign + num_traits::NumAssignRef,
{
    pub fn new(setpoint: T, k_p: T, k_i: T, k_d: T) -> Self {
        Self {
            setpoint,
            k_p,
            k_i,
            k_d,
            deviation_integral: T::zero(),
            previous_deviation: T::zero(),
        }
    }

    pub fn update(&mut self, current: T, delta_time: T) -> T {
        let current_deviation = self.setpoint.clone() - current;

        let prop = current_deviation.clone() * self.k_p.clone();

        self.deviation_integral += current_deviation.clone() * delta_time.clone();
        let inte = self.deviation_integral.clone() * self.k_i.clone();

        let deri = (current_deviation.clone() - self.previous_deviation.clone()) * self.k_d.clone()
            / delta_time;
        self.previous_deviation = current_deviation;

        return prop + inte + deri;
    }
}
