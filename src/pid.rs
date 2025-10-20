use num_traits;

pub struct PidRegulator<T> {
    pub setpoint: T,
    pub k_p: T,
    pub k_i: T,
    pub k_d: T,

    circular: Option<T>,
    deviation_integral: T,
    previous_deviation: T,
}

impl<T> PidRegulator<T>
where
    T: Clone + num_traits::Num + num_traits::NumAssign + num_traits::NumAssignRef + PartialOrd,
{
    pub fn new(setpoint: T, k_p: T, k_i: T, k_d: T, circular: Option<T>) -> Self {
        Self {
            setpoint,
            k_p,
            k_i,
            k_d,
            circular,
            deviation_integral: T::zero(),
            previous_deviation: T::zero(),
        }
    }

    pub fn update(&mut self, current: T, delta_time: T) -> T {
        let mut current_deviation = self.setpoint.clone() - current;
        if let Some(circular) = &self.circular {
            current_deviation = current_deviation % (circular.clone() + circular.clone());
            if current_deviation > circular.clone() {
                current_deviation -= circular.clone() + circular.clone();
            } else if current_deviation < T::zero() - circular.clone() {
                current_deviation += circular.clone() + circular.clone();
            }
        }

        let prop = current_deviation.clone() * self.k_p.clone();

        self.deviation_integral += current_deviation.clone() * delta_time.clone();
        let inte = self.deviation_integral.clone() * self.k_i.clone();

        let deri = (current_deviation.clone() - self.previous_deviation.clone()) * self.k_d.clone()
            / delta_time;
        self.previous_deviation = current_deviation;

        return prop + inte + deri;
    }
}
