#![no_std]

use num_traits::{self, float::FloatCore};

pub struct PID<T> {
    /// Desired setpoint.
    pub setpoint: T,

    // Values from the previous update.
    error: T,
    integral: T,
    derivative: T,
    measurement: T,

    // Controller gains and constants.
    kp: T,
    ki: T,
    kd: T,
    tc: T,

    /// Lower bound of the integral term.
    imin: T,
    /// Upper bound of the integral term.
    imax: T,

    /// Lower bound of the controller output.
    omin: T,
    /// Upper bound of the controller output.
    omax: T,
}

impl<T: FloatCore> PID<T> {
    pub fn new(kp: T, ki: T, kd: T, tau: T, sampling_time: T, setpoint: T) -> Self {
        let two = T::from(2.0_f32).unwrap();
        let one_half = T::from(0.5_f32).unwrap();

        Self {
            setpoint,

            error: T::zero(),
            integral: T::zero(),
            derivative: T::zero(),
            measurement: T::zero(),

            kp,
            ki: ki * sampling_time * one_half,
            kd: kd * -two,
            tc: (two * tau - sampling_time) / (two * tau + sampling_time),

            imin: T::neg_infinity(),
            imax: T::infinity(),

            omin: T::neg_infinity(),
            omax: T::infinity(),
        }
    }

    pub fn bound_integral(&mut self, min: T, max: T) -> &mut Self {
        assert!(min <= max);
        self.imin = min;
        self.imax = max;
        self
    }

    pub fn bound_output(&mut self, min: T, max: T) -> &mut Self {
        assert!(min <= max);
        self.omin = min;
        self.omax = max;
        self
    }

    pub fn update(&mut self, measurement: T) -> T {
        let error = self.setpoint - measurement;

        let proportional = self.kp * error;
        // Calculate integral term and clamp it to prevent windup.
        let integral = self.ki * (error + self.error) + self.integral;
        self.integral = num_traits::clamp(integral, self.imin, self.imax);
        // Derivative on measurement to prevent a kick during setpoint changes.
        self.derivative = self.kd * (measurement - self.measurement) + self.tc * self.derivative;

        let output = proportional + self.integral + self.derivative;
        num_traits::clamp(output, self.omin, self.omax)
    }
}
