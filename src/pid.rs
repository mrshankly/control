use num_traits::float::FloatCore;

/// Implementation of a proportional–integral–derivative controller.
pub struct PID<T> {
    /// Desired setpoint.
    pub setpoint: T,

    /// Error value from the previous update.
    error: T,
    /// Integral value from the previous update.
    integral: T,
    /// Derivative value from the previous update.
    derivative: T,
    /// Previous measurement value.
    measurement: T,

    /// Proportional coefficient.
    p: T,
    /// Integral coefficient.
    i: T,
    /// Derivative coefficient.
    d: T,
    /// Coefficient for the derivative low-pass filter.
    t: T,

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
    /// Creates a new `PID` with a proportional gain of `kp`, integral gain of `ki`
    /// and derivative gain of `kd`.
    ///
    /// `tau` represents the time constant of the derivative low-pass filter.
    ///
    /// `sampling_time` is the time difference in seconds between two consecutive
    /// step operations.
    ///
    /// # Examples
    ///
    /// ```
    ///
    /// ```
    pub fn new(kp: T, ki: T, kd: T, tau: T, sampling_time: T, setpoint: T) -> Self {
        let two = T::from(2.0_f32).expect("Unable to cast from 2.0");
        let half = T::from(0.5_f32).expect("Unable to cast from 0.5");

        Self {
            setpoint,

            error: T::zero(),
            integral: T::zero(),
            derivative: T::zero(),
            measurement: T::zero(),

            p: kp,
            i: half * ki * sampling_time,
            d: -two * kd,
            t: (two * tau - sampling_time) / (two * tau + sampling_time),

            imin: T::neg_infinity(),
            imax: T::infinity(),

            omin: T::neg_infinity(),
            omax: T::infinity(),
        }
    }

    /// Indicates that the integral term should be restricted to a certain interval.
    /// Useful to prevent [integral windup].
    ///
    /// [integral windup]: https://en.wikipedia.org/wiki/PID_controller#Integral_windup
    ///
    /// # Panics
    ///
    /// Panics if `min` > `max`.
    ///
    /// # Examples
    ///
    /// ```
    ///
    /// ```
    pub fn bound_integral(&mut self, min: T, max: T) -> &mut Self {
        assert!(min <= max);
        self.imin = min;
        self.imax = max;
        self
    }

    /// Indicates that the controller output should be restricted to a certain interval.
    ///
    /// # Panics
    ///
    /// Panics if `min` > `max`.
    ///
    /// # Examples
    ///
    /// ```
    ///
    /// ```
    pub fn bound_output(&mut self, min: T, max: T) -> &mut Self {
        assert!(min <= max);
        self.omin = min;
        self.omax = max;
        self
    }

    /// Performs a single step of the control loop. It should be called exactly
    /// once every `sampling_time` seconds.
    pub fn step(&mut self, measurement: T) -> T {
        let error = self.setpoint - measurement;

        let proportional = self.p * error;
        // Calculate integral term and clamp it to prevent windup.
        let integral = self.i * (error + self.error) + self.integral;
        self.integral = num_traits::clamp(integral, self.imin, self.imax);
        // Derivative on measurement to prevent a kick during setpoint changes.
        self.derivative = self.d * (measurement - self.measurement) + self.t * self.derivative;

        let output = proportional + self.integral + self.derivative;
        num_traits::clamp(output, self.omin, self.omax)
    }
}
