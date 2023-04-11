use std::time::Duration;

fn main() {
    // Sim parameters
    let mut sim = sim::Sim::default();
    let mut controller = Control::default();

    // Desired position, velocity, and acceleration in metric units
    let desired: [f64; 3] = [10.0, 0.0, 0.0];

    // Set the control gains
    controller.set_pos_gains(6.75, 0.85, 4.5);
    controller.set_vel_gains(0.75, 0.25, 0.5); // Currently not used
    controller.set_accel_gains(1.0, 1.0, 1.0); // Currently not used

    loop {
        // Get sensor feedback
        let state: [f64; 3] = [sim.pos(), sim.velocity(), sim.accl()];

        // Run the controller
        let thrust_percentage: f64 = controller.run(state, desired);

        sim.tick(thrust_percentage);
        println!(
            "y = {} v = {} a = {}",
            sim.pos(),
            sim.velocity(),
            sim.accl()
        );
        std::thread::sleep(Duration::from_millis(10))
    }
}

pub struct PIDController {
    e: f64,
    e_prev: f64,
    e_rate: f64,
    e_integral: f64,
    dt: f64,
    kp: f64,
    ki: f64,
    kd: f64,
}

impl Default for PIDController {
    fn default() -> Self {
        Self {
            e: 0.0,
            e_prev: 0.0,
            e_rate: 0.0,
            e_integral: 0.0,
            dt: 0.01,
            kp: 0.0,
            ki: 0.0,
            kd: 0.0,
        }
    }
}

impl PIDController {
    pub fn set_gains(&mut self, kp: f64, ki: f64, kd: f64) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }

    pub fn set_error(&mut self, value: f64, desired: f64) {
        self.e = desired - value;
        self.e_rate = (self.e - self.e_prev) / self.dt;
        self.e_prev = self.e;
        self.e_integral = self.e_integral + self.e * self.dt;
    }

    pub fn calculate_control(&self) -> f64 {
        return self.kp * self.e + self.ki * self.e_integral + self.kd * self.e_rate;
    }
}

pub struct Control {
    position_controller: PIDController,
    velocity_controller: PIDController,
    acceleration_controller: PIDController,
    max_thrust: f64,
}

impl Default for Control {
    fn default() -> Self {
        Self {
            position_controller: PIDController::default(),
            velocity_controller: PIDController::default(),
            acceleration_controller: PIDController::default(),
            max_thrust: 20.0,
        }
    }
}

impl Control {
    pub fn set_pos_gains(&mut self, kp: f64, ki: f64, kd: f64) {
        self.position_controller.set_gains(kp, ki, kd);
    }

    pub fn set_vel_gains(&mut self, kp: f64, ki: f64, kd: f64) {
        self.velocity_controller.set_gains(kp, ki, kd);
    }

    pub fn set_accel_gains(&mut self, kp: f64, ki: f64, kd: f64) {
        self.acceleration_controller.set_gains(kp, ki, kd);
    }

    pub fn run(&mut self, state: [f64; 3], desired: [f64; 3]) -> f64{
        // Determine position control -- outer loop
        let pos_control: f64 = {
            // Calculate pos error states
            self.position_controller.set_error(state[0], desired[0]);
            self.position_controller.calculate_control()
        };

        // Calculate velocity feedforward term
        let vel_ff = -state[1] + pos_control;

        // Calculate velocity error states -- mid level loop
        let vel_feedback: f64 = {
            // Calculate pos error states
            self.velocity_controller.set_error(0.0, 0.0); // Zeroed out -- not in use atm
            self.velocity_controller.calculate_control()
        };

        let vel_control = vel_ff + vel_feedback;

        // Calculate acceleration feedforward term
        let accel_ff = -state[2] + vel_control;

        // Calculate acceleration error states -- inner loop
        let accel_feedback: f64 = {
            // Calculate pos error states
            self.velocity_controller.set_error(0.0, 0.0); // Zeroed out -- not in use atm
            self.velocity_controller.calculate_control()
        };

        let accel_control = accel_ff + accel_feedback;

        // Vehicle weight = 1kg, so accel = thrust
        let thrust: f64 = accel_control;

        // Clamp thrust percentage between 0 and 1
        let mut thrust_percentage: f64 = thrust / self.max_thrust;
        if thrust_percentage > 1.0 {
            thrust_percentage = 1.0;
        } else if thrust_percentage < 0.0 {
            thrust_percentage = 0.0;
        }

        return thrust_percentage;
    }
}
