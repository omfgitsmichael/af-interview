/**
 * Michael Renzetti - Main file AF Interview submission.
 *
 * The controller(s) used are all PID control algorithms augmented with a feedforward component. In the sims current state
 * there are 3 control loops all ran in a cascaded control approach. The outer most loop is the position control loop.
 * The total position control is used as part of the feedforward component of the velocity controller (mid-level), and the
 * total velocity control is used as a part of the feedforward component for the acceleration controller (low-level).
 *
 * The PID controllers were tuned basically using a trial-and-error approach. The velocity (mid-level) and acceleration (low-level)
 * controllers are currently only P controllers because the benefit I assumed I would gain did not seem to be worth the hassle of
 * the amount of tuning I was having to go through given the magnitude of steady-state error I was currently facing. However, for a
 * real system I believe this is the approach I would do to properly control the vechile as the position control would fine tune
 * any position errors, the velocity control would fine tune any velocity errors, and the acceleration control would fine tune any
 * acceleration errors.
 *
 * To test this simulation I used the println command to print the states at each iteration and see how the system was working. I originally
 * was only working with the PID controller and then I added the feedforward term afterward to help cancel out the noise and that improved
 * performance significantly. When debugging I also used the println command. After satisfied with how the system was performing I added the
 * plots for further analysis to help me get a graphical representation of the system. I believe I noticed a bug in the sim code early on.
 * When the position is zero and the acceleration is negative, the velocity with continue to negatively increase even though the vehicle
 * is not moving, i.e., position remains at zero.
 *
 * The steady-state error of the system seems to be generally be around ~+-0.1 meters. Given the magnitude of the disturbances acting on the
 * system I felt that the ~0.1 meters of error was acceptable. The system overshoot seems to be ~0% making the system potentially seem
 * `overdamped` while the system settling time seems to be around 2.5 seconds. The system response is different for various altitude hoverings.
 * This seems to be likely because there is no negative thrust so it overshoots a lot (but seems to stabilize for all cases I have tried).
 * Would likely require gain scheduling for various altitude to accomodate for the lack of negative thrust control. To improve controller
 * fidelity I would likely add anti-windup methods on the integrators and tune the ID terms for the mid and low-level controllers.
**/
use std::time::Duration;

fn main() {
    // Sim parameters
    const T_INIT: f64 = 0.0;
    const T_FINAL: f64 = 15.0;
    const DT: f64 = 0.01;

    let mut control_values = Vec::new();
    let mut height_values = Vec::new();
    let mut index1 = Vec::new();
    let mut index2 = Vec::new();
    let mut time: f64 = T_INIT;

    let mut sim = sim::Sim::default();
    let mut controller = Control::default();

    // Desired position, velocity, and acceleration in metric units
    let desired: [f64; 3] = [10.0, 0.0, 0.0];

    // Set the control gains
    controller.set_pos_gains(6.75, 0.85, 4.5);
    controller.set_vel_gains(0.25, 0.0, 0.0);
    controller.set_accel_gains(0.25, 0.0, 0.0);

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

        control_values.push(thrust_percentage);
        height_values.push(sim.pos());
        index1.push(time);
        index2.push(time);

        time = time + DT;

        if time >= T_FINAL {
            break;
        }

        std::thread::sleep(Duration::from_millis(10))
    }

    let trace1 = plotly::Scatter::new(index1, height_values).name("trace1");
    let trace2 = plotly::Scatter::new(index2, control_values).name("trace2");

    let mut plot1 = plotly::Plot::new();
    plot1.add_trace(trace1);
    let layout1 = plotly::Layout::new().title("Vehicle Height Over Time".into());
    plot1.set_layout(layout1);
    plot1.show();

    let mut plot2 = plotly::Plot::new();
    plot2.add_trace(trace2);
    let layout2 = plotly::Layout::new().title("Control Output Over Time".into());
    plot2.set_layout(layout2);
    plot2.show();
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

    pub fn run(&mut self, state: [f64; 3], desired: [f64; 3]) -> f64 {
        // Determine position control -- outer loop
        let pos_control: f64 = {
            // Calculate pos error states
            self.position_controller.set_error(state[0], desired[0]);
            self.position_controller.calculate_control()
        };

        // Calculate velocity feedforward term
        let vel_ff = desired[1] + pos_control;

        // Calculate velocity error states -- mid level loop
        let vel_feedback: f64 = {
            // Calculate pos error states
            self.velocity_controller.set_error(state[1], desired[1]);
            self.velocity_controller.calculate_control()
        };

        let vel_control = vel_ff + vel_feedback;

        // Calculate acceleration feedforward term
        let accel_ff = desired[2] + vel_control;

        // Calculate acceleration error states -- inner loop
        let accel_feedback: f64 = {
            // Calculate pos error states
            self.velocity_controller.set_error(state[2], desired[2]);
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
