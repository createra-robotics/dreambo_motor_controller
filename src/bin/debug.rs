use std::{thread::sleep, time::Duration};
use dreambo_motor_controller::DreamboMotorController;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // let serial_port = "/dev/ttyACM0"; // Adjust this to your serial port
    let serial_port = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79034031-if00";
    let mut c = DreamboMotorController::new(serial_port)?;

    // [left_pitch, left_yaw, right_pitch, right_yaw, nose_0, nose_1, nose_2]
    let lower = [-0.5, 0.0, -0.5, 0.0, 0.0, 0.0, 0.0];
    let upper = [0.5, 0.0, 0.5, 0.0, 0.2, -0.2, 0.2];

    c.enable_torque()?;

    loop {
        c.set_all_goal_positions(lower)?;
        sleep(Duration::from_millis(1000));
        c.set_all_goal_positions(upper)?;
        sleep(Duration::from_millis(1000));
    }
}
