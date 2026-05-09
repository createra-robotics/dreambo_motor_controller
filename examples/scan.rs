use std::time::Duration;

use dreambo_motor_controller::SERVO_BAUD;

fn main() {
    let port = std::env::args()
        .nth(1)
        .expect("usage: scan <serial_port>");

    let fph = servocom::FeetechProtocolHandler::new();
    let mut serial_port = serialport::new(&port, SERVO_BAUD)
        .timeout(Duration::from_millis(20))
        .open()
        .expect("failed to open serial port");

    let _ = serial_port.clear(serialport::ClearBuffer::Input);
    println!("Scanning IDs 1..=20 at {} baud...", SERVO_BAUD);
    for id in 1u8..=20 {
        match fph.ping(serial_port.as_mut(), id) {
            Ok(true) => println!("  id {id}: OK"),
            Ok(false) => {}
            Err(e) => println!("  id {id}: err {e:?}"),
        }
    }
}