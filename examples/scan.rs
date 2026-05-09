use std::time::Duration;

use dreambo_motor_controller::{SM40BL_BAUD, STS3025BL_BAUD};

fn main() {
    let port = std::env::args()
        .nth(1)
        .expect("usage: scan <serial_port>");

    let fph = servocom::FeetechProtocolHandler::new();
    let mut serial_port = serialport::new(&port, SM40BL_BAUD)
        .timeout(Duration::from_millis(20))
        .open()
        .expect("failed to open serial port");

    for &baud in &[SM40BL_BAUD, STS3025BL_BAUD] {
        serial_port.set_baud_rate(baud).expect("set_baud_rate failed");
        let _ = serial_port.clear(serialport::ClearBuffer::Input);
        println!("Scanning IDs 1..=20 at {} baud...", baud);
        for id in 1u8..=20 {
            match fph.ping(serial_port.as_mut(), id) {
                Ok(true) => println!("  id {id}: OK"),
                Ok(false) => {}
                Err(e) => println!("  id {id}: err {e:?}"),
            }
        }
    }
}
