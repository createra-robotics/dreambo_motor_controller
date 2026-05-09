use std::time::Duration;
use dreambo_motor_controller::{
    DreamboMotorController, SM40BL_BAUD, STS3025BL_BAUD, control_loop::read_pos,
};
use servocom::servo::feetech::{sm40bl, sts3025bl};

const N: usize = 1000;

fn main() {
    let serialport = "/dev/ttyACM0";

    {
        let mut c = DreamboMotorController::new(serialport).unwrap();

        let tic = std::time::Instant::now();
        for _ in 0..N {
            let _ = read_pos(&mut c, 3);
        }
        let elapsed = tic.elapsed();
        println!("Full loop read elapsed time: {:?}", elapsed);

        let tic = std::time::Instant::now();
        for _ in 0..N {
            let _ = c.read_all_positions();
        }
        let elapsed = tic.elapsed();
        println!("Controller read all positions elapsed time: {:?}", elapsed);
    }
    {
        let fph = servocom::FeetechProtocolHandler::new();

        let mut serial_port = serialport::new(serialport, SM40BL_BAUD)
            .timeout(Duration::from_millis(10))
            .open()
            .unwrap();

        let tic = std::time::Instant::now();
        for _ in 0..N {
            serial_port.set_baud_rate(SM40BL_BAUD).unwrap();
            let _ = sm40bl::sync_read_present_position(&fph, serial_port.as_mut(), &[1, 2, 3, 4]);
            serial_port.set_baud_rate(STS3025BL_BAUD).unwrap();
            let _ = sts3025bl::sync_read_present_position(&fph, serial_port.as_mut(), &[5, 6, 7]);
        }
        let elapsed = tic.elapsed();
        println!("Feetech sync read (with baud switching) elapsed time: {:?}", elapsed);
    }
}
