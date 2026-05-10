use std::{collections::HashMap, time::Duration};
use log::warn;
use servocom::servo::feetech::{sm40bl, sts3025bl};

pub const SERVO_BAUD: u32 = 1_000_000;

pub struct DreamboMotorController {
    protocol: servocom::FeetechProtocolHandler,
    port: Box<dyn serialport::SerialPort>,
    all_ids: [u8; 7],
}

const LEFT_ARM_IDS: [u8; 2] = [1, 2]; // pitch, yaw
const RIGHT_ARM_IDS: [u8; 2] = [3, 4]; // pitch, yaw
const ARM_IDS: [u8; 4] = [1, 2, 3, 4];
const NOSE_IDS: [u8; 3] = [5, 6, 7];

impl DreamboMotorController {
    pub fn new(port: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let protocol = servocom::FeetechProtocolHandler::new();
        let port = serialport::new(port, SERVO_BAUD)
            .timeout(Duration::from_millis(10))
            .open()?;
        let all_ids = [
            ARM_IDS[0], ARM_IDS[1], ARM_IDS[2], ARM_IDS[3],
            NOSE_IDS[0], NOSE_IDS[1], NOSE_IDS[2],
        ];
        Ok(Self {
            protocol,
            port,
            all_ids,
        })
    }

    pub fn get_motor_name_id(&self) -> HashMap<String, u8> {
        let mut motor_id_name = HashMap::new();
        motor_id_name.insert("left_arm_pitch".to_string(), LEFT_ARM_IDS[0]);
        motor_id_name.insert("left_arm_yaw".to_string(), LEFT_ARM_IDS[1]);
        motor_id_name.insert("right_arm_pitch".to_string(), RIGHT_ARM_IDS[0]);
        motor_id_name.insert("right_arm_yaw".to_string(), RIGHT_ARM_IDS[1]);
        motor_id_name.insert("nose_top".to_string(), NOSE_IDS[0]);
        motor_id_name.insert("nose_left".to_string(), NOSE_IDS[1]);
        motor_id_name.insert("nose_right".to_string(), NOSE_IDS[2]);
        motor_id_name
    }

    pub fn reboot(
        &mut self,
        reboot_timeout: Duration,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let faulty_ids: Vec<u8> = self.all_ids.to_vec();
        let name2id = self.get_motor_name_id();
        let id2name: HashMap<u8, String> = name2id.into_iter().map(|(name, id)| (id, name)).collect();

        for id in &faulty_ids {
            let name = id2name.get(id).unwrap();
            warn!("Rebooting motor {} (id={})", name, id);
            self.protocol.reboot(self.port.as_mut(), *id)?;
        }

        let mut missing_ids = faulty_ids.clone();
        let start_time = std::time::Instant::now();
        while !missing_ids.is_empty() && start_time.elapsed() < reboot_timeout {
            std::thread::sleep(Duration::from_millis(100));
            missing_ids = missing_ids
                .into_iter()
                .filter(|id| {
                    let ping_result = self.protocol.ping(self.port.as_mut(), *id);
                    match ping_result {
                        Ok(res) => !res,
                        Err(_) => true,
                    }
                })
                .collect();
        }
        for id in &missing_ids {
            let name = id2name.get(id).unwrap();
            warn!(
                "Motor {} (id={}) did not respond after reboot within timeout",
                name, id
            );
        }

        if missing_ids.is_empty() {
            Ok(())
        } else {
            let names = missing_ids
                .iter()
                .map(|id| id2name.get(id).unwrap().to_string())
                .collect::<Vec<String>>();
            Err(format!(
                "Some motors did not respond after reboot ({:?} - ids: {:?})",
                names, missing_ids
            )
            .into())
        }
    }

    pub fn check_missing_ids(&mut self) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
        let mut missing_ids = Vec::new();

        for id in self.all_ids {
            match self.protocol.ping(self.port.as_mut(), id) {
                Ok(true) => {}
                _ => missing_ids.push(id),
            }
        }

        Ok(missing_ids)
    }

    /// Read the current input voltage of all servos.
    /// Returns an array of 7 voltages (in 0.1V units) in the following order:
    /// [left_arm_pitch, left_arm_yaw, right_arm_pitch, right_arm_yaw, nose_top, nose_left, nose_right]
    pub fn read_all_voltages(&mut self) -> Result<[u8; 7], Box<dyn std::error::Error>> {
        let arm_volts = sm40bl::sync_read_present_voltage(
            &self.protocol,
            self.port.as_mut(),
            &ARM_IDS,
        )?;
        let nose_volts = sts3025bl::sync_read_present_voltage(
            &self.protocol,
            self.port.as_mut(),
            &NOSE_IDS,
        )?;

        Ok([
            arm_volts[0], arm_volts[1], arm_volts[2], arm_volts[3],
            nose_volts[0], nose_volts[1], nose_volts[2],
        ])
    }

    /// Read the current position of all servos.
    /// Returns an array of 7 positions in the following order:
    /// [left_arm_pitch, left_arm_yaw, right_arm_pitch, right_arm_yaw, nose_top, nose_left, nose_right]
    pub fn read_all_positions(&mut self) -> Result<[f64; 7], Box<dyn std::error::Error>> {
        let arm_pos = sm40bl::sync_read_present_position(
            &self.protocol,
            self.port.as_mut(),
            &ARM_IDS,
        )?;
        let nose_pos = sts3025bl::sync_read_present_position(
            &self.protocol,
            self.port.as_mut(),
            &NOSE_IDS,
        )?;

        Ok([
            arm_pos[0], arm_pos[1], arm_pos[2], arm_pos[3],
            nose_pos[0], nose_pos[1], nose_pos[2],
        ])
    }

    /// Set the goal position of all servos.
    /// The positions array must be in the following order:
    /// [left_arm_pitch, left_arm_yaw, right_arm_pitch, right_arm_yaw, nose_top, nose_left, nose_right]
    pub fn set_all_goal_positions(
        &mut self,
        positions: [f64; 7],
    ) -> Result<(), Box<dyn std::error::Error>> {
        sm40bl::sync_write_goal_position(
            &self.protocol,
            self.port.as_mut(),
            &ARM_IDS,
            &[positions[0], positions[1], positions[2], positions[3]],
        )?;
        sts3025bl::sync_write_goal_position(
            &self.protocol,
            self.port.as_mut(),
            &NOSE_IDS,
            &[positions[4], positions[5], positions[6]],
        )?;
        Ok(())
    }

    pub fn set_left_arm_position(
        &mut self,
        position: [f64; 2],
    ) -> Result<(), Box<dyn std::error::Error>> {
        sm40bl::sync_write_goal_position(
            &self.protocol,
            self.port.as_mut(),
            &LEFT_ARM_IDS,
            &position,
        )?;
        Ok(())
    }

    pub fn set_right_arm_position(
        &mut self,
        position: [f64; 2],
    ) -> Result<(), Box<dyn std::error::Error>> {
        sm40bl::sync_write_goal_position(
            &self.protocol,
            self.port.as_mut(),
            &RIGHT_ARM_IDS,
            &position,
        )?;
        Ok(())
    }

    pub fn set_arms_position(
        &mut self,
        position: [f64; 4],
    ) -> Result<(), Box<dyn std::error::Error>> {
        sm40bl::sync_write_goal_position(
            &self.protocol,
            self.port.as_mut(),
            &ARM_IDS,
            &position,
        )?;
        Ok(())
    }

    pub fn set_nose_position(
        &mut self,
        position: [f64; 3],
    ) -> Result<(), Box<dyn std::error::Error>> {
        sts3025bl::sync_write_goal_position(
            &self.protocol,
            self.port.as_mut(),
            &NOSE_IDS,
            &position,
        )?;
        Ok(())
    }

    pub fn is_torque_enabled(&mut self) -> Result<bool, Box<dyn std::error::Error>> {
        let arm_torque = sm40bl::sync_read_torque_enable(
            &self.protocol,
            self.port.as_mut(),
            &ARM_IDS,
        )?;
        let nose_torque = sts3025bl::sync_read_torque_enable(
            &self.protocol,
            self.port.as_mut(),
            &NOSE_IDS,
        )?;

        Ok(arm_torque.iter().chain(nose_torque.iter()).all(|&x| x))
    }

    pub fn enable_torque(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        self.set_torque(true)
    }

    pub fn enable_torque_on_ids(&mut self, ids: &[u8]) -> Result<(), Box<dyn std::error::Error>> {
        self.set_torque_on_ids(ids, true)
    }

    pub fn disable_torque(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        self.set_torque(false)
    }

    pub fn disable_torque_on_ids(&mut self, ids: &[u8]) -> Result<(), Box<dyn std::error::Error>> {
        self.set_torque_on_ids(ids, false)
    }

    pub fn enable_arms(&mut self, enable: bool) -> Result<(), Box<dyn std::error::Error>> {
        sm40bl::sync_write_torque_enable(
            &self.protocol,
            self.port.as_mut(),
            &ARM_IDS,
            &[enable; 4],
        )?;
        Ok(())
    }

    pub fn enable_nose(&mut self, enable: bool) -> Result<(), Box<dyn std::error::Error>> {
        sts3025bl::sync_write_torque_enable(
            &self.protocol,
            self.port.as_mut(),
            &NOSE_IDS,
            &[enable; 3],
        )?;
        Ok(())
    }

    fn set_torque(&mut self, enable: bool) -> Result<(), Box<dyn std::error::Error>> {
        sm40bl::sync_write_torque_enable(
            &self.protocol,
            self.port.as_mut(),
            &ARM_IDS,
            &[enable; 4],
        )?;
        sts3025bl::sync_write_torque_enable(
            &self.protocol,
            self.port.as_mut(),
            &NOSE_IDS,
            &[enable; 3],
        )?;
        Ok(())
    }

    fn set_torque_on_ids(
        &mut self,
        ids: &[u8],
        enable: bool,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let (arm_targets, nose_targets) = split_ids_by_family(ids);

        if !arm_targets.is_empty() {
            let enables = vec![enable; arm_targets.len()];
            sm40bl::sync_write_torque_enable(
                &self.protocol,
                self.port.as_mut(),
                &arm_targets,
                &enables,
            )?;
        }
        if !nose_targets.is_empty() {
            let enables = vec![enable; nose_targets.len()];
            sts3025bl::sync_write_torque_enable(
                &self.protocol,
                self.port.as_mut(),
                &nose_targets,
                &enables,
            )?;
        }
        Ok(())
    }

    pub fn read_raw_bytes(
        &mut self,
        id: u8,
        address: u8,
        length: u8,
    ) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
        self.protocol.read(self.port.as_mut(), id, address, length)
    }

    pub fn write_raw_bytes(
        &mut self,
        id: u8,
        address: u8,
        data: &[u8],
    ) -> Result<(), Box<dyn std::error::Error>> {
        self.protocol
            .write(self.port.as_mut(), id, address, data)
    }

    pub fn write_raw_packet(&mut self, data: &[u8]) -> Result<Vec<u8>, std::io::Error> {
        self.port.write_all(data)?;
        self.port.flush()?;

        let mut n = self.port.bytes_to_read()? as usize;
        let start = std::time::Instant::now();
        while n == 0 && start.elapsed() < Duration::from_millis(10) {
            std::thread::sleep(Duration::from_millis(5));
            n = self.port.bytes_to_read()? as usize;
        }
        let mut buff = vec![0u8; n];
        self.port.read_exact(&mut buff)?;

        Ok(buff)
    }
}

fn split_ids_by_family(ids: &[u8]) -> (Vec<u8>, Vec<u8>) {
    let mut arms = Vec::new();
    let mut nose = Vec::new();
    for &id in ids {
        if ARM_IDS.contains(&id) {
            arms.push(id);
        } else if NOSE_IDS.contains(&id) {
            nose.push(id);
        }
    }
    (arms, nose)
}