use std::{collections::HashMap, time::Duration};
use log::warn;
use motorcom::damiao::{
    mit::{MitLimits, MitSetpoint},
    ControlMode, ControlSetpoint, Damiao, DM4310, DM4340P,
};
use motorcom::transport::SocketCanTransport;
use servocom::servo::feetech::{sm40bl, sts3025bl};

pub const SERVO_BAUD: u32 = 1_000_000;
pub const DEFAULT_CAN_BUS: &str = "can0";

const LEFT_ARM_IDS: [u8; 2] = [1, 2]; // pitch, yaw
const RIGHT_ARM_IDS: [u8; 2] = [3, 4]; // pitch, yaw
const ARM_IDS: [u8; 4] = [1, 2, 3, 4];
const NOSE_IDS: [u8; 3] = [5, 6, 7];

const NECK_YAW: usize = 0;
const NECK_PITCH: usize = 1;
const NECK_ROLL: usize = 2;
const NECK_NAMES: [&str; 3] = ["neck_yaw", "neck_pitch", "neck_roll"];

/// Per-joint Damiao config: `(can_id, master_id, MIT limits, (kp, kd))`.
///
/// IDs and gains match the test program's `DM_MOTORS` / `DM_GAINS` tables:
///   yaw   = DM-J4310-2EC  kp=30 kd=1
///   pitch = DM-J4340P-2EC kp=60 kd=2
///   roll  = DM-J4340P-2EC kp=60 kd=2
const NECK_CONFIGS: [(u16, u16, MitLimits, f32, f32); 3] = [
    (1, 1, DM4310, 30.0, 1.0),
    (2, 2, DM4340P, 60.0, 2.0),
    (3, 3, DM4340P, 60.0, 2.0),
];

/// Safe per-joint position range (rad), offsets from the homed zero.
/// Every neck setter clamps to these bounds before commanding the motor,
/// so callers can't accidentally drive the head past its mechanical envelope
/// — even motorcom's own ±p_max (12.5 rad) is far wider than what the
/// dreambo head can physically tolerate.
///
/// Order: [yaw, pitch, roll]. Values mirror the test program's `DM_MOTORS`
/// table.
pub const NECK_POSITION_LIMITS: [(f64, f64); 3] = [
    (-0.5, 0.5),  // yaw   — DM-J4310-2EC
    (0.0, 0.45),  // pitch — DM-J4340P-2EC (asymmetric: head can't tilt up past 0)
    (-0.4, 0.4),  // roll  — DM-J4340P-2EC
];

pub struct DreamboMotorController {
    protocol: servocom::FeetechProtocolHandler,
    port: Option<Box<dyn serialport::SerialPort>>,
    all_ids: [u8; 7],

    can_bus: SocketCanTransport,
    neck_motors: [Damiao; 3],
    neck_setpoints: [MitSetpoint; 3],
    neck_torque_enabled: bool,
}

impl DreamboMotorController {
    /// Construct a controller bound to the given serial port (arms+nose Feetech
    /// bus) and CAN bus (neck Damiao motors).
    ///
    /// Pass `port = None` to build a neck-only controller — useful for smoke
    /// tests on the CAN side without the Feetech hardware attached. Methods
    /// that touch the servo bus (`set_*_arm_position`, `set_nose_position`,
    /// `read_all_*`, `enable_torque`, ...) will return an error in that case.
    pub fn new(port: Option<&str>, can_bus: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let protocol = servocom::FeetechProtocolHandler::new();
        let port = match port {
            Some(p) => Some(
                serialport::new(p, SERVO_BAUD)
                    .timeout(Duration::from_millis(10))
                    .open()?,
            ),
            None => None,
        };
        let all_ids = [
            ARM_IDS[0], ARM_IDS[1], ARM_IDS[2], ARM_IDS[3],
            NOSE_IDS[0], NOSE_IDS[1], NOSE_IDS[2],
        ];

        let can_transport = SocketCanTransport::open(can_bus)
            .map_err(|e| format!("failed to open CAN bus '{}': {}", can_bus, e))?;

        let neck_motors: [Damiao; 3] = [
            Damiao::new(NECK_CONFIGS[0].0, NECK_CONFIGS[0].1).with_limits(NECK_CONFIGS[0].2),
            Damiao::new(NECK_CONFIGS[1].0, NECK_CONFIGS[1].1).with_limits(NECK_CONFIGS[1].2),
            Damiao::new(NECK_CONFIGS[2].0, NECK_CONFIGS[2].1).with_limits(NECK_CONFIGS[2].2),
        ];
        let neck_setpoints: [MitSetpoint; 3] = [
            MitSetpoint { q: 0.0, dq: 0.0, kp: NECK_CONFIGS[0].3, kd: NECK_CONFIGS[0].4, tau: 0.0 },
            MitSetpoint { q: 0.0, dq: 0.0, kp: NECK_CONFIGS[1].3, kd: NECK_CONFIGS[1].4, tau: 0.0 },
            MitSetpoint { q: 0.0, dq: 0.0, kp: NECK_CONFIGS[2].3, kd: NECK_CONFIGS[2].4, tau: 0.0 },
        ];

        Ok(Self {
            protocol,
            port,
            all_ids,
            can_bus: can_transport,
            neck_motors,
            neck_setpoints,
            neck_torque_enabled: false,
        })
    }

    /// Whether the servo (Feetech) bus is configured. When `false`, all
    /// arm/nose methods will return an error.
    pub fn has_servo_bus(&self) -> bool {
        self.port.is_some()
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

    /// CAN-side motor identifiers — keyed by joint name, value is the
    /// Damiao `motor_id` (CAN arbitration id the motor listens on).
    pub fn get_neck_motor_name_id(&self) -> HashMap<String, u16> {
        let mut name_id = HashMap::new();
        for (i, name) in NECK_NAMES.iter().enumerate() {
            name_id.insert(name.to_string(), self.neck_motors[i].motor_id);
        }
        name_id
    }

    pub fn reboot(
        &mut self,
        reboot_timeout: Duration,
    ) -> Result<(), Box<dyn std::error::Error>> {
        if self.port.is_none() {
            // No servo bus to reboot. Caller wants neck-only mode.
            return Ok(());
        }
        let faulty_ids: Vec<u8> = self.all_ids.to_vec();
        let name2id = self.get_motor_name_id();
        let id2name: HashMap<u8, String> = name2id.into_iter().map(|(name, id)| (id, name)).collect();

        for id in &faulty_ids {
            let name = id2name.get(id).unwrap();
            warn!("Rebooting motor {} (id={})", name, id);
            let port = port_or_err(&mut self.port)?;
            self.protocol.reboot(port, *id)?;
        }

        let mut missing_ids = faulty_ids.clone();
        let start_time = std::time::Instant::now();
        while !missing_ids.is_empty() && start_time.elapsed() < reboot_timeout {
            std::thread::sleep(Duration::from_millis(100));
            let port = port_or_err(&mut self.port)?;
            let protocol = &self.protocol;
            missing_ids = missing_ids
                .into_iter()
                .filter(|id| {
                    let ping_result = protocol.ping(port, *id);
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
        if self.port.is_none() {
            return Ok(Vec::new());
        }
        let mut missing_ids = Vec::new();
        let all_ids = self.all_ids;
        let port = port_or_err(&mut self.port)?;
        let protocol = &self.protocol;
        for id in all_ids {
            match protocol.ping(port, id) {
                Ok(true) => {}
                _ => missing_ids.push(id),
            }
        }

        Ok(missing_ids)
    }

    /// Returns the names of neck joints that did not respond to a feedback
    /// request. An empty vector means all three are reachable on the CAN bus.
    pub fn check_missing_neck_motors(&mut self) -> Vec<String> {
        let mut missing = Vec::new();
        for (i, dm) in self.neck_motors.iter().enumerate() {
            if dm.request_feedback(&mut self.can_bus).is_err() {
                missing.push(NECK_NAMES[i].to_string());
            }
        }
        missing
    }

    /// Clear faults and re-enable MIT control mode on each neck motor.
    /// Mirrors `reboot()` for the servo bus; safe to call before `enable_neck(true)`.
    pub fn reboot_neck(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        for (i, dm) in self.neck_motors.iter().enumerate() {
            if let Err(e) = dm.clear_error(&mut self.can_bus) {
                warn!("Failed to clear errors on {}: {}", NECK_NAMES[i], e);
            }
            dm.ensure_control_mode(&mut self.can_bus, ControlMode::Mit)
                .map_err(|e| format!("{}: ensure MIT mode failed: {}", NECK_NAMES[i], e))?;
        }
        self.neck_torque_enabled = false;
        Ok(())
    }

    /// Read the current input voltage of all servos.
    /// Returns an array of 7 voltages (in 0.1V units) in the following order:
    /// [left_arm_pitch, left_arm_yaw, right_arm_pitch, right_arm_yaw, nose_top, nose_left, nose_right]
    pub fn read_all_voltages(&mut self) -> Result<[u8; 7], Box<dyn std::error::Error>> {
        let port = port_or_err(&mut self.port)?;
        let arm_volts = sm40bl::sync_read_present_voltage(&self.protocol, port, &ARM_IDS)?;
        let nose_volts = sts3025bl::sync_read_present_voltage(&self.protocol, port, &NOSE_IDS)?;

        Ok([
            arm_volts[0], arm_volts[1], arm_volts[2], arm_volts[3],
            nose_volts[0], nose_volts[1], nose_volts[2],
        ])
    }

    /// Read the current position of all servos.
    /// Returns an array of 7 positions in the following order:
    /// [left_arm_pitch, left_arm_yaw, right_arm_pitch, right_arm_yaw, nose_top, nose_left, nose_right]
    pub fn read_all_positions(&mut self) -> Result<[f64; 7], Box<dyn std::error::Error>> {
        let port = port_or_err(&mut self.port)?;
        let arm_pos = sm40bl::sync_read_present_position(&self.protocol, port, &ARM_IDS)?;
        let nose_pos = sts3025bl::sync_read_present_position(&self.protocol, port, &NOSE_IDS)?;

        Ok([
            arm_pos[0], arm_pos[1], arm_pos[2], arm_pos[3],
            nose_pos[0], nose_pos[1], nose_pos[2],
        ])
    }

    /// Read the current position of each neck joint via Damiao feedback
    /// requests. Order: [yaw, pitch, roll].
    pub fn read_neck_positions(&mut self) -> Result<[f64; 3], Box<dyn std::error::Error>> {
        let mut out = [0.0_f64; 3];
        for (i, dm) in self.neck_motors.iter().enumerate() {
            let state = dm.request_feedback(&mut self.can_bus).map_err(|e| {
                format!("read_neck_positions({}): {}", NECK_NAMES[i], e)
            })?;
            out[i] = state.position as f64;
        }
        Ok(out)
    }

    /// Set the goal position of all servos.
    /// The positions array must be in the following order:
    /// [left_arm_pitch, left_arm_yaw, right_arm_pitch, right_arm_yaw, nose_top, nose_left, nose_right]
    pub fn set_all_goal_positions(
        &mut self,
        positions: [f64; 7],
    ) -> Result<(), Box<dyn std::error::Error>> {
        let port = port_or_err(&mut self.port)?;
        sm40bl::sync_write_goal_position(
            &self.protocol,
            port,
            &ARM_IDS,
            &[positions[0], positions[1], positions[2], positions[3]],
        )?;
        sts3025bl::sync_write_goal_position(
            &self.protocol,
            port,
            &NOSE_IDS,
            &[positions[4], positions[5], positions[6]],
        )?;
        Ok(())
    }

    pub fn set_left_arm_position(
        &mut self,
        position: [f64; 2],
    ) -> Result<(), Box<dyn std::error::Error>> {
        let port = port_or_err(&mut self.port)?;
        sm40bl::sync_write_goal_position(&self.protocol, port, &LEFT_ARM_IDS, &position)?;
        Ok(())
    }

    pub fn set_right_arm_position(
        &mut self,
        position: [f64; 2],
    ) -> Result<(), Box<dyn std::error::Error>> {
        let port = port_or_err(&mut self.port)?;
        sm40bl::sync_write_goal_position(&self.protocol, port, &RIGHT_ARM_IDS, &position)?;
        Ok(())
    }

    pub fn set_arms_position(
        &mut self,
        position: [f64; 4],
    ) -> Result<(), Box<dyn std::error::Error>> {
        let port = port_or_err(&mut self.port)?;
        sm40bl::sync_write_goal_position(&self.protocol, port, &ARM_IDS, &position)?;
        Ok(())
    }

    pub fn set_nose_position(
        &mut self,
        position: [f64; 3],
    ) -> Result<(), Box<dyn std::error::Error>> {
        let port = port_or_err(&mut self.port)?;
        sts3025bl::sync_write_goal_position(&self.protocol, port, &NOSE_IDS, &position)?;
        Ok(())
    }

    /// Hard safety bounds for each neck joint (rad). Order: [yaw, pitch, roll].
    /// These are the same limits the setters silently clamp against.
    pub fn neck_position_limits(&self) -> [(f64, f64); 3] {
        NECK_POSITION_LIMITS
    }

    /// Set the goal position of all 3 neck joints. Order: [yaw, pitch, roll].
    /// Sends an MIT setpoint with the per-joint kp/kd; returns the latest
    /// motor positions read back from each reply.
    ///
    /// Each component is clamped into `neck_position_limits()` first — a
    /// requested position outside the safe range logs a warning and uses
    /// the clamped value.
    pub fn set_neck_position(
        &mut self,
        position: [f64; 3],
    ) -> Result<[f64; 3], Box<dyn std::error::Error>> {
        let clamped = [
            clamp_neck_position(0, position[0]),
            clamp_neck_position(1, position[1]),
            clamp_neck_position(2, position[2]),
        ];
        self.command_neck_mit(clamped, "set_neck_position")
    }

    pub fn set_neck_yaw_position(
        &mut self,
        position: f64,
    ) -> Result<f64, Box<dyn std::error::Error>> {
        self.set_single_neck_joint(NECK_YAW, position)
    }

    pub fn set_neck_pitch_position(
        &mut self,
        position: f64,
    ) -> Result<f64, Box<dyn std::error::Error>> {
        self.set_single_neck_joint(NECK_PITCH, position)
    }

    pub fn set_neck_roll_position(
        &mut self,
        position: f64,
    ) -> Result<f64, Box<dyn std::error::Error>> {
        self.set_single_neck_joint(NECK_ROLL, position)
    }

    fn set_single_neck_joint(
        &mut self,
        index: usize,
        position: f64,
    ) -> Result<f64, Box<dyn std::error::Error>> {
        let q = clamp_neck_position(index, position);
        self.neck_setpoints[index].q = q as f32;
        let setpoint = ControlSetpoint::Mit(self.neck_setpoints[index]);
        let state = self.neck_motors[index]
            .control(&mut self.can_bus, &setpoint)
            .map_err(|e| format!("set_{}_position: {}", NECK_NAMES[index], e))?;
        Ok(state.position as f64)
    }

    /// Override the MIT impedance gains (kp, kd) for a single neck joint.
    /// Index order matches `set_neck_position`: 0=yaw, 1=pitch, 2=roll.
    pub fn set_neck_gains(
        &mut self,
        index: usize,
        kp: f32,
        kd: f32,
    ) -> Result<(), Box<dyn std::error::Error>> {
        if index >= 3 {
            return Err(format!("neck index out of range: {}", index).into());
        }
        self.neck_setpoints[index].kp = kp;
        self.neck_setpoints[index].kd = kd;
        Ok(())
    }

    /// Home all 3 neck joints to the nearest physical zero.
    ///
    /// After a power cycle the Damiao multi-turn counter resets, so the
    /// reported angle for a joint can land anywhere on the circle even
    /// though the joint hasn't moved. Commanding "go to 0" naively would
    /// then drive the head a full turn. Instead this routine snaps each
    /// goal to the **nearest multiple of 2π** of the joint's current
    /// reported position — physically equivalent to zero, but guaranteed
    /// within ±π of where the joint already is.
    ///
    /// Behaviour:
    /// 1. Reads the current position of each neck joint.
    /// 2. Computes `goal[i] = round(start[i] / 2π) * 2π`.
    /// 3. Applies optional kp/kd overrides to all three joints (`None`
    ///    keeps the current per-joint gain).
    /// 4. Enables neck torque if it wasn't already (does *not* disable on
    ///    exit — caller owns torque lifecycle, same as `set_neck_position`).
    /// 5. Ramps each joint's commanded position toward its goal at
    ///    `speed` rad/s (10 ms control period). Targets are held at the
    ///    goal once they arrive.
    /// 6. Returns `Ok(observed)` once every joint is within `tolerance`
    ///    rad of its goal; returns `Err(...)` on timeout.
    ///
    /// Ramp targets bypass `NECK_POSITION_LIMITS` clamping because the
    /// targets are in the motor-reported (2π-shifted) frame and can fall
    /// outside the joint-space envelope even though the physical pose is
    /// inside it. Once homing settles, the firmware reports the homed
    /// zero and subsequent `set_neck_position` calls clamp normally.
    pub fn neck_home(
        &mut self,
        speed: f64,
        tolerance: f64,
        kp: Option<f32>,
        kd: Option<f32>,
    ) -> Result<[f64; 3], Box<dyn std::error::Error>> {
        const TWO_PI: f64 = 2.0 * std::f64::consts::PI;
        const DT: Duration = Duration::from_millis(10);

        if !(speed > 0.0) {
            return Err(format!("neck_home: speed must be positive, got {}", speed).into());
        }
        if !(tolerance > 0.0) {
            return Err(
                format!("neck_home: tolerance must be positive, got {}", tolerance).into(),
            );
        }

        if kp.is_some() || kd.is_some() {
            for i in 0..3 {
                if let Some(v) = kp {
                    self.neck_setpoints[i].kp = v;
                }
                if let Some(v) = kd {
                    self.neck_setpoints[i].kd = v;
                }
            }
        }

        let start = self.read_neck_positions()?;
        let mut goals = [0.0_f64; 3];
        for i in 0..3 {
            goals[i] = (start[i] / TWO_PI).round() * TWO_PI;
        }

        if !self.neck_torque_enabled {
            self.enable_neck(true)?;
        }

        let mut targets = start;
        let step = speed * DT.as_secs_f64();
        let max_distance = (0..3)
            .map(|i| (start[i] - goals[i]).abs())
            .fold(0.0_f64, f64::max);
        let max_iters = (max_distance / step) as usize + 2_000;

        let mut settled = [false; 3];
        let mut observed = start;

        for _ in 0..max_iters {
            for j in 0..3 {
                if settled[j] {
                    continue;
                }
                let diff = targets[j] - goals[j];
                if diff > step {
                    targets[j] -= step;
                } else if diff < -step {
                    targets[j] += step;
                } else {
                    targets[j] = goals[j];
                }
            }

            observed = self.command_neck_mit(targets, "neck_home")?;

            for j in 0..3 {
                if !settled[j]
                    && (targets[j] - goals[j]).abs() < f64::EPSILON
                    && (observed[j] - goals[j]).abs() < tolerance
                {
                    settled[j] = true;
                }
            }

            if settled.iter().all(|&s| s) {
                return Ok(observed);
            }

            std::thread::sleep(DT);
        }

        Err(format!(
            "neck_home: timed out before settling. observed={:?}, goals={:?}, settled={:?}",
            observed, goals, settled
        )
        .into())
    }

    /// Send an MIT setpoint to all three neck joints using the current
    /// per-joint kp/kd, and return the position read back from each reply.
    /// The `position` array is sent verbatim — callers that need the
    /// joint-space safety clamp must apply `clamp_neck_position` first.
    /// `context` is used to tag any per-joint error message.
    fn command_neck_mit(
        &mut self,
        position: [f64; 3],
        context: &str,
    ) -> Result<[f64; 3], Box<dyn std::error::Error>> {
        let mut observed = [0.0_f64; 3];
        for i in 0..3 {
            self.neck_setpoints[i].q = position[i] as f32;
            let setpoint = ControlSetpoint::Mit(self.neck_setpoints[i]);
            let state = self.neck_motors[i]
                .control(&mut self.can_bus, &setpoint)
                .map_err(|e| format!("{}({}): {}", context, NECK_NAMES[i], e))?;
            observed[i] = state.position as f64;
        }
        Ok(observed)
    }

    pub fn is_torque_enabled(&mut self) -> Result<bool, Box<dyn std::error::Error>> {
        let port = port_or_err(&mut self.port)?;
        let arm_torque = sm40bl::sync_read_torque_enable(&self.protocol, port, &ARM_IDS)?;
        let nose_torque = sts3025bl::sync_read_torque_enable(&self.protocol, port, &NOSE_IDS)?;

        Ok(arm_torque.iter().chain(nose_torque.iter()).all(|&x| x))
    }

    /// In-memory torque state for the neck joints. The Damiao firmware does
    /// not expose a readable torque-enable register — enable/disable are
    /// FF-prefix commands — so we track the last commanded state ourselves.
    pub fn is_neck_torque_enabled(&self) -> bool {
        self.neck_torque_enabled
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
        let port = port_or_err(&mut self.port)?;
        sm40bl::sync_write_torque_enable(&self.protocol, port, &ARM_IDS, &[enable; 4])?;
        Ok(())
    }

    pub fn enable_nose(&mut self, enable: bool) -> Result<(), Box<dyn std::error::Error>> {
        let port = port_or_err(&mut self.port)?;
        sts3025bl::sync_write_torque_enable(&self.protocol, port, &NOSE_IDS, &[enable; 3])?;
        Ok(())
    }

    /// Enable or disable all three neck motors. On enable, first puts each
    /// motor into MIT mode (writes register 10) before issuing the enable
    /// command, so the subsequent `set_neck_position` calls aren't silently
    /// dropped because the motor was left in POS_VEL/VEL/FORCE_POS.
    pub fn enable_neck(&mut self, enable: bool) -> Result<(), Box<dyn std::error::Error>> {
        for (i, dm) in self.neck_motors.iter().enumerate() {
            if enable {
                dm.ensure_control_mode(&mut self.can_bus, ControlMode::Mit)
                    .map_err(|e| format!("{}: ensure MIT mode failed: {}", NECK_NAMES[i], e))?;
                dm.enable(&mut self.can_bus)
                    .map_err(|e| format!("{}: enable failed: {}", NECK_NAMES[i], e))?;
            } else {
                dm.disable(&mut self.can_bus)
                    .map_err(|e| format!("{}: disable failed: {}", NECK_NAMES[i], e))?;
            }
        }
        self.neck_torque_enabled = enable;
        Ok(())
    }

    fn set_torque(&mut self, enable: bool) -> Result<(), Box<dyn std::error::Error>> {
        let port = port_or_err(&mut self.port)?;
        sm40bl::sync_write_torque_enable(&self.protocol, port, &ARM_IDS, &[enable; 4])?;
        sts3025bl::sync_write_torque_enable(&self.protocol, port, &NOSE_IDS, &[enable; 3])?;
        Ok(())
    }

    fn set_torque_on_ids(
        &mut self,
        ids: &[u8],
        enable: bool,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let (arm_targets, nose_targets) = split_ids_by_family(ids);
        if arm_targets.is_empty() && nose_targets.is_empty() {
            return Ok(());
        }

        let port = port_or_err(&mut self.port)?;
        if !arm_targets.is_empty() {
            let enables = vec![enable; arm_targets.len()];
            sm40bl::sync_write_torque_enable(&self.protocol, port, &arm_targets, &enables)?;
        }
        if !nose_targets.is_empty() {
            let enables = vec![enable; nose_targets.len()];
            sts3025bl::sync_write_torque_enable(&self.protocol, port, &nose_targets, &enables)?;
        }
        Ok(())
    }

    pub fn read_raw_bytes(
        &mut self,
        id: u8,
        address: u8,
        length: u8,
    ) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
        let port = port_or_err(&mut self.port)?;
        self.protocol.read(port, id, address, length)
    }

    pub fn write_raw_bytes(
        &mut self,
        id: u8,
        address: u8,
        data: &[u8],
    ) -> Result<(), Box<dyn std::error::Error>> {
        let port = port_or_err(&mut self.port)?;
        self.protocol.write(port, id, address, data)
    }

    pub fn write_raw_packet(&mut self, data: &[u8]) -> Result<Vec<u8>, std::io::Error> {
        let port = self.port.as_deref_mut().ok_or_else(|| {
            std::io::Error::new(
                std::io::ErrorKind::Unsupported,
                "serial port not configured (neck-only controller)",
            )
        })?;
        port.write_all(data)?;
        port.flush()?;

        let mut n = port.bytes_to_read()? as usize;
        let start = std::time::Instant::now();
        while n == 0 && start.elapsed() < Duration::from_millis(10) {
            std::thread::sleep(Duration::from_millis(5));
            n = port.bytes_to_read()? as usize;
        }
        let mut buff = vec![0u8; n];
        port.read_exact(&mut buff)?;

        Ok(buff)
    }
}

/// Clamp a requested neck position to the safe per-joint range and log a
/// warning whenever the input was out of bounds. Centralising the clamp here
/// means every command path (`set_neck_position`, `set_neck_*_position`,
/// future control-loop commands routed through `set_single_neck_joint`)
/// enforces the same envelope — there is no way to bypass it from outside.
fn clamp_neck_position(index: usize, requested: f64) -> f64 {
    let (lo, hi) = NECK_POSITION_LIMITS[index];
    let clamped = requested.clamp(lo, hi);
    if (clamped - requested).abs() > f64::EPSILON {
        warn!(
            "{} requested position {:.4} rad out of safe range [{:.4}, {:.4}]; clamped to {:.4}",
            NECK_NAMES[index], requested, lo, hi, clamped
        );
    }
    clamped
}

/// Free function (vs. a `&mut self` helper method) so the caller can hold the
/// returned `&mut SerialPort` and still freely access `self.protocol`,
/// `self.neck_motors`, etc. via disjoint-field NLL borrows.
fn port_or_err<'a>(
    port: &'a mut Option<Box<dyn serialport::SerialPort>>,
) -> Result<&'a mut (dyn serialport::SerialPort + 'static), Box<dyn std::error::Error>> {
    match port.as_deref_mut() {
        Some(p) => Ok(p),
        None => Err("serial port not configured (neck-only controller)".into()),
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