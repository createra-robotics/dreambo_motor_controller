use log::info;
use pyo3::prelude::*;
use pyo3_stub_gen::derive::{gen_stub_pyclass, gen_stub_pymethods};

use std::{
    collections::HashMap,
    fmt::Debug,
    sync::{Arc, Mutex},
    time::{Duration, SystemTime, UNIX_EPOCH},
};

use tokio::{
    sync::mpsc::{self, Sender},
    time,
};

use crate::DreamboMotorController;

/// Minimum input voltage (in 0.1V units) required before the control loop
/// considers the rail stable. SM40BL and STS3025BL both run at 12V nominal.
const MIN_VOLTAGE_TENTHS: u8 = 110;

#[gen_stub_pyclass]
#[pyclass]
#[derive(Debug, Clone, Copy)]
pub struct DreamboTorsoPosition {
    #[pyo3(get)]
    pub left_arm: [f64; 2],
    #[pyo3(get)]
    pub right_arm: [f64; 2],
    #[pyo3(get)]
    pub nose: [f64; 3],
    #[pyo3(get)]
    pub timestamp: f64, // seconds since UNIX epoch
}

/// Execute an operation with automatic retry on transient failures
///
/// Handles brief USB interruptions with fast retries
/// Fallback to control loop's slower retry mechanism for persistent issues
fn with_retry<T, F>(mut op: F, attempts: u64) -> Result<T, Box<dyn std::error::Error>>
where
    F: FnMut() -> Result<T, Box<dyn std::error::Error>>,
{
    const RETRY_DELAY_MS: u64 = 20;

    for attempt in 0..attempts {
        match op() {
            Ok(val) => {
                if attempt > 0 {
                    info!("Serial I/O recovered after {} retries", attempt);
                }
                return Ok(val);
            }
            Err(e) if attempt < attempts - 1 => {
                // Only retry on transient errors
                let is_transient = e
                    .downcast_ref::<std::io::Error>()
                    .map(|io_err| {
                        matches!(
                            io_err.kind(),
                            std::io::ErrorKind::TimedOut | std::io::ErrorKind::Interrupted
                        )
                    })
                    .unwrap_or(false);

                if is_transient {
                    std::thread::sleep(Duration::from_millis(RETRY_DELAY_MS));
                } else {
                    // Non-transient error, fail immediately
                    return Err(e);
                }
            }
            Err(e) => return Err(e),
        }
    }
    unreachable!()
}

#[gen_stub_pymethods]
#[pymethods]
impl DreamboTorsoPosition {
    #[new]
    pub fn new(left_arm: Vec<f64>, right_arm: Vec<f64>, nose: Vec<f64>) -> Self {
        if left_arm.len() != 2 || right_arm.len() != 2 || nose.len() != 3 {
            panic!("Each arm must have 2 positions and the nose must have 3 positions.");
        }
        DreamboTorsoPosition {
            left_arm: [left_arm[0], left_arm[1]],
            right_arm: [right_arm[0], right_arm[1]],
            nose: [nose[0], nose[1], nose[2]],
            timestamp: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or(Duration::from_secs(0))
                .as_secs_f64(),
        }
    }

    fn __repr__(&self) -> pyo3::PyResult<String> {
        Ok(format!(
            "DreamboTorsoPosition(left_arm={:?}, right_arm={:?}, nose={:?}, timestamp={:.3})",
            self.left_arm, self.right_arm, self.nose, self.timestamp
        ))
    }
}

pub struct DreamboControlLoop {
    loop_handle: Arc<Mutex<Option<std::thread::JoinHandle<()>>>>,
    stop_signal: Arc<Mutex<bool>>,
    tx: Sender<MotorCommand>,
    last_position: Arc<Mutex<Result<DreamboTorsoPosition, MotorError>>>,
    last_torque: Arc<Mutex<Result<bool, MotorError>>>,
    last_stats: Option<(Duration, Arc<Mutex<ControlLoopStats>>)>,
    motor_name_id: HashMap<String, u8>,
}

#[derive(Debug, Clone)]
pub enum MotorCommand {
    SetAllGoalPositions {
        positions: DreamboTorsoPosition,
    },
    SetLeftArm {
        position: [f64; 2],
    },
    SetRightArm {
        position: [f64; 2],
    },
    SetArms {
        position: [f64; 4],
    },
    SetNose {
        position: [f64; 3],
    },
    EnableTorque(),
    EnableTorqueOnIds {
        ids: Vec<u8>,
    },
    DisableTorque(),
    DisableTorqueOnIds {
        ids: Vec<u8>,
    },
    EnableArms {
        enable: bool,
    },
    EnableNose {
        enable: bool,
    },
    ReadRawBytes {
        id: u8,
        addr: u8,
        length: u8,
        tx: std::sync::mpsc::Sender<Vec<u8>>,
    },
    WriteRawBytes {
        id: u8,
        addr: u8,
        data: Vec<u8>,
    },

    WriteRawPacket {
        packet: Vec<u8>,
        tx: std::sync::mpsc::Sender<Vec<u8>>,
    },
}

#[gen_stub_pyclass]
#[pyclass]
#[derive(Clone)]
pub struct ControlLoopStats {
    #[pyo3(get)]
    pub period: Vec<f64>,
    #[pyo3(get)]
    pub read_dt: Vec<f64>,
    #[pyo3(get)]
    pub write_dt: Vec<f64>,
}

#[pymethods]
impl ControlLoopStats {
    fn __repr__(&self) -> pyo3::PyResult<String> {
        Ok(format!(
            "ControlLoopStats(period=~{:.2?}ms, read_dt=~{:.2?} ms, write_dt=~{:.2?} ms)",
            self.period.iter().sum::<f64>() / self.period.len() as f64 * 1000.0,
            self.read_dt.iter().sum::<f64>() / self.read_dt.len() as f64 * 1000.0,
            self.write_dt.iter().sum::<f64>() / self.write_dt.len() as f64 * 1000.0,
        ))
    }
}

impl std::fmt::Debug for ControlLoopStats {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.__repr__().unwrap())
    }
}

#[derive(Debug, Clone)]
pub enum MotorError {
    MissingMotors(Vec<String>),
    CommunicationError(),
    NoPowerError(),
    VoltageRampUpTimeoutError(u8, Duration),
    PortNotFound(String),
    CouldNotOpenPort(String),
}

impl std::error::Error for MotorError {}
impl std::fmt::Display for MotorError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MotorError::MissingMotors(names) => {
                write!(f, "Missing motors: {:?}!", names)
            }
            MotorError::CommunicationError() => {
                write!(
                    f,
                    "Motor communication error! Check connections and power supply."
                )
            }
            MotorError::NoPowerError() => {
                write!(
                    f,
                    "No motors detected. Check if the power supply is connected and turned on!"
                )
            }
            MotorError::PortNotFound(port) => {
                write!(
                    f,
                    "Check if your USB cable is connected. Could not find port: {}!",
                    port
                )
            }
            MotorError::CouldNotOpenPort(port) => {
                write!(
                    f,
                    "Could not open serial port: {}! Check permissions and if the port is already in use.",
                    port
                )
            }
            MotorError::VoltageRampUpTimeoutError(voltage, duration) => {
                write!(
                    f,
                    "Voltage did not ramp up to 12V (got {:.1}V) within {:?}!",
                    *voltage as f64 / 10.0,
                    duration
                )
            }
        }
    }
}

impl DreamboControlLoop {
    pub fn new(
        serialport: String,
        read_position_loop_period: Duration,
        stats_pub_period: Option<Duration>,
        read_allowed_retries: u64,
        voltage_rampup_timeout: Duration,
    ) -> Result<Self, MotorError> {
        let stop_signal = Arc::new(Mutex::new(false));
        let stop_signal_clone = stop_signal.clone();

        let (tx, rx) = mpsc::channel(100);

        let last_stats = stats_pub_period.map(|period| {
            (
                period,
                Arc::new(Mutex::new(ControlLoopStats {
                    period: Vec::new(),
                    read_dt: Vec::new(),
                    write_dt: Vec::new(),
                })),
            )
        });
        let last_stats_clone = last_stats.clone();

        // Validate serial port based on operating system

        // On Unix-like systems, check if the port path exists
        #[cfg(not(windows))]
        if !std::path::Path::new(&serialport).exists() {
            return Err(MotorError::PortNotFound(serialport));
        }
        // On Windows, validate COM port format
        #[cfg(windows)]
        if !serialport.starts_with("COM") {
            return Err(MotorError::PortNotFound(serialport));
        }

        let mut c = DreamboMotorController::new(serialport.as_str())
            .map_err(|_| MotorError::CouldNotOpenPort(serialport.clone()))?;

        match c.check_missing_ids() {
            Ok(missing_ids) if missing_ids.len() == 7 => {
                return Err(MotorError::NoPowerError());
            }
            Ok(missing_ids) if !missing_ids.is_empty() => {
                let id_to_name: HashMap<u8, String> = c
                    .get_motor_name_id()
                    .iter()
                    .map(|(name, id)| (id.clone(), name.clone()))
                    .collect();

                let missing_motors: Vec<String> = missing_ids
                    .iter()
                    .map(|id| {
                        id_to_name
                            .get(id)
                            .unwrap_or(&format!("Unknown({})", id))
                            .clone()
                    })
                    .collect();
                return Err(MotorError::MissingMotors(missing_motors));
            }
            Ok(_) => {}
            Err(_) => return Err(MotorError::CommunicationError()),
        }

        // Wait until voltage is stable at the 12V rail
        info!("Waiting for voltage to be stable at 12V...");
        let mut current_voltage = with_retry(|| c.read_all_voltages(), read_allowed_retries)
            .map_err(|_| MotorError::CommunicationError())?;
        let start_time = SystemTime::now();
        while current_voltage
            .iter()
            .any(|&v| v < MIN_VOLTAGE_TENTHS && start_time.elapsed().unwrap() < voltage_rampup_timeout)
        {
            std::thread::sleep(Duration::from_millis(100));
            current_voltage = with_retry(|| c.read_all_voltages(), read_allowed_retries)
                .map_err(|_| MotorError::CommunicationError())?;
        }
        if current_voltage.iter().any(|&v| v < MIN_VOLTAGE_TENTHS) {
            return Err(MotorError::VoltageRampUpTimeoutError(
                current_voltage.iter().cloned().min().unwrap_or(0),
                voltage_rampup_timeout,
            ));
        }
        info!(
            "Voltage is stable at ~12V: {:?} (took {:?})",
            current_voltage,
            start_time.elapsed().unwrap()
        );

        let motor_name_id = c.get_motor_name_id();

        // Reboot all motors
        c.reboot(Duration::from_secs(1))
            .map_err(|_| MotorError::CommunicationError())?;

        // Init last position by trying to read current positions
        // If the init fails, it probably means we have an hardware issue
        // so it's better to fail.
        let last_position = read_pos(&mut c, read_allowed_retries)?;
        let last_torque = with_retry(|| c.is_torque_enabled(), read_allowed_retries)
            .map_err(|_| MotorError::CommunicationError())?;

        let last_position = Arc::new(Mutex::new(Ok(last_position)));
        let last_position_clone = last_position.clone();

        let last_torque = Arc::new(Mutex::new(Ok(last_torque)));
        let last_torque_clone = last_torque.clone();

        let loop_handle = std::thread::spawn(move || {
            run(
                c,
                stop_signal_clone,
                rx,
                last_position_clone,
                last_torque_clone,
                last_stats_clone,
                read_position_loop_period,
                read_allowed_retries,
            );
        });

        Ok(DreamboControlLoop {
            loop_handle: Arc::new(Mutex::new(Some(loop_handle))),
            stop_signal,
            tx,
            last_position,
            last_torque,
            last_stats,
            motor_name_id,
        })
    }

    pub fn close(&self) {
        if let Ok(mut stop) = self.stop_signal.lock() {
            *stop = true;
        }
        match self.loop_handle.lock() {
            Ok(mut opt_handle) => {
                if let Some(handle) = opt_handle.take() {
                    if let Err(e) = handle.join() {
                        log::error!("Failed to join control loop thread: {:?}", e);
                    }
                }
            }
            Err(poisoned) => {
                // If the mutex is poisoned, try to recover the handle
                let mut opt_handle = poisoned.into_inner();
                if let Some(handle) = opt_handle.take() {
                    if let Err(e) = handle.join() {
                        log::error!("Failed to join control loop thread (poisoned): {:?}", e);
                    }
                }
            }
        }
    }

    pub fn get_motor_name_id(&self) -> HashMap<String, u8> {
        self.motor_name_id.clone()
    }

    pub fn push_command(
        &self,
        command: MotorCommand,
    ) -> Result<(), mpsc::error::SendError<MotorCommand>> {
        self.tx.blocking_send(command)
    }

    pub fn get_last_position(&self) -> Result<DreamboTorsoPosition, MotorError> {
        let guard = match self.last_position.lock() {
            Ok(guard) => guard,
            Err(poisoned) => {
                log::error!("last_position mutex was poisoned");
                poisoned.into_inner()
            }
        };
        match &*guard {
            Ok(pos) => Ok(*pos),
            Err(e) => Err(e.clone()),
        }
    }

    pub fn is_torque_enabled(&self) -> Result<bool, MotorError> {
        let guard = match self.last_torque.lock() {
            Ok(guard) => guard,
            Err(poisoned) => {
                log::error!("last_torque mutex was poisoned");
                poisoned.into_inner()
            }
        };
        match &*guard {
            Ok(enabled) => Ok(*enabled),
            Err(e) => Err(e.clone()),
        }
    }

    pub fn get_stats(&self) -> Result<Option<ControlLoopStats>, MotorError> {
        match self.last_stats {
            Some((_, ref stats)) => {
                let guard = match stats.lock() {
                    Ok(guard) => guard,
                    Err(poisoned) => {
                        log::error!("last_stats mutex was poisoned");
                        poisoned.into_inner()
                    }
                };
                Ok(Some(guard.clone()))
            }
            None => Ok(None),
        }
    }

    pub fn async_read_raw_bytes(
        &self,
        id: u8,
        addr: u8,
        length: u8,
    ) -> Result<Vec<u8>, MotorError> {
        let (tx, rx) = std::sync::mpsc::channel();
        let command = MotorCommand::ReadRawBytes { id, addr, length, tx };
        self.push_command(command)
            .map_err(|_| MotorError::CommunicationError())?;
        rx.recv_timeout(Duration::from_secs(1))
            .map_err(|_| MotorError::CommunicationError())
    }

    pub fn async_write_raw_bytes(&self, id: u8, addr: u8, data: Vec<u8>) -> Result<(), MotorError> {
        let command = MotorCommand::WriteRawBytes { id, addr, data };
        self.push_command(command)
            .map_err(|_| MotorError::CommunicationError())?;
        Ok(())
    }

    /// PID coefficient registers on SM40BL / STS3025BL: P=21, D=22, I=23 (each u8).
    pub fn async_read_pid_gains(&self, id: u8) -> Result<(u16, u16, u16), MotorError> {
        const P_GAIN_ADDR: u8 = 21;

        self.async_read_raw_bytes(id, P_GAIN_ADDR, 3)
            .and_then(|data| {
                if data.len() != 3 {
                    return Err(MotorError::CommunicationError());
                }
                let p_gain = data[0] as u16;
                let d_gain = data[1] as u16;
                let i_gain = data[2] as u16;
                Ok((p_gain, i_gain, d_gain))
            })
    }

    pub fn async_write_pid_gains(
        &self,
        id: u8,
        p_gain: u16,
        i_gain: u16,
        d_gain: u16,
    ) -> Result<(), MotorError> {
        const P_GAIN_ADDR: u8 = 21;

        let data = vec![p_gain as u8, d_gain as u8, i_gain as u8];

        self.async_write_raw_bytes(id, P_GAIN_ADDR, data)
    }
}

impl Drop for DreamboControlLoop {
    fn drop(&mut self) {
        self.close();
    }
}

fn run(
    mut c: DreamboMotorController,
    stop_signal: Arc<Mutex<bool>>,
    mut rx: mpsc::Receiver<MotorCommand>,
    last_position: Arc<Mutex<Result<DreamboTorsoPosition, MotorError>>>,
    last_torque: Arc<Mutex<Result<bool, MotorError>>>,
    last_stats: Option<(Duration, Arc<Mutex<ControlLoopStats>>)>,
    read_position_loop_period: Duration,
    read_allowed_retries: u64,
) {
    tokio::runtime::Runtime::new().unwrap().block_on(async {
        let mut interval = time::interval(read_position_loop_period);

        // Stats related variables
        let mut stats_t0 = std::time::Instant::now();
        let mut read_dt = Vec::new();
        let mut write_dt = Vec::new();

        let mut last_read_tick = std::time::Instant::now();

        loop {
            tokio::select! {
                maybe_command = rx.recv() => {
                    if let Some(command) = maybe_command {
                        let write_tick = std::time::Instant::now();
                        if handle_commands(&mut c, last_torque.clone(), command, read_allowed_retries).is_ok() {
                            if last_stats.is_some() {
                                let elapsed = write_tick.elapsed().as_secs_f64();
                                write_dt.push(elapsed);
                            }
                        }
                    }
                }
                _ = interval.tick() => {
                    let read_tick = std::time::Instant::now();
                    if let Some((_, stats)) = &last_stats {
                        stats.lock().unwrap().period.push(read_tick.duration_since(last_read_tick).as_secs_f64());
                        last_read_tick = read_tick;
                    }

                    match read_pos(&mut c, read_allowed_retries) {
                        Ok(positions) => {
                            if let Ok(mut pos) = last_position.lock() {
                                *pos = Ok(positions);
                            }
                        },
                        Err(e) => {
                            if let Ok(mut pos) = last_position.lock() {
                                *pos = Err(e);
                            }
                        },
                    }
                    if last_stats.is_some() {
                        let elapsed = read_tick.elapsed().as_secs_f64();
                        read_dt.push(elapsed);
                    }

                    if let Some((period, stats)) = &last_stats
                        && stats_t0.elapsed() > *period {
                            stats.lock().unwrap().read_dt.extend(read_dt.iter().cloned());
                            stats.lock().unwrap().write_dt.extend(write_dt.iter().cloned());

                            read_dt.clear();
                            write_dt.clear();
                            stats_t0 = std::time::Instant::now();
                    }
                }
            }

            if *stop_signal.lock().unwrap() {
                // Drain the command channel before exiting
                loop {
                    if rx.is_empty() {
                        break;
                    }
                    if let Some(command) = rx.recv().await {
                        let _ = handle_commands(&mut c, last_torque.clone(), command, read_allowed_retries);
                    }
                }
                break;
            }
        }
    })
}

fn handle_commands(
    controller: &mut DreamboMotorController,
    last_torque: Arc<Mutex<Result<bool, MotorError>>>,
    command: MotorCommand,
    read_allowed_retries: u64,
) -> Result<(), Box<dyn std::error::Error>> {
    use MotorCommand::*;

    match command {
        SetAllGoalPositions { positions } => controller
            .set_all_goal_positions([
                positions.left_arm[0],
                positions.left_arm[1],
                positions.right_arm[0],
                positions.right_arm[1],
                positions.nose[0],
                positions.nose[1],
                positions.nose[2],
            ])
            .map(|_| ()),
        SetLeftArm { position } => controller.set_left_arm_position(position).map(|_| ()),
        SetRightArm { position } => controller.set_right_arm_position(position).map(|_| ()),
        SetArms { position } => controller.set_arms_position(position).map(|_| ()),
        SetNose { position } => controller.set_nose_position(position).map(|_| ()),
        EnableTorque() => {
            let res = controller.enable_torque();
            if res.is_ok()
                && let Ok(mut torque) = last_torque.lock()
            {
                *torque = Ok(true);
            }
            res.map(|_| ())
        }
        EnableTorqueOnIds { ids } => {
            let res = controller.enable_torque_on_ids(&ids);
            if res.is_ok()
                && let Ok(mut torque) = last_torque.lock()
            {
                *torque = Ok(true);
            }
            res.map(|_| ())
        }
        DisableTorque() => {
            let res = controller.disable_torque();
            if res.is_ok()
                && let Ok(mut torque) = last_torque.lock()
            {
                *torque = Ok(false);
            }
            res.map(|_| ())
        }
        DisableTorqueOnIds { ids } => {
            let res = controller.disable_torque_on_ids(&ids);
            if res.is_ok()
                && let Ok(mut torque) = last_torque.lock()
            {
                *torque = Ok(false);
            }
            res.map(|_| ())
        }
        EnableArms { enable } => controller.enable_arms(enable).map(|_| ()),
        EnableNose { enable } => controller.enable_nose(enable).map(|_| ()),
        ReadRawBytes { id, addr, length, tx } => {
            let data = with_retry(
                || controller.read_raw_bytes(id, addr, length),
                read_allowed_retries,
            )?;
            let _ = tx.send(data);
            Ok(())
        }
        WriteRawBytes { id, addr, data } => {
            controller.write_raw_bytes(id, addr, &data).map(|_| ())
        }
        WriteRawPacket { packet, tx } => {
            let response = controller.write_raw_packet(&packet)?;
            tx.send(response)?;
            Ok(())
        }
    }
}

pub fn read_pos(
    c: &mut DreamboMotorController,
    read_allowed_retries: u64,
) -> Result<DreamboTorsoPosition, MotorError> {
    with_retry(|| c.read_all_positions(), read_allowed_retries)
        .map(|positions| {
            let now = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_else(|_| std::time::Duration::from_secs(0));
            DreamboTorsoPosition {
                left_arm: [positions[0], positions[1]],
                right_arm: [positions[2], positions[3]],
                nose: [positions[4], positions[5], positions[6]],
                timestamp: now.as_secs_f64(),
            }
        })
        .map_err(|_| MotorError::CommunicationError())
}
