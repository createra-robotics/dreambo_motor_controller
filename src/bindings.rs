use std::{collections::HashMap, sync::mpsc::channel, time::Duration};

use crate::control_loop::{
    ControlLoopStats, DreamboControlLoop, DreamboTorsoPosition, MotorCommand,
};

use pyo3::{prelude::*, types::PyBytes};
use pyo3_stub_gen::{
    define_stub_info_gatherer,
    derive::{gen_stub_pyclass, gen_stub_pymethods},
};

use crate::DreamboMotorController as Controller;

#[gen_stub_pyclass]
#[pyclass(frozen)]
struct DreamboMotorController {
    inner: std::sync::Mutex<Controller>,
}

#[gen_stub_pymethods]
#[pymethods]
impl DreamboMotorController {
    /// Create a new motor controller for the given serial port and CAN bus.
    ///
    /// # Arguments
    /// * `serialport` - Path to (Unix) or COM ID (Windows) of the serial port.
    ///   Pass `None` to build a neck-only (CAN) controller; arm/nose methods
    ///   will then raise an exception.
    /// * `can_bus` - SocketCAN interface name for the neck Damiao motors (default `"can0"`).
    #[new]
    #[pyo3(signature = (serialport = None, can_bus = String::from("can0")))]
    fn new(serialport: Option<String>, can_bus: String) -> PyResult<Self> {
        let inner = Controller::new(serialport.as_deref(), &can_bus)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(DreamboMotorController {
            inner: std::sync::Mutex::new(inner),
        })
    }

    /// Is torque enabled on all motors
    fn is_torque_enabled(&self) -> PyResult<bool> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .is_torque_enabled()
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Enable torque on all motors.
    fn enable_torque(&self) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .enable_torque()
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    /// Enable torque on ids
    fn enable_torque_on_ids(&self, ids: Vec<u8>) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .enable_torque_on_ids(&ids)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    /// Disable torque on all motors.
    fn disable_torque(&self) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .disable_torque()
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    /// Disable torque on ids
    fn disable_torque_on_ids(&self, ids: Vec<u8>) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;
        inner
            .disable_torque_on_ids(&ids)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    /// Read all motor positions as a 7-element array.
    /// Order: [left_arm_pitch, left_arm_yaw, right_arm_pitch, right_arm_yaw, nose_top, nose_left, nose_right]
    fn read_all_positions(&self) -> PyResult<[f64; 7]> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .read_all_positions()
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Set goal positions for all motors (7 values).
    ///
    /// # Arguments
    /// * `positions` - Array of 7 goal positions
    ///   (left_arm_pitch, left_arm_yaw, right_arm_pitch, right_arm_yaw, nose_top, nose_left, nose_right).
    fn set_all_goal_positions(&self, positions: [f64; 7]) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .set_all_goal_positions(positions)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    /// Set goal positions for the left arm (2 values: pitch, yaw).
    fn set_left_arm_position(&self, position: [f64; 2]) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .set_left_arm_position(position)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    /// Set goal positions for the right arm (2 values: pitch, yaw).
    fn set_right_arm_position(&self, position: [f64; 2]) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .set_right_arm_position(position)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    /// Set goal positions for both arms (4 values: left pitch, left yaw, right pitch, right yaw).
    fn set_arms_position(&self, position: [f64; 4]) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .set_arms_position(position)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    /// Set goal positions for the nose (3 values).
    fn set_nose_position(&self, position: [f64; 3]) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .set_nose_position(position)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    /// Set goal positions for the neck (3 values: yaw, pitch, roll). Returns
    /// the observed positions reported by the motor replies.
    fn set_neck_position(&self, position: [f64; 3]) -> PyResult<[f64; 3]> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;
        inner
            .set_neck_position(position)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    fn set_neck_yaw_position(&self, position: f64) -> PyResult<f64> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;
        inner
            .set_neck_yaw_position(position)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    fn set_neck_pitch_position(&self, position: f64) -> PyResult<f64> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;
        inner
            .set_neck_pitch_position(position)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    fn set_neck_roll_position(&self, position: f64) -> PyResult<f64> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;
        inner
            .set_neck_roll_position(position)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Hard safety bounds (lower, upper) in rad for each neck joint.
    /// Order: [yaw, pitch, roll]. Every neck setter clamps to these bounds
    /// before commanding the motor.
    fn neck_position_limits(&self) -> PyResult<[(f64, f64); 3]> {
        let inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;
        Ok(inner.neck_position_limits())
    }

    /// Read the current neck positions (yaw, pitch, roll) via a Damiao
    /// feedback request.
    fn read_neck_positions(&self) -> PyResult<[f64; 3]> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;
        inner
            .read_neck_positions()
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Override the MIT impedance gains (kp, kd) for a single neck joint.
    /// `index`: 0=yaw, 1=pitch, 2=roll.
    fn set_neck_gains(&self, index: usize, kp: f32, kd: f32) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;
        inner
            .set_neck_gains(index, kp, kd)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Enable or disable the neck motors. On enable, also forces each motor
    /// into MIT control mode.
    fn enable_neck(&self, enable: bool) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;
        inner
            .enable_neck(enable)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Last commanded torque-enable state for the neck (in-memory; the
    /// Damiao firmware does not expose a readable enable register).
    fn is_neck_torque_enabled(&self) -> PyResult<bool> {
        let inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;
        Ok(inner.is_neck_torque_enabled())
    }

    /// Enable or disable the arm motors.
    fn enable_arms(&self, enable: bool) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .enable_arms(enable)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    /// Enable or disable the nose motors.
    fn enable_nose(&self, enable: bool) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .enable_nose(enable)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    /// Write raw packet data to the serial port.
    ///
    /// # Arguments
    /// * `data` - Byte array of raw packet data to send.
    fn write_raw_packet(&self, data: Py<PyBytes>, py: Python) -> PyResult<()> {
        let bytes = data.as_bytes(py);
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;
        inner
            .write_raw_packet(bytes)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }
}

#[gen_stub_pyclass]
#[pyclass]
struct DreamboPyControlLoop {
    inner: std::sync::Arc<DreamboControlLoop>,
}

#[gen_stub_pymethods]
#[pymethods]
impl DreamboPyControlLoop {
    /// Create a new control loop for the motor controller.
    ///
    /// # Arguments
    /// * `serialport` - Path to (Unix) or COM ID (Windows) of the serial port.
    ///   Pass `None` to build a neck-only (CAN) loop — arm/nose commands then
    ///   raise on use.
    /// * `read_position_loop_period` - Period between control loop updates.
    /// * `can_bus` - SocketCAN interface name for the neck Damiao motors (default `"can0"`).
    /// * `allowed_retries` - Number of allowed retries for reading positions.
    /// * `stats_pub_period` - Optional period for publishing stats.
    /// * `voltage_rampup_timeout` - Maximum time to wait for the 12V rail to stabilize.
    #[new]
    #[pyo3(signature = (
        serialport,
        read_position_loop_period,
        can_bus = String::from("can0"),
        allowed_retries = 5,
        stats_pub_period = None,
        voltage_rampup_timeout = Duration::from_secs(30),
    ))]
    fn new(
        serialport: Option<String>,
        read_position_loop_period: Duration,
        can_bus: String,
        allowed_retries: u64,
        stats_pub_period: Option<Duration>,
        voltage_rampup_timeout: Duration,
    ) -> PyResult<Self> {
        let control_loop = DreamboControlLoop::new(
            serialport,
            can_bus,
            read_position_loop_period,
            stats_pub_period,
            allowed_retries,
            voltage_rampup_timeout,
        )
        .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(DreamboPyControlLoop {
            inner: std::sync::Arc::new(control_loop),
        })
    }

    /// Close the control loop and release resources.
    fn close(&self) -> PyResult<()> {
        self.inner.close();
        Ok(())
    }

    /// Get the id/name motors used in this controller.
    fn get_motor_name_id(&self) -> HashMap<String, u8> {
        self.inner.get_motor_name_id()
    }

    /// Get the last successfully read motor positions.
    fn get_last_position(&self) -> PyResult<DreamboTorsoPosition> {
        self.inner
            .get_last_position()
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Set goal positions for all motors.
    fn set_all_goal_positions(&self, positions: DreamboTorsoPosition) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::SetAllGoalPositions { positions })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Set goal positions for the left arm (2 values: pitch, yaw).
    fn set_left_arm_position(&self, position: [f64; 2]) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::SetLeftArm { position })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Set goal positions for the right arm (2 values: pitch, yaw).
    fn set_right_arm_position(&self, position: [f64; 2]) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::SetRightArm { position })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Set goal positions for both arms (4 values).
    fn set_arms_position(&self, position: [f64; 4]) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::SetArms { position })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Set goal positions for the nose (3 values).
    fn set_nose_position(&self, position: [f64; 3]) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::SetNose { position })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Set goal positions for the neck (3 values: yaw, pitch, roll).
    fn set_neck_position(&self, position: [f64; 3]) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::SetNeck { position })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    fn set_neck_yaw_position(&self, position: f64) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::SetNeckYaw { position })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    fn set_neck_pitch_position(&self, position: f64) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::SetNeckPitch { position })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    fn set_neck_roll_position(&self, position: f64) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::SetNeckRoll { position })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Enable or disable the neck motors.
    fn enable_neck(&self, enable: bool) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::EnableNeck { enable })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Check torque enabled status.
    fn is_torque_enabled(&self) -> PyResult<bool> {
        self.inner
            .is_torque_enabled()
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Enable torque on all motors.
    fn enable_torque(&self) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::EnableTorque())
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Enable torque on ids.
    fn enable_torque_on_ids(&self, ids: Vec<u8>) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::EnableTorqueOnIds { ids })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Disable torque on all motors.
    fn disable_torque(&self) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::DisableTorque())
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Disable torque on ids.
    fn disable_torque_on_ids(&self, ids: Vec<u8>) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::DisableTorqueOnIds { ids })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Enable or disable the arm motors.
    fn enable_arms(&self, enable: bool) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::EnableArms { enable })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Enable or disable the nose motors.
    fn enable_nose(&self, enable: bool) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::EnableNose { enable })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Get the latest control loop statistics, if available.
    fn get_stats(&self) -> PyResult<Option<ControlLoopStats>> {
        self.inner
            .get_stats()
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Perform an asynchronous raw read of motor bytes.
    fn async_read_raw_bytes(&self, id: u8, addr: u8, length: u8) -> PyResult<Vec<u8>> {
        self.inner
            .async_read_raw_bytes(id, addr, length)
            .map_err(|e| {
                pyo3::exceptions::PyRuntimeError::new_err(format!(
                    "Failed to read raw bytes: {}",
                    e
                ))
            })
    }

    /// Perform an asynchronous raw write of motor bytes.
    fn async_write_raw_bytes(&self, id: u8, addr: u8, data: Vec<u8>) -> PyResult<()> {
        self.inner
            .async_write_raw_bytes(id, addr, data)
            .map_err(|e| {
                pyo3::exceptions::PyRuntimeError::new_err(format!(
                    "Failed to write raw bytes: {}",
                    e
                ))
            })
    }

    /// Read PID gains for a given motor id. Returns (P, I, D).
    fn async_read_pid_gains(&self, id: u8) -> PyResult<(u16, u16, u16)> {
        self.inner.async_read_pid_gains(id).map_err(|e| {
            pyo3::exceptions::PyRuntimeError::new_err(format!(
                "Failed to read pid gains: {}",
                e
            ))
        })
    }

    /// Write PID gains for a given motor id.
    fn async_write_pid_gains(&self, id: u8, p: u16, i: u16, d: u16) -> PyResult<()> {
        self.inner.async_write_pid_gains(id, p, i, d).map_err(|e| {
            pyo3::exceptions::PyRuntimeError::new_err(format!(
                "Failed to write pid gains: {}",
                e
            ))
        })
    }

    fn write_raw_packet(&self, data: Py<PyBytes>, py: Python) -> PyResult<Vec<u8>> {
        let bytes = data.as_bytes(py);
        let (tx, rx) = channel();
        self.inner
            .push_command(MotorCommand::WriteRawPacket {
                packet: bytes.to_vec(),
                tx,
            })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        let first_packet = rx.recv().map_err(|e| {
            pyo3::exceptions::PyRuntimeError::new_err(format!(
                "Failed to receive raw packet response: {}",
                e
            ))
        })?;
        Ok(first_packet)
    }
}

#[pyo3::pymodule]
fn dreambo_motor_controller(m: &Bound<'_, PyModule>) -> PyResult<()> {
    pyo3_log::init();

    m.add_class::<DreamboMotorController>()?;
    m.add_class::<DreamboPyControlLoop>()?;
    m.add_class::<DreamboTorsoPosition>()?;
    m.add_class::<ControlLoopStats>()?;

    Ok(())
}

define_stub_info_gatherer!(stub_info);
