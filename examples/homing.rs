//! Smoke-test for the `neck_home()` API: drives the 3 neck joints to the
//! nearest physical zero (multi-turn-safe — see `neck_home` docs).
//!
//! The Feetech serial bus (arms + nose) is **not** opened by default —
//! neck-only mode. Pass `--serial-port <path>` to also bring up that bus.
//!
//! ## Bring the CAN bus up first
//!
//! ```text
//! sudo ip link set can0 up type can bitrate 1000000
//! ```
//!
//! ## Run
//!
//! ```text
//! cargo run --example homing
//!
//! # Optional flags (defaults shown):
//! cargo run --example homing -- \
//!     --can-bus can0 \
//!     --speed 0.4 \
//!     --tolerance 0.15
//!
//! # Also open the Feetech bus (arms+nose stay torque-off):
//! cargo run --example homing -- --serial-port /dev/ttyACM0
//! ```

use std::process;

use clap::Parser;
use dreambo_motor_controller::{DreamboMotorController, DEFAULT_CAN_BUS};

#[derive(Parser, Debug)]
#[command(about = "Home the dreambo neck joints to nearest 2π via neck_home().")]
struct Args {
    /// Serial port for the arm/nose Feetech bus (optional — omit for neck-only).
    #[arg(short, long)]
    serial_port: Option<String>,

    /// SocketCAN interface for the neck Damiao motors.
    #[arg(long, default_value = DEFAULT_CAN_BUS)]
    can_bus: String,

    /// Ramp speed (rad/s).
    #[arg(long, default_value_t = 0.4)]
    speed: f64,

    /// "Settled" position tolerance (rad).
    #[arg(long, default_value_t = 0.15)]
    tolerance: f64,

    /// Optional MIT kp override applied to all three joints.
    #[arg(long)]
    kp: Option<f32>,

    /// Optional MIT kd override applied to all three joints.
    #[arg(long)]
    kd: Option<f32>,

    /// Disable neck torque before exiting (default: leave torque on, matching the API).
    #[arg(long)]
    disable_after: bool,
}

fn main() {
    if let Err(e) = run() {
        eprintln!("Error: {e}");
        process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    let mut c = DreamboMotorController::new(args.serial_port.as_deref(), &args.can_bus)?;

    let missing = c.check_missing_neck_motors();
    if !missing.is_empty() {
        return Err(format!("neck motors not responding on CAN: {:?}", missing).into());
    }

    let start = c.read_neck_positions()?;
    println!("Starting positions (rad): {:+.3?}", start);
    println!(
        "Homing at {:.2} rad/s (tolerance {:.3} rad)...",
        args.speed, args.tolerance
    );

    let observed = c.neck_home(args.speed, args.tolerance, args.kp, args.kd)?;
    println!("Settled at: {:+.4?}", observed);

    if args.disable_after {
        c.enable_neck(false)?;
        println!("Neck torque disabled.");
    }

    Ok(())
}
