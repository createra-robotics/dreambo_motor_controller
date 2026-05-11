//! Home the 3 neck joints (NECK_YAW, NECK_PITCH, NECK_ROLL) to the nearest
//! multiple of 2π — a quick smoke test that the motorcom CAN integration
//! is wired up correctly through `DreamboMotorController`.
//!
//! This goes through the high-level controller (not raw motorcom) on
//! purpose: it exercises `enable_neck`, `read_neck_positions`,
//! `set_neck_yaw/pitch/roll_position`, and `set_neck_gains` — i.e. exactly
//! the same surface that downstream code will use.
//!
//! The Feetech serial bus (arms + nose) is **not** opened — neck-only mode.
//! Pass `--serial-port <path>` to also bring up that bus if you want a
//! full-system smoke test.
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
//!
//! ## What it does
//!
//! 1. Opens the serial bus (for arms/nose — they stay torque-off) and the
//!    CAN bus (for the neck Damiao motors).
//! 2. Reads the starting position of each neck joint via a Damiao feedback
//!    request.
//! 3. Computes the nearest multiple of 2π for each joint — after a power
//!    cycle the multi-turn counter resets, so the joint can report any
//!    angle; rounding the goal to the nearest 2π keeps travel within ±π.
//! 4. Enables neck torque (forces MIT mode then sends `enable`).
//! 5. Ramps each joint to its goal at `--speed` rad/s. The MIT impedance
//!    gains come from the controller's per-motor defaults (kp=30/kd=1 for
//!    yaw, kp=60/kd=2 for pitch/roll); override via `--kp` / `--kd` if
//!    you want softer or stiffer behaviour during the smoke test.
//! 6. Once all three settle within `--tolerance` rad of their goals, the
//!    example disables neck torque and exits.

use std::{
    process,
    sync::{atomic::{AtomicBool, Ordering}, Arc},
    thread,
    time::{Duration, Instant},
};

use clap::Parser;
use dreambo_motor_controller::{DreamboMotorController, DEFAULT_CAN_BUS};

const TWO_PI: f64 = 2.0 * std::f64::consts::PI;
const DT: Duration = Duration::from_millis(10);
const JOINT_NAMES: [&str; 3] = ["neck_yaw", "neck_pitch", "neck_roll"];

#[derive(Parser, Debug)]
#[command(about = "Home the dreambo neck joints to nearest 2π via motorcom.")]
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

    /// Optional MIT kp override applied to all three joints (otherwise the
    /// per-motor defaults baked into DreamboMotorController are used).
    #[arg(long)]
    kp: Option<f32>,

    /// Optional MIT kd override applied to all three joints.
    #[arg(long)]
    kd: Option<f32>,
}

fn main() {
    if let Err(e) = run() {
        eprintln!("Error: {e}");
        process::exit(1);
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    let running = Arc::new(AtomicBool::new(true));
    {
        let r = running.clone();
        ctrlc::set_handler(move || r.store(false, Ordering::SeqCst))
            .expect("failed to install Ctrl-C handler");
    }

    let mut c = DreamboMotorController::new(args.serial_port.as_deref(), &args.can_bus)?;

    // Override gains if the user asked for it. Use whichever combination is
    // provided; missing fields keep the per-motor default.
    if args.kp.is_some() || args.kd.is_some() {
        for i in 0..3 {
            let kp = args.kp.unwrap_or(if i == 0 { 30.0 } else { 60.0 });
            let kd = args.kd.unwrap_or(if i == 0 { 1.0 } else { 2.0 });
            c.set_neck_gains(i, kp, kd)?;
            println!("  {} gains overridden: kp={} kd={}", JOINT_NAMES[i], kp, kd);
        }
    }

    let missing = c.check_missing_neck_motors();
    if !missing.is_empty() {
        return Err(format!("neck motors not responding on CAN: {:?}", missing).into());
    }

    let start = c.read_neck_positions()?;
    let mut goals = [0.0_f64; 3];
    for i in 0..3 {
        goals[i] = (start[i] / TWO_PI).round() * TWO_PI;
        println!(
            "  {} at {:+.3} rad, homing to {:+.3} rad (distance {:+.3})",
            JOINT_NAMES[i],
            start[i],
            goals[i],
            start[i] - goals[i],
        );
    }

    println!("Enabling neck torque...");
    c.enable_neck(true)?;

    let mut targets = start;
    let step = args.speed * DT.as_secs_f64();
    let max_distance = (0..3)
        .map(|i| (start[i] - goals[i]).abs())
        .fold(0.0_f64, f64::max);
    let max_iters = (max_distance / step) as usize + 2_000;

    let mut settled = [false; 3];
    let mut all_settled = false;
    let t0 = Instant::now();

    for i in 0..max_iters {
        if !running.load(Ordering::SeqCst) {
            println!("Interrupted — disabling neck torque.");
            break;
        }

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

        let observed = c.set_neck_position(targets)?;

        for j in 0..3 {
            if !settled[j] && (targets[j] - goals[j]).abs() < f64::EPSILON
                && (observed[j] - goals[j]).abs() < args.tolerance
            {
                println!(
                    "  {} reached zero (pos={:+.4})",
                    JOINT_NAMES[j], observed[j],
                );
                settled[j] = true;
            }
        }

        if settled.iter().all(|&s| s) {
            all_settled = true;
            break;
        }

        if i % 100 == 0 {
            println!(
                "  t={:5.2}s  target={:+.3?}  observed={:+.3?}",
                t0.elapsed().as_secs_f64(),
                targets,
                observed,
            );
        }

        thread::sleep(DT);
    }

    if !all_settled && running.load(Ordering::SeqCst) {
        eprintln!("Timed out before all neck joints settled at zero.");
    }

    if let Err(e) = c.enable_neck(false) {
        eprintln!("Failed to disable neck torque on shutdown: {e}");
    }
    println!("Done.");
    Ok(())
}