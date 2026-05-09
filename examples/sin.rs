//! Dreambo 正弦运动示例
//!
//! 本示例驱动 Dreambo 机器人的全部 7 个舵机（4 个手臂 SM40BL +
//! 3 个鼻子 STS3025BL），让它们同步跟随同一条正弦曲线，用来验证
//! 总线通信和位置跟踪是否正常。
//!
//! 执行流程：
//!   1. 打开串口并初始化 `DreamboMotorController`，控制器会在 SM40BL
//!      与 STS3025BL 各自的波特率之间自动切换。
//!   2. 使能所有舵机的扭矩。
//!   3. 注册 Ctrl-C 处理器：按下 Ctrl-C 时把原子布尔置为 false，
//!      用于优雅退出。
//!   4. 主循环：基于经过时间 t 计算 `pos = 30° × sin(2π × 0.25 × t)`，
//!      将同一个目标位置同时下发给 7 个舵机；随后回读当前位置并
//!      打印每个舵机的跟踪误差。
//!   5. 收到 Ctrl-C 后跳出循环，关闭所有舵机扭矩，让机器人自然下垂
//!      后退出。

use std::f64::consts::PI;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use dreambo_motor_controller::DreamboMotorController;

fn main() {
    let serialport = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79034031-if00";
    let mut c = DreamboMotorController::new(serialport).unwrap();

    c.enable_torque().unwrap();

    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || r.store(false, Ordering::SeqCst))
        .expect("failed to install Ctrl-C handler");

    let t0 = std::time::Instant::now();
    let amp = 30.0_f64.to_radians();
    let freq = 0.25;

    while running.load(Ordering::SeqCst) {
        let t = t0.elapsed().as_secs_f64();
        let pos = (2.0 * PI * freq * t).sin() * amp;

        c.set_all_goal_positions([pos; 7]).unwrap();

        let cur = c.read_all_positions().unwrap();

        let errors = cur
            .iter()
            .zip([pos; 7].iter())
            .map(|(cur, goal)| (cur - goal).abs())
            .collect::<Vec<_>>();
        println!("Errors: {:?}", errors);
    }

    println!("Disabling torque...");
    if let Err(e) = c.disable_torque() {
        eprintln!("Failed to disable torque on shutdown: {e}");
    }
}
