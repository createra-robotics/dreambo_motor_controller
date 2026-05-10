# Dreambo Robot Motor Controller

Handles communication with the gimbal-driven spherical joint arms (FEETECH SM40BL x 4), eyes, nose, eyelid, eyebrow, ears and tail (FEETECH STS3025BL x 12).
Also provides a python binding available via pip.

## Preparation

Install [Dreambo Servo Wizard](https://github.com/createra-robotics/dreambo_servo_wizard) and rename all the servo IDs.

- Gimbal-driven spherical joint arms: all servos are SM40BL
  * Left Arm Pitch Servo ID: 1
  * Left Arm Yaw Servo ID: 2
  * Right Arm Pitch Servo ID: 3
  * Right Arm Yaw Servo ID: 4

- Logarithmic spiral-shaped robotic nose: all servos are STS3025BL
  * Nose Top Servo ID: 5
  * Nose Left Servo ID: 6
  * Nose Right Servo ID: 7

- Coming soon:
  - Left Eyeball Yaw Servo ID: 8
  - Right Eyeball Yaw Servo ID: 9
  - Eyes Pitch Servo ID: 10
  - Eyelid Pitch Servo ID: 11
  - Eyebrow Pitch Servo ID: 12
  - Left Ear Pitch: 13
  - Right Ear Pitch: 14
  - Tail Pitch: 15
  - Tail Yaw: 16
  - Neck Yaw: DM-J4310-2EC
  - Neck Pitch: DM-J4340P-2EC
  - Neck Roll: DM-J4340P-2EC

## To install locally 

```bash
pip install maturin
```

## To build the wheel
```bash
pip install -e . --verbose
```

## To install the wheel

```bash
cd `target/wheels`
pip install dreambo_motor_controller...
```

## Quickstart (Python)

```python
from dreambo_motor_controller import DreamboMotorController

c = DreamboMotorController(serialport="/dev/ttyACM0")
c.enable_torque()

# 7 positions: [left_arm_pitch, left_arm_yaw,
#               right_arm_pitch, right_arm_yaw,
#               nose_top, nose_left, nose_right]
c.set_all_goal_positions([0.0] * 7)

# Or drive groups individually
c.set_left_arm_position([0.1, 0.0])
c.set_right_arm_position([-0.1, 0.0])
c.set_nose_position([0.0, 0.0, 0.0])

print(c.read_all_positions())
```

