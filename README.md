# Dreambo Robot Servo Controller

Handles communication with the gimbal-driven spherical joint arms (FEETECH SM40BL x 4), eyes, nose, eyelid, eyebrow, ears and tail (FEETECH STS3025BL x 12).
Also provides a python binding available via pip.

## Preparation

Install [Dreambo Servo Wizard](https://github.com/createra-robotics/dreambo_servo_wizard) and rename all the servo IDs.

- Left Arm Pitch: 1
- Left Arm Yaw: 2
- Right Arm Pitch: 3
- Right Arm Yaw: 4
- Nose[0]: 5
- Nose[1]: 6
- Nose[2]: 7
- Left Eyeball Yaw: 8
- Right Eyeball Yaw: 9
- Eyes Pitch: 10
- Eyelid Pitch: 11
- Eyebrow Pitch: 12
- Left Ear Pitch: 13
- Right Ear Pitch: 14
- Tail Pitch: 15
- Tail Yaw: 16

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
pip install dreambo_servo_controller...
```

