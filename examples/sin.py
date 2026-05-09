import numpy as np
import time
from dreambo_motor_controller import DreamboMotorController

def main():
    c = DreamboMotorController(serialport="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79034031-if00")

    c.enable_torque()

    amp = np.deg2rad(30.0)
    freq = 0.25

    t0 = time.time()

    while True:
        t = time.time() - t0
        pos = amp * np.sin(2 * np.pi * freq * t)

        c.set_all_goal_positions([pos] * 7)

        cur = c.read_all_positions()

        errors = np.abs(np.array(cur) - pos)
        print(f"Current position: {cur}, Goal position: {pos}, Errors: {errors}")

        time.sleep(0.01)


if __name__ == "__main__":
    main()