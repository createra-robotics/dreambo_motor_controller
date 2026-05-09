from datetime import timedelta
from dreambo_motor_controller import DreamboPyControlLoop
import time
import numpy as np

MOTOR_ID = 1  # left_arm_pitch (SM40BL)
SERIAL_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79034031-if00"

# SM40BL / STS3025BL register layout (relevant subset):
#   addr 13: max_temperature_limit (u8)
#   addr 56: present_position (i16, 2 bytes)
TEMP_LIMIT_ADDR = 13
PRESENT_POSITION_ADDR = 56

TYPE_FROM_LENGTH = {
    1: np.int8,
    2: np.int16,
    4: np.int32,
}


def main():
    control_loop = DreamboPyControlLoop(
        SERIAL_PORT,
        timedelta(seconds=1.0 / 100.0),
        5,
        timedelta(seconds=1),
        timedelta(seconds=30),
    )

    data = control_loop.async_read_raw_bytes(
        id=MOTOR_ID,
        addr=TEMP_LIMIT_ADDR,
        length=1,
    )
    initial_temperature_limit = np.frombuffer(data, dtype=TYPE_FROM_LENGTH[1])
    print(f"Initial temperature limit : {initial_temperature_limit}")
    control_loop.async_write_raw_bytes(
        id=MOTOR_ID,
        addr=TEMP_LIMIT_ADDR,
        data=(initial_temperature_limit + 5).tobytes(),
    )
    data = control_loop.async_read_raw_bytes(
        id=MOTOR_ID,
        addr=TEMP_LIMIT_ADDR,
        length=1,
    )
    modified_temperature_limit = np.frombuffer(data, dtype=TYPE_FROM_LENGTH[1])
    print(f"Modified temperature limit (+5): {modified_temperature_limit}")
    control_loop.async_write_raw_bytes(
        id=MOTOR_ID,
        addr=TEMP_LIMIT_ADDR,
        data=(initial_temperature_limit).tobytes(),
    )

    for _ in range(10):
        data = control_loop.async_read_raw_bytes(
            id=MOTOR_ID,
            addr=PRESENT_POSITION_ADDR,
            length=2,
        )
        present_position = np.frombuffer(data, dtype=TYPE_FROM_LENGTH[2])
        print(f"Present position : {present_position}")
        time.sleep(0.1)


if __name__ == "__main__":
    main()
