#!/usr/bin/env python3
"""Print live attitude (roll/pitch/yaw) from the flight controller (CubeOrange+) over MAVLink/USB."""
import math
import time

from pymavlink import mavutil

FCC_PORT = "/dev/ttyACM0"
BAUD = 115200


def main():
    print(f"Connecting to FCC on {FCC_PORT} @ {BAUD}...")
    master = mavutil.mavlink_connection(FCC_PORT, baud=BAUD)
    master.wait_heartbeat()
    print(f"Heartbeat received (sysid={master.target_system}, compid={master.target_component})")

    while True:
        msg = master.recv_match(type="ATTITUDE", blocking=True, timeout=5)
        if msg is None:
            print("No ATTITUDE message received (timeout)")
            continue
        roll = math.degrees(msg.roll)
        pitch = math.degrees(msg.pitch)
        yaw = math.degrees(msg.yaw)
        print(f"roll={roll:7.2f}  pitch={pitch:7.2f}  yaw={yaw:7.2f}")
        time.sleep(0.1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
