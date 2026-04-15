#!/usr/bin/env python3
"""
GELLO hardware diagnostic — tests both left and right GELLO devices
and reports which joints are readable.

Usage (from inside the container):
    docker compose exec bimanual_teleop python3 test_gello.py
"""

import sys
import time
import numpy as np

LEFT_PORT  = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO5209-if00-port0"
RIGHT_PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO528D-if00-port0"

# Standard GELLO config for UR (6 joints + gripper = IDs 1-7)
JOINT_IDS     = (1, 2, 3, 4, 5, 6, 7)
JOINT_OFFSETS = (0.0,) * 7
JOINT_SIGNS   = (1,) * 7


def test_gello(name, port):
    print(f"\n{'='*50}")
    print(f"  {name} GELLO  ->  {port.split('/')[-1]}")
    print(f"{'='*50}")

    try:
        from gello.robots.dynamixel import DynamixelRobot
        robot = DynamixelRobot(
            joint_ids=JOINT_IDS,
            joint_offsets=JOINT_OFFSETS,
            joint_signs=JOINT_SIGNS,
            port=port,
            baudrate=57600,
        )
    except Exception as e:
        print(f"  FAIL  Failed to initialise: {e}")
        return False

    print(f"  Driver OK -- reading 10 samples...\n")

    samples = []
    failures = 0

    for i in range(10):
        try:
            q = robot.get_joint_state()
            samples.append(q)
            deg = [round(float(v) * 180 / 3.14159, 1) for v in q]
            print(f"  [{i+1:02d}]  {deg}")
        except Exception as e:
            failures += 1
            print(f"  [{i+1:02d}]  READ FAILED: {e}")
        time.sleep(0.2)

    print()
    if len(samples) == 0:
        print("  FAIL  No readings -- device unresponsive.")
        return False

    arr = np.array(samples)
    ranges = arr.max(axis=0) - arr.min(axis=0)
    print(f"  Samples  : {len(samples)}/10")
    print(f"  Failures : {failures}/10")
    print(f"  Range    : {[round(float(r)*180/3.14159, 2) for r in ranges]} deg")

    stuck = [i+1 for i, r in enumerate(ranges) if r < 1e-4]
    if stuck:
        print(f"  WARN  Joints with zero movement (stuck/dead): {stuck}")

    if failures > 5:
        print("  FAIL  Too many read failures -- cable or servo issue.")
        return False

    print("  OK  GELLO readable.")
    return True


if __name__ == "__main__":
    left_ok  = test_gello("LEFT ",  LEFT_PORT)
    right_ok = test_gello("RIGHT", RIGHT_PORT)

    print(f"\n{'='*50}")
    print(f"  LEFT  GELLO : {'OK     ' if left_ok  else 'FAILED '}")
    print(f"  RIGHT GELLO : {'OK     ' if right_ok else 'FAILED '}")
    print(f"{'='*50}\n")

    sys.exit(0 if (left_ok and right_ok) else 1)
