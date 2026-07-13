import os
os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "common"

from pymavlink import mavutil
import math
import time
import sys

PORT = "COM12"
BAUD = 115200

FULL_SCALE_ANGLE = 45.0
MAV_CMD_DO_SET_ACTUATOR = 187


def angle_to_v(angle_deg: float) -> float:
    v = angle_deg / FULL_SCALE_ANGLE
    return max(-1.0, min(1.0, v))


def send_actuators(master, angles):
    """
    angles[0] -> MAIN1
    angles[1] -> MAIN2
    angles[2] -> MAIN3
    angles[3] -> MAIN4
    angles[4] -> MAIN5
    angles[5] -> MAIN6
    """

    values = [angle_to_v(a) for a in angles]

    # 固定指定 PX4 目标
    target_system = 1
    target_component = 1

    master.mav.command_long_send(
        target_system,
        target_component,
        MAV_CMD_DO_SET_ACTUATOR,
        0,
        values[0],   # param1 -> Offboard Actuator Set 1 -> MAIN1
        values[1],   # param2 -> Offboard Actuator Set 2 -> MAIN2
        values[2],   # param3 -> Offboard Actuator Set 3 -> MAIN3
        values[3],   # param4 -> Offboard Actuator Set 4 -> MAIN4
        values[4],   # param5 -> Offboard Actuator Set 5 -> MAIN5
        values[5],   # param6 -> Offboard Actuator Set 6 -> MAIN6
        0            # param7
    )

    print("angles:", angles)
    print("values:", ["%.3f" % v for v in values])


def parse_angles(line):
    line = line.replace(",", " ")
    return [float(x) for x in line.split()]


def main():
    print(f"Connecting to PX4 on {PORT} ...")

    master = mavutil.mavlink_connection(
        PORT,
        baud=BAUD,
        source_system=255,
        source_component=190
    )

    # 不 wait_heartbeat，避免 pymavlink 接收消息时崩溃
    time.sleep(2.0)

    print("Connected. 不等待 heartbeat，直接发送 MAVLink command。")
    print("输入 6 个角度，例如：10 -10 20 0 30 -30")
    print("对应关系：MAIN1 MAIN2 MAIN3 MAIN4 MAIN5 MAIN6")
    print("输入 z：全部回中")
    print("输入 q：回中并退出")
    print()

    send_actuators(master, [0, 0, 0, 0, 0, 0])

    while True:
        line = input("angles> ").strip()

        if not line:
            continue

        if line.lower() == "q":
            send_actuators(master, [0, 0, 0, 0, 0, 0])
            time.sleep(0.2)
            print("Exit.")
            break

        if line.lower() == "z":
            send_actuators(master, [0, 0, 0, 0, 0, 0])
            continue

        try:
            angles = parse_angles(line)
        except ValueError:
            print("输入错误。示例：10 -10 20 0 30 -30")
            continue

        if len(angles) != 6:
            print("需要输入 6 个角度。示例：10 -10 20 0 30 -30")
            continue

        send_actuators(master, angles)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted.")
        sys.exit(0)