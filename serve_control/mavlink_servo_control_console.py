# -*- coding: utf-8 -*-
import os

os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "common"

from pymavlink import mavutil
import sys
import threading
import time


PORT = "COM6"
BAUD = 115200

FULL_SCALE_ANGLE = 45.0
MAV_CMD_DO_SET_ACTUATOR = 187
ACK_TIMEOUT_S = 2.0

# QGC 中 Arm switch channel 配的是 Channel 5。
# 你的遥控器当前是 CH5 低位约 988 时解锁，所以默认 low=armed。
ARM_SWITCH_CHANNEL = 5
ARM_SWITCH_THRESHOLD = 1500
ARM_SWITCH_HIGH_IS_ARMED = False


def angle_to_v(angle_deg: float) -> float:
    v = angle_deg / FULL_SCALE_ANGLE
    return max(-1.0, min(1.0, v))


def boxed_print(lines):
    width = max(len(line) for line in lines)
    border = "+" + "-" * (width + 2) + "+"
    print(border)
    for line in lines:
        print("| " + line.ljust(width) + " |")
    print(border)


def heartbeat_is_armed(heartbeat_msg) -> bool:
    if heartbeat_msg is None:
        return False
    return bool(heartbeat_msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def rc_channel_is_armed(rc_value) -> bool:
    if rc_value is None:
        return False
    if ARM_SWITCH_HIGH_IS_ARMED:
        return rc_value >= ARM_SWITCH_THRESHOLD
    return rc_value <= ARM_SWITCH_THRESHOLD


def get_arm_state(heartbeat_msg, rc_arm_value):
    if rc_arm_value is not None:
        return rc_channel_is_armed(rc_arm_value), f"CH{ARM_SWITCH_CHANNEL}={rc_arm_value}"
    return heartbeat_is_armed(heartbeat_msg), "heartbeat"


def arm_state_text(heartbeat_msg, rc_arm_value):
    armed, source = get_arm_state(heartbeat_msg, rc_arm_value)
    text = "已解锁" if armed else "未解锁"
    return f"{text} ({source})"


def get_rc_arm_value(msg):
    if msg is None:
        return None
    if msg.get_type() not in ("RC_CHANNELS", "RC_CHANNELS_RAW"):
        return None

    value = getattr(msg, f"chan{ARM_SWITCH_CHANNEL}_raw", None)
    if value in (None, 0, 65535):
        return None
    return int(value)


def send_gcs_heartbeat(master, lock):
    with lock:
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )


def start_heartbeat_thread(master, lock, stop_event):
    def worker():
        while not stop_event.is_set():
            try:
                send_gcs_heartbeat(master, lock)
            except Exception as exc:
                print(f"发送地面站 heartbeat 失败: {exc}")
            stop_event.wait(1.0)

    thread = threading.Thread(target=worker, daemon=True)
    thread.start()
    return thread


def drain_status_messages(master, heartbeat_msg=None, rc_arm_value=None):
    while True:
        msg = master.recv_match(
            type=["HEARTBEAT", "RC_CHANNELS", "RC_CHANNELS_RAW"],
            blocking=False,
        )
        if msg is None:
            break

        if msg.get_type() == "HEARTBEAT":
            if msg.get_srcComponent() == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
                heartbeat_msg = msg
            continue

        value = get_rc_arm_value(msg)
        if value is not None:
            rc_arm_value = value

    return heartbeat_msg, rc_arm_value


def wait_for_px4(master, timeout_s=30):
    print("正在等待 PX4 heartbeat...")
    msg = master.wait_heartbeat(timeout=timeout_s)
    if msg is None:
        raise TimeoutError(f"{timeout_s} 秒内没有收到 PX4 heartbeat，请检查串口和波特率。")

    target_system = master.target_system or msg.get_srcSystem()
    target_component = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
    print(f"已收到 PX4 heartbeat: system={target_system}, component={msg.get_srcComponent()}")
    return target_system, target_component, msg


def wait_command_ack(master, command, timeout_s=ACK_TIMEOUT_S):
    deadline = time.time() + timeout_s

    while time.time() < deadline:
        msg = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.2)
        if msg is None:
            continue
        if msg.command != command:
            continue
        return mavutil.mavlink.enums["MAV_RESULT"][msg.result].name

    return None


def send_actuators(master, lock, target_system, target_component, angles):
    values = [angle_to_v(a) for a in angles]

    with lock:
        master.mav.command_long_send(
            target_system,
            target_component,
            MAV_CMD_DO_SET_ACTUATOR,
            0,
            values[0],
            values[1],
            values[2],
            values[3],
            values[4],
            values[5],
            0,
        )

    ack = wait_command_ack(master, MAV_CMD_DO_SET_ACTUATOR)

    print("角度:", angles)
    print("归一化输出:", ["%.3f" % v for v in values])
    if ack is None:
        print("命令回执: 超时/无回复")
    else:
        print(f"命令回执: {ack}")


def parse_angles(line):
    line = line.replace(",", " ")
    return [float(x) for x in line.split()]


def print_intro():
    boxed_print(
        [
            "PX4 舵机控制脚本 - 命令行旧版",
            "功能: MAV_CMD_DO_SET_ACTUATOR 控制 MAIN1-MAIN6",
            "解锁判断: 优先使用遥控器 Channel 5",
            "输入: 6 个角度，例如 10 -10 20 0 30 -30",
            "命令: z=全部回中, s=查看状态, q=回中并退出",
        ]
    )
    print()


def print_status(heartbeat_msg, rc_arm_value):
    armed, _ = get_arm_state(heartbeat_msg, rc_arm_value)
    print(f"飞控状态: {arm_state_text(heartbeat_msg, rc_arm_value)}")
    if rc_arm_value is None:
        print(f"提示: 暂未收到 RC Channel {ARM_SWITCH_CHANNEL}，临时使用 heartbeat 判断。")
    if not armed:
        print("提示: 当前判断为未解锁；若舵机已经能动，请检查 CH5 开关方向阈值设置。")


def main():
    print_intro()
    print(f"正在连接 PX4: {PORT}, {BAUD} baud...")

    master = mavutil.mavlink_connection(
        PORT,
        baud=BAUD,
        source_system=255,
        source_component=190,
        autoreconnect=True,
        force_connected=True,
    )

    lock = threading.Lock()
    stop_event = threading.Event()
    start_heartbeat_thread(master, lock, stop_event)

    try:
        target_system, target_component, heartbeat_msg = wait_for_px4(master)
        rc_arm_value = None

        time.sleep(0.5)
        heartbeat_msg, rc_arm_value = drain_status_messages(master, heartbeat_msg, rc_arm_value)

        print("MAVLink 会话已建立。")
        print_status(heartbeat_msg, rc_arm_value)
        print()

        send_actuators(master, lock, target_system, target_component, [0, 0, 0, 0, 0, 0])

        while True:
            heartbeat_msg, rc_arm_value = drain_status_messages(
                master,
                heartbeat_msg,
                rc_arm_value,
            )
            line = input("角度> ").strip()

            if not line:
                continue

            cmd = line.lower()

            if cmd == "s":
                heartbeat_msg, rc_arm_value = drain_status_messages(
                    master,
                    heartbeat_msg,
                    rc_arm_value,
                )
                print_status(heartbeat_msg, rc_arm_value)
                continue

            if cmd == "q":
                print_status(heartbeat_msg, rc_arm_value)
                send_actuators(master, lock, target_system, target_component, [0, 0, 0, 0, 0, 0])
                time.sleep(0.2)
                print("已回中，退出。")
                break

            if cmd == "z":
                print_status(heartbeat_msg, rc_arm_value)
                send_actuators(master, lock, target_system, target_component, [0, 0, 0, 0, 0, 0])
                continue

            try:
                angles = parse_angles(line)
            except ValueError:
                print("输入格式错误。示例: 10 -10 20 0 30 -30")
                continue

            if len(angles) != 6:
                print("需要输入 6 个角度。示例: 10 -10 20 0 30 -30")
                continue

            print_status(heartbeat_msg, rc_arm_value)
            send_actuators(master, lock, target_system, target_component, angles)
    finally:
        stop_event.set()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n已中断。")
        sys.exit(0)
