# -*- coding: utf-8 -*-
import os

os.environ["MAVLINK20"] = "1"
os.environ["MAVLINK_DIALECT"] = "common"

from pymavlink import mavutil
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

try:
    from serial.tools import list_ports
except ImportError:
    list_ports = None

DEFAULT_PORT = "COM6"
BAUD = 115200

MAV_CMD_DO_SET_ACTUATOR = 187
HEARTBEAT_TIMEOUT_S = 30

# QGC 中 Arm switch channel 配的是 Channel 5。
# CH5 高位认为已解锁。如果你的遥控器开关方向相反，把 True 改成 False。
ARM_SWITCH_CHANNEL = 5
ARM_SWITCH_THRESHOLD = 1500
ARM_SWITCH_HIGH_IS_ARMED = True

# 每路配置:
# enabled: 默认是否参与发送。
# angle_limit: 输入角度的满量程。45 表示 +/-45 度映射到 MAVLink -1/+1。
# pwm_min/pwm_max: 只用于界面估算和提醒；真实 PWM 范围要在 QGC Actuators 页面设置。
SERVO_CONFIGS = [
    {"name": "MAIN1", "enabled": True, "angle_limit": 45.0, "pwm_min": 1000, "pwm_max": 2000},
    {"name": "MAIN2", "enabled": True, "angle_limit": 45.0, "pwm_min": 1000, "pwm_max": 2000},
    {"name": "MAIN3", "enabled": True, "angle_limit": 45.0, "pwm_min": 1000, "pwm_max": 2000},
    {"name": "MAIN4", "enabled": True, "angle_limit": 45.0, "pwm_min": 1000, "pwm_max": 2000},
    {"name": "MAIN5", "enabled": False, "angle_limit": 45.0, "pwm_min": 500, "pwm_max": 2500},
    {"name": "MAIN6", "enabled": False, "angle_limit": 45.0, "pwm_min": 500, "pwm_max": 2500},
]


def clamp(value, low, high):
    return max(low, min(high, value))


def angle_to_value(angle_deg, angle_limit):
    if angle_limit <= 0:
        angle_limit = 45.0
    return clamp(angle_deg / angle_limit, -1.0, 1.0)


def value_to_pwm(value, pwm_min, pwm_max):
    center = (pwm_min + pwm_max) / 2.0
    half_range = (pwm_max - pwm_min) / 2.0
    return int(round(center + clamp(value, -1.0, 1.0) * half_range))


def rc_channel_is_armed(rc_value):
    if rc_value is None:
        return False
    if ARM_SWITCH_HIGH_IS_ARMED:
        return rc_value >= ARM_SWITCH_THRESHOLD
    return rc_value <= ARM_SWITCH_THRESHOLD


def heartbeat_is_armed(heartbeat_msg):
    if heartbeat_msg is None:
        return False
    return bool(heartbeat_msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def get_rc_arm_value(msg):
    if msg is None:
        return None
    if msg.get_type() not in ("RC_CHANNELS", "RC_CHANNELS_RAW"):
        return None

    value = getattr(msg, f"chan{ARM_SWITCH_CHANNEL}_raw", None)
    if value in (None, 0, 65535):
        return None
    return int(value)


class ServoControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("PX4 舵机控制")
        self.root.geometry("820x520")
        self.root.minsize(760, 470)

        self.master = None
        self.target_system = None
        self.target_component = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.event_queue = queue.Queue()
        self.connected = False
        self.connection_id = 0
        self.heartbeat_started = False
        self.latest_heartbeat = None
        self.rc_arm_value = None
        self.last_ack = "无"

        self.rows = []
        self.port_var = tk.StringVar(value=DEFAULT_PORT)
        self.status_var = tk.StringVar(value="未连接")
        self.arm_var = tk.StringVar(value="解锁状态: 未知")
        self.ack_var = tk.StringVar(value="命令回执: 无")

        self.build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(100, self.process_events)
        self.connect_async()

    def build_ui(self):
        intro = (
            "PX4 舵机控制页面\n"
            "1. 勾选实际使用的 MAIN 通道，未勾选通道发送 0。\n"
            "2. 不同舵机的 1000-2000us / 500-2500us 请在 QGC Actuators 的 Minimum/Maximum 设置。\n"
            "3. 本页面的 PWM 范围只用于估算显示，发送给 PX4 的仍是 -1 到 +1 归一化值。"
        )
        intro_frame = ttk.LabelFrame(self.root, text="使用提示")
        intro_frame.pack(fill="x", padx=12, pady=(12, 8))
        ttk.Label(intro_frame, text=intro, justify="left").pack(anchor="w", padx=10, pady=8)

        port_frame = ttk.LabelFrame(self.root, text="连接设置")
        port_frame.pack(fill="x", padx=12, pady=(0, 8))
        ttk.Label(port_frame, text="串口:").pack(side="left", padx=(10, 4), pady=8)
        self.port_combo = ttk.Combobox(
            port_frame,
            textvariable=self.port_var,
            values=self.get_available_ports(),
            width=12,
        )
        self.port_combo.pack(side="left", padx=(0, 8), pady=8)
        ttk.Label(port_frame, text=f"波特率: {BAUD}").pack(side="left", padx=(0, 12), pady=8)
        ttk.Button(port_frame, text="刷新端口", command=self.refresh_ports).pack(side="left", padx=(0, 8), pady=8)
        ttk.Button(port_frame, text="连接/重连", command=self.connect_async).pack(side="left", pady=8)

        status_frame = ttk.Frame(self.root)
        status_frame.pack(fill="x", padx=12, pady=(0, 8))
        ttk.Label(status_frame, textvariable=self.status_var).pack(side="left", padx=(0, 18))
        ttk.Label(status_frame, textvariable=self.arm_var).pack(side="left", padx=(0, 18))
        ttk.Label(status_frame, textvariable=self.ack_var).pack(side="left")

        table = ttk.Frame(self.root)
        table.pack(fill="both", expand=True, padx=12, pady=8)

        headers = ["启用", "通道", "角度", "满量程角度", "PWM范围", "估算PWM", "操作"]
        for col, title in enumerate(headers):
            ttk.Label(table, text=title).grid(row=0, column=col, sticky="w", padx=4, pady=4)

        table.columnconfigure(2, weight=1)

        for index, config in enumerate(SERVO_CONFIGS):
            self.add_servo_row(table, index, config)

        buttons = ttk.Frame(self.root)
        buttons.pack(fill="x", padx=12, pady=(4, 12))
        ttk.Button(buttons, text="发送启用通道", command=self.send_enabled).pack(side="left")
        ttk.Button(buttons, text="全部回中", command=self.center_all).pack(side="left", padx=8)
        ttk.Button(buttons, text="全部启用", command=lambda: self.set_all_enabled(True)).pack(side="left")
        ttk.Button(buttons, text="全部禁用", command=lambda: self.set_all_enabled(False)).pack(side="left", padx=8)
        ttk.Button(buttons, text="查看状态", command=self.refresh_status_text).pack(side="right")

    def get_available_ports(self):
        if list_ports is None:
            return [self.port_var.get() or DEFAULT_PORT]

        ports = [port.device for port in list_ports.comports()]
        current = self.port_var.get() or DEFAULT_PORT
        if current and current not in ports:
            ports.insert(0, current)
        return ports or [current]

    def refresh_ports(self):
        ports = self.get_available_ports()
        self.port_combo.configure(values=ports)
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])
        self.status_var.set("端口列表已刷新")

    def add_servo_row(self, parent, index, config):
        enabled_var = tk.BooleanVar(value=config["enabled"])
        angle_var = tk.DoubleVar(value=0.0)
        limit_var = tk.DoubleVar(value=config["angle_limit"])
        min_var = tk.IntVar(value=config["pwm_min"])
        max_var = tk.IntVar(value=config["pwm_max"])
        pwm_var = tk.StringVar(value="1500")

        row = {
            "enabled": enabled_var,
            "angle": angle_var,
            "limit": limit_var,
            "pwm_min": min_var,
            "pwm_max": max_var,
            "pwm": pwm_var,
        }
        self.rows.append(row)

        r = index + 1
        ttk.Checkbutton(parent, variable=enabled_var).grid(row=r, column=0, padx=4, pady=5)
        ttk.Label(parent, text=config["name"]).grid(row=r, column=1, sticky="w", padx=4)

        angle_entry = ttk.Entry(parent, textvariable=angle_var, width=8)
        angle_entry.grid(row=r, column=2, sticky="ew", padx=4)
        angle_entry.bind("<Return>", lambda _event, i=index: self.send_one(i))
        angle_entry.bind("<KeyRelease>", lambda _event, i=index: self.update_pwm_label(i))

        limit_entry = ttk.Entry(parent, textvariable=limit_var, width=8)
        limit_entry.grid(row=r, column=3, padx=4)
        limit_entry.bind("<Return>", lambda _event, i=index: self.update_limit(i))
        limit_entry.bind("<KeyRelease>", lambda _event, i=index: self.update_pwm_label(i))

        pwm_frame = ttk.Frame(parent)
        pwm_frame.grid(row=r, column=4, padx=4)
        min_entry = ttk.Entry(pwm_frame, textvariable=min_var, width=6)
        min_entry.pack(side="left")
        ttk.Label(pwm_frame, text="-").pack(side="left")
        max_entry = ttk.Entry(pwm_frame, textvariable=max_var, width=6)
        max_entry.pack(side="left")
        min_entry.bind("<Return>", lambda _event, i=index: self.update_pwm_label(i))
        max_entry.bind("<Return>", lambda _event, i=index: self.update_pwm_label(i))
        min_entry.bind("<KeyRelease>", lambda _event, i=index: self.update_pwm_label(i))
        max_entry.bind("<KeyRelease>", lambda _event, i=index: self.update_pwm_label(i))

        ttk.Label(parent, textvariable=pwm_var, width=8).grid(row=r, column=5, padx=4)
        ttk.Button(parent, text="发送本路", command=lambda i=index: self.send_one(i)).grid(row=r, column=6, padx=4)

        self.update_pwm_label(index)

    def connect_async(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("缺少端口", "请选择或输入一个串口，例如 COM6。")
            return

        self.connection_id += 1
        connection_id = self.connection_id
        self.connected = False
        self.latest_heartbeat = None
        self.rc_arm_value = None
        self.target_system = None
        self.ack_var.set("命令回执: 无")

        old_master = self.master
        self.master = None
        if old_master is not None:
            try:
                old_master.close()
            except Exception:
                pass

        self.status_var.set(f"正在连接: {port}, {BAUD}...")
        self.arm_var.set("解锁状态: 等待连接")
        thread = threading.Thread(target=self.connect_worker, args=(port, connection_id), daemon=True)
        thread.start()

    def connect_worker(self, port, connection_id):
        try:
            master = mavutil.mavlink_connection(
                port,
                baud=BAUD,
                source_system=255,
                source_component=190,
                autoreconnect=True,
                force_connected=True,
            )
            if connection_id != self.connection_id:
                master.close()
                return

            self.master = master
            self.start_heartbeat_thread_once()
            msg = master.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT_S)
            if msg is None:
                raise TimeoutError("等待 PX4 heartbeat 超时")
            if connection_id != self.connection_id:
                master.close()
                return

            self.target_system = master.target_system or msg.get_srcSystem()
            self.latest_heartbeat = msg
            self.connected = True
            self.event_queue.put(
                (
                    "connected",
                    f"已连接: {port}, system={self.target_system}, component={msg.get_srcComponent()}",
                )
            )
            self.start_receive_thread(master, connection_id)
        except Exception as exc:
            if connection_id == self.connection_id and self.master is not None:
                try:
                    self.master.close()
                except Exception:
                    pass
                self.master = None
                self.connected = False
            self.event_queue.put(("error", f"连接失败: {port}: {exc}"))

    def start_heartbeat_thread_once(self):
        if self.heartbeat_started:
            return
        self.heartbeat_started = True

        def worker():
            while not self.stop_event.is_set():
                if self.master is not None:
                    try:
                        with self.lock:
                            self.master.mav.heartbeat_send(
                                mavutil.mavlink.MAV_TYPE_GCS,
                                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                0,
                                0,
                                mavutil.mavlink.MAV_STATE_ACTIVE,
                            )
                    except Exception as exc:
                        self.event_queue.put(("error", f"发送 heartbeat 失败: {exc}"))
                self.stop_event.wait(1.0)

        threading.Thread(target=worker, daemon=True).start()

    def start_receive_thread(self, master, connection_id):
        def worker():
            while (
                not self.stop_event.is_set()
                and connection_id == self.connection_id
                and master is self.master
            ):
                try:
                    msg = master.recv_match(blocking=True, timeout=0.2)
                except Exception as exc:
                    if connection_id != self.connection_id:
                        break
                    self.event_queue.put(("error", f"接收 MAVLink 消息失败: {exc}"))
                    time.sleep(0.5)
                    continue

                if msg is None:
                    continue

                msg_type = msg.get_type()
                if msg_type == "HEARTBEAT":
                    if msg.get_srcComponent() == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
                        self.latest_heartbeat = msg
                        self.event_queue.put(("status", None))
                elif msg_type in ("RC_CHANNELS", "RC_CHANNELS_RAW"):
                    value = get_rc_arm_value(msg)
                    if value is not None:
                        self.rc_arm_value = value
                        self.event_queue.put(("status", None))
                elif msg_type == "COMMAND_ACK":
                    if msg.command == MAV_CMD_DO_SET_ACTUATOR:
                        result = mavutil.mavlink.enums["MAV_RESULT"][msg.result].name
                        self.last_ack = result
                        self.event_queue.put(("ack", result))

        threading.Thread(target=worker, daemon=True).start()

    def process_events(self):
        while True:
            try:
                event, payload = self.event_queue.get_nowait()
            except queue.Empty:
                break

            if event == "connected":
                self.status_var.set(payload)
                self.refresh_status_text()
            elif event == "status":
                self.refresh_status_text()
            elif event == "ack":
                self.ack_var.set(f"命令回执: {payload}")
            elif event == "error":
                self.status_var.set(payload)

        self.root.after(100, self.process_events)

    def get_arm_state_text(self):
        if self.rc_arm_value is not None:
            armed = rc_channel_is_armed(self.rc_arm_value)
            text = "已解锁" if armed else "未解锁"
            return f"解锁状态: {text} (CH{ARM_SWITCH_CHANNEL}={self.rc_arm_value})"

        armed = heartbeat_is_armed(self.latest_heartbeat)
        text = "已解锁" if armed else "未知/未解锁"
        return f"解锁状态: {text} (heartbeat)"

    def refresh_status_text(self):
        self.arm_var.set(self.get_arm_state_text())

    def update_limit(self, index):
        row = self.rows[index]
        limit = abs(float(row["limit"].get()))
        if limit <= 0:
            limit = 45.0
            row["limit"].set(limit)
        angle = clamp(float(row["angle"].get()), -limit, limit)
        row["angle"].set(angle)
        self.update_pwm_label(index)

    def update_pwm_label(self, index):
        row = self.rows[index]
        try:
            angle = float(row["angle"].get())
            limit = abs(float(row["limit"].get()))
            pwm_min = int(row["pwm_min"].get())
            pwm_max = int(row["pwm_max"].get())
        except (tk.TclError, ValueError):
            return

        value = angle_to_value(angle, limit)
        row["pwm"].set(str(value_to_pwm(value, pwm_min, pwm_max)))

    def build_values(self):
        values = []
        angles = []
        for index, row in enumerate(self.rows):
            self.update_limit(index)
            enabled = bool(row["enabled"].get())
            angle = float(row["angle"].get()) if enabled else 0.0
            limit = abs(float(row["limit"].get()))
            value = angle_to_value(angle, limit)
            angles.append(angle)
            values.append(value)
        return angles, values

    def send_values(self, values):
        if not self.connected or self.master is None or self.target_system is None:
            messagebox.showwarning("未连接", "还没有连接到 PX4。")
            return

        with self.lock:
            self.master.mav.command_long_send(
                self.target_system,
                self.target_component,
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
        self.ack_var.set("命令回执: 等待中...")

    def send_enabled(self):
        angles, values = self.build_values()
        self.send_values(values)
        print("角度:", angles)
        print("归一化输出:", ["%.3f" % value for value in values])

    def send_one(self, index):
        self.update_limit(index)
        self.rows[index]["enabled"].set(True)
        self.send_enabled()

    def center_all(self):
        for row in self.rows:
            row["angle"].set(0.0)
        for index in range(len(self.rows)):
            self.update_pwm_label(index)
        self.send_enabled()

    def set_all_enabled(self, enabled):
        for row in self.rows:
            row["enabled"].set(enabled)

    def on_close(self):
        self.stop_event.set()
        if self.master is not None:
            try:
                self.master.close()
            except Exception:
                pass
        self.root.destroy()


def main():
    root = tk.Tk()
    app = ServoControlApp(root)
    root.mainloop()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n已中断。")
        sys.exit(0)
