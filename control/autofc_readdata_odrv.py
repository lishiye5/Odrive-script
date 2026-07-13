#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
与 fc_readdata.py 功能一致的脚本，区别：
  - 连接方式：使用 odrive.find_any() 替代串口 ASCII 协议
  - 回中周期：1 圈（原版为 16 圈）
  - 新增 odrivetool 风格的速度/位置交互控制指令
"""

import odrive
from odrive.enums import *
import time
import threading
import os
import csv
import math

# --- 日志目录与文件 ---
log_folder = "data/raw"
if not os.path.exists(log_folder):
    os.makedirs(log_folder)

filename = f"{log_folder}/experiment_{time.strftime('%m%d_%H%M%S')}.csv"

with open(filename, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["time", "iq", "vbus", "target_vel", "position"])

# --- 回中周期（1 圈） ---
RATIO = 1

# --- 帮助信息 ---
HELP_TEXT = """
╔══════════════════════════════════════════════════════════╗
║  ODrive 交互控制指令                                     ║
╠══════════════════════════════════════════════════════════╣
║  m             开关监测（Iq/vbus/目标速度/位置 → CSV）   ║
║  sweep         自动速度扫描：1→20 turns/s，每30s切换      ║
║                同时自动开启监测，完成后自动停止            ║
║  ss            手动停止自动扫描                           ║
║  h             回中（回到最近 {ratio} 圈整数倍）              ║
║  v <speed>     速度控制模式，目标速度（turns/s）            ║
║                 例: v 2.0   v -1.5   v 0                 ║
║  cycle <v1> <v2> 周期变速：0–180° 用 v1，180–360° 用 v2 ║
║                 例: cycle 3.0 0.6                      ║
║  sc            停止周期变速（目标速度归零）              ║
║  p <pos>       位置控制模式，目标位置（turns）              ║
║                 例: p 5.0   p -3.0   p 0                 ║
║  idle          电机释放（IDLE 状态）                       ║
║  closed        电机进入闭环运行                            ║
║  state         查看当前电机状态                            ║
║  help / ?      显示本帮助                                  ║
║  exit          退出程序                                    ║
╚══════════════════════════════════════════════════════════╝
""".format(ratio=RATIO)


class ODriveMonitor:
    def __init__(self):
        print("正在寻找 ODrive 设备...")
        self.odrv = odrive.find_any()
        print("ODrive 已连接！")

        self.axis = self.odrv.axis0

        self.running = True
        self.monitoring = False
        self.auto_sweep = False       # 自动速度扫描标志
        self.cycle_velocity = False   # 周期内两段速度控制标志
        self.cycle_forward_vel = 0.0
        self.cycle_return_vel = 0.0
        self.target_vel = 0.0         # 当前目标速度
        self.lock = threading.Lock()
        self.start_time = time.time()
        self._vbus = 0.0  # 由独立线程更新的缓存值
        self._vbus_lock = threading.Lock()

        # 打开 CSV 文件用于持续写入
        self.f = open(filename, 'a', newline='')
        self.writer = csv.writer(self.f)

    # ── 基础读取 ──────────────────────────────

    def get_Iq(self):
        with self.lock:
            return self.axis.motor.current_control.Iq_measured

    def get_vbus(self):
        with self.lock:
            return self.odrv.vbus_voltage

    def get_pos(self):
        with self.lock:
            return self.axis.encoder.pos_estimate

    def get_vel(self):
        with self.lock:
            return self.axis.encoder.vel_estimate

    # ── 控制指令 ──────────────────────────────

    def set_velocity(self, speed, require_sweep=False):
        """切换到速度控制模式，设置目标速度（turns/s）"""
        speed = float(speed)
        if not math.isfinite(speed):
            raise ValueError("速度必须是有限数")
        if not require_sweep:
            self.cycle_velocity = False
        with self.lock:
            # 自动扫频在等锁期间可能已被 cycle/p/h/idle 取消。
            if require_sweep and not self.auto_sweep:
                return False
            if require_sweep:
                self.cycle_velocity = False
            self.axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            self.axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
            self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self.axis.controller.input_vel = speed
            self.target_vel = speed
        print(f"[速度] 目标速度: {speed:.3f} turns/s")
        return True

    def start_cycle_velocity(self, forward_vel, return_vel):
        """按编码器每圈相位切换两档速度。"""
        forward_vel = float(forward_vel)
        return_vel = float(return_vel)
        if not all(math.isfinite(v) and v != 0.0 for v in (forward_vel, return_vel)):
            raise ValueError("两档速度必须是非零有限数")
        if forward_vel * return_vel <= 0.0:
            raise ValueError("两档速度必须同方向")

        # 周期变速和自动扫频都会写 input_vel，两者不能同时运行。
        self.auto_sweep = False
        with self.lock:
            position = self.axis.encoder.pos_estimate
            phase = position % RATIO
            initial_vel = forward_vel if phase < RATIO / 2 else return_vel
            self.axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            self.axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
            self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self.axis.controller.input_vel = initial_vel
            self.cycle_forward_vel = forward_vel
            self.cycle_return_vel = return_vel
            self.target_vel = initial_vel
            self.cycle_velocity = True

        cycle_frequency = 1.0 / (
            RATIO / (2.0 * abs(forward_vel))
            + RATIO / (2.0 * abs(return_vel))
        )
        print(
            f"[周期变速] 0–180°: {forward_vel:.3f} turns/s | "
            f"180–360°: {return_vel:.3f} turns/s | "
            f"理论周期频率: {cycle_frequency:.3f} Hz"
        )

    def stop_cycle_velocity(self):
        """停止周期变速并将速度指令归零。"""
        self.cycle_velocity = False
        with self.lock:
            self.axis.controller.input_vel = 0.0
            self.target_vel = 0.0
        print("[周期变速] 已停止，目标速度已归零")

    def set_position(self, pos):
        """切换到位置控制模式，设置目标位置（turns）"""
        self.auto_sweep = False
        self.cycle_velocity = False
        with self.lock:
            self.axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            self.axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
            self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self.axis.controller.input_pos = float(pos)
            self.target_vel = 0.0
        print(f"[位置] 目标位置: {float(pos):.3f} turns")

    def set_idle(self):
        """释放电机"""
        self.auto_sweep = False
        self.cycle_velocity = False
        with self.lock:
            self.axis.controller.input_vel = 0.0
            self.target_vel = 0.0
            self.axis.requested_state = AXIS_STATE_IDLE
        print("[系统] 电机已释放 (IDLE)")

    def set_closed_loop(self):
        """进入闭环运行"""
        with self.lock:
            self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        print("[系统] 电机已进入闭环运行")

    def show_state(self):
        """打印当前电机状态"""
        with self.lock:
            state = self.axis.current_state
            mode = self.axis.controller.config.control_mode
        pos = self.get_pos()
        vel = self.get_vel()
        mode_str = "速度控制" if mode == CONTROL_MODE_VELOCITY_CONTROL else \
                   "位置控制" if mode == CONTROL_MODE_POSITION_CONTROL else \
                   f"模式{mode}"
        print(f"[状态] 位置: {pos:.4f} turns | 速度: {vel:.4f} turns/s | 模式: {mode_str} | 状态码: {state}")

    def fin_home(self):
        """回中逻辑：以 1 圈为周期，回到最近的整数圈"""
        self.auto_sweep = False
        self.cycle_velocity = False
        try:
            now_pos = self.get_pos()
        except Exception:
            print("[错误] 无法获取位置数据")
            return

        target = round(now_pos / RATIO) * RATIO

        with self.lock:
            self.axis.controller.input_vel = 0.0
            self.target_vel = 0.0
        time.sleep(0.05)

        with self.lock:
            self.axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            self.axis.controller.input_pos = target

        print(f"[系统] 回中指令已执行 | {now_pos:.3f} turns → {target} turns")

    # ── 后台监测线程 ──────────────────────────

    def background_vbus(self):
        """独立线程：低速更新 vbus 缓存，不阻塞 Iq 采集"""
        while self.running:
            try:
                with self.lock:
                    v = self.odrv.vbus_voltage
                with self._vbus_lock:
                    self._vbus = v
            except Exception:
                pass
            time.sleep(0.1)  # 10Hz 更新，够用了

    def background_sweep(self):
        """后台线程：每 30s 切换速度 1→2→3→...→20，配合监测使用"""
        while self.running:
            if self.auto_sweep and self.monitoring:
                for speed in range(1, 11):  # 1, 2, 3, ..., 20
                    if not self.auto_sweep or not self.running:
                        break
                    print(f"\n[Sweep] ========== 切换到速度 {speed}/20 ==========")
                    if not self.set_velocity(speed, require_sweep=True):
                        break
                    # 等待 30 秒，每秒检查一次是否被中断
                    for _ in range(30):
                        if not self.auto_sweep or not self.running:
                            break
                        time.sleep(1)
                # 扫描完成
                if self.auto_sweep:
                    print("\n[Sweep] ========== 速度扫描完成 (1~20) ==========")
                    self.auto_sweep = False
                    self.monitoring = False
                    print("[Sweep] 监测已自动停止")
            else:
                time.sleep(0.5)  # 未启动扫描时低频率检查

    def background_monitor(self):
        """后台线程：高频采集 Iq + 写入 CSV"""
        last_print = 0.0
        sample_count = 0

        while self.running:
            if self.monitoring:
                with self.lock:
                    i_q = self.axis.motor.current_control.Iq_measured
                    position = self.axis.encoder.pos_estimate
                    target_vel = self.target_vel
                relative_time = time.time() - self.start_time

                with self._vbus_lock:
                    v_bus = self._vbus

                try:
                    self.writer.writerow([
                        f"{relative_time:.4f}", i_q, v_bus, target_vel, position
                    ])
                    sample_count += 1

                    # 每 20 次 flush 一次
                    if sample_count % 20 == 0:
                        self.f.flush()

                    # 每 0.5s 打印一次，减少控制台阻塞
                    if relative_time - last_print >= 0.5:
                        last_print = relative_time
                        print(f"[DATA] T:{relative_time:>6.2f}s | I:{i_q:>7.3f}A | P:{abs(i_q * v_bus):>6.2f}W | ~{sample_count / (relative_time + 0.001):.0f} Hz")
                except Exception as e:
                    print("写入失败:", e)
            else:
                time.sleep(0.05)  # 未监测时降低空转频率

    def background_cycle_velocity(self):
        """后台线程：根据编码器相位切换每圈前、后半程速度。"""
        while self.running:
            if not self.cycle_velocity:
                time.sleep(0.02)
                continue

            try:
                with self.lock:
                    # 手动命令可能在本线程等锁时关闭周期控制，
                    # 因此写入 ODrive 前在锁内再确认一次。
                    if self.cycle_velocity:
                        position = self.axis.encoder.pos_estimate
                        phase = position % RATIO
                        new_target = (
                            self.cycle_forward_vel
                            if phase < RATIO / 2
                            else self.cycle_return_vel
                        )
                        if new_target != self.target_vel:
                            self.axis.controller.input_vel = new_target
                            self.target_vel = new_target
            except Exception as e:
                self.cycle_velocity = False
                print(f"[ERROR] 周期变速控制停止: {e}")

            time.sleep(0.005)  # 约 200 Hz 相位检查

    def stop(self):
        """安全退出"""
        self.running = False
        if hasattr(self, 'f'):
            self.f.close()

    # ── 主交互循环 ───────────────────────────

    def start(self):
        threading.Thread(target=self.background_vbus, daemon=True).start()
        threading.Thread(target=self.background_monitor, daemon=True).start()
        threading.Thread(target=self.background_sweep, daemon=True).start()
        threading.Thread(target=self.background_cycle_velocity, daemon=True).start()

        print(HELP_TEXT)
        while True:
            try:
                user_input = input("CMD >> ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\n[系统] 中断，安全退出...")
                self.running = False
                self.set_idle()
                break

            if not user_input:
                continue

            parts = user_input.split(maxsplit=1)
            cmd = parts[0].lower()
            arg = parts[1] if len(parts) > 1 else None

            if cmd == 'exit':
                self.running = False
                self.set_idle()
                break

            elif cmd == 'm':
                self.monitoring = not self.monitoring
                print(f"[系统] 监测已{'开启' if self.monitoring else '关闭'}")

            elif cmd == 'h':
                self.fin_home()

            elif cmd == 'v':
                if arg is None:
                    print("[错误] 用法: v <速度(turns/s)>  例: v 2.0")
                else:
                    try:
                        self.auto_sweep = False
                        self.set_velocity(arg)
                    except ValueError:
                        print(f"[错误] 无效的速度值: {arg}")

            elif cmd == 'cycle':
                if arg is None:
                    print("[错误] 用法: cycle <0–180°速度> <180–360°速度>  例: cycle 3.0 0.6")
                else:
                    try:
                        values = arg.split()
                        if len(values) != 2:
                            raise ValueError("需要两个速度值")
                        self.start_cycle_velocity(*values)
                    except ValueError as e:
                        print(f"[错误] 无效的周期变速参数: {e}")

            elif cmd in ('sc', 'stop_cycle'):
                self.stop_cycle_velocity()

            elif cmd == 'p':
                if arg is None:
                    print("[错误] 用法: p <位置(turns)>  例: p 5.0")
                else:
                    try:
                        self.set_position(arg)
                    except ValueError:
                        print(f"[错误] 无效的位置值: {arg}")

            elif cmd == 'idle':
                self.set_idle()

            elif cmd == 'closed':
                self.set_closed_loop()

            elif cmd == 'sweep':
                # 启动自动速度扫描：1→20，每 30s 切换一次
                self.cycle_velocity = False
                self.monitoring = True
                self.auto_sweep = True
                print("[Sweep] 启动自动扫描！速度 1→20，每 30s 切换，监测已自动开启")

            elif cmd in ('ss', 'stop_sweep'):
                self.auto_sweep = False
                self.monitoring = False
                print("[Sweep] 扫描已手动停止")

            elif cmd == 'state':
                self.show_state()

            elif cmd in ('help', '?'):
                print(HELP_TEXT)

            else:
                print(f"[系统] 未知指令: {user_input}  输入 help 查看帮助")

        self.stop()
        print("程序已退出。")


if __name__ == "__main__":
    ODriveMonitor().start()
