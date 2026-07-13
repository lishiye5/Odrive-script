#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ODrive 云台模式极简速度/位置控制与实时力矩计算脚本
"""

import odrive
from odrive.enums import *
import time

def main():
    print("正在寻找 ODrive 设备...")
    odrv0 = odrive.find_any()
    print("ODrive 已连接！")

    axis = odrv0.axis0

    # 自动获取当前硬件测得的相电阻与力矩常数
    R = axis.motor.config.phase_resistance if axis.motor.config.phase_resistance > 0 else 1.9326
    Kt = axis.motor.config.torque_constant if axis.motor.config.torque_constant > 0 else 0.03308
    print(f"载入硬件参数 -> 电阻 R: {R:.4f} Ω | 力矩常数 Kt: {Kt:.5f} N.m/A")

    # =================【模式选择开关】=================
    # 想测速度模式就把这设为 True；想测位置模式就设为 False
    TEST_VELOCITY_MODE = True
    # =================================================

    if TEST_VELOCITY_MODE:
        print("配置为：速度控制模式...")
        axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        axis.controller.input_vel = 1.0  # 设定目标速度：-1 转/秒
    else:
        print("配置为：位置控制模式...")
        axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        axis.controller.input_pos = axis.encoder.pos_estimate  # 锁在当前位置

    print("使能电机进入闭环运行...")
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.5)

    print("\n开始实时监控。堵转电机，观察力矩！(按 Ctrl+C 退出)")
    print("-" * 80)

    try:
        while True:
            # 1. 计算当前的瞬态速度误差
            vel_error = axis.controller.input_vel - axis.encoder.vel_estimate

            # 2. 还原比例项(P)和读取积分项(I)的控制电压总量
            v_p = vel_error * axis.controller.config.vel_gain
            v_i = axis.controller.vel_integrator_torque
            v_q_total = abs(v_p + v_i)

            # 3. 欧姆定律反推实时物理力矩 (设置 0.015V 滤除静态噪声)
            if v_q_total > 0.015:
                real_torque = (v_q_total / R) * Kt
            else:
                real_torque = 0.0

            # 4. 获取当前速度和位置反馈
            actual_vel = axis.encoder.vel_estimate
            actual_pos = axis.encoder.pos_estimate

            # 5. 原地刷新打印数据
            print(
                f"\r实际位置: {actual_pos:6.2f} 圈 | 实际速度: {actual_vel:6.2f} turns/s | 等效总电压: {v_q_total:5.2f} V | 💥实时力矩: {real_torque:.5f} N.m",
                end="", flush=True
            )

            time.sleep(0.1)  # 100ms 刷新周期 (10Hz)

    except KeyboardInterrupt:
        print("\n\n用户中断，安全释放电机...")
        axis.controller.input_vel = 0.0
        axis.requested_state = AXIS_STATE_IDLE
        print("⏹️ 电机已释放 (IDLE)。")

if __name__ == "__main__":
    main()