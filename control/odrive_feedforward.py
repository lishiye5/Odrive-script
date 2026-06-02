#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import odrive
from odrive.enums import *
import time

def main():
    print("正在寻找 ODrive 设备...")
    # 自动寻找并连接到电脑上的第一个 ODrive
    odrv0 = odrive.find_any()
    print("ODrive 已连接！")

    # 定义我们要控制的轴，这里默认是 axis0
    axis = odrv0.axis0

    print("开始配置基本参数...")
    # 1. 设置为位置控制模式
    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    # 2. 设置为透传（直接输入）模式
    axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    # 3. 提高速度限制，防止前馈速度触发过速保护（这里设为 10 转/秒）
    axis.controller.config.vel_limit = 10.0

    print("使能电机进入闭环运行状态...")
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    
    # 等待电机状态真正切换成功
    time.sleep(0.5)
    if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
        print("错误：电机未能成功进入闭环控制，请检查硬件供电或是否有报错(dump_errors)！")
        return

    # 获取当前电机的初始估计位置
    start_pos = axis.encoder.pos_estimate
    NUM_TURNS = 5  # 要转多少圈
    target_end = start_pos + NUM_TURNS
    current_pos = start_pos

    dt = 0.01  # 循环刷新周期：10ms (100Hz)

    # 速度配置（单位：转/秒，turns/s）
    V_FAST = 5.0  # 每圈前半圈速度
    V_SLOW = 1.0  # 每圈后半圈速度

    print(f"电机当前位置: {start_pos:.3f} 圈，准备开始运动...")
    print(f"共转 {NUM_TURNS} 圈，每圈: 前半圈飞速（{V_FAST} turns/s），后半圈龟速（{V_SLOW} turns/s）")

    try:
        while current_pos < target_end:
            # 计算当前运动的相对进度（走了多少圈）
            progress = current_pos - start_pos
            # 当前圈内的小数部分，每圈都有快慢阶段
            in_turn = progress % 1.0

            # 根据当前圈内的进度动态选择速度前馈
            if in_turn < 0.5:
                vel_ff = V_FAST
            else:
                vel_ff = V_SLOW

            # 位置是速度对时间的积分
            current_pos += vel_ff * dt

            # 截断保护，防止最后一刻过冲
            if current_pos > target_end:
                current_pos = target_end
                vel_ff = 0.0

            # 写入 ODrive 对应的底层寄存器
            axis.controller.input_pos = current_pos
            # 兼容老版本固件(v3.6)，使用 input_vel 充当速度前馈
            axis.controller.input_vel = vel_ff

            # 维持高频循环的时间间隔
            time.sleep(dt)

        print(f"运动完成！当前电机最终位置: {axis.encoder.pos_estimate:.3f} 圈")

    except KeyboardInterrupt:
        # 允许用户中途按 Ctrl+C 安全退出
        print("\n用户中断，停止发送指令。")
        axis.controller.input_vel = 0.0

if __name__ == "__main__":
    main()