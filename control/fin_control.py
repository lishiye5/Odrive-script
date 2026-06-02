import serial
import time

# --- 配置区 ---
PORT = 'COM10'        # 务必改成你的 COM 号
BAUD = 115200
RATIO = 16           # 减速比


def fin_home(ser):
    """鲁棒性增强的自动化回中逻辑"""
    print("[系统] 开始回中，正在尝试获取位置...")

    # --- 1. 清空残留数据 ---
    # 这一步极其重要！把之前 v 0 10 留在串口里的回复全删掉
    ser.reset_input_buffer()

    now_pos = None
    # --- 2. 循环尝试读取，最多尝试 10 次 ---
    for i in range(10):
        ser.write(b"r axis0.encoder.pos_estimate\n")
        time.sleep(0.05)  # 给 ODrive 处理时间

        if ser.in_waiting:
            response = ser.readline().decode().strip()
            print(f"[调试] 第 {i + 1} 次读取内容: '{response}'")
            try:
                # 尝试转换，如果成功就跳出循环
                now_pos = float(response)
                break
            except ValueError:
                # 如果读到的是文字而不是数字，继续读下一行
                continue
        else:
            print(f"[调试] 第 {i + 1} 次读取：缓冲区无数据")

    if now_pos is None:
        print("回中失败：多次尝试后仍无法读取有效位置数据")
        return

    # --- 3. 计算并分步发送指令 ---
    try:
        target_pos = round(now_pos / RATIO) * RATIO

        # 不要打包发送，分步发送更安全，模拟手动输入
        # a. 停止速度
        
        ser.write(b"w axis0.controller.input_vel 0\n")
        time.sleep(0.02)
        # b. 切换模式
        ser.write(b"w axis0.controller.config.control_mode 1\n")
        time.sleep(0.02)
        # c. 移动到目标
        cmd = f"p 0 {target_pos} 0 0\n"
        ser.write(cmd.encode())

        print(f"回中成功！当前:{now_pos:.2f} -> 目标:{target_pos}")
    except Exception as e:
        print(f"执行回中动作时出错: {e}")

def start_interactive():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
        print(f"已连接到 {PORT}。输入指令后按回车发送。")
        print("输入 'h' 执行自动回中 | 输入 'exit' 退出程序")
        print("-" * 40)

        while True:
            # 获取键盘输入
            user_input = input("ODrive >> ").strip()

            if user_input.lower() == 'exit':
                break
            elif user_input.lower() == 'h':
                fin_home(ser)
            elif user_input:
                # 像串口助手一样原样发送手动指令
                # 自动帮你补齐 \n，你只需要输 v 0 10 即可
                full_cmd = user_input + "\n"
                ser.write(full_cmd.encode())
                
                # 等待并打印 ODrive 的回复（如果有的话）
                time.sleep(0.05)
                while ser.in_waiting:
                    print("Reply:", ser.readline().decode().strip())

        ser.close()
    except Exception as e:
        print(f"串口错误: {e}")


def monitor_live_power(port='COM3'):
    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
        print(f"{'时间(s)':<8} {'电流(A)':<10} {'电压(V)':<10} {'实时功率(W)':<10}")
        print("-" * 45)

        start_time = time.time()
        while True:
            # 1. 读电流
            ser.write(b"r axis0.motor.current_control.Iq_measured\n")
            i_q = float(ser.readline().decode().strip() or 0)

            # 2. 读电压
            ser.write(b"r vbus_voltage\n")
            v_bus = float(ser.readline().decode().strip() or 0)

            # 3. 计算
            now = time.time() - start_time
            power = abs(i_q * v_bus)  # 取绝对值代表消耗的总功率

            # 4. 打印结果
            print(f"{now:<8.2f} {i_q:<10.4f} {v_bus:<10.2f} {power:<10.2f}")

            time.sleep(0.05)  # 20Hz 的刷新频率，足够观察摆动特性

    except KeyboardInterrupt:
        print("\n停止监控")
        ser.close()

if __name__ == "__main__":
    start_interactive()