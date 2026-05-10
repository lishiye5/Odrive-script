import serial
import time
import threading , os , csv

# 在类初始化或脚本开头定义
log_folder = "data/raw"
if not os.path.exists(log_folder):
    os.makedirs(log_folder)

# 生成唯一的文件名，防止覆盖之前的实验数据
filename = f"{log_folder}/experiment_{time.strftime('%m%d_%H%M%S')}.csv"

# 写入表头（只在文件创建时执行一次）
with open(filename, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["time", "iq", "vbus"])

# --- 配置区 ---
PORT = 'COM10'
BAUD = 115200
RATIO = 16


class ODriveMonitor:
    def __init__(self):
        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=0.1)
            self.running = True
            self.monitoring = False
            self.lock = threading.Lock()  # 增加线程锁，防止读写冲突
            print(f"已连接到 {PORT}")
            self.start_time = time.time()
            # 在初始化时就打开文件，这样 background_monitor 就不需要反复打开关闭
            self.f = open(filename, 'a', newline='')
            self.writer = csv.writer(self.f)
        except Exception as e:
            print(f"连接失败: {e}");
            exit()

    def send_and_receive(self, cmd, wait_reply=True):
        """带锁的串口通信，确保指令不冲突"""
        with self.lock:
            self.ser.write(f"{cmd}\n".encode())
            if wait_reply:
                return self.ser.readline().decode().strip()
            return None

    def background_monitor(self):
        """后台线程：持续打印电流，不再原地刷新"""
        while self.running:
            if self.monitoring:
                # 获取数据
                i_raw = self.send_and_receive("r axis0.motor.current_control.Iq_measured")
                v_raw = self.send_and_receive("r vbus_voltage")
                # 计算相对时间（标定时横坐标更清晰）
                relative_time = time.time() - self.start_time

                try:
                    i_q = float(i_raw)
                    v_bus = float(v_raw)
                    # 写入 CSV
                    self.writer.writerow([f"{relative_time:.4f}", i_q, v_bus])
                    # 定期刷入磁盘，防止崩溃丢失数据
                    if int(relative_time * 10) % 10 == 0:
                        self.f.flush()

                    print(f"[DATA] T:{relative_time:>6.2f}s | I:{i_q:>7.3f}A | P:{abs(i_q * v_bus):>6.2f}W")
                except Exception as e:
                    print("解析失败:", repr(i_raw), repr(v_raw), e)
            time.sleep(0.01)  # 稍微降低频率，防止指令堆积

    def stop(self):
        """退出时安全关闭文件"""
        self.running = False
        if hasattr(self, 'f'):
            self.f.close()
        self.ser.close()


    def fin_home(self):
        """更安全的回中逻辑"""
        # print("[系统] 准备回中，先停止电机...")
        # self.send_and_receive("v 0 0")  # 先停下
        # time.sleep(0.05)
        pos_raw = self.send_and_receive("r axis0.encoder.pos_estimate")
        time.sleep(0.05)
        try:
            now_pos = float(pos_raw)
            # 计算目标：最近的16的倍数
            target = round(now_pos / RATIO) * RATIO

            # 关键：先切模式，再设位置，防止疯转
            self.send_and_receive("v 0 0")  # 先停下
            self.send_and_receive("w axis0.controller.config.control_mode 1")
            self.send_and_receive(f"p 0 {target} 0 0")
            print(f"[系统] 回中指令已执行，目标位置: {target}")
        except:
            print("[错误] 无法获取位置数据")

    def start(self):
        threading.Thread(target=self.background_monitor, daemon=True).start()

        print("\n--- 指令: 'm':开关监测 | 'h':回中 | 'exit':退出 ---")
        while True:
            user_input = input("CMD >> ").strip().lower()
            if user_input == 'exit':
                self.running = False
                break
            elif user_input == 'm':
                self.monitoring = not self.monitoring
                print(f"[系统] 监测已{'开启' if self.monitoring else '关闭'}")
            elif user_input == 'h':
                self.fin_home()
            elif user_input:
                self.send_and_receive(user_input, wait_reply=False)

        self.ser.close()


if __name__ == "__main__":
    ODriveMonitor().start()
