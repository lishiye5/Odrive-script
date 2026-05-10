"""
这是Deng FOC的快速配置程序
可以自行按照
Deng FOC主板参数/电机参数/编码器参数/控制参数
进行修改配置
main中的每个函数都有详细的说明方便修改
PS:如果用conda环境，注意修改到安装了odrive的环境内运行
"""

import time
from odrive.enums import *
from odrive.utils import *
import odrive

# limit 放在一起

# Deng FOC参数
board_parameter = {"brake_res": 0,  # 耗散电阻值(根据实际接入的功率电阻值输入，不接为0)
                   "dc_max_pos_cur": 30,  # 电源的过流保护的电流值
                   "dc_max_neg_cur": -3.0,  # 反向电流的过流保护阈值
                   "under_volt_level": 8.0,  # 欠压保护阈值
                   "over_volt_level": 30.0,  # 过压保护阈值
                   "max_regen_current": 0  # 制动回流电流值
                   }
# 电机相关参数
motor_parameter = {"pole_pairs": 7,  # 电机的极对
                   "motor_type": MOTOR_TYPE_HIGH_CURRENT,  # 电机类型
                   "cur_lim": 12,  # 电机电流限制（A）
                   "cal_cur": 5,  # 电机校准电流限制（A）
                   "cal_vol": 1,  # 电机校准电压限制（v)
                   "requested_cur_range": 25,  # 电流采样范围（A）
                   "kv": 1800  # 电机kv值
                   }
# 编码器参数 for AS5047P
encoder_parameter = {"mode": ["5047_ABI", "5047_SPI"],
                     "encoder_mode": [ENCODER_MODE_INCREMENTAL, ENCODER_MODE_SPI_ABS_AMS],  # 编码器模式
                     "cpr": [4000, 2 ** 14],  # 编码器cpr
                     "bandwidth": 3000,  # 带宽
                     "encoder_cs_pin": 8,  # SPI CS引脚
                     "cali_cur": 5,  # 配置编码器时校准电机的电流
                     "cali_ramp_dis": 3.1415927410125732,  # 配置编码器时电机转动距离
                     "cali_ramp_time": 0.4,  # 配置编码器时电流提高的时间
                     "cali_accel": 20,  # 配置编码器时电机的加速度
                     "cali_vel": 40,  # 配置编码器时电机的速度
                     }
# 控制器基本参数
controller_base_parameter = {"vel_lim": 160,  # 速度限制（[turn/s）
                             "vel_gain": 0.003,  # 速度环增益
                             "pos_gain": 8,  # 位置环增益
                             "vel_integrator": 0.02,  # 速度环积分
                             }
# 控制模式
contorl_mode = ["vel_passthrough", "vel_ramp", "pos_passthrough", "pos_trap_traj", "torque_passthrough", "torque_ramp"]
# 速度模式
velocity_mode_parameter = {"control_mode": CONTROL_MODE_VELOCITY_CONTROL,  # 设置速度控制
                           "input_mode": [INPUT_MODE_PASSTHROUGH, INPUT_MODE_VEL_RAMP],  # 设置输入模式
                           "vel_ramp_rate": 5,  # 设置速度爬升速率
                           "inertia": 0.1,  # 设置惯性
                           }
# 位置模式
position_mode_parameter = {"control_mode": CONTROL_MODE_POSITION_CONTROL,  # 设置位置控制
                           "input_mode": [INPUT_MODE_PASSTHROUGH, INPUT_MODE_TRAP_TRAJ],  # 设置输入模式
                           "trap_traj_vel_lim": 30,  # 梯形轨迹速度限制
                           "trap_traj_accel_limit": 5,  # 梯形轨迹加速度限制
                           "trap_traj_decel_limit": 5,  # 梯形轨迹减速度限制
                           }
# 力矩模式
torque_mode_parameter = {"control_mode": CONTROL_MODE_TORQUE_CONTROL,  # 设置力矩控制
                         "input_mode": [INPUT_MODE_PASSTHROUGH, INPUT_MODE_TORQUE_RAMP],  # 设置输入模式
                         "tor_ramp_rate": 0,  # 设置力矩爬升模式
                         }


def set_board_param(odrv, board_param):
    # print("Setting board parameters")

    odrv.config.brake_resistance = board_param["brake_res"]
    odrv.config.dc_bus_undervoltage_trip_level = board_param["under_volt_level"]
    odrv.config.dc_bus_overvoltage_trip_level = board_param["over_volt_level"]
    odrv.config.dc_max_positive_current = board_param["dc_max_pos_cur"]
    odrv.config.dc_max_negative_current = board_param["dc_max_neg_cur"]
    odrv.config.max_regen_current = board_param["max_regen_current"]

    # odrv.save_configuration()

def set_motor_param(odrv, motor_param):
    print("正在设置电机参数")

    odrv.axis0.motor.config.pole_pairs = motor_param["pole_pairs"]
    odrv.axis0.motor.config.motor_type = motor_param["motor_type"]
    odrv.axis0.motor.config.current_lim = motor_param["cur_lim"]
    odrv.axis0.motor.config.calibration_current = motor_param["cal_cur"]
    odrv.axis0.motor.config.resistance_calib_max_voltage = motor_param["cal_vol"]
    odrv.axis0.motor.config.requested_current_range = motor_param["requested_cur_range"]
    odrv.axis0.motor.config.torque_constant = 8.27/motor_param["kv"]

    # odrv.save_configuration()


def set_encoder_param(odrv, encoder_param):
    encoder_mode = int(input("选择编码器模式：\n"
                         "1.AS5047P-ABI(每次上电必须来回转以校准编码器)\n"
                         "2.AS5047P-SPI\n"))


    if encoder_mode == 1:
        print("正在设置编码器为ABI模式")
        odrv.axis0.encoder.config.mode = encoder_param["encoder_mode"][0]
        odrv.axis0.encoder.config.cpr = encoder_param["cpr"][0]
    elif encoder_mode == 2:
        print("正在设置编码器为SPI模式")
        odrv.axis0.encoder.config.mode = encoder_param["encoder_mode"][1]
        odrv.axis0.encoder.config.cpr = encoder_param["cpr"][1]
        odrv.axis0.encoder.config.abs_spi_cs_gpio_pin = encoder_param["encoder_cs_pin"]

    odrv.axis0.encoder.config.bandwidth = encoder_param["bandwidth"]
    odrv.axis0.config.calibration_lockin.current = encoder_param["cali_cur"]
    odrv.axis0.config.calibration_lockin.ramp_distance = encoder_param["cali_ramp_dis"]
    odrv.axis0.config.calibration_lockin.ramp_time = encoder_param["cali_ramp_time"]
    odrv.axis0.config.calibration_lockin.accel = encoder_param["cali_accel"]
    odrv.axis0.config.calibration_lockin.vel = encoder_param["cali_vel"]

    # odrv.save_configuration()


def set_controller_param(odrv, controller_base_param):
    print("Setting controller base param")

    odrv.axis0.controller.config.vel_limit = controller_base_param["vel_lim"]
    odrv.axis0.controller.config.vel_gain = controller_base_param["vel_gain"]
    odrv.axis0.controller.config.pos_gain = controller_base_param["pos_gain"]
    odrv.axis0.controller.config.vel_integrator_gain = controller_base_param["vel_integrator"]

    # odrv.save_configuration()


def set_velocity_mode(odrv, velocity_mode_param, mode):
    print("设置电机为速度模式")

    odrv.axis0.controller.config.control_mode = velocity_mode_param["control_mode"]
    if mode == "passthrough":
        odrv.axis0.controller.config.input_mode = velocity_mode_param["input_mode"][0]
    elif mode == "velocity_ramp":
        odrv.axis0.controller.config.input_mode = velocity_mode_param["input_mode"][1]
        odrv.axis0.controller.config.vel_ramp_rate = velocity_mode_param["vel_ramp_rate"]
        odrv.axis0.controller.config.inertia = velocity_mode_param["inertia"]

    # odrv.save_configuration()


def set_torque_mode(odrv, torque_mode_param, mode):
    print("设置电机为扭矩模式")

    odrv.axis0.controller.config.control_mode = torque_mode_param["control_mode"]
    if mode == "passthrough":
        odrv.axis0.controller.config.input_mode = torque_mode_param["input_mode"][0]
    elif mode == "torque_ramp":
        odrv.axis0.controller.config.input_mode = torque_mode_param["input_mode"][1]
        odrv.axis0.controller.config.torque_ramp_rate = torque_mode_param["tor_ramp_rate"]

    # odrv.save_configuration()


def set_position_mode(odrv, position_mode_param, mode):
    print("设置电机为位置模式")

    odrv.axis0.controller.config.control_mode = position_mode_param["control_mode"]
    if mode == "passthrough":
        odrv.axis0.controller.config.input_mode = position_mode_param["input_mode"][0]
    elif mode == "position_traj_ramp":
        odrv.axis0.controller.config.input_mode = position_mode_param["input_mode"][1]
        odrv.axis0.trap_traj.config.vel_limit = position_mode_param["trap_traj_vel_lim"]
        odrv.axis0.trap_traj.config.accel_limit = position_mode_param["trap_traj_accel_limit"]
        odrv.axis0.trap_traj.config.decel_limit = position_mode_param["trap_traj_decel_limit"]

    # odrv.save_configuration()


def print_configs(odrv):
    print("MOTOR config")
    print(odrv.axis0.motor.config)
    print("---------------------------------------------------------------------")
    print("ENCODER config")
    print(odrv.axis0.encoder.config)
    print("---------------------------------------------------------------------")
    print("CONTROLLER config")
    print(odrv.axis0.controller.config)

    # odrv.sava_configuration()

def disable_speed_err(odrv):
    odrv.axis0.controller.config.enable_overspeed_error = False

def set_control_mode(odrv):

    set_controller_param(odrv0, controller_base_parameter)  # 设置控制器基本参数
    mode = int(input("需要设置该电机为：\n"
                     "0.速度直接模式\n"
                     "1.速度爬升模式\n"
                     "2.位置直接模式\n"
                     "3.位置梯形轨迹模式\n"
                     "4.力矩直接模式\n"
                     "5.力矩爬升模式\n"))

    if mode == 0:
        set_velocity_mode(odrv0, velocity_mode_parameter, "passthrough")  # 速度直接模式
    elif mode == 1:
        set_velocity_mode(odrv0, velocity_mode_parameter, "velocity_ramp")  # 速度爬升模式
    elif mode == 2:
        set_position_mode(odrv0, position_mode_parameter, "passthrough")  # 位置直接模式
    elif mode == 3:
        set_position_mode(odrv0, position_mode_parameter, "position_traj_ramp")  # 位置梯形轨迹模式
    elif mode == 4:
        set_torque_mode(odrv0, torque_mode_parameter, "passthrough")  # 力矩直接模式
    elif mode == 5:
        set_torque_mode(odrv0, torque_mode_parameter, "torque_ramp")  # 力矩爬升模式

def calibrate_motor(odrv):
    print('正在校准电机……')
    odrv.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    while odrv.axis0.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)
    if odrv.axis0.error != AXIS_ERROR_NONE:
        dump_errors(odrv)
        exit()
    print("校准电机成功")
    if odrv.axis0.encoder.config.mode == 0:  # ABI mode
        odrv.axis0.motor.config.pre_calibrated = True  # ABI 预校准电机
    elif odrv.axis0.encoder.config.mode == 257:  # SPI mode
        odrv.axis0.config.startup_motor_calibration = True  # SPI 开机校准电机 TODO SPI下是不是也可以预校准电机？
    time.sleep(1)




def calibrate_encoder(odrv):
    print('正在校准编码器……')
    odrv.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    while odrv.axis0.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)
    if odrv.axis0.error != AXIS_ERROR_NONE:
        dump_errors(odrv)
        exit()
    if not odrv.axis0.encoder.is_ready:
        print("Failed to calibrate encoder! Quitting")
        exit()
    print("校准编码器成功")
    if odrv.axis0.encoder.config.mode == 0:  # ABI mode
        odrv.axis0.config.startup_encoder_offset_calibration = True  # ABI 必须开机校准编码器
    elif odrv.axis0.encoder.config.mode == 257:  # SPI mode
        odrv.axis0.encoder.config.pre_calibrated = True  # SPI
    time.sleep(1)


def close_loop(odrv):
    print('正在测试闭环……')
    odrv.axis0.controller.input_vel = 0
    odrv.axis0.controller.input_pos = odrv.axis0.encoder.pos_estimate
    odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    cls_sucs = input("检查是否进入闭环模式？ (Y/N)")
    if cls_sucs.upper() == "Y":
        odrv.axis0.config.startup_closed_loop_control = True
        # odrv.save_configuration()
    else:
        pass




# def wait_and_exit_on_error(ax):
#     while ax.current_state != AXIS_STATE_IDLE:
#         time.sleep(0.1)
#     if ax.error != AXIS_ERROR_NONE:
#         dump_errors(odrv, True)
#         exit()

def USBerror():
    print("Device disconnected")


if __name__ == '__main__':
    # 连接Deng FOC 注意退出odrivetool，否则无法连接
    odrv0 = odrive.find_any()
    odrv0.__channel__._channel_broken.subscribe(USBerror)
    # print_configs(odrv0)


    set_flag = input("是否需要设置参数？ (Y/N)")
    if set_flag.upper() == "Y":
        # 设置Deng FOC基本参数
        set_board_param(odrv0, board_parameter)
        # 设置电机参数
        set_motor_param(odrv0, motor_parameter)
        # 设置编码器参数 
        set_encoder_param(odrv0, encoder_parameter)

        # disable_speed_err(odrv0)


        set_control_mode(odrv0)


    cali_motor_flag = input("是否需要校准电机？ (Y/N)")
    if cali_motor_flag.upper() == "Y":
        calibrate_motor(odrv0)
    cali_encoder_flag = input("是否需要校准编码器？ (Y/N)")
    if cali_encoder_flag.upper() == "Y":
        calibrate_encoder(odrv0)
    closeloop_flag = input("是否需要上电进入闭环模式？ (Y/N)")
    if closeloop_flag.upper() == "Y":
        close_loop(odrv0)
    save_flag = input("是否保存设置？ (Y/N)")
    if save_flag.upper() == "Y":
        odrv0.save_configuration()
    reboot_flag = input("是否需要重启？ (Y/N)")
    if reboot_flag.upper() == "Y":
        odrv0.reboot()

    print("Done!")

