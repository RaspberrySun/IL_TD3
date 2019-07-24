# -*- coding: UTF-8 -*-
import time
import numpy as np
from pymavlink import mavutil


class PID:
    # 初始化
    def __init__(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.output = 0.0

    # 设置目标值
    def set_target(self, target):
        self.SetPoint = target

    """
    计算PID值:
    u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
    """
    def calcu(self, feedback_value):
        error = self.SetPoint - feedback_value
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error
        if delta_time >= self.sample_time:
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time
            if delta_time > 0:
                self.DTerm = delta_error / delta_time + 1e-4
            # 保存上一次误差和时间为下一次运算
            self.last_time = self.current_time
            self.last_error = error
            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

    # 设置Kp值
    def set_kp(self, p_gain):
        self.Kp = p_gain

    # 设置Ki值
    def set_ki(self, i_gain):
        self.Ki = i_gain

    # 设置Kd值
    def set_kd(self, d_gain):
        self.Kd = d_gain

    # 设间隔时间
    def set_time(self, sample_time):
        self.sample_time = sample_time


master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
master.wait_heartbeat()


# Arm 解锁BlueRov2
def arm():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)


# Disarm 锁定BlueRov2
def disarm():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)


def depth_data():
    """
    读取深度信息
    :return: depth
    """
    dep = 0
    while True:
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'VFR_HUD':
            dep = -msg.alt
            break
    return dep


def angle_data():
    """
    读取角度信息
    :return: pitch_angle, roll_angle
    """
    theta, phi = 0, 0
    while True:
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'ATTITUDE':
            theta = msg.pitch
            phi = msg.roll
            break
    return theta, phi


def set_rc_channel_pwm(id, pwm=1500):
    """ 设置 RC 通道 pwm 值
    参数:
        id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if id < 1:
        print("Channel does not exist.")
        return
    if id < 9:
        rc_channel_values = [65535 for _ in range(8)]
        rc_channel_values[id - 1] = pwm
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            *rc_channel_values)


def read_pwm():
    pwm1, pwm2, pwm3, pwm4 = 1500, 1500, 1500, 1500
    while True:
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'SERVO_OUTPUT_RAW':
            pwm1 = msg.servo5_raw
            pwm2 = msg.servo6_raw
            pwm3 = msg.servo7_raw
            pwm4 = msg.servo8_raw
            break
    return pwm1, pwm2, pwm3, pwm4


def change_mode(mode='MANUAL'):
    """
    改变工作模式
    :param mode: 'MANUAL' or 'STABILIZE' or 'ALT_HOLD'
    """
    if mode not in master.mode_mapping():
        print("Unknown mode : {}".format(mode))
        print('Try: ', list(master.mode_mapping().keys()))
        exit(1)
    mode_id = master.mode_mapping()[mode]
    master.set_mode(mode_id)


# 设置每个推进器的pwm
def set_motor_pwm(channel, pwm):
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
                                 0, channel, 4, pwm, 100, 1, 0, 0)


def force_distribute(out1, out2, out3):
    """
       推力分配计算
       :param out1: Z force
       :param out2: M force
       :param out3: K force
       :return: 分配的4个推进器的推力,4x1矩阵
    """
    arr1 = np.mat([[1, 1, 1, 1],  [-0.12, -0.12, 0.12, 0.12], [0.218, -0.218, 0.218, -0.218]])
    arr2 = np.mat([[out1], [out2], [out3]])
    return arr1.I * arr2


def save_data(filename, depth_data, roll_data, pitch_data, pwm):
    with open(filename, 'a') as f:
        f.writelines(str(depth_data)[:5] + '  ' + str(roll_data)[:7] + '  ' + str(pitch_data)[:7] + '      ' + str(pwm[0])+'\n')


def pid_for_init(d_exp, p_exp, r_exp):
    depth_hold = PID(50, 0.5, 0.01)
    pitch_hold = PID(8, 0.5, 0.01)
    roll_hold = PID(8, 0.5, 0.01)
    change_mode()
    arm()
    while True:
        depth = depth_data()
        p_angle, r_angle = angle_data()
        depth_hold.set_target(d_exp)
        pitch_hold.set_target(p_exp)
        roll_hold.set_target(r_exp)
        # depth_hold.set_time(0.1)
        # pitch_hold.set_time(0.1)
        # roll_hold.set_time(0.1)
        depth_hold.calcu(depth)
        pitch_hold.calcu(p_angle)
        roll_hold.calcu(r_angle)
        thruster = []
        f = force_distribute(depth_hold.output, pitch_hold.output, roll_hold.output)
        for i in range(4):
            thruster.append(f[i, 0])
        thruster = np.clip(thruster, -34.3, 34.3)
        pwm = [t / 34.3 * 400 + 1500 for t in thruster]
        set_motor_pwm(4, pwm[0])
        set_motor_pwm(5, pwm[1])
        set_motor_pwm(6, pwm[2])
        set_motor_pwm(7, pwm[3])
        f = [read_pwm()]
        # print(depth, r_angle, p_angle, f, '\n')
        if abs(depth-d_exp) <= 0.1 and abs(p_angle-p_exp) <= 0.1 and abs(r_angle-r_exp) <= 0.1:
            change_mode("ALT_HODE")
        save_data(file_name, depth, r_angle, p_angle, f)


if __name__ == "__main__":
    while True:
        try:
            arm()

            change_mode("STABILIZE")
            set_rc_channel_pwm(3, 1400)
            #set_rc_channel_pwm(4, 1400)
            #set_rc_channel_pwm(5, 1600)
        except KeyboardInterrupt:
            break







