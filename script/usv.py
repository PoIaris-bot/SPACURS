import numpy as np
from math import atan, atan2
from path_planning import rrt

CONFIG_FILE_PATH = '/home/polaris/catkin_ws/src/spacurs/script/config.ini'


def constraint(value, lower, upper):
    if value > upper:
        value = upper
    if value < lower:
        value = lower
    return value


class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.e = 0
        self.e_sum = 0

    def output(self, e):
        de = e - self.e
        self.e_sum += e
        self.e = e
        return self.kp * e + self.ki * self.e_sum + self.kd * de


class USV:
    def __init__(self, x0, x_goal):
        self.x = x0
        self.path = rrt(x0, x_goal, CONFIG_FILE_PATH)
        self.path_point_num = self.path.shape[0]
        self.target_idx = 1
        self.Rth = 1
        self.length = 0.7
        self.Delta = 2 * self.length
        self.speed_controller = PID(1, 0, 1)
        self.steer_controller = PID(1, 0, 1)

    def update(self, x):
        self.x = x
        dist = np.linalg.norm(x[:2] - self.path[self.target_idx, :])
        if dist < self.Rth:
            self.target_idx += 1
            if self.target_idx >= self.path_point_num:
                self.target_idx = self.path_point_num - 1

    def control(self):
        target = self.path[self.target_idx, :]
        target_prev = self.path[self.target_idx - 1, :]
        alpha = np.pi / 2 - atan2(target[1] - target_prev[1], target[0] - target_prev[0])
        error = np.sin(alpha) * (self.x[0] - target[0]) - np.cos(alpha) * (self.x[1] - target[1])
        phi_d = alpha + atan(-error / self.Delta)
        phi = np.pi / 2 - self.x[2]
        phi_e = phi_d - phi
        if phi_e > np.pi:
            phi_e -= 2 * np.pi
        if phi_e < -np.pi:
            phi_e += 2 * np.pi

        dist = np.linalg.norm(self.x[:2] - target)
        base_speed = self.speed_controller.output(dist)
        steer_speed = self.steer_controller.output(phi_e)
        left_speed = str(int(constraint(base_speed + steer_speed, 0, 90)))
        if len(left_speed) == 1:
            left_speed = '0' + left_speed
        right_speed = str(int(constraint(base_speed - steer_speed, 0, 90)))
        if len(right_speed) == 1:
            right_speed = '0' + right_speed

        command = '1' + left_speed + right_speed
        return command

    def data(self):
        x = self.x[:2].tolist()
        target_point = self.path[self.target_idx, :].tolist()
        path = self.path.T.ravel().tolist()
        return x + target_point + [self.path_point_num] + path
