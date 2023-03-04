import numpy as np


def remap(theta):
    return np.arctan2(np.sin(theta), np.cos(theta))


def constraint(value, lower, upper):
    if value > upper:
        value = upper
    if value < lower:
        value = lower
    return value


def generate_path(x, y, theta, x_goal, y_goal, r1=3, r2=2):
    theta_goal = remap(np.arctan2(y_goal - y, x_goal - x))
    flag = remap(theta - theta_goal) < 0

    if flag:
        x_o = x + r1 * np.cos(theta + np.pi / 2)
        y_o = y + r1 * np.sin(theta + np.pi / 2)
    else:
        x_o = x + r1 * np.cos(theta - np.pi / 2)
        y_o = y + r1 * np.sin(theta - np.pi / 2)

    beta = remap(np.arctan2(y_goal - y_o, x_goal - x_o))
    alpha = np.arccos(r1 / np.sqrt((y_goal - y_o) ** 2 + (x_goal - x_o) ** 2))
    gamma1 = remap(np.arctan2(y - y_o, x - x_o)) + 2 * np.pi
    if flag:
        gamma2 = remap(beta - alpha) + 2 * np.pi
    else:
        gamma2 = remap(beta + alpha) + 2 * np.pi

    x_c = x_o + r1 * np.cos(gamma2)
    y_c = y_o + r1 * np.sin(gamma2)

    if flag:
        if gamma2 < gamma1:
            gamma2 += 2 * np.pi
    else:
        if gamma2 > gamma1:
            gamma1 += 2 * np.pi

    gammas = np.arange(gamma1, gamma2, np.pi / 10 if flag else -np.pi / 10)
    arc = np.array([[x_o, y_o]]).T + r1 * np.array([np.cos(gammas), np.sin(gammas)])
    num_arc_point = arc.shape[1] + 1

    theta_line = remap(np.arctan2(y_goal - y_c, x_goal - x_c))
    line = np.array([[x_c + 0.1 * np.cos(theta_line), x_goal], [y_c + 0.1 * np.sin(theta_line), y_goal]])

    thetas = np.arange(theta_line + np.pi / 6, theta_line + np.pi / 6 + 2 * np.pi, np.pi / 10)
    circle = np.array([[x_goal, y_goal]]).T + r2 * np.array([np.cos(thetas), np.sin(thetas)])
    num_circle_point = circle.shape[1]

    path = np.hstack([arc, line, circle])
    return path, (x_o, y_o), (num_arc_point, num_circle_point)


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.e = 0
        self.e_sum = 0

    def output(self, e):
        de = e - self.e
        self.e_sum = constraint(self.e_sum + e, -1, 1)
        self.e = e
        return self.kp * e + self.ki * self.e_sum + self.kd * de
