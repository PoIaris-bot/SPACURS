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

    xo = x + r1 * np.cos(theta + np.pi / (2 if flag else -2))
    yo = y + r1 * np.sin(theta + np.pi / (2 if flag else -2))

    beta = remap(np.arctan2(y_goal - yo, x_goal - xo))
    alpha = np.arccos((r1 - r2) / np.sqrt((y_goal - yo) ** 2 + (x_goal - xo) ** 2))

    gamma1 = remap(np.arctan2(y - yo, x - xo)) + 2 * np.pi
    gamma2 = remap(beta + (-alpha if flag else alpha)) + 2 * np.pi

    x1 = xo + r1 * np.cos(gamma2)
    y1 = yo + r1 * np.sin(gamma2)
    x2 = x_goal + r2 * np.cos(gamma2)
    y2 = y_goal + r2 * np.sin(gamma2)

    if flag and gamma2 < gamma1:
        gamma2 += 2 * np.pi
    if not flag and gamma2 > gamma1:
        gamma1 += 2 * np.pi

    gammas = np.linspace(gamma1, gamma2, round(abs(gamma1 - gamma2) * r1), endpoint=False)
    arc = np.array([[xo, yo]]).T + r1 * np.array([np.cos(gammas), np.sin(gammas)])

    line = np.linspace([x1, y1], [x2, y2], round(np.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)), endpoint=False).T

    thetas = np.linspace(gamma2, gamma2 + np.pi * (2 if flag else -2), round(2 * np.pi * r2), endpoint=False)
    circle = np.array([[x_goal, y_goal]]).T + r2 * np.array([np.cos(thetas), np.sin(thetas)])

    path = np.hstack([arc, line, circle])
    circle_idx = arc.shape[1] + line.shape[1]
    return path, circle_idx


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
