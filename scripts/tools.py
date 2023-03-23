import rospy
import numpy as np


def remap_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def constraint(value, lower, upper):
    if value > upper:
        value = upper
    if value < lower:
        value = lower
    return value


class KalmanFilter:
    def __init__(self, x, a):
        self.x = x
        self.n = x.shape[0]
        self.P = np.eye(self.n)

        self.a = a
        self.A = None
        self.C = np.eye(self.n)

        self.q = 1e-2 * np.eye(self.n)
        self.Q = None
        self.R = 1e-2 * np.eye(self.n)

        self.timestamp = rospy.get_time()

    def predict(self):
        timestamp = rospy.get_time()
        dt = timestamp - self.timestamp
        self.A = self.a * dt + np.eye(self.n)
        self.Q = self.q * dt

        self.x = np.dot(self.A, self.x)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

        self.timestamp = timestamp

    def update(self, y):
        innovation = y - np.dot(self.C, self.x)
        lambda_t = np.dot(np.dot(self.C, self.P), self.C.T) + self.R
        kalman_gain = np.dot(np.dot(self.P, self.C.T), np.linalg.inv(lambda_t))
        self.x = self.x + np.dot(kalman_gain, innovation)
        self.P = self.P - np.dot(np.dot(kalman_gain, self.C), self.P)


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
