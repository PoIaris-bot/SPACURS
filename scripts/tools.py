import rospy
import numpy as np


def remap_angle(angle, mode="-pi_pi"):
    is_ndarray = isinstance(angle, np.ndarray)
    if not is_ndarray:
        angle = np.array([angle])

    angle = np.arctan2(np.sin(angle), np.cos(angle))
    if mode == "0_2pi":
        angle[np.where(angle < 0)] += 2 * np.pi

    if not is_ndarray:
        return float(angle)
    return angle


def constraint(value, lower, upper):
    if value > upper:
        value = upper
    if value < lower:
        value = lower
    return value


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


class ParticleSwarmOptimizer:
    def __init__(self, dim, num_particle, lower_bound, upper_bound, max_iter, fitness_func):
        self.dim = dim
        self.lower_bound = np.array(lower_bound).reshape(1, -1)
        self.upper_bound = np.array(upper_bound).reshape(1, -1)
        self.max_iter = max_iter
        self.fitness_func = fitness_func

        self.x = self.lower_bound + np.random.random(size=(num_particle, dim)) * (self.upper_bound - self.lower_bound)
        self.v = np.random.randn(num_particle, dim)

        self.fitness_particle_best = self.fitness_func(self.x)
        self.x_particle_best = self.x

        self.fitness_global_best = np.min(self.fitness_particle_best)
        self.x_global_best = self.x_particle_best[np.argmin(self.fitness_particle_best), :]

    def run(self):
        for _ in range(self.max_iter):
            self.v = 0.6 * self.v + np.random.randn() * (self.x_particle_best - self.x) + np.random.randn() * (
                    self.x_global_best - self.x)
            self.x = self.x + self.v

            for i in range(self.dim):
                self.x[:, i] = np.clip(self.x[:, i], self.lower_bound[0, i], self.upper_bound[0, i])

            fitness = self.fitness_func(self.x)
            idx = np.where(fitness < self.fitness_particle_best)
            if len(idx):
                self.fitness_particle_best[idx] = fitness[idx]
                self.x_particle_best[idx, :] = self.x[idx, :]

            self.fitness_global_best = np.min(self.fitness_particle_best)
            self.x_global_best = self.x_particle_best[np.argmin(self.fitness_particle_best), :]
        return self.x_global_best, self.fitness_global_best
