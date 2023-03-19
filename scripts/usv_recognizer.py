#!/usr/bin/env python
import os
import rospy
import pathlib
import numpy as np
from std_msgs.msg import Int32MultiArray, Float32MultiArray

FILE = pathlib.Path(__file__).resolve()
ROOT = FILE.parents[1]
DIRECTORY = os.path.join(ROOT, 'result')
if not os.path.exists(DIRECTORY):
    os.makedirs(DIRECTORY)


class KalmanRecognizer:
    def __init__(self, v, tau):
        self.x = np.zeros((11, 1))
        self.P = np.eye(11)

        self.v = v
        self.tau = tau

        self.A = None
        self.C = np.hstack([np.eye(3), np.zeros((3, 8))])

        self.q = 1e-2 * np.eye(11)
        self.Q = None
        self.R = 1e-2 * np.eye(3)

        self.timestamp = rospy.get_time()

    def predict(self, tau):
        sigma, delta = self.tau
        self.tau = tau
        u, v, r = self.v

        timestamp = rospy.get_time()
        dt = timestamp - self.timestamp

        a = dt * np.array([
            [v * r, u, sigma, 0, 0, 0, 0, 0],
            [0, 0, 0, u * r, v, 0, 0, 0],
            [0, 0, 0, 0, 0, u * v, r, delta]
        ])

        self.A = np.block([
            [np.zeros((3, 3)), a],
            [np.zeros((8, 3)), np.eye(8)]
        ])

        self.Q = self.q * dt

        self.x = np.dot(self.A, self.x)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

        self.timestamp = timestamp

    def update(self, v):
        y = np.array(v).reshape((3, 1)) - np.array(self.v).reshape(3, 1)
        self.v = v

        innovation = y - np.dot(self.C, self.x)
        lambda_t = np.dot(np.dot(self.C, self.P), self.C.T) + self.R
        kalman_gain = np.dot(np.dot(self.P, self.C.T), np.linalg.inv(lambda_t))
        self.x = self.x + np.dot(kalman_gain, innovation)
        self.P = self.P - np.dot(np.dot(kalman_gain, self.C), self.P)


class USVRecognizer:
    def __init__(self):
        rospy.init_node('usv_recognizer', anonymous=True)
        self.recognizer = None
        self.tau = None
        self.cache = []
        rospy.Subscriber('/pose', Float32MultiArray, self.pose_callback, queue_size=1)
        rospy.Subscriber('/control', Int32MultiArray, self.control_callback, queue_size=1)
        rospy.spin()

    def pose_callback(self, message):
        x, y, theta, v_x, v_y, omega = message.data

        u = v_x * np.cos(theta) + v_y * np.sin(theta)
        v = -v_x * np.sin(theta) + v_y * np.cos(theta)
        r = omega

        if self.recognizer is None:
            if self.tau is not None:
                self.recognizer = KalmanRecognizer([u, v, r], self.tau)
        else:
            self.recognizer.predict(self.tau)
            self.recognizer.update([u, v, r])
            self.cache.append([rospy.get_time(), *self.recognizer.x[3:].ravel().tolist()])
            if len(self.cache) > 50:
                np.savetxt(os.path.join(DIRECTORY, 'param.txt'), np.array(self.cache).squeeze(), delimiter=',')

    def control_callback(self, message):
        left_output, right_output = message.data
        sigma = (left_output + right_output) / 2
        delta = (left_output - right_output) / 2
        self.tau = [sigma, delta]


if __name__ == '__main__':
    try:
        USVRecognizer()
    except rospy.ROSInterruptException:
        pass
