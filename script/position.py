#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from nlink_parser.msg import LinktrackNodeframe1
from utils import remap


class KalmanFilter:
    def __init__(self, x0):
        self.x = x0
        self.P = np.zeros((4, 4))

        self.A_tilde = np.array([
            [0, 0, 1, 0],
            [0, 0, 0, 1],
            [0, 0, 0, 0],
            [0, 0, 0, 0]
        ])

        self.A = None
        self.C = np.eye(4)

        self.Q_tilde = 0.01 * np.eye(4)
        self.Q = None
        self.R = 0.01 * np.eye(4)

    def predict(self, dt):
        self.A = self.A_tilde * dt + np.eye(4)
        self.Q = self.Q_tilde * dt

        self.x = np.dot(self.A, self.x)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

    def update(self, y):
        innovation = y - np.dot(self.C, self.x)
        lambda_t = np.dot(np.dot(self.C, self.P), self.C.T) + self.R
        kalman_gain = np.dot(np.dot(self.P, self.C.T), np.linalg.inv(lambda_t))
        self.x = self.x + np.dot(kalman_gain, innovation)
        self.P = self.P - np.dot(np.dot(kalman_gain, self.C), self.P)


class Position:
    def __init__(self):
        rospy.init_node('position', anonymous=True)

        self.cache = {}
        self.node = {}
        self.x = None
        self.timestamp = rospy.get_time()

        self.publisher = rospy.Publisher('/pose', Float32MultiArray, queue_size=1)
        rospy.Subscriber('/nlink_linktrack_nodeframe1', LinktrackNodeframe1, self.callback, queue_size=1)
        rospy.spin()

    def callback(self, message):
        if message.nodes:
            dt = rospy.get_time() - self.timestamp
            for node in message.nodes:
                x, y = node.pos_3d[:2]
                if node.id not in self.cache.keys():
                    v_x, v_y = 0, 0
                else:
                    v_x = (x - self.cache[node.id][0]) / dt
                    v_y = (y - self.cache[node.id][1]) / dt
                self.cache[node.id] = [x, y, v_x, v_y]

                if node.id not in self.node.keys():
                    self.node[node.id] = KalmanFilter(np.array([[x, y, v_x, v_y]]).T)
                else:
                    self.node[node.id].predict(dt)
                    self.node[node.id].update(np.array([[x, y, v_x, v_y]]).T)

            node_id = sorted(list(self.node.keys()))
            head = self.node[node_id[0]]
            rear = self.node[node_id[1]]

            x = (head.x[0] + rear.x[0]) / 2
            y = (head.x[1] + rear.x[1]) / 2
            theta = np.arctan2(head.x[1] - rear.x[1], head.x[0] - rear.x[0])
            if self.x is None:
                v_x, v_y, omega = 0, 0, 0
            else:
                v_x = (x - self.x[0]) / dt
                v_y = (y - self.x[1]) / dt
                omega = remap(theta - self.x[2]) / dt

            self.publisher.publish(Float32MultiArray(data=[x, y, theta, v_x, v_y, omega]))
            self.x = [x, y, theta]
            self.timestamp = rospy.get_time()


if __name__ == '__main__':
    try:
        Position()
    except rospy.ROSInterruptException:
        pass
