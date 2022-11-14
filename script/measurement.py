#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from nlink_parser.msg import LinktrackNodeframe1


class LKF:
    def __init__(self, x, y, v_x, v_y):
        self.x = np.array([[x, y, v_x, v_y]]).T

        self.P = np.zeros((4, 4))

        self.A_tilde = np.array([
            [0, 0, 1, 0],
            [0, 0, 0, 1],
            [0, 0, 0, 0],
            [0, 0, 0, 0.]
        ])
        self.B_tilde = np.array([
            [0, 0],
            [0, 0],
            [1, 0],
            [0, 1.]
        ])

        self.A = None
        self.B = None
        self.C = np.eye(4)

        Q_tilde = np.random.randn(4, 4)
        self.Q_tilde = np.dot(Q_tilde, Q_tilde.T)
        self.Q = None
        R = np.random.randn(2, 2)
        self.R = np.dot(R, R.T)

    def discretize(self, dt):
        self.A = self.A_tilde + np.eye(4) * dt
        self.B = self.B_tilde * dt
        self.Q = self.Q_tilde * dt

    def predict(self, u, dt):
        self.discretize(dt)
        self.x = np.dot(self.A, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

    def update(self, y):
        innovation = y - np.dot(self.C, self.x)
        lambda_t = np.dot(np.dot(self.C, self.P), self.C.T) + self.R
        kalman_gain = np.dot(np.dot(self.P, self.C.T), np.linalg.inv(lambda_t))
        self.x = self.x + np.dot(kalman_gain, innovation)
        self.P = self.P - np.dot(np.dot(kalman_gain, self.C), self.P)


class Measurement:
    def __init__(self):
        rospy.init_node('measurement', anonymous=True)
        self.caches = {}
        self.LKFs = {}
        self.t = rospy.get_time()
        self.publisher = rospy.Publisher('/measurement', Float32MultiArray, queue_size=1)
        rospy.Subscriber('/nlink_linktrack_nodeframe1', LinktrackNodeframe1, self.callback, queue_size=1)

    def callback(self, msg):
        nodes = msg.nodes
        if nodes:
            t = rospy.get_time()
            dt = t - self.t
            for node in nodes:
                x, y = node.pos_3d[:2]
                if node.id not in self.caches.keys():
                    v_x = v_y = a_x = a_y = 0
                else:
                    v_x = (x - self.caches[node.id][0]) / dt
                    v_y = (y - self.caches[node.id][1]) / dt
                    a_x = (v_x - self.caches[node.id][2]) / dt
                    a_y = (v_y - self.caches[node.id][3]) / dt
                self.caches[node.id] = [x, y, v_x, v_y]
                if node.id not in self.LKFs.keys():
                    self.LKFs[node.id] = LKF(x, y, v_x, v_y)
                else:
                    self.LKFs[node.id].predict(np.array([[a_x, a_y]]).T, dt)
                    self.LKFs[node.id].update(np.array([[x, y, v_x, v_y]]).T)
            data = []
            node_ids = list(self.LKFs.keys())
            for i in range(len(node_ids) // 2):
                node_id = node_ids[i * 2] / 2 + 1
                head = self.LKFs[node_ids[i * 2]]
                rear = self.LKFs[node_ids[i * 2 + 1]]
                x = (head.x[0] + rear.x[0]) / 2
                y = (head.x[1] + rear.x[1]) / 2
                theta = np.atan2(head.x[1] - rear.x[1], head.x[0] - rear.x[0])
                data += [node_id, x, y, theta]
            self.publisher.publish(Float32MultiArray(data=data))
            self.t = t


if __name__ == '__main__':
    try:
        measurement = Measurement()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
