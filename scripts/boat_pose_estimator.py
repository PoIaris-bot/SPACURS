#!/usr/bin/env python
import rospy
import numpy as np
from tools import remap_angle, KalmanFilter
from std_msgs.msg import Float32MultiArray
from nlink_parser.msg import LinktrackTagframe0, LinktrackNodeframe2


class BoatPoseEstimator:
    def __init__(self):
        rospy.init_node('boat_pose_estimator', anonymous=True)

        self.head = None
        self.updated_head = False

        self.rear = None
        self.updated_rear = False

        self.boat = None

        self.publisher = rospy.Publisher('/pose', Float32MultiArray, queue_size=1)
        rospy.Subscriber('/nlink_linktrack_tagframe0', LinktrackTagframe0, self.callback, queue_size=1)
        rospy.Subscriber('/nlink_linktrack_nodeframe2', LinktrackNodeframe2, self.callback, queue_size=1)

        while not rospy.is_shutdown():
            if self.updated_head and self.updated_rear:
                x = (self.head.x[0, 0] + self.rear.x[0, 0]) / 2
                y = (self.head.x[1, 0] + self.rear.x[1, 0]) / 2
                v_x = (self.head.x[2, 0] + self.rear.x[2, 0]) / 2
                v_y = (self.head.x[3, 0] + self.rear.x[3, 0]) / 2
                omega = (self.head.x[4, 0] + self.rear.x[4, 0]) / 2
                theta = np.arctan2(self.head.x[1, 0] - self.rear.x[1, 0], self.head.x[0, 0] - self.rear.x[0, 0])

                if self.boat is None:
                    self.boat = KalmanFilter(
                        np.array([[x, y, theta, v_x, v_y, omega]]).T,
                        np.array([
                            [0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 1],
                            [0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0],
                        ])
                    )
                else:
                    self.boat.predict()
                    self.boat.update(np.array([[x, y, theta, v_x, v_y, omega]]).T)
                    self.boat.x[2, 0] = remap_angle(self.boat.x[2, 0])

                self.publisher.publish(Float32MultiArray(data=[x, y, theta, v_x, v_y, omega]))
                self.updated_head = False
                self.updated_rear = False

    def callback(self, message):
        def update(node, _x, _y, _v_x, _v_y, _omega):
            if node is None:
                node = KalmanFilter(
                    np.array([[_x, _y, _v_x, _v_y, _omega]]).T,
                    np.array([
                        [0, 0, 1, 0, 0],
                        [0, 0, 0, 1, 0],
                        [0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0]
                    ])
                )
            else:
                node.predict()
                node.update(np.array([[_x, _y, _v_x, _v_y, _omega]]).T)
            return node

        x, y = message.pos_3d[:2]
        v_x, v_y = message.vel_3d[:2]
        omega = message.imu_gyro_3d[2]

        if message.id % 2 != 0:
            self.head = update(self.head, x, y, v_x, v_y, omega)
            self.updated_head = True
        else:
            self.rear = update(self.rear, x, y, v_x, v_y, omega)
            self.updated_rear = True


if __name__ == '__main__':
    try:
        BoatPoseEstimator()
    except rospy.ROSInterruptException:
        pass
