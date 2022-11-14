#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from xbee import XBeeSender
from spacurs.msg import Point, PointArray
from controller import FuzzyController, PurePursuitController
from tools import compute_radius, find_closest_point, compute_error


class SPACURS:
    def __init__(self):
        rospy.init_node('spacurs_demo', anonymous=True)
        # XBee module
        node_name = rospy.get_name()
        port = rospy.get_param(node_name + '/port')
        baud_rate = rospy.get_param(node_name + '/baud_rate')
        remote_nodes = eval(rospy.get_param(node_name + '/remote_nodes'))
        self.xbee_sender = None
        while self.xbee_sender is None:
            self.xbee_sender = XBeeSender(port, baud_rate, remote_nodes)

        # controller
        self.control_rate = 10
        self.control_count = 0
        self.speed_controller = FuzzyController(speed_min=1, speed_max=5, r_min=0.1, r_mid=0.7, r_max=5, dr_max=0.3)
        self.steer_controller = PurePursuitController(kp=0.25, length=0.2, steer_min=80, steer_max=110)

        self.preview = 10
        self.segment = 20
        self.poses = {}
        self.paths = {}
        self.r = {}
        self.closest = {}
        self.target = {}

        self.publisher = rospy.Publisher('point_array', PointArray, queue_size=10)
        rospy.Subscriber('/measurement', Float32MultiArray, self.callback, queue_size=1)

    def callback(self, msg):
        if msg.data:
            for i in range(len(msg.data) // 4):
                node_id, x, y, theta = msg.data[i * 4: i * 4 + 4]
                self.poses[int(node_id)] = np.array([x, y, theta])
            if not self.paths:
                self.generate_circle_paths()

            self.control_count += 1
            if self.control_count >= 50 / self.control_rate:
                self.control_count = 0
                self.send_command()
                self.visualize()

    def generate_circle_paths(self):
        for node_id in self.poses.keys():
            r = 1.2
            x, y, theta = self.poses[node_id]
            num = 250
            self.paths[node_id] = np.array([[x + r * np.sin(theta), y - r * np.cos(theta)]]) + r * np.array(
                [np.cos(np.linspace(theta + np.pi / 2, theta - 3 * np.pi / 2, num)),
                 np.sin(np.linspace(theta + np.pi / 2, theta - 3 * np.pi / 2, num))]).T
            self.closest[node_id] = 5

    def send_command(self):
        for node_id in self.poses.keys():
            pose = self.poses[node_id]
            path = self.paths[node_id]
            closest = self.closest[node_id]

            closest += find_closest_point(pose, path[closest:closest + self.segment, :])
            self.closest[node_id] = closest
            if closest < self.paths[node_id].shape[0] - 3 - self.preview:
                target = self.target[node_id] = closest + self.preview
                point1 = path[target, :]
                point2 = path[target + 1, :]
                point3 = path[target + 2, :]
                r = compute_radius(point1, point2, point3)
                dr = 0 if node_id not in self.r.keys() else (r - self.r[node_id])
                self.r[node_id] = r

                error = compute_error(pose, point1)
                preview_distance = np.sqrt((pose[0] - point1[0]) ** 2 + (pose[1] - point1[1]) ** 2)
                speed = str(self.speed_controller.output(r, dr) + 1)
                steer = str(self.steer_controller.output(error, preview_distance))
                if len(steer) == 2:
                    steer = '0' + steer
                command = speed + steer
                self.xbee_sender.send_to_one(str(node_id), command)
            else:
                self.xbee_sender.send_to_one(str(node_id), '0095')

    def visualize(self):
        point_array = PointArray()
        for node_id in self.paths.keys():
            point = Point()
            point.type = 0
            point.id = node_id
            point.x = self.paths[node_id][:, 0].tolist()
            point.y = self.paths[node_id][:, 1].tolist()
            point_array.points.append(point)
        for node_id in self.poses.keys():
            point = Point()
            point.type = 1
            point.id = node_id
            point.x = [self.poses[node_id].x]
            point.y = [self.poses[node_id].y]
            point_array.points.append(point)
        for node_id in self.target.keys():
            point = Point()
            point.type = 2
            point.id = node_id
            point.x = [self.paths[node_id][self.target[node_id], 0]]
            point.y = [self.paths[node_id][self.target[node_id], 1]]
            point_array.points.append(point)
        self.publisher.publish(point_array)


if __name__ == '__main__':
    try:
        spacurs = SPACURS()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
