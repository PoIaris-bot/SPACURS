#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from xbee import XBeeSender
from controller import FuzzyController, PurePursuitController
from tools import compute_radius, find_closest_point, compute_error


class Node:
    def __init__(self, x0):
        self.x = x0
        self.path = None

        self.r = 1
        self.path_point_num = 250

        self.target_id = None
        self.closest_id = None
        self.init_path()

        self.preview = 10
        self.segment = 20

        self.speed_controller = FuzzyController(speed_min=1, speed_max=4, r_min=0.1, r_mid=0.7, r_max=5, dr_max=0.3)
        self.steer_controller = PurePursuitController(kp=0.2, length=0.2, steer_min=80, steer_max=110)

    def init_path(self):
        x, y, theta = self.x
        self.path = np.array([[x + self.r * np.sin(theta), y - self.r * np.cos(theta)]]) + self.r * np.array(
            [np.cos(np.linspace(theta + np.pi / 2, theta - 3 * np.pi / 2, self.path_point_num)),
             np.sin(np.linspace(theta + np.pi / 2, theta - 3 * np.pi / 2, self.path_point_num))]).T
        self.closest_id = 5

    def update(self, x):
        self.x = x
        self.closest_id += find_closest_point(self.x, self.path[self.closest_id:self.closest_id + self.segment, :])

    def control_output(self):
        if self.closest_id < self.path_point_num - 3 - self.preview:
            self.target_id = self.closest_id + self.preview
            point1 = self.path[self.target_id, :]
            point2 = self.path[self.target_id + 1, :]
            point3 = self.path[self.target_id + 2, :]
            r = compute_radius(point1, point2, point3)

            error = compute_error(self.x, point1)
            preview_distance = np.sqrt((self.x[0] - point1[0]) ** 2 + (self.x[1] - point1[1]) ** 2)
            speed = str(int(self.speed_controller.output(r)))
            steer = str(int(self.steer_controller.output(error, preview_distance)))
            if len(steer) == 2:
                steer = '0' + steer
            command = speed + steer
        else:
            command = '0095'
        return command

    def state(self):
        x = self.x[:2].tolist()
        target_point = self.path[self.target_id, :].tolist()
        path = self.path.T.ravel().tolist()
        return x + target_point + path


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

        self.control_rate = 10
        self.control_count = 0

        self.nodes = {}

        self.publisher = rospy.Publisher('visualization', Float32MultiArray, queue_size=1)
        rospy.Subscriber('/measurement', Float32MultiArray, self.callback, queue_size=1)

    def callback(self, msg):
        if msg.data:
            for i in range(len(msg.data) // 4):
                node_id, x, y, theta = msg.data[i * 4: i * 4 + 4]
                node_id = int(node_id)

                if node_id not in self.nodes.keys():
                    self.nodes[node_id] = Node(np.array([x, y, theta]))
                else:
                    self.nodes[node_id].update(np.array([x, y, theta]))

            self.control_count += 1
            if self.control_count >= 50 / self.control_rate:
                self.control_count = 0

                data = []
                for node_id in self.nodes.keys():
                    command = self.nodes[node_id].control_output()
                    self.xbee_sender.send_to_one(str(node_id + 14), command)
                    data.append(node_id)
                    data += self.nodes[node_id].state()
                self.publisher.publish(Float32MultiArray(data=data))



if __name__ == '__main__':
    try:
        spacurs = SPACURS()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
