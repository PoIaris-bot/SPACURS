#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from usv import USV
from xbee import XBeeSender


class SPACURS:
    def __init__(self):
        rospy.init_node('spacurs', anonymous=True)
        # XBee module
        node_name = rospy.get_name()
        port = rospy.get_param(node_name + '/port')
        baud_rate = rospy.get_param(node_name + '/baud_rate')
        remote_nodes = eval(rospy.get_param(node_name + '/remote_nodes'))
        self.xbee_sender = None
        while self.xbee_sender is None:
            self.xbee_sender = XBeeSender(port, baud_rate, remote_nodes)

        self.control_rate = 10
        self.measure_rate = 50
        self.control_count = 0

        self.USVs = {}

        self.publisher = rospy.Publisher('visualization', Float32MultiArray, queue_size=1)
        rospy.Subscriber('/measurement', Float32MultiArray, self.callback, queue_size=1)

    def callback(self, msg):
        if msg.data:
            for i in range(len(msg.data) // 4):
                node_id, x, y, theta = msg.data[i * 4: i * 4 + 4]
                node_id = int(node_id)

                if node_id not in self.USVs.keys():
                    x_goal, y_goal = map(eval, input('Please enter destination for USV {}: '.format(node_id)).split())
                    self.USVs[node_id] = USV(np.array([x, y, theta]), np.array([x_goal, y_goal]))
                else:
                    self.USVs[node_id].update(np.array([x, y, theta]))

            self.control_count += 1
            if self.control_count >= self.measure_rate / self.control_rate:
                self.control_count = 0

                data = []
                for node_id in self.USVs.keys():
                    data.append(node_id)
                    data += self.USVs[node_id].data()

                    command = self.USVs[node_id].control()
                    self.xbee_sender.send_to_one(str(node_id), command)
                self.publisher.publish(Float32MultiArray(data=data))


if __name__ == '__main__':
    try:
        spacurs = SPACURS()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
