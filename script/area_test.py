#!/usr/bin/env python3
import numpy as np
import rospy
from circle import make_circle
from std_msgs.msg import Float32MultiArray
from matplotlib import pyplot as plt


class AreaTest:
    def __init__(self):
        self.cache = [[], []]
        self.count = 0
        plt.ion()

        rospy.init_node('area_test', anonymous=True)
        rospy.Subscriber('/measurement', Float32MultiArray, self.callback)

    def callback(self, msg):
        def draw_circle(c, r):
            x = np.linspace(c[0] - r, c[0] + r, 5000)
            y1 = np.sqrt(r ** 2 - (x - c[0]) ** 2) + c[1]
            y2 = -np.sqrt(r ** 2 - (x - c[0]) ** 2) + c[1]

            plt.plot(x, y1, c='k')
            plt.plot(x, y2, c='k')
            plt.text(c[0], c[1], 'center = (%.3fm, %.3fm)\nr = %.3fm' % (c[0], c[1], r), fontsize=15, c='r')

        if msg.data:
            for i in range(len(msg.data) // 4):
                node_id, x, y, theta = msg.data[i * 4: i * 4 + 4]
                node_id = int(node_id)
                self.cache[0].append(x)
                self.cache[1].append(y)
                self.count += 1

                plt.clf()
                area_info = make_circle([(x, y) for x, y in zip(self.cache[0], self.cache[1])])
                plt.scatter(self.cache[0], self.cache[1])
                draw_circle(area_info[:2], area_info[2])
                plt.xlabel('x/m')
                plt.ylabel('y/m')
                plt.axis('square')
                plt.pause(0.1)


if __name__ == '__main__':
    try:
        area_test = AreaTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
