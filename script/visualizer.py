#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from spacurs.msg import PointArray


class Visualizer:
    def __init__(self):
        rospy.init_node('visualizer', anonymous=True)
        plt.ion()

        self.data = [{}, {}, {}]  # [{paths}, {poses}, {targets}]
        self.color = [['r-', 'g-'], ['m.', 'c.'], ['b*', 'k*']]
        self.legend_prefix = ['path', 'node', 'target']

        rospy.Subscriber('/point_array', PointArray, self.callback)

    def callback(self, point_array):
        plt.clf()
        legend = []
        for point in point_array.points:
            if point.type == 1 and point.id in self.data[point.type].keys():
                self.data[point.type][point.id]['x'] += point.x
                self.data[point.type][point.id]['y'] += point.y
            else:
                self.data[point.type][point.id] = {
                    'x': point.x,
                    'y': point.y,
                    'c': self.color[point.type][point.id - 1]
                }
            plt.plot(self.data[point.type][point.id]['x'], self.data[point.type][point.id]['y'],
                     self.data[point.type][point.id]['c'])
            legend.append(self.legend_prefix[point.type] + str(point.id))
        plt.xlabel('x/m')
        plt.ylabel('y/m')
        plt.legend(legend)
        plt.axis('square')
        plt.grid(True)
        plt.legend(legend)
        plt.pause(0.01)


if __name__ == '__main__':
    try:
        visualizer = Visualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
