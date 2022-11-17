#!/usr/bin/env python3
import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray


class Visualizer:
    def __init__(self):
        rospy.init_node('visualizer', anonymous=True)
        plt.ion()

        self.path_point_num = 250
        self.N = 1 + 2 + 2 + self.path_point_num * 2

        self.caches = {}
        self.color = [['k.', 'm*', 'r-'], ['g.', 'c*', 'b-']]

        rospy.Subscriber('/visualization', Float32MultiArray, self.callback)

    def callback(self, msg):
        if msg.data:
            plt.clf()
            legend = []
            for i in range(len(msg.data) // self.N):
                node_id = int(msg.data[i * self.N])
                x = msg.data[i * self.N + 1]
                y = msg.data[i * self.N + 2]
                target_point = msg.data[i * self.N + 3:i * self.N + 5]
                path = [
                    msg.data[i * self.N + 5:i * self.N + 5 + self.path_point_num],
                    msg.data[i * self.N + 5 + self.path_point_num:(i + 1) * self.N]
                ]
                if node_id not in self.caches.keys():
                    self.caches[node_id] = {'x': [x], 'y': [y]}
                else:
                    self.caches[node_id]['x'].append(x)
                    self.caches[node_id]['y'].append(y)

                plt.plot(self.caches[node_id]['x'], self.caches[node_id]['y'], self.color[node_id - 1][0])
                plt.plot(target_point[0], target_point[1], self.color[node_id - 1][1])
                plt.plot(path[0], path[1], self.color[node_id - 1][2])
                legend += ['node ' + str(node_id), 'target ' + str(node_id), 'path ' + str(node_id)]
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
