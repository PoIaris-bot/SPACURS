#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray


class Visualizer:
    def __init__(self):
        rospy.init_node('visualizer', anonymous=True)
        plt.ion()

        node_name = rospy.get_name()
        self.mode = rospy.get_param(node_name + '/mode')

        self.caches = {}
        self.color = [['k.', 'bo', 'r*-'], ['g.', 'co', 'b*-']]
        if self.mode == 'auto':
            rospy.Subscriber('/visualization', Float32MultiArray, self.callback, queue_size=1)
        else:
            rospy.Subscriber('/measurement', Float32MultiArray, self.callback, queue_size=1)

    def callback(self, msg):
        if msg.data:
            plt.clf()
            legend = []

            if self.mode == 'auto':
                index = 0
                while index < len(msg.data):
                    node_id = int(msg.data[index])
                    x = msg.data[index + 1]
                    y = msg.data[index + 2]
                    target_point = msg.data[index + 3:index + 5]
                    print(target_point)
                    path_point_num = int(msg.data[index + 5])
                    path = [
                        msg.data[index + 6:index + 6 + path_point_num],
                        msg.data[index + 6 + path_point_num:index + 6 + path_point_num * 2]
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
                    index += 6 + 2 * path_point_num
            else:
                for i in range(len(msg.data) // 4):
                    node_id, x, y, theta = msg.data[i * 4: i * 4 + 4]
                    node_id = int(node_id)

                    if node_id not in self.caches.keys():
                        self.caches[node_id] = {'x': [x], 'y': [y]}
                    else:
                        self.caches[node_id]['x'].append(x)
                        self.caches[node_id]['y'].append(y)
                    plt.plot(self.caches[node_id]['x'], self.caches[node_id]['y'], self.color[node_id - 1][0])
                    legend.append('node ' + str(node_id))
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
