#!/usr/bin/env python
import queue
import rospy
import matplotlib.pyplot as plt
from multiprocessing import Process, Queue
from std_msgs.msg import Float32MultiArray
from spacurs.msg import Path

message_queue = Queue(1)


def subscriber():
    rospy.init_node('visualizer', anonymous=True)
    rospy.Subscriber('/path', Path, callback, queue_size=1)
    rospy.Subscriber('/position', Float32MultiArray, callback, queue_size=1)
    rospy.spin()


def callback(message):
    global message_queue
    try:
        _ = message_queue.get(False)
    except queue.Empty:
        pass
    message_queue.put(message)


if __name__ == '__main__':
    cache = {'x': [], 'y': []}
    path = None
    closest_idx = None
    target_idx = None
    Process(target=subscriber, args=()).start()
    plt.ion()
    while True:
        try:
            message = message_queue.get(False)
            plt.clf()
            if isinstance(message, Float32MultiArray):
                cache['x'].append(message.data[0])
                cache['y'].append(message.data[1])
            elif isinstance(message, Path):
                if message.x:
                    closest_idx = int(message.x[0])
                    target_idx = int(message.x[1])
                    path = [message.x[2:], message.y[2:]]
                else:
                    path = None

            plt.plot(cache['x'], cache['y'], 'g.')
            if path is not None:
                plt.plot(path[0], path[1], 'b.-')
                plt.plot(path[0][closest_idx], path[1][closest_idx], 'r*')
                plt.plot(path[0][target_idx], path[1][target_idx], 'r*')
                plt.legend(['usv', 'path', 'closest', 'target'])
            else:
                plt.legend(['usv'])
            plt.xlabel('x/m')
            plt.ylabel('y/m')
            plt.axis('square')
            plt.grid(True)
            plt.pause(0.01)
        except queue.Empty:
            pass
