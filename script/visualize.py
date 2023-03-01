#!/usr/bin/env python
import os
import time
import queue
import rospy
import pathlib
import matplotlib.pyplot as plt
from multiprocessing import Process, Queue
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool, SetBoolResponse
from spacurs.msg import Path

FILE = pathlib.Path(__file__).resolve()
ROOT = FILE.parents[1]


def subscriber():
    rospy.init_node('visualize', anonymous=True)
    rospy.Subscriber('/path', Path, path_callback, queue_size=1)
    rospy.Subscriber('/pose', Float32MultiArray, pose_callback, queue_size=1)
    rospy.Service('save_plot', SetBool, handler)
    rospy.spin()


def handler(request):
    global save_queue
    if request.data:
        try:
            _ = save_queue.get(False)
        except queue.Empty:
            pass
        save_queue.put(True)
        return SetBoolResponse(success=True)
    return SetBoolResponse(success=False)


def path_callback(path):
    global path_queue
    try:
        _ = path_queue.get(False)
    except queue.Empty:
        pass
    path_queue.put(path)


def pose_callback(pose):
    global pose_queue
    try:
        _ = pose_queue.get(False)
    except queue.Empty:
        pass
    pose_queue.put(pose)


if __name__ == '__main__':
    path_queue = Queue(1)
    pose_queue = Queue(1)
    save_queue = Queue(1)

    cache = {'x': [], 'y': []}
    path = None
    closest_idx = None
    target_idx = None

    p = Process(target=subscriber, args=())
    p.daemon = True
    p.start()

    plt.ion()
    while not rospy.is_shutdown():
        try:
            path_message = path_queue.get(False)
            if path_message.x:
                closest_idx = int(path_message.x[0])
                target_idx = int(path_message.x[1])
                path = [path_message.x[2:], path_message.y[2:]]
            else:
                path = None
        except queue.Empty:
            pass

        try:
            pose_message = pose_queue.get(False)
            cache['x'].append(pose_message.data[0])
            cache['y'].append(pose_message.data[1])
        except queue.Empty:
            pass

        plt.clf()
        plt.plot(cache['x'], cache['y'], '.-g')
        if path is not None:
            plt.plot(path[0], path[1], '.:b')
            plt.plot(path[0][closest_idx], path[1][closest_idx], '*c')
            plt.plot(path[0][target_idx], path[1][target_idx], '*r')
            plt.legend(['usv', 'path', 'closest', 'target'])
        else:
            plt.legend(['usv'])
        plt.xlabel('x/m')
        plt.ylabel('y/m')
        plt.axis('square')
        plt.grid(True)

        try:
            save_plot = save_queue.get(False)
        except queue.Empty:
            save_plot = False
        if save_plot:
            filepath = os.path.join(ROOT, 'result/{}.png'.format(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())))
            plt.savefig(filepath)
            if os.path.exists(filepath):
                print('plot saved')
            else:
                print('plot not saved')
            save_plot = False

        plt.pause(0.01)
