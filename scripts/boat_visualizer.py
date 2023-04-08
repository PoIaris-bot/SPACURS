#!/usr/bin/env python
import os
import time
import rospy
import pathlib
import matplotlib.pyplot as plt
from spacurs.msg import Path
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool, SetBoolResponse

FILE = pathlib.Path(__file__).resolve()
ROOT = FILE.parents[1]
DIRECTORY = os.path.join(ROOT, "result")
if not os.path.exists(DIRECTORY):
    os.makedirs(DIRECTORY)


class BoatVisualizer:
    def __init__(self):
        cache = {'x': [], 'y': []}
        path = None

        self.path_message = None
        self.updated_path = False
        self.pose_message = None
        self.updated_pose = False
        self.save_plot = False
        self.open_plot = False

        rospy.init_node("boat_visualizer", anonymous=True)
        rospy.Subscriber("/path", Path, self.path_callback, queue_size=1)
        rospy.Subscriber("/pose", Float32MultiArray, self.pose_callback, queue_size=1)
        rospy.Service("save_plot", SetBool, self.save_plot_handler)
        rospy.Service("open_plot", SetBool, self.open_plot_handler)

        plt.ion()
        while not rospy.is_shutdown():
            if self.open_plot:
                if self.updated_path:
                    if self.path_message.x:
                        path = [self.path_message.x, self.path_message.y]
                    else:
                        path = None
                    self.updated_path = False

                if self.updated_pose:
                    if len(cache['x']) > 500:
                        cache['x'].pop(0)
                        cache['y'].pop(0)
                    cache['x'].append(self.pose_message.data[0])
                    cache['y'].append(self.pose_message.data[1])
                    self.updated_pose = False

                plt.clf()
                plt.plot(cache['x'], cache['y'], ".-g")
                if path is not None:
                    plt.plot(path[0], path[1], ".:b")
                    plt.legend(["boat", "path"])
                else:
                    plt.legend(["boat"])
                plt.xlabel("x/m")
                plt.ylabel("y/m")
                plt.axis("square")
                plt.grid(True)

                if self.save_plot:
                    filepath = f"{DIRECTORY}/{time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())}.png"
                    plt.savefig(filepath)
                    print(f"plot {'' if os.path.exists(filepath) else 'not '}saved")
                    self.save_plot = False

                plt.pause(0.01)
            else:
                plt.close()

    def save_plot_handler(self, request):
        if request.data:
            self.save_plot = True
            return SetBoolResponse(success=True)
        else:
            self.save_plot = False
            return SetBoolResponse(success=False)

    def open_plot_handler(self, request):
        if request.data:
            self.open_plot = True
            return SetBoolResponse(success=True)
        else:
            self.open_plot = False
            return SetBoolResponse(success=False)

    def path_callback(self, message):
        self.path_message = message
        self.updated_path = True

    def pose_callback(self, message):
        self.pose_message = message
        self.updated_pose = True


if __name__ == "__main__":
    try:
        BoatVisualizer()
    except rospy.ROSInterruptException:
        pass
