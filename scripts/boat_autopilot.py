#!/usr/bin/env python
import sys
import rospy
import numpy as np
from spacurs.msg import Path
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from tools import remap_angle, constraint, PIDController


def boat_navigator(x, y, theta, x_goal, y_goal, r1=3, r2=2):
    theta_goal = remap_angle(np.arctan2(y_goal - y, x_goal - x))
    flag = remap_angle(theta - theta_goal) < 0

    xo = x + r1 * np.cos(theta + np.pi / (2 if flag else -2))
    yo = y + r1 * np.sin(theta + np.pi / (2 if flag else -2))

    beta = remap_angle(np.arctan2(y_goal - yo, x_goal - xo))
    alpha = np.arccos((r1 - r2) / np.sqrt((y_goal - yo) ** 2 + (x_goal - xo) ** 2))

    gamma1 = remap_angle(np.arctan2(y - yo, x - xo)) + 2 * np.pi
    gamma2 = remap_angle(beta + (-alpha if flag else alpha)) + 2 * np.pi

    x1 = xo + r1 * np.cos(gamma2)
    y1 = yo + r1 * np.sin(gamma2)
    x2 = x_goal + r2 * np.cos(gamma2)
    y2 = y_goal + r2 * np.sin(gamma2)

    if flag and gamma2 < gamma1:
        gamma2 += 2 * np.pi
    if not flag and gamma2 > gamma1:
        gamma1 += 2 * np.pi

    gammas = np.linspace(gamma1, gamma2, round(abs(gamma1 - gamma2) * r1), endpoint=False)
    arc = np.array([[xo, yo]]).T + r1 * np.array([np.cos(gammas), np.sin(gammas)])

    num_line_point = round(np.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2))
    line = np.array([
        np.linspace(x1, x2, num_line_point, endpoint=False),
        np.linspace(y1, y2, num_line_point, endpoint=False)
    ]).reshape(2, -1)

    thetas = np.linspace(gamma2, gamma2 + np.pi * (2 if flag else -2), round(2 * np.pi * r2), endpoint=False)
    circle = np.array([[x_goal, y_goal]]).T + r2 * np.array([np.cos(thetas), np.sin(thetas)])

    path = np.hstack([arc, line, circle])
    circle_idx = arc.shape[1] + line.shape[1]
    return path, circle_idx


class BoatAutopilot:
    def __init__(self):
        rospy.init_node("boat_autopilot", anonymous=True)

        self.auto = False

        self.control_rate = 10
        self.measure_rate = 50
        self.control_count = 0

        self.goal = None
        self.path = None
        self.circle_idx = None
        self.closest_idx = None
        self.segment = 5

        self.length = 0.7
        self.Delta = 3 * self.length  # best 3

        self.speed_controller = None
        self.steer_controller = None

        self.control_publisher = rospy.Publisher("control", Int32MultiArray, queue_size=1)
        self.path_publisher = rospy.Publisher("path", Path, queue_size=1)

        rospy.Subscriber("/pose", Float32MultiArray, self.callback, queue_size=1)

        rospy.Service("switch_mode", SetBool, self.switch_mode_handler)

        rospy.spin()

    def switch_mode_handler(self, request):
        if request.data:
            self.auto = True
            return SetBoolResponse(success=True)
        else:
            self.auto = False
            return SetBoolResponse(success=False)

    def callback(self, message):
        if self.auto:
            x, y, theta, v_x, v_y, omega = message.data
            v = np.sqrt(v_x ** 2 + v_y ** 2)

            if self.path is None:
                # generate path
                if self.goal is None:
                    if sys.version[0] == '2':
                        self.goal = list(map(eval, raw_input("goal: ").split()))
                    else:
                        self.goal = list(map(eval, input("goal: ").split()))
                    return
                else:
                    self.path, self.circle_idx = boat_navigator(x, y, theta, *self.goal)
                    self.closest_idx = 0

                    self.speed_controller = PIDController(20, 10, 100)  # best 20 10 100
                    self.steer_controller = PIDController(70, 10, 500)  # TODO: best 70 10 500， try 80 10 500
            else:
                # calculate target point
                end_idx = self.path.shape[1] - 1
                if self.closest_idx + self.segment > end_idx + 1:
                    path_segment = np.hstack([
                        self.path[:, self.closest_idx:end_idx + 1],
                        self.path[:, self.circle_idx:self.circle_idx + self.closest_idx + self.segment - end_idx - 1]
                    ])
                else:
                    path_segment = self.path[:, self.closest_idx:self.closest_idx + self.segment]
                distance = np.linalg.norm(np.array([[x, y]]).T - path_segment, axis=0)
                self.closest_idx += np.argmin(distance)
                if self.closest_idx > end_idx:
                    self.closest_idx = self.circle_idx + self.closest_idx - end_idx - 1

                self.control_count += 1
                if self.control_count > self.measure_rate / self.control_rate:
                    self.control_count = 0

                    x_closest, y_closest = self.path[:, self.closest_idx]
                    if self.closest_idx == end_idx:
                        x_target, y_target = self.path[:, self.circle_idx]
                    else:
                        x_target, y_target = self.path[:, self.closest_idx + 1]

                    alpha = np.arctan2(y_target - y_closest, x_target - x_closest)
                    x_d = x_closest + self.Delta * np.cos(alpha)
                    y_d = y_closest + self.Delta * np.sin(alpha)

                    x_e = x_d - x
                    y_e = y_d - y
                    u1 = 1.0 * np.tanh(3.0 * x_e)  # best 1.0 3.0
                    u2 = 1.0 * np.tanh(3.0 * y_e)  # best 1.0 3.0
                    v_d = np.sqrt(u1 ** 2 + u2 ** 2)
                    v_e = v_d - v

                    theta_d = remap_angle(np.pi / 2 - np.arctan2(y_d - y, x_d - x))
                    theta = remap_angle(np.pi / 2 - theta)
                    theta_e = remap_angle(theta_d - theta)
                    omega_d = -(3.0 * theta_e + 0.1 * np.sign(theta_e))  # best 3.0 0.1
                    omega_e = omega_d - omega

                    base_speed = constraint(45 + self.speed_controller.output(v_e), 0, 90)  # best 0 90
                    steer_speed = constraint(self.steer_controller.output(omega_e), -50, 50)  # best -50 50
                    left_speed = int(constraint(base_speed - steer_speed, 0, 90))
                    right_speed = int(constraint(base_speed + steer_speed, 0, 90))
                    self.control_publisher.publish(Int32MultiArray(data=[left_speed, right_speed]))

            self.path_publisher.publish(Path(
                x=self.path[0, :].tolist(),
                y=self.path[1, :].tolist()
            ))
        else:
            self.goal = None
            self.path = None
            self.control_count += 1
            if self.control_count > self.measure_rate / self.control_rate:
                self.control_count = 0
                self.path_publisher.publish(Path())


if __name__ == "__main__":
    try:
        BoatAutopilot()
    except rospy.ROSInterruptException:
        pass
