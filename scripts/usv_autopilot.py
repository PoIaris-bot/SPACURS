#!/usr/bin/env python
import rospy
import numpy as np
from spacurs.msg import Path
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from tools import remap_angle, constraint, PIDController


def generate_path(x, y, theta, x_goal, y_goal, r1=3, r2=2):
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

    line = np.linspace([x1, y1], [x2, y2], round(np.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)), endpoint=False).T

    thetas = np.linspace(gamma2, gamma2 + np.pi * (2 if flag else -2), round(2 * np.pi * r2), endpoint=False)
    circle = np.array([[x_goal, y_goal]]).T + r2 * np.array([np.cos(thetas), np.sin(thetas)])

    path = np.hstack([arc, line, circle])
    circle_idx = arc.shape[1] + line.shape[1]
    return path, circle_idx


class USVAutopilot:
    def __init__(self):
        rospy.init_node('usv_autopilot', anonymous=True)

        self.auto = False

        self.control_rate = 10
        self.measure_rate = 50
        self.control_count = 0

        self.path = None
        self.circle_idx = None
        self.target_idx = None
        self.segment = 5

        self.length = 0.7
        self.Delta = 3 * self.length

        self.speed_controller = None
        self.steer_controller = None

        self.control_publisher = rospy.Publisher('control', Int32MultiArray, queue_size=1)
        self.path_publisher = rospy.Publisher('path', Path, queue_size=1)

        rospy.Subscriber('/pose', Float32MultiArray, self.callback, queue_size=1)

        rospy.Service('switch_mode', SetBool, self.switch_mode_handler)

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
                x_goal, y_goal = 20, 25
                # x_goal, y_goal = 0, 10
                self.path, self.circle_idx = generate_path(x, y, theta, x_goal, y_goal)
                self.target_idx = 1

                self.speed_controller = PIDController(20, 10, 200)  # TODO
                self.steer_controller = PIDController(30, 10, 200)  # TODO

            else:
                # calculate target point
                end_idx = self.path.shape[1] - 1
                if self.target_idx + self.segment > end_idx + 1:
                    path_segment = np.hstack([
                        self.path[:, self.target_idx:end_idx + 1],
                        self.path[:, self.circle_idx:self.circle_idx + self.target_idx + self.segment - end_idx - 1]
                    ])
                else:
                    path_segment = self.path[:, self.target_idx:self.target_idx + self.segment]
                distance = np.linalg.norm(np.array([[x, y]]).T - path_segment, axis=0)
                self.target_idx += np.argmin(distance)
                if self.target_idx > end_idx:
                    self.target_idx = self.circle_idx + self.target_idx - end_idx - 1

                self.control_count += 1
                if self.control_count > self.measure_rate / self.control_rate:
                    self.control_count = 0

                    target = self.path[:, self.target_idx]
                    if self.target_idx == end_idx:
                        target_next = self.path[:, self.circle_idx]
                    else:
                        target_next = self.path[:, self.target_idx + 1]

                    alpha = np.arctan2(target_next[1] - target[1], target_next[0] - target[0])
                    x_d = target[0] + self.Delta * np.cos(alpha)
                    y_d = target[1] + self.Delta * np.sin(alpha)

                    x_e = x_d - x
                    y_e = y_d - y
                    u1 = 2 * np.tanh(1 * x_e)
                    u2 = 2 * np.tanh(1 * y_e)
                    v_d = np.sqrt(u1 ** 2 + u2 ** 2)
                    v_e = v_d - v

                    theta_d = remap_angle(np.pi / 2 - np.arctan2(y_d - y, x_d - x))
                    theta = remap_angle(np.pi / 2 - theta)
                    theta_e = remap_angle(theta_d - theta)
                    omega_d = 2 * theta_e + 0.1 * np.sign(theta_e)
                    omega_e = omega_d - omega

                    base_speed = constraint(45 + self.speed_controller.output(v_e), 0, 90)  # TODO
                    steer_speed = constraint(self.steer_controller.output(omega_e), -60, 60)  # TODO
                    left_speed = int(constraint(base_speed + steer_speed, 0, 90))
                    right_speed = int(constraint(base_speed - steer_speed, 0, 90))
                    self.control_publisher.publish(Int32MultiArray(data=[left_speed, right_speed]))

            self.path_publisher.publish(Path(
                x=[self.target_idx] + self.path[0, :].tolist(),
                y=[self.target_idx] + self.path[1, :].tolist()
            ))
        else:
            self.path = None
            self.control_count += 1
            if self.control_count > self.measure_rate / self.control_rate:
                self.control_count = 0
                self.path_publisher.publish(Path())


if __name__ == '__main__':
    try:
        USVAutopilot()
    except rospy.ROSInterruptException:
        pass
