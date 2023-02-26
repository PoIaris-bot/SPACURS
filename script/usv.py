#!/usr/bin/env python
import rospy
import pygame
import numpy as np
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from spacurs.msg import Path


def remap(value):
    while value >= np.pi or value < -np.pi:
        if value >= np.pi:
            value -= 2 * np.pi
        if value < -np.pi:
            value += 2 * np.pi
    return value


def constraint(value, lower, upper):
    if value > upper:
        value = upper
    if value < lower:
        value = lower
    return value


def generate_path(x, y, theta, x_goal, y_goal, r1=3, r2=2):
    theta_goal = remap(np.arctan2(y_goal - y, x_goal - x))
    flag = remap(theta - theta_goal) < 0

    if flag:
        x_o = x + r1 * np.cos(theta + np.pi / 2)
        y_o = y + r1 * np.sin(theta + np.pi / 2)
    else:
        x_o = x + r1 * np.cos(theta - np.pi / 2)
        y_o = y + r1 * np.sin(theta - np.pi / 2)

    beta = remap(np.arctan2(y_goal - y_o, x_goal - x_o))
    alpha = np.arccos(r1 / np.sqrt((y_goal - y_o) ** 2 + (x_goal - x_o) ** 2))
    gamma1 = remap(np.arctan2(y - y_o, x - x_o)) + 2 * np.pi
    if flag:
        gamma2 = remap(beta - alpha) + 2 * np.pi
    else:
        gamma2 = remap(beta + alpha) + 2 * np.pi

    x_c = x_o + r1 * np.cos(gamma2)
    y_c = y_o + r1 * np.sin(gamma2)

    if flag:
        if gamma2 < gamma1:
            gamma2 += 2 * np.pi
    else:
        if gamma2 > gamma1:
            gamma1 += 2 * np.pi

    gammas = np.arange(gamma1, gamma2, np.pi / 20 if flag else -np.pi / 20)
    arc = np.array([[x_o, y_o]]).T + r1 * np.array([np.cos(gammas), np.sin(gammas)])
    length = np.sqrt((y_goal - y_c) ** 2 + (x_goal - x_c) ** 2)
    line = np.array([np.linspace(x_c, x_goal, round(length / 0.5)), np.linspace(y_c, y_goal, round(length / 0.5))])
    theta_end = remap(np.arctan2(y_goal - y_c, x_goal - x_c)) + np.pi / 6
    thetas = np.arange(theta_end, theta_end + 2 * np.pi, np.pi / 15)
    circle = np.array([[x_goal, y_goal]]).T + r2 * np.array([np.cos(thetas), np.sin(thetas)])  # 20 points
    path = np.hstack([arc, line, circle])
    idx = arc.shape[1] + line.shape[1]
    return path, idx


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.e = 0
        self.e_sum = 0

    def output(self, e):
        de = e - self.e
        self.e_sum += e
        self.e = e
        return self.kp * e + self.ki * self.e_sum + self.kd * de


class USV:
    def __init__(self):
        rospy.init_node('usv', anonymous=True)

        pygame.init()
        pygame.joystick.init()
        joystick_connected = False

        if pygame.joystick.get_count() > 0:
            _ = pygame.joystick.Joystick(0)
            rospy.loginfo('joystick connected')
            joystick_connected = True
        else:
            rospy.loginfo('no joystick connected')

        self.mode = 'lock'
        mode_request = None
        rospy.loginfo('lock mode')

        button_mode_dict = {0: 'auto', 1: 'manual', 3: 'lock'}
        left_speed, right_speed = 0, 0
        self.control_rate = 10
        self.measure_rate = 50
        self.control_count = 0

        self.path = None
        self.circle_idx = None
        self.num_path_point = None
        self.target_idx = None
        self.closest_idx = None
        self.speed_controller = None
        self.steer_controller = None

        self.x_d = None
        self.y_d = None
        self.theta_d = None
        self.timestamp = rospy.get_time()

        self.preview = 10
        self.segment = 20  # segment < 30

        self.control_publisher = rospy.Publisher('control', Int32MultiArray, queue_size=1)
        self.path_publisher = rospy.Publisher('path', Path, queue_size=1)
        rospy.Subscriber('/position', Float32MultiArray, self.callback, queue_size=1)

        last_connect_request = rospy.get_time()
        last_publish = rospy.get_time()

        while not rospy.is_shutdown():
            # reconnect joystick
            if rospy.get_time() - last_connect_request > 5.0:
                if pygame.joystick.get_count() == 0 and joystick_connected:
                    rospy.loginfo('joystick disconnected')
                    joystick_connected = False

                if not joystick_connected:
                    pygame.joystick.init()
                    if pygame.joystick.get_count() > 0:
                        _ = pygame.joystick.Joystick(0)
                        joystick_connected = True
                        rospy.loginfo('joystick connected')
                last_connect_request = rospy.get_time()

            # read joystick input
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button in button_mode_dict.keys():
                        mode_request = button_mode_dict[event.button]

                if event.type == pygame.JOYAXISMOTION:
                    if event.axis == 5:  # LT: left motor speed
                        left_speed = int((event.value + 1) * 45)
                    if event.axis == 4:  # RT: right motor speed
                        right_speed = int((event.value + 1) * 45)

            if mode_request is not None:
                self.mode = mode_request
                rospy.loginfo(f'switch to {mode_request} mode')
                mode_request = None

            if rospy.get_time() - last_publish > 1 / self.control_rate:
                if self.mode == 'lock':
                    self.control_publisher.publish(Int32MultiArray(data=[1, 0, 0]))
                elif self.mode == 'manual':
                    self.control_publisher.publish(Int32MultiArray(data=[1, left_speed, right_speed]))
                last_publish = rospy.get_time()

    def callback(self, message):
        if self.mode == 'auto':
            x, y, theta, v_x, v_y, omega = message.data
            v = np.sqrt(v_x ** 2 + v_y ** 2)
            if self.path is None:
                x_goal, y_goal = 30, 30
                # x_goal, y_goal = map(eval, input('please enter a destination for usv: ').split())
                self.path, self.circle_idx = generate_path(x, y, theta, x_goal, y_goal)
                self.num_path_point = self.path.shape[1]
                self.closest_idx = 0
                self.target_idx = self.closest_idx + self.preview

                self.speed_controller = PIDController(15, 0, 0)  # TODO
                self.steer_controller = PIDController(20, 0, 0)  # TODO

                self.path_publisher.publish(Path(
                    x=[self.closest_idx, self.target_idx, *self.path[0, :].tolist()],
                    y=[self.closest_idx, self.target_idx, *self.path[1, :].tolist()]
                ))
            else:
                if self.closest_idx + self.segment > self.num_path_point - 1:
                    if self.closest_idx == self.num_path_point:
                        path_segment = self.path[:, self.circle_idx:self.circle_idx + self.segment]
                    else:
                        path_segment = np.hstack([
                            self.path[:, self.closest_idx:],
                            self.path[:, self.circle_idx:self.circle_idx + self.segment + self.closest_idx - self.num_path_point]
                        ])
                else:
                    path_segment = self.path[:, self.closest_idx:self.closest_idx + self.segment]
                distances = np.linalg.norm(np.array([[x, y]]).T - path_segment, axis=0)
                self.closest_idx += np.argmin(distances)
                if self.closest_idx > self.num_path_point - 1:
                    self.closest_idx += self.circle_idx - self.num_path_point
                self.target_idx = self.closest_idx + self.preview
                if self.target_idx > self.num_path_point - 1:
                    self.target_idx += self.circle_idx - self.num_path_point

                self.control_count += 1
                if self.control_count > self.measure_rate / self.control_rate:
                    self.control_count = 0
                    timestamp = rospy.get_time()
                    dt = timestamp - self.timestamp

                    x_d = self.path[0, self.target_idx]
                    y_d = self.path[1, self.target_idx]
                    u1 = 2 * np.tanh(x_d - x)  # TODO
                    u2 = 2 * np.tanh(y_d - y)  # TODO
                    u1 += (x_d - self.x_d) / dt if self.x_d is not None else 0
                    u2 += (y_d - self.y_d) / dt if self.y_d is not None else 0
                    v_d = np.sqrt(u1 ** 2 + u2 ** 2)
                    v_e = v_d - v

                    theta_d = np.arctan2(u2, u1)  # [-pi, pi]
                    s = remap(theta_d - theta)  # [-pi, pi]
                    omega_d = 0.9 * s + 0.1 * np.sign(s)  # TODO
                    omega_d += (theta_d - self.theta_d) / dt if self.theta_d is not None else 0
                    omega_e = omega_d - omega

                    base_speed = 45 + constraint(self.speed_controller.output(v_e), -45, 45)  # TODO
                    steer_speed = constraint(self.steer_controller.output(omega_e), -60, 60)  # TODO
                    left_speed = int(constraint(base_speed - steer_speed, 0, 90))
                    right_speed = int(constraint(base_speed + steer_speed, 0, 90))
                    self.control_publisher.publish(Int32MultiArray(data=[1, left_speed, right_speed]))

                    self.path_publisher.publish(Path(
                        x=[self.closest_idx, self.target_idx, *self.path[0, :].tolist()],
                        y=[self.closest_idx, self.target_idx, *self.path[1, :].tolist()]
                    ))

                    self.x_d = x_d
                    self.y_d = y_d
                    self.theta_d = theta_d
                    self.timestamp = timestamp
        else:
            self.path = None
            self.control_count += 1
            if self.control_count > self.measure_rate / self.control_rate:
                self.control_count = 0
                self.path_publisher.publish(Path())


if __name__ == '__main__':
    try:
        USV()
    except rospy.ROSInterruptException:
        pass
