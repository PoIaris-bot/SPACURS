#!/usr/bin/env python
import os
import rospy
import pygame
import numpy as np
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from std_srvs.srv import SetBool
from utils import generate_path, PIDController, remap, constraint
from spacurs.msg import Path


class USV:
    def __init__(self):
        rospy.init_node('usv', anonymous=True)

        pygame.init()
        pygame.joystick.init()
        joystick_connected = False

        if pygame.joystick.get_count() > 0:
            _ = pygame.joystick.Joystick(0)
            print('joystick connected')
            joystick_connected = True
        else:
            print('no joystick connected')

        self.mode = 'lock'
        mode_request = None
        print('lock mode')

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

        self.preview = 5
        self.segment = 10  # segment < 30

        self.control_publisher = rospy.Publisher('control', Int32MultiArray, queue_size=1)
        self.path_publisher = rospy.Publisher('path', Path, queue_size=1)
        rospy.wait_for_service('save_plot')
        save_plot = rospy.ServiceProxy('save_plot', SetBool)
        rospy.Subscriber('/pose', Float32MultiArray, self.callback, queue_size=1)

        last_connect_request = rospy.get_time()
        last_publish = rospy.get_time()

        self.cache = []

        while not rospy.is_shutdown():
            # reconnect joystick
            if rospy.get_time() - last_connect_request > 5.0:
                if pygame.joystick.get_count() == 0 and joystick_connected:
                    print('joystick disconnected')
                    joystick_connected = False

                if not joystick_connected:
                    pygame.joystick.init()
                    if pygame.joystick.get_count() > 0:
                        _ = pygame.joystick.Joystick(0)
                        joystick_connected = True
                        print('joystick connected')
                last_connect_request = rospy.get_time()

            # read joystick input
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button in button_mode_dict.keys():
                        mode_request = button_mode_dict[event.button]
                    if event.button == 4:  # Y
                        save_plot(True)
                    if event.button == 11:  # Menu
                        os.system('rosnode kill -a')
                        pid = os.popen('pgrep rosmaster').read().strip()
                        os.system('kill -9 %s' % pid)

                if event.type == pygame.JOYAXISMOTION:
                    if event.axis == 5:  # LT: left motor speed
                        left_speed = int((event.value + 1) * 45)
                    if event.axis == 4:  # RT: right motor speed
                        right_speed = int((event.value + 1) * 45)

            if mode_request is not None:
                self.mode = mode_request
                print(f'switch to {mode_request} mode')
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
                # x_goal, y_goal = 30, 30
                x_goal, y_goal = 10, 10
                # x_goal, y_goal = map(eval, input('please enter a destination for usv: ').split())
                self.path, self.circle_idx = generate_path(x, y, theta, x_goal, y_goal)
                self.num_path_point = self.path.shape[1]
                self.closest_idx = 0
                self.target_idx = self.closest_idx + self.preview

                self.speed_controller = PIDController(20, 0, 0)  # TODO
                self.steer_controller = PIDController(30, 0, 10)  # TODO

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
                            self.path[:,
                            self.circle_idx:self.circle_idx + self.segment + self.closest_idx - self.num_path_point]
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
                    omega_d = 0.2 * s + 0.0001 * np.sign(s)  # TODO
                    omega_d += (theta_d - self.theta_d) / dt if self.theta_d is not None else 0
                    omega_e = omega_d - omega

                    base_speed = constraint(45 + self.speed_controller.output(v_e), 0, 90)  # TODO
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
