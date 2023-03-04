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
        left_dir, right_dir = 0, 0
        self.control_rate = 10
        self.measure_rate = 50
        self.control_count = 0

        self.path = None
        self.x_o = None
        self.y_o = None
        self.num_arc_point = None
        self.num_circle_point = None

        self.target_idx = None
        self.is_line = False

        self.r1 = 3
        self.r2 = 2

        self.length = 0.7
        self.Delta = 3 * self.length

        # self.x_goal, self.y_goal = 35, 30
        self.x_goal, self.y_goal = 0, 10
        # self.x_goal, self.y_goal = map(eval, input('please enter a destination for usv: ').split())

        self.steer_controller = None

        self.control_publisher = rospy.Publisher('control', Int32MultiArray, queue_size=1)
        self.path_publisher = rospy.Publisher('path', Path, queue_size=1)
        rospy.wait_for_service('save_plot')
        save_plot = rospy.ServiceProxy('save_plot', SetBool)
        rospy.Subscriber('/pose', Float32MultiArray, self.callback, queue_size=1)

        last_connect_request = rospy.get_time()
        last_publish = rospy.get_time()

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
                    if event.button == 6:  # LB
                        left_dir = not left_dir
                    if event.button == 7:  # RB
                        right_dir = not right_dir
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
                left_dir, right_dir = 0, 0
                mode_request = None

            if rospy.get_time() - last_publish > 1 / self.control_rate:
                if self.mode == 'lock':
                    self.control_publisher.publish(Int32MultiArray(data=[1, 0, 0]))
                elif self.mode == 'manual':
                    speed_mode = left_dir << 1 | right_dir + 1
                    self.control_publisher.publish(Int32MultiArray(data=[speed_mode, left_speed, right_speed]))
                last_publish = rospy.get_time()

    def callback(self, message):
        if self.mode == 'auto':
            x, y, theta = message.data[:3]
            if self.path is None:
                # generate path
                self.path, (self.x_o, self.y_o), (self.num_arc_point, self.num_circle_point) = generate_path(x, y, theta, self.x_goal, self.y_goal, self.r1, self.r2)
                self.target_idx = 1

                self.steer_controller = PIDController(80, 10, 500)  # TODO: best 80 10 500

                self.path_publisher.publish(Path(
                    x=[self.target_idx, *self.path[0, :].tolist()],
                    y=[self.target_idx, *self.path[1, :].tolist()]
                ))
            else:
                # calculate target point
                if self.target_idx < self.num_arc_point - 1:
                    distance = np.linalg.norm(np.array([[x, y]]).T - self.path[:, self.target_idx:self.num_arc_point], axis=0)
                    self.target_idx += np.argmin(distance)
                    base_speed = constraint(25 * self.r1, 0, 90)  # TODO
                    self.is_line = False

                    if self.target_idx == self.num_arc_point - 1:
                        self.target_idx = self.num_arc_point
                        base_speed = 90  # TODO
                        self.is_line = True
                elif self.target_idx == self.num_arc_point:
                    base_speed = 90  # TODO
                    self.is_line = True

                    if np.linalg.norm(np.array([[x, y]]).T - np.array([[self.x_goal, self.y_goal]]).T, axis=0) < self.r2 / 2:
                        self.target_idx += 1
                        base_speed = constraint(25 * self.r2, 0, 90)  # TODO
                        self.is_line = False
                else:
                    num_segment_point = self.num_circle_point // 6
                    if self.target_idx + num_segment_point > self.num_arc_point + self.num_circle_point:
                        path_segment = np.hstack([
                            self.path[:, self.target_idx:],
                            self.path[:, self.num_arc_point + 1:num_segment_point - self.num_circle_point + self.target_idx]
                        ])
                    else:
                        path_segment = self.path[:, self.target_idx:self.target_idx + num_segment_point]
                    distance = np.linalg.norm(np.array([[x, y]]).T - path_segment, axis=0)
                    self.target_idx += np.argmin(distance)
                    if self.target_idx > self.num_arc_point + self.num_circle_point:
                        self.target_idx -= self.num_circle_point
                    base_speed = constraint(25 * self.r2, 0, 90)  # TODO
                    self.is_line = False

                self.control_count += 1
                if self.control_count > self.measure_rate / self.control_rate:
                    self.control_count = 0

                    if self.is_line:
                        target = self.path[:, self.target_idx]
                        target_prev = self.path[:, self.target_idx - 1]
                        beta = np.arctan2(target[1] - target_prev[1], target[0] - target_prev[0])
                        alpha = remap(np.pi / 2 - beta)
                        error = np.sin(beta) * (x - target[0]) - np.cos(beta) * (y - target[1])
                        phi_d = remap(alpha + np.arctan(-error / (2 * self.Delta)))

                        # target = self.path[:, self.target_idx]
                        # phi_d = remap(np.pi / 2 - np.arctan2(target[1] - y, target[0] - x))

                        phi = remap(np.pi / 2 - theta)
                        phi_e = remap(phi_d - phi)
                    else:
                        target = self.path[:, self.target_idx]
                        if self.target_idx == self.num_arc_point + self.num_circle_point:
                            target_next = self.path[:, self.num_arc_point + 1]
                        else:
                            target_next = self.path[:, self.target_idx + 1]
                        alpha = np.arctan2(target_next[1] - target[1], target_next[0] - target[0])
                        x_los = target[0] + self.Delta * np.cos(alpha)
                        y_los = target[1] + self.Delta * np.sin(alpha)

                        phi_d = remap(np.pi / 2 - np.arctan2(y_los - y, x_los - x))
                        phi = remap(np.pi / 2 - theta)
                        phi_e = remap(phi_d - phi)

                    steer_speed = constraint(self.steer_controller.output(phi_e), -60, 60)  # TODO: best -60 60
                    left_speed = int(constraint(base_speed + steer_speed, 0, 90))
                    right_speed = int(constraint(base_speed - steer_speed, 0, 90))
                    self.control_publisher.publish(Int32MultiArray(data=[1, left_speed, right_speed]))

                    self.path_publisher.publish(Path(
                        x=[self.target_idx, *self.path[0, :].tolist()],
                        y=[self.target_idx, *self.path[1, :].tolist()]
                    ))
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
