#!/usr/bin/env python
import os
import rospy
import pygame
from std_srvs.srv import SetBool
from std_msgs.msg import Int32MultiArray


def ground_console():
    rospy.init_node('console', anonymous=True)

    pygame.init()
    pygame.joystick.init()
    joystick_connected = False

    if pygame.joystick.get_count() > 0:
        _ = pygame.joystick.Joystick(0)
        print('joystick connected')
        joystick_connected = True
    else:
        print('no joystick connected')

    mode = 'lock'
    print('lock mode')

    button_mode_dict = {0: 'auto', 1: 'manual', 3: 'lock'}
    left_output, right_output = 0, 0

    control_rate = 10
    control_publisher = rospy.Publisher('control', Int32MultiArray, queue_size=1)

    rospy.wait_for_service('save_plot')
    save_plot = rospy.ServiceProxy('save_plot', SetBool)

    rospy.wait_for_service('switch_mode')
    switch_mode = rospy.ServiceProxy('switch_mode', SetBool)

    rospy.wait_for_service('open_plot')
    open_plot = rospy.ServiceProxy('open_plot', SetBool)
    open_plot_flag = False

    last_connect_request = rospy.get_time()
    last_control_publish = rospy.get_time()

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
                if event.button in button_mode_dict.keys():  # A: auto, B: manual, X: lock
                    mode = button_mode_dict[event.button]
                    switch_mode(True if mode == 'auto' else False)
                    print(f'switch to {mode} mode')
                    left_output, right_output = 0, 0

                    if mode == 'auto':
                        open_plot_flag = False
                        open_plot(open_plot_flag)

                if event.button == 4:  # Y: save plot
                    save_plot(True)

                if event.button == 10:  # View: open plot
                    open_plot_flag = not open_plot_flag
                    open_plot(open_plot_flag)

                if event.button == 11:  # Menu: shutdown
                    os.system('rosnode kill -a')
                    pid = os.popen('pgrep rosmaster').read().strip()
                    os.system('kill -9 %s' % pid)

            if event.type == pygame.JOYAXISMOTION:
                if event.axis == 5:  # LT: left motor
                    left_output = int((event.value + 1) * 45)

                if event.axis == 4:  # RT: right motor
                    right_output = int((event.value + 1) * 45)

        if rospy.get_time() - last_control_publish > 1 / control_rate:
            if mode == 'lock':
                control_publisher.publish(Int32MultiArray(data=[0, 0]))
            elif mode == 'manual':
                control_publisher.publish(Int32MultiArray(data=[left_output, right_output]))
            last_control_publish = rospy.get_time()


if __name__ == '__main__':
    try:
        ground_console()
    except rospy.ROSInterruptException:
        pass
