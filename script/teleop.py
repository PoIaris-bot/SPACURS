#!/usr/bin/env python
import rospy
import pygame
from xbee import XBeeSender


def map_speed(value):
    speed = str(round((value + 1) * 10))
    if len(speed) == 1:
        speed = '0' + speed
    return speed


def map_mode(left_dir, right_dir):
    mode = left_dir << 1 | right_dir
    return str(mode + 1)


def teleop():
    rospy.init_node('teleop', anonymous=True)
    # XBee module
    node_name = rospy.get_name()
    port = rospy.get_param(node_name + '/port')
    baud_rate = rospy.get_param(node_name + '/baud_rate')
    nodes = eval(rospy.get_param(node_name + '/remote_nodes'))
    xbee_sender = None
    while xbee_sender is None:
        xbee_sender = XBeeSender(port, baud_rate, nodes)

    node_index = 0
    node_num = len(nodes)
    left_speed = right_speed = '00'
    left_dir = right_dir = 0
    control_rate = 20

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() > 0:
        _ = pygame.joystick.Joystick(0)
        print('Joystick connected!')
    else:
        print('Joystick not connected!')
        return

    done = False
    start = rospy.get_time()
    while not done:
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == 10:  # View: change node
                    node_index = (node_index + 1) % node_num
                if event.button == 11:  # Menu: exit
                    left_speed = right_speed = '00'
                    done = True
                if event.button == 6:  # LB: change left motor direction
                    left_dir = not left_dir
                if event.button == 7:  # RB: change right motor direction
                    right_dir = not right_dir
            if event.type == pygame.JOYAXISMOTION:
                if event.axis == 5:  # LT: left motor speed
                    left_speed = map_speed(event.value)
                if event.axis == 4:  # RT: right motor speed
                    right_speed = map_speed(event.value)
        end = rospy.get_time()
        if end - start > 1 / control_rate:
            command = map_mode(left_dir, right_dir) + left_speed + right_speed
            # print(nodes[node_index], command)
            xbee_sender.send_to_one(nodes[node_index], command)
            start = end
    pygame.quit()


if __name__ == '__main__':
    try:
        teleop()
    except rospy.ROSInterruptException:
        pass
