#!/usr/bin/env python
import rospy
import serial
from tools import constraint
from std_msgs.msg import Int32MultiArray


class Commander:
    def __init__(self):
        rospy.init_node('commander', anonymous=True)

        ros_node_name = rospy.get_name()
        port = rospy.get_param(ros_node_name + '/port')
        baud_rate = rospy.get_param(ros_node_name + '/baud_rate')

        self.serial = serial.Serial(port, baud_rate, timeout=1)
        self.serial.flushInput()
        self.serial.flushOutput()

        rospy.Subscriber('/control', Int32MultiArray, self.callback, queue_size=1)
        rospy.spin()

    def callback(self, message):
        left_output, right_output = message.data

        left_output = str(constraint(left_output, 0, 90))
        right_output = str(constraint(right_output, 0, 90))

        if len(left_output) == 1:
            left_output = '0' + left_output
        if len(right_output) == 1:
            right_output = '0' + right_output
        command = 'cmd' + left_output + right_output

        self.serial.write(command.encode())


if __name__ == '__main__':
    try:
        Commander()
    except rospy.ROSInterruptException:
        pass
