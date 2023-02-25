#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray
from digi.xbee.devices import XBeeDevice
from digi.xbee.exception import XBeeDeviceException, TransmitException


class XBee:
    def __init__(self):
        rospy.init_node('xbee', anonymous=True)

        node_name = rospy.get_name()
        port = rospy.get_param(node_name + '/port')
        baud_rate = rospy.get_param(node_name + '/baud_rate')
        node_id = str(rospy.get_param(node_name + '/node_id'))

        self.xbee_device = XBeeDevice(port, baud_rate)
        self.node = None

        try:
            self.xbee_device.open()
            xbee_network = self.xbee_device.get_network()
            try:
                self.node = xbee_network.discover_device(node_id)  # 尝试连接子板
            except ValueError:
                pass
        except XBeeDeviceException:
            pass

        rospy.Subscriber('/control', Int32MultiArray, self.callback, queue_size=1)
        rospy.spin()

    def callback(self, message):
        mode, left_speed, right_speed = map(str, message.data)
        left_speed = left_speed if len(left_speed) == 2 else '0' + left_speed
        right_speed = right_speed if len(right_speed) == 2 else '0' + right_speed
        command = mode + left_speed + right_speed
        try:
            if self.node:
                self.xbee_device.send_data(self.node, command)
        except TransmitException:
            pass


if __name__ == '__main__':
    try:
        XBee()
    except rospy.ROSInterruptException:
        pass
