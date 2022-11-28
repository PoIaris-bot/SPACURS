import time
from digi.xbee.devices import XBeeDevice
from digi.xbee.exception import XBeeDeviceException, TransmitException


class XBeeSender:
    def __init__(self, port, baud_rate, remote_nodes):
        self.port = port
        self.baud_rate = baud_rate
        self.remote_nodes = remote_nodes

        print('+--------------------------------+')
        print('|  XBee Initialization started   |')
        print('+--------------------------------+')
        print('Please wait...')
        self.device = XBeeDevice(self.port, self.baud_rate)  # 主板对象
        self.end_devices = {}  # 连接上的子板

        try:
            self.device.open()
            self.xbee_network = self.device.get_network()
            for remote_node in self.remote_nodes:
                time.sleep(0.2)
                try:
                    end_device = self.xbee_network.discover_device(remote_node)  # 尝试连接子板
                    if end_device:
                        self.end_devices[remote_node] = end_device
                except ValueError:
                    pass
            if self.end_devices:
                print('End device', self.end_devices.keys(), 'are connected')
            else:
                print('No connected end device')
        except XBeeDeviceException:
            pass
        print('+--------------------------------+')
        print('|  XBee Initialization finished  |')
        print('+--------------------------------+')

    def send_to_all(self, message):
        if self.end_devices:
            succeeded_nodes = []
            failed_nodes = []
            for remote_node in self.end_devices.keys():
                try:
                    if self.end_devices[remote_node]:
                        self.device.send_data(self.end_devices[remote_node], message)
                        succeeded_nodes.append(remote_node)
                except TransmitException:
                    failed_nodes.append(remote_node)
                time.sleep(0.01)
            if succeeded_nodes:
                print('Succeeded in sending message', message, 'to end devices', succeeded_nodes)
            if failed_nodes:
                print('Failed to send message', message, 'to end devices', failed_nodes)
        else:
            print('No connected end device')

    def send_to_one(self, remote_node, message):
        if remote_node in self.end_devices.keys():
            try:
                if self.end_devices[remote_node]:
                    self.device.send_data(self.end_devices[remote_node], message)
                    print('Succeeded in sending message', message, 'to end device', remote_node)
            except TransmitException:
                print('Failed to send message', message, 'to end device', remote_node)
        else:
            print('End device', remote_node, 'is not connected')

    def __del__(self):
        if self.device is not None and self.device.is_open():
            self.device.close()
