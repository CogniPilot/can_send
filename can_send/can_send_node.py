#!/usr/bin/env python3
import rclpy
import numpy as np
import time
import can
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import Parameter, ParameterType, ParameterDescriptor
from can_msgs.msg import Frame

class CANSend(Node):
    def __init__(self):
        super().__init__('can_send_node')

        can_channel_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='CAN channel to send CAN frame on.')

        can_input_topic_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='can input topic name.')

        self.declare_parameter("can_input_topic", "/can0_send", 
            can_input_topic_descriptor)

        self.declare_parameter("can_channel", "can0", 
            can_channel_descriptor)

        self.CANChannel = self.get_parameter("can_channel").value
        self.CANSubTopic = self.get_parameter("can_input_topic").value

        self.bus = can.Bus(channel='{:s}'.format(self.CANChannel), interface='socketcan')
        self.CANSub = self.create_subscription(Frame, '{:s}'.format(self.CANSubTopic), self.TransmitMessageToSocketCAN, 20)

    def TransmitMessageToSocketCAN(self, msg):
        SocketCANMessage = can.Message(arbitration_id=msg.id, data=msg.data.tolist(), is_extended_id=msg.is_extended, dlc=msg.dlc, )
        self.bus.send(SocketCANMessage)

def main(args=None):
    rclpy.init()
    CNS = CANSend()
    rclpy.spin(CNS)
    CNS.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
