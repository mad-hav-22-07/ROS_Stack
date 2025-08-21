#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class encoder(Node):
    def __init__(self):
        super().__init__("encoder")
        self.publisher=self.create_publisher(Float32, 'rpm', 10)
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.timer = self.create_timer(0.25, self.serial_data)

    
    def serial_data(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            int_line = float(line)
            msg = Float32()
            msg.data = int_line
            self.publisher.publish(msg)
            self.get_logger().info(f"Publishing: {line}")  

def main(args=None):
    rclpy.init(args=args)
    node = encoder()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()





