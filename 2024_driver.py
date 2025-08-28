import rclpy
from rclpy.node import Node
from serial import *
from std_msgs.msg import Int8
from std_msgs.msg import Int64MultiArray
from std_msgs.msg import Float64MultiArray


class MotorDriver(Node):
    def __init__(self, serPort):
        super().__init__('MotorDriver')

        # # Define the port of motor controller here with a parameter. 
        # self.declare_parameter('port', serPort)
        # # Get the value of port parameter
        self.serPort = serPort

        # Get the loop mode
        self.mode = "0"

        # Try to open the serial port and save it as self.ser
        try:
            self.ser = Serial(
                port = self.serPort,
                baudrate = 115200,
                parity = PARITY_NONE,
                stopbits = STOPBITS_ONE,
                bytesize = EIGHTBITS,
                timeout = 0.1
            )    
            self.ser.isOpen()
            print("Opened port", self.ser, "!")

        # If port is busy try closing it and opening it again
        except IOError:
            self.ser = Serial(
                port = self.serPort,
                baudrate = 115200,
                parity = PARITY_NONE,
                stopbits = STOPBITS_ONE,
                bytesize = EIGHTBITS,
                timeout = 0.1
            )
            print("Waiting for port to close")
            self.ser.close()
            self.ser.open()
            print("Opened port", self.ser, "!")

        # Subscribe to the thr topic. Data is Float array, of format [LeftThrottle, RightThrottle]
        self.thr_sub = self.create_subscription(
            Int64MultiArray,
            'thr',
            self.thr_callback,
            10
        )
        self.thr_sub

        # Subscribe to the estop topic. Data is Int8. 0 if disengage and 1 if engage the Estop.
        self.estop_sub = self.create_subscription(
            Int8,
            'estop',
            self.estop_callback,
            10
        )
        self.estop_sub

        self.pub_motorState = self.create_publisher(Int8,'motorState', 10)
        self.motorState = Int8()
        self.motorState.data = 1

        self.pub_hallspeed = self.create_publisher(Int64MultiArray,'hall_RPM', 10)
        self.hallspeed = Int64MultiArray()
        self.hallspeed.data = [0, 0]

        self.pub_amps = self.create_publisher(Float64MultiArray,'amps', 10)
        self.amps = Float64MultiArray()
        self.amps.data = [0.0, 0.0]

        self.send_mode(1)
        self.send_mode(2)
        self.save_conf()

        self.amps_counter = 1
        
    # Sends operating mode to the motor controller.
    def save_conf(self):

        # Format of the message to be sent is "!G <motor> <throttle>\r"
        x = f'%EESAV\r'

        # Write to serial
        self.ser.write(x.encode('utf-8'))

        echoFlag = 1
        # Controller sends an echo of every character received. We read this echo to confirm if controller works.
        if (self.mcSerialRead() != x[:-1]):
            self.pub_motorState.publish(self.motorState)

            self.get_logger().warn("EchoErr")
            echoFlag = 0


        ackFlag = 1
        # Read ack message from controller. It must be a + if command is accepted.
        if (self.mcSerialRead() != "+"):
            self.pub_motorState.publish(self.motorState)

            self.get_logger().warn("AckErr")
            ackFlag = 0

        if echoFlag and ackFlag :
            self.get_logger().info("Saved config to controller")

        print(x)

    # Function to read lines from the motor controller.
    # Motor controller sends lines terminated by \r alone. Due to this, we can't use ser.readlines() as that
    # waits for \n to terminate it's line.
    def mcSerialRead(self):
        x = ''
        line = ''

        while x != b'\r':
            # x is byte character. When converted to str it looks like b'<char>' (including quotes)
            # which is why we use slice 2:-1 to omit the b' and ' from both ends.
            line += str(x)[2:-1]

            x = self.ser.read()

        return line

    # Sends operating mode to the motor controller.
    def send_mode(self, motor_num):

        # Format of the message to be sent is "!G <motor> <throttle>\r"
        x = f'^MMOD {motor_num} {str(self.mode)}\r'

        # Write to serial
        self.ser.write(x.encode('utf-8'))

        echoFlag = 1
        # Controller sends an echo of every character received. We read this echo to confirm if controller works.
        if (self.mcSerialRead() != x[:-1]):
            self.pub_motorState.publish(self.motorState)

            self.get_logger().warn("EchoErr")
            echoFlag = 0

        ackFlag = 1
        # Read ack message from controller. It must be a + if command is accepted.
        if (self.mcSerialRead() != "+"):
            self.pub_motorState.publish(self.motorState)

            self.get_logger().warn("AckErr")
            ackFlag = 0

        if echoFlag and ackFlag :
            self.get_logger().info("Set mode to " + str(self.mode))
        
        print(x)

    # Sends setpoint to the motor controller.
    def send_setpoint(self, setpoint, motor_num):

        # Format of the message to be sent is "!G <motor> <throttle>\r"
        x = f'!G {motor_num} {str(setpoint)}\r'

        # Write to serial
        self.ser.write(x.encode('utf-8'))

        # Controller sends an echo of every character received. We read this echo to confirm if controller works.
        if (self.mcSerialRead() != x[:-1]):
            self.pub_motorState.publish(self.motorState)

            self.get_logger().warn("EchoErr")
        
        # Read ack message from controller. It must be a + if command is accepted.
        if (self.mcSerialRead() != "+"):
            self.pub_motorState.publish(self.motorState)

            self.get_logger().warn("AckErr")

    def hall_speed(self, motor_num):
        # Format of the message to be sent
        x = f'?BS {motor_num}\r'

        # Write to serial
        self.ser.write(x.encode('utf-8'))

        # Controller sends an echo of every character received. We read this echo to confirm if controller works.
        if (self.mcSerialRead() != x[:-1]):
            self.pub_motorState.publish(self.motorState)

            self.get_logger().warn("EchoErr")
        
        # Read response message from controller. It must be a + if command is accepted.
        return int(self.mcSerialRead()[3:].strip())
    
    def motor_amps(self):
        # Format of the message to be sent
        x = f'?A\r'

        # Write to serial
        self.ser.write(x.encode('utf-8'))

        # Controller sends an echo of every character received. We read this echo to confirm if controller works.
        if (self.mcSerialRead() != x[:-1]):
            self.pub_motorState.publish(self.motorState)

            self.get_logger().warn("EchoErr")
        
        # Read response message from controller. It must be a + if command is accepted.
        amps = self.mcSerialRead()[2:].strip()
        amps = amps.split(':')
        amps[0] = float(amps[0]) / 10.0
        amps[1] = float(amps[1]) / 10.0

        return amps
    
    # Callback directly sends received throttle data to serial.
    def thr_callback(self, thr):
        self.send_setpoint(thr.data[0], 1)
        self.send_setpoint(thr.data[1], 2)

        self.hallspeed.data = [self.hall_speed(1), self.hall_speed(2)]
        self.pub_hallspeed.publish(self.hallspeed)

        self.amps_counter += 1

        if self.amps_counter == 10:
            self.amps.data = self.motor_amps()
            self.pub_amps.publish(self.amps)
            self.amps_counter = 1

    def estop_callback(self, msg):
        print('omale')
        if msg.data == 1:

            # Write to serial - !EX\r is Engage Estop.
            self.ser.write(b"!EX\r")

            # Controller sends an echo of every character received. We read this echo to confirm if controller works.
            if (self.mcSerialRead() != "!EX"):
                self.pub_motorState.publish(self.motorState)

                self.get_logger().warn("EchoErr")
            
            # Read ack message from controller. It must be a + if command is accepted.
            if (self.mcSerialRead() != "+"):
                self.pub_motorState.publish(self.motorState)

                self.get_logger().warn("AckErr")

        elif msg.data == 0:

            # Write to serial - !MG\r is Disengage Estop.
            self.ser.write(b"!MG\r")

            # Controller sends an echo of every character received. We read this echo to confirm if controller works.
            if (self.mcSerialRead() != "!MG"):
                self.pub_motorState.publish(self.motorState)

                self.get_logger().warn("EchoErr")
            
            # Read ack message from controller. It must be a + if command is accepted.
            if (self.mcSerialRead() != "+"):
                self.pub_motorState.publish(self.motorState)

                self.get_logger().warn("AckErr")


def main(args=None):
    rclpy.init(args=args)

    motor_driver = MotorDriver(serPort="/dev/serial/by-id/usb-RoboteQ_RoboteQ_SBLG2360T_HQBLABlQMTlZMDIg_207D30763931-if00")

    rclpy.spin(motor_driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()