#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import sys,termios,tty,select

class key_board_node(Node):
    
    def __init__(self):
        super().__init__("keyboard_control")
        self.key_stroke = self.create_publisher(Float32MultiArray,"keystroke",10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.timer = self.create_timer(0.1,self.key_loop)
    
    def key_loop(self):
        msg = Float32MultiArray()
        msg.data = [0.0, 0.0]

        key = self.getkey()
        if key is None:
            return  # no key pressed 

        if key == "w":
            msg.data = [10.0, 10.0]
        elif key == "a":
            msg.data = [10.0, 0.0]
        elif key == "s":
            msg.data = [-10.0, -10.0]
        elif key == "d":
            msg.data = [0.0, 10.0]
        elif key == "q":
            self.get_logger().info("Quit key pressed. Shutting down...")
            rclpy.shutdown()
            return
        else:
            self.get_logger().info(f"Unknown key pressed: {key}")
            return

        # only publish if valid key
        self.key_stroke.publish(msg)
        self.get_logger().info(f"Sent {msg.data}")
            
    def getkey(self):
        tty.setraw(sys.stdin.fileno())
        rlist,_,_ = select.select([sys.stdin],[],[],0.1)
        key = sys.stdin.read(1) if rlist else None
        termios.tcsetattr(sys.stdin,termios.TCSADRAIN,self.settings)
        return key


def main(args=None):
    rclpy.init(args=args)
    node = key_board_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Exiting")
    finally:
        rclpy.shutdown()

if __name__ =="__main__":
    main()
