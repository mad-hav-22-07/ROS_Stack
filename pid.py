import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
import math

class Pid(Node):  
    def __init__(self):  
        super().__init__('pid') 
        
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.0
        
        self.prev_time = self.get_clock().now()
        self.prev_err = [0.0, 0.0]   
        self.integral = [0.0, 0.0] 
        self.radius = 5.0

        self.v = [0.0, 0.0]   
        self.rpm = [0.0, 0.0]
        
        self.subs1 = self.create_subscription(Float32MultiArray,'/cmd_vel_nav',self.cmd_callback,10)
        self.subs2 = self.create_subscription(Float32MultiArray,'rpm',self.rpm_callback,10)
        self.subs3 = self.create_subscription(Float32MultiArray,'keystroke',self.cmd_callback,10)
        self.publisher = self.create_publisher(Float32MultiArray, 'Thr', 10)
        
    def cmd_callback(self, msg):
        self.v[0] = (msg.data[0]) / (2 * math.pi * self.radius) * 60 # required rpms
        self.v[1] = (msg.data[1]) / (2 * math.pi * self.radius) * 60 
    
    def rpm_callback(self, msg):
        self.rpm = msg.data
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        self.prev_time = current_time

        if dt == 0:
            return	# Avoid division by zero
        
        output=[0.0,0.0]

        for i in range(2):
            error = self.v[i] - self.rpm[i]
            self.integral[i] += error * dt
            derivative = (error - self.prev_err[i]) / dt
                
            output[i] = self.kp * error + self.ki * self.integral[i] + self.kd * derivative

            self.prev_err[i] = error
            self.get_logger().info(f"Motor {i+1}: Error={error:.3f}, Output={output[i]:.3f}")
            
        
        out_msg = Float32MultiArray()
        out_msg.data = output
        self.publisher.publish(out_msg)
        

        
def main(args=None):
    rclpy.init(args=args)
    node = Pid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__== '__main__':
    main()
