#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custommsgs.msg import Rpm, Thr 
from geometry_msgs.msg import Twist
import math


class pid(Node):
    def __init__(self):
        super().__init__('pid')

        #vars n cons

        self.r =0.124
        self.l =0

        self.Kp = 1.0
        self.Ki = 0.2
        self.Kd = 0

        self.left_enc =0
        self.right_enc =0

        self.left_thr =0
        self.right_thr =0

        self.left_sp =0
        self.right_sp =0

        self.l_int =0
        self.r_int =0

        self.l_err_prev = 0
        self.r_err_prev = 0

        self.imax = 300/self.Ki

        self.prev_time = self.get_clock().now().nanoseconds/1e9

         
        #Subs
        self.cmd_sub = self.create_subscription(Twist,'/cmd_vel_nav',self.cmd_callback,10)
        self.enc_sub = self.create_subscription(Rpm,'rpm',self.rpm_callback,10)
        #self.cmd_sub = self.create_subscription

        #Pubs
        self.thr_pub = self.create_publisher(Thr,'/thr',10)

        #Tim
        self.timer = self.create_timer(0.1,self.control_loop)

    def cmd_callback(self,msg: Twist):
        v =msg.linear.x
        w = msg.angular.z

        v_l = v - (w*self.l/2)
        v_r = v + (w*self.l/2)

        self.left_sp = v_l*30/(math.pi*self.r)
        self.right_sp = v_r*30/(math.pi*self.r)

    def rpm_callback(self,msg: Rpm):
        self.left_enc = msg.left_enc
        self.right_enc = msg.right_enc

    def control_loop(self):

        now = self.get_clock().now().nanoseconds/1e9
        dt = now - self.prev_time
        self.prev_time = now
        err_l = self.left_sp- self.left_enc
        self.l_int += err_l
        #self.l_int = max(min(self.l_int, self.imax), -self.imax)



        deriv_l = (err_l -self.l_err_prev)/dt

        out_l = self.Kp*err_l + self.Ki*self.l_int + self.Kd*deriv_l

        self.l_err_prev = err_l

        err_r = self.right_sp- self.right_enc

        self.r_int += err_r



        deriv_r = (err_r -self.r_err_prev)/dt

        out_r = self.Kp*err_r + self.Ki*self.r_int + self.Kd*deriv_r

        self.r_err_prev = err_r

        msg = Thr()
        msg.left_thr = out_l
        msg.right_thr = out_r

        self.thr_pub.publish(msg)
        self.get_logger().info(f"SP: {self.left_sp:.2f}, {self.right_sp:.2f} | ENC: {self.left_enc:.2f}, {self.right_enc:.2f} | THR: {out_l:.2f}, {out_r:.2f}")


        



    




      

     
    

    




        





def main():
    rclpy.init()
    node =pid()
    rclpy.spin(pid)
    node.destroy_node()
    rclpy.shutdown()

