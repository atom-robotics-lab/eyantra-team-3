#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class BotController(Node):

    def __init__(self):
        super().__init__('bot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel/bot1', 10)

    def rpm(self, x, y, z):

        twist_msg = Twist()
        twist_msg.linear.x = x
        twist_msg.linear.y = y
        twist_msg.linear.z = z

        self.publisher.publish(twist_msg)

        print(f"X: {x}, Y: {y}, Z: {z}")

    def inverse_kinematics(self, chasis_velocity, desired_angle):
        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 
        #	-> Use the target velocity you calculated for the robot in previous task, and
        #	Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
        #	Publish the calculated efforts to actuate robot by applying force vectors on provided topics
        ############################################   

        #desired_angle += math.radians(-60)
        left_wheel_force_x = chasis_velocity * math.cos( math.radians(150) - desired_angle) # RED
        right_wheel_force_x = chasis_velocity * math.cos( math.radians(30) - desired_angle) #BLUE
        bottom_wheel_force_x = chasis_velocity * math.cos( math.radians(270) - desired_angle) #GREEN

        print("FORCE: ", left_wheel_force_x, " , ", right_wheel_force_x, " , ", bottom_wheel_force_x)
        self.rpm(left_wheel_force_x, right_wheel_force_x, bottom_wheel_force_x)

def main(args=None):
    rclpy.init(args=args)
    bot = BotController()

    while rclpy.ok():
        bot.square()
        rclpy.spin_once(bot)

    bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()