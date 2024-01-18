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

    def square(self):
        self.rpm( 90.0, 30.0, 180.0)
        time.sleep(2)

        self.rpm(180.0, 70.0, 40.0)
        time.sleep(2)

        self.rpm( 90.0, 150.0, 0.0)
        time.sleep(2)

        self.rpm(0.0, 110.0, 140.0)
        time.sleep(2)

        print("Square made")
        self.rpm(90.0, 90.0, 90.0)

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