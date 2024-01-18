# #! /usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# import time
# import math

# class BotController(Node):

#     def __init__(self):
#         super().__init__('bot_controller')
#         self.publisher = self.create_publisher(Twist, '/cmd_vel/bot1', 10)

#     def calculate_forces(self, angle, distance):
#         max_rpm = 90.0

#         x_rpm = (max_rpm * math.cos(math.radians(90 + angle))) + 90.0
#         y_rpm = (max_rpm * math.cos(math.radians(180 + angle))) + 90.0
#         z_rpm = (max_rpm * math.cos(math.radians(angle))) + 90.0

#         twist_msg = Twist()
#         twist_msg.linear.x = x_rpm
#         twist_msg.linear.y = y_rpm
#         twist_msg.linear.z = z_rpm

#         self.publisher.publish(twist_msg)

#         print(f"X: {x_rpm}, Y: {y_rpm}, Z: {z_rpm}")
#         time.sleep(distance * 2.5)

#         # Reset forces to zero
#         twist_msg.linear.x = 0.0
#         twist_msg.linear.y = 0.0
#         twist_msg.linear.z = 0.0

#         self.publisher.publish(twist_msg)

#         print("Setting forces to zero...")

#     def square(self, size):
#         self.calculate_forces(0, size)
#         time.sleep(1)

#         self.calculate_forces(90, size)
#         time.sleep(1)

#         self.calculate_forces(180, size)
#         time.sleep(1)

#         self.calculate_forces(270, size)
#         time.sleep(1)

#         print("Square made")

# def main(args=None):
#     rclpy.init(args=args)
#     bot = BotController()

#     while rclpy.ok():
#         bot.square(10)
#         rclpy.spin_once(bot)

#     bot.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class BotController(Node):

    def __init__(self):
        super().__init__('bot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel/bot3', 10)

    def rpm(self, x, y, z):

        twist_msg = Twist()
        twist_msg.linear.x = x
        twist_msg.linear.y = y
        twist_msg.linear.z = z

        self.publisher.publish(twist_msg)

        print(f"X: {x}, Y: {y}, Z: {z}")

    def square(self):
        self.rpm( 180.0, 20.0, 100.0)
        time.sleep(2)

        self.rpm( 160.0, 100.0, 0.0)
        time.sleep(2)

        self.rpm(0.0, 100.0, 140.0)
        time.sleep(2)

        print("Triangle made")
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