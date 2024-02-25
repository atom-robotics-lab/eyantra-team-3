#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class Test(Node):
    def __init__(self):
        super().__init__('test_publisher')

        self.bot_reached_publisher_1 = self.create_publisher(Bool, '/bot1_reached', 10)
        self.bot_reached_publisher_2 = self.create_publisher(Bool, '/bot2_reached', 10)
        self.bot_reached_publisher_3 = self.create_publisher(Bool, '/bot3_reached', 10)

def main(args=None):
    rclpy.init(args=args)
    bot = Test()
    bool_msg = Bool()
    bool_msg.data = True

    while rclpy.ok():  # Loop while ROS is okay
        bot.bot_reached_publisher_1.publish(bool_msg)
        bot.bot_reached_publisher_2.publish(bool_msg)
        bot.bot_reached_publisher_3.publish(bool_msg)

        print("called")
        
        # rclpy.spin(bot)  # Spin once to publish messages
    rclpy.shutdown()

if __name__ == '__main__':
    main()
