# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int32
# from rclpy.node import Node 
# from geometry_msgs.msg import Twist
# import time


# class Publisher(Node):
#     def __init__(self):
#         super().__init__('Publisher_node')
       
#         self.publisher_1 = self.create_publisher(Twist , "/cmd_vel/bot2",10)
        
#         self.generate_square()
#         #self.i = 0
    
        
       
#     def generate_square(self):
#         self.s1=[160,90,0,90,90]
#         self.s2=[0,155,140,90,90]
#         self.s3=[90,40,120,90,90]
        
#         #if self.i<=len(self.s1):
#         msg = Twist()
#         msg.linear.x = float(160)
#         msg.linear.y = float(0)
#         msg.linear.z = float(90) 
#         self.publisher_1.publish(msg)
#         self.get_logger().info('Publishing: "%s"' % msg.linear.x)
#         self.get_logger().info('Publishing: "%s"' % msg.linear.y)
#         self.get_logger().info('Publishing: "%s"' % msg.linear.z)
#         time.sleep(2)
#         msg.linear.x = float(90)
#         msg.linear.y = float(155)
#         msg.linear.z = float(40) 
#         self.publisher_1.publish(msg)
#         self.get_logger().info('Publishing: "%s"' % msg.linear.x)
#         self.get_logger().info('Publishing: "%s"' % msg.linear.y)
#         self.get_logger().info('Publishing: "%s"' % msg.linear.z)
#         time.sleep(1.1)
#         msg.linear.x = float(0)
#         msg.linear.y = float(140)
#         msg.linear.z = float(120) 
#         self.publisher_1.publish(msg)
#         self.get_logger().info('Publishing: "%s"' % msg.linear.x)
#         self.get_logger().info('Publishing: "%s"' % msg.linear.y)
#         self.get_logger().info('Publishing: "%s"' % msg.linear.z)
#         time.sleep(1.7)
#         msg.linear.x = float(90)
#         msg.linear.y = float(90)
#         msg.linear.z = float(90) 
#         self.publisher_1.publish(msg)
#         self.get_logger().info('Publishing: "%s"' % msg.linear.x)
#         self.get_logger().info('Publishing: "%s"' % msg.linear.y)
#         self.get_logger().info('Publishing: "%s"' % msg.linear.z)
#         time.sleep(2000000000000)

#         self.get_logger().info('Publishing: "%s"' % msg.linear.x)
#         self.get_logger().info('Publishing: "%s"' % msg.linear.y)
#         self.get_logger().info('Publishing: "%s"' % msg.linear.z)
#         self.get_logger().info('Publishing: "%s"' % self.i)
        

#     #def generate_triangle(self):
#         #self.s1
            
        
        
# def main(args = None):
#     rclpy.init(args=args)
#     Publisher_node = Publisher()
#     rclpy.spin(Publisher_node)
#     Publisher_node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()


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
        self.publisher = self.create_publisher(Twist, '/cmd_vel/bot2', 10)

    def rpm(self, x, y, z):

        twist_msg = Twist()
        twist_msg.linear.x = x
        twist_msg.linear.y = y
        twist_msg.linear.z = z

        self.publisher.publish(twist_msg)

        print(f"X: {x}, Y: {y}, Z: {z}")

    def square(self):
        self.rpm( 160.0, 0.0, 90.0)
        time.sleep(2)

        self.rpm( 90.0, 155.0, 40.0)
        time.sleep(1.1)

        self.rpm(0.0, 140.0, 120.0)
        time.sleep(1.7)

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