#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from std_msgs.msg import Bool

class StopService(Node):
    def __init__(self):

        super().__init__('stop_service')
        self.subscription_bot1 = self.create_subscription(Bool, '/bot1_reached', self.bot_1, 10)
        self.subscription_bot2 = self.create_subscription(Bool, '/bot2_reached', self.bot_2, 10)
        self.subscription_bot3 = self.create_subscription(Bool, '/bot3_reached', self.bot_3, 10)

        self.bool_msg_bot1 = False
        self.bool_msg_bot2 = False
        self.bool_msg_bot3 = False


    def make_service(self):
        self.srv = self.create_service(Empty, 'Stop_Flag', self.stop_flag_callback)
        print("Service active")

    def stop_flag_callback(self, request, response):
            self.get_logger().info('StopFlag service invoked successfully.')

    def bot_1(self, msg):
        self.bool_msg_bot1 = msg.data
        print(msg.data)

    def bot_2(self, msg):
        self.bool_msg_bot2 = msg.data

    def bot_3(self, msg):
        self.bool_msg_bot3 = msg.data
    
def main(args=None):
    rclpy.init(args=args)

    stop_service = StopService()

    while rclpy.ok():
        rclpy.spin_once(stop_service)
        
        if stop_service.bool_msg_bot1 and stop_service.bool_msg_bot2 and stop_service.bool_msg_bot3:
            stop_service.make_service()
    stop_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()