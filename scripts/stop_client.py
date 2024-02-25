#! /usr/bin/env python3
import sys
from std_srvs.srv import Empty
from std_msgs.msg import Bool
import rclpy
from rclpy.node import Node

class StopClientAsync(Node):

    def __init__(self):
        super().__init__('stop_client_async')
        self.cli = self.create_client(Empty, 'Stop_Flag')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        print("Connected")

        self.subscription_bot1 = self.create_subscription(Bool, '/bot1_reached', self.bot_1, 10)
        self.subscription_bot2 = self.create_subscription(Bool, '/bot2_reached', self.bot_2, 10)
        self.subscription_bot3 = self.create_subscription(Bool, '/bot3_reached', self.bot_3, 10)

        self.bool_msg_bot1 = False
        self.bool_msg_bot2 = False
        self.bool_msg_bot3 = False

        self.req = Empty.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)

    def bot_1(self, msg):
        self.bool_msg_bot1 = msg.data

    def bot_2(self, msg):
        self.bool_msg_bot2 = msg.data

    def bot_3(self, msg):
        self.bool_msg_bot3 = msg.data

def main(args=None):
    rclpy.init(args=args)

    stop_client = StopClientAsync()
    # stop_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(stop_client)
        
        if stop_client.bool_msg_bot1 and stop_client.bool_msg_bot2 and stop_client.bool_msg_bot3:
            
            stop_client.send_request()
            stop_client.get_logger().info("All conditions met. Sending service request.")
            break

    stop_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()