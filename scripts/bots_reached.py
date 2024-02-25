#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from stop_flag.srv import Empty
from std_msgs.msg import Bool

class BotController(Node):
    
    def __init__(self):

        super().__init__('bot_controller')
        self.subscription_bot1 = self.create_subscription(Bool, '/bot1_reached', self.bot_1, 10)
        self.subscription_bot1 = self.create_subscription(Bool, '/bot2_reached', self.bot_2, 10)
        self.subscription_bot1 = self.create_subscription(Bool, '/bot3_reached', self.bot_3, 10)
        self.client_reached = self.create_client(srv_type = Empty, srv_name='Stop_Flag')
        self.bool_msg_bot1 = False
        self.bool_msg_bot2 = False
        self.bool_msg_bot3 = False

    def send_request(self, bot):
        request_bot = Empty.Request()
        request_bot.msg = bot
        future = self.client_docking.call_async(request_bot)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def bot_1(self, msg):
        self.bool_msg_bot1 = msg.data
    
    def bot_2(self, msg):
        self.bool_msg_bot2 = msg.data
    
    def bot_3(self, msg):
        self.bool_msg_bot3 = msg.data


def main(args=None):
    rclpy.init(args=args)
    bot = BotController()
    while rclpy.ok(): 
        if bot.bool_msg_bot1 and bot.bool_msg_bot2 and bot.bool_msg_bot3 :
            bot.send_request(False)
            break
        else : 
            rclpy.spin_once(bot)

if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from std_srvs.srv import Empty
# from geometry_msgs.msg import Pose2D



# class BotController(Node):

#     def __init__(self):
#         super().__init__('bot_controller')
#         self.bool_msg_bot1 = False
#         self.bool_msg_bot2 = False
#         self.bool_msg_bot3 = False

#         self.subscription_bot1 = self.create_subscription(Bool, '/bot1_reached', self.bot_1, 10)
#         self.subscription_bot2 = self.create_subscription(Bool, '/bot2_reached', self.bot_2, 10)
#         self.subscription_bot3 = self.create_subscription(Bool, '/bot3_reached', self.bot_3, 10)

#         self.srv = self.create_service(Empty, 'Stop_Flag', self.stop_flag_callback)

#         # Create a service client to call StopFlag service
#         self.stop_flag_client = self.create_client(Empty, 'stop_flag')
#         while not self.stop_flag_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('Service not available, waiting again...')

#     def bot_1(self, msg):
#         self.bool_msg_bot1 = True
#         self.check_and_call_service()

#     def bot_2(self, msg):
#         self.bool_msg_bot2 = True
#         self.check_and_call_service()

#     def bot_3(self, msg):
#         self.bool_msg_bot3 = True
#         self.check_and_call_service()

#     def check_and_call_service(self):
#         if self.bool_msg_bot1 and self.bool_msg_bot2 and self.bool_msg_bot3:
#             self.call_stop_flag_service()

#     def call_stop_flag_service(self):
#         request = Empty.Request()
#         future = self.stop_flag_client.call_async(request)
#         future.add_done_callback(self.stop_flag_callback)

#     def stop_flag_callback(self, future):
#         try:
#             response = future.result()
#             self.get_logger().info('StopFlag service invoked successfully.')
#         except Exception as e:
#             self.get_logger().error('Failed to invoke StopFlag service: %s' % e)

# def main(args=None):
#     rclpy.init(args=args)
#     bot = BotController()
#     rclpy.spin(bot)
#     bot.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()  