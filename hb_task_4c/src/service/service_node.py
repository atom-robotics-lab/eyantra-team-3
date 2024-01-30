import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from my_robot_interfaces.srv import NextGoal
import numpy as np
import random
import time

# Define the color coordinates
red = [(450, 250),(449, 208),(448, 170),(445, 138),(442, 114),(437, 102),(432, 100),(426, 111),(419, 132),(411, 162),(402, 199),(393, 240),(383, 282),(372, 321),(361, 355),(349, 380),(337, 395),(324, 399),(311, 392),(297, 373),(283, 344),(270, 308),(256, 268),(242, 226),(228, 186),(214, 151),(200, 124),(187, 106),(174, 100),(161, 105),(149, 121)]
green = [(150, 120),(138, 145),(126, 179),(116, 219),(106, 260),(96, 301),(88, 338),(80, 368),(73, 389),(67, 399),(61, 397),(57, 384),(54, 360),(51, 328),(50, 289),(50, 248),(50, 206),(52, 168),(54, 137),(57, 114),(62, 101),(67, 101),(73, 111),(80, 133),(88, 164),(97, 201),(106, 242),(116, 284),(127, 323),(139, 356),(150, 381)]
blue = [(149, 379),(162, 395),(175, 399),(188, 392),(201, 374),(215, 345),(229, 310),(243, 270),(257, 228),(271, 188),(285, 152),(298, 124),(312, 106),(325, 100),(338, 104),(350, 120),(362, 147),(373, 181),(384, 220),(394, 262),(403, 303),(412, 340),(420, 369),(426, 390),(433, 399),(438, 397),(442, 383),(445, 359),(448, 326),(449, 288),(449, 246)]

class ServiceNode(Node):

    def __init__(self):
        super().__init__('service_node')
        self.service = self.create_service(
            NextGoal, 'next_goal', self.next_goal_callback)
        self.publish_color = self.create_publisher(String, '/color', 10)
        self.flag = 0
        self.logger_flag = 1
        self.count = 0

    def next_goal_callback(self, request, response):
        msg = String()

        color_request = request.request_goal

        if color_request < len(self.color_list):
            x, y = self.color_list[color_request]
            self.flag = 0
        else:
            self.flag = 1

        msg.data = self.color_name
        self.publish_color.publish(msg)

        response.x_goal = x
        response.y_goal = y
        response.theta_goal = 0.0
        response.end_of_list = self.flag

        if self.logger_flag == 1:
            self.get_logger().info("Service started...")
            self.logger_flag = 0
        time.sleep(1)
        return response

def main(args=None):
    rclpy.init(args=args)
    service_node = ServiceNode()

    # Select a random color
    colors = {
        "red": red,
        "green": green,
        "blue": blue
    }
    color_name = random.choice(list(colors.keys()))
    color_data = colors[color_name]

    service_node.color_name = color_name
    service_node.color_list = color_data

    rclpy.spin(service_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
