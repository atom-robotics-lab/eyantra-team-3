#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
import time
import math
import threading

class BotController(Node):
    
    def __init__(self, bot_name, pen_topic, bot_topic, bot_reached_topic, goals, aruco_topic):

        super().__init__(f'bot_controller_{bot_name}')
        self.publisher = self.create_publisher(Twist, f'/cmd_vel/bot{bot_name}', 10)
        self.bool_publisher = self.create_publisher(Bool, f'/pen{pen_topic}_down', 10)
        self.bot_reached_publisher = self.create_publisher(Bool, f'/bot{bot_name}_reached', 10)
        self.subscription = self.create_subscription(Pose2D, f'/pen{pen_topic}_pose', self.aruco_feedback_cb, 10)
        self.err_x = 0
        self.err_y = 0
        self.err_theta = 0
        self.hola_x = 0
        self.hola_y = 0
        self.hola_theta = 0
        self.bot_name = bot_name
        self.pen_topic = pen_topic
        self.bot_topic = bot_topic
        self.bot_reached_topic = bot_reached_topic
        self.goals = goals

    def rpm(self, x, y, z):
        twist_msg = Twist()
        twist_msg.linear.x = x
        twist_msg.linear.y = y
        twist_msg.linear.z = z
        self.publisher.publish(twist_msg)
        print(f"X: {x}, Y: {y}, Z: {z}")
        
    def transform(self, arr):
        x = arr[0]
        y = arr[1]
        x -= 250
        y *= -1
        y += 250
        return (x, y)

    def inverse_kinematics(self, vel_x, vel_y, omega, speed_factor, max_force):
        kp_t = 2.4
        kp_r = 1.5
        kp_l = 1.4

        top_wheel_force = ((0.66 * vel_x) + (0.33 * omega)) * kp_t * speed_factor
        right_wheel_force = ((-0.33 * vel_x) + (-0.58 * vel_y) + (0.33 * omega)) * kp_r * speed_factor
        left_wheel_force = ((-0.33 * vel_x) + (0.58 * vel_y) + (0.33 * omega)) * kp_l * speed_factor

        left_wheel_force = (left_wheel_force / max_force) * 90 + 90.5
        left_wheel_force = round(left_wheel_force) * 1.0

        right_wheel_force = (right_wheel_force / max_force) * 90 + 90.5
        right_wheel_force = round(right_wheel_force) * 1.0

        top_wheel_force = (top_wheel_force / max_force) * 90 + 90.5
        top_wheel_force = round(top_wheel_force) * 1.0

        self.rpm(top_wheel_force, right_wheel_force, left_wheel_force)

    def aruco_feedback_cb(self, msg):
        self.hola_x = msg.x
        self.hola_y = msg.y
        self.hola_theta = msg.theta - math.pi/2 - 0.4
        self.hola_x -= 250
        self.hola_y *= -1
        self.hola_y += 250
        self.hola_theta *= -1
        print("current position: ", self.hola_x, " ", self.hola_y, " ", self.hola_theta)

def bot_task(bot_name, pen_topic, bot_topic, bot_reached_topic, goals, aruco_topic):
    rclpy.init()
    bot = BotController(bot_name, pen_topic, bot_topic, bot_reached_topic, goals, aruco_topic)
    point = 0
    while rclpy.ok():
        threshold = 20.0
        theta_goal = 0.0
        x_goal, y_goal = bot.transform(bot.goals[point])
        distance = math.sqrt((x_goal - bot.hola_x) ** 2 + (y_goal - bot.hola_y) ** 2)
        print(f"Distance to goal {point}: {distance}")
        if distance > threshold:
            vel_x = (x_goal - bot.hola_x) / 100
            vel_y = (y_goal - bot.hola_y) / 100
            omega = bot.hola_theta - theta_goal
            if omega < -math.pi:
                omega += 2 * math.pi
            elif omega > math.pi:
                omega -= 2 * math.pi
            speed_factor = 1
            max_force = 0.5 * 250 * math.sqrt(2)
            bot.inverse_kinematics(vel_x, vel_y, omega, speed_factor, max_force)
        else:
            bot.bool_publisher.publish(True)
            bot.bot_reached_publisher.publish(True)
            print("I'm here")
            point += 1
            if point == len(bot.goals):
                point = 0
                print("going back to 0")
            time.sleep(5)
    rclpy.shutdown()

if __name__ == '__main__':
    bot1_thread = threading.Thread(target=bot_task, args=(1, 1, 1, 1, [(100, 100), (200, 200)], 'aruco_1'))
    bot2_thread = threading.Thread(target=bot_task, args=(2, 2, 2, 2, [(300, 300), (400, 400)], 'aruco_2'))
    bot3_thread = threading.Thread(target=bot_task, args=(3, 3, 3, 3, [(500, 500), (600, 600)], 'aruco_3'))
    
    bot1_thread.start()
    bot2_thread.start()
    bot3_thread.start()
    
    bot1_thread.join()
    bot2_thread.join()
    bot3_thread.join()
