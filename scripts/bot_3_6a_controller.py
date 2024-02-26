#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
import time
import math


# hola_theta = hola_x = hola_y = 0
max_force = 0
kp = 0

class BotController(Node):
    
    def __init__(self):

        super().__init__('bot_controller_3')
        self.publisher = self.create_publisher(Twist, '/cmd_vel/bot3', 10)
        self.bool_publsiher = self.create_publisher(Bool, '/pen3_down',10)
        self.bot_reached_publsiher = self.create_publisher(Bool, '/bot3_reached',10)
        self.subscription = self.create_subscription(Pose2D, '/pen3_pose', self.aruco_feedback_cb, 10)
        self.err_x = 0
        self.err_y = 0
        self.err_theta = 0
        self.hola_x = 0
        self.hola_y = 0
        self.hola_theta = 0
        

    def rpm(self, x, y, z):

        twist_msg = Twist()
        twist_msg.linear.x = x
        twist_msg.linear.y = y
        twist_msg.linear.z = z

        self.publisher.publish(twist_msg)

        print(f"X: {x}, Y: {y}, Z: {z}")

    
    def get_next_pose(self, point) :
        #Blue
        self.goals = [(305, 154),(295, 167),(285, 182),(276, 197),(267, 212),(259, 229),(251, 246),(244, 263),(238, 280),(233, 297),(228, 315),(225, 332),(222, 348),(220, 364),(219, 380),(219, 394),(219, 408),(221, 420),(223, 431),(225, 441),(229, 450),(232, 457),(236, 462),(241, 466),(245, 469),(250, 469),(254, 469),(259, 466),(263, 462),(267, 456),(271, 449),(274, 440),(276, 430),(278, 418),(280, 406),(280, 392),(280, 378),(279, 362),(277, 346),(274, 329),(270, 312),(265, 295),(260, 278),(254, 260),(247, 243),(239, 227),(231, 210),(222, 195),(213, 180),(203, 166),(193, 153),(183, 141),(173, 130),(163, 120),(153, 112),(144, 105),(134, 99),(126, 94),(118, 92),(111, 90),(105, 90),(99, 91),(95, 93),(92, 96),(90, 101),(90, 107),(90, 113),(92, 120),(96, 129),(101, 137),(107, 147),(114, 156),(123, 166),(133, 176),(144, 186),(157, 196),(170, 206),(185, 216),(200, 225),(216, 234),(232, 242),(249, 249),(266, 256),(283, 262),(301, 267),(318, 271),(335, 275),(351, 277),(367, 279),(383, 280),(397, 280),(410, 279),(422, 278),(433, 276),(443, 273),(451, 270),(458, 266),(463, 262),(467, 257),(469, 253),(469, 248)]
        return self.transform(self.goals[point])
    
    def transform(self,arr):
        x = arr[0]
        y = arr[1]

        x -= 250

        y *= -1
        y += 250

        return (x,y)

    
    def inverse_kinematics(self, vel_x, vel_y, omega, speed_factor, max_force):
        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 
        #	-> Use the target velocity you calculated for the robot in previous task, and
        #	Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
        #	Publish the calculated efforts to actuate robot by applying force vectors on provided topics
        ############################################   

        kp_t = 2.0
        kp_r = 0.85
        kp_l = 1.0

        top_wheel_force = ((0.66 * vel_x) + (0.33 * omega) ) * kp_t * speed_factor

        right_wheel_force = ((-0.33 * vel_x) + (-0.58 * vel_y) + (0.33 * omega)) * kp_r * speed_factor

        left_wheel_force = ((-0.33 * vel_x) + (0.58 * vel_y) + (0.33 * omega)) * kp_l * speed_factor

        print("force: ",top_wheel_force, ", ", right_wheel_force, ", " , left_wheel_force )

        left_wheel_force = (left_wheel_force/max_force)*90 + 90.5
        left_wheel_force = round(left_wheel_force) * 1.0

        right_wheel_force = (right_wheel_force/max_force)*90 + 90.5
        right_wheel_force = round(right_wheel_force) * 1.0

        top_wheel_force = (top_wheel_force/max_force)*90 + 90.5
        top_wheel_force = round(top_wheel_force) * 1.0

        self.rpm( top_wheel_force, right_wheel_force, left_wheel_force)
    

    def aruco_feedback_cb(self, msg):
	############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 
        #	-> Receive & store the feedback / coordinates found by aruco detection logic.
        #	-> This feedback plays the same role as the 'Odometry' did in the previous task.
        # global hola_x, hola_y, hola_theta
        self.hola_x = msg.x
        self.hola_y = msg.y
        self.hola_theta = msg.theta #- math.pi/2 - 0.4

        self.hola_x -=250
        self.hola_y *=-1
        self.hola_y +=250

        self.hola_theta *= -1

        print("current position: ", self.hola_x , " ", self.hola_y, " ", self.hola_theta)

        ############################################

def main(args=None):
    global kp
    rclpy.init(args=args)
    bot = BotController()
    point = 0
    while rclpy.ok(): 
        
        threshold = 5.0
        theta_goal  = 0.0
        x_goal, y_goal = bot.get_next_pose(point)
        
        bool_msg = Bool()  # Create a std_msgs.msg.Bool message object

        if(point > 0 and point < len(bot.goals)) :
            bool_msg.data = True  # Set its value to False
            bot.bool_publsiher.publish(bool_msg)  # Publish the message

        bool_msg.data = False
        bot.bot_reached_publsiher.publish(bool_msg)

        print("GOAL: ", x_goal, " , ", y_goal, " , ", theta_goal, "\n")
        print("CURRENT POSITION: ", bot.hola_x, " , ", bot.hola_y, " , ", bot.hola_theta, "\n")
        ####################################################
        
        # Calculate Error from feedback

        bot.err_x = x_goal - bot.hola_x
        bot.err_y = y_goal - bot.hola_y
        bot.err_theta = theta_goal - bot.hola_theta
        print("ERROR: ", bot.err_x, " , ", bot.err_y, " , " , bot.err_theta, " \n ")

        if abs(bot.err_x) <= threshold and abs(bot.err_y) <= threshold:

            print(f"Reached point no.: {point}")
            point+=1
            bot.get_next_pose(point)
            if point == len(bot.goals) - 1:
                bool_msg.data = False
                bot.bool_publsiher.publish(bool_msg)

                bool_msg.data = True
                bot.bot_reached_publsiher.publish(bool_msg)
                print("Bot Movement Stopped")
                bot.inverse_kinematics(0, 0, 0, 0, 1.0)
                bool_msg.data = False
                bot.bool_publsiher.publish(bool_msg)
                break
            continue

        kp = 10.0
        ka = 1000.0
        speed_factor = 1.0
        
        if abs(bot.err_x) <= 20.0 or abs(bot.err_y) <= 20.0:
                speed_factor = 20.0

        max_force  = kp * 250 * math.sqrt(2)

        
        vel_x = bot.err_x * kp 
        vel_y = bot.err_y * kp
        omega = bot.err_theta * ka

        # Change the frame by using Rotation Matrix (If you find it required)

        chasis_velocity = math.sqrt(abs(vel_x*vel_x) + abs(vel_y*vel_y))
        chasis_velocity = abs(chasis_velocity)
 

        # Find the required force vectors for individual wheels from it.(Inverse Kinematics)
        bot.inverse_kinematics(vel_x, vel_y, omega, speed_factor, max_force)
        print("VELOCITY: ", chasis_velocity, " , ", "ANGLE: ",  omega)     

        rclpy.spin_once(bot)

    bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

