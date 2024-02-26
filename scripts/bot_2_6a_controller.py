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

        super().__init__('bot_controller_2')
        self.publisher = self.create_publisher(Twist, '/cmd_vel/bot2', 1)
        self.bool_publsiher = self.create_publisher(Bool, '/pen2_down',10)
        self.bot_reached_publsiher = self.create_publisher(Bool, '/bot2_reached',10)
        self.subscription = self.create_subscription(Pose2D, '/pen2_pose', self.aruco_feedback_cb, 10)
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
        #Green
        self.goals = [(304, 345),(315, 357),(325, 368),(335, 378),(345, 386),(354, 394),(363, 399),(372, 404),(380, 407),(387, 409),(394, 409),(399, 409),(403, 406),(407, 403),(409, 399),(409, 393),(409, 387),(407, 380),(404, 371),(399, 363),(393, 354),(386, 344),(377, 334),(367, 324),(356, 314),(344, 304),(331, 294),(316, 284),(301, 275),(286, 266),(269, 258),(252, 251),(235, 244),(218, 238),(200, 233),(183, 228),(166, 225),(150, 222),(134, 220),(118, 219),(104, 219),(91, 220),(78, 221),(67, 223),(57, 226),(49, 229),(42, 233),(36, 237),(32, 241),(30, 246),(30, 250),(31, 255),(33, 259),(38, 263),(43, 267),(51, 271),(60, 274),(70, 277),(81, 279),(94, 280),(108, 280),(122, 280),(138, 279),(154, 277),(171, 274),(188, 270),(205, 265),(222, 260),(240, 253),(257, 246),(274, 239),(290, 230),(305, 221),(320, 212),(334, 202),(347, 192),(359, 182),(370, 172),(380, 162),(388, 152),(395, 143),(400, 134),(405, 125),(408, 117),(409, 110),(409, 104),(408, 99),(406, 95),(402, 92),(398, 90),(392, 90),(385, 90),(378, 93),(370, 96),(361, 101),(352, 107),(342, 115),(332, 124),(322, 134),(312, 145),(302, 158)]
        
        return self.transform(self.goals[point])
        
    def transform(self,arr):
        x = arr[0]
        y = arr[1]

        x -= 250
        y *= -1
        y += 250

        return x,y

    
    def inverse_kinematics(self, vel_x, vel_y, omega, speed_factor, max_force):
        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 
        #	-> Use the target velocity you calculated for the robot in previous task, and
        #	Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
        #	Publish the calculated efforts to actuate robot by applying force vectors on provided topics
        ############################################   

        kp_t = 2.4
        kp_r = 1.5
        kp_l = 1.4

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
        # self.hola_y *-1

        self.hola_theta *= -1
        # if (self.hola_theta <= 0):
        #     -(self.hola_theta + math.radians(90))
        
        #     self.hola_theta - math.radians(90)
        print("current position: ", self.hola_x , " ", self.hola_y, " ", self.hola_theta)

        ############################################

def main(args=None):
    global kp
    rclpy.init(args=args)
    bot = BotController()
    point = 0
    bool_msg = Bool()  # Create a std_msgs.msg.Bool message object
    while rclpy.ok(): 
        
        threshold = 5.0
        theta_goal  = 0.0
        x_goal, y_goal = bot.get_next_pose(point)
        

        if(point > 0 and point < len(bot.goals)) :
            bool_msg.data = True  # Set its value to False
            bot.bool_publsiher.publish(bool_msg)  # Publish the message
            # time.sleep(5)


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
        speed_factor = 2.0
        
        if abs(bot.err_x) <= 30.0 or abs(bot.err_y) <= 30.0:
                speed_factor = 20.0

        max_force  = kp * 250 * math.sqrt(2)

        vel_x = bot.err_x * kp 
        vel_y = bot.err_y * kp
        omega = bot.err_theta * ka

        chasis_velocity = math.sqrt(abs(vel_x*vel_x) + abs(vel_y*vel_y))
        chasis_velocity = abs(chasis_velocity)

        bot.inverse_kinematics(vel_x, vel_y, omega, speed_factor, max_force)
        print("VELOCITY: ", chasis_velocity, " , ", "ANGLE: ",  omega)     

        rclpy.spin_once(bot)

    bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


