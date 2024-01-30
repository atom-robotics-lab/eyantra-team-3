#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
import time
import math


# hola_theta = hola_x = hola_y = 0

class BotController(Node):
    
    def __init__(self):

        super().__init__('bot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel/bot3', 10)
        self.bool_publsiher = self.create_publisher(Bool, '/pen3_down',10)
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


    def inverse_kinematics(self, vel_x, vel_y, chasis_velocity):
        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 
        #	-> Use the target velocity you calculated for the robot in previous task, and
        #	Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
        #	Publish the calculated efforts to actuate robot by applying force vectors on provided topics
        ############################################   

        #desired_angle += math.radians(-60)

        l = 8.76 
        w = 1
        radius_wheel = 1.9

        left_wheel_force_x = (-l*w -0.5 * (vel_x) + math.sin(math.radians(60)*vel_y)) / radius_wheel
        right_wheel_force_x = (-l*w -0.5*(vel_x) + (-math.sin(math.radians(60))*vel_x)) / radius_wheel
        bottom_wheel_force_x = (-l*w + vel_x) / radius_wheel

        print("FORCE: Left:", left_wheel_force_x, " , Right:", right_wheel_force_x, " , Top:", bottom_wheel_force_x)

        left_wheel_force_x = (left_wheel_force_x/chasis_velocity)*90 + 90
        right_wheel_force_x = (right_wheel_force_x/chasis_velocity)*90 + 90
        bottom_wheel_force_x = (bottom_wheel_force_x/chasis_velocity)*90 + 90  

        self.rpm(left_wheel_force_x,  bottom_wheel_force_x, right_wheel_force_x)
        # self.bool_publsiher.publish(0)

    def aruco_feedback_cb(self, msg):
	############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 
        #	-> Receive & store the feedback / coordinates found by aruco detection logic.
        #	-> This feedback plays the same role as the 'Odometry' did in the previous task.
        # global hola_x, hola_y, hola_theta
        self.hola_x = msg.x
        self.hola_y = msg.y
        self.hola_theta = msg.theta
        print("current position: ", self.hola_x , " ", self.hola_y, " ")

        ############################################

    def get_next_pose(point)
        #Blue
        goals = [(149, 379),(162, 395),(175, 399),(188, 392),(201, 374),(215, 345),(229, 310),(243, 270),(257, 228),(271, 188),(285, 152),(298, 124),(312, 106),(325, 100),(338, 104),(350, 120),(362, 147),(373, 181),(384, 220),(394, 262),(403, 303),(412, 340),(420, 369),(426, 390),(433, 399),(438, 397),(442, 383),(445, 359),(448, 326),(449, 288),(449, 246)]

        return goals[point][0],goals[point][1]

def main(args=None):
    rclpy.init(args=args)
    bot = BotController()

    while rclpy.ok(): 
        
        # x_goal      = 200.0
        # y_goal      = 150.0
        # theta_goal  = 0.0
        threshold = 2.0
        point = 0
        x_goal, y_goal = get_next_pose(point)

        print("GOAL: ", x_goal, " , ", y_goal, " , ", theta_goal, "\n")
        print("CURRENT POSITION: ", bot.hola_x, " , ", bot.hola_y, " , ", bot.hola_theta, "\n")
        ####################################################
        
        # Calculate Error from feedback

        bot.err_x = x_goal - bot.hola_x
        bot.err_y = y_goal - bot.hola_y
        bot.err_theta = 0
        print("ERROR: ", bot.err_x, " , ", bot.err_y, " \n ")

        if bot.err_x <= threshold and bot.err_y <= threshold:
            print(f"Reached point no.: {point}")
            point+=1
            get_next_pose(point)

        kp = 0.5

        # if( bot.err_x <= 1.0 or bot.err_y <= 1.0):
        #     kp = 15.0

        if( abs(bot.err_x) <= 5.0 and abs(bot.err_y) <= 5.0):
            avg_error =  (abs(bot.err_x) + abs(bot.err_y)) / 2.0
            if (avg_error != 0.0):
                kp = 6.9/(avg_error ** 1.8)

        # if( abs(bot.err_x) <= 1.0 or abs(bot.err_y) <= 1.0):
        #     kp = 2.8
        
        vel_x = bot.err_x * kp 
        vel_y = bot.err_y * kp
        # Change the frame by using Rotation Matrix (If you find it required)

        if(abs(bot.err_x) <= 1.0 and abs(bot.err_y) <= 1.0):
            print("reached the required destination \n")
            print("giving next coordinates \n")
            count+=1
            print("POINTS DONE:",count)
            
            bot.err_x = 0
            bot.err_y = 0
            vel_x = 0.0
            vel_y = 0.0
        ############     DO NOT MODIFY THIS       #########
            bot.index += 1
            if bot.flag == 1 :
                bot.index = 0
            bot.send_request(bot.index)

        chasis_velocity = math.sqrt(abs(vel_x*vel_x) + abs(vel_y*vel_y))
        chasis_velocity = abs(chasis_velocity) 

        x = vel_x
        y = vel_y

        if(abs(y)!= 0 and abs(x)!= 0):
            ang = math.atan(abs(y)/abs(x))* 180/3.14

        if(x<0 and y>0):  #2nd
            ds_ang = math.radians(105-ang)
            
        elif(x<0 and y<0):  #3rd
            ds_ang = math.radians(100+ang)
            
        elif(x>0 and y<0):  #4th
            ds_ang = math.radians(175+ang)
            
        else: #1st
            ds_ang = math.radians(248 + ang)

        desired_angle = ds_ang
        
        # Find the required force vectors for individual wheels from it.(Inverse Kinematics)
        bot.inverse_kinematics(vel_x, vel_y, chasis_velocity)
        print("VELOCITY: ", chasis_velocity, " , ", "ANGLE: ", desired_angle, " \n ")     

        # while rclpy.ok():
            # bot.square()
        rclpy.spin_once(bot)

    bot.rpm(90, 90, 90)
    bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()