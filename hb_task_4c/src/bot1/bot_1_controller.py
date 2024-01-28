#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
import time
import math


hola_theta = hola_x = hola_y = 0

def aruco_feedback_cb(msg):
	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Receive & store the feedback / coordinates found by aruco detection logic.
	#	-> This feedback plays the same role as the 'Odometry' did in the previous task.
    global hola_x, hola_y, hola_theta
    hola_x = msg.x
    hola_y = msg.y
    hola_theta = msg.theta
    print("current position: ", hola_x , " ", hola_y, " ")

	############################################


class BotController(Node):

    def __init__(self):
        super().__init__('bot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel/bot1', 10)
        self.bool_publsiher = self.create_publisher(Bool, '/pen1_down'10)
        self.subs=self.create_subscription(Pose2D, '/pen1_pose', aruco_feedback_cb, 10)

    def rpm(self, x, y, z):

        twist_msg = Twist()
        twist_msg.linear.x = x
        twist_msg.linear.y = y
        twist_msg.linear.z = z

        self.publisher.publish(twist_msg)

        print(f"X: {x}, Y: {y}, Z: {z}")

    def inverse_kinematics(self, chasis_velocity, desired_angle):
        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 
        #	-> Use the target velocity you calculated for the robot in previous task, and
        #	Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
        #	Publish the calculated efforts to actuate robot by applying force vectors on provided topics
        ############################################   

        #desired_angle += math.radians(-60)

        left_wheel_force_x = chasis_velocity * math.cos( math.radians(150) - desired_angle) # RED
        right_wheel_force_x = chasis_velocity * math.cos( math.radians(30) - desired_angle) #BLUE
        bottom_wheel_force_x = chasis_velocity * math.cos( math.radians(270) - desired_angle) #GREEN
        print("FORCE: ", left_wheel_force_x, " , ", right_wheel_force_x, " , ", bottom_wheel_force_x)

        left_wheel_force_x = (left_wheel_force_x/chasis_velocity)*90 + 90
        right_wheel_force_x = (right_wheel_force_x/chasis_velocity)*90 + 90
        bottom_wheel_force_x = (bottom_wheel_force_x/chasis_velocity)*90 + 90       
        self.rpm(left_wheel_force_x , right_wheel_force_x , bottom_wheel_force_x)
        self.bool_publsiher.publish(0)

def main(args=None):
    rclpy.init(args=args)
    bot = BotController()

    x_goal      = 400.0
    y_goal      = 400.0
    theta_goal  = 0.0

    print("GOAL: ", x_goal, " , ", y_goal, " , ", theta_goal, "\n")
    print("CURRENT POSITION: ", hola_x, " , ", hola_y, " , ", hola_theta, "\n")
    ####################################################
    
    # Calculate Error from feedback

    bot.err_x = x_goal - 200 #hola_x
    bot.err_y = y_goal - 200 #hola_y
    bot.err_theta = 0
    print("ERROR: ", bot.err_x, " , ", bot.err_y, " \n ")

    kp = 1.5

    # if( bot.err_x <= 1.0 or bot.err_y <= 1.0):
    #     kp = 15.0

    if( abs(bot.err_x) <= 5.0 and abs(bot.err_y) <= 5.0):
        avg_error =  (abs(bot.err_x) + abs(bot.err_y)) / 2.0
        if (avg_error != 0.0):
            kp = 6.9/(avg_error ** 1.8)

    # if( abs(bot.err_x) <= 1.0 or abs(bot.err_y) <= 1.0):
    #     kp = 2.8
    
    vel_x = bot.err_x * kp * 0.105 #/ 5
    vel_y = bot.err_y * kp * 0.1 #/ 5
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
    chasis_velocity = abs(chasis_velocity) * 30

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
    bot.inverse_kinematics(chasis_velocity, desired_angle)
    print("VELOCITY: ", chasis_velocity, " , ", "ANGLE: ", desired_angle, " \n ")     

    while rclpy.ok():
        # bot.square()
        rclpy.spin_once(bot)

    bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()