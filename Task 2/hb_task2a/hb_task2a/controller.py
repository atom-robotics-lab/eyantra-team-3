#! /usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Hologlyph Bots (HB) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 2A of Hologlyph Bots (HB) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''


# Team ID:		[ 2096 ]
# Author List: [CHAYAN KHETAN , SHLOK M SHARMA]		
# Filename:		controller.py
# Functions:
#			[ Comma separated list of functions in this file ]
# Nodes:		Add your publishing and subscribing node


################### IMPORT MODULES #######################

import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import PoseArray

from nav_msgs.msg import Odometry
import time
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.srv import NextGoal             

# You can add more if required
##############################################################

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


# Initialize Global variables

PI=3.14
wrench=Wrench()

################# ADD UTILITY FUNCTIONS HERE #################

##############################################################


# Define the HBController class, which is a ROS node
class HBController(Node):
    def __init__(self):
        super().__init__('hb_controller')
        
        # Initialze Publisher and Subscriber
        # NOTE: You are strictly NOT-ALLOWED to use "cmd_vel" or "odom" topics in this task
	    #	Use the below given topics to generate motion for the robot.
	    #   /hb_bot_1/left_wheel_force,
	    #   /hb_bot_1/right_wheel_force,
	    #   /hb_bot_1/left_wheel_force


        self.lw=self.create_publisher(Wrench, '/hb_bot_1/left_wheel_force', 10)
        self.rw=self.create_publisher(Wrench, '/hb_bot_1/right_wheel_force', 10)
        self.fw=self.create_publisher(Wrench, '/hb_bot_1/rear_wheel_force', 10)
        self.subs=self.create_subscription(Pose2D, '/detected_aruco', aruco_feedback_cb, 10)
    




        # For maintaining control loop rate.
        self.rate = self.create_rate(100)


        # client for the "next_goal" service
        self.cli = self.create_client(NextGoal, 'next_goal')      
        self.req = NextGoal.Request() 
        self.index = 0

    
    # Method to create a request to the "next_goal" service
    def send_request(self, request_goal):
        self.req.request_goal = request_goal
        self.future = self.cli.call_async(self.req)
        time.sleep(1)
        

    def inverse_kinematics(self, chasis_velocity, desired_angle):
        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 
        #	-> Use the target velocity you calculated for the robot in previous task, and
        #	Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
        #	Publish the calculated efforts to actuate robot by applying force vectors on provided topics
        ############################################   

        left_wheel_force_x = chasis_velocity * math.cos( math.radians(150) - desired_angle) # RED
        right_wheel_force_x = chasis_velocity * math.cos( math.radians(30) - desired_angle) #BLUE
        bottom_wheel_force_x = chasis_velocity * math.cos( math.radians(270) - desired_angle) #GREEN

        print("FORCE: ", left_wheel_force_x, " , ", right_wheel_force_x, " , ", bottom_wheel_force_x)

        wrench.force.y = round(left_wheel_force_x, 2)
        self.lw.publish(wrench)

        wrench.force.y = round(right_wheel_force_x, 2)
        self.rw.publish(wrench)

        wrench.force.y = round(bottom_wheel_force_x, 2)
        self.fw.publish(wrench)
        return

def main(args=None):
    rclpy.init(args=args)
    
    # Create an instance of the HBController class
    hb_controller = HBController()
   
    # Send an initial request with the index from ebot_controller.index
    hb_controller.send_request(hb_controller.index)

    

    # Main loop
    while rclpy.ok():

        # Check if the service call is done
        if hb_controller.future.done():
            try:
                # response from the service call
                response = hb_controller.future.result()
            except Exception as e:
                hb_controller.get_logger().infselfo('Service call failed %r' % (e,))
            else:
                #########           GOAL POSE             #########
                x_goal      = response.x_goal
                y_goal      = response.y_goal
                theta_goal  = response.theta_goal
                hb_controller.flag = response.end_of_list

                print("GOAL: ", x_goal, " , ", y_goal, " , ", theta_goal, "\n")
                print("CURRENT POSITION: ", hola_x, " , ", hola_y, " , ", hola_theta, "\n")
                ####################################################
               
                # Calculate Error from feedback

                hb_controller.err_x = x_goal - hola_x
                hb_controller.err_y = y_goal - hola_y
                hb_controller.err_theta = 0
                print("ERROR: ", hb_controller.err_x, " , ", hb_controller.err_y, " \n ")

                kp = 1.4

                # if( hb_controller.err_x <= 1.0 or hb_controller.err_y <= 1.0):
                #     kp = 15.0

                if( abs(hb_controller.err_x) <= 0.5 or abs(hb_controller.err_y) <= 0.5):
                    kp = 1.85
                
                vel_x = hb_controller.err_x * kp *1.05
                vel_y = hb_controller.err_y * kp
                # Change the frame by using Rotation Matrix (If you find it required)

                if(abs(hb_controller.err_x) <= 0.1 and abs(hb_controller.err_y) <= 0.1):
                    print("reached the required destination \n")
                    print("giving next coordinates \n")
                    hb_controller.err_x = 0
                    hb_controller.err_y = 0
                    vel_x = 0.0
                    vel_y = 0.0
                ############     DO NOT MODIFY THIS       #########
                    hb_controller.index += 1
                    if hb_controller.flag == 1 :
                        hb_controller.index = 0
                    hb_controller.send_request(hb_controller.index)

                # elif (abs(hb_controller.err_x) >= 0.5 and  abs(hb_controller.err_y) >= 0.5) :
                #     vel_x = hb_controller.err_x*kp 
                #     vel_y = hb_controller.err_y*kp
                
                # elif(abs(hb_controller.err_x) <= 0.5 and abs(hb_controller.err_y) >= 0.5):
                #     hb_controller.err_x = 0
                #     vel_y = hb_controller.err_y*kp
                #     #vel_x = 0.0
                # elif(abs(hb_controller.err_x) >= 0.5 and abs(hb_controller.err_y) <= 0.5) :
                #     hb_controller.err_y = 0
                #     vel_x = hb_controller.err_x*kp
                #     # vel_y = 0.0
 
                # Calculate the required velocity of bot for the next iteration(s)
                # Changing the angles.. reference 2012'

                chasis_velocity = math.sqrt(abs(vel_x*vel_x) + abs(vel_y*vel_y))
                chasis_velocity = abs(chasis_velocity) * 25

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

                #print(ds_ang)

                # if(vel_x >= 0.1):
                #     velocity_theta =  math.atan(vel_y / vel_x)
                # else:
                #     velocity_theta = math.radians(90)
                
                # if(vel_y < 0):
                #     desired_angle = math.radians(90) - velocity_theta
                # else :
                #     desired_angle = math.radians(360) - (math.radians(90) - velocity_theta) 

                desired_angle = ds_ang
                
                # Find the required force vectors for individual wheels from it.(Inverse Kinematics)
                hb_controller.inverse_kinematics(chasis_velocity, desired_angle)
                print("VELOCITY: ", chasis_velocity, " , ", "ANGLE: ", desired_angle, " \n ")     
                                
                # Modify the condition to Switch to Next goal (given position in pixels instead of meters) ## ISKO DEKHNA HAIIIIII
               
                  
                

                ####################################################

        # Spin once to process callbacks
        rclpy.spin_once(hb_controller)
    
    # Destroy the node and shut down ROS
    hb_controller.destroy_node()
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()
    