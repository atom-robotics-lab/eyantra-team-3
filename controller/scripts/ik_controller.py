#! /usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Hologlyph Bots (HB) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 2B of Hologlyph Bots (HB) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''


# Team ID:		[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		feedback.py
# Functions:
#			[ Comma separated list of functions in this file ]
# Nodes:		Add your publishing and subscribing node


################### IMPORT MODULES #######################

import rclpy
from rclpy.node import Node
import time
import math
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import PoseArray    






hola_theta=0
hola_x=0
hola_y = 0


                                                    # Initialize Global variables

PI=3.14
wrench=Wrench()
l=0.6

class HBController(Node):
    def __init__(self):
        super().__init__('hb_controller')
        

        # Initialise the required variables
        self.bot_1_x = []
        self.bot_1_y = []
        self.bot_1_theta = 0.0

        # Initialze Publisher and Subscriber
        # NOTE: You are strictly NOT-ALLOWED to use "cmd_vel" or "odom" topics in this task
	    #	Use the below given topics to generate motion for the robot.
	    #   /hb_bot_1/left_wheel_force,
	    #   /hb_bot_1/right_wheel_force,
	    #   /hb_bot_1/left_wheel_force

        #Similar to this you can create subscribers for hb_bot_2 and hb_bot_3









                                                                #Subscribers
        """self.subscription = self.create_subscription(
            Goal,  
            'hb_bot_1/goal',  
            self.goalCallBack,  # Callback function to handle received messages
            10  # QoS profile, here it's 10 which means a buffer size of 10 messages
        )  
        """
        self.subs=self.create_subscription(Pose2D, '/detected_aruco_1',self.aruco_feedback_cb, 10)




                                                                # Publishers 
        self.lw=self.create_publisher(Wrench, '/hb_bot_1/left_wheel_force', 10)
        self.rw=self.create_publisher(Wrench, '/hb_bot_1/right_wheel_force', 10)
        self.fw=self.create_publisher(Wrench, '/hb_bot_1/rear_wheel_force', 10)




                                                                # Prevent unused variable warning
        

         

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)

    def inverse_kinematics(self,vel):
        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 
        #	-> Use the target velocity you calculated for the robot in previous task, and
        #	Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
        #	Publish the calculated efforts to actuate robot by applying force vectors on provided topics
        ############################################
        #pass


        w=vel[2]

        left_wheel_force_x = -vel[0]*math.sin(math.radians(30)+hola_theta) - vel[1]*math.cos(math.radians(30) + hola_theta) + l*w
        right_wheel_force_x = -vel[0]*math.cos(math.radians(90) - hola_theta) + vel[1]*math.sin(math.radians(90) - hola_theta) + l*w
        bottom_wheel_force_x = vel[0] + l*w #GREEN

        print("FORCE: ", left_wheel_force_x, " , ", right_wheel_force_x, " , ", bottom_wheel_force_x)

        wrench.force.y = round(left_wheel_force_x, 2)
        self.lw.publish(wrench)

        wrench.force.y = round(right_wheel_force_x, 2)
        self.rw.publish(wrench)

        wrench.force.y = round(bottom_wheel_force_x, 2)
        self.fw.publish(wrench)
        #return 




    def aruco_feedback_cb(self,msg):
	

	
        global hola_x, hola_y, hola_theta
        hola_x = msg.x
        hola_y = msg.y
        hola_theta =  msg.theta
        #print("current position: ", hola_x , " ", hola_y, " ")

        print(" aruco _feedback is runnung /n")

    

    

    def goalCallBack(self, msg):
        self.bot_1_x= msg.x
        self.bot_1_y = msg.y
        self.bot_1_theta= msg.theta


        print("          goal_callback is running /n")



"""
        for waypoint_pose in msg.poses:
            self.bot_1_x.append(waypoint_pose.position.x)
            self.bot_1_y.append(waypoint_pose.position.y)

            orientation_q = waypoint_pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            theta_goal = euler_from_quaternion (orientation_list)[2]
            #theta_goals.append(theta_goal)
            self.bot_1_theta=theta_goal
"""

    


def main(args=None):
    rclpy.init(args=args)
    
    hb_controller = HBController()
    count = 0
    # Main loop
    while rclpy.ok():

        # list of poses is created and abhi usko iterate and get the poses
        # the no of items in list and run till the list is not complte 
        # count 
        
        
        
        #while (count!= len(hb_controller.bot_1_x)):
        print("len of array is :",len(hb_controller.bot_1_x))
        #print("goal is : ",hb_controller.bot_1_x[count],"  present position is :",hola_x)
        # main code
        # find err in x,y,theta
        hb_controller.err_x = 50.0 - hola_x#hb_controller.bot_1_x[count] - hola_x
        hb_controller.err_y = 50.0 - hola_y #hb_controller.bot_1_y[count] - hola_y
        hb_controller.err_theta = 3.14 - hola_theta#hb_controller.bot_1_theta - hola_theta
        print("ERROR: ", hb_controller.err_x, " , ", hb_controller.err_y, " \n "," ,",hb_controller.err_theta)
        print("present X:", hola_x,"    Y:",hola_y,"    THETA: ",hola_theta)
        


        kp = 1.2
        ka = 0.8

        if( hb_controller.err_x <= 1.0 or hb_controller.err_y <= 1.0):
            kp = 10.0
            

        if( abs(hb_controller.err_x) <= 3.0 or abs(hb_controller.err_y) <= 3.0):
            kp = 2.0
            ka = 1.2
        
        vel_x = hb_controller.err_x * kp * 0.105
        vel_y = hb_controller.err_y * kp * 0.1
        vel_theta = hb_controller.err_theta * ka
        # Change the frame by using Rotation Matrix (If you find it required)
        vel = [vel_x,vel_y,vel_theta]
        print("vel in X: ",vel_x,"  vel in  Y : ",vel_y,"    vel in THETA :",vel_theta)
        hb_controller.inverse_kinematics(vel)

        
        #elif (abs(hb_controller.err_x) >= 0.5 and  abs(hb_controller.err_y) >= 0.5) :
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



        if(abs(hb_controller.err_x) <= 1.0 and abs(hb_controller.err_y) <= 1.0 and abs(hb_controller.err_theta)<=1.0):
            print("reached the required destination \n")
            print("giving next coordinates \n")
            
            print("POINTS DONE:",count)
            
            hb_controller.err_x = 0.0
            hb_controller.err_y = 0.0
            hb_controller.err_theta = 0.0
            vel_x = 0.0
            vel_y = 0.0
            vel_theta = 0.0
            count=count+1

        """
        chasis_velocity = math.sqrt(abs(vel_x*vel_x) + abs(vel_y*vel_y))
        chasis_velocity = abs(chasis_velocity) * 35
        print(" chasis velocity :  ", chasis_velocity)
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
        """
        #print(ds_ang)

        # if(vel_x >= 0.1):
        #     velocity_theta =  math.atan(vel_y / vel_x)
        # else:
        #     velocity_theta = math.radians(90)
        
        # if(vel_y < 0):
        #     desired_angle = math.radians(90) - velocity_theta
        # else :
        #     desired_angle = math.radians(360) - (math.radians(90) - velocity_theta) 

        #desired_angle = ds_ang






















        # Spin once to process callbacks
        rclpy.spin_once(hb_controller)
    
    # Destroy the node and shut down ROS
    hb_controller.destroy_node()
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()