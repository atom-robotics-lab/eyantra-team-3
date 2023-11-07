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
from my_robot_interfaces.msg import Goal  
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import PoseArray        

#GLOBAL VARIABLES
hola_theta=0
hola_x=0
hola_y = 0
count = 0
PI=3.14
wrench=Wrench()
l=2.0

class HBController(Node):
    def __init__(self):
        super().__init__('hb_controller_2')
        

        # Initialise the required variables
        self.bot_1_x = []
        self.bot_1_y = []
        self.bot_1_theta = 0.0

        self.subscription = self.create_subscription(Goal,'/hb_bot_2/goal',self.goalCallBack, 10)    # Callback function to handle received message  # QoS profile, here it's 10 which means a buffer size of 10 messages
        
        self.subs=self.create_subscription(Pose2D, '/detected_aruco_2',self.aruco_feedback_cb, 10)

        self.lw=self.create_publisher(Wrench, '/hb_bot_2/left_wheel_force', 10)
        self.rw=self.create_publisher(Wrench, '/hb_bot_2/right_wheel_force', 10)
        self.fw=self.create_publisher(Wrench, '/hb_bot_2/rear_wheel_force', 10)

        # For maintaining control loop rate.
        #self.rate = self.create_rate(0.1)


    def goalCallBack(self, msg):
        self.bot_1_x= msg.x
        self.bot_1_y = msg.y
        self.bot_1_theta= msg.theta


    def inverse_kinematics(self,vel):
        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 
        #	-> Use the target velocity you calculated for the robot in previous task, and
        #	Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
        #	Publish the calculated efforts to actuate robot by applying force vectors on provided topics
        ############################################
        #pass
        # w=vel[2]


        # left_wheel_force_x = -vel[0]*math.sin(math.radians(30)) - vel[1]*math.cos(math.radians(30) ) + l*w
        # right_wheel_force_x = -vel[0]*math.cos(math.radians(60)) + vel[1]*math.sin(math.radians(60) ) + l*w
        # bottom_wheel_force_x = vel[0] + l*w #GREEN

        vel_x = vel[0]
        vel_y = vel[1]
        vel_w = vel[2]

        chasis_velocity = math.sqrt(abs(vel_x*vel_x) + abs(vel_y*vel_y))
        chasis_velocity = abs(chasis_velocity) #* 0.30

        x = vel_x
        y = vel_y

        if(abs(y)!= 0 and abs(x)!= 0):
            ang = math.atan(abs(y)/abs(x)) * 180/3.14

        else :
            ang = 0.0

        if(x<0 and y>0):  #2nd
            ds_ang = math.radians(105-ang)
            
        elif(x<0 and y<0):  #3rd
            ds_ang = math.radians(100+ang)
            
        elif(x>0 and y<0):  #4th
            ds_ang = math.radians(175+ang)
            
        else: #1st
            ds_ang = math.radians(245 + ang)

        
        desired_angle = ds_ang

        left_wheel_force_x = chasis_velocity * math.cos( math.radians(150) - desired_angle) * 0.40 # RED
        right_wheel_force_x = chasis_velocity * math.cos( math.radians(30) - desired_angle) * 0.40 #BLUE
        bottom_wheel_force_x = chasis_velocity * math.cos( math.radians(270) - desired_angle) * 0.40 #GREEN

        print("FORCE: ", left_wheel_force_x, " , ", right_wheel_force_x, " , ", bottom_wheel_force_x)

        wrench.force.y = left_wheel_force_x
        self.lw.publish(wrench)

        wrench.force.y = right_wheel_force_x
        self.rw.publish(wrench)

        wrench.force.y = bottom_wheel_force_x
        self.fw.publish(wrench)
        #return 

    def aruco_feedback_cb(self,msg):
	
        global hola_x, hola_y, hola_theta
        hola_x = msg.x
        hola_y = msg.y

        #Normalise
        hola_x = hola_x - 250.0
        hola_y = - (hola_y - 250.0)
        hola_theta =  msg.theta

def main(args=None):
    rclpy.init(args=args)
    
    hb_controller = HBController()
    global count
    hb_controller.l1=[] # self made 
    hb_controller.l2=[]

    for i in hb_controller.bot_1_x:
        hb_controller.l1.append(i)

    for j in hb_controller.bot_1_y:
        hb_controller.l2.append(j)

    hb_controller.bot_1_x = hb_controller.l1
    hb_controller.bot_1_y = hb_controller.l2

    # Main loop
    while rclpy.ok():
  
        if hb_controller.bot_1_x != []: 
            
            if (hb_controller.bot_1_theta > 1.47) :
                hb_controller.bot_1_theta = hb_controller.bot_1_theta - 1.47

            elif (hb_controller.bot_1_theta <= -1.47):
                hb_controller.bot_1_theta = hb_controller.bot_1_theta + 1.47


            # hb_controller.bot_1_x[count] = hb_controller.bot_1_x[count] - 250.0
            # hb_controller.bot_1_y[count] = 250.0 - hb_controller.bot_1_y[count]

            hb_controller.err_x = (hb_controller.bot_1_x[count] - 250.0) - hola_x                 
            hb_controller.err_y = (250.0 - hb_controller.bot_1_y[count]) - hola_y                                                
            hb_controller.err_theta = hb_controller.bot_1_theta - hola_theta      

            print( "GOAL: ", hb_controller.bot_1_x[count] - 250, " , ", hb_controller.bot_1_y[count] - 250, " , ", hb_controller.bot_1_theta, "\n" ) 
            print( "ERROR: ", hb_controller.err_x, " , ", hb_controller.err_y," ,",hb_controller.err_theta,"\n" )
            print( "CURRENT POSITION:", hola_x," , ", hola_y, " , ", hola_theta, "\n" )
            

            if hb_controller.err_x < 5.0 or hb_controller.err_y < 5.0:
                kp = 5.0
                ka = 7.0  
                vel_x = hb_controller.err_x * kp * 4.0    #    * 0.105
                vel_y = hb_controller.err_y * kp  * 4.0   # * 0.1
                vel_theta = hb_controller.err_theta * ka
                          
                
            else:
                kp =   4.0           
                ka =   3.5
                vel_x = hb_controller.err_x * kp     #    * 0.105
                vel_y = hb_controller.err_y * kp      # * 0.1
                vel_theta = hb_controller.err_theta * ka


            # vel_x = hb_controller.err_x * kp     #    * 0.105
            # vel_y = hb_controller.err_y * kp      # * 0.1
            # vel_theta = hb_controller.err_theta * ka
            vel = [vel_x, vel_y, vel_theta]

            print( "VELOCITY: ",vel_x, " , ",vel_y, " , ",vel_theta, "\n" )
            print ( " kp: ", kp , " ka: ", ka, "\n" )

            hb_controller.inverse_kinematics(vel)

            if( hb_controller.err_x <= 2.0 and hb_controller.err_y <= 2.0 and hb_controller.err_theta <= 2.0):
                # hb_controller.err_x = 0.0
                # hb_controller.err_y = 0.0
                # hb_controller.err_theta = 0.0
                # vel_x = 0.0
                # vel_y = 0.0
                # vel_theta = 0.0
                #if hb_controller.a ==  True:

                count = count+1
                if count == len(hb_controller.bot_1_x) :
                    break
            
            print( "COUNT : ", count, "REMAINING:", len(hb_controller.bot_1_x) - count, "\n" )

        # Spin once to process callbacks
       
        rclpy.spin_once(hb_controller)
    vel = [0,0,0]
    hb_controller.inverse_kinematics(vel)
    
    # Destroy the node and shut down ROS
    hb_controller.destroy_node()
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()