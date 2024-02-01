#! /usr/bin/env python3
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


                                                    # Initialize Global variables


hola_theta=0
hola_x=0
hola_y = 0
PI=3.14
values90_180 = list(range(90,181))
values0_90 = list(range(0,91))
l = 8.76   #distance from center to wheel
r = 1.9    # radius of wheel
class HBController(Node):
    def __init__(self):
        super().__init__('hb_controller')
        

        # Initialise the required variables
        # hexagon shape , triangle , rectangle
        self.bot_1_x = [200,175,125,100,125,175,200, 300,400,300,400, 200,400,400,200,200]
        self.bot_1_y = [150,200,200,150,100,100,150, 100,100,200,100, 300,300,400,400,300]
        self.bot_1_theta = 0.0

        
                                                                #Subscribers
                                                                #publisher
        

        self.subs=self.create_subscription(Pose2D, '/pen2_pose',self.aruco_feedback_cb, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel/bot2', 10)

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)






    def inverse_kinematics(self,vel,err_position,hola_theta):

        theta_val = hola_theta - 90
        
        w = theta_val/10  # omega of bot 

        #left_wheel_force_x = -vel[0]*math.sin(math.radians(30)+hola_theta) - vel[1]*math.cos(math.radians(30) + hola_theta) + l*w
        #right_wheel_force_x = -vel[0]*math.cos(math.radians(90) - hola_theta) + vel[1]*math.sin(math.radians(90) - hola_theta) + l*w
        #top_wheel_force_x = vel[0] + l*w #GREEN
        left_wheel_force_x = -l*w -math.cos(math.radians(60)+ math.radians(theta_val))*(vel[0]) + math.sin(math.radians(60)+math.radians(theta_val))*vel[1]
        right_wheel_force_x = -l*w -math.cos(math.radians(60) + (-1)*math.radians(theta_val))*(vel[0]) + (-math.sin(math.radians(60)+math.radians(theta_val))*vel[1])
        top_wheel_force_x = -l*w + math.cos(math.radians(theta_val))*vel[0] + math.sin(math.radians(theta_val))*vel[1]
        print("FORCE: Left:", left_wheel_force_x*0.52631, " , Right:", right_wheel_force_x*0.52631, " , Top:", top_wheel_force_x*0.52631)

        
        twist_msg = Twist()
        max_value = float(max(abs(err_position[0]),abs(err_position[1])))
        
        twist_msg.angular.x = max_value

        # calculation for wheels force 
        max_vel = (abs(vel[0])+abs(vel[1]))/2
        wheel_vel_y = ((left_wheel_force_x/r)/max_vel)*90 +90
        wheel_vel_x = ((right_wheel_force_x/r)/max_vel)*90 +90
        wheel_vel_z = ((top_wheel_force_x/r)/max_vel)*90 +90
        print("left_rpm:",wheel_vel_y,"right_rpm:",wheel_vel_x,"top_rpm:",wheel_vel_z)



        twist_msg.linear.y = wheel_vel_y
        twist_msg.linear.x = wheel_vel_x
        twist_msg.linear.z = wheel_vel_z
        # publish velocities
        self.publisher.publish(twist_msg)
    







    def aruco_feedback_cb(self,msg):
        global hola_x, hola_y, hola_theta
        hola_x = msg.x
        hola_y = msg.y
        hola_theta =  msg.theta
        #print("current position: ", hola_x , " ", hola_y, " ")

        print(" aruco _feedback is runnung /n")

    

    



def main(args=None):
    rclpy.init(args=args)
    
    hb_controller = HBController()
    count = 0
    # Main loop
    while rclpy.ok():

       
        print("len of array is :",len(hb_controller.bot_1_x))
        
        hb_controller.err_x = hb_controller.bot_1_x[count] - hola_x
        hb_controller.err_y = hb_controller.bot_1_y[count] - hola_y
        #hb_controller.err_theta = 3.14 - hola_theta#hb_controller.bot_1_theta - hola_theta
        print("ERROR: ", hb_controller.err_x, " , ", hb_controller.err_y, " \n "," ,")
        print("present X:", hola_x,"    Y:",hola_y,"    THETA: ",hola_theta)
        


        kp = 1
        ka = 0.8

        if( abs(hb_controller.err_x) <= 1.0 or abs(hb_controller.err_y) <= 1.0):
            kp = 0.5
            

        if( abs(hb_controller.err_x) <= 3.0 or abs(hb_controller.err_y) <= 3.0):
            kp = 0.7
            ka = 1.2
        
        vel_x = hb_controller.err_x * kp #* 0.105
        vel_y = hb_controller.err_y * kp #* 0.1
        
        err_position = [hb_controller.err_x,hb_controller.err_y]
        #print(err_position)
        vel = [vel_x,vel_y]
        print("vel in X: ",vel_x,"  vel in  Y : ",vel_y)
        hb_controller.inverse_kinematics(vel,err_position,hola_theta)

        

        # Calculate the required velocity of bot for the next iteration(s)
        # Changing the angles.. 



        #                                                  check if it reached or not

        if(abs(hb_controller.err_x) <= 1.0 and abs(hb_controller.err_y) <= 1.0):
            print("reached the required destination \n")
            print("giving next coordinates \n")
            
            print("POINTS DONE:",count)
            
            hb_controller.err_x = 0.0
            hb_controller.err_y = 0.0
            hb_controller.err_theta = 0.0
            vel_x = 0.0
            vel_y = 0.0
            count=count+1
            print("count value :",count)

        rclpy.spin_once(hb_controller)
    
    # Destroy the node and shut down ROS
    hb_controller.destroy_node()
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()