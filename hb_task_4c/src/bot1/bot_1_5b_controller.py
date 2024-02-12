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

        super().__init__('bot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel/bot1', 10)
        self.bool_publsiher = self.create_publisher(Bool, '/pen1_down',10)
        self.subscription = self.create_subscription(Pose2D, '/pen1_pose', self.aruco_feedback_cb, 10)
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


    def servo_map(self, current_force, max_force= 0.5 * 250 * math.sqrt(2), servo_min=90, servo_max=180, min_force= - kp * 250 * math.sqrt(2)):
        val = int((current_force - min_force) * (servo_max - servo_min) / (max_force - min_force) + servo_min)
        if current_force < 0 :
            val =  abs(90 - val)
            val = 90- val
        return val
    
    def get_next_pose(self, point) :
        #Red
        self.goals = [(450, 250),(449, 208),(448, 170),(445, 138),(442, 114),(437, 102),(432, 100),(426, 111),(419, 132),(411, 162),(402, 199),(393, 240),(383, 282),(372, 321),(361, 355),(349, 380),(337, 395),(324, 399),(311, 392),(297, 373),(283, 344),(270, 308),(256, 268),(242, 226),(228, 186),(214, 151),(200, 124),(187, 106),(174, 100),(161, 105),(149, 121)]
        for i in range(len(self.goals)):
            self.goals[i] = self.transform(self.goals[i][0],self.goals[i][1])
            
        return self.goals[point][0], self.goals[point][1]
    
    def transform(self,x,y):
        x -= 250

        y *= -1
        y += 250

        return x,y
    
    def quadrant_shift(self, x, y):
        if(x>0 and y>0):   #1st
            return -x,y
        if(x<0 and y>0):   #2nd
            return  x,-y
        elif(x<0 and y<0): #3rd
            return -x,y
        else:              #4th
            return x,-y

    
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

        # kp_t = 1.0
        # kp_r = 1.23
        # kp_l = 3.2

        # if (vel_x < 0 and vel_y < 0 ):
        #     kp_t = 2.0
        #     kp_r = 0.85
        #     kp_l = 1.1

        top_wheel_force = ((0.66 * vel_x) + (0.33 * omega) ) * kp_t * speed_factor
        # top_wheel_force = self.servo_map(top_wheel_force) * 1.0 # to convert into float value
        
        right_wheel_force = ((-0.33 * vel_x) + (-0.58 * vel_y) + (0.33 * omega)) * kp_r * speed_factor
        # right_wheel_force = self.servo_map(right_wheel_force) * 1.0

        left_wheel_force = ((-0.33 * vel_x) + (0.58 * vel_y) + (0.33 * omega)) * kp_l * speed_factor
        # left_wheel_force = self.servo_map(left_wheel_force) * 1.0 

        print("force: ",top_wheel_force, ", ", right_wheel_force, ", " , left_wheel_force )

        left_wheel_force = (left_wheel_force/max_force)*90 + 90.5
        left_wheel_force = round(left_wheel_force) * 1.0

        right_wheel_force = (right_wheel_force/max_force)*90 + 90.5
        right_wheel_force = round(right_wheel_force) * 1.0
        #tuning ;
        # if (right_wheel_force < 90):
        #     right_wheel_force += 30
        # elif(right_wheel_force > 90) :
        #     right_wheel_force = right_wheel_force - 30
        
        top_wheel_force = (top_wheel_force/max_force)*90 + 90.5
        top_wheel_force = round(top_wheel_force) * 1.0

        self.rpm( top_wheel_force, right_wheel_force, left_wheel_force)
        # self.bool_publsiher.publish(0)
    

    def aruco_feedback_cb(self, msg):
	############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 
        #	-> Receive & store the feedback / coordinates found by aruco detection logic.
        #	-> This feedback plays the same role as the 'Odometry' did in the previous task.
        # global hola_x, hola_y, hola_theta
        self.hola_x = msg.x
        self.hola_y = msg.y
        self.hola_theta = msg.theta + math.pi/2

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
        
        threshold = 10.0
        theta_goal  = 0.0 
        x_goal, y_goal = bot.get_next_pose(point)
        x_goal, y_goal = bot.quadrant_shift(x_goal, y_goal)
        
        bool_msg.data = True  # Set its value to False
        bot.bool_publsiher.publish(bool_msg)  # Publish the message
        
        # time.sleep(1)
        
        # bool_msg.data = True  # Update the value to True
        # bot.bool_publsiher.publish(bool_msg)  # Publish the updated message
        

        print("GOAL: ", x_goal, " , ", y_goal, " , ", theta_goal, "\n")
        print("CURRENT POSITION: ", bot.hola_x, " , ", bot.hola_y, " , ", bot.hola_theta, "\n")
        ####################################################
        
        # Calculate Error from feedback

        bot.err_x = x_goal - bot.hola_x
        bot.err_y = y_goal - bot.hola_y
        bot.err_theta = theta_goal - bot.hola_theta
        print("ERROR: ", bot.err_x, " , ", bot.err_y, " , " , bot.err_theta, " \n ")

        if abs(bot.err_x) <= threshold and abs(bot.err_y) <= threshold:
            # print("reached")
            # bot.rpm(90.0, 90.0, 90.0)
            # break
            print(f"Reached point no.: {point}")
            point+=1
            bot.get_next_pose(point)
            if point == len(bot.goals) - 1:
                bool_msg = False
                bot.bool_publsiher.publish(bool_msg)
                break
            continue

        kp = 10.0
        ka = 1000.0
        speed_factor = 1.0
        
        if abs(bot.err_x) <= 25.0 or abs(bot.err_y) <= 25.0:
                speed_factor = 3.1

        max_force  = kp * 250 * math.sqrt(2)

        # if( bot.err_x <= 1.0 or bot.err_y <= 1.0):
        #     kp = 15.0

        # if( abs(bot.err_x) <= 5.0 and abs(bot.err_y) <= 5.0):
        #     avg_error =  (abs(bot.err_x) + abs(bot.err_y)) / 2.0
        #     if (avg_error != 0.0):
        #         kp = 6.9/(avg_error ** 1.8)

        # if( abs(bot.err_x) <= 1.0 or abs(bot.err_y) <= 1.0):
        #     kp = 2.8
        
        vel_x = bot.err_x * kp 
        vel_y = bot.err_y * kp
        omega = bot.err_theta * ka

        # Change the frame by using Rotation Matrix (If you find it required)

        chasis_velocity = math.sqrt(abs(vel_x*vel_x) + abs(vel_y*vel_y))
        chasis_velocity = abs(chasis_velocity)
 
        # if chasis_velocity >= 700 :
        #     chasis_velocity = 700

        # Find the required force vectors for individual wheels from it.(Inverse Kinematics)
        bot.inverse_kinematics(vel_x, vel_y, omega, speed_factor, max_force)
        print("VELOCITY: ", chasis_velocity, " , ", "ANGLE: ",  omega)     

        # while rclpy.ok():
            # bot.square()
        # time.sleep(1)
        rclpy.spin_once(bot)

    bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()