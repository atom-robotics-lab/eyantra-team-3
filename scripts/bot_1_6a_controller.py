#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
import time
import math

max_force = 0
kp = 0

class BotController(Node):
    
    def __init__(self):

        super().__init__('bot_controller_1')
        self.publisher = self.create_publisher(Twist, '/cmd_vel/bot1', 10)
        self.bool_publsiher = self.create_publisher(Bool, '/pen1_down',10)
        self.bot_reached_publsiher = self.create_publisher(Bool, '/bot1_reached',10)
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

    def get_next_pose(self, point) :
        #Red
        self.goals = [(470, 250),(469, 245),(466, 240),(462, 236),(456, 232),(449, 228),(441, 225),(431, 223),(419, 221),(407, 219),(393, 219),(379, 219),(363, 220),(347, 222),(330, 225),(313, 229),(296, 233),(279, 239),(262, 245),(244, 252),(228, 259),(211, 268),(196, 276),(181, 286),(167, 295),(153, 305),(141, 315),(130, 325),(121, 335),(112, 345),(105, 355),(99, 364),(95, 373),(92, 381),(90, 388),(90, 394),(91, 399),(93, 404),(96, 407),(101, 409),(106, 409),(113, 409),(120, 407),(128, 403),(137, 399),(146, 393),(156, 385),(166, 377),(176, 367),(186, 355),(196, 343),(206, 330),(215, 315),(224, 300),(233, 284),(241, 268),(249, 251),(256, 234),(262, 217),(267, 199),(271, 182),(275, 165),(277, 149),(279, 133),(280, 117),(280, 103),(279, 90),(278, 77),(276, 66),(273, 57),(270, 48),(266, 41),(262, 36),(258, 32),(253, 30),(249, 30),(244, 31),(240, 33),(235, 38),(231, 44),(228, 51),(225, 60),(222, 71),(220, 82),(219, 95),(219, 109),(219, 123),(221, 139),(223, 155),(226, 172),(229, 189),(234, 206),(240, 224),(246, 241),(253, 258),(261, 275),(269, 291),(278, 306),(288, 321),(297, 335),(307, 348)]
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

        self.hola_x = msg.x
        self.hola_y = msg.y
        self.hola_theta = msg.theta #- math.pi/2 - 0.4

        self.hola_x -=250
        self.hola_y *=-1
        self.hola_y +=250
        # self.hola_y *-1

        self.hola_theta *= -1
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
        
        
        if(point > 0 and point < len(bot.goals)) :
            bool_msg.data = True  # Set its value to False
            bot.bool_publsiher.publish(bool_msg)  # Publish the message
            # time.sleep(10s)


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

        if abs(bot.err_x) <= threshold and abs(bot.err_y) <= threshold :
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

        kp = 5.0
        ka = 1000.0
        speed_factor = 8.0
        
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
