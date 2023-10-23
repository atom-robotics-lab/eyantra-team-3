#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.srv import NextGoal

class HBTask1BController(Node):

    def __init__(self):
        super().__init__('hb_task_1b')
        
        # Initialze Publisher and Subscriber
        # We'll leave this for you to figure out the syntax for
        # initialising publisher and subscriber of cmd_vel and odom respectively
        self.pub=self.create_publisher(Twist,"/cmd_vel",10)
        self.subscription =self.create_subscription(Odometry,"/odom",self.odometryCb, 10)
        

        # Declare a Twist message
        self.vel = Twist()
        # Initialise the required variables to 0
        self.hb_x = 0
        self.hb_y = 0
        self.hb_theta = 0


        # For maintaining control loop rate.
        self.rate = self.create_rate(100)
        # Initialise variables that may be needed for the control loop
        x = [4, -4, -4, 4, 0]
        y = [4, 4, -4, -4, 0]
        theta = [0, math.pi/2, math.pi, -math.pi/2, 0]

        # For ex: x_d, y_d, theta_d (in **meters** and **radians**) for defining desired goal-pose.
        # and also Kp values for the P Controller


        # client for the "next_goal" service
        self.cli = self.create_client(NextGoal, 'next_goal')      
        self.req = NextGoal.Request() 
        self.index = 0

    def send_request(self, index):
        self.req.request_goal = index
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
            
    def odometryCb(self, msg):
        # Write your code to take the msg and update the three variables
        global hb_x, hb_y, hb_theta
        self.hb_x=msg.pose.pose.position.x
        self.hb_y=msg.pose.pose.position.y
        self.hb_theta=msg.pose.pose.orientation.w
        #print(msg)
        #updatation done ....
        # doubtful......................................
        

def main(args=None):
    rclpy.init(args=args)

    # Create an instance of the EbotController class
    ebot_controller = HBTask1BController()

    # Define the square coordinates
    x_square = [4,4,0,0]
    y_square = [0,4,4,0]

    # Initialize the index to 0
    ebot_controller.index = 0

    # Main loop
    while rclpy.ok():
        
        # Extract goal coordinates from the square
        x_goal = x_square[ebot_controller.index]
        y_goal = y_square[ebot_controller.index]
        
        # Calculate error in global frame
        ebot_controller.err_x = x_goal - ebot_controller.hb_x
        ebot_controller.err_y = y_goal - ebot_controller.hb_y
        
        # Calculate control commands (P controller)
        kp = 0.95
        ebot_controller.v_x = (math.cos(ebot_controller.hb_theta) * ebot_controller.err_x +
                                math.sin(ebot_controller.hb_theta) * ebot_controller.err_y) * kp
        ebot_controller.v_y = (-math.sin(ebot_controller.hb_theta) * ebot_controller.err_x +
                                math.cos(ebot_controller.hb_theta) * ebot_controller.err_y) * kp
        ebot_controller.w_theta = 0.0  # No angular velocity for a square path

        # Publish control commands
        ebot_controller.control = Twist()
        ebot_controller.control.linear.x = ebot_controller.v_x
        ebot_controller.control.linear.y = ebot_controller.v_y
        ebot_controller.control.angular.z = ebot_controller.w_theta

        ebot_controller.pub.publish(ebot_controller.control)

        # Sleep for 1 second at each point
        time.sleep(1)

        # Stop the robot after reaching the corner
        control = Twist()
        ebot_controller.pub.publish(control)  # Publish zero velocities

        # Sleep for 1 second to stop at the corner
        time.sleep(1)

        
        ############     DO NOT MODIFY THIS       #########
        ebot_controller.index += 1
        if ebot_controller.index >= len(x_square):
            ebot_controller.index = 0
        ebot_controller.send_request(ebot_controller.index)
        ####################################################

        # Spin once to process callbacks
        rclpy.spin_once(ebot_controller)


    # Destroy the node and shut down ROS
    ebot_controller.destroy_node()
    rclpy.shutdown()

if __name__=="__main__" :
    main()