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

    # Send an initial request with the index from ebot_controller.index
    #ebot_controller.send_request(ebot_controller.index)
    c_x=0
    x = [4, -4, -4, 4, 0]
    y = [4, 4, -4, -4, 0]
    theta = [0, math.pi/2, math.pi, -math.pi/2, 0]
    
    # Main loop
    while rclpy.ok():
        print("ok")

        # Check if the service call is done
        """if ebot_controller.future.done():
            try:
                # response from the service call
                response = ebot_controller.future.result()
            except Exception as e:
                ebot_controller.get_logger().infselfo(
                    'Service call failed %r' % (e,))
            else:
        #########           GOAL POSE             #########
        """
        for i in range(0,5):
            x_goal      = x[i]#response.x_goal
            y_goal      = y[i]#response.y_goal
            theta_goal  = theta[i]#response.theta_goal
            #ebot_controller.flag = response.end_of_list
            ####################################################
            kp=1
            # Find error (in x, y and theta) in global frame
            # the /odom topic is giving pose of the robot in global frame
            # the desired pose is declared above and defined by you in global frame
            # therefore calculate error in global frame
            ebot_controller.err_x=x_goal-ebot_controller.hb_x

            ebot_controller.err_y=y_goal-ebot_controller.hb_y
            #ebot_controller.err_theta=math.atan2(x_goal-ebot_controller.hb_x, y_goal-ebot_controller.hb_y)
            #ebot_controller.err_theta=-ebot_controller.hb_theta
            ebot_controller.err_theta=ebot_controller.hb_theta - theta_goal

            # (Calculate error in body frame)
            # But for Controller outputs robot velocity in robot_body frame, 
            # i.e. velocity are define is in x, y of the robot frame, 
            # Notice: the direction of z axis says the same in global and body frame
            # therefore the errors will have have to be calculated in body frame.
            #ebot_controller.v_x=ebot_controller.err_x
            #ebot_controller.v_y=ebot_controller.err_y
            ebot_controller.w_theta=ebot_controller.err_theta

            
            
            ebot_controller.v_x=(math.cos(ebot_controller.hb_theta)*ebot_controller.err_x + math.sin(ebot_controller.hb_theta)*ebot_controller.err_y)*kp
            ebot_controller.v_y=(-math.sin(ebot_controller.hb_theta)*ebot_controller.err_x + math.cos(ebot_controller.hb_theta)*ebot_controller.err_y)*kp
            


            # 
            # This is probably the crux of Task 1, figure this out and rest should be fine.
            
            ebot_controller.control=Twist()
            # Finally implement a P controller 
            # to react to the error with velocities in x, y and theta.

            # main wala tha ye 

            #ebot_controller.control.linear.x=kp*ebot_controller.err_x
            #ebot_controller.control.linear.y=kp*ebot_controller.err_y                
            #ebot_controller.control.angular.z=kp*ebot_controller.err_theta

            # main khatam hua

            ebot_controller.control.linear.x=ebot_controller.v_x
            ebot_controller.control.linear.y=ebot_controller.v_y
            ebot_controller.control.angular.z=float(ebot_controller.w_theta)





            #ebot_controller.err_x=0.0
            
            
            #ebot_controller.err_y=0.0










            

            # Safety Check
            # make sure the velocities are within a range.
            # for now since we are in a simulator and we are not dealing with actual physical limits on the system 
            # we may get away with skipping this step. But it will be very necessary in the long run.
            # send data to the topic.............
            #print(ebot_controller.control)
            print(x_goal,y_goal,theta_goal)
            print(ebot_controller.err_x, ebot_controller.err_y ,ebot_controller.err_theta)
            print(ebot_controller.v_x, ebot_controller.v_y, ebot_controller.w_theta)
            #print(ebot_controller.control)
            ebot_controller.pub.publish(ebot_controller.control)
            #If Condition is up to you
            time.sleep(1)
            
            ############     DO NOT MODIFY THIS       #########
            ebot_controller.index += 1
            #if ebot_controller.flag == 1 :
                #ebot_controller.index = 0
            ebot_controller.send_request(ebot_controller.index)
            ####################################################

            # Spin once to process callbacks
            rclpy.spin_once(ebot_controller)

    # Destroy the node and shut down ROS
    ebot_controller.destroy_node()
    rclpy.shutdown()

if __name__=="__main__" :
    main()


