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
################### IMPORT MODULES #######################
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import math
import numpy as np

# Import the required modules
##############################################################
class ArUcoDetector(Node):
    

    def __init__(self):
        super().__init__('ar_uco_detector')

        # Subscribe the topic /camera/image_raw
        self.subscription = self.create_subscription(Image,'/camera/image_raw',self.image_callback,10)

        self.bridge = CvBridge()

        self.publisher = self.create_publisher(Pose2D, '/detected_aruco', 10)

        #Test values
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def image_callback(self, msg):
        try:
            #convert ROS image to opencv image
            cv_image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

            #Detect Aruco marker

            #4x4 dict
            aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

            parameters = aruco.DetectorParameters()

            #Distortion matrix
            dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])

            cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])

            #Find markers
            (corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image, aruco_dict, parameters=parameters)

            #Lag gye :D
            #X,y,theta nikalna hai

            try:
                #print(corners, ids, rejected)
                #print(ids)
                for i in range(len(ids)):
                    ids = ids.flatten()

                    # loop over the detected ArUCo corners
                    for (markerCorner, markerID) in zip(corners, ids):

                        # extract the marker corners (which are always returned in
                        # top-left, top-right, bottom-right, and bottom-left order)
                        corners = markerCorner.reshape((4, 2))
                        (topLeft, topRight, bottomRight, bottomLeft) = corners

                        # convert each of the (x, y)-coordinate pairs to integers
                        topRight = (int(topRight[0]), int(topRight[1]))
                        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                        topLeft = (int(topLeft[0]), int(topLeft[1]))

                        # draw the bounding box of the ArUCo detection
                        cv2.line(cv_image, topLeft, topRight, (0, 255, 0), 1)
                        cv2.line(cv_image, topRight, bottomRight, (0, 255, 0), 1)
                        cv2.line(cv_image, bottomRight, bottomLeft, (0, 255, 0), 1)
                        cv2.line(cv_image, bottomLeft, topLeft, (0, 255, 0), 1)

                        
                        # compute and draw the center (x, y)-coordinates of the ArUco marker
                        self.x = float((topLeft[0] + bottomRight[0]) / 2.0)
                        self.y = float((topLeft[1] + bottomRight[1]) / 2.0)
                        
                        cv_x = self.x
                        cv_y = self.y

                        # self.x /= 2
                        # self.y /= 2
                        # #Process coords

                        # #Offset centre
                        # self.x -= 250.0
                        # self.y -= 250.0

                        # #Flip axis
                        # #self.x *= -1
                        # self.y *= -1

                        # #Normalize with Gazebo
                        # self.x /=2
                        # self.y /=2

                        # self.x *= (5/2)
                        # self.y *= (5/2)

                        cv2.circle(cv_image, (int(cv_x), int(cv_y)), 2, (0, 0, 255), -1)

                        #calc theta
                        dx = bottomLeft[0] - bottomRight[0]
                        dy = bottomLeft[1] - bottomRight[1]

                        self.theta = -math.atan(dy/dx) *1.05

                        #Publish the bot coordinates to the topic  /detected_aruco
                        if markerID == 1:
                            #Create msg
                            pose_msg = Pose2D()
                            pose_msg.x = self.x #* (25/26)
                            pose_msg.y = self.y #* (25/26)
                            pose_msg.theta = self.theta
                            
                            self.publisher.publish(pose_msg)

                        # draw the ArUco marker ID on the image
                        cv2.putText(cv_image,str(markerID), (topLeft[0], topLeft[1] - 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
                        print("[INFO] ArUco marker ID: {}".format(markerID))
                
                if len(ids) > 0:
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners,0.05,cam_mat,dist_mat)
                    rotation_matrix, _ = cv2.Rodrigues(rvecs)

                    # Extract the yaw angle from the rotation matrix
                    yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
                    print(yaw)

            
            except Exception as e:
                print(e)    

            cv2.imshow("Image", cv2.resize(cv_image,(500,500)))
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(e)

def main(args=None):
    rclpy.init(args=args)

    aruco_detector = ArUcoDetector()

    rclpy.spin(aruco_detector)

    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
