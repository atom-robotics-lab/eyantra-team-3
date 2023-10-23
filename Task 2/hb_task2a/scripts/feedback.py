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
import geometry_msgs.msg
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco

# Import the required modules
##############################################################
class ArUcoDetector(Node):

    def __init__(self):
        super().__init__('ar_uco_detector')

        # Subscribe the topic /camera/image_raw
        self.subscription = self.create_subscription(Image,'/camera/image_raw',self.image_callback,10)

        self.bridge = CvBridge()

        self.publisher = self.create_publisher(Pose2D, '/detected_aruco', 10)



    def image_callback(self, msg):
        try:
            #convert ROS image to opencv image
            cv_image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

            #Detect Aruco marker

            #5x5 dict
            aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)

            parameters = aruco.DetectorParameters_create()

            #Find markers
            (corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image, aruco_dict, parameters=parameters)

            #Lag gye :D
            #X,y,theta nikalna hai

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
                    cv2.line(cv_image, topLeft, topRight, (0, 255, 0), 2)
                    cv2.line(cv_image, topRight, bottomRight, (0, 255, 0), 2)
                    cv2.line(cv_image, bottomRight, bottomLeft, (0, 255, 0), 2)
                    cv2.line(cv_image, bottomLeft, topLeft, (0, 255, 0), 2)


                    # compute and draw the center (x, y)-coordinates of the ArUco marker
                    cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                    cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                    cv2.circle(cv_image, (cX, cY), 4, (0, 0, 255), -1)

                    # draw the ArUco marker ID on the image
                    cv2.putText(cv_image, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    print("[INFO] ArUco marker ID: {}".format(markerID))
            
            if len(ids) > 0:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners,0.05)

            cv2.imshow("Image", cv_image)
            cv2.waitKey(1)


            #Publish the bot coordinates to the topic  /detected_aruco

            #Test values
            x = 1.0
            y = 2.0
            theta = 0.5

            #Create msg
            pose_msg = Pose2D()
            pose_msg.x = x
            pose_msg.y = y
            pose_msg.theta = theta
            
            self.publisher.publish(pose_msg)

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
