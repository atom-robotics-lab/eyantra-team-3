#! /usr/bin/env python3

################### IMPORT MODULES #######################
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import math
import numpy as np

# Camera calib info
##############################################################
# Camera calibration info
image_width = 640
image_height = 480
camera_matrix_data = [361.35897, 0., 292.63878, 0., 361.61727, 210.95102, 0., 0., 1.]
distortion_coefficients_data = [0.047427, -0.005604, 0.0235, -0.004758, 0.000000]

# Create camera matrix and distortion coefficients
camera_matrix = np.array(camera_matrix_data).reshape(3, 3)
distortion_coefficients = np.array(distortion_coefficients_data).reshape(1, 5)

DIM = (image_width, image_height)
K = np.array(camera_matrix)
D = np.array(distortion_coefficients[:, :4])  # Use only the first 4 coefficients


class ArUcoDetector(Node):

    def image_callback(self,msg):

        # print("Img received")
        try:
            #convert ROS image to opencv image
            image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

            #cv_image = self.undistort(self,image)
            cv_image = self.align(image,0)
            
            # cv2.imshow("Image",self.align(cv_image))
            # cv2.waitKey(1)
            # return
        
            # #Distortion matrix

            aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

            parameters = aruco.DetectorParameters_create()
            #Find markers
            (corners, ids, rejected) = aruco.detectMarkers(cv_image,aruco_dict, parameters=parameters)

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

                    bounding_box = (topRight,bottomRight,bottomLeft,topLeft)
                    # self.last_bounding_boxes[markerID] = bounding_box

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

                    cv2.circle(cv_image, (int(cv_x), int(cv_y)), 2, (0, 0, 255), -1)

                    center_x = (topLeft[0] + topRight[0] + bottomLeft[0] + bottomRight[0])/4
                    center_y = (topLeft[1] + topRight[1] + bottomLeft[1] + bottomRight[1])/4

                    diff_x = center_x - 250.0
                    diff_y = center_y - 250.0

                    # mult_x = 500/(topRight[0] - topLeft[0])
                    # # print(topRight[0], topLeft[0])
                    # mult_y = 500/(topLeft[1] - bottomLeft[1])

                    # cv_x += diff_x
                    # cv_y += diff_y

                    # cv_x *= mult_x
                    # cv_y *= mult_y

                    self.x = cv_x
                    self.y = cv_y

                    # cv_x -= 250
                    # cv_y -= 250

                    # cv_x *= (250/225)
                    # cv_y *= (250/225)

                    # cv_x += 250
                    # cv_y += 250

                    #calc theta
                    dx = bottomLeft[0] - bottomRight[0]
                    dy = bottomLeft[1] - bottomRight[1]   
                    
                    if dx!=0:                        
                        self.theta = -math.atan(dy/dx) *1.05
                        if self.theta<0:
                            self.theta = math.pi + self.theta

                        if bottomLeft[1]<bottomRight[1]:
                            self.theta += math.pi
                    else:
                        self.theta = 0.0

                    
                    if markerID in [1,2,3]:

                        pose_msg = Pose2D()

                        pen_dist = 7.5
                        x_off = pen_dist * 2 * math.sin(self.theta)
                        y_off = pen_dist * 2 * math.cos(self.theta)

                        print(f"{markerID} : x : {cv_x} y : {cv_y}")

                        pose_msg.x = cv_x
                        pose_msg.y = cv_y

                        # pose_msg.x = float(cv_x + x_off)
                        # pose_msg.y = float(cv_y + y_off)

                        if self.theta > math.radians(180):
                            self.theta = math.radians(360) - self.theta
                        
                        if (bottomLeft[0] < bottomRight[0] and bottomLeft[1] < bottomRight[1]) :
                            self.theta *= -1

                        if (bottomLeft[0] > bottomRight[0] and bottomLeft[1] < bottomRight[1]):  
                            self.theta *= -1
               
                        pose_msg.theta = self.theta #+ math.pi/2 + .4


                        cv2.circle(cv_image, (int(self.x + x_off), int(self.y + y_off)), 3, (255, 0, 0), -1)
                        
                        # Publish pose_msg based on markerID
                        if markerID == 1:
                            self.pen1_pub.publish(pose_msg)
                            if self.pen1_down:
                                self.red.append((int(self.x + x_off), int(self.y + y_off)))

                        elif markerID == 2:
                            self.pen2_pub.publish(pose_msg)
                            if self.pen2_down:
                                self.green.append((int(self.x + x_off), int(self.y + y_off)))

                        elif markerID == 3:
                            self.pen3_pub.publish(pose_msg)
                            if self.pen3_down:
                                self.blue.append((int(self.x + x_off), int(self.y + y_off)))

                        
                        pose_x = (bottomLeft[0] + bottomRight[0])/2
                        pose_y = (bottomLeft[1] + bottomRight[1])/2
                        
                        # cv2.circle(cv_image, (int(pose_x), int(pose_y)), 3, (255, 0, 0), -1)

                    # draw the ArUco marker ID on the image
                    cv2.putText(cv_image,str(markerID), (topLeft[0], topLeft[1] - 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)

                    self.draw_points(cv_image, self.blue, self.green, self.red)
                    # print("[INFO] ArUco marker ID: {}".format(markerID))

                    # rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, distortion_coefficients)

                    # if rvecs is not None and tvecs is not None:
                    #     for rvec, tvec in zip(rvecs, tvecs):
                    #         rotation_matrix, _ = cv2.Rodrigues(rvec)
                    #         yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
                    #         pr
                    # int("Yaw angle:", np.degrees(yaw))
    
            except Exception as e:
                print(e)    

            #cv2.imshow("Image", cv2.resize(cv_image, (500, 500)))
            cv2.imshow("Image",cv_image)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('r'):
                self.red = []

            if key == ord('b'):
                self.blue = []

            if key == ord('g'):
                self.green = []
        
        except Exception as e:
            self.get_logger().error(e)

    def align(self,image,margin):
        image = image[60+margin:402-margin,85+margin:388-margin]  #height, width  
        image = cv2.resize(image,(500,500))

        return image
    
    def draw_points(self, frame, blue, green, red):
        for point in blue:
            cv2.circle(frame, tuple(point), 2, (255, 0, 0), -1)

        for point in green:
            cv2.circle(frame, tuple(point), 2, (0, 255, 0), -1)
        
        for point in red:
            cv2.circle(frame, tuple(point), 2, (0, 0, 255), -1)
                
    def undistort(self,image):
        image = image[0:480,40:640]

        # Camera calibration info
        image_width = 640
        image_height = 480
        camera_matrix_data = [361.35897, 0., 292.63878, 0., 361.61727, 210.95102, 0., 0., 1.]
        distortion_coefficients_data = [0.047427, -0.005604, 0.0235, -0.004758, 0.000000]

        # Create camera matrix and distortion coefficients
        camera_matrix = np.array(camera_matrix_data).reshape(3, 3)
        distortion_coefficients = np.array(distortion_coefficients_data).reshape(1, 5)

        DIM = (image_width, image_height)
        K = np.array(camera_matrix)
        D = np.array(distortion_coefficients[:, :4])  # Use only the first 4 coefficients

        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
        undistorted_img = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        undistorted_img = undistorted_img[0:480,70:550]

        # cv2.imshow("Original Image", image)

        cv2.imshow("Undistorted Image", cv2.resize(undistorted_img,(500,500)))
        #cv2.imshow("Undistorted Image", undistorted_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        return undistorted_img

    def pen_callback_1(self, msg):
        self.pen1_down = msg

    def pen_callback_2(self, msg):
        self.pen2_down = msg

    def pen_callback_3(self, msg):
        self.pen3_down = msg
        
    def __init__(self):
        super().__init__('ar_uco_detector')

        # Subscribe the topic /camera/image_raw
        self.subscription = self.create_subscription(
            Image,
            '/image_rect_color',
            self.image_callback,
            10)

        self.pen_1 = self.create_subscription(Bool, '/pen1_down', self.pen_callback_1, 10)
        self.pen_2 = self.create_subscription(Bool, '/pen2_down', self.pen_callback_2, 10)
        self.pen_3 = self.create_subscription(Bool, '/pen3_down', self.pen_callback_3, 10)

        self.bridge = CvBridge()

        self.pen1_pub = self.create_publisher(Pose2D, '/pen1_pose', 10)
        self.pen2_pub = self.create_publisher(Pose2D, '/pen2_pose', 10)
        self.pen3_pub = self.create_publisher(Pose2D, '/pen3_pose', 10)

        # self.ids=[1,2,3,4,8,10,12]
        #Test values
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.blue = []
        self.green = []
        self.red = []

        self.pen1_down = False
        self.pen2_down = False
        self.pen3_down = False

        #self.image_callback()

def main(args=None):
    rclpy.init(args=args)

    aruco_detector = ArUcoDetector()

    rclpy.spin(aruco_detector)

    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()