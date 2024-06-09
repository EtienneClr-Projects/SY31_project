#! /usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from geometry_msgs.msg import PoseStamped

from enum import Enum

# Enum for the direction
class Direction(Enum):
    GAUCHE = 1
    DROITE = 2
    TOUT_DROIT = 3
    UNDEFINED = 4

def direction_to_string(direction: Direction):
    if direction == Direction.GAUCHE:
        return "GAUCHE"
    elif direction == Direction.DROITE:
        return "DROITE"
    elif direction == Direction.TOUT_DROIT:
        return "TOUT_DROIT"
    else:
        return "UNDEFINED"

class DirectionNode:
    def __init__(self):
        # Creates a node called and registers it to the ROS master
        rospy.init_node('direction_node')
        rospy.loginfo("direction node starting...")

        # CvBridge is used to convert ROS messages to matrices manipulable by OpenCV
        self.bridge = CvBridge()

        # Publisher to the output topics.
        self.pub_img = rospy.Publisher('~output', Image, queue_size=10)

        # Subscriber to the input topic. self.callback is called when a message is received
        self.image_subscriber = rospy.Subscriber('/image', Image, self.image_callback)

        self.ultrasound_subscriber = rospy.Subscriber('/ultrasound', Int32, self.ultrasound_callback)
        self.last_distance = None

        self.direction_publisher = rospy.Publisher('/direction', String, queue_size=10)

        self.pose_subscriber = rospy.Subscriber('/estimated_pose', PoseStamped, self.pose_callback)
        self.current_pose = None # (x, y, theta)

        self.direction = Direction.UNDEFINED
        self.angle_before_turning = None

        rospy.loginfo("direction node started !")
    
    def pose_callback(self, msg: PoseStamped):
        # x,y,theta
        theta = 2*np.arctan2(msg.pose.orientation.z, msg.pose.orientation.w)
        self.current_pose = (msg.pose.position.x, msg.pose.position.y, theta)

    def ultrasound_callback(self, msg: Int32):
        # print("ULTRASOUND: ", msg.data)
        self.last_distance = msg.data

    def image_callback(self, msg: Image):
        '''
        Function called when an image is received.
        msg: Image message received
        img_bgr: Width*Height*3 Numpy matrix storing the image
        '''

        if self.last_distance is None:
            return
        if self.current_pose is None:
            return
        
        # Convert ROS Image -> OpenCV
        try:
            img_bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn(e)
            return
        # bgr to hsv

        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

        # red mask of the image
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        # lower_red = np.array([60, 60, 120])
        # upper_red = np.array([90, 110, 210])
        mask = cv2.inRange(img_hsv, lower_red, upper_red)

        # count the number of red pixels
        red_count = cv2.countNonZero(mask)
        # print("red_count: ", red_count)

        # blue mask of the image
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([140, 255, 255])
        mask = cv2.inRange(img_hsv, lower_blue, upper_blue)

        # count the number of blue pixels
        blue_count = cv2.countNonZero(mask)
        # print("blue_count: ", blue_count)

        # if there is more blue than red, then we must go
        threshold = 50000
        # we display the distance only if we're close enough
        if self.last_distance < 800:
            if blue_count > red_count:
                if blue_count > threshold:
                    if self.direction != Direction.GAUCHE:
                        print("GAUCHE")
                        self.angle_before_turning = self.current_pose[2]
                    self.direction = Direction.GAUCHE
            else:
                if red_count > threshold:
                    if self.direction != Direction.DROITE:
                        print("DROITE")
                        self.angle_before_turning = self.current_pose[2]
                    self.direction = Direction.DROITE

        # if we're too far from a wall, we go straight 
        # elif self.last_distance > 4000:

        # if the difference between current angle and angle before turning is >=90°, we go straight
        if self.angle_before_turning is not None:
            if abs(self.current_pose[2] - self.angle_before_turning) >= np.pi/2:
                if self.direction != Direction.TOUT_DROIT:
                    print("TOUT DROIT")
                self.direction = Direction.TOUT_DROIT

        self.direction_publisher.publish(direction_to_string(self.direction))
    

        """
        IF WE NEED TO VISUALIZE AN IMAGE, PUT IT IN img_bgr
        """
        # Convert OpenCV -> ROS Image and publish
        try:
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(img_bgr, "bgr8")) # /!\ 'mono8' for grayscale images, 'bgr8' for color images
        except CvBridgeError as e:
            rospy.logwarn(e)

if __name__ == '__main__':
    # Start the node and wait until it receives a message or stopped by Ctrl+C
    node = DirectionNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass