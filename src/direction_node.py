#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from enum import Enum

from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


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

        # Subscriber to the input topic. self.callback is called when a message is received
        self.image_subscriber = rospy.Subscriber('/image', Image, self.image_callback)

        self.front_distance_subscriber = rospy.Subscriber('/front_distance', Float32, self.front_distance_callback)
        self.last_distance = None

        self.direction_publisher = rospy.Publisher('/direction', String, queue_size=10)

        self.text_direction_publisher = rospy.Publisher('/text_direction', Marker, queue_size=10)
        self.dir_to_display = None

        self.pose_subscriber = rospy.Subscriber('/estimated_pose', PoseStamped, self.pose_callback)
        self.current_pose = None # (x, y, theta)

        self.direction = Direction.UNDEFINED
        self.angle_before_turning = None

        rospy.loginfo("direction node started !")

        print("TOUT DROIT")
        self.publish_text_marker("TOUT DROIT")
        self.dir_to_display = "TOUT DROIT"
    
    def pose_callback(self, msg: PoseStamped):
        # x,y,theta
        theta = 2*np.arctan2(msg.pose.orientation.z, msg.pose.orientation.w)
        self.current_pose = (msg.pose.position.x, msg.pose.position.y, theta)

    def front_distance_callback(self, msg):
        self.last_distance = msg.data

    def publish_text_marker(self, text):
        marker = Marker()
        
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "ns"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Position du texte
        marker.pose.position.x = 0
        marker.pose.position.y = -0.4
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 1.0
        marker.pose.orientation.w = 0.0

        # Échelle du texte
        marker.scale.z = 0.1  # Hauteur du texte

        # Couleur du texte
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        # Texte à afficher
        if self.last_distance is None:
            return
        marker.text = text +"  " +str(round(self.last_distance,3))

        # never delete the marker
        marker.lifetime = rospy.Duration(0)
        self.text_direction_publisher.publish(marker)

    def image_callback(self, msg: Image):
        '''
        Function called when an image is received.
        msg: Image message received
        img_bgr: Width*Height*3 Numpy matrix storing the image
        '''
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

        # blue mask of the image
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([140, 255, 255])
        mask = cv2.inRange(img_hsv, lower_blue, upper_blue)

        # count the number of blue pixels
        blue_count = cv2.countNonZero(mask)

        # if there is more blue than red, then we must go
        threshold = 50000
        if self.last_distance is None:
            return
        if self.current_pose is None:
            return
        

        if blue_count > red_count:
            if blue_count > threshold:
                if self.dir_to_display != "GAUCHE":
                    if self.last_distance < 0.2: # we display the distance only if we're close enough
                        print("GAUCHE", self.last_distance)
                        self.dir_to_display = "GAUCHE"
                        self.angle_before_turning = self.current_pose[2]
                self.direction = Direction.GAUCHE
        else:
            if red_count > threshold:
                if self.dir_to_display != "DROITE":
                    if self.last_distance < 0.2: # we display the distance only if we're close enough
                        print("DROITE")
                        self.dir_to_display = "DROITE"
                        self.angle_before_turning = self.current_pose[2]
                self.direction = Direction.DROITE


        # if the difference between current angle and angle before turning is >=90°, we go straight
        if self.angle_before_turning is not None:
            if abs(self.current_pose[2] - self.angle_before_turning) >= np.pi/2:
                if self.dir_to_display != "TOUT DROIT":
                    print("TOUT DROIT")
                    self.dir_to_display = "TOUT DROIT"
                self.direction = Direction.TOUT_DROIT



        if self.direction == Direction.GAUCHE or self.direction == Direction.DROITE:
            # if we are at a modulo of 90° we can publish GAUCHE/DROITE
            angle_degrees = np.rad2deg(self.current_pose[2])
            angle_degrees = angle_degrees % 90
            if angle_degrees < 5 or angle_degrees > 85:
                self.direction_publisher.publish(direction_to_string(self.direction))
        else:
            self.direction_publisher.publish(direction_to_string(self.direction))
    
        self.publish_text_marker(self.dir_to_display)

if __name__ == '__main__':
    # Start the node and wait until it receives a message or stopped by Ctrl+C
    node = DirectionNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass