#!/usr/bin/env python3

import rospy

from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan


class DistanceNode:
    def __init__(self):
        # Creates a node called and registers it to the ROS master
        rospy.init_node('front_distance_node')
        rospy.loginfo("front distance node starting...")

        self.lidar_subscriber = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        self.front_distance = None

        self.distance_publisher = rospy.Publisher('/front_distance', Float32, queue_size=10)

        rospy.loginfo("fleche direction node started !")
    

    def lidar_callback(self, msg):
        # front distance is the distance at angle 180
        self.front_distance = msg.ranges[0]
        if self.front_distance < msg.range_min:
            self.front_distance = msg.range_min
        self.distance_publisher.publish(self.front_distance)
    
if __name__ == '__main__':
    node = DistanceNode()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("front distance node stopped")
        pass    
