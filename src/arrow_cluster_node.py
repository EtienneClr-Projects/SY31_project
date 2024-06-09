#!/usr/bin/env python3

import rospy
import numpy as np
import math as m
import cv2

from sensor_msgs.point_cloud2 import read_points
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

from tf.transformations import quaternion_from_euler

class FlecheClusterNode:
    def __init__(self):
        # Creates a node called and registers it to the ROS master
        rospy.init_node('fleche_cluster_node')
        rospy.loginfo("fleche cluster node starting...")

        # Subscriber to the input topic. self.callback is called when a message is received
        self.estimate_pose_subscriber = rospy.Subscriber("/estimated_pose", PoseStamped, self.receive_estimated_pose)
        self.current_pose = np.array([0., 0., 0.]) # x, y, theta

        self.direction_subscriber = rospy.Subscriber('/direction', String, self.direction_callback)

        self.pub_markers_points = rospy.Publisher('/points', MarkerArray, queue_size=10)
        self.pub_markers_arrows = rospy.Publisher('/arrows', MarkerArray, queue_size=10)

        self.front_distance_subscriber = rospy.Subscriber("/front_distance", Float32, self.front_distance_callback)
        self.front_distance = None

        rospy.loginfo("fleche direction node started !")
        

        self.liste_rouge = []
        self.liste_bleu = []

    def front_distance_callback(self, msg):
        self.front_distance = msg.data
        
    def dist(self,pt1,pt2):
        return m.sqrt((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2)
        
    
    def receive_estimated_pose(self, msg):
        self.current_pose = np.array([msg.pose.position.x,
                                        msg.pose.position.y,
                                        m.atan2(msg.pose.orientation.z, msg.pose.orientation.w) * 2])
        
    def direction_callback(self,msg):
        if self.front_distance is None:
            return
        clusters_bleu = {}
        clusters_rouge = {}
        if msg.data == "GAUCHE":
            self.liste_bleu.append(self.current_pose[:2] + np.array([self.front_distance*m.cos(self.current_pose[2]),self.front_distance*m.sin(self.current_pose[2])]))
            
        elif msg.data == "DROITE":
            self.liste_rouge.append(self.current_pose[:2] + np.array([self.front_distance*m.cos(self.current_pose[2]),self.front_distance*m.sin(self.current_pose[2])]))

        
        if len(self.liste_bleu) != 0:
            current_cluster_index = 0
            clusters_bleu[0] = [self.liste_bleu[0]]
            for i in range (1, len(self.liste_bleu)):
                if self.dist(self.liste_bleu[i-1],self.liste_bleu[i]) > 0.3:
                    current_cluster_index +=1
                    clusters_bleu[current_cluster_index] = []
                clusters_bleu[current_cluster_index].append(self.liste_bleu[i])

        if len(self.liste_rouge) != 0:            
            current_cluster_index = 0 
            clusters_rouge[0] = [self.liste_rouge[0]]
            for i in range (1, len(self.liste_rouge)):
                if self.dist(self.liste_rouge[i-1],self.liste_rouge[i]) > 0.3:
                    current_cluster_index +=1
                    clusters_rouge[current_cluster_index] = []
                clusters_rouge[current_cluster_index].append(self.liste_rouge[i])
            
        
        markers_points = MarkerArray()
        # display all blue point from liste_bleu
        for i in range(len(self.liste_bleu)):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = Point(self.liste_bleu[i][0], self.liste_bleu[i][1], 0)
            marker.pose.orientation.w = 1
            marker.scale.x, marker.scale.y, marker.scale.z = 0.02, 0.02, 0.02
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0, 0, 1, 0.5
            marker.lifetime = rospy.Duration(0)
            markers_points.markers.append(marker)

        # display all red point from liste_rouge
        for i in range(len(self.liste_rouge)):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.id = i + len(self.liste_bleu)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = Point(self.liste_rouge[i][0], self.liste_rouge[i][1], 0)
            marker.pose.orientation.w = 1
            marker.scale.x, marker.scale.y, marker.scale.z = 0.02, 0.02, 0.02
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1, 0, 0, 0.5
            marker.lifetime = rospy.Duration(0)
            markers_points.markers.append(marker)




        markers_arrows = MarkerArray()
        for c, points in clusters_bleu.items():
            points = np.array(points)
            if len(points) < 2:
                continue
        
            # Calculate cluster center
            minP = np.min(points, axis=0)
            maxP = np.max(points, axis=0)
            center = (minP + maxP)/2
            
            vector = maxP - minP
            angle = m.atan2(vector[1],vector[0])
        
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.id = c
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position = Point(center[0], center[1], 0)
            marker.pose.orientation.w = 1
            [marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w] = quaternion_from_euler(0.0, 0.0, angle)
            marker.scale.x, marker.scale.y, marker.scale.z = 0.05,0.05, 0.05
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0, 0, 1, 1
            marker.lifetime = rospy.Duration(0)
            markers_arrows.markers.append(marker)
        
        for c, points in clusters_rouge.items(): # .items returns a list of tuples (key, value)
            points = np.array(points)   
            if len(points) < 2:
                continue 
        
            # Calculate cluster center
            minP = np.min(points, axis=0)
            maxP = np.max(points, axis=0)
            center = (minP + maxP)/2
        
            vector = maxP - minP
            angle = m.atan2(vector[1],vector[0])
        
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.id = c
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position = Point(center[0], center[1], 0)
            marker.pose.orientation.w = 1
            [marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w] = quaternion_from_euler(0.0, 0.0, angle)
            marker.scale.x, marker.scale.y, marker.scale.z = 0.05,0.05,0.05
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1, 0,0, 1
            marker.lifetime = rospy.Duration(0)
            markers_arrows.markers.append(marker)

        self.pub_markers_arrows.publish(markers_arrows)
        self.pub_markers_points.publish(markers_points)
        
    
    
if __name__ == '__main__':
    node = FlecheClusterNode()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("fleche cluster node stopped")
        pass    
