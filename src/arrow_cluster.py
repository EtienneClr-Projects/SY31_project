import rospy
import numpy as np
import math as m
import cv2

from sensor_msgs.point_cloud2 import read_points
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Int32, String
from geometry_msgs.msg import PoseStamped

from tf.transformations import quaternion_from_euler

class FlecheClusterNode:
    def __init__(self):
        # Creates a node called and registers it to the ROS master
        rospy.init_node('fleche_cluster_node')
        rospy.loginfo("fleche cluster node starting...")

        # Subscriber to the input topic. self.callback is called when a message is received
        self.estimate_pose_subscriber = rospy.Subscriber("/estimated_pose", PoseStamped, self.receive_estimated_pose)
        self.current_pose = np.array([0., 0., 0.]) # x, y, theta

        self.ultrason_subscriber = rospy.Subscriber("/ultrasound", Int32, self.ultrason_callback)

        self.direction_subscriber = rospy.Subscriber('/direction', String, self.direction_callback)

        self.pub_bboxes = rospy.Publisher('/lidar/bboxes', MarkerArray, queue_size=10)

        rospy.loginfo("fleche direction node started !")
        

        self.liste_rouge = []
        self.liste_bleu = []
        
    def dist(self,pt1,pt2):
        return m.sqrt((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2 + (pt1[2]-pt2[2])**2)
        
    
    def receive_estimated_pose(self, msg):
        self.current_pose = np.array([msg.pose.position.x,
                                        msg.pose.position.y,
                                        m.atan2(msg.pose.orientation.z, msg.pose.orientation.w) * 2])
        
    def ultrason_callback(self, msg):
        self.dist_ultrason = msg.data
        
        
    def direction_callback(self,msg):
        clusters_bleu = {}
        clusters_rouge = {}
        if msg.data == "GAUCHE":
            self.liste_bleu.append(self.current_pose + [self.dist_ultrason*m.cos(self.current_pose[2]),self.dist_ultrason*m.sin(self.current_pose[2])]) # TODO
            
        elif msg.data == "DROITE":
            self.liste_rouge.append(self.current_pose + [self.dist_ultrason*m.cos(self.current_pose[2]),self.dist_ultrason*m.sin(self.current_pose[2])])

        print(len(self.liste_bleu), len(self.liste_rouge))
        
        if len(self.liste_bleu) != 0:
            current_cluster_index = 0
            clusters_bleu[0] = [self.liste_bleu[0]]
            for i in range (1, len(self.liste_bleu)):
                if self.dist(self.liste_bleu[i-1],self.liste_bleu[i]) > 0.2:
                    current_cluster_index +=1
                    clusters_bleu[current_cluster_index] = []
                clusters_bleu[current_cluster_index].append(self.liste_bleu[i])

        if len(self.liste_rouge) != 0:            
            current_cluster_index = 0 
            clusters_rouge[0] = [self.liste_rouge[0]]
            for i in range (1, len(self.liste_rouge)):
                if self.dist(self.liste_rouge[i-1],self.liste_rouge[i]) > 0.2:
                    current_cluster_index +=1
                    clusters_rouge[current_cluster_index] = []
                clusters_rouge[current_cluster_index].append(self.liste_rouge[i])
            
            
        
        bboxes = MarkerArray()
        for c, points in clusters_bleu.items():
            if c == 0:
                continue
            points = np.array(points)    
        
            # Calculate cluster center
            minP = np.min(points, axis=0)
            maxP = np.max(points, axis=0)
            center = (minP + maxP)/2
        
            # Calculate cluster length and width
            width, length = np.max(points-center, axis=0)[0] * 2, np.max(points-center, axis=0)[1] * 2
            
            vector = maxP - minP
            angle = m.atan2(vector[1],vector[0])
        
            bbox = Marker()
            # bbox.header = msg.header
            bbox.header.frame_id = "odom"
            bbox.id = c
            bbox.type = Marker.ARROW
            bbox.action = Marker.ADD
            bbox.pose.position = Point(center[0], center[1], 0)
            bbox.pose.orientation.w = 1
            [bbox.pose.orientation.x, bbox.pose.orientation.y, bbox.pose.orientation.z, bbox.pose.orientation.w] = quaternion_from_euler(0.0, 0.0, angle)
            bbox.scale.x, bbox.scale.y, bbox.scale.z = 0.3,0.3, 0.3
            bbox.color.r, bbox.color.g, bbox.color.b, bbox.color.a = 0, 0, 1, 0.5
            bbox.lifetime = rospy.Duration(0.2)
            bboxes.markers.append(bbox)
            c+=1
        
        for c, points in clusters_rouge.items():
            if c == 0:
                continue
            points = np.array(points)    
        
            # Calculate cluster center
            minP = np.min(points, axis=0)
            maxP = np.max(points, axis=0)
            center = (minP + maxP)/2
        
            # Calculate cluster length and width
            width, length = np.max(points-center, axis=0)[0] * 2, np.max(points-center, axis=0)[1] * 2
            
            vector = maxP - minP
            angle = m.atan2(vector[1],vector[0])
        
            bbox = Marker()
            # bbox.header = msg.header
            bbox.header.frame_id = "odom"
            bbox.id = c
            bbox.type = Marker.ARROW
            bbox.action = Marker.ADD
            bbox.pose.position = Point(center[0], center[1], 0)
            bbox.pose.orientation.w = 1
            [bbox.pose.orientation.x, bbox.pose.orientation.y, bbox.pose.orientation.z, bbox.pose.orientation.w] = quaternion_from_euler(0.0, 0.0, angle)
            bbox.scale.x, bbox.scale.y, bbox.scale.z = 0.3,0.3, 0.3
            bbox.color.r, bbox.color.g, bbox.color.b, bbox.color.a = 1, 0,0, 0.5
            bbox.lifetime = rospy.Duration(0.2)
            bboxes.markers.append(bbox)
            c+=1

        self.pub_bboxes.publish(bboxes)
        
    
    
if __name__ == '__main__':
    node = FlecheClusterNode()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("fleche cluster node stopped")
        pass    
