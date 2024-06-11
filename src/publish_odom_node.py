#!/usr/bin/env python3

import rospy
import tf2_ros

from geometry_msgs.msg import TransformStamped, PoseStamped


class Publisher():
    """
    Only used to published the TF between odom and base_link for easier display of topics with RVIZ
    """
    def __init__(self) -> None:
        rospy.init_node('odom_to_base_link_tf_broadcaster')
        rospy.Subscriber('/estimated_pose', PoseStamped, self.odom_callback)

    def odom_callback(self, msg):
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation

        br.sendTransform(t)

if __name__ == "__main__":
    node = Publisher()
    try:
        rospy.spin()
    except rospy.ROSInternalException:
        pass
