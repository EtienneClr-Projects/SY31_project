#!/usr/bin/env python3

import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from turtlebot3_msgs.msg import SensorState
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

from tf.transformations import quaternion_from_euler

def coordinates_to_message(x, y, O, t):
    msg = PoseStamped()
    msg.pose.position.x = x
    msg.pose.position.y = y
    [msg.pose.orientation.x,
     msg.pose.orientation.y,
     msg.pose.orientation.z,
     msg.pose.orientation.w] = quaternion_from_euler(0.0, 0.0, O)
    msg.header.stamp = t
    msg.header.frame_id = 'odom'
    return msg

class Odom2PoseNode:
    def __init__(self):
        rospy.init_node('odom2pose')
        rospy.loginfo("odom2pose node starting...")

        # Constants
        self.ENCODER_RESOLUTION = 4096
        self.WHEEL_RADIUS = 0.033
        self.WHEEL_SEPARATION = 0.160

        # Variables
        self.x_odom, self.y_odom, self.O_odom = 0, 0, 0
        self.x_gyro, self.y_gyro, self.O_gyro = 0, 0, 0
        self.prev_left_encoder = 0
        self.prev_right_encoder = 0
        self.prev_gyro_t = 0
        self.v = 0

        # Publishers
        self.pub_enco = rospy.Publisher('/pose_enco', PoseStamped, queue_size=10)
        self.pub_gyro = rospy.Publisher('/pose_gyro', PoseStamped, queue_size=10)
        
        self.pub_enco_theta = rospy.Publisher('/enco_theta', Float32, queue_size=10)
        self.pub_gyro_theta = rospy.Publisher('/gyro_theta', Float32, queue_size=10)

        self.pub_estimated_pose = rospy.Publisher('/estimated_pose', PoseStamped, queue_size=10)

        # Subscribers
        self.sub_gyro = rospy.Subscriber('/imu', Imu, self.callback_gyro)
        self.sub_enco = rospy.Subscriber('/sensor_state', SensorState, self.callback_enco)

        # 10Hz callback to merge the two sources of odometry
        rospy.Timer(rospy.Duration(0.1), self.callback_merge)

        rospy.loginfo("odom2pose node started !")

    def callback_merge(self, event):
        # Merge the two sources of odometry
        
        # We take the x and y from the encoders only
        pose = [0, 0, 0]
        pose[0] = self.x_odom
        pose[1] = self.y_odom
        pose[2] = self.O_odom
        # pose[2] = self.O_gyro
        # Publish the final pose
        msg = coordinates_to_message(pose[0], pose[1], pose[2], rospy.Time.now())
        self.pub_estimated_pose.publish(msg)

    def callback_enco(self, sensor_state):
        # Compute the differential in encoder count
        d_left_encoder = sensor_state.left_encoder-self.prev_left_encoder
        d_right_encoder = sensor_state.right_encoder-self.prev_right_encoder
        if self.prev_left_encoder == 0:
            self.prev_left_encoder = sensor_state.left_encoder
            self.prev_right_encoder = sensor_state.right_encoder
            return
        self.prev_right_encoder = sensor_state.right_encoder
        self.prev_left_encoder = sensor_state.left_encoder

        # Compute the linear and angular velocity (self.v and w)
        phi_left_encoder = d_left_encoder *2*np.pi/self.ENCODER_RESOLUTION  # rad/s
        phi_right_encoder = d_right_encoder * 2*np.pi/self.ENCODER_RESOLUTION  # rad/s
        
        vg = phi_left_encoder * self.WHEEL_RADIUS
        vd = phi_right_encoder * self.WHEEL_RADIUS
        
        self.v = (vg + vd) / 2
        w = (vd - vg) / self.WHEEL_SEPARATION
        
        # Update x_odom, y_odom and O_odom accordingly
        self.x_odom += self.v*np.cos(self.O_odom)
        self.y_odom += self.v*np.sin(self.O_odom)
        self.O_odom += w
        
        msg = coordinates_to_message(self.x_odom, self.y_odom, self.O_odom, sensor_state.header.stamp)
        self.pub_enco.publish(msg)
        self.pub_enco_theta.publish(self.O_odom)

    def callback_gyro(self, gyro):
        # if self.v == 0:
        #     return

        # Compute the elapsed time
        t = gyro.header.stamp.to_sec()
        dt = t - self.prev_gyro_t
        if self.prev_gyro_t == 0:
            self.prev_gyro_t = t
            self.gyro_vx = 0
            self.gyro_vy = 0
            return
        self.prev_gyro_t = t

        # compute the angular velocity
        # az = gyro.angular_velocity.z-0.052
        az = gyro.angular_velocity.z
        # if az < 0.1:
        #     print(az)

        if az < 0.01:
            az = 0.0
        #     az = 0.0
        
        # print()
        #print(az)
        
        
        # update O_gyro, x_gyro and y_gyro accordingly (using self.v)
        # self.x_gyro += self.v*np.cos(self.O_gyro)*dt
        # self.y_gyro += self.v*np.sin(self.O_gyro)*dt
        self.O_gyro += az*dt
        
        
        msg = coordinates_to_message(self.x_gyro, self.y_gyro, self.O_gyro, gyro.header.stamp)
        self.pub_gyro.publish(msg)
        self.pub_gyro_theta.publish(self.O_gyro)

if __name__ == '__main__':
    node = Odom2PoseNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass