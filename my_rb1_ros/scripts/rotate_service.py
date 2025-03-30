#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from my_rb1_ros.srv import RotateRequest, RotateResponse, Rotate
from nav_msgs.msg import Odometry
import time
import math
from tf.transformations import euler_from_quaternion

class RB1RotateServerClass(object):

    def __init__(self):
        # creates the service server
        self.my_service = rospy.Service('/rotate_robot', Rotate, self.service_callback) # create the Service called my_service with the defined callback
        rospy.loginfo("Service Ready")
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_subs = rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        self.rotate_cmd = Twist()
        self.odom_data = Odometry()
        self.request = RotateRequest()
        self.req_deg = None
        self.response  = RotateResponse()

    def cmd_rotate(self):
        rospy.loginfo("Starting rotation!")
        if self.request.degrees >= 0:
            self.rotate_cmd.angular.z = 0.2
        else:
            self.rotate_cmd.angular.z = -0.2
        while self.cmd_pub.get_num_connections() < 1:
            pass
        self.cmd_pub.publish(self.rotate_cmd)

    def cmd_stop(self):
        rospy.loginfo("Stopping rotation!")
        self.rotate_cmd.angular.z = 0.0
        while self.cmd_pub.get_num_connections() < 1:
            pass
        self.cmd_pub.publish(self.rotate_cmd)

    def conv_quat_to_deg(self, data : Odometry):
        orientation_q = data.pose.pose.orientation
        quaternion = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        _, _, yaw = euler_from_quaternion(quaternion)
        yaw_degrees = math.degrees(yaw)
        return yaw_degrees

    def rotate_rb1_by(self):
        req_deg = abs(self.request.degrees)
        # read initial pose
        start_angle = self.conv_quat_to_deg(self.odom_data)
        # pub rotation cmd
        self.cmd_rotate()
        # monitor rotation
        start_time = time.time()
        timeout = 120.0
        current_angle = self.conv_quat_to_deg(self.odom_data)
        difference_angle = abs(current_angle - start_angle)
        while difference_angle <= req_deg:
            current_angle = self.conv_quat_to_deg(self.odom_data)
            difference_angle = abs(current_angle - start_angle)
            current_time = time.time()
            if current_time - start_time > timeout:
                self.response.result = "RB1 did not rotate successfully - Timeout error"
                break
            rospy.sleep(0.1)
        self.cmd_stop()
        self.response.result = "RB1 rotated successfully"

    def odometry_callback(self, data : Odometry):
        self.odom_data = data

    def service_callback(self, request: RotateRequest):
        rospy.loginfo("Service Requested")
        self.request = request
        try:
            self.rotate_rb1_by()
        except rospy.ServiceException:
            self.response.result = "Failed to call rotate_robot - Communication error"
        rospy.loginfo("Service completed")
        return self.response # the service Response class
    

if __name__ == '__main__':
  rospy.init_node('rotate_rb1_service_server')
  RB1RotateServerClass()
  rospy.spin()