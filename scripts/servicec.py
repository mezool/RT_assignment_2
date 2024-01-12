#!/usr/bin/env python

import rospy
from assignment_2_2023.srv import GetRobotStatus
from geometry_msgs.msg import PoseStamped
from assignment_2_2023.msg import Custom
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from std_srvs.srv import SetBool
import math

class RobotStatusNode:
    def __init__(self):
        rospy.init_node('robot_status_node')

        # Subscribers
        rospy.Subscriber('/reaching_goal/feedback', PoseStamped, self.feedback_callback)
        rospy.Subscriber('/robot_positionvelocity', Custom, custom_callback)


        # Publishers
        self.distance_pub = rospy.Publisher('/distance_to_target', Float64, queue_size=10)
        self.speed_pub = rospy.Publisher('/average_speed', Float64, queue_size=10)

        # Service
        rospy.Service('/get_robot_status', GetRobotStatus, self.get_robot_status)

        # Variables
        self.target_pose = PoseStamped()
        self.robot_pose = Odometry()
        self.prev_robot_pose = Odometry()
        self.speed_sum = 0.0
        self.samples_count = 0

    def feedback_callback(self, feedback):
        self.target_pose = feedback

    def odom_callback(self, odom):
        self.robot_pose = odom
        distance = math.sqrt((self.target_pose.pose.position.x - self.robot_pose.pose.pose.position.x)**2 +
                             (self.target_pose.pose.position.y - self.robot_pose.pose.pose.position.y)**2)
        self.distance_pub.publish(distance)

        if self.samples_count > 0:
            current_speed = self.calculate_speed()
            self.speed_pub.publish(current_speed)

    def calculate_speed(self):
        current_speed = round(math.sqrt((self.robot_pose.pose.pose.position.x - self.prev_robot_pose.pose.pose.position.x)**2 +
                                  (self.robot_pose.pose.pose.position.y - self.prev_robot_pose.pose.pose.position.y)**2) / 0.1, 4)  # Assuming 0.1s between samples
        self.speed_sum += current_speed
        self.samples_count += 1
        return self.speed_sum / self.samples_count

    def get_robot_status(self, request):
        distance = math.sqrt((self.target_pose.pose.position.x - self.robot_pose.pose.pose.position.x)**2 +
                             (self.target_pose.pose.position.y - self.robot_pose.pose.pose.position.y)**2)
        average_speed = self.calculate_speed() if self.samples_count > 0 else 0.0

        return distance, average_speed

if __name__ == '__main__':
    try:
        robot_status_node = RobotStatusNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion")

