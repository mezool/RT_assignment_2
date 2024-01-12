#!/usr/bin/env python

import rospy
from assignment_2_2023.srv import GetTargetPosition
from geometry_msgs.msg import PoseStamped

class GetTargetPositionNode:
    def __init__(self):
        rospy.init_node('get_target_position_node')

        # Service
        rospy.Service('/get_target_position', GetTargetPosition, self.get_target_position)

        # Variable
        self.target_pose = PoseStamped()

    def get_target_position(self, request):
        return self.target_pose.pose.position

if __name__ == '__main__':
    try:
        get_target_position_node = GetTargetPositionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion")

