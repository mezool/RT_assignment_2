#!/usr/bin/env python

import rospy
from assignment_2_2023.srv import GetTargetPosition
from assignment_2_2023.msg import Goal

class GetTargetPositionNode:
    def __init__(self):
        rospy.init_node('get_target_position_node')

        self.target_msg = None

        rospy.Subscriber('/target_info', Goal, self.target_callback)

        self.get_target_position_service = rospy.Service(
            '/get_target_position', GetTargetPosition, self.handle_get_target_position
        )

    def target_callback(self, target_msg):
        self.target_msg = target_msg

    def handle_get_target_position(self, request):
        response = GetTargetPositionResponse()

        if self.target_msg is not None:
            response.target_x = self.target_msg.target_x
            response.target_y = self.target_msg.target_y
            response.success = True
        else:
            response.success = False

        return response

if __name__ == "__main__":
    try:
        get_target_position_node = GetTargetPositionNode()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
