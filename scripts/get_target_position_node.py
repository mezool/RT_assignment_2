#!/usr/bin/env python

import rospy
from assignment_2_2023.srv import GetTargetPosition, GetTargetPositionResponse
from assignment_2_2023.msg import Goal

class GetTargetPositionNode:
    def __init__(self):
        rospy.init_node('get_target_position_node')
        #set callback function of service
        self.get_target_position_service = rospy.Service('/get_target_info', GetTargetPosition, self.handle_get_target_position)
        #subscribe Goal.msg  
        self.latest_goal_msg = None      
        rospy.Subscriber('/target_info', Goal, self.target_callback)

        rospy.spin()

    def handle_get_target_position(self, request):
        if self.latest_goal_msg is not None:
            # substitute target_x and target_y to the response of GetTargetPosition.srv 
            response = GetTargetPositionResponse()
            response.target_x = self.latest_goal_msg.target_x
            response.target_y = self.latest_goal_msg.target_y
            return response
        else:
            rospy.logwarn("No target information available")
            return None

    def target_callback(self, goal_msg):
        # receive goal_msg
        self.latest_goal_msg = goal_msg
        rospy.loginfo("Received Goal message: target_x={}, target_y={}".format(goal_msg.target_x, goal_msg.target_y))


if __name__ == "__main__":
    try:
        get_target_position_node = GetTargetPositionNode()
    except rospy.ROSInterruptException:
        pass
