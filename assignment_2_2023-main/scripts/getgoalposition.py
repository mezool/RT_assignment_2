#!/usr/bin/env python

import rospy
from assignment_2_2023.srv import GetTargetPosition, GetTargetPositionResponse
from assignment_2_2023.msg import PlanningGoal
from geometry_msgs.msg import PoseStamped

class GetGoalPositionService:
    def __init__(self):
        rospy.init_node('get_goal_position_service')
        
        # Create the service server
        self.get_goal_position_service = rospy.Service(
            '/get_goal_position', GetTargetPosition, self.get_goal_position_callback)

        # self.goal_position = Point()
        
    def get_goal_position_callback(self,request):
        # Extract the goal position from the received PlanningGoal message
        target_x = request.target_pose.pose.position.x
        target_y = request.target_pose.pose.position.y

        response = GetTargetPositionResponse()
        response.target_x = target_x
        response.target_y = target_y
        #response.goal_position = goal_position

        return response

if __name__ == "__main__":
    try:
        get_goal_position_service = GetGoalPositionService()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass