#!/usr/bin/env python

import rospy
from assignment_2_2023.srv import GetRobotStatus
from assignment_2_2023.msg import Goal, Custom
from nav_msgs.msg import Odometry

class RobotStatusNode:
    def __init__(self):
        rospy.init_node('robot_status_node')

        # set default value of parameter '/window_size' 
        self.window_size = rospy.get_param('window_size', 10)

        # set service and callback function
        self.robot_status_service = rospy.Service('/get_robot_status', GetRobotStatus, self.handle_robot_status)

        # set subscriber
        rospy.Subscriber('/target_info', Goal, self.target_callback)
        rospy.Subscriber('/robot_info', Custom, self.robot_info_callback)

        # list of velocity
        self.robot_speeds = []

        # initialize the robot's position
        self.robot_position = None

        rospy.spin()

    def target_callback(self, goal_msg):
        # Processing upon receipt of a Goal message
        self.target_position = (goal_msg.target_x, goal_msg.target_y)

    def robot_info_callback(self, custom_msg):
        # Processing upon receipt of a Custom message
        self.robot_position = (custom_msg.robpos_x, custom_msg.robpos_y)
        robot_speed = (custom_msg.robvel_x**2 + custom_msg.robvel_y**2)**0.5
        self.robot_speeds.append(robot_speed)

    def handle_robot_status(self, request):
        # calculate the distance
        if hasattr(self, 'target_position') and hasattr(self, 'robot_position'):
            distance_to_target = ((self.target_position[0] - self.robot_position[0])**2 +
                                  (self.target_position[1] - self.robot_position[1])**2)**0.5
        else:
            rospy.logwarn("Target or robot position not available.")
            distance_to_target = 0.0

        # calculate the average speed
        if len(self.robot_speeds) > 0:
            average_speed = sum(self.robot_speeds[-self.window_size:]) / len(self.robot_speeds[-self.window_size:])
        else:
            rospy.logwarn("No robot speed information available.")
            average_speed = 0.0

        return distance_to_target, average_speed

if __name__ == "__main__":
    try:
        robot_status_node = RobotStatusNode()
    except rospy.ROSInterruptException:
        pass
