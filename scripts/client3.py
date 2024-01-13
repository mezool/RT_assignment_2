#!/usr/bin/env python

import rospy
import actionlib
from std_msgs.msg import Header
from assignment_2_2023.msg import PlanningAction, PlanningGoal, Custom, Goal
from nav_msgs.msg import Odometry
from assignment_2_2023.srv import GetTargetPosition

class ActionClientNode:
    def __init__(self):
        #initialize ROS node
        rospy.init_node('action_client_node')

        #Create action client node
        self.action_client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        self.action_client.wait_for_server()

        # Publish Custom msg
        self.robot_info_pub = rospy.Publisher('/robot_info', Custom, queue_size=10)
        self.robot_target_pub = rospy.Publisher('/target_info', Goal, queue_size=10)

        # Subscribe information
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def odom_callback(self, odom_msg):
        # Publish the msgs of robot's position and velocity
        custom_msg = Custom()
        custom_msg.robpos_x = odom_msg.pose.pose.position.x
        custom_msg.robpos_y = odom_msg.pose.pose.position.y
        custom_msg.robvel_x = odom_msg.twist.twist.linear.x
        custom_msg.robvel_y = odom_msg.twist.twist.linear.y

        self.robot_info_pub.publish(custom_msg)

    def set_goal_interactively(self):
        target_x = float(input("Enter the target x-coordinate: "))
        target_y = float(input("Enter the target y-coordinate: "))

        # set goal position and publish the msg
        
        goal = PlanningGoal()
        goal.target_pose.header = Header()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = target_x
        goal.target_pose.pose.position.y = target_y
        goal.target_pose.pose.orientation.w = 1.0  # Facing forward
        
        target_msg = Goal()
        target_msg.target_x = target_x
        target_msg.target_y = target_y

        self.robot_target_pub.publish(target_msg)        
        # self.action_client.send_goal(goal, feedback_cb=self.feedback_callback) # for debug
        self.action_client.send_goal(goal)

        # code about cancel the goal
        cancel_input = input("Do you cancel the goal? (y/n): ")
        if cancel_input.lower() == 'y':
            self.action_client.cancel_goal()
            print("Goal has been canceled.")
            return
        # Wait for the result or cancellation
        self.action_client.wait_for_result()

        # Print the final result
        print("Goal Status:", self.action_client.get_state())
        print("Goal Result:", self.action_client.get_result())

    def feedback_callback(self, feedback):
        # Handle feedback here if needed
        print("Feedback received:", feedback) # for debug

if __name__ == "__main__":
    try:
        client_node = ActionClientNode()

        # Example: Set a goal interactively
        client_node.set_goal_interactively()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass