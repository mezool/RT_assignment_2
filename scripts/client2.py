#!/usr/bin/env python

import rospy
import actionlib
from std_srvs.srv import SetBool
from nav_msgs import Odometry
from geometry_msgs.msg import PoseStamped, Pose, Twist, Point
from assignment_2_2023.msg import PlanningAction, PlanningGoal, PlanningFeedback, PlanningResult, Custom
#from assignment_2_2023-main import Custom

class ActionClientNode:
    def __init__(self):
        rospy.init_node('action_client_node_fake')

        # make action client
        self.client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        rospy.loginfo("client was made")

        # wait until server running
        self.client.wait_for_server()
        rospy.loginfo("client finish waiting")

        # Service clients for go_to_point_switch and wall_follower_switch
        self.go_to_point_switch = rospy.ServiceProxy('/go_to_point_switch', SetBool)
        self.wall_follower_switch = rospy.ServiceProxy('/wall_follower_switch', SetBool)
        rospy.loginfo("robot is moving")

        # Subsrcibers
        self.odom_pos = rospy.Subscriber('/odom', Odometry, self.odom_pos_callback)
        rospy.loginfo("odom subscribed")

        #Publishers
        self.robot_position_pub = rospy.Publisher('/robot_positionvelocity', Custom, queue_size=100)
        rospy.loginfo("Publisher created")


    def odom_pos_callback(self, data):
        x_position = data.pose.pose.Point.x
        y_position = data.pose.pose.Point.y
        x_vel = data.twist.twist.linear.x
        y_vel = data.twist.twist.linear.y
        
        # create custom message of type Custom
        self.msg = Custom()
        self.msg.robpos_x = x_position
        self.msg.robpos_y = y_position
        self.msg.robvel_x = x_vel
        self.msg.robvel_y = y_vel
        
        self.robot_position_pub.publish(self.msg)

    def send_goal(self, target_x, target_y):
        # make the goal of the action
        goal = PlanningGoal()
        goal.target_pose.pose.position.x = target_x  # x-coordinte of target
        goal.target_pose.pose.position.y = target_y  # x-coordinte of target
        goal.target_pose.pose.orientation.w = 1.0  # quaternion of therotation

        # send the goal of the action
        self.client.send_goal(goal, feedback_cb=self.feedback_callback)

        # confirm if user caccel the action
        cancel_input = input("Do you cancel the goal? (y/n): ")
        if cancel_input.lower() == 'y':
            self.client.cancel_goal()
            print("Goal has been canceled.")
            return

        # wait for the result of the action
        self.client.wait_for_result()

    def feedback_callback(self, feedback):
        position = feedback.actual_pose.position
        velocity = Twist()

        self.robot_position_pub.publish(position)
        self.robot_velocity_pub.publish(velocity)

        rospy.loginfo("Feedback status: %s", feedback.stat)

if __name__ == '__main__':
    try:
        # create an instance of the ActionClientNode class
        action_client_node = ActionClientNode()

        # desired postion assigned by user
        target_x = float(input("Enter target x position: "))
        target_y = float(input("Enter target y position: "))
 
        action_client_node.send_goal(target_x, target_y)

    except ValueError:
        print("Invalid input. Please enter numerical values.")		
    except rospy.ROSInterruptException:
        print("Program interrupted before completion")


