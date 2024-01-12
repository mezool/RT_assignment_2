#! /usr/bin/env python

import rospy
import actionlib
from assignment_2_2023.msg import PlanningAction, PlanningGoal
from geometry_msgs.msg import Point

def feedback_callback(feedback):
    #deal with the feedback
    print("Received feedback. Current state:", feedback.stat)

def action_client(target_x, target_y):
    # initialize the ROS node
    rospy.init_node('action_client_nodee')

    # make action client
    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)

    # wait until server running
    client.wait_for_server()

    #set the callback function
    client.feedback_callback = feedback_callback

    # make the goal of the action
    goal = PlanningGoal()
    goal.target_pose.pose.position.x = target_x  # x-coordinte of target
    goal.target_pose.pose.position.y = target_y  # x-coordinte of target
    goal.target_pose.pose.orientation.w = 1.0  # quaternion of therotation

    # send the goal of the action
    client.send_goal(goal)

    # confirm if user caccel the action
    cancel_input = input("Do you cancel the goal? (y/n): ")
    if cancel_input.lower() == 'y':
        client.cancel_goal()
        print("Goal has been canceled.")
        return

    # wait for the result of the action
    client.wait_for_result()

    # get the result of the action
    result = client.get_result()

    # show the result
    print("Action Result:", result)

if __name__ == '__main__':
    try:
        # desired postion assigned by user
        target_x = float(input("Enter target x position: "))
        target_y = float(input("Enter target y position: "))
 
        action_client(target_x, target_y)
    except ValueError:
        print("Invalid input. Please enter numerical values.")		
    except rospy.ROSInterruptException:
        print("Program interrupted before completion")

