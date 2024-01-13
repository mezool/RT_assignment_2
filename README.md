# RT Second Assignment
This repository contains the codes of the second assignment for Research Track I course.

# What I did
Apart from the codes already given, I made three python codes (client3.py, get_target_position_node.py, robot_status_node.py), msg files and srv files to run the python codes, and a launch file to launch action client and service nodes.

I added some comment out on some other codes given for my understanding, but I don't change them.

# How to run
To start Rviz, Gazebo and Action Service Node, please start the launch file with the following code. 
```bash
roslaunch assignment_2_2023 assignment1.launch
```
Then please start the next launch file which starts action client node and service nodes with the following code.
```bash
roslaunch assignment_2_2023 client.launch window_size:=5
```
In this launch file, there is an arg `window_size' (the size of the averaging window to calculate the robot's average speed.). In this case, the value is set as 5. If the argument is not set, the parameter takes the default value of 10.

After starting these codes, the action client node ask you the target position by saying "Enter the target x-coordinate:" and "Enter the target y-coordinate:". That's how you can set the target position.
After setting the position, it asks you that "Do you cancel the goal? (y/n):" and if you type "y", you can cancel the goal.

# Calling the service nodes
## Get the last target (service)
To get the latest target set by the user, you have to call `/get_target_info` with the following code.
```bash
rosservice call /get_target_info
```
## Get target distance and average velocity.
To get the target distance and average velocity, you have to call `/get_robot_status` with the following code.
```bash
rosservice call /get_robot_status
```

# Structure of the action client code
These codes are built by Python scripts.
The pseudocode of the action client node is as follows:
```bash
# Initialize ROS node and create necessary components
initialize ROS node 'action_client_node'
create SimpleActionClient for '/reaching_goal'
create publishers for '/robot_info' and '/target_info'
subscribe to '/odom' topic with odom_callback function

# Define callback function for receiving odometry information
def odom_callback(odom_msg):
    extract relevant information from odometry message
    create Custom message with extracted information
    publish Custom message to '/robot_info'

# Define method for setting the goal interactively
def set_goal_interactively():
    prompt user for target coordinates
    create PlanningGoal and Goal messages with target coordinates
    publish Goal message to '/target_info'
    send goal to action server

    allow user to cancel the goal
    wait for the result or cancellation
    print the final result

# Define callback function for receiving feedback from the action server
def feedback_callback(feedback):
    handle feedback if needed
    print received feedback

# Start the main execution
if __name__ is "__main__":
    try:
        create instance of ActionClientNode
        set a goal interactively
        keep the node alive

    except ROSInterruptException:
        handle ROSInterruptException
```

# Further improvements
Author doesn't have prerequisite knowledge of ROS, so these codes may not be so elegant. The naming of nodes and variables could have been made a little easier to understand.
