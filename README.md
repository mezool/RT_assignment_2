# RT Second Assignment
This repository contains the codes of the second assignment for Research Track I course.

# What I did
Apart from the codes already given, I made three python codes (client3.py, get_target_position_node.py, robot_status_node.py), msg files and srv files to run the python codes, and a launch file to launch action client and service nodes.

I added some comment out on some other codes I gave, but I don't change them.

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
