# Authors

- Luca Bricarello 5248168
- Barbara Ratto 4952202
- Francesca Magno 5294873
- Martino Arecco 5273621
- Filippo Gorini 4828475

# Requirements

 - ROS2 jazzy
 - install aruco_marker model in gazebo model path

# How to run the code
After sourcing the right setup files, to run the code you simply need to clone this repository inside your ROS workspace, compile it with colcon build, then choose which simulation to run:

- long mission with 2 wheels robot
- long mission with 4 wheels robot
- short mission with 2 wheels robot
- short mission with 4 wheels robot

The difference between the long and the short mission is simple.

In the long mission once the robot has identified a marker it gets closer to it to make it bigger in the modified image.

In the short mission the robot does not get closer, so the marker will be smaller in the modified image.
The commands to launch this missions are:

	- ros2 launch erl_assignment_1 simulation_and_mission.launch.py
	- ros2 launch erl_assignment_1 simulation_and_mission_skid.launch.py
	- ros2 launch erl_assignment_1 short_simulation_and_mission.launch.py
	- ros2 launch erl_assignment_1 short_simulation_and_mission_skid.launch.py

This command will launch the whole simulation plus the node that implements the required task.

The simulation works with compressed images, so to view the camera of the robot rqt can be used.
rqt can also be used to look for the modified images at the topic '/robot/processed_image'.

The modified image is also shown through a cv2 window that opens when the image is pubblished.

# Videos of the missions
Here are 2 videos that show how the 2 of the missions work on the simulator.

Video of the short mission with 4 wheel skid steer robot

https://github.com/user-attachments/assets/44646f7d-8eec-4331-99ba-498bf0d11d79

Video of the long mission with 2 wheel robot

https://github.com/user-attachments/assets/c897d308-0eae-44c4-89da-0f537b42cb5d

Video of test done on the real robot in lab

<video src="https://github.com/LucaBricarello/erl_assignment_1/raw/main/media/real_robot_test.mp4" controls="controls" style="max-width: 100%;">
</video>

# Small description of the Package
The package has a lot of components this is a brief overview of the main ones.

## Mission nodes
There are 3 mission nodes.

mission_node (mission.py), this node implements the long mission.

mission_node_short (mission_node_short.py), this node implements the short mission.

mission_real (mission_real.py), this node has the parameters used to run the mission on the real robots in class.

## Robot models
In the urdf folder there are the files related to 2 different robots, the one with 2 wheels and the one with 4 wheels.

## World
In the worlds folder there is the file of the custom gazebo world created for this assignment.
Pay attention you need to install the aruco marker.

## Launch files
In the launch folder there are all the launch files used to run the simulation.


