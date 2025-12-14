# Experimental Robotics Lab - Assignment 1
This ROS 2 package contains the simulation and control nodes for an autonomous robot searching for ArUco markers. Developed for the Experimental Robotics Lab class, it allows you to run missions with either a 2-wheel or 4-wheel robot in Gazebo, using computer vision to detect targets and navigate the environment.

## Authors
- Luca Bricarello: 5248168
- Barbara Ratto: 4952202
- Francesca Magno: 5294873
- Martino Arecco: 5273621
- Filippo Gorini: 4828475

## Requirements
Ensure the following are installed and configured before running the simulation:
 - **ROS2 distro**: Jazzy
 - **Gazebo**
 - **Gazebo Models**: the `aruco_marker` model must be installed in your Gazebo model path. To get the model you can clone this [repository](https://github.com/LucaBricarello/aruco_box.git).
 - **cv_bridge**
 - **OpenCV (with ArUco module enabled)**: required for ArUco marker detection (`cv2.aruco`)
 - **tf-transformations**: used to convert orientation data from quaternions to Euler angles.

## How to run the code
### 1. Installation and Compilation
1. Source your ROS 2 environment.
2. Clone this repository into the `src/` folder of your ROS 2 workspace.
3. Build your workspace using colcon:
	```
	cd ~/<your_workspace>
	colcon build
	source install/setup.bash
	```
### 2. Choose a Mission
You can run the simulation with either a **2-wheel robot** or a **4-wheel (skid steer) robot**. Furthermore, there are two types of missions:
- **Long Mission**: When a marker is identified, the robot approaches it to increase the marker size within the modified image frame.
- **Short Mission**: The robot identifies the marker but does not approach it, this is faster but the marker size will be smaller in the modified image frame.
### 3. Launch Commands:
Once you have correctly built and sourced your workspace, you can run one of the following commands to launch the full simulation and the mission node:
- **Long Mission (2-Wheel Robot)**:
```
ros2 launch erl_assignment_1 simulation_and_mission.launch.py
```
- **Long Mission (4-Wheel Skid Steer Robot)**:
```
ros2 launch erl_assignment_1 simulation_and_mission_skid.launch.py
```
- **Short Mission (2-Wheel Robot)**:
```
ros2 launch erl_assignment_1 short_simulation_and_mission.launch.py
```
- **Short Mission (4-Wheel Skid Steer Robot)**:
```
ros2 launch erl_assignment_1 short_simulation_and_mission_skid.launch.py
```
### 4. Visualizing the Output
The simulation uses compressed images. To visualize the processed output or the robot's camera feed, use `rqt`:
- **Processed Image**: Subscribe to the topic `/robot/processed_image`.
- **Camera Feed**: View the raw camera topic via the Image View plugin.

> **Note:** A generic CV2 window displaying the modified image will also open automatically when the image is published.

## Videos of the missions
Below you can find demonstrations of the missions running both in the simulator and on a real robot (Husarion ROSbot 2).

**Short mission with 4 wheel skid steer robot**:

https://github.com/user-attachments/assets/44646f7d-8eec-4331-99ba-498bf0d11d79

**Long mission with 2-wheel robot**:

https://github.com/user-attachments/assets/c897d308-0eae-44c4-89da-0f537b42cb5d

**Test on a ROSBOT2 in the lab**:

https://github.com/user-attachments/assets/00b47534-8c44-469d-9178-79713b2685a8

## Package Overview
This package is made of many different components, each one responsible for a different aspect, like: mission execution, robot modeling, simulation setup and system launch orchestration.
Below is a brief rundown of the main components and of their functionalities.

### 1. Mission Nodes (`/erl_assignment_1`)
The mission logic is implemented through three dedicated ROS 2 nodes, all located in the `scripts/` folder. Each node corresponds to a different mission variant:

* **`mission_node`** (`mission.py`)
    * Implements the **Long Mission** logic. In this configuration, once an ArUco marker is detected, the robot actively navigates closer to the marker in order to increase its apparent size in the processed camera image.
* **`mission_node_short`** (`mission_short.py`)
    * Implements the **Short Mission** logic (robot identifies marker without approaching). The robot detects and processes the marker but does not perform an approach maneuver, resulting in a smaller marker appearance in the modified image stream.
* **`mission_node_real`** (`mission_real.py`)
    * Adapts the **Long Mission** logic for execution on a physical ROSbot2 robot. Compared to the simulation version, this node subscribes to a different camera feed topic and includes adjusted control gains, velocity limits, and distance thresholds to ensure slower and safer movement.

### 2. Robot Models (`/urdf`)
The `urdf/` folder contains the Universal Robot Description Format (URDF) files describing the two different robot model: 
* **2-Wheel Robot:** Standard differential drive chassis.
* **4-Wheel Robot:** Skid-steer chassis.

### 3. Simulation World (`/worlds`)
Contains the custom Gazebo world file designed for this assignment.
> **Note:** You must ensure the `aruco_marker` model is correctly installed in your Gazebo model path for the world to load without errors.

### 4. Launch Files (`/launch`)
The `launch/` folder contains Python-based ROS 2 launch files used to coordinate and start the entire system. These launch files are responsible for:
* Selecting and spawning the appropriate robot model (2-wheel or 4-wheel).
* Launching the Gazebo simulation environment with the custom world.
* Starting the appropriate mission node.
