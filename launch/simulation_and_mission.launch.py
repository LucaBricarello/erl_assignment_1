import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_erl_assignment_1 = get_package_share_directory('erl_assignment_1')

    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_erl_assignment_1, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            # 'rviz': 'true',
        }.items()
    )

    mission_node = Node(
        package='erl_assignment_1',
        executable='mission_node',
        name='assignment_node',
        output='screen',
        parameters=[
            {'use_sim_time': True}       # Important to synchronize with Gazebo
        ]
    )

    mission_with_delay = TimerAction(
        period=10.0,                     # wait 10 seconds before starting the mission node
        actions=[mission_node]
    )

    ld = LaunchDescription()

    ld.add_action(spawn_robot_launch)
    ld.add_action(mission_with_delay)

    return ld