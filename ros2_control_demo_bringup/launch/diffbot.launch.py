# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get URDF via xacro
    robot_localization_file_path = PathJoinSubstitution(
    	[
    		FindPackageShare("ros2_control_demo_bringup"),
            	"config",
            	"ekf.yaml"
        ]
    )
            		
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("diffbot_description"), "urdf", "diffbot.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_demo_bringup"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("diffbot_description"), "config", "diffbot.rviz"]
    )

# include realsense
    realsense_run = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("realsense2_camera"), '/launch', '/rs_launch.py']),
                    launch_arguments = {'pointcloud.enable':'true', 
                                        'enable_gyro':'true',
                                        'enable_accel':'true',
                                        'depth_module.profile':'640x480x30'}.items(),
            ),
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("/diff_drive_controller/cmd_vel", "/cmd_vel"),
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "-c", "/controller_manager"],
    )
  # Start robot localization using an Extended Kalman filter
    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path, 
        ])
        
    # Delay rviz start after `joint_state_broadcaster`
    delay_robot_control_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_localization = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_robot_localization_cmd,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rviz_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    nodes = [
        control_node,
        robot_state_pub_node,
        robot_controller_spawner,
        delay_robot_control_state,
        start_robot_localization_cmd,
        realsense_run,
        rviz_node,
    ]

    return LaunchDescription(nodes)
