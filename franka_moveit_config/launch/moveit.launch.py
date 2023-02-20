#  Copyright (c) 2021 Franka Emika GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

# This file is an adapted version of
# https://github.com/ros-planning/moveit_resources/blob/ca3f7930c630581b5504f3b22c40b4f82ee6369d/panda_moveit_config/launch/demo.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            Shutdown)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    moveit_config = (
            MoveItConfigsBuilder("franka")
            .joint_limits("config/joint_limits.yaml")
            .robot_description_semantic("srdf/franka.srdf")
            .robot_description(file_path=get_package_share_directory('franka_description') + '/robots/franka.urdf')
            .to_moveit_configs()
            )



    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict()
        ],
    )

    # RViz
    rviz_base = os.path.join(get_package_share_directory('franka_moveit_config'), 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'moveit.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[moveit_config.robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory('franka_moveit_config'),
        'config',
        'ros2_controllers.yaml',
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        remappings=[('joint_states', 'franka/joint_states')],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        on_exit=Shutdown(),
    )

    # Load controllers
    load_controllers = []
    for controller in ['panda_arm_controller', 'joint_state_broadcaster']:
        load_controllers += [
            ExecuteProcess(
                cmd=['ros2 run controller_manager spawner {}'.format(controller)],
                shell=True,
                output='screen',
            )
        ]

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'source_list': ['franka/joint_states', 'panda_gripper/joint_states'], 'rate': 30}],
    )

    #gripper_launch_file = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource([PathJoinSubstitution(
    #        [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
    #    launch_arguments={'robot_ip': 192.168.106.39,
    #                      use_fake_hardware_parameter_name: false}.items(),
    #)
    return LaunchDescription(
        [
         rviz_node,
         robot_state_publisher,
         run_move_group_node,
         ros2_control_node,
         joint_state_publisher,
    #     gripper_launch_file
         ]
        + load_controllers
    )
