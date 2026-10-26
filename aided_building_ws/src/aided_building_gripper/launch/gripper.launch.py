"""Franka Hand bringup with one interface for simulation and hardware.

Both modes expose /franka_gripper/{homing,move,grasp,gripper_action,stop}.

    ros2 launch aided_building_gripper gripper.launch.py use_sim:=true
    ros2 launch aided_building_gripper gripper.launch.py use_sim:=false robot_ip:=<ip>
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def real_gripper(context):
    """Resolved only in real mode, so simulation does not need libfranka."""
    if LaunchConfiguration('use_sim').perform(context) == 'true':
        return []
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('franka_gripper'),
            'launch', 'gripper.launch.py')),
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip'),
            'robot_type': 'fr3',
            'namespace': '',
            'use_fake_hardware': 'false',
        }.items(),
    )]


def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')

    sim_config = os.path.join(
        get_package_share_directory('aided_building_gripper'),
        'config', 'sim_gripper.yaml')

    sim_gripper = Node(
        package='aided_building_gripper',
        executable='sim_gripper_server',
        name='franka_gripper',
        output='screen',
        parameters=[sim_config, {
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        condition=IfCondition(use_sim),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true',
                              description='true: Gazebo, false: real Franka Hand'),
        DeclareLaunchArgument('robot_ip', default_value='',
                              description='Robot IP (real mode only)'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use the Gazebo clock (sim mode only)'),
        sim_gripper,
        OpaqueFunction(function=real_gripper),
    ])
