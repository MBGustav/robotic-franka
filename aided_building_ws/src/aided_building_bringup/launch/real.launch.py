"""Real Franka robot with the same interface as sim.launch.py.

    ros2 launch aided_building_bringup real.launch.py robot_ip:=<IP>
    ros2 launch aided_building_bringup real.launch.py use_fake_hardware:=true

Brings up the official franka_bringup/franka.launch.py (hardware, robot state,
franka_gripper) and adds fr3_arm_controller, so the team code sees the same
topics and actions as in Gazebo:
    /fr3_arm_controller/follow_joint_trajectory, /fr3_arm_controller/joint_trajectory
    /franka_gripper/{homing,move,grasp,gripper_action,stop}   (real robot only)
    /franka/joint_targets, /franka/go_safe_zone, ...           (aided_building_comm)

On the robot the arm is torque-controlled (fr3_arm_controller_effort.yaml);
with use_fake_hardware it is position-controlled, because the mock hardware
only follows position commands. The fake gripper only publishes joint states.

franka_bringup (and libfranka) is looked up only here, so the Gazebo sim does
not need it.
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


def launch_robot(context):
    config_dir = os.path.join(
        get_package_share_directory('aided_building_bringup'), 'config')
    fake = LaunchConfiguration('use_fake_hardware').perform(context) == 'true'
    arm_params = os.path.join(
        config_dir,
        'fr3_arm_controller_position.yaml' if fake else 'fr3_arm_controller_effort.yaml')

    franka = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('franka_bringup'), 'launch', 'franka.launch.py')),
        launch_arguments={
            'robot_type': LaunchConfiguration('robot_type'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'load_gripper': LaunchConfiguration('load_gripper'),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'controllers_yaml': os.path.join(config_dir, 'real_controllers.yaml'),
        }.items(),
    )

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='arm_controller_spawner',
        arguments=[
            'fr3_arm_controller',
            '--param-file', arm_params,
            '--controller-manager-timeout', '60',
        ],
        output='screen',
    )
    return [franka, arm_controller]


def generate_launch_description():
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', os.path.join(
                get_package_share_directory('franka_description'),
                'rviz', 'visualize_franka.rviz'),
            '-f', 'base',
        ],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )

    comm = Node(
        package='aided_building_comm',
        executable='comm_node',
        name='franka_comm',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_comm')),
        parameters=[{
            'use_sim_time': False,
            'auto_move_to_safe_zone': ParameterValue(
                LaunchConfiguration('auto_move_to_safe_zone'), value_type=bool),
            'move_duration': ParameterValue(
                LaunchConfiguration('move_duration'), value_type=float),
            'safe_zone_positions': LaunchConfiguration('safe_zone_joints'),
            'work_zone_positions': LaunchConfiguration('work_zone_joints'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip', default_value='172.16.0.2',
            description='Hostname or IP address of the robot'),
        DeclareLaunchArgument(
            'robot_type', default_value='fr3',
            description='Arm model (franka_description robot type)'),
        DeclareLaunchArgument(
            'use_fake_hardware', default_value='false',
            description='Mock hardware instead of the robot (no libfranka connection)'),
        DeclareLaunchArgument(
            'load_gripper', default_value='true',
            description='Franka Hand and the official franka_gripper node'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz'),
        DeclareLaunchArgument(
            'start_comm', default_value='true',
            description='Start the team command node (aided_building_comm)'),
        DeclareLaunchArgument(
            'auto_move_to_safe_zone', default_value='false',
            description='Move to SAFE_ZONE on startup (off on the real robot)'),
        DeclareLaunchArgument(
            'move_duration', default_value='3.0',
            description='Trajectory duration for comm_node moves [s]'),
        DeclareLaunchArgument(
            'safe_zone_joints',
            default_value='[0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398]',
            description='SAFE_ZONE joint positions (fr3_joint1..7)'),
        DeclareLaunchArgument(
            'work_zone_joints', default_value='[0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.5]',
            description='WORK_ZONE joint positions (fr3_joint1..7)'),

        OpaqueFunction(function=launch_robot),
        rviz,
        comm,
    ])
