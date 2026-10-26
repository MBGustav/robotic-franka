import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def build_robot_state_publisher(context, xyz, rpy, use_sim_time):
    xyz_value = context.perform_substitution(xyz)
    rpy_value = context.perform_substitution(rpy)
    use_sim_time_value = context.perform_substitution(use_sim_time) == 'true'

    xacro_file = os.path.join(
        get_package_share_directory('aided_building_description'),
        'urdf',
        'franka_gazebo.urdf.xacro')
    robot_description = xacro.process_file(
        xacro_file,
        mappings={
            'xyz': xyz_value,
            'rpy': rpy_value,
        }).toxml()

    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time_value,
        }],
    )]


def generate_launch_description():
    xyz = LaunchConfiguration('xyz')
    rpy = LaunchConfiguration('rpy')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_comm = LaunchConfiguration('start_comm')
    scenario = LaunchConfiguration('scenario')
    spawn_operator_hand = LaunchConfiguration('spawn_operator_hand')
    hand_x = LaunchConfiguration('hand_x')
    hand_y = LaunchConfiguration('hand_y')
    hand_z = LaunchConfiguration('hand_z')
    hand_roll = LaunchConfiguration('hand_roll')
    hand_pitch = LaunchConfiguration('hand_pitch')
    hand_yaw = LaunchConfiguration('hand_yaw')
    auto_move_to_safe_zone = LaunchConfiguration('auto_move_to_safe_zone')
    startup_delay = LaunchConfiguration('startup_delay')
    move_duration = LaunchConfiguration('move_duration')
    safe_zone_joints = LaunchConfiguration('safe_zone_joints')
    work_zone_joints = LaunchConfiguration('work_zone_joints')
    start_gripper = LaunchConfiguration('start_gripper')
    spawn_object = LaunchConfiguration('object')

    franka_share = get_package_share_directory('franka_description')
    description_share = get_package_share_directory('aided_building_description')
    resource_path = ':'.join(filter(None, [
        os.path.dirname(description_share),
        os.path.join(description_share, 'models'),
        os.path.dirname(franka_share),
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    ]))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        remappings=[('/clock', '/clock_raw')],
        output='screen',
    )

    clock_filter = Node(
        package='aided_building_comm',
        executable='clock_filter',
        name='clock_filter',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_franka',
        arguments=['-topic', '/robot_description', '-name', 'franka'],
        output='screen',
    )

    load_controllers = Node(
        package='controller_manager',
        executable='spawner',
        name='controller_spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager-timeout', '300',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='arm_controller_spawner',
        arguments=[
            'fr3_arm_controller',
            'fr3_gripper_controller',
            '--controller-manager-timeout', '300',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', os.path.join(franka_share, 'rviz', 'visualize_franka.rviz'),
            '-f', 'world',
        ],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    comm = Node(
        package='aided_building_comm',
        executable='comm_node',
        name='franka_comm',
        output='screen',
        condition=IfCondition(start_comm),
        parameters=[{
            'use_sim_time': True,
            'auto_move_to_safe_zone': ParameterValue(auto_move_to_safe_zone, value_type=bool),
            'startup_delay': ParameterValue(startup_delay, value_type=float),
            'move_duration': ParameterValue(move_duration, value_type=float),
            'safe_zone_positions': safe_zone_joints,
            'work_zone_positions': work_zone_joints,
        }],
    )

    spawn_operator_hand = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_operator_hand',
        arguments=[
            '-name', 'operator_hand',
            '-file', os.path.join(get_package_share_directory('aided_building_description'), 'models', 'operator_hand.sdf'),
            '-x', hand_x,
            '-y', hand_y,
            '-z', hand_z,
            '-R', hand_roll,
            '-P', hand_pitch,
            '-Y', hand_yaw,
        ],
        output='screen',
        condition=IfCondition(spawn_operator_hand),
    )

    # Both poses put the cylinder between the open fingers of the initial pose
    # (hand TCP at x = 0.307, z = 0.487, fingers closing along y).
    spawn_grasp_object = [
        Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{name}',
            arguments=[
                '-name', name,
                '-file', os.path.join(description_share, 'models', f'{name}.sdf'),
                '-x', '0.307', '-y', '0.0', '-z', z, '-P', pitch,
            ],
            output='screen',
            condition=IfCondition(spawn_object),
        )
        for name, z, pitch in (
            ('grasp_stand', '0.2385', '0.0'),
            ('grasp_cylinder', '0.487', '1.570796'),
        )
    ]

    gripper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('aided_building_gripper'),
            'launch', 'gripper.launch.py')),
        launch_arguments={
            'use_sim': 'true',
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(start_gripper),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gz_args', default_value='-r empty.sdf',
            description="Gazebo arguments ('-r -s empty.sdf' runs without GUI)"),
        DeclareLaunchArgument(
            'xyz', default_value='0 0 0',
            description='Robot base position in the world'),
        DeclareLaunchArgument(
            'rpy', default_value='0 0 0',
            description='Robot base orientation in the world'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo clock'),
        DeclareLaunchArgument(
            'start_comm', default_value='true',
            description='Start the simple ROS 2 command node'),
        DeclareLaunchArgument(
            'start_gripper', default_value='true',
            description='Start the simulated gripper server (/franka_gripper/*)'),
        DeclareLaunchArgument(
            'object', default_value='false',
            description='Spawn a 20 mm cylinder between the fingers to test grasp'),
        DeclareLaunchArgument(
            'scenario', default_value='scenario_1',
            description='Scenario profile from zones.yaml (scenario_1/2/3)'),
        DeclareLaunchArgument(
            'spawn_operator_hand', default_value='true',
            description='Spawn operator hand/PCB model in Gazebo'),
        DeclareLaunchArgument(
            'hand_x', default_value='0.4',
            description='Operator hand X position'),
        DeclareLaunchArgument(
            'hand_y', default_value='0.0',
            description='Operator hand Y position'),
        DeclareLaunchArgument(
            'hand_z', default_value='0.25',
            description='Operator hand Z position'),
        DeclareLaunchArgument(
            'hand_roll', default_value='0.0',
            description='Operator hand roll'),
        DeclareLaunchArgument(
            'hand_pitch', default_value='0.0',
            description='Operator hand pitch'),
        DeclareLaunchArgument(
            'hand_yaw', default_value='0.0',
            description='Operator hand yaw'),
        DeclareLaunchArgument(
            'auto_move_to_safe_zone', default_value='true',
            description='Automatically move to SAFE_ZONE on startup'),
        DeclareLaunchArgument(
            'startup_delay', default_value='10.0',
            description='Delay before auto-moving to SAFE_ZONE'),
        DeclareLaunchArgument(
            'move_duration', default_value='3.0',
            description='Trajectory duration for moves'),
        DeclareLaunchArgument(
            'safe_zone_joints', default_value='[0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398]',
            description='SAFE_ZONE joint positions (fr3_joint1..7)'),
        DeclareLaunchArgument(
            'work_zone_joints', default_value='[0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.5]',
            description='WORK_ZONE joint positions (fr3_joint1..7)'),

        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        gazebo,
        clock_bridge,
        clock_filter,
        OpaqueFunction(
            function=build_robot_state_publisher,
            args=[xyz, rpy, use_sim_time]),
        spawn_robot,
        spawn_operator_hand,
        *spawn_grasp_object,
        TimerAction(period=5.0, actions=[load_controllers]),
        TimerAction(period=10.0, actions=[load_arm_controller]),
        rviz,
        comm,
        gripper,
    ])
