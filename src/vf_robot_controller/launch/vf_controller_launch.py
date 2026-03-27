"""
vf_robot_controller_launch.py

Launches Nav2 controller_server with VFRobotController as the local planner.
Keeps global planner as the Nav2 default (NavFn / Hybrid-A*).

Usage:
  ros2 launch vf_robot_controller vf_controller_launch.py \\
    params_file:=/path/to/hospital_params.yaml \\
    use_3d:=true

Arguments:
  params_file   Path to your nav2 params YAML (default: hospital_params.yaml)
  use_3d        Enable 3D volumetric clearance (requires 3D sensor)
  log_level     Logging verbosity (default: info)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare('vf_robot_controller')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [pkg_share, 'config', 'hospital_params.yaml']),
        description='Full path to the Nav2 params YAML file')

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='ROS logging level (debug/info/warn/error)')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock')

    params_file   = LaunchConfiguration('params_file')
    log_level     = LaunchConfiguration('log_level')
    use_sim_time  = LaunchConfiguration('use_sim_time')

    # ── Controller Server ──────────────────────────────────────────────────
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('cmd_vel', 'cmd_vel'),
            ('odom',    'odom'),
        ]
    )

    # ── Lifecycle Manager for controller_server ────────────────────────────
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_controller',
        output='screen',
        parameters=[{
            'use_sim_time':  use_sim_time,
            'autostart':     True,
            'node_names':    ['controller_server']
        }]
    )

    return LaunchDescription([
        params_file_arg,
        log_level_arg,
        use_sim_time_arg,
        controller_server,
        lifecycle_manager,
    ])
