from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_actions = []

    launch_actions.extend([
        DeclareLaunchArgument('namespace', default_value='',
                              description='Namespace for all CAN nodes'),
        
        # For sender and recveiver
        DeclareLaunchArgument('enable_can_fd', default_value='false',
                              description='Enable CAN FD mode for all interfaces'),
        DeclareLaunchArgument('use_bus_time', default_value='false',
                              description='Use bus time for all nodes'),

        # For recveiver
        DeclareLaunchArgument('enable_frame_loopback', default_value='false',
                              description='Enable frame loopback for receivers'),
        DeclareLaunchArgument('interval_sec', default_value='5.0',
                              description='Interval in seconds for all nodes'),
        
        DeclareLaunchArgument('filters', default_value='0:0',
                              description='CAN filters'),

        # For sender
        DeclareLaunchArgument('timeout_sec', default_value='0.01',
                              description=''),

        # For node lifecycle
        DeclareLaunchArgument('auto_configure', default_value='true',
                              description='Auto-configure lifecycle nodes'),
        DeclareLaunchArgument('auto_activate', default_value='true',
                              description='Auto-activate lifecycle nodes'),
    ])

    can_brigde_pkg = FindPackageShare('can_brigde')

    interfaces = ['can2']

    for interface in interfaces:
        receiver_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([can_brigde_pkg, 'launch', 'socket_can_receiver.launch.py'])
            ),
            launch_arguments={
                'interface': interface,
                'enable_can_fd': LaunchConfiguration('enable_can_fd'),
                'namespace': LaunchConfiguration('namespace'),
                'enable_frame_loopback': LaunchConfiguration('enable_frame_loopback'),
                'interval_sec': LaunchConfiguration('interval_sec'),
                'use_bus_time': LaunchConfiguration('use_bus_time'),
                'filters': LaunchConfiguration('filters'),
                'auto_configure': LaunchConfiguration('auto_configure'),
                'auto_activate': LaunchConfiguration('auto_activate'),
                'node_name': [TextSubstitution(text='socketcan_receiver_'), TextSubstitution(text=interface)],
            }.items()
        )

        sender_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([can_brigde_pkg, 'launch', 'socket_can_sender.launch.py'])
            ),
            launch_arguments={
                'interface': interface,
                'enable_can_fd': LaunchConfiguration('enable_can_fd'),
                'namespace': LaunchConfiguration('namespace'),
                'timeout_sec': LaunchConfiguration('timeout_sec'),
                'use_bus_time': LaunchConfiguration('use_bus_time'),
                'auto_configure': LaunchConfiguration('auto_configure'),
                'auto_activate': LaunchConfiguration('auto_activate'),
                'node_name': [TextSubstitution(text='socketcan_sender_'), TextSubstitution(text=interface)],
            }.items()
        )

        launch_actions.extend([receiver_launch, sender_launch])

    return LaunchDescription(launch_actions)