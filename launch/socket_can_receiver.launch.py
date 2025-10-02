from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def create_can_receiver_node(interface, node_name, enable_can_fd, namespace):
    """
    Helper function to create a LifecycleNode and its event handlers for a given CAN interface.
    """
    
    node = LifecycleNode(
        package='ros2_socketcan',
        executable='socket_can_receiver_node_exe',
        name=node_name,
        namespace=namespace,
        parameters=[{
            'interface': interface,
            'enable_can_fd': enable_can_fd,
            'enable_frame_loopback': LaunchConfiguration('enable_frame_loopback'),
            'interval_sec': LaunchConfiguration('interval_sec'),
            'filters': LaunchConfiguration('filters'),
            'use_bus_time': LaunchConfiguration('use_bus_time'),
        }],
        remappings=[
            ('from_can_bus', [interface, TextSubstitution(text='/from_can_bus')]),
            ('from_can_bus_fd', [interface, TextSubstitution(text='/from_can_bus_fd')])
        ],
        output='screen'
    )

    configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_configure')),
    )

    activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_activate')),
    )

    return [node, configure_event_handler, activate_event_handler]


def generate_launch_description():
    launch_actions = []

    launch_actions.extend([
        DeclareLaunchArgument('namespace', description='Namespace for CAN node'),
        DeclareLaunchArgument('node_name', description='Node name'),

        DeclareLaunchArgument('interface', description='CAN interface'),
        DeclareLaunchArgument('enable_can_fd', description='Enable CAN FD mode'),
        
        DeclareLaunchArgument('enable_frame_loopback', description='Enable frame loopback'),
        DeclareLaunchArgument('interval_sec', description='Interval in seconds'),
        DeclareLaunchArgument('use_bus_time', description='Use bus time'),
        DeclareLaunchArgument('filters', description='CAN filters'),

        DeclareLaunchArgument('auto_configure', description='Auto-configure lifecycle node'),
        DeclareLaunchArgument('auto_activate', description='Auto-activate lifecycle node'),
    ])

    launch_actions.extend(
        create_can_receiver_node(
            interface=LaunchConfiguration('interface'),
            node_name=[
                LaunchConfiguration('interface'), TextSubstitution(text='_receiver')],
            enable_can_fd=LaunchConfiguration('enable_can_fd'),
            namespace=LaunchConfiguration('namespace')
        )
    )

    return LaunchDescription(launch_actions)