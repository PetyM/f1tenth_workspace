from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import pathlib
import math

def generate_launch_description():
    ld = LaunchDescription()

    maps_folder = pathlib.Path(__file__).parent.resolve() / 'f1tenth_racetracks'

    map = 'Nuerburgring'
    opponent = False
    time_limit = -1.0
    lap_limit = -1.0
    start_time_delta = 10.0
    driver = 'PureFTG'

    positions = {
        'Spielberg': {
            'sx': 0.0,
            'sy': 0.0,
            'stheta': math.radians(190),
        },
        'Nuerburgring': {
            'sx': 0.0,
            'sy': 0.0,
            'stheta': math.radians(220.0),
        },
        'Melbourne': {
            'sx': 0.0,
            'sy': 0.0,
            'stheta': math.radians(135.0),
        }
    }

    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[{'simulate_opponent': opponent},
                    {'map_path': map},
                    {'sx': positions[map]['sx']},
                    {'sy': positions[map]['sy']},
                    {'stheta': positions[map]['stheta']},
                    {'sx1': positions[map]['sx']},
                    {'sy1': positions[map]['sy']},
                    {'stheta1': positions[map]['stheta']},
                    {'time_limit': time_limit},
                    {'lap_limit': lap_limit},
                    {'start_time_delta': start_time_delta}],
        arguments=["--ros-args", "--log-level", "warn"]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', './f1tenth_gym_ros/launch/gym_bridge.rviz',
                   "--ros-args", "--log-level", "warn"]
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': f'{maps_folder}/{map}/{map}_map.yaml'},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'output': 'screen'},
                    {'use_sim_time': True}],
        arguments=["--ros-args", "--log-level", "warn"]
    )

    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}],
        arguments=["--ros-args", "--log-level", "warn"]
    )

    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', './f1tenth_gym_ros/launch/ego_racecar.xacro'])}],
        remappings=[('/robot_description', 'ego_robot_description')],
        arguments=["--ros-args", "--log-level", "warn"]
    )

    ego_agent_node = Node(
        package='agent',
        executable='samplingagent',
        name='ego_agent',
        parameters=[{'opponent_present': opponent},
                    {'map_name': map},
                    {'map_folder_path': f'{maps_folder}/{map}'}],
        arguments=["--ros-args", "--log-level", "warn"]
    )

    ld.add_action(rviz_node)
    ld.add_action(bridge_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    ld.add_action(ego_robot_publisher)
    ld.add_action(ego_agent_node)

    if opponent:
        opp_robot_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='opp_robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', './f1tenth_gym_ros/launch/opp_racecar.xacro'])}],
            remappings=[('/robot_description', 'opp_robot_description')],
            arguments=["--ros-args", "--log-level", "warn"]
        )

        opp_agent_node = Node(
            package='agent',
            executable='deagent',
            name='opp_agent',
            parameters=[{'map_name': map},
                        {'map_folder_path': f'{maps_folder}/{map}'},
                        {'agent_namespace': 'opp_racecar'},
                        {'driver': driver}],
            arguments=["--ros-args", "--log-level", "warn"]
        )
        
        ld.add_action(opp_agent_node)
        ld.add_action(opp_robot_publisher)

    return ld