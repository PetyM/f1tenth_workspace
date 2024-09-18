from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.logging import get_logger
from ament_index_python.packages import get_package_share_directory
import pathlib
import math
import random

def generate_launch_description():
    ld = LaunchDescription()
    OPPONENT = True
    MAPS_FOLDER = pathlib.Path(__file__).parent.resolve() / 'f1tenth_racetracks'
    LAP_LIMIT = 3.0
    TIME_LIMIT = -1.0

    MAPS = ['Spielberg', 'Nuerburgring', 'Melbourne']
    AGENTS = ['samplingagent', 'reactiveagent', 'samplingagentlimited']
    REACTIVE_DRIVERS = ['FTG', 'DEmax']

    time_delta = random.uniform(5.0, 10.0)
    map = 1
    agent = 0
    driver = 0
    opponent_agent = 1
    opponent_driver = 1
    velocity_limit = -1.0
    opponent_velocity_limit = 6.0

    STARTING_POSITIONS = {
        'Spielberg': {
            'sx': 0.0,
            'sy': -0.7,
            'stheta': math.radians(190.0),
            'sx1': -3.0,
            'sy1': -1.5,
            'stheta1': math.radians(190.0),
        },
        'Nuerburgring': {
            'sx': 0.0,
            'sy': -0.8,
            'stheta': math.radians(220.0),
            'sx1': -2.0,
            'sy1': -2.5,
            'stheta1': math.radians(220.0),
        },
        'Melbourne': {
            'sx': 0.0,
            'sy': -0.8,
            'stheta': math.radians(135.0),
            'sx1': -2.0,
            'sy1': 0.8,
            'stheta1': math.radians(135.0),
        }
    }

    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[{'simulate_opponent': OPPONENT},
                    {'map_path': MAPS[map]},
                    {'sx': STARTING_POSITIONS[MAPS[map]]['sx']},
                    {'sy': STARTING_POSITIONS[MAPS[map]]['sy']},
                    {'stheta': STARTING_POSITIONS[MAPS[map]]['stheta']},
                    {'sx1': STARTING_POSITIONS[MAPS[map]]['sx1']},
                    {'sy1': STARTING_POSITIONS[MAPS[map]]['sy1']},
                    {'stheta1': STARTING_POSITIONS[MAPS[map]]['stheta1']},
                    {'time_limit': TIME_LIMIT},
                    {'lap_limit': LAP_LIMIT},
                    {'start_time_delta': time_delta}],
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
        parameters=[{'yaml_filename': f'{MAPS_FOLDER}/{MAPS[map]}/{MAPS[map]}_map.yaml'},
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
        executable=AGENTS[agent],
        name='ego_agent',
        parameters=[{'opponent_present': OPPONENT},
                    {'map_name': MAPS[map]},
                    {'map_folder_path': f'{MAPS_FOLDER}/{MAPS[map]}'},
                    {'driver': REACTIVE_DRIVERS[driver]},
                    {'velocity_limit': velocity_limit}],
        arguments=["--ros-args", "--log-level", "warn"]
    )

    ld.add_action(rviz_node)
    ld.add_action(bridge_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    ld.add_action(ego_robot_publisher)
    ld.add_action(ego_agent_node)

    if OPPONENT:
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
            executable=AGENTS[opponent_agent],
            name='opp_agent',
            parameters=[{'map_name': MAPS[map]},
                        {'map_folder_path': f'{MAPS_FOLDER}/{MAPS[map]}'},
                        {'agent_namespace': 'opp_racecar'},
                        {'driver': REACTIVE_DRIVERS[opponent_driver]},
                        {'velocity_limit': opponent_velocity_limit}],
            arguments=["--ros-args", "--log-level", "warn"]
        )
        
        detector_node = Node(
            package='agent',
            executable='detector',
            name='detector',
            parameters=[],
            arguments=["--ros-args", "--log-level", "warn"]
        )

        ld.add_action(opp_agent_node)
        ld.add_action(opp_robot_publisher)
        ld.add_action(detector_node)


    get_logger().info(f"Launching '{AGENTS[agent]}' {f'({REACTIVE_DRIVERS[driver]})' if agent == 1 else ''} on map '{MAPS[map]}")

    return ld