from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import pathlib

def generate_launch_description():
    ld = LaunchDescription()

    maps_folder = pathlib.Path(__file__).parent.resolve() / 'f1tenth_racetracks'
    map = 'Spielberg'
    opponent = False

    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[{'simulate_opponent': opponent},
                    {'map_path': f'{maps_folder}/{map}/{map}_map.png'}],
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

    mppi_controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='ego_agent',
        parameters=[{'planner_plugin': "nav2_mppi_controller/MPPIController"},
                    {'max_vel_x': 0.5},
                    {'max_vel_y': 0.0},
                    {'max_vel_theta': 1.0},
                    {'min_vel_x': 0.0},
                    {'min_vel_theta': -1.0},
                    {'acc_lim_x': 0.5},
                    {'acc_lim_theta': 0.5}]
    )

    ego_agent_node = Node(
        package='agent',
        executable='mppiagent',
        name='opp_agent',
        parameters=[{'map_name': map},
                    {'map_folder_path': f'{maps_folder}/{map}'},
                    {'agent_namespace': 'opp_racecar'}],
        arguments=["--ros-args", "--log-level", "warn"]
    )

    ld.add_action(rviz_node)
    ld.add_action(bridge_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    ld.add_action(ego_robot_publisher)
    ld.add_action(mppi_controller_node)
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
            executable='purepursuitagent',
            name='opp_agent',
            parameters=[{'map_name': map},
                        {'map_folder_path': f'{maps_folder}/{map}'},
                        {'agent_namespace': 'opp_racecar'},
                        {'velocity_gain': 0.5}],
            arguments=["--ros-args", "--log-level", "warn"]
        )
        
        ld.add_action(opp_agent_node)
        ld.add_action(opp_robot_publisher)

    return ld