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
                    {'node_names': ['map_server', 'ego_agent_controller']}],
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
        name='ego_agent_controller',
        parameters=[{
                'controller_frequency': 30.0,
                'FollowPath': {
                    'plugin': 'nav2_mppi_controller::MPPIController',
                    'time_steps': 56,
                    'model_dt': 0.05,
                    'batch_size': 2000,
                    'vx_std': 5.0,
                    'vy_std': 5.0,
                    'wz_std': 1.0,
                    'vx_max': 20.0,
                    'vx_min': -5.0,
                    'vy_max': 20.0,
                    'wz_max': 1.9,
                    'iteration_count': 1,
                    'prune_distance': 1.7,
                    'transform_tolerance': 0.1,
                    'temperature': 0.3,
                    'gamma': 0.015,
                    'motion_model': 'Ackermann',
                    'visualize': True,
                    'TrajectoryVisualizer': {
                        'trajectory_step': 5,
                        'time_step': 3
                    },
                    'AckermannConstraints': {
                        'min_turning_r': 0.2
                    },
                    'critics': [
                        "ConstraintCritic", "CostCritic", "GoalCritic", 
                        "GoalAngleCritic", "PathAlignCritic", 
                        "PathFollowCritic", "PathAngleCritic", "PreferForwardCritic"
                    ],
                    'ConstraintCritic': {
                        'enabled': True,
                        'cost_power': 1,
                        'cost_weight': 4.0
                    },
                    'GoalCritic': {
                        'enabled': True,
                        'cost_power': 1,
                        'cost_weight': 5.0,
                        'threshold_to_consider': 1.4
                    },
                    'GoalAngleCritic': {
                        'enabled': True,
                        'cost_power': 1,
                        'cost_weight': 3.0,
                        'threshold_to_consider': 0.5
                    },
                    'PreferForwardCritic': {
                        'enabled': True,
                        'cost_power': 1,
                        'cost_weight': 5.0,
                        'threshold_to_consider': 0.5
                    },
                    'CostCritic': {
                        'enabled': True,
                        'cost_power': 1,
                        'cost_weight': 3.81,
                        'critical_cost': 300.0,
                        'consider_footprint': True,
                        'collision_cost': 1000000.0,
                        'near_goal_distance': 1.0,
                        'trajectory_point_step': 2
                    },
                    'PathAlignCritic': {
                        'enabled': True,
                        'cost_power': 1,
                        'cost_weight': 14.0,
                        'max_path_occupancy_ratio': 0.05,
                        'trajectory_point_step': 4,
                        'threshold_to_consider': 0.5,
                        'offset_from_furthest': 20,
                        'use_path_orientations': False
                    },
                    'PathFollowCritic': {
                        'enabled': True,
                        'cost_power': 1,
                        'cost_weight': 5.0,
                        'offset_from_furthest': 5,
                        'threshold_to_consider': 1.4
                    },
                    'PathAngleCritic': {
                        'enabled': True,
                        'cost_power': 1,
                        'cost_weight': 2.0,
                        'offset_from_furthest': 4,
                        'threshold_to_consider': 0.5,
                        'max_angle_to_furthest': 1.0,
                        'forward_preference': True
                    }
                }
            }],
            arguments=["--ros-args", "--log-level", "info"]
    )

    ego_agent_node = Node(
        package='agent',
        executable='mppiagent',
        name='ego_agent',
        parameters=[{'map_name': map},
                    {'map_folder_path': f'{maps_folder}/{map}'}],
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