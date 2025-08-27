#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os

def generate_launch_description():
    # Declare launch arguments for system configuration
    declare_use_gpu = DeclareLaunchArgument(
        'use_gpu',
        default_value='true',
        description='Whether to use GPU for lane detection'
    )
    declare_debug_mode = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        description='Enable debug logging'
    )
    declare_save_data = DeclareLaunchArgument(
        'save_data',
        default_value='true',
        description='Save performance data and plots'
    )
    declare_stanley_gain = DeclareLaunchArgument(
        'stanley_gain',
        default_value='0.5',
        description='Stanley controller gain parameter'
    )
    declare_pid_kp = DeclareLaunchArgument(
        'pid_kp',
        default_value='0.1',
        description='PID proportional gain'
    )
    declare_pid_ki = DeclareLaunchArgument(
        'pid_ki',
        default_value='0.01',
        description='PID integral gain'
    )
    declare_pid_kd = DeclareLaunchArgument(
        'pid_kd',
        default_value='0.05',
        description='PID derivative gain'
    )
    declare_lookahead_distance = DeclareLaunchArgument(
        'lookahead_distance',
        default_value='5.0',
        description='Lookahead distance for path following'
    )
    declare_desired_speed = DeclareLaunchArgument(
        'desired_speed',
        default_value='30.0',
        description='Desired vehicle speed in km/h'
    )
    declare_max_steering_angle = DeclareLaunchArgument(
        'max_steering_angle',
        default_value='0.5',
        description='Maximum steering angle in radians'
    )

    # Get launch configurations
    use_gpu = LaunchConfiguration('use_gpu')
    debug_mode = LaunchConfiguration('debug_mode')
    stanley_gain = LaunchConfiguration('stanley_gain')
    pid_kp = LaunchConfiguration('pid_kp')
    pid_ki = LaunchConfiguration('pid_ki')
    pid_kd = LaunchConfiguration('pid_kd')
    lookahead_distance = LaunchConfiguration('lookahead_distance')
    desired_speed = LaunchConfiguration('desired_speed')
    max_steering_angle = LaunchConfiguration('max_steering_angle')

    # Lane Detection Node (NO remapping, uses original topics)
    lane_detection_node = Node(
        package='lka',  # Replace with your actual package name
        executable='lanedetection.py',
        name='lane_detection_node',
        output='screen',
        parameters=[
            {'use_gpu': use_gpu}
        ],
        # No remappings block here!
        arguments=['--ros-args', '--log-level', 'INFO'] if not debug_mode else 
                 ['--ros-args', '--log-level', 'DEBUG']
    )

    # Lane Keeping Assist Node (NO remapping, uses original topics)
    lane_keeping_assist_node = TimerAction(
        period=2.0,  # 2 second delay
        actions=[
            Node(
                package='lka',  # Replace with your actual package name
                executable='lka.py',
                name='lane_keeping_assist_node',
                output='screen',
                parameters=[
                    {'k': stanley_gain},
                    {'pid_kp': pid_kp},
                    {'pid_ki': pid_ki},
                    {'pid_kd': pid_kd},
                    {'lookahead_distance': lookahead_distance},
                    {'smoothing_window': 10},
                    {'max_steering_angle': max_steering_angle},
                    {'desired_speed': desired_speed}
                ],
                # No remappings block here!
                arguments=['--ros-args', '--log-level', 'INFO'] if not debug_mode else 
                         ['--ros-args', '--log-level', 'DEBUG']
            )
        ]
    )

    # Optional: RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(os.path.dirname(__file__), 'lane_keeping_rviz.rviz')],
        condition=IfCondition(LaunchConfiguration('debug_mode')),
        output='screen'
    )

    # Optional: RQT for parameter tuning
    rqt_reconfigure_node = ExecuteProcess(
        cmd=['rqt', '--standalone', 'rqt_reconfigure'],
        condition=IfCondition(LaunchConfiguration('debug_mode')),
        output='screen'
    )

    # Create launch description
    ld = LaunchDescription()
    ld.add_action(declare_use_gpu)
    ld.add_action(declare_debug_mode)
    ld.add_action(declare_save_data)
    ld.add_action(declare_stanley_gain)
    ld.add_action(declare_pid_kp)
    ld.add_action(declare_pid_ki)
    ld.add_action(declare_pid_kd)
    ld.add_action(declare_lookahead_distance)
    ld.add_action(declare_desired_speed)
    ld.add_action(declare_max_steering_angle)
    ld.add_action(lane_detection_node)
    ld.add_action(lane_keeping_assist_node)
    ld.add_action(rviz_node)
    ld.add_action(rqt_reconfigure_node)
    return ld

if __name__ == '__main__':
    generate_launch_description()