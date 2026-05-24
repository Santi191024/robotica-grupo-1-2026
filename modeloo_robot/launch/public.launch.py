
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    #NODOS PREDEFINIDOS
        # Nodo de robot_state_publisher
 
    # Launch joint state broadcaster
    start_joint_state_broadcaster_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen')
    start_arm_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller'],
        output='screen')

    # Add delay to joint state broadcaster (if necessary)
    delayed_start = TimerAction(
        period=1.0,
        actions=[start_joint_state_broadcaster_cmd]
    )

    load_joint_state_broadcaster_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_joint_state_broadcaster_cmd,
            on_exit=[start_arm_controller_cmd]))

    

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the actions to the launch description in sequence
    #ld.add_action(robot_state_publisher_node)
    #ld.add_action(control_node)
    ld.add_action(delayed_start)
    ld.add_action(load_joint_state_broadcaster_cmd)
    

    return ld