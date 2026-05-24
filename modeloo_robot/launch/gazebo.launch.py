import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_name = 'modeloo_robot'
    pkg_path = get_package_share_directory(package_name)

    urdf_path = os.path.join(pkg_path, 'urdf', 'V2.SLDASM.urdf')

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen',
        additional_env={
            'GZ_SIM_SYSTEM_PLUGIN_PATH': '/opt/ros/jazzy/lib:/home/gabriel/gz_ws/install/gz_ros2_control/lib'
        }
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time
        }]
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'scara_robot',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                output="screen",
            )
        ]
    )

    arm_controller = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_controller"],
                output="screen",
            )
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher,
        gazebo,
        spawn_entity,
        joint_state_broadcaster,
        arm_controller,
    ])