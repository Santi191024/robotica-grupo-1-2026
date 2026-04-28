import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory('scara_robot_description')
    urdf_path  = os.path.join(pkg_path, 'urdf',   'SCARA_JH.urdf')
    world_path = os.path.join(pkg_path, 'worlds',  'scara_world.sdf')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # ── Variables de entorno ─────────────────────────────────────────
    set_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/jazzy/lib'
    )
    # Permite que Gazebo resuelva los meshes del paquete
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_path, '..', '..')  # share/
    )

    # ── Gazebo ──────────────────────────────────────────────────────
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    # ── Robot State Publisher ────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }],
        output='screen',
    )

    # ── Bridge ROS ↔ GZ ─────────────────────────────────────────────
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/Union_1_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/union_2_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/prismatic_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        output='screen',
    )

    # ── Spawn robot ── timer 10 s para dar tiempo a Gazebo ───────────
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name',  'scara_robot',
            '-x', '0', '-y', '0', '-z', '0.1',
            '-R', '0', '-P', '0', '-Y', '0',
        ],
        output='screen',   # muestra errores de spawn en terminal
    )

    return LaunchDescription([
        set_plugin_path,
        set_resource_path,
        gz_sim,
        TimerAction(period=2.0,  actions=[robot_state_publisher]),
        TimerAction(period=3.0,  actions=[gz_bridge]),
        TimerAction(period=10.0, actions=[spawn_robot]),   # ← 5→10 s
    ])