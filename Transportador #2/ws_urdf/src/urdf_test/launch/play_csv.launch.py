import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Obtener la ruta de tu paquete urdf_test
    pkg_share = get_package_share_directory('urdf_test')

    # 1. Incluimos tu launch original de Gazebo
    # Esto abrirá Gazebo, cargará el robot y activará el 'arm_controller'
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
        )
    )

    # 2. Nodo que ejecuta tu script de Python (el "reproductor" del CSV)
    # Lo envolvemos en un TimerAction para darle tiempo a Gazebo de arrancar
    # He puesto 12 segundos para estar seguros de que los controladores ya estén activos
    csv_player_node = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='urdf_test',
                executable='csv_trajectory_player',
                name='csv_trajectory_player',
                output='screen',
                emulate_tty=True
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        csv_player_node
    ])
