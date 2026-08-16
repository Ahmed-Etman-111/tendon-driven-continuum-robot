import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'tendon_continuum'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. Process the Xacro file into URDF
    xacro_file = os.path.join(pkg_share, 'urdf', 'continuum.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # 2. Robot State Publisher Node (broadcasts the URDF to the system)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_urdf}]
    )

    # 3. Start Gazebo Harmonic (Empty World)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # 4. Spawn the robot into Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'tendon_robot',
            '-string', robot_urdf,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01' # Spawns slightly above ground
        ],
        output='screen'
    )

# 5. Start your Python Rod Visualizer Node
    rod_visualizer_node = Node(
        package='tendon_continuum',
        executable='rod_visualizer.py',
        output='screen'
    )

    # 6. Start RViz 2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

# 7. Joint State Publisher GUI (To manually test the bending)
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )    

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        rod_visualizer_node, # Added
        rviz_node,            # Added
        jsp_gui_node          # Added
    ])