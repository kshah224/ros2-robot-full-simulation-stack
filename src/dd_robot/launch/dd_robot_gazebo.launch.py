import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    pkg_name = 'dd_robot'
    xacro_path = os.path.join(get_package_share_directory(pkg_name), 'urdf/dd_robot2.urdf')
    world_path = os.path.join(get_package_share_directory(pkg_name), 'worlds/empty_world.world')

    # Process xacro file
    robot_description = xacro.process_file(xacro_path).toxml()

    # Start Ignition Gazebo with the world
    ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_path, '--render-engine', 'ogre2', '--verbose', '-r'],  # -r to auto-start simulation
        output='screen'
    )

    # Publish robot_description on topic
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
        output='screen',
        additional_env={
            '__NV_PRIME_RENDER_OFFLOAD': '1'
        }
    )

    # Spawn robot into Ignition using the /create service
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'ddrobot', '-topic', 'robot_description',
                   '-x', '0', '-y', '0', '-z', '0.1'],
        output='screen'
    )

    return LaunchDescription([
        ign_gazebo,
        robot_state_publisher,
        spawn_robot
    ])
