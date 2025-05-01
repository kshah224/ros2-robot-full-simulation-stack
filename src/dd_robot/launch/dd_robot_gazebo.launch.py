import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import xacro
from launch.actions import TimerAction

def generate_launch_description():

    pkg_name = 'dd_robot'
    xacro_path = os.path.join(get_package_share_directory(pkg_name), 'urdf/dd_robot.xacro')
    world_path = os.path.join(get_package_share_directory(pkg_name), 'worlds/empty_world.world')

    # Process xacro file
    robot_description = xacro.process_file(xacro_path).toxml()

    # Start Ignition Gazebo with the world
    ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_path, '--render-engine', 'ogre2', '--verbose', '-r'],  # -r to auto-start simulation
        output='screen',
        additional_env={
            'LIBGL_ALWAYS_SOFTWARE': '1'
        }
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

    bridges = [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='laser_bridge',
            arguments=['/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_bridge',
            arguments=['/camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image'],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='velocity_bridge',
            arguments=[
                '/model/ddrobot/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='joint_state_bridge',
            arguments=[
                '/world/car_world/model/ddrobot/joint_state@sensor_msgs/msg/JointState@ignition.msgs.Model',
                '--ros-args', '--remap', '/world/car_world/model/ddrobot/joint_state:=/joint_states'
            ],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='imu_bridge',
            arguments=['/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU'],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/ddrobot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                '--ros-args', '-r', '/model/ddrobot/odometry:=/odom'
            ],
            output='screen'
        )
    ]

    tf_nodes = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'ddrobot/odom'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'ddrobot/base_link/lidar'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'ddrobot/base_link/camera'],
            output='screen'
        )
    ]
    
    # Spawn robot into Ignition using the /create service
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'ddrobot', '-topic', 'robot_description',
                   '-x', '0', '-y', '0', '-z', '0.2'],
        output='screen'
    )

    return LaunchDescription([
        ign_gazebo,
        robot_state_publisher,
        spawn_robot,
        *bridges,
        *tf_nodes
    ])
