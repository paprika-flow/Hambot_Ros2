import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. Paths to packages
    pkg_description = get_package_share_directory('hambot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # 2. Process Xacro file to raw XML
    xacro_file = os.path.join(pkg_description, 'urdf', 'hambot.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # 3. Robot State Publisher Node (publishes /tf tree)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': True
        }]
    )

    # 4. Include Gazebo Launch (Starts Gazebo with an empty world)
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r -s empty.sdf'}.items() # -r starts simulation immediately, -s starts server mode
    )

    # 5. Spawn Robot Node (injects robot from /robot_description topic to Gazebo)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'hambot',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.5'
        ],
        output='screen'
    )

    # 6. ROS-Gazebo Bridge (Bridges topics between ROS 2 and Gazebo)
    # '@' translates direction, '[' denotes topic type matching
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo_sim,
        spawn_robot,
        bridge
    ])