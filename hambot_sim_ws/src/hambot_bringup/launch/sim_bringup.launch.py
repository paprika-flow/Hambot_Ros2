import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_description = get_package_share_directory('hambot_description')
    pkg_bringup = get_package_share_directory('hambot_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Process Xacro
    xacro_file = os.path.join(pkg_description, 'urdf', 'hambot.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': True
        }]
    )

    # Path to our newly created custom SDF world file
    world_file = os.path.join(pkg_bringup, 'worlds', 'campus_sidewalk.sdf')

    # Include Gazebo Launch (Forces server-only headless mode to save CPU, launching our custom world)
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        # Loads our custom sidewalk world
        launch_arguments={'gz_args': f'-r {world_file}'}.items() 
    )

    # Spawn Robot Node (Spawns robot at the back of the 5-meter sidewalk straightaway)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'hambot',
            '-topic', 'robot_description',
            '-x', '-4.5', '-y', '0.0', '-z', '0.1' # Spawn coordinates
        ],
        output='screen'
    )

    # ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/segmentation/labels_map@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/segmentation/colored_map@sensor_msgs/msg/Image[ignition.msgs.Image',
        ],
        output='screen'
    )

    # 7. Custom Sidewalk Segmenter Node (Processes raw VNC images into a binary mask)
    sidewalk_segmenter = Node(
        package='hambot_bringup',
        executable='sidewalk_segmenter.py', # Matches the filename we installed in CMake
        output='screen',
        parameters=[{'use_sim_time': True}] # Crucial: ensures OpenCV/image timestamps match simulation clock
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo_sim,
        spawn_robot,
        bridge,
        sidewalk_segmenter
    ])