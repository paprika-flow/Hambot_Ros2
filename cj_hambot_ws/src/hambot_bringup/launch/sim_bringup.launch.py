import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_description = get_package_share_directory("hambot_description")
    pkg_bringup = get_package_share_directory("hambot_bringup")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # Process Xacro
    xacro_file = os.path.join(pkg_description, "urdf", "hambot.urdf.xacro")
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_raw, "use_sim_time": True}],
    )

    # Path to newly created custom SDF world file
    world_file = os.path.join(pkg_bringup, "worlds", "campus_sidewalk.sdf")

    # Include Gazebo Launch
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {world_file}"}.items(),
    )

    # Spawn Robot Node
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "hambot",
            "-topic",
            "robot_description",
            "-x",
            "-4.5",
            "-y",
            "0.0",
            "-z",
            "0.1",
        ],
        output="screen",
    )

    # ROS-Gazebo Bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            "/segmentation/labels_map@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/segmentation/colored_map@sensor_msgs/msg/Image[ignition.msgs.Image",
            # --- BRIDGES TO VIEW OUTPUTS BACK IN GAZEBO ---
            # Bridge the Sidewalk Mask from ROS 2 -> Gazebo (if not already present)
            "/camera/sidewalk_mask@sensor_msgs/msg/Image]ignition.msgs.Image",
            # Bridge the Voronoi Debug Image from ROS 2 -> Gazebo
            "/voronoi/debug_image@sensor_msgs/msg/Image]ignition.msgs.Image",
        ],
        output="screen",
    )

    # Custom Sidewalk Segmenter Node
    sidewalk_segmenter = Node(
        package="hambot_local_nav",
        executable="sidewalk_segmenter",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Voronoi Path Planner Node (Processes segmenter masks into skeleton lanes)
    voronoi_path_planner = Node(
        package="hambot_local_nav",
        executable="voronoi_path_planner",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "input_topic": "/camera/sidewalk_mask",
                "target_gray": 255,  # Matches the binary output scale of the segmenter node
                "resize_width": 960,
                "resize_height": 720,
            }
        ],
    )

    # Local Navigator Node (Centering Control)
    centroid_navigator = Node(
        package="hambot_local_nav",
        executable="centroid_navigator",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            gazebo_sim,
            spawn_robot,
            bridge,
            sidewalk_segmenter,
            voronoi_path_planner,
            centroid_navigator,
        ]
    )
