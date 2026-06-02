import os
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def parse_spawn_points(world_path):
    """
    Read <frame> elements from SDF world file.
    Extract name=start_position_N → x, y, z, yaw.
    Return list of dicts sorted by N.
    """
    if not os.path.exists(world_path):
        return []

    with open(world_path) as f:
        content = f.read()

    # Match: <frame name="start_position_(\d+)">\n  <pose>X Y Z R P Y</pose>
    pattern = (
        r'<frame\s+name="start_position_(\d+)"[^>]*>\s*'
        r'<pose>([\d.-]+)\s+([\d.-]+)\s+([\d.-]+)\s+[\d.-]+\s+[\d.-]+\s+([\d.-]+)</pose>'
    )
    matches = re.findall(pattern, content)

    points = []
    for num_str, x_str, y_str, z_str, yaw_str in matches:
        points.append({
            'id': int(num_str),
            'x': float(x_str),
            'y': float(y_str),
            'z': float(z_str),
            'yaw': float(yaw_str),
        })

    points.sort(key=lambda p: p['id'])
    return points


def spawn_robot_action(context):
    """Create spawn robot node using the selected start point parsed from SDF."""
    pkg_bringup = get_package_share_directory('hambot_bringup')
    world_file = os.path.join(pkg_bringup, 'worlds', 'campus_map.sdf')
    points = parse_spawn_points(world_file)
    num_points = len(points)

    point_str = LaunchConfiguration('start_point').perform(context)
    idx = int(point_str) if point_str.isdigit() else 1
    idx = max(1, min(idx, num_points))  # clamp to valid range

    pose = points[idx - 1]  # 1-indexed arg → 0-indexed list

    return [
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'hambot',
                '-topic', 'robot_description',
                '-x', str(pose['x']),
                '-y', str(pose['y']),
                '-z', str(pose['z']),
                '-Y', str(pose['yaw']),
            ],
            output='screen',
        )
    ]


def generate_launch_description():
    pkg_description = get_package_share_directory('hambot_description')
    pkg_bringup = get_package_share_directory('hambot_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # --- Count available spawn points for help text ---
    world_file = os.path.join(pkg_bringup, 'worlds', 'campus_map.sdf')
    num_points = len(parse_spawn_points(world_file))

    # --- Launch args ---
    start_point_arg = DeclareLaunchArgument(
        'start_point',
        default_value='1',
        description=(
            f'Spawn point index (1-{num_points}). '
            f'Parsed automatically from <frame name="start_position_N"> '
            f'in campus_map.sdf. Default=1.'
        )
    )

    # --- Robot model ---
    xacro_file = os.path.join(pkg_description, 'urdf', 'hambot.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': True
        }]
    )

    # --- World ---
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # --- Spawn robot (reads start_point arg at runtime via OpaqueFunction) ---
    spawn_robot = OpaqueFunction(function=spawn_robot_action)

    # --- Bridge ---
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
            # ROS 2 → Gazebo (debug display)
            '/camera/sidewalk_mask@sensor_msgs/msg/Image]ignition.msgs.Image',
            '/costmap/debug_image@sensor_msgs/msg/Image]ignition.msgs.Image',
            '/costmap/overlay_image@sensor_msgs/msg/Image]ignition.msgs.Image',
        ],
        output='screen',
    )

    # --- Perception: sidewalk segmenter ---
    sidewalk_segmenter = Node(
        package='hambot_bringup',
        executable='sidewalk_segmenter.py',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # --- Navigation: costmap planner ---
    costmap_navigator = Node(
        package='hambot_bringup',
        executable='costmap_navigator.py',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'segmentation_topic': '/camera/sidewalk_mask',
            'map_forward': 3.0,
            'map_backward': 0.5,
            'map_lateral': 1.5,
            'cell_size': 0.05,
            'cam_height': 0.341,
            'cam_forward': -0.079,
            'cam_hfov': 1.274,
            'cam_width': 640,
            'cam_height_px': 480,
            'linear_speed': 0.25,
            'kp_angular': 0.8,
            'lidar_topic': '/scan',
            'robot_radius': 0.30,
            'obstacle_inflation': 0.15,
        }]
    )

    return LaunchDescription([
        start_point_arg,
        robot_state_publisher,
        gazebo_sim,
        spawn_robot,
        bridge,
        sidewalk_segmenter,
        costmap_navigator,
    ])
