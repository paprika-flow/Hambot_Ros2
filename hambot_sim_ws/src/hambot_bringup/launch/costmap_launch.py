import os
import re
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


SDF_WORLD = 'campus_map2.sdf'  # active world file


def parse_waypoints(world_path):
    """
    Read all <frame> elements from SDF world file.
    Returns list of dicts: {'label': 'name', 'x': x, 'y': y, 'z': z, 'yaw': yaw}.
    Sorted by label.
    """
    if not os.path.exists(world_path):
        return []

    with open(world_path) as f:
        content = f.read()

    # Match: <frame name="ANYTHING">\n  <pose>X Y Z R P Y</pose>
    pattern = (
        r'<frame\s+name="([^"]+)"[^>]*>\s*'
        r'<pose>([\d.-]+)\s+([\d.-]+)\s+([\d.-]+)\s+[\d.-]+\s+[\d.-]+\s+([\d.-]+)</pose>'
    )
    matches = re.findall(pattern, content)

    points = []
    for label, x_str, y_str, z_str, yaw_str in matches:
        points.append({
            'label': label,
            'x': float(x_str),
            'y': float(y_str),
            'z': float(z_str),
            'yaw': float(yaw_str),
        })

    points.sort(key=lambda p: p['label'])
    return points


def spawn_robot_action(context):
    """
    Create spawn robot node.
    If route file provided, reads 'start' waypoint from YAML.
    Otherwise uses 'spawnpoint' argument.
    """
    pkg_bringup = get_package_share_directory('hambot_bringup')
    world_file = os.path.join(pkg_bringup, 'worlds', SDF_WORLD)
    points = parse_waypoints(world_file)
    num_points = len(points)

    # Determine start label: route file takes priority
    route_arg = LaunchConfiguration('route').perform(context).strip()
    label_arg = None

    if route_arg:
        try:
            with open(route_arg) as f:
                route_data = yaml.safe_load(f)
            label_arg = route_data.get('start', '').strip()
        except Exception:
            pass

    if not label_arg:
        label_arg = LaunchConfiguration('spawnpoint').perform(context).strip()

    if not label_arg:
        label_arg = points[0]['label'] if points else ''

    rotation_arg = LaunchConfiguration('rotation').perform(context).strip()

    # Find matching waypoint
    pose = next((p for p in points if p['label'] == label_arg), None)
    if pose is None and num_points > 0:
        pose = points[0]
    elif pose is None:
        return []

    x = str(pose['x'])
    y = str(pose['y'])
    z = str(pose['z'])
    yaw = str(pose['yaw'])
    if rotation_arg:
        yaw = rotation_arg

    return [
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'hambot',
                '-topic', 'robot_description',
                '-x', x,
                '-y', y,
                '-z', z,
                '-Y', yaw,
            ],
            output='screen',
        )
    ]


def generate_launch_description():
    pkg_description = get_package_share_directory('hambot_description')
    pkg_bringup = get_package_share_directory('hambot_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # --- Count available waypoints for help text ---
    world_file = os.path.join(pkg_bringup, 'worlds', SDF_WORLD)
    points = parse_waypoints(world_file)
    num_points = len(points)
    label_list = ', '.join(p['label'] for p in points[:6])
    if len(points) > 6:
        label_list += ', ...'
    first_label = points[0]['label'] if points else 'none'

    # --- Launch args ---
    route_arg = DeclareLaunchArgument(
        'route',
        default_value='',
        description=(
            f'Path to route YAML file. '
            f'If set, robot spawns at `start:` waypoint and follows waypoint list. '
            f'If empty, use spawnpoint arg instead.'
        )
    )
    spawnpoint_arg = DeclareLaunchArgument(
        'spawnpoint',
        default_value=first_label,
        description=(
            f'Waypoint label to spawn at (used only if route not provided). '
            f'Available: {label_list}. '
            f'Default: {first_label}.'
        )
    )
    rotation_arg = DeclareLaunchArgument(
        'rotation',
        default_value='',
        description='Optional: override yaw (radians). If empty, use waypoint default.'
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
            '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
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
            'route_file': LaunchConfiguration('route'),
            'odom_topic': '/odom',
        }]
    )

    return LaunchDescription([
        route_arg,
        spawnpoint_arg,
        rotation_arg,
        robot_state_publisher,
        gazebo_sim,
        spawn_robot,
        bridge,
        sidewalk_segmenter,
        costmap_navigator,
    ])
