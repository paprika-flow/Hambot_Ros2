import os
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
import xacro

def parse_spawn_points(world_path):
    if not os.path.exists(world_path):
        return []

    with open(world_path) as f:
        content = f.read()

    pattern = (
        r'<frame\s+name="[a-zA-Z_]+_(\d+)"[^>]*>\s*'
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
    pkg_bringup = get_package_share_directory('hambot_bringup')
    world_name = LaunchConfiguration('world_name').perform(context)
    world_file = os.path.join(pkg_bringup, 'worlds', f"{world_name}.sdf")
    
    points = parse_spawn_points(world_file)
    num_points = len(points)

    point_str = LaunchConfiguration('start_point').perform(context)
    idx = int(point_str) if point_str.isdigit() else 1
    
    if num_points > 0:
        idx = max(1, min(idx, num_points))  
        pose = points[idx - 1]  
    else:
        pose = {'x': 0.0, 'y': 0.0, 'z': 0.1, 'yaw': 0.0}

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
    
    # 1. World Config Launch Arguments
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='campus_sidewalk',
        description='Name of the world file to load (without .sdf extension)'
    )

    start_point_arg = DeclareLaunchArgument(
        'start_point',
        default_value='1',
        description='Spawn point index. Parsed automatically from the SDF file. Default=1.'
    )

    # Topological Route Parameters
    start_node_arg = DeclareLaunchArgument(
        'start_node',
        default_value='0',
        description='Starting topological node ID for route planning.'
    )

    end_node_arg = DeclareLaunchArgument(
        'end_node',
        default_value='19',
        description='Target topological destination node ID.'
    )

    # Default model path
    default_model_path = os.path.join(
        get_package_share_directory('hambot_bringup'),
        'models',
        'best_voronoi_model.pkl'
    )
    
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=default_model_path,
        description='Absolute path to the trained scikit-learn pipeline (.pkl)'
    )

    collect_data_arg = DeclareLaunchArgument(
        'collect_data',
        default_value='false',
        choices=['true', 'false'],
        description='Set to true to launch the dataset collector node'
    )

    dataset_mode_arg = DeclareLaunchArgument(
        'dataset_mode',
        default_value='straight',
        choices=['straight', 'split'],
        description='Target directory mapping: "straight" or "split"'
    )

    save_directory_expr = PythonExpression([
        "'processed_rosmaster_photos_splits/processed_rosmaster_photos_splits/mask' if '",
        LaunchConfiguration('dataset_mode'),
        "' == 'split' else 'processed_rosmaster_photos/mask'"
    ])

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

    gz_args_expr = PythonExpression([
        "'-r ' + '", pkg_bringup, "/worlds/' + '", LaunchConfiguration('world_name'), "' + '.sdf'"
    ])

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args_expr}.items()
    )

    spawn_robot = OpaqueFunction(function=spawn_robot_action)

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
            '/camera/sidewalk_mask@sensor_msgs/msg/Image]ignition.msgs.Image',
            '/voronoi/debug_image@sensor_msgs/msg/Image]ignition.msgs.Image',
        ],
        output='screen'
    )

    sidewalk_segmenter = Node(
        package='hambot_bringup',
        executable='sidewalk_segmenter.py',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    voronoi_path_planner = Node(
        package='hambot_bringup',
        executable='voronoi_path_planner.py',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/camera/sidewalk_mask',
            'target_gray': 255,  
            'resize_width': 960,
            'resize_height': 720,
            'model_path': LaunchConfiguration('model_path')
        }]
    )

    voronoi_navigator = Node(
        package='hambot_bringup',
        executable='voronoi_navigator.py',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'kp_area': 0.25,
            'kd_area': 0.05,
            'kp_pos': 0.2,
            'kd_pos': 0.05,
            'kp_side': 0.25,
            'kd_side': 0.025,
            'target_linear_speed': 0.16,
            'max_angular_speed': 0.15,
            'obstacle_threshold': 0.45,  
            'forward_fov_deg': 40.0,
            'split_threshold': 0.6,
            'min_split_duration': 7.0,
            'way_area_threshold': 6000.0,
            'split_direction': 'straight' # overridden dynamically by topic
        }]
    )

    # =====================================================================
    # GLOBAL PLANNER INTEGRATION
    # =====================================================================
    global_planner = Node(
        package='hambot_bringup',
        executable='global_planner.py',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'world_name': LaunchConfiguration('world_name'),
            'start_node': LaunchConfiguration('start_node'),
            'end_node': LaunchConfiguration('end_node')
        }]
    )

    dataset_collector = Node(
        package='hambot_bringup',
        executable='dataset_collector.py',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'save_directory': save_directory_expr,
            'save_interval': 1.0,           
            'target_gray_value': 15         
        }],
        condition=IfCondition(LaunchConfiguration('collect_data'))
    )

    return LaunchDescription([
        world_name_arg,
        start_point_arg,
        start_node_arg,
        end_node_arg,
        model_path_arg,
        collect_data_arg,
        dataset_mode_arg,
        robot_state_publisher,
        gazebo_sim,
        spawn_robot,
        bridge,
        sidewalk_segmenter,
        voronoi_path_planner,
        voronoi_navigator,
        global_planner,
        dataset_collector
    ])