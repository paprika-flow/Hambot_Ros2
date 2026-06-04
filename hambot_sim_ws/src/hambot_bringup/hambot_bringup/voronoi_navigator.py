#!/usr/bin/env python3
import math
import time
import threading
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, PoseArray
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


# =====================================================================
# BLENDED PID CONTROL ENGINE WITH TRANSITION-SAFE DERIVATIVES
# =====================================================================
class VoronoiNavigationEngine:
    def __init__(
        self, 
        kp_area: float = 0.025, 
        kd_area: float = 0.0125, 
        kp_pos: float = 0.0,
        kd_pos: float = 0.0,
        kp_side: float = 0.025,
        kd_side: float = 0.01,             
        target_linear_speed: float = 0.16, 
        max_angular_speed: float = 0.1,
        smoothing_factor: float = 0.07,   
        area_normalization_scale: float = 50000.0,
        image_width: float = 960.0,
        area_deadband: float = 0.015,
        pos_deadband: float = 0.03,
        side_deadband: float = 0.03,
        min_side_distance: float = 0.0,
        deactivate_area_threshold: float = 700.0  # Threshold to deactivate the area PD loop
    ):
        self.kp_area = kp_area
        self.kd_area = kd_area
        self.kp_pos = kp_pos
        self.kd_pos = kd_pos
        self.kp_side = kp_side
        self.kd_side = kd_side
        self.target_linear_speed = target_linear_speed
        self.max_angular_speed = max_angular_speed
        self.smoothing_factor = smoothing_factor
        self.area_normalization_scale = area_normalization_scale
        self.image_width = image_width
        self.min_side_distance = min_side_distance
        self.deactivate_area_threshold = deactivate_area_threshold
        
        # Deadband Threshold Filters
        self.area_deadband = area_deadband
        self.pos_deadband = pos_deadband
        self.side_deadband = side_deadband
        
        # State tracking for multi-derivative logic
        self.prev_error_area = 0.0
        self.prev_error_pos = 0.0
        self.prev_error_side = 0.0
        
        # Tracks coordinate validity in previous frame to shield against derivative spikes
        self.prev_pos_valid = False
        self.prev_side_valid = False
        
        self.prev_time = None
        self.smoothed_angular_vel = 0.0

    def compute_controls(
        self, 
        area_left: float, 
        area_right: float, 
        path_x: float, 
        side_mid_x: float,
        side_dist: float,
        scan_min_dist: float, 
        obstacle_threshold: float
    ) -> tuple:
        """
        Computes velocities utilizing independent proportional and derivative control loops
        for area imbalance, path center offsets, and entry mouth alignment.
        """
        # 1. Emergency Obstacle Halt
        # if scan_min_dist < obstacle_threshold:
        #     self.smoothed_angular_vel = 0.0
        #     return 0.0, 0.0

        current_time = time.time()
        
        if self.prev_time is None:
            dt = 0.033
        else:
            dt = current_time - self.prev_time
            if dt <= 0:
                dt = 0.001

        # --- CONDITIONAL AREA DEACTIVATION ---
        # If the sidewalk entry width falls below 700, deactivate the side vector area loop.
        # This keeps the side vector midpoint tracking active, while discarding area imbalance noise.
        if side_dist is not None and side_dist < self.deactivate_area_threshold:
            area_left = 0.0
            area_right = 0.0

        # 2. Area Proportional Term with Deadband
        raw_diff = area_left - area_right
        error_area = raw_diff / self.area_normalization_scale
        if abs(error_area) < self.area_deadband:
            error_area = 0.0
        p_term_area = self.kp_area * error_area
        
        # 3. Path Position Proportional Term with Deadband
        error_pos = 0.0
        pos_valid = (path_x is not None)
        if pos_valid:
            center_x = self.image_width / 2.0
            error_pos = (center_x - path_x) / center_x
            if abs(error_pos) < self.pos_deadband:
                error_pos = 0.0
        p_term_pos = self.kp_pos * error_pos
        
        # 4. Side Vector Distance Proportional Term with Deadband & Minimum Spacing Discard
        error_side = 0.0
        # If spacing is below min_side_distance (480.0), completely stop considering side vector positioning
        side_valid = (side_mid_x is not None and side_dist is not None and side_dist == 0.0 and side_dist >= self.min_side_distance)
        if side_valid:
            center_x = self.image_width / 2.0
            # Scale-invariant offset based on actual width between points:
            error_side = (2.0 * (center_x - side_mid_x)) / side_dist
            if abs(error_side) < self.side_deadband:
                error_side = 0.0
        p_term_side = self.kp_side * error_side
        
        # Combined Proportional Command
        p_term = p_term_area + p_term_pos + p_term_side
        
        # 5. Independent Derivative Calculations (Transition-Safe)
        d_term_area = 0.0
        if dt > 0:
            raw_derivative_area = (error_area - self.prev_error_area) / dt
            d_term_area = self.kd_area * raw_derivative_area

        d_term_pos = 0.0
        if dt > 0 and pos_valid and self.prev_pos_valid:
            raw_derivative_pos = (error_pos - self.prev_error_pos) / dt
            d_term_pos = self.kd_pos * raw_derivative_pos

        d_term_side = 0.0
        if dt > 0 and side_valid and self.prev_side_valid:
            raw_derivative_side = (error_side - self.prev_error_side) / dt
            d_term_side = self.kd_side * raw_derivative_side

        # Combined Derivative Command
        d_term = d_term_area + d_term_pos + d_term_side
        
        # Combined raw angular command
        raw_angular_vel = p_term + d_term
        
        # Clamp raw steering command
        raw_angular_vel = max(-self.max_angular_speed, min(self.max_angular_speed, raw_angular_vel))
        
        # 6. Temporal Low-Pass Filtering (EMA)
        self.smoothed_angular_vel = (self.smoothing_factor * raw_angular_vel) + \
                                    ((1.0 - self.smoothing_factor) * self.smoothed_angular_vel)
                                    
        # State updates for tracking next frames
        self.prev_error_area = error_area
        self.prev_error_pos = error_pos
        self.prev_error_side = error_side
        self.prev_pos_valid = pos_valid
        self.prev_side_valid = side_valid
        self.prev_time = current_time
        
        # Adaptive Linear Velocity
        norm_correction = abs(raw_angular_vel) / self.max_angular_speed
        speed_multiplier = max(0.3, 1.0 - norm_correction * 0.7)
        linear_vel = self.target_linear_speed * speed_multiplier
        
        return linear_vel, self.smoothed_angular_vel


# =====================================================================
# ROS 2 WRAPPER NODE
# =====================================================================
class VoronoiNavigator(Node):
    def __init__(self):
        super().__init__('voronoi_navigator')
        
        # Declare configurable parameters
        self.declare_parameter('kp_area', 0.5)
        self.declare_parameter('kd_area', 0.125)
        self.declare_parameter('kp_pos', 0.2)
        self.declare_parameter('kd_pos', 0.05)
        self.declare_parameter('kp_side', 0.5)
        self.declare_parameter('kd_side', 0.1)
        self.declare_parameter('target_linear_speed', 0.16)
        self.declare_parameter('max_angular_speed', 0.2)
        self.declare_parameter('smoothing_factor', 0.15)
        self.declare_parameter('obstacle_threshold', 0.45)
        self.declare_parameter('forward_fov_deg', 40.0)
        self.declare_parameter('invert_area_sign', False)  
        self.declare_parameter('area_normalization_scale', 10000.0)
        self.declare_parameter('image_width', 960.0)
        self.declare_parameter('use_path_position', True)
        self.declare_parameter('use_side_position', True)
        self.declare_parameter('min_side_distance', 0.0)           # Threshold for vector convergence
        self.declare_parameter('deactivate_area_threshold', 750.0)   # Threshold to disable the visual area PD
        
        # Deadbands (Noise Filters)
        self.declare_parameter('area_deadband', 0.015)
        self.declare_parameter('pos_deadband', 0.03)
        self.declare_parameter('side_deadband', 0.03)

        # Instantiate control engine
        self.engine = VoronoiNavigationEngine(
            kp_area=self.get_parameter('kp_area').value,
            kd_area=self.get_parameter('kd_area').value,
            kp_pos=self.get_parameter('kp_pos').value,
            kd_pos=self.get_parameter('kd_pos').value,
            kp_side=self.get_parameter('kp_side').value,
            kd_side=self.get_parameter('kd_side').value,
            target_linear_speed=self.get_parameter('target_linear_speed').value,
            max_angular_speed=self.get_parameter('max_angular_speed').value,
            smoothing_factor=self.get_parameter('smoothing_factor').value,
            area_normalization_scale=self.get_parameter('area_normalization_scale').value,
            image_width=self.get_parameter('image_width').value,
            area_deadband=self.get_parameter('area_deadband').value,
            pos_deadband=self.get_parameter('pos_deadband').value,
            side_deadband=self.get_parameter('side_deadband').value,
            min_side_distance=self.get_parameter('min_side_distance').value,
            deactivate_area_threshold=self.get_parameter('deactivate_area_threshold').value
        )
        
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.forward_fov_rad = math.radians(self.get_parameter('forward_fov_deg').value)
        self.invert_area_sign = self.get_parameter('invert_area_sign').value
        self.use_path_position = self.get_parameter('use_path_position').value
        self.use_side_position = self.get_parameter('use_side_position').value
        
        self.lock = threading.Lock()
        self.latest_scan_min = float('inf')
        
        # Placeholders for tracking inputs
        self.latest_area_left = 0.0
        self.latest_area_right = 0.0
        self.latest_path_x = None
        self.latest_side_mid_x = None
        self.latest_side_dist = None
        
        callback_group = ReentrantCallbackGroup()
        
        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.latest_only_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.area_sub = self.create_subscription(
            Float32,
            '/voronoi/area_difference',
            self.area_callback,
            self.latest_only_qos,
            callback_group=callback_group
        )
        
        self.area_left_sub = self.create_subscription(
            Float32,
            '/voronoi/area_left',
            self.area_left_callback,
            self.latest_only_qos,
            callback_group=callback_group
        )
        
        self.area_right_sub = self.create_subscription(
            Float32,
            '/voronoi/area_right',
            self.area_right_callback,
            self.latest_only_qos,
            callback_group=callback_group
        )

        self.path_sub = self.create_subscription(
            PoseArray,
            '/voronoi/best_path',
            self.best_path_callback,
            self.latest_only_qos,
            callback_group=callback_group
        )

        self.side_mid_sub = self.create_subscription(
            Float32,
            '/voronoi/side_vector_mid_x',
            self.side_mid_callback,
            self.latest_only_qos,
            callback_group=callback_group
        )

        self.side_dist_sub = self.create_subscription(
            Float32,
            '/voronoi/side_vector_distance',
            self.side_dist_callback,
            self.latest_only_qos,
            callback_group=callback_group
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            self.latest_only_qos,
            callback_group=callback_group
        )
        
        self.get_logger().info(
            f"Voronoi Blended PID Navigator initialized (Scale-Invariant Side Offset active).\n"
            f"Gains: area_PD={self.engine.kp_area}/{self.engine.kd_area}, "
            f"pos_PD={self.engine.kp_pos}/{self.engine.kd_pos}, "
            f"side_PD={self.engine.kp_side}/{self.engine.kd_side}\n"
            f"Deadbands: area={self.engine.area_deadband}, path_pos={self.engine.pos_deadband}, side_pos={self.engine.side_deadband}\n"
            f"Thresholds: deact_area={self.engine.deactivate_area_threshold}, min_spacing={self.engine.min_side_distance}"
        )

    def scan_callback(self, msg: LaserScan):
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        half_fov = self.forward_fov_rad / 2.0

        valid_ranges = []
        for i, dist in enumerate(msg.ranges):
            angle = angle_min + (i * angle_inc)
            angle = math.atan2(math.sin(angle), math.cos(angle))

            if -half_fov <= angle <= half_fov:
                if msg.range_min <= dist <= msg.range_max:
                    valid_ranges.append(dist)

        with self.lock:
            self.latest_scan_min = min(valid_ranges) if valid_ranges else float('inf')

    def area_left_callback(self, msg: Float32):
        with self.lock:
            self.latest_area_left = msg.data

    def area_right_callback(self, msg: Float32):
        with self.lock:
            self.latest_area_right = msg.data

    def best_path_callback(self, msg: PoseArray):
        with self.lock:
            if len(msg.poses) >= 2:
                p0_x = msg.poses[0].position.x
                p1_x = msg.poses[1].position.x
                self.latest_path_x = (p0_x + p1_x) / 2.0
            else:
                self.latest_path_x = None

    def side_mid_callback(self, msg: Float32):
        with self.lock:
            if msg.data >= 0.0:
                self.latest_side_mid_x = msg.data
            else:
                self.latest_side_mid_x = None

    def side_dist_callback(self, msg: Float32):
        with self.lock:
            if msg.data >= 0.0:
                self.latest_side_dist = msg.data
            else:
                self.latest_side_dist = None

    def area_callback(self, msg: Float32):
        with self.lock:
            scan_dist = self.latest_scan_min
            area_left = self.latest_area_left
            area_right = self.latest_area_right
            path_x = self.latest_path_x if self.use_path_position else None
            side_mid_x = self.latest_side_mid_x if self.use_side_position else None
            side_dist = self.latest_side_dist if self.use_side_position else None
            
        if self.invert_area_sign:
            area_left, area_right = area_right, area_left
            center_x = self.engine.image_width / 2.0
            if path_x is not None:
                path_x = 2.0 * center_x - path_x
            if side_mid_x is not None:
                side_mid_x = 2.0 * center_x - side_mid_x
            
        linear_vel, angular_vel = self.engine.compute_controls(
            area_left, area_right, path_x, side_mid_x, side_dist, scan_dist, self.obstacle_threshold
        )
        
        raw_diff = area_left - area_right
        path_str = f"PathX: {path_x:5.1f}" if path_x is not None else "PathX: None"
        side_str = f"SideX: {side_mid_x:5.1f}" if side_mid_x is not None else "SideX: None"
        dist_str = f"SideDist: {side_dist:5.1f}" if side_dist is not None else "SideDist: None"
        
        # Determine tracking statuses for diagnostic terminal logs
        area_active = "ACTIVE" if (side_dist is None or side_dist >= self.engine.deactivate_area_threshold) else "INACTIVE"
        side_active = "ACTIVE" if (side_mid_x is not None and side_dist is not None and side_dist >= self.engine.min_side_distance) else "INACTIVE"
        
        self.get_logger().info(
            f"Areas -> L: {area_left:6.1f} | R: {area_right:6.1f} (Area: {area_active}) | "
            f"RawDiff: {raw_diff:6.1f} | {path_str} | {side_str} ({dist_str}, Side: {side_active}) | "
            f"Cmd Angular: {angular_vel:6.3f} rad/s",
            throttle_duration_sec=0.2
        )
        
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_pub.publish(cmd)

    def publish_stop_cmd(self):
        stop_msg = Twist()
        self.cmd_pub.publish(stop_msg)


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = VoronoiNavigator()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop_cmd()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()