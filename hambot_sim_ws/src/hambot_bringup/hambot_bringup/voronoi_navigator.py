#!/usr/bin/env python3
import math
import time
import threading
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


# =====================================================================
# OPTIMIZED AREA-BASED CONTROL ENGINE (Zero ROS Dependency)
# =====================================================================
class VoronoiNavigationEngine:
    def __init__(
        self, 
        kp_area: float = 1.0, 
        kd_area: float = 0.5, 
        target_linear_speed: float = 0.16, 
        max_angular_speed: float = 0.5,
        smoothing_factor: float = 0.10,  # Alpha for EMA filter (lower = smoother)
        area_normalization_scale: float = 10000.0  # Constant to scale raw diffs smoothly
    ):
        self.kp_area = kp_area
        self.kd_area = kd_area
        self.target_linear_speed = target_linear_speed
        self.max_angular_speed = max_angular_speed
        self.smoothing_factor = smoothing_factor
        self.area_normalization_scale = area_normalization_scale
        
        # State tracking for derivative and smoothing
        self.prev_error = 0.0
        self.prev_time = None
        self.smoothed_angular_vel = 0.0

    def compute_controls(self, area_left: float, area_right: float, scan_min_dist: float, obstacle_threshold: float) -> tuple:
        """
        Computes smooth linear and angular velocities using a PD loop on the scaled raw area difference.
        """
        # 1. Emergency Obstacle Halt
        if scan_min_dist < obstacle_threshold:
            self.smoothed_angular_vel = 0.0
            return 0.0, 0.0

        current_time = time.time()
        
        # Calculate time step
        if self.prev_time is None:
            dt = 0.033  # Nominal dt assuming ~30 Hz updates
        else:
            dt = current_time - self.prev_time
            if dt <= 0:
                dt = 0.001

        # 2. Compute Scaled Raw Area Difference
        raw_diff = area_left - area_right
        
        # Normalize by a constant scale factor to keep errors smooth and linear
        error = raw_diff / self.area_normalization_scale
        
        # Proportional term (drives the robot to center)
        p_term = self.kp_area * error
        
        # Derivative term (dampens oscillation by predicting error change rate)
        d_term = 0.0
        if dt > 0:
            raw_derivative = (error - self.prev_error) / dt
            d_term = self.kd_area * raw_derivative
            
        raw_angular_vel = p_term + d_term
        
        # Clamp raw steering command
        raw_angular_vel = max(-self.max_angular_speed, min(self.max_angular_speed, raw_angular_vel))
        
        # 3. Temporal Low-Pass Filtering (Exponential Moving Average)
        self.smoothed_angular_vel = (self.smoothing_factor * raw_angular_vel) + \
                                    ((1.0 - self.smoothing_factor) * self.smoothed_angular_vel)
                                    
        # State updates
        self.prev_error = error
        self.prev_time = current_time
        
        # 4. Adaptive Linear Velocity (slow down slightly during large corrections)
        speed_multiplier = max(0.3, 1.0 - abs(error) * 1.5)
        linear_vel = self.target_linear_speed * speed_multiplier
        
        return linear_vel, self.smoothed_angular_vel


# =====================================================================
# ROS 2 WRAPPER NODE
# =====================================================================
class VoronoiNavigator(Node):
    def __init__(self):
        super().__init__('voronoi_navigator')
        
        # Declare configurable parameters
        self.declare_parameter('kp_area', 1.2)
        self.declare_parameter('kd_area', 0.3)
        self.declare_parameter('target_linear_speed', 0.16)
        self.declare_parameter('max_angular_speed', 0.6)
        self.declare_parameter('smoothing_factor', 0.15)
        self.declare_parameter('obstacle_threshold', 0.45)
        self.declare_parameter('forward_fov_deg', 40.0)
        self.declare_parameter('invert_area_sign', False)  
        self.declare_parameter('area_normalization_scale', 10000.0) # Scale parameter

        # Instantiate control engine
        self.engine = VoronoiNavigationEngine(
            kp_area=self.get_parameter('kp_area').value,
            kd_area=self.get_parameter('kd_area').value,
            target_linear_speed=self.get_parameter('target_linear_speed').value,
            max_angular_speed=self.get_parameter('max_angular_speed').value,
            smoothing_factor=self.get_parameter('smoothing_factor').value,
            area_normalization_scale=self.get_parameter('area_normalization_scale').value
        )
        
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.forward_fov_rad = math.radians(self.get_parameter('forward_fov_deg').value)
        self.invert_area_sign = self.get_parameter('invert_area_sign').value
        
        self.lock = threading.Lock()
        self.latest_scan_min = float('inf')
        
        # Placeholders for tracking individual areas
        self.latest_area_left = 0.0
        self.latest_area_right = 0.0
        
        callback_group = ReentrantCallbackGroup()
        
        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Strict QoS Profile to always use the LATEST message and drop lag
        self.latest_only_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Listen strictly to the area difference topic to act as loop heartbeat
        self.area_sub = self.create_subscription(
            Float32,
            '/voronoi/area_difference',
            self.area_callback,
            self.latest_only_qos,
            callback_group=callback_group
        )
        
        # Subscriptions for individual side-vector areas
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
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            self.latest_only_qos,
            callback_group=callback_group
        )
        
        self.get_logger().info(
            f"Voronoi Area PD Navigator initialized (Scaled Raw Diff Mode).\n"
            f"Parameters: kp_area={self.engine.kp_area}, kd_area={self.engine.kd_area}, "
            f"scale={self.engine.area_normalization_scale}, invert_sign={self.invert_area_sign}"
        )

    def scan_callback(self, msg: LaserScan):
        """Monitors a forward-facing wedge for physical obstacles."""
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

    # Callbacks to capture individual areas
    def area_left_callback(self, msg: Float32):
        with self.lock:
            self.latest_area_left = msg.data

    def area_right_callback(self, msg: Float32):
        with self.lock:
            self.latest_area_right = msg.data

    def area_callback(self, msg: Float32):
        """Processes incoming area difference measurements using raw inputs."""
        with self.lock:
            scan_dist = self.latest_scan_min
            area_left = self.latest_area_left
            area_right = self.latest_area_right
            
        # Swap left/right if the steering coordinates are inverted
        if self.invert_area_sign:
            area_left, area_right = area_right, area_left
            
        # Compute smooth command outputs based on raw areas
        linear_vel, angular_vel = self.engine.compute_controls(
            area_left, area_right, scan_dist, self.obstacle_threshold
        )
        
        # Log parameters with a 0.2 second throttle to keep terminal readable
        raw_diff = area_left - area_right
        norm_error = raw_diff / self.engine.area_normalization_scale
        
        self.get_logger().info(
            f"Areas -> Left: {area_left:7.1f} | Right: {area_right:7.1f} | "
            f"Raw Diff: {raw_diff:7.1f} | Norm Error: {norm_error:6.3f} | "
            f"Cmd Angular Speed: {angular_vel:6.3f} rad/s",
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