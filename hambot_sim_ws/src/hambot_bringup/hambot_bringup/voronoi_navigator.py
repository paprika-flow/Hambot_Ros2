#!/usr/bin/env python3
import math
import time
import threading
from collections import deque
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
        deactivate_area_threshold: float = 700.0  
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
        
        self.area_deadband = area_deadband
        self.pos_deadband = pos_deadband
        self.side_deadband = side_deadband
        
        self.prev_error_area = 0.0
        self.prev_error_pos = 0.0
        self.prev_error_side = 0.0
        
        self.prev_pos_valid = False
        self.prev_side_valid = False
        
        self.prev_time = None
        self.smoothed_angular_vel = 0.0
        
        # State tracking flag for dynamic safety override
        self.opposite_target_active = False

    def compute_controls(
        self, 
        area_left: float, 
        area_right: float, 
        path_x: float, 
        side_mid_x: float,
        side_dist: float,
        scan_min_dist: float, 
        obstacle_threshold: float,
        handling_split: bool = False,
        split_direction: str = 'straight',
        split_turn_kp: float = 0.15,
        split_turn_kd: float = 0.03
    ) -> tuple:
        current_time = time.time()
        
        if self.prev_time is None:
            dt = 0.033
        else:
            dt = current_time - self.prev_time
            if dt <= 0:
                dt = 0.001

        # Checked against zero division using 1e-5 floor guard
        pos_valid = (path_x is not None)
        side_valid = (side_mid_x is not None and side_dist is not None and side_dist > max(self.min_side_distance, 1e-5))

        self.opposite_target_active = False

        if handling_split:
            # =========================================================
            # SPLIT CONTROL LAWS (NESTED BY SELECTED DIRECTION)
            # =========================================================
            error_area = 0.0
            error_side = 0.0
            p_term_area = 0.0
            p_term_side = 0.0
            
            error_pos = 0.0
            center_x = self.image_width / 2.0
            
            if split_direction == 'straight':
                if pos_valid:
                    error_pos = (center_x - path_x) / center_x
                
                # Active centerline constants for straight tracking
                kp_pos_active = max(self.kp_pos * 3.0, 0.6)
                kd_pos_active = max(self.kd_pos * 2.0, 0.1)
                    
            elif split_direction == 'left':
                # Check if left side-vector has hit boundaries or is zero (off-sidewalk safety check)
                left_off_sidewalk = False
                if side_valid:
                    left_x = side_mid_x - (side_dist / 2.0)
                    if left_x <= 10.0 or area_left <= 0.01:
                        left_off_sidewalk = True
                
                if left_off_sidewalk:
                    # Safety Override: Target the opposite (right) vector to recover center
                    self.opposite_target_active = True
                    if side_valid:
                        right_x = side_mid_x + (side_dist / 2.0)
                        error_pos = (center_x - right_x) / center_x
                else:
                    if side_valid:
                        error_pos = (center_x - left_x) / center_x
                
                kp_pos_active = split_turn_kp
                kd_pos_active = split_turn_kd
                    
            elif split_direction == 'right':
                # Check if right side-vector has hit boundaries or is zero (off-sidewalk safety check)
                right_off_sidewalk = False
                if side_valid:
                    right_x = side_mid_x + (side_dist / 2.0)
                    if right_x >= (self.image_width - 10.0) or area_right <= 0.01:
                        right_off_sidewalk = True
                
                if right_off_sidewalk:
                    # Safety Override: Target the opposite (left) vector to recover center
                    self.opposite_target_active = True
                    if side_valid:
                        left_x = side_mid_x - (side_dist / 2.0)
                        error_pos = (center_x - left_x) / center_x
                else:
                    if side_valid:
                        error_pos = (center_x - right_x) / center_x
                
                kp_pos_active = split_turn_kp
                kd_pos_active = split_turn_kd

            if abs(error_pos) < self.pos_deadband:
                error_pos = 0.0
                
            p_term_pos = kp_pos_active * error_pos
            p_term = p_term_pos
            
            d_term_pos = 0.0
            if dt > 0 and pos_valid and self.prev_pos_valid:
                raw_derivative_pos = (error_pos - self.prev_error_pos) / dt
                d_term_pos = kd_pos_active * raw_derivative_pos
                
            d_term = d_term_pos

        else:
            # =========================================================
            # NORMAL NAVIGATION LAW: BLENDED PD LOOPS
            # =========================================================
            if side_dist is not None and side_dist < self.deactivate_area_threshold:
                area_left = 0.0
                area_right = 0.0

            raw_diff = area_left - area_right
            error_area = raw_diff / self.area_normalization_scale
            if abs(error_area) < self.area_deadband:
                error_area = 0.0
            p_term_area = self.kp_area * error_area
            
            # Forced to 0.0 during normal navigation and early split ahead phases
            p_term_pos = 0.0
            error_pos = 0.0
            
            error_side = 0.0
            if side_valid:
                center_x = self.image_width / 2.0
                error_side = (2.0 * (center_x - side_mid_x)) / side_dist
                if abs(error_side) < self.side_deadband:
                    error_side = 0.0
            p_term_side = self.kp_side * error_side
            
            p_term = p_term_area + p_term_pos + p_term_side
            
            d_term_area = 0.0
            if dt > 0:
                raw_derivative_area = (error_area - self.prev_error_area) / dt
                d_term_area = self.kd_area * raw_derivative_area

            # Forced to 0.0 during normal navigation and early split ahead phases
            d_term_pos = 0.0

            d_term_side = 0.0
            if dt > 0 and side_valid and self.prev_side_valid:
                raw_derivative_side = (error_side - self.prev_error_side) / dt
                d_term_side = self.kd_side * raw_derivative_side

            d_term = d_term_area + d_term_pos + d_term_side
        
        raw_angular_vel = p_term + d_term
        raw_angular_vel = max(-self.max_angular_speed, min(self.max_angular_speed, raw_angular_vel))
        
        self.smoothed_angular_vel = (self.smoothing_factor * raw_angular_vel) + \
                                    ((1.0 - self.smoothing_factor) * self.smoothed_angular_vel)
                                    
        self.prev_error_area = error_area
        self.prev_error_pos = error_pos
        self.prev_error_side = error_side
        self.prev_pos_valid = pos_valid
        self.prev_side_valid = side_valid
        self.prev_time = current_time
        
        norm_correction = abs(raw_angular_vel) / self.max_angular_speed
        speed_multiplier = max(0.3, 1.0 - norm_correction * 0.7)
        
        base_speed = self.target_linear_speed * 0.85 if handling_split else self.target_linear_speed
        linear_vel = base_speed * speed_multiplier
        
        return linear_vel, self.smoothed_angular_vel


# =====================================================================
# ROS 2 WRAPPER NODE
# =====================================================================
class VoronoiNavigator(Node):
    # State Machine Constants
    STATE_NORMAL = 0
    STATE_SPLIT_AHEAD = 1
    STATE_HANDLING_SPLIT = 2

    def __init__(self):
        super().__init__('voronoi_navigator')
        
        self.declare_parameter('kp_area', 0.35)
        self.declare_parameter('kd_area', 0.075)
        self.declare_parameter('kp_pos', 0.2)
        self.declare_parameter('kd_pos', 0.05)
        self.declare_parameter('kp_side', 0.35)
        self.declare_parameter('kd_side', 0.05)
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
        self.declare_parameter('min_side_distance', 0.0)           
        self.declare_parameter('deactivate_area_threshold', 750.0)   
        self.declare_parameter('area_deadband', 0.015)
        self.declare_parameter('pos_deadband', 0.03)
        self.declare_parameter('side_deadband', 0.03)

        # Hysteresis Split Parameters
        self.declare_parameter('split_threshold', 0.75)             
        self.declare_parameter('split_exit_prob_threshold', 0.8)    
        self.declare_parameter('split_exit_area_threshold', 2000.0)  
        
        self.declare_parameter('split_direction', 'straight')       
        
        # Split Turn Tuning Parameters
        self.declare_parameter('split_turn_kp', 0.22)                       
        self.declare_parameter('split_turn_kd', 0.1)                       
        self.declare_parameter('consecutive_frames_threshold', 4)       
        
        self.split_threshold = self.get_parameter('split_threshold').value
        self.split_exit_prob_threshold = self.get_parameter('split_exit_prob_threshold').value
        self.split_exit_area_threshold = self.get_parameter('split_exit_area_threshold').value
        self.consecutive_frames_threshold = self.get_parameter('consecutive_frames_threshold').value
        self.consecutive_splits_length = 15

        self.split_history = deque(maxlen=self.consecutive_splits_length)
        
        # State machine tracking variables
        self.nav_state = self.STATE_NORMAL
        self.large_area_consecutive_count = 0
        self.clear_area_consecutive_count = 0
        self.split_handling_start_time = 0.0
        self._was_handling_split = False

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
        
        self.latest_area_left = 0.0
        self.latest_area_right = 0.0
        self.latest_path_x = None
        self.latest_side_mid_x = None
        self.latest_side_dist = None
        self.latest_split_prob = -1.0
        
        callback_group = ReentrantCallbackGroup()
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
        
        self.split_prob_sub = self.create_subscription(
            Float32,
            '/voronoi/split_probability',
            self.split_prob_callback,
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
        
        self.get_logger().info("Voronoi Blended PID Navigator with Hysteresis, Safety Guardrail, & Arc Turning initialized.")

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

    def split_prob_callback(self, msg: Float32):
        with self.lock:
            self.latest_split_prob = msg.data
            if msg.data >= 0.0:
                self.split_history.append(msg.data)

    def area_callback(self, msg: Float32):
        split_direction = self.get_parameter('split_direction').get_parameter_value().string_value
        if split_direction not in ['straight', 'left', 'right']:
            split_direction = 'straight'

        split_turn_kp = self.get_parameter('split_turn_kp').get_parameter_value().double_value
        split_turn_kd = self.get_parameter('split_turn_kd').get_parameter_value().double_value
        consecutive_frames_threshold = self.get_parameter('consecutive_frames_threshold').get_parameter_value().integer_value

        with self.lock:
            scan_dist = self.latest_scan_min
            area_left = self.latest_area_left
            area_right = self.latest_area_right
            path_x = self.latest_path_x if self.use_path_position else None
            side_mid_x = self.latest_side_mid_x if self.use_side_position else None
            side_dist = self.latest_side_dist if self.use_side_position else None
            split_prob = self.latest_split_prob
            
            history_length = len(self.split_history)
            if history_length >= self.consecutive_splits_length:
                split_avg = sum(self.split_history) / float(self.consecutive_splits_length)
            else:
                split_avg = 0.0
            
        # Determine if either side-vector area exceeds the trigger limit
        any_area_big = (area_left >= self.split_exit_area_threshold or area_right >= self.split_exit_area_threshold)
        
        # Debounce logic for entry and exit transitions using a single frame threshold
        if any_area_big:
            self.large_area_consecutive_count += 1
            self.clear_area_consecutive_count = 0
        else:
            self.large_area_consecutive_count = 0
            self.clear_area_consecutive_count += 1

        # Evaluate confirmation states based on consecutive frames
        areas_opened_confirmed = (self.large_area_consecutive_count >= consecutive_frames_threshold)
        areas_cleared_confirmed = (self.clear_area_consecutive_count >= consecutive_frames_threshold)

        # =====================================================================
        # THREE-STATE MACHINE TRANSITIONS (DEBOUNCED)
        # =====================================================================
        if self.nav_state == self.STATE_NORMAL:
            if split_avg >= self.split_threshold:
                if areas_opened_confirmed:
                    self.nav_state = self.STATE_HANDLING_SPLIT
                    self.split_handling_start_time = time.time()
                    self.get_logger().info(f"STATE TRANSITION: [NORMAL] -> [HANDLING SPLIT] (Dir: {split_direction.upper()})")
                else:
                    self.nav_state = self.STATE_SPLIT_AHEAD
                    self.get_logger().info("STATE TRANSITION: [NORMAL] -> [SPLIT AHEAD] (Split detected ahead. Navigating normally.)")

        elif self.nav_state == self.STATE_SPLIT_AHEAD:
            if areas_opened_confirmed:
                self.nav_state = self.STATE_HANDLING_SPLIT
                self.split_handling_start_time = time.time()
                self.get_logger().info(f"STATE TRANSITION: [SPLIT AHEAD] -> [HANDLING SPLIT] (Areas opened up continuously! Activating {split_direction.upper()} controls.)")
            elif split_avg < self.split_threshold:
                self.nav_state = self.STATE_NORMAL
                self.get_logger().info("STATE TRANSITION: [SPLIT AHEAD] -> [NORMAL] (Split cleared before areas opened up.)")

        elif self.nav_state == self.STATE_HANDLING_SPLIT:
            prob_cleared = (split_avg < self.split_exit_prob_threshold)
            
            # The exit condition now requires areas to be cleared for the specified consecutive frames
            if prob_cleared and areas_cleared_confirmed:
                self.nav_state = self.STATE_NORMAL
                self.large_area_consecutive_count = 0
                self.clear_area_consecutive_count = 0
                self.get_logger().info("STATE TRANSITION: [HANDLING SPLIT] -> [NORMAL] (Split successfully cleared.)")

        handling_split_flag = (self.nav_state == self.STATE_HANDLING_SPLIT)

        # Reset transition derivative jump spikes
        if not handling_split_flag and self._was_handling_split:
            self.engine.prev_time = None

        if self.invert_area_sign:
            area_left, area_right = area_right, area_left
            center_x = self.engine.image_width / 2.0
            if path_x is not None:
                path_x = 2.0 * center_x - path_x
            if side_mid_x is not None:
                side_mid_x = 2.0 * center_x - side_mid_x
            
        # Compute controls
        linear_vel, angular_vel = self.engine.compute_controls(
            area_left, area_right, path_x, side_mid_x, side_dist, scan_dist, self.obstacle_threshold,
            handling_split=handling_split_flag, split_direction=split_direction,
            split_turn_kp=split_turn_kp, split_turn_kd=split_turn_kd
        )
        
        self._was_handling_split = handling_split_flag
        
        # Log formatting
        raw_diff = area_left - area_right
        path_str = f"PathX: {path_x:5.1f}" if path_x is not None else "PathX: None"
        side_str = f"SideX: {side_mid_x:5.1f}" if side_mid_x is not None else "SideX: None"
        dist_str = f"SideDist: {side_dist:5.1f}" if side_dist is not None else "SideDist: None"
        
        area_active = "ACTIVE" if (side_dist is None or side_dist >= self.engine.deactivate_area_threshold) else "INACTIVE"
        side_active = "ACTIVE" if (side_mid_x is not None and side_dist is not None and side_dist >= self.engine.min_side_distance) else "INACTIVE"
        
        if history_length >= 10:
            prob_str = f"SPLIT PROB: {split_prob*100.0:5.1f}% (Avg: {split_avg*100.0:5.1f}%)"
        else:
            prob_str = f"SPLIT PROB: {split_prob*100.0:5.1f}% (Avg: N/A {history_length}/10)"

        # Set status logs depending on active state
        if self.nav_state == self.STATE_NORMAL:
            status_str = "STATUS: [NORMAL]"
        elif self.nav_state == self.STATE_SPLIT_AHEAD:
            status_str = "STATUS: [SPLIT AHEAD (Normal Areas, Navigating Normally...)]"
        elif self.nav_state == self.STATE_HANDLING_SPLIT:
            reasons = []
            if split_avg >= self.split_exit_prob_threshold:
                reasons.append(f"Prob: {split_avg*100.0:.1f}%")
            if area_left >= self.split_exit_area_threshold:
                reasons.append(f"AreaL: {area_left:.1f}")
            if area_right >= self.split_exit_area_threshold:
                reasons.append(f"AreaR: {area_right:.1f}")
            reasons_str = ", ".join(reasons)
            
            opp_str = " | OPPOSITE GUARDRAIL ACTIVE!" if self.engine.opposite_target_active else ""
            status_str = f"STATUS: [SPLIT ACTIVE (Held by: {reasons_str}) | DIR: {split_direction.upper()} (ARC TURN){opp_str}]"
        
        self.get_logger().info(
            f"Areas -> L: {area_left:6.1f} | R: {area_right:6.1f} (Area: {area_active}) | "
            f"RawDiff: {raw_diff:6.1f} | {path_str} | {side_str} ({dist_str}, Side: {side_active}) | "
            f"{prob_str} {status_str} | Cmd Angular: {angular_vel:6.3f} rad/s",
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

    '''
    Open a new terminal and print these commands to change the parameters for the direction of the split.

    STRAIGHT 

    ros2 param set /voronoi_navigator split_direction straight

    RIGHT

    ros2 param set /voronoi_navigator split_direction right

    LEFT

    ros2 param set /voronoi_navigator split_direction left

    '''