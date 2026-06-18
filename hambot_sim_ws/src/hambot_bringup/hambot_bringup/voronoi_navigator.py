#!/usr/bin/env python3
import math
import time
import threading
from collections import deque
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32, String
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

        # GLOBAL AREA DEACTIVATION: Sets areas to 0.0 if the corridor narrows below the threshold,
        # affecting both normal corridor driving and active split-handling turn overrides.
        if side_dist is not None and side_dist < self.deactivate_area_threshold:
            area_left = 0.0
            area_right = 0.0

        pos_valid = (path_x is not None)
        side_valid = (side_mid_x is not None and side_dist is not None and side_dist > max(self.min_side_distance, 1e-5))

        self.opposite_target_active = False

        if handling_split:
            error_area = 0.0
            error_side = 0.0
            p_term_area = 0.0
            p_term_side = 0.0
            
            error_pos = 0.0
            center_x = self.image_width / 2.0
            
            if split_direction == 'straight':
                if pos_valid:
                    error_pos = (center_x - path_x) / center_x
                kp_pos_active = max(self.kp_pos * 3.0, 0.6)
                kd_pos_active = max(self.kd_pos * 2.0, 0.1)
                    
            elif split_direction == 'left':
                left_off_sidewalk = False
                if side_valid:
                    left_x = side_mid_x - (side_dist / 2.0)
                    # Triggered immediately if area_left is deactivated to 0.0
                    if left_x <= 10.0 or area_left <= 0.01:
                        left_off_sidewalk = True
                
                if left_off_sidewalk:
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
                right_off_sidewalk = False
                if side_valid:
                    right_x = side_mid_x + (side_dist / 2.0)
                    # Triggered immediately if area_right is deactivated to 0.0
                    if right_x >= (self.image_width - 10.0) or area_right <= 0.01:
                        right_off_sidewalk = True
                
                if right_off_sidewalk:
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
            # (Redundant deactivation check removed here as it is handled at the start)
            raw_diff = area_left - area_right
            error_area = raw_diff / self.area_normalization_scale
            if abs(error_area) < self.area_deadband:
                error_area = 0.0
            p_term_area = self.kp_area * error_area
            
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
        
        base_speed = self.target_linear_speed * 1 if handling_split else self.target_linear_speed
        linear_vel = base_speed * speed_multiplier
        
        return linear_vel, self.smoothed_angular_vel

# =====================================================================
# ROS 2 WRAPPER NODE
# =====================================================================
class VoronoiNavigator(Node):
    STATE_NORMAL = 0
    STATE_SPLIT_AHEAD = 1
    STATE_HANDLING_SPLIT = 2

    def __init__(self):
        super().__init__('voronoi_navigator')
        
        self.declare_parameter('kp_area', 0.25)
        self.declare_parameter('kd_area', 0.05)
        self.declare_parameter('kp_pos', 0.2)
        self.declare_parameter('kd_pos', 0.05)
        self.declare_parameter('kp_side', 0.25)
        self.declare_parameter('kd_side', 0.025)
        self.declare_parameter('target_linear_speed', 0.35)
        self.declare_parameter('max_angular_speed', 0.13)
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

        self.declare_parameter('split_threshold', 0.5)             
        self.declare_parameter('split_exit_prob_threshold', 0.5)    
        self.declare_parameter('split_entry_area_threshold', 4000.0)  
        self.declare_parameter('split_exit_area_threshold', 2000.0)   
        
        self.declare_parameter('split_direction', 'straight')       
        
        self.declare_parameter('split_turn_kp', 0.22)                       
        self.declare_parameter('split_turn_kd', 0.1)                       
        self.declare_parameter('consecutive_frames_threshold_in', 15) 
        self.declare_parameter('consecutive_frames_threshold_out', 5)       
        self.declare_parameter('way_area_threshold', 6000.0)
        
        # Reduced default verification window to be responsive in simulator
        self.declare_parameter('min_split_duration', 3.0)                
        self.declare_parameter('require_target_area_confirmation', True) 

        self.split_threshold = self.get_parameter('split_threshold').value
        self.split_exit_prob_threshold = self.get_parameter('split_exit_prob_threshold').value
        self.split_entry_area_threshold = self.get_parameter('split_entry_area_threshold').value
        self.split_exit_area_threshold = self.get_parameter('split_exit_area_threshold').value
        self.consecutive_frames_threshold_in = self.get_parameter('consecutive_frames_threshold_in').value
        self.consecutive_frames_threshold_out = self.get_parameter('consecutive_frames_threshold_out').value
        self.way_area_threshold = self.get_parameter('way_area_threshold').value
        self.min_split_duration = self.get_parameter('min_split_duration').value
        self.require_target_area_confirmation = self.get_parameter('require_target_area_confirmation').value
        
        self.consecutive_splits_length = 25
        self.split_history = deque(maxlen=self.consecutive_splits_length)
        
        self.area_left_window = deque(maxlen=15)
        self.area_right_window = deque(maxlen=15)

        self.nav_state = self.STATE_NORMAL
        self.large_area_consecutive_count = 0
        self.clear_area_consecutive_count = 0
        self.split_handling_start_time = 0.0
        self._was_handling_split = False

        self.split_ways_record = {'left': 0, 'straight': 0, 'right': 0, 'total_frames': 0}

        self.genuine_split_confirmed = False
        self.target_area_observed_high = False

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
        self.latest_path_angle = None
        self.latest_path_top_x = None
        self.latest_side_mid_x = None
        self.latest_side_dist = None
        self.latest_split_prob = -1.0
        self.latest_candidate_paths = []
        
        self.active_split_direction = self.get_parameter('split_direction').value
        
        callback_group = ReentrantCallbackGroup()
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.target_dir_sub = self.create_subscription(
            String,
            '/global_planner/target_direction',
            self.target_direction_callback,
            10,
            callback_group=callback_group
        )
        
        self.turn_completed_pub = self.create_publisher(
            String,
            '/global_planner/turn_completed',
            10
        )
        
        self.latest_only_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.area_sub = self.create_subscription(
            Float32, '/voronoi/area_difference', self.area_callback, self.latest_only_qos, callback_group=callback_group
        )
        self.area_left_sub = self.create_subscription(
            Float32, '/voronoi/area_left', self.area_left_callback, self.latest_only_qos, callback_group=callback_group
        )
        self.area_right_sub = self.create_subscription(
            Float32, '/voronoi/area_right', self.area_right_callback, self.latest_only_qos, callback_group=callback_group
        )
        self.path_sub = self.create_subscription(
            PoseArray, '/voronoi/best_path', self.best_path_callback, self.latest_only_qos, callback_group=callback_group
        )
        self.candidate_paths_sub = self.create_subscription(
            PoseArray, '/voronoi/candidate_paths', self.candidate_paths_callback, self.latest_only_qos, callback_group=callback_group
        )
        self.side_mid_sub = self.create_subscription(
            Float32, '/voronoi/side_vector_mid_x', self.side_mid_callback, self.latest_only_qos, callback_group=callback_group
        )
        self.side_dist_sub = self.create_subscription(
            Float32, '/voronoi/side_vector_distance', self.side_dist_callback, self.latest_only_qos, callback_group=callback_group
        )
        self.split_prob_sub = self.create_subscription(
            Float32, '/voronoi/split_probability', self.split_prob_callback, self.latest_only_qos, callback_group=callback_group
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, self.latest_only_qos, callback_group=callback_group
        )
        
        self.get_logger().info("Voronoi Navigator initialized with active Global Planner integration.")

    def target_direction_callback(self, msg: String):
        with self.lock:
            direction = msg.data.lower()
            if direction in ['straight', 'left', 'right', 'stop']:
                if self.active_split_direction != direction:
                    self.active_split_direction = direction
                    self.get_logger().info(f"COORDINATION: Split direction updated dynamically to -> {direction.upper()}")

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
        with self.lock: self.latest_area_left = msg.data

    def area_right_callback(self, msg: Float32):
        with self.lock: self.latest_area_right = msg.data

    def best_path_callback(self, msg: PoseArray):
        with self.lock:
            if len(msg.poses) >= 2:
                p0_x, p0_y = msg.poses[0].position.x, msg.poses[0].position.y
                p1_x, p1_y = msg.poses[1].position.x, msg.poses[1].position.y
                if p0_y > p1_y:
                    p0_x, p0_y, p1_x, p1_y = p1_x, p1_y, p0_x, p0_y
                self.latest_path_x = (p0_x + p1_x) / 2.0
                self.latest_path_angle = math.atan2(p1_y - p0_y, p1_x - p0_x) * 180.0 / math.pi
                self.latest_path_top_x = p1_x
            else:
                self.latest_path_x = None
                self.latest_path_angle = None
                self.latest_path_top_x = None

    def candidate_paths_callback(self, msg: PoseArray):
        with self.lock:
            self.latest_candidate_paths = []
            num_poses = len(msg.poses)
            for i in range(0, num_poses - 1, 2):
                p0_x, p0_y = msg.poses[i].position.x, msg.poses[i].position.y
                p1_x, p1_y = msg.poses[i+1].position.x, msg.poses[i+1].position.y
                if p0_y > p1_y:
                    p0_x, p0_y, p1_x, p1_y = p1_x, p1_y, p0_x, p0_y
                angle = math.atan2(p1_y - p0_y, p1_x - p0_x) * 180.0 / math.pi
                self.latest_candidate_paths.append((angle, p1_x))

    def side_mid_callback(self, msg: Float32):
        with self.lock: self.latest_side_mid_x = msg.data if msg.data >= 0.0 else None

    def side_dist_callback(self, msg: Float32):
        with self.lock: self.latest_side_dist = msg.data if msg.data >= 0.0 else None

    def split_prob_callback(self, msg: Float32):
        with self.lock:
            self.latest_split_prob = msg.data
            if msg.data >= 0.0:
                self.split_history.append(msg.data)

    def reset_split_ways_record(self):
        self.split_ways_record = {'left': 0, 'straight': 0, 'right': 0, 'total_frames': 0}

    def log_final_split_summary(self):
        tf = self.split_ways_record['total_frames']
        if tf > 0:
            pct_l = (self.split_ways_record['left'] / tf) * 100.0
            pct_s = (self.split_ways_record['straight'] / tf) * 100.0
            pct_r = (self.split_ways_record['right'] / tf) * 100.0
            self.get_logger().info(
                f"\n====================================================\n"
                f"       SPLIT ACTIVE LAYOUT DETECTED (Frames: {tf})\n"
                f"====================================================\n"
                f"  LEFT WAY:     {pct_l:5.1f}%\n"
                f"  STRAIGHT WAY: {pct_s:5.1f}%\n"
                f"  RIGHT WAY:    {pct_r:5.1f}%\n"
                f"===================================================="
            )

    def log_passed_split_characteristics(self, duration: float):
        tf = self.split_ways_record['total_frames']
        if tf > 0:
            pct_l = (self.split_ways_record['left'] / tf) * 100.0
            pct_s = (self.split_ways_record['straight'] / tf) * 100.0
            pct_r = (self.split_ways_record['right'] / tf) * 100.0
            self.get_logger().info(
                f"\n====================================================\n"
                f"   SUCCESSFULLY PASSED GENUINE SPLIT (Cleared!)\n"
                f"   Navigated Direction: {self.active_split_direction.upper()}\n"
                f"   Active Phase Duration: {duration:.2f} seconds\n"
                f"====================================================\n"
                f"  SPLIT CHARACTERISTICS (APPROACH SUMMARY):\n"
                f"  LEFT BRANCH:     {pct_l:5.1f}%\n"
                f"  STRAIGHT BRANCH: {pct_s:5.1f}%\n"
                f"  RIGHT BRANCH:    {pct_r:5.1f}%\n"
                f"===================================================="
            )

    def classify_current_frame_ways(self, area_left, area_right, candidate_paths):
        left_detected, straight_detected, right_detected = False, False, False
        center_x = self.engine.image_width / 2.0
        offset = self.engine.image_width * 0.104
        left_bound, right_bound = center_x - offset, center_x + offset

        for path_angle, path_top_x in candidate_paths:
            if (70.0 <= path_angle <= 110.0) or (left_bound <= path_top_x <= right_bound):
                straight_detected = True
            if (path_angle > 110.0) and (path_top_x < left_bound):
                left_detected = True
            if (path_angle < 70.0) and (path_top_x > right_bound):
                right_detected = True

        if area_left >= self.way_area_threshold: left_detected = True
        if area_right >= self.way_area_threshold: right_detected = True

        self.split_ways_record['total_frames'] += 1
        if left_detected: self.split_ways_record['left'] += 1
        if straight_detected: self.split_ways_record['straight'] += 1
        if right_detected: self.split_ways_record['right'] += 1

    def area_callback(self, msg: Float32):
        split_turn_kp = self.get_parameter('split_turn_kp').value
        split_turn_kd = self.get_parameter('split_turn_kd').value
        consecutive_frames_threshold_in = self.get_parameter('consecutive_frames_threshold_in').value
        consecutive_frames_threshold_out = self.get_parameter('consecutive_frames_threshold_out').value

        with self.lock:
            scan_dist = self.latest_scan_min
            area_left = self.latest_area_left
            area_right = self.latest_area_right
            path_x = self.latest_path_x if self.use_path_position else None
            candidate_paths = list(self.latest_candidate_paths) if self.use_path_position else []
            side_mid_x = self.latest_side_mid_x if self.use_side_position else None
            side_dist = self.latest_side_dist if self.use_side_position else None
            split_prob = self.latest_split_prob
            split_direction = self.active_split_direction
            
            history_length = len(self.split_history)
            split_avg = sum(list(self.split_history)[:-15]) / 10.0 if history_length >= 25 else 0.0

        if split_direction == 'stop':
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            self.get_logger().info("COORDINATION: 'STOP' command active. Halting navigation.", throttle_duration_sec=1.0)
            return

        self.area_left_window.append(area_left)
        self.area_right_window.append(area_right)

        avg_left = sum(self.area_left_window) / len(self.area_left_window) if self.area_left_window else area_left
        avg_right = sum(self.area_right_window) / len(self.area_right_window) if self.area_right_window else area_right

        if self.nav_state == self.STATE_SPLIT_AHEAD:
            self.classify_current_frame_ways(area_left, area_right, candidate_paths)

        if self.nav_state == self.STATE_HANDLING_SPLIT:
            any_area_big = (avg_left >= self.split_exit_area_threshold or avg_right >= self.split_exit_area_threshold)
        else:
            any_area_big = (area_left >= self.split_entry_area_threshold or area_right >= self.split_entry_area_threshold)
        
        if any_area_big:
            self.large_area_consecutive_count += 1
            self.clear_area_consecutive_count = 0
        else:
            self.large_area_consecutive_count = 0
            self.clear_area_consecutive_count += 1

        areas_opened_confirmed = (self.large_area_consecutive_count >= consecutive_frames_threshold_in)
        areas_cleared_confirmed = (self.clear_area_consecutive_count >= consecutive_frames_threshold_out)

        # =====================================================================
        # STATE TRANSITIONS
        # =====================================================================
        if self.nav_state == self.STATE_NORMAL:
            if split_avg >= self.split_threshold:
                self.reset_split_ways_record()
                if areas_opened_confirmed:
                    self.classify_current_frame_ways(area_left, area_right, candidate_paths)
                    self.nav_state = self.STATE_HANDLING_SPLIT
                    self.split_handling_start_time = time.time()
                    self.genuine_split_confirmed = False
                    self.target_area_observed_high = False
                    self.get_logger().info(f"STATE TRANSITION: [NORMAL] -> [HANDLING SPLIT] (Dir: {split_direction.upper()})")
                    self.log_final_split_summary()
                else:
                    self.nav_state = self.STATE_SPLIT_AHEAD
                    self.get_logger().info("STATE TRANSITION: [NORMAL] -> [SPLIT AHEAD]")

        elif self.nav_state == self.STATE_SPLIT_AHEAD:
            if areas_opened_confirmed:
                self.nav_state = self.STATE_HANDLING_SPLIT
                self.split_handling_start_time = time.time()
                self.genuine_split_confirmed = False
                self.target_area_observed_high = False
                self.get_logger().info(f"STATE TRANSITION: [SPLIT AHEAD] -> [HANDLING SPLIT] (Dir: {split_direction.upper()})")
                self.log_final_split_summary()
            elif split_avg < self.split_threshold:
                self.nav_state = self.STATE_NORMAL
                self.get_logger().info("STATE TRANSITION: [SPLIT AHEAD] -> [NORMAL]")

        elif self.nav_state == self.STATE_HANDLING_SPLIT:
            duration = time.time() - self.split_handling_start_time
            
            if self.require_target_area_confirmation:
                if split_direction == 'left' and avg_left >= self.way_area_threshold:
                    self.target_area_observed_high = True
                elif split_direction == 'right' and avg_right >= self.way_area_threshold:
                    self.target_area_observed_high = True
                elif split_direction == 'straight' and (avg_left >= self.way_area_threshold or avg_right >= self.way_area_threshold):
                    self.target_area_observed_high = True
            else:
                self.target_area_observed_high = True

            if duration >= self.min_split_duration and self.target_area_observed_high:
                if not self.genuine_split_confirmed:
                    self.genuine_split_confirmed = True
                    self.get_logger().info(f"COORDINATION: Split execution verified ({duration:.1f}s). Ready to signal completion.")

            if areas_cleared_confirmed:
                self.nav_state = self.STATE_NORMAL
                self.large_area_consecutive_count = 0
                self.clear_area_consecutive_count = 0
                
                if self.genuine_split_confirmed:
                    self.get_logger().info("STATE TRANSITION: [HANDLING SPLIT] -> [NORMAL] (Split cleared!)")
                    self.log_passed_split_characteristics(duration)
                    
                    completion_msg = String()
                    completion_msg.data = "completed"
                    self.turn_completed_pub.publish(completion_msg)
                    self.get_logger().info("COORDINATION: Sent clearance signal to global planner.")
                else:
                    self.get_logger().warn(
                        f"STATE TRANSITION: [HANDLING SPLIT] -> [NORMAL] (Exited, but DISMISSED as transient noise! Duration: {duration:.2f}s). "
                        f"No clearance message sent to global planner. "
                        f"Requires minimum duration >= {self.min_split_duration}s (current: {duration:.2f}s) and "
                        f"target_area_observed_high={self.target_area_observed_high} (current: {self.target_area_observed_high})."
                    )

        handling_split_flag = (self.nav_state == self.STATE_HANDLING_SPLIT)

        if not handling_split_flag and self._was_handling_split:
            self.engine.prev_time = None

        if self.invert_area_sign:
            area_left, area_right = area_right, area_left
            center_x = self.engine.image_width / 2.0
            if path_x is not None: path_x = 2.0 * center_x - path_x
            if side_mid_x is not None: side_mid_x = 2.0 * center_x - side_mid_x
            
        linear_vel, angular_vel = self.engine.compute_controls(
            area_left, area_right, path_x, side_mid_x, side_dist, scan_dist, self.obstacle_threshold,
            handling_split=handling_split_flag, split_direction=split_direction,
            split_turn_kp=split_turn_kp, split_turn_kd=split_turn_kd
        )
        
        self._was_handling_split = handling_split_flag
        
        # Diagnostics Formatting
        raw_diff = area_left - area_right
        path_str = f"PathX: {path_x:5.1f}" if path_x is not None else "PathX: None"
        side_str = f"SideX: {side_mid_x:5.1f}" if side_mid_x is not None else "SideX: None"
        dist_str = f"SideDist: {side_dist:5.1f}" if side_dist is not None else "SideDist: None"
        
        if history_length >= 15:
            prob_str = f"SPLIT PROB: {split_prob*100.0:5.1f}% (Avg (Delayed): {split_avg*100.0:5.1f}%)"
        else:
            prob_str = f"SPLIT PROB: {split_prob*100.0:5.1f}% (Avg (Delayed): N/A {history_length}/15)"

        # =====================================================================
        # STATE PRINTING RESTORATION (BRINGING BACK THE ORIGINAL STATE INFO)
        # =====================================================================
        layout_str = ""
        if self.nav_state != self.STATE_NORMAL:
            tf = self.split_ways_record['total_frames']
            if tf > 0:
                pct_l = (self.split_ways_record['left'] / tf) * 100.0
                pct_s = (self.split_ways_record['straight'] / tf) * 100.0
                pct_r = (self.split_ways_record['right'] / tf) * 100.0
                layout_str = f"L {pct_l:.0f}%, S {pct_s:.0f}%, R {pct_r:.0f}%"
            else:
                layout_str = "Tracking..."

        if self.nav_state == self.STATE_NORMAL:
            status_str = "STATUS: [NORMAL]"
        elif self.nav_state == self.STATE_SPLIT_AHEAD:
            status_str = f"STATUS: [SPLIT AHEAD ({layout_str}) (Normal Areas, Navigating Normally...)]"
        elif self.nav_state == self.STATE_HANDLING_SPLIT:
            reasons = []
            if split_avg >= self.split_exit_prob_threshold:
                reasons.append(f"Prob: {split_avg*100.0:.1f}%")
            if avg_left >= self.split_exit_area_threshold:
                reasons.append(f"AvgL: {avg_left:.1f}")
            if avg_right >= self.split_exit_area_threshold:
                reasons.append(f"AvgR: {avg_right:.1f}")
            reasons_str = ", ".join(reasons)
            
            opp_str = " | OPPOSITE GUARDRAIL ACTIVE!" if self.engine.opposite_target_active else ""
            status_str = f"STATUS: [SPLIT ACTIVE ({layout_str}) (Held by: {reasons_str}) | DIR: {split_direction.upper()} (ARC TURN){opp_str}]"
        
        self.get_logger().info(
            f"Areas -> L: {area_left:6.1f} (Avg: {avg_left:6.1f}) | R: {area_right:6.1f} (Avg: {avg_right:6.1f}) | "
            f"RawDiff: {raw_diff:6.1f} | {path_str} | {side_str} ({dist_str}) | "
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