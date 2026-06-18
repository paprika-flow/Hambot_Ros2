#!/usr/bin/env python3
"""
costmap_navigator.py — Local costmap planner for sidewalk navigation.

Architecture:
  BehaviorTree    — Framework: Sequence, Fallback, Condition, Action nodes
  CostmapGrid     — Pure numpy costmap. No ROS.
  CostmapNavigator — ROS 2 node. Uses BT for decision-making.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math
import heapq
import os
import yaml
from ament_index_python.packages import get_package_share_directory


# ════════════════════════════════════════════════════════════════
# BEHAVIOR TREE FRAMEWORK
# ════════════════════════════════════════════════════════════════

class BT:
    """Status codes every node returns."""
    SUCCESS = 'SUCCESS'
    FAILURE = 'FAILURE'
    RUNNING = 'RUNNING'


class BTNode:
    """Every node has a tick(blackboard) that returns a BT status."""
    def tick(self, bb):
        raise NotImplementedError


class Sequence(BTNode):
    """
    Run children left to right.
    - If child returns SUCCESS → advance to next child
    - If child returns FAILURE → whole sequence FAILURE
    - If child returns RUNNING → pause, resume same child next tick
    - All children SUCCESS → whole sequence SUCCESS

    Stores current child index on blackboard as 'bt:seq:<name>:i'.
    """

    def __init__(self, children, name='seq'):
        self.children = children
        self.name = name

    def tick(self, bb):
        key = f'bt:seq:{self.name}:i'
        i = bb.get(key, 0)

        while i < len(self.children):
            status = self.children[i].tick(bb)
            if status == BT.RUNNING:
                bb[key] = i
                return BT.RUNNING
            if status == BT.FAILURE:
                bb.pop(key, None)
                return BT.FAILURE
            i += 1  # SUCCESS → next child

        bb.pop(key, None)
        return BT.SUCCESS


class Fallback(BTNode):
    """
    Run children left to right.
    - If child returns FAILURE → advance to next child
    - If child returns SUCCESS → whole fallback SUCCESS
    - If child returns RUNNING → pause, resume same child next tick
    - All children FAILURE → whole fallback FAILURE
    """

    def __init__(self, children, name='fb'):
        self.children = children
        self.name = name

    def tick(self, bb):
        key = f'bt:fb:{self.name}:i'
        i = bb.get(key, 0)

        while i < len(self.children):
            status = self.children[i].tick(bb)
            if status == BT.RUNNING:
                bb[key] = i
                return BT.RUNNING
            if status == BT.SUCCESS:
                bb.pop(key, None)
                return BT.SUCCESS
            i += 1  # FAILURE → next child

        bb.pop(key, None)
        return BT.FAILURE


class Condition(BTNode):
    """
    Leaf: checks a condition immediately.
    fn(bb) returns True → SUCCESS, False → FAILURE.
    """

    def __init__(self, fn, name='cond'):
        self.fn = fn
        self.name = name

    def tick(self, bb):
        return BT.SUCCESS if self.fn(bb) else BT.FAILURE


class Action(BTNode):
    """
    Leaf: does work, may be multi-tick.
    fn(bb) returns BT.SUCCESS / BT.FAILURE / BT.RUNNING.
    """

    def __init__(self, fn, name='act'):
        self.fn = fn
        self.name = name

    def tick(self, bb):
        return self.fn(bb)


# ════════════════════════════════════════════════════════════════
# ROUTE MANAGER
# ════════════════════════════════════════════════════════════════

def euler_from_quaternion(q):
    """Extract yaw from ROS Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RouteManager:
    """
    Tracks progress through an ordered list of waypoints.

    Each tick: caller calls check_arrival(x, y, yaw).
    If distance to current waypoint < threshold, advance index.
    If waypoint is significantly behind robot, skip it.

    Public interface:
      .current()      → waypoint name or None
      .current_pose() → (x, y) or None
      .finished       → bool (True after last waypoint consumed)
      .advance_count  → int (how many waypoints completed)
    """

    def __init__(self, waypoint_names, waypoint_poses, threshold=1.5):
        """
        waypoint_names: ordered list of strings (from route file)
        waypoint_poses: dict name → (x_world, y_world)
        threshold: meters — robot within this = arrived
        """
        self.names = waypoint_names
        self.poses = waypoint_poses
        self.threshold = threshold
        self.index = 0
        self.finished = False
        self.advance_count = 0
        self._arrived = set()  # waypoint indices already claimed

    def current(self):
        """Return current waypoint name or None if finished."""
        if self.finished or self.index >= len(self.names):
            return None
        name = self.names[self.index]
        if name == '__stop__':
            return None
        return name

    def current_pose(self):
        """Return (x, y) of current waypoint or None."""
        name = self.current()
        if name is None:
            return None
        pose = self.poses.get(name)
        if pose is None:
            return None
        return pose  # (x, y) only

    def check_arrival(self, x, y, yaw):
        """
        Check if robot reached current waypoint.
        Advances index if within threshold or significantly behind.
        Call every tick. Logs advances.
        Returns (advanced, skipped) booleans.
        """
        if self.finished or self.index >= len(self.names):
            return False, False

        name = self.names[self.index]

        # __stop__ is end marker
        if name == '__stop__':
            self.finished = True
            return True, False

        pose = self.poses.get(name)
        if pose is None:
            # Unknown waypoint name — skip it
            self.index += 1
            self.advance_count += 1
            return False, 'unknown'

        # Already claimed this index?
        if self.index in self._arrived:
            return False, False

        wx, wy = pose[:2]
        dx = wx - x
        dy = wy - y
        dist = math.hypot(dx, dy)

        # Vector from robot to waypoint (forward component in robot's heading)
        fwd = dx * math.cos(yaw) + dy * math.sin(yaw)

        # Arrived by proximity
        if dist < self.threshold:
            self._arrived.add(self.index)
            self.index += 1
            self.advance_count += 1
            return True, False

        # Waypoint significantly behind AND close — robot missed/passed it
        if fwd < -self.threshold * 2 and dist < self.threshold * 4:
            self._arrived.add(self.index)
            self.index += 1
            self.advance_count += 1
            return False, True

        return False, False

    def remaining(self):
        """Return number of waypoints left (excluding __stop__)."""
        count = 0
        for i in range(self.index, len(self.names)):
            if self.names[i] == '__stop__':
                break
            count += 1
        return count

    def total_waypoints(self):
        """Total waypoints excluding __stop__."""
        return sum(1 for n in self.names if n != '__stop__')

    def progress_str(self):
        total = self.total_waypoints()
        done = self.advance_count
        if total == 0:
            return 'unknown'
        return f'{done}/{total} waypoints'


# ════════════════════════════════════════════════════════════════
# COSTMAP GRID
# ════════════════════════════════════════════════════════════════

class CostmapGrid:
    """2D grid of drivable/obstacle/unknown costs.

    Values:
      0     = free (sidewalk interior)
      1-99  = edge gradient (near grass)
      100   = hard edge
      254   = unknown
      255   = lethal (grass, LiDAR obstacle)
    """

    def __init__(self, rows, cols, cell_size, map_fwd, obs_inflate):
        self.rows = rows
        self.cols = cols
        self.cell_size = cell_size
        self.map_fwd = map_fwd
        self.obs_inflate = obs_inflate
        self.costmap = np.full((rows, cols), 254, dtype=np.uint8)

    def fill_from_camera(self, mask, px_valid, px_row, px_col, robot_cell):
        self.costmap.fill(254)
        valid = px_valid
        r = px_row[valid]
        c = px_col[valid]
        mv = mask[valid]
        sw = mv > 127
        self.costmap[r[sw], c[sw]] = 0
        nosw = mv <= 127
        self.costmap[r[nosw], c[nosw]] = 255
        self.costmap[robot_cell] = 0

    def inflate_edges(self, grass_mask):
        if not np.any(grass_mask):
            return
        dist = self._distance_transform(grass_mask, 40)
        max_d = 8
        on_sw = (self.costmap == 0) & ~grass_mask
        edge = np.zeros((self.rows, self.cols), dtype=np.uint8)
        edge[on_sw] = np.clip(
            100 * (1.0 - dist[on_sw].astype(np.float32) / max_d), 0, 100
        ).astype(np.uint8)
        self.costmap = np.maximum(
            self.costmap.astype(np.uint16), edge.astype(np.uint16)
        ).astype(np.uint8)

    def apply_lidar_gradient(self, points, robot_cell):
        if not points:
            return
        hits = np.zeros((self.rows, self.cols), dtype=bool)
        map_lat = self.cols * self.cell_size / 2.0
        for (x, y) in points:
            r = self.rows - 1 - int(x / self.cell_size)
            c = int((y + map_lat) / self.cell_size)
            if 0 <= r < self.rows and 0 <= c < self.cols:
                hits[r, c] = True
        if not np.any(hits):
            return
        inflate = int(self.obs_inflate / self.cell_size)
        dist = self._distance_transform(hits, inflate + 2)
        obs = np.zeros((self.rows, self.cols), dtype=np.uint8)
        near = dist <= inflate
        obs[near] = (255 * (1.0 - dist[near].astype(np.float32) / inflate)).astype(np.uint8)
        self.costmap = np.maximum(
            self.costmap.astype(np.uint16), obs.astype(np.uint16)
        ).astype(np.uint8)
        self.costmap[robot_cell] = 0

    def fill_unknown_forward(self, seen_cols_per_row):
        for r in range(self.rows):
            if r not in seen_cols_per_row:
                continue
            left, right = seen_cols_per_row[r]
            row = self.costmap[r, :]
            unk = (row[left:right + 1] == 254)
            row[left:right + 1][unk] = 0

    def find_goal(self, goal_row, bias_col=None):
        lo = min(self.rows - 2, goal_row + 5)
        hi = max(1, goal_row - 5)
        for r in range(lo, hi - 1, -1):
            sw = np.where(self.costmap[r, :] < 200)[0]
            if len(sw) > 0:
                if bias_col is not None:
                    center = max(0, min(self.cols - 1, bias_col))
                else:
                    center = self.cols // 2
                best = sw[np.argmin(np.abs(sw - center))]
                return (r, best)
        return None

    def astar(self, start, goal):
        if not (0 <= goal[0] < self.rows and 0 <= goal[1] < self.cols):
            return None
        if self.costmap[goal] >= 255:
            return None
        h = lambda a, b: math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)
        cm = self.costmap
        open_set = [(0.0, start)]
        came_from = {}
        g_score = {start: 0.0}
        max_iter = self.rows * self.cols * 2
        for _ in range(max_iter):
            if not open_set:
                break
            _, cur = heapq.heappop(open_set)
            if cur == goal:
                break
            cr, cc = cur
            for nr, nc in [(cr - 1, cc), (cr + 1, cc), (cr, cc - 1), (cr, cc + 1)]:
                if not (0 <= nr < self.rows and 0 <= nc < self.cols):
                    continue
                cell = cm[nr, nc]
                if cell >= 255:
                    continue
                move = 1.0 + (cell / 255.0) * 10.0
                tent = g_score[cur] + move
                if (nr, nc) not in g_score or tent < g_score[(nr, nc)]:
                    came_from[(nr, nc)] = cur
                    g_score[(nr, nc)] = tent
                    heapq.heappush(open_set, (tent + h((nr, nc), goal), (nr, nc)))
        else:
            return None
        if goal not in came_from and cur != goal:
            return None
        path = []
        node = goal if (goal in came_from or goal == start) else cur
        while node in came_from:
            path.append(node)
            node = came_from[node]
        path.append(start)
        path.reverse()
        return path

    def _distance_transform(self, seed_mask, max_iter):
        dist = np.full((self.rows, self.cols), 255, dtype=np.uint16)
        dist[seed_mask] = 0
        for _ in range(max_iter):
            prev = dist.copy()
            d = dist
            d[1:, :] = np.minimum(d[1:, :], (prev[:-1, :] + 1).astype(np.uint16))
            d[:-1, :] = np.minimum(d[:-1, :], (prev[1:, :] + 1).astype(np.uint16))
            d[:, 1:] = np.minimum(d[:, 1:], (prev[:, :-1] + 1).astype(np.uint16))
            d[:, :-1] = np.minimum(d[:, :-1], (prev[:, 1:] + 1).astype(np.uint16))
            if np.array_equal(d, prev):
                break
        return dist


# ════════════════════════════════════════════════════════════════
# BT CONDITIONS & ACTIONS
# ════════════════════════════════════════════════════════════════

# ── Route conditions ──

def cond_route_active(bb):
    """Route loaded and not finished."""
    route = bb.get('route')
    if route is None or route.finished:
        return False
    # Current waypoint exists OR we're at __stop__ (need CheckArrival to set finished)
    if route.current() is not None:
        return True
    # Check if index points to __stop__ without finished flag yet
    idx = route.index
    return idx < len(route.names) and route.names[idx] == '__stop__'


def cond_route_inactive(bb):
    """No route loaded (free navigation mode)."""
    route = bb.get('route')
    return route is None


def cond_route_finished(bb):
    """Route loaded and complete."""
    route = bb.get('route')
    if route is None:
        return False
    return route.finished


def cond_goal_found(bb):
    return bb.get('goal') is not None


def cond_path_found(bb):
    p = bb.get('path')
    return p is not None and len(p) >= 2


# ── Route actions ──

def act_check_arrival(bb):
    """
    Check odometry against current waypoint.
    Advances route index if close enough or waypoint is behind.
    Always SUCCESS — checking never fails.
    """
    route = bb.get('route')
    node = bb['node']
    if route is None or route.finished:
        return BT.SUCCESS

    x = bb.get('odom_x', 0.0)
    y = bb.get('odom_y', 0.0)
    yaw = bb.get('odom_yaw', 0.0)

    # Save current waypoint name before advance (for ARRIVED log)
    old_name = route.current()

    advanced, skipped = route.check_arrival(x, y, yaw)

    if advanced:
        if old_name:
            node.get_logger().info(f'ARRIVED at {old_name}')
        cur = route.current()
        if cur:
            pose = route.current_pose()
            dist_str = ''
            if pose:
                dx = bb.get('odom_x', 0.0) - pose[0]
                dy = bb.get('odom_y', 0.0) - pose[1]
                dist_str = f' \u2014 {math.hypot(dx, dy):.1f}m away'
            node.get_logger().info(
                f'NEXT WP: {cur} at ({pose[0]:.1f}, {pose[1]:.1f}){dist_str}'
            )
        else:
            node.get_logger().info('ARRIVED at final waypoint \u2014 mission complete')
    elif skipped is True:
        rem = route.remaining()
        cur = route.current()
        if cur:
            pose = route.current_pose()
            if pose:
                node.get_logger().info(
                    f'Skipped waypoint (behind). NEXT WP: {cur} at ({pose[0]:.1f}, {pose[1]:.1f}) — {rem} remaining.'
                )
        else:
            node.get_logger().info(f'Skipped waypoint "{old_name}" — reached __stop__')
    elif skipped == 'unknown':
        node.get_logger().warn(f'Skipped waypoint "{old_name}" — not found in SDF.')

    return BT.SUCCESS


def act_compute_waypoint_bias(bb):
    """
    Compute lateral costmap bias toward current waypoint.
    Converts world-frame waypoint → robot-relative lateral offset → costmap column shift.
    Stores result in bb['bias_col'].
    """
    route = bb.get('route')
    if route is None or route.finished:
        bb['bias_col'] = None
        return BT.SUCCESS

    name = route.current()
    if name is None:
        bb['bias_col'] = None
        return BT.SUCCESS

    pose = route.current_pose()
    if pose is None:
        bb['bias_col'] = None
        return BT.SUCCESS

    x = bb.get('odom_x', 0.0)
    y = bb.get('odom_y', 0.0)
    yaw = bb.get('odom_yaw', 0.0)
    cols = bb['cols']
    cs = bb['cell_size']

    wx, wy = pose

    # Vector from robot to waypoint, in world frame
    dx_world = wx - x
    dy_world = wy - y

    # Rotate into robot's local frame
    dx_local = dx_world * math.cos(yaw) + dy_world * math.sin(yaw)
    dy_local = -dx_world * math.sin(yaw) + dy_world * math.cos(yaw)

    # Log throttled to ~1 Hz (every 5th frame at 5 Hz camera)
    node = bb['node']
    route = bb.get('route')
    cur_name = route.current() if route else None
    dist = math.hypot(dx_world, dy_world)

    counter = bb.setdefault('_log_counter', 0) + 1
    bb['_log_counter'] = counter
    do_log = (counter % 5 == 0)

    if dx_local < -1.0:
        bb['bias_col'] = None
        if do_log:
            node.get_logger().info(
                f'route: odom=({x:.2f},{y:.2f}) yaw={yaw:.2f} '
                f'wp={cur_name} at ({wx:.1f},{wy:.1f}) '
                f'local=({dx_local:.1f},{dy_local:.1f}) dist={dist:.1f}m '
                f'behind bias_col=None'
            )
        return BT.SUCCESS

    # Convert lateral offset to costmap column
    col_offset = int(dy_local / cs)
    bias_col = (cols // 2) + col_offset
    bias_col = max(0, min(cols - 1, bias_col))
    bb['bias_col'] = bias_col

    if do_log:
        node.get_logger().info(
            f'route: odom=({x:.2f},{y:.2f}) yaw={yaw:.2f} '
            f'wp={cur_name} at ({wx:.1f},{wy:.1f}) '
            f'local=({dx_local:.1f},{dy_local:.1f}) dist={dist:.1f}m '
            f'active bias_col={bias_col}'
        )

    return BT.SUCCESS


# ── Core navigation actions ──

def act_find_goal(bb):
    """Run goal search on the built costmap. Optionally shifted by bias_col."""
    grid = bb['grid']
    bias_col = bb.get('bias_col', None)
    goal = grid.find_goal(bb['goal_row'], bias_col=bias_col)
    bb['goal'] = goal
    return BT.SUCCESS if goal is not None else BT.FAILURE


def act_plan_path(bb):
    """Run A*. Stores path in bb."""
    grid = bb['grid']
    start = bb['robot_cell']
    goal = bb['goal']
    path = grid.astar(start, goal)
    bb['path'] = path
    return BT.SUCCESS if path and len(path) >= 2 else BT.FAILURE


def act_follow_path(bb):
    """
    Compute steering angle from A* path + cross-track centering.
    Publishes cmd_vel immediately.
    Returns SUCCESS (steady state — re-evaluates next tick).
    """
    path = bb['path']
    node = bb['node']
    grid = bb['grid']
    kp = bb['kp']
    speed = bb['linear_speed']
    rows = bb['rows']
    cols = bb['cols']

    twist = Twist()
    if len(path) < 2:
        node.vel_pub.publish(twist)
        return BT.SUCCESS

    la_cells = int(1.0 / grid.cell_size)
    la = min(la_cells, len(path) - 1)
    sr, sc = path[0]
    tr, tc = path[la]
    dr = -(tr - sr)
    dc = tc - sc

    if dr <= 0:
        node.vel_pub.publish(twist)
        return BT.SUCCESS

    path_angle = math.atan2(dc * grid.cell_size, dr * grid.cell_size)

    # Cross-track centering
    ctr_row = max(0, rows - 1 - la_cells)
    cross_angle = 0.0
    row_costs = grid.costmap[ctr_row, :]
    sw = np.where(row_costs < 100)[0]
    if len(sw) > 4:
        center = (sw[0] + sw[-1]) / 2.0
        cte = center - cols // 2
        half = max(1, (sw[-1] - sw[0]) / 2.0)
        cross_angle = kp * (cte / half) * 0.4

    final = 0.6 * path_angle + 0.4 * cross_angle
    # Dynamic speed: slower on sharp turns, faster on straights
    # max_angular = max reasonable turn rate (rad/s)
    max_angular = 0.5
    speed_scale = max(0.25, 1.0 - abs(final) / max_angular)
    twist.linear.x = speed * speed_scale
    twist.angular.z = final
    node.vel_pub.publish(twist)
    return BT.SUCCESS


def act_stop(bb):
    """Publish zero cmd_vel. Used as last-resort fallback."""
    node = bb['node']
    node.vel_pub.publish(Twist())
    return BT.SUCCESS


def act_log_mission_complete(bb):
    """Log mission complete. Does not stop — visual centering keeps robot on path."""
    if bb.get('_mission_logged', False):
        return BT.SUCCESS
    bb['_mission_logged'] = True
    node = bb['node']
    node.get_logger().info('ARRIVED at final waypoint — mission complete')
    node.get_logger().info('Stopping.')
    return BT.SUCCESS


# ════════════════════════════════════════════════════════════════
# ROS 2 NODE
# ════════════════════════════════════════════════════════════════

class CostmapNavigator(Node):
    """
    ROS node. Builds costmap on each camera frame, then runs BT
    to decide what to do with it.
    """

    def __init__(self):
        super().__init__('costmap_navigator')

        # ── Parameters ──
        self.declare_parameter('segmentation_topic', '/camera/sidewalk_mask')
        self.declare_parameter('map_forward', 3.0)
        self.declare_parameter('map_backward', 0.5)
        self.declare_parameter('map_lateral', 1.5)
        self.declare_parameter('cell_size', 0.05)
        self.declare_parameter('cam_height', 0.341)
        self.declare_parameter('cam_forward', -0.079)
        self.declare_parameter('cam_hfov', 1.274)
        self.declare_parameter('cam_width', 640)
        self.declare_parameter('cam_height_px', 480)
        self.declare_parameter('linear_speed', 0.25)
        self.declare_parameter('kp_angular', 0.8)
        self.declare_parameter('lidar_topic', '/scan')
        self.declare_parameter('robot_radius', 0.15)
        self.declare_parameter('obstacle_inflation', 0.30)
        self.declare_parameter('route_file', '')  # path to route YAML
        self.declare_parameter('odom_topic', '/odom')

        seg_topic = self.get_parameter('segmentation_topic').value
        lidar_topic = self.get_parameter('lidar_topic').value
        mf = self.get_parameter('map_forward').value
        mb = self.get_parameter('map_backward').value
        ml = self.get_parameter('map_lateral').value
        cs = self.get_parameter('cell_size').value
        self.cam_h = self.get_parameter('cam_height').value
        self.cam_x = self.get_parameter('cam_forward').value
        hfov = self.get_parameter('cam_hfov').value
        cw = self.get_parameter('cam_width').value
        ch = self.get_parameter('cam_height_px').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.kp = self.get_parameter('kp_angular').value
        obs_inflate = self.get_parameter('obstacle_inflation').value

        # Grid
        self.rows = int((mf + mb) / cs)
        self.cols = int((2 * ml) / cs)
        self.goal_row = int(mf * 0.7 / cs)
        self.robot_cell = (self.rows - 1, self.cols // 2)
        self.map_fwd = mf
        self.map_bwd = mb
        self.map_lat = ml

        # Camera intrinsics
        self.fx = cw / (2.0 * math.tan(hfov / 2.0))
        self.fy = ch / (2.0 * math.tan(hfov * 3.0 / 4.0 / 2.0))
        self.cx = cw / 2.0
        self.cy = ch / 2.0
        self.cam_w = cw
        self.cam_hpx = ch

        # Projection table
        self._build_projection_table()

        # Costmap grid
        self.grid = CostmapGrid(self.rows, self.cols, cs, mf, obs_inflate)

        # LiDAR
        self.latest_lidar_points = []

        # ROS wiring
        self.create_subscription(Image, seg_topic, self.mask_callback, 10)
        self.create_subscription(LaserScan, lidar_topic, self.lidar_callback, 10)
        odom_topic = self.get_parameter('odom_topic').value
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/costmap/debug_image', 10)

        # ── Odometry state ──
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.start_world_x = self.start_world_y = self.start_world_yaw = 0.0

        # ── Route loading ──
        route_file = self.get_parameter('route_file').value
        route = self._load_route(route_file)

        # ── Blackboard ──
        # Shared state read/written by BT nodes + callbacks
        self.bb = {
            'node': self,
            'grid': self.grid,
            'route': route,
            'goal_row': self.goal_row,
            'robot_cell': self.robot_cell,
            'rows': self.rows,
            'cols': self.cols,
            'cell_size': float(cs),
            'kp': self.kp,
            'linear_speed': self.linear_speed,
            'goal': None,
            'path': None,
            'bias_col': None,
            'odom_x': 0.0,
            'odom_y': 0.0,
            'odom_yaw': 0.0,
        }

        # ── Behavior Tree ──
        # Root: Fallback — try behaviors in priority order
        #   1. RouteNavigate (Sequence): active route → check arrival → bias → plan → steer
        #   2. FreeNavigate (Sequence): no route → plan → steer (original behavior)
        #   3. Stop (Action): last resort

        # Common steering subtree
        steer_subtree = Sequence([
            Action(act_find_goal, 'FindGoal'),
            Action(act_plan_path, 'PlanPath'),
            Action(act_follow_path, 'FollowPath'),
        ], name='Steer')

        self.bt_root = Fallback([
            # Route-guided navigation (waypoints active)
            Sequence([
                Condition(cond_route_active, 'RouteActive?'),
                Action(act_check_arrival, 'CheckArrival'),
                Action(act_compute_waypoint_bias, 'ComputeBias'),
                steer_subtree,
            ], name='RouteNavigate'),

            # Free navigation (no route loaded)
            Sequence([
                Condition(cond_route_inactive, 'NoRoute?'),
                steer_subtree,
            ], name='FreeNavigate'),

            # Route finished — log and stop
            Sequence([
                Condition(cond_route_finished, 'RouteFinished?'),
                Action(act_log_mission_complete, 'LogComplete'),
                Action(act_stop, 'Stop'),
            ], name='MissionComplete'),

            # Safety fallback
            Action(act_stop, 'Stop'),
        ], name='Root')

        route_str = route.progress_str() if route else 'none (free mode)'
        self.get_logger().info(
            f'BT Navigator started. Grid: {self.rows}x{self.cols}. '
            f'Route: {route_str}. '
            f'Tree: {len(self.bt_root.children)} branches.'
        )

    # ── Route loading ──────────────────────────────────────────

    def _load_route(self, route_file):
        """
        Load route YAML and SDF waypoints. Returns RouteManager or None.
        Route file format:
          start: sp_1
          waypoints:
            - sp_1
            - wp_crossway_1
            - __stop__
        """
        if not route_file:
            return None

        if not os.path.isfile(route_file):
            self.get_logger().warn(f'Route file not found: {route_file}')
            return None

        # Read route file
        with open(route_file) as f:
            data = yaml.safe_load(f)

        if not data or 'waypoints' not in data:
            self.get_logger().warn(f'No waypoints in route file: {route_file}')
            return None

        waypoint_names = data['waypoints']
        if not waypoint_names:
            self.get_logger().warn('Empty waypoint list in route file.')
            return None

        # Parse SDF for all waypoint coordinates
        try:
            pkg_bringup = get_package_share_directory('hambot_bringup')
            sdf_path = os.path.join(pkg_bringup, 'worlds', 'campus_map2.sdf')
        except Exception:
            self.get_logger().warn('Could not find SDF world file.')
            return None

        waypoint_poses = self._parse_sdf_waypoints(sdf_path)

        if not waypoint_poses:
            self.get_logger().warn(f'No waypoints found in SDF: {sdf_path}')
            return None

        # Log route summary
        num_wp = sum(1 for n in waypoint_names if n != '__stop__')
        start_name = data.get('start', waypoint_names[0] if waypoint_names else '?')
        self.get_logger().info(f'Route loaded: {num_wp} waypoints, start={start_name}')

        names_str = ' \u2192 '.join(waypoint_names)
        self.get_logger().info(f'Route waypoints: {names_str}')

        # Store waypoints in world frame directly
        route_poses = {name: (x, y) for name, (x, y, _) in waypoint_poses.items()}

        route = RouteManager(waypoint_names, route_poses, threshold=1.5)

        # Store start pose for odometry→world transform
        start_name = data.get('start', waypoint_names[0] if waypoint_names else '')
        start_pose = waypoint_poses.get(start_name)
        if start_pose:
            self.start_world_x = start_pose[0]
            self.start_world_y = start_pose[1]
            self.start_world_yaw = start_pose[2]
            self.get_logger().info(
                f'Starting at {start_name} (x={self.start_world_x:.1f}, '
                f'y={self.start_world_y:.1f}, yaw={self.start_world_yaw:.2f})'
            )
        else:
            self.start_world_x = self.start_world_y = self.start_world_yaw = 0.0

        cur = route.current()
        if cur:
            pose = route.current_pose()
            if pose:
                self.get_logger().info(
                    f'NEXT WP: {cur} at ({pose[0]:.1f}, {pose[1]:.1f}) (world frame)'
                )

        return route

    # ── SDF parser ─────────────────────────────────────────────

    def _parse_sdf_waypoints(self, sdf_path):
        """
        Read <frame> elements from SDF world file.
        Returns dict: name -> (x, y)
        """
        if not os.path.isfile(sdf_path):
            self.get_logger().warn(f'SDF not found: {sdf_path}')
            return {}

        import re

        with open(sdf_path) as f:
            content = f.read()

        pattern = (
            r'<frame\s+name="([^"]+)"[^>]*>\s*'
            r'<pose>([\d.-]+)\s+([\d.-]+)\s+[\d.-]+\s+[\d.-]+\s+[\d.-]+\s+([\d.-]+)</pose>'
        )
        matches = re.findall(pattern, content)

        poses = {}
        for label, x_str, y_str, yaw_str in matches:
            poses[label] = (float(x_str), float(y_str), float(yaw_str))

        if not poses:
            self.get_logger().warn('No <frame> elements found in SDF.')

        return poses

    # ── Coordinate transform ──────────────────────────────────

    def _odom_to_world(self, ox, oy, oyaw):
        """
        Convert odometry-frame pose to world frame using start pose.
        Odometry starts at (0, 0, 0) at the robot's spawn point.
        """
        wx = self.start_world_x + ox * math.cos(self.start_world_yaw) - oy * math.sin(self.start_world_yaw)
        wy = self.start_world_y + ox * math.sin(self.start_world_yaw) + oy * math.cos(self.start_world_yaw)
        wyaw = self.start_world_yaw + oyaw
        wyaw = math.atan2(math.sin(wyaw), math.cos(wyaw))  # wrap to [-pi, pi]
        return wx, wy, wyaw

    # ── Odometry callback ──────────────────────────────────────

    def odom_callback(self, msg):
        """Update robot world position from odometry."""
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        oyaw = euler_from_quaternion(msg.pose.pose.orientation)

        # Transform odometry to world frame
        self.odom_x, self.odom_y, self.odom_yaw = self._odom_to_world(ox, oy, oyaw)

        self.bb['odom_x'] = self.odom_x
        self.bb['odom_y'] = self.odom_y
        self.bb['odom_yaw'] = self.odom_yaw

    # ── Projection table ────────────────────────────────────────

    def _build_projection_table(self):
        H, W = self.cam_hpx, self.cam_w
        u, v = np.meshgrid(np.arange(W), np.arange(H))
        dx = (u.astype(np.float32) - self.cx) / self.fx
        dy = (v.astype(np.float32) - self.cy) / self.fy
        dz = np.ones_like(dx)
        norm = np.sqrt(dx * dx + dy * dy + dz * dz)
        dx /= norm
        dy /= norm
        dz /= norm
        rx, ry, rz = dz, -dx, -dy
        down = rz < 0
        t = -self.cam_h / rz
        gx = self.cam_x + t * rx
        gy = t * ry
        ml = self.map_lat
        mf = self.map_fwd
        mb = self.map_bwd
        valid = down & (gx >= -mb) & (gx <= mf) & (gy >= -ml) & (gy <= ml)
        self.px_valid = valid.copy()
        self.px_row = np.full((H, W), -1, dtype=np.int16)
        self.px_col = np.full((H, W), -1, dtype=np.int16)
        cs = self.get_parameter('cell_size').value
        self.px_row[valid] = (self.rows - 1 - (gx[valid] / cs).astype(np.int16))
        self.px_col[valid] = ((gy[valid] + ml) / cs).astype(np.int16)
        for arr in [self.px_row, self.px_col]:
            arr[arr < 0] = -1
        self.px_row[self.px_row >= self.rows] = -1
        self.px_col[self.px_col >= self.cols] = -1
        self.px_valid = (self.px_row >= 0) & (self.px_col >= 0)

        # Seen column ranges per row for unknown fill
        self.seen_cols_per_row = {}
        if np.any(self.px_valid):
            seen_rows = self.px_row[self.px_valid]
            seen_cols = self.px_col[self.px_valid]
            for r, c in zip(seen_rows, seen_cols):
                if r not in self.seen_cols_per_row:
                    self.seen_cols_per_row[r] = [c, c]
                else:
                    if c < self.seen_cols_per_row[r][0]:
                        self.seen_cols_per_row[r][0] = c
                    if c > self.seen_cols_per_row[r][1]:
                        self.seen_cols_per_row[r][1] = c
        self.seen_cols_per_row = {r: tuple(v) for r, v in self.seen_cols_per_row.items()}

    # ── Callbacks ───────────────────────────────────────────────

    def mask_callback(self, msg):
        """
        Every camera frame:
          1. Build costmap (always runs)
          2. Run BT to decide what to do
        """
        try:
            # ── 1. Decode ──
            if msg.encoding == 'mono8':
                mask = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            elif msg.encoding in ('rgb8', 'bgr8'):
                raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                mask = raw[:, :, 0]
                if msg.encoding == 'bgr8':
                    mask = raw[:, :, 2]
            else:
                self.get_logger().warn(f'Unknown encoding: {msg.encoding}')
                return

            # ── 2. Build costmap (pre-processing, always runs) ──
            self.grid.fill_from_camera(mask, self.px_valid, self.px_row,
                                       self.px_col, self.robot_cell)
            grass_mask = (self.grid.costmap == 255)
            self.grid.inflate_edges(grass_mask)
            self.grid.apply_lidar_gradient(self.latest_lidar_points, self.robot_cell)
            self.grid.fill_unknown_forward(self.seen_cols_per_row)
            self.grid.costmap[self.robot_cell] = 0

            # ── 3. Clear previous results in blackboard ──
            # BT will re-compute goal/path each tick based on fresh costmap
            self.bb['goal'] = None
            self.bb['path'] = None

            # ── 4. Run Behavior Tree ──
            status = self.bt_root.tick(self.bb)

            # ── 5. Debug visualization ──
            goal = self.bb.get('goal')
            path = self.bb.get('path')
            self._publish_debug(path, goal)

            if status == BT.FAILURE:
                self.get_logger().warn(
                    f'BT returned FAILURE — all behaviors exhausted.',
                    throttle_duration_sec=2.0
                )
                self._publish_stop()

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def lidar_callback(self, msg):
        pts = []
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                if x >= 0 and x <= self.map_fwd and abs(y) <= self.map_lat:
                    pts.append((x, y))
            angle += msg.angle_increment
        self.latest_lidar_points = pts

    # ── Helpers ─────────────────────────────────────────────────

    def _publish_stop(self):
        self.vel_pub.publish(Twist())

    def _publish_debug(self, path, goal):
        """Same debug viz as before — unchanged."""
        H, W = self.rows, self.cols
        cm = self.grid.costmap
        debug = np.zeros((H, W, 3), dtype=np.uint8)

        debug[cm == 254] = (30, 0, 30)
        debug[cm == 255] = (255, 0, 0)
        edge = (cm > 0) & (cm < 100)
        if np.any(edge):
            frac = cm[edge].astype(np.float32) / 100.0
            debug[edge, 0] = (frac * 255).astype(np.uint8)
            debug[edge, 1] = ((1.0 - frac) * 255).astype(np.uint8)
            debug[edge, 2] = 0
        debug[cm == 0] = (0, 200, 0)

        if path:
            for r, c in path:
                debug[r, c] = (255, 255, 0)

        if goal is not None:
            gr, gc = goal
            if 0 <= gr < H and 0 <= gc < W:
                debug[max(0, gr - 1):min(H, gr + 2), max(0, gc - 1):min(W, gc + 2)] = (0, 255, 255)

        rr, rc = self.robot_cell
        debug[max(0, rr - 1):min(H, rr + 2), max(0, rc - 1):min(W, rc + 2)] = (0, 0, 255)
        debug = debug[:, ::-1, :]
        scale = 6
        SH, SW = H * scale, W * scale
        big = np.zeros((SH, SW, 3), dtype=np.uint8)
        for i in range(3):
            big[:, :, i] = np.repeat(np.repeat(debug[:, :, i], scale, axis=0), scale, axis=1)

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = SH
        msg.width = SW
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = SW * 3
        msg.data = big.tobytes()
        self.debug_pub.publish(msg)


# ════════════════════════════════════════════════════════════════
# ENTRY POINT
# ════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = CostmapNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
