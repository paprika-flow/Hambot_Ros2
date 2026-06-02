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
import numpy as np
import math
import heapq


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

    def find_goal(self, goal_row):
        lo = min(self.rows - 2, goal_row + 5)
        hi = max(1, goal_row - 5)
        for r in range(lo, hi - 1, -1):
            sw = np.where(self.costmap[r, :] < 200)[0]
            if len(sw) > 0:
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

def cond_goal_found(bb):
    return bb.get('goal') is not None


def cond_path_found(bb):
    p = bb.get('path')
    return p is not None and len(p) >= 2


def act_find_goal(bb):
    """Run goal search on the built costmap. Stores result in bb."""
    grid = bb['grid']
    goal = grid.find_goal(bb['goal_row'])
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
    twist.linear.x = speed
    twist.angular.z = final
    node.vel_pub.publish(twist)
    return BT.SUCCESS


def act_stop(bb):
    """Publish zero cmd_vel. Used as last-resort fallback."""
    node = bb['node']
    node.vel_pub.publish(Twist())
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
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/costmap/debug_image', 10)

        # ── Blackboard ──
        # Shared state read/written by BT nodes + callbacks
        self.bb = {
            'node': self,
            'grid': self.grid,
            'goal_row': self.goal_row,
            'robot_cell': self.robot_cell,
            'rows': self.rows,
            'cols': self.cols,
            'kp': self.kp,
            'linear_speed': self.linear_speed,
            'goal': None,
            'path': None,
        }

        # ── Behavior Tree ──
        # Root: Fallback — try behaviors in priority order
        #   1. Navigate (Sequence): find goal → plan path → follow path
        #   2. Stop (Action): last resort, publish zero
        self.bt_root = Fallback([
            Sequence([
                Action(act_find_goal, 'FindGoal'),
                Action(act_plan_path, 'PlanPath'),
                Action(act_follow_path, 'FollowPath'),
            ], name='Navigate'),
            Action(act_stop, 'Stop'),
        ], name='Root')

        self.get_logger().info(
            f'BT Navigator started. Grid: {self.rows}x{self.cols}. '
            f'Tree: {len(self.bt_root.children)} branches.'
        )

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
