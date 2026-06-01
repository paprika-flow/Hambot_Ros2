#!/usr/bin/env python3
"""
costmap_navigator.py — Local costmap planner for sidewalk navigation.

Architecture:
  CostmapGrid       — Pure numpy grid. No ROS. Pipeline logic.
  CostmapNavigator  — ROS 2 node. Thin. Delegates to CostmapGrid.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import math
import heapq


# ════════════════════════════════════════════════════════════════
# COSTMAP GRID  (pure numpy, no ROS)
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

    # ── Camera fill ─────────────────────────────────────────────

    def fill_from_camera(self, mask, px_valid, px_row, px_col, robot_cell):
        """Project segmentation mask into costmap."""
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

    # ── Edge inflation ──────────────────────────────────────────

    def inflate_edges(self, grass_mask):
        """Gradient cost near grass edges. Caps at 100."""
        if not np.any(grass_mask):
            return

        dist = self._distance_transform(grass_mask, 40)
        max_d = 8  # 40 cm
        on_sw = (self.costmap == 0) & ~grass_mask
        edge = np.zeros((self.rows, self.cols), dtype=np.uint8)
        edge[on_sw] = np.clip(
            100 * (1.0 - dist[on_sw].astype(np.float32) / max_d), 0, 100
        ).astype(np.uint8)
        self.costmap = np.maximum(
            self.costmap.astype(np.uint16), edge.astype(np.uint16)
        ).astype(np.uint8)

    # ── LiDAR gradient ──────────────────────────────────────────

    def apply_lidar_gradient(self, points, robot_cell):
        """Gradient cost from LiDAR points. Core=255, fades to 0 over inflate radius."""
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

    # ── Unknown fill ────────────────────────────────────────────

    def fill_unknown_forward(self, seen_cols_per_row):
        """
        Fill unknown cells only within camera's actual visible column range.
        seen_cols_per_row: dict row->(left_col, right_col) of projected pixels.
        Unknown cells outside these columns stay unknown.
        """
        for r in range(self.rows):
            if r not in seen_cols_per_row:
                continue
            left, right = seen_cols_per_row[r]
            row = self.costmap[r, :]
            unk = (row[left:right+1] == 254)
            row[left:right+1][unk] = 0

    # ── Goal search ─────────────────────────────────────────────

    def find_goal(self, goal_row):
        """
        Find most-center traversable cell near goal_row.
        Accepts any cell with cost < 200 (edge gradient fine, lethal blocked).
        Goal shifts off-center if obstacle blocks middle — A* routes around.
        """
        lo = min(self.rows - 2, goal_row + 5)
        hi = max(1, goal_row - 5)
        for r in range(lo, hi - 1, -1):
            sw = np.where(self.costmap[r, :] < 230)[0]
            if len(sw) > 0:
                center = self.cols // 2
                best = sw[np.argmin(np.abs(sw - center))]
                return (r, best)
        return None

    # ── A* ──────────────────────────────────────────────────────

    def astar(self, start, goal):
        """A* on 4-connected grid. Returns path or None."""
        if not (0 <= goal[0] < self.rows and 0 <= goal[1] < self.cols):
            return None
        if self.costmap[goal] >= 255:
            return None

        h = lambda a, b: math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
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
            for nr, nc in [(cr-1,cc),(cr+1,cc),(cr,cc-1),(cr,cc+1)]:
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
            return None  # max_iter hit

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

    # ── Distance transform (shared helper) ──────────────────────

    def _distance_transform(self, seed_mask, max_iter):
        """Multi-pass distance from True cells in seed_mask."""
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
# ROS 2 NODE
# ════════════════════════════════════════════════════════════════

class CostmapNavigator(Node):
    """Thin ROS node. Owns CostmapGrid. Handles I/O only."""

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

        seg_topic   = self.get_parameter('segmentation_topic').value
        lidar_topic = self.get_parameter('lidar_topic').value
        mf = self.get_parameter('map_forward').value
        mb = self.get_parameter('map_backward').value
        ml = self.get_parameter('map_lateral').value
        cs = self.get_parameter('cell_size').value
        self.cam_h = self.get_parameter('cam_height').value
        self.cam_x = self.get_parameter('cam_forward').value
        hfov = self.get_parameter('cam_hfov').value
        cw   = self.get_parameter('cam_width').value
        ch   = self.get_parameter('cam_height_px').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.kp = self.get_parameter('kp_angular').value
        obs_inflate = self.get_parameter('obstacle_inflation').value

        # Grid dimensions
        self.rows = int((mf + mb) / cs)
        self.cols = int((2 * ml) / cs)
        self.goal_row = int(mf * 0.7 / cs)
        self.robot_cell = (self.rows - 1, self.cols // 2)
        self.map_fwd = mf
        self.map_bwd = mb
        self.map_lat = ml

        # Camera intrinsics
        self.fx = cw / (2.0 * math.tan(hfov / 2.0))
        self.fy = ch / (2.0 * math.tan(hfov * 3.0/4.0 / 2.0))
        self.cx = cw / 2.0
        self.cy = ch / 2.0
        self.cam_w = cw
        self.cam_hpx = ch

        # Precompute projection table
        self._build_projection_table()

        # Costmap grid
        self.grid = CostmapGrid(self.rows, self.cols, cs, mf, obs_inflate)

        # LiDAR state
        self.latest_lidar_points = []

        # ROS 2
        self.create_subscription(Image, seg_topic, self.mask_callback, 10)
        self.create_subscription(LaserScan, lidar_topic, self.lidar_callback, 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/costmap/debug_image', 10)

        self.get_logger().info(f'Started. Grid: {self.rows}x{self.cols}, sub: {seg_topic}')

    # ── Projection table ────────────────────────────────────────

    def _build_projection_table(self):
        """Precompute pixel→costmap cell mapping (runs once at startup)."""
        H, W = self.cam_hpx, self.cam_w
        u, v = np.meshgrid(np.arange(W), np.arange(H))
        dx = (u.astype(np.float32) - self.cx) / self.fx
        dy = (v.astype(np.float32) - self.cy) / self.fy
        dz = np.ones_like(dx)
        norm = np.sqrt(dx*dx + dy*dy + dz*dz)
        dx /= norm; dy /= norm; dz /= norm

        # Camera optical → base_link
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

        # Clamp
        for arr in [self.px_row, self.px_col]:
            arr[arr < 0] = -1
        self.px_row[self.px_row >= self.rows] = -1
        self.px_col[self.px_col >= self.cols] = -1
        self.px_valid = (self.px_row >= 0) & (self.px_col >= 0)

        # Build column range per row for unknown fill
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
        # Convert to tuples
        self.seen_cols_per_row = {r: tuple(v) for r, v in self.seen_cols_per_row.items()}

    # ── Callbacks ───────────────────────────────────────────────

    def mask_callback(self, msg):
        try:
            # 1. Decode
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

            # 2. Pipeline
            self.grid.fill_from_camera(mask, self.px_valid, self.px_row, self.px_col, self.robot_cell)
            grass_mask = (self.grid.costmap == 255)
            self.grid.inflate_edges(grass_mask)
            self.grid.apply_lidar_gradient(self.latest_lidar_points, self.robot_cell)
            self.grid.fill_unknown_forward(self.seen_cols_per_row)
            self.grid.costmap[self.robot_cell] = 0

            # 3. Goal
            goal = self.grid.find_goal(self.goal_row)
            if goal is None:
                self.get_logger().warn('No goal found', throttle_duration_sec=2.0)
                self._publish_debug(None, None)
                self._publish_stop()
                return

            # 4. Plan
            path = self.grid.astar(self.robot_cell, goal)
            if path is None or len(path) < 2:
                self.get_logger().warn('A* failed', throttle_duration_sec=1.0)
                self._publish_debug(None, goal)
                self._publish_stop()
                return

            # 5. Steer
            twist = self._steer_from_path(path)
            self.get_logger().info(
                f'Path {len(path)} cells | cmd: lin={twist.linear.x:.2f} ang={twist.angular.z:.2f}',
                throttle_duration_sec=1.0
            )
            self._publish_debug(path, goal)

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def lidar_callback(self, msg):
        """Store LiDAR points in base_link coords."""
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

    # ── Steering ────────────────────────────────────────────────

    def _steer_from_path(self, path):
        twist = Twist()
        if len(path) < 2:
            twist.linear.x = 0.0
            self.vel_pub.publish(twist)
            return twist

        la_cells = int(1.0 / self.grid.cell_size)
        la = min(la_cells, len(path) - 1)
        sr, sc = path[0]
        tr, tc = path[la]
        dr = -(tr - sr)
        dc = tc - sc

        if dr <= 0:
            twist.linear.x = 0.0
            self.vel_pub.publish(twist)
            return twist

        path_angle = math.atan2(dc * self.grid.cell_size, dr * self.grid.cell_size)

        # Cross-track
        ctr_row = max(0, self.rows - 1 - la_cells)
        cross_angle = 0.0
        row_costs = self.grid.costmap[ctr_row, :]
        sw = np.where(row_costs < 100)[0]
        if len(sw) > 4:
            center = (sw[0] + sw[-1]) / 2.0
            cte = center - self.cols // 2
            half = max(1, (sw[-1] - sw[0]) / 2.0)
            cross_angle = self.kp * (cte / half) * 0.4

        final = 0.6 * path_angle + 0.4 * cross_angle
        twist.linear.x = self.linear_speed
        twist.angular.z = final
        self.vel_pub.publish(twist)
        return twist

    def _publish_stop(self):
        self.vel_pub.publish(Twist())

    # ── Debug visualization ─────────────────────────────────────

    def _publish_debug(self, path, goal):
        H, W = self.rows, self.cols
        cm = self.grid.costmap
        debug = np.zeros((H, W, 3), dtype=np.uint8)

        debug[cm == 254] = (30, 0, 30)      # unknown → purple
        debug[cm == 255] = (255, 0, 0)      # lethal → RED (rgb8: R=255)
        # Edge gradient: green(0) → yellow(50) → red(100)
        edge = (cm > 0) & (cm < 100)
        if np.any(edge):
            frac = cm[edge].astype(np.float32) / 100.0
            # Channel 0=R, 1=G, 2=B
            debug[edge, 0] = (frac * 255).astype(np.uint8)         # R 0→255
            debug[edge, 1] = ((1.0 - frac) * 255).astype(np.uint8) # G 255→0
            debug[edge, 2] = 0                                      # B=0
        debug[cm == 0] = (0, 200, 0)        # free → green

        if path:
            for r, c in path:
                debug[r, c] = (255, 255, 0)  # path → cyan

        if goal is not None:
            gr, gc = goal
            if 0 <= gr < H and 0 <= gc < W:
                debug[max(0,gr-1):min(H,gr+2), max(0,gc-1):min(W,gc+2)] = (0, 255, 255)

        # Robot = BLUE
        rr, rc = self.robot_cell
        debug[max(0,rr-1):min(H,rr+2), max(0,rc-1):min(W,rc+2)] = (0, 0, 255)

        debug = debug[:, ::-1, :]   # flip to match left=robot-left

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
