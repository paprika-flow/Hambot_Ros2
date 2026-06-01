#!/usr/bin/env python3
"""
costmap_navigator.py — Local costmap planner for sidewalk navigation.

Pipeline:
  1. Receive binary sidewalk mask from segmenter
  2. Back-project each pixel to ground plane using camera model
  3. Fill 2D costmap grid (robot-centered, base_link frame)
  4. Find best forward goal point
  5. Run A* from robot to goal
  6. Extract steering angle from path → cmd_vel
  7. Publish debug visualization

Frame conventions:
  - Camera optical: Z=forward, X=right, Y=down
  - base_link:      X=forward, Y=left, Z=up
  - Costmap row 0 = farthest forward, rows increase toward robot
  - Costmap col 0 = leftmost, cols increase toward right
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point
import numpy as np
import math
import heapq


class CostmapNavigator(Node):
    def __init__(self):
        super().__init__('costmap_navigator')

        # ── Parameters ──────────────────────────────────────────────
        # use_sim_time is auto-declared by ROS 2 Humble — do not re-declare
        self.declare_parameter('segmentation_topic', '/camera/sidewalk_mask')
        self.declare_parameter('map_forward', 3.0)      # m ahead of robot
        self.declare_parameter('map_backward', 0.5)     # m behind robot
        self.declare_parameter('map_lateral', 1.5)      # m each side
        self.declare_parameter('cell_size', 0.05)       # 5 cm per cell

        # Camera geometry
        self.declare_parameter('cam_height', 0.341)     # Z above ground
        self.declare_parameter('cam_forward', -0.079)   # X in base_link
        self.declare_parameter('cam_hfov', 1.274)       # horizontal FOV (rad)
        self.declare_parameter('cam_width', 640)
        self.declare_parameter('cam_height_px', 480)

        # Control gains
        self.declare_parameter('linear_speed', 0.25)    # m/s
        self.declare_parameter('kp_angular', 0.8)       # steering P gain

        seg_topic = self.get_parameter('segmentation_topic').value
        mf = self.get_parameter('map_forward').value
        mb = self.get_parameter('map_backward').value
        ml = self.get_parameter('map_lateral').value
        self.cell_size = self.get_parameter('cell_size').value
        self.cam_h = self.get_parameter('cam_height').value
        self.cam_x = self.get_parameter('cam_forward').value
        self.cam_hfov = self.get_parameter('cam_hfov').value
        self.cam_w = self.get_parameter('cam_width').value
        self.cam_hpx = self.get_parameter('cam_height_px').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.kp = self.get_parameter('kp_angular').value

        # Costmap dimensions (cells)
        self.map_fwd = mf
        self.map_bwd = mb
        self.map_lat = ml
        self.rows = int((mf + mb) / self.cell_size)     # total rows
        self.cols = int((2 * ml) / self.cell_size)      # total cols
        self.goal_row = int(mf * 0.7 / self.cell_size)  # default goal ~2.1m ahead

        # Camera intrinsics
        self.fx = self.cam_w / (2.0 * math.tan(self.cam_hfov / 2.0))
        self.fy = self.cam_hpx / (2.0 * math.tan(self.cam_hfov * 3.0/4.0 / 2.0))
        self.cx = self.cam_w / 2.0
        self.cy = self.cam_hpx / 2.0

        # Precompute pixel-to-cell lookup table
        self._build_projection_table()

        # Costmap: 0=free, 254=unknown, 255=lethal
        self.costmap = np.full((self.rows, self.cols), 254, dtype=np.uint8)

        # ── ROS 2 plumbing ──────────────────────────────────────
        self.mask_sub = self.create_subscription(
            Image, seg_topic, self.mask_callback, 10
        )
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(
            Image, '/costmap/debug_image', 10
        )

        self.get_logger().info(
            f'CostmapNavigator started. Map: {self.rows}x{self.cols} '
            f'({(mf+mb):.1f}m x {2*ml:.1f}m, {self.cell_size:.2f}m/cell). '
            f'Sub: {seg_topic}'
        )

    # ────────────────────────────────────────────────────────────
    # PROJECTION TABLE
    # ────────────────────────────────────────────────────────────

    def _build_projection_table(self):
        """
        Precompute for each pixel (u,v) the costmap cell (r,c) it maps to,
        and whether the projection is valid (hits ground within bounds).

        Stored as arrays of shape (H, W):
          self.px_valid[u,v] = bool
          self.px_row[u,v]   = costmap row index
          self.px_col[u,v]   = costmap col index
        """
        H, W = self.cam_hpx, self.cam_w
        u_map, v_map = np.meshgrid(np.arange(W), np.arange(H))

        # Ray direction in camera optical frame (Z=forward, X=right, Y=down)
        dx = (u_map.astype(np.float32) - self.cx) / self.fx
        dy = (v_map.astype(np.float32) - self.cy) / self.fy
        dz = np.ones_like(dx)

        norm = np.sqrt(dx*dx + dy*dy + dz*dz)
        dx /= norm
        dy /= norm
        dz /= norm

        # Transform to base_link: X=forward=dz, Y=left=-dx, Z=up=-dy
        rx = dz        # forward
        ry = -dx       # left
        rz = -dy       # up

        # Ground intersection at z=0. Camera at (cam_x, 0, cam_h) in base_link
        mask_down = rz < 0  # ray points down — hits ground
        t = -self.cam_h / rz
        gx = self.cam_x + t * rx
        gy = t * ry

        # Check bounds
        in_bounds = (
            mask_down &
            (gx >= -self.map_bwd) &
            (gx <= self.map_fwd) &
            (gy >= -self.map_lat) &
            (gy <= self.map_lat)
        )

        # Map to costmap indices
        # Row: robot at bottom (row=rows-1), forward at top (row=0)
        # gx = 0 → row = rows-1 (robot position row)
        # gx = map_fwd → row = 0
        self.px_valid = in_bounds
        self.px_row = np.full((H, W), -1, dtype=np.int16)
        self.px_col = np.full((H, W), -1, dtype=np.int16)

        self.px_row[in_bounds] = (
            self.rows - 1 - (gx[in_bounds] / self.cell_size).astype(np.int16)
        )
        self.px_col[in_bounds] = (
            (gy[in_bounds] + self.map_lat) / self.cell_size
        ).astype(np.int16)

        # Clamp to grid bounds (should already be in bounds, but protect)
        self.px_row[self.px_row < 0] = -1
        self.px_row[self.px_row >= self.rows] = -1
        self.px_col[self.px_col < 0] = -1
        self.px_col[self.px_col >= self.cols] = -1

        r_valid = self.px_row >= 0
        c_valid = self.px_col >= 0
        self.px_valid = r_valid & c_valid

    # ────────────────────────────────────────────────────────────
    # CALLBACK
    # ────────────────────────────────────────────────────────────

    def mask_callback(self, msg: Image):
        try:
            # 1. Decode mask (mono8 or rgb8)
            if msg.encoding == 'mono8':
                mask = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width
                )
            elif msg.encoding in ('rgb8', 'bgr8'):
                raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3
                )
                mask = raw[:, :, 0]  # use red channel
                if msg.encoding == 'bgr8':
                    mask = raw[:, :, 2]
            else:
                self.get_logger().warn(f'Unknown encoding: {msg.encoding}')
                return

            # 2. Fill costmap from projection table
            self.costmap.fill(254)  # default: unknown

            # Valid pixels are those with ground projection in bounds
            valid = self.px_valid
            rows_v = self.px_row[valid]
            cols_v = self.px_col[valid]
            mask_v = mask[valid]

            # Sidewalk (255 in binary mask) → cost 0
            sw = mask_v > 127
            self.costmap[rows_v[sw], cols_v[sw]] = 0

            # Non-sidewalk visible pixels → cost 255 (lethal obstacle)
            nosw = mask_v <= 127
            self.costmap[rows_v[nosw], cols_v[nosw]] = 255

            # Mark robot's own cell as free — robot stands on sidewalk
            self.costmap[self.rows - 1, self.cols // 2] = 0

            # ── DIAGNOSTICS ──
            n_valid = np.count_nonzero(self.px_valid)
            n_sw = np.sum(mask_v > 127)
            n_grass = np.sum(mask_v <= 127)
            n_cost0 = np.count_nonzero(self.costmap == 0)
            n_cost255 = np.count_nonzero(self.costmap == 255)
            n_cost254 = np.count_nonzero(self.costmap == 254)
            self.get_logger().info(
                f'DIAG: valid_pixels={n_valid} sw_pix={n_sw} grass_pix={n_grass} | '
                f'costmap: 0={n_cost0} 255={n_cost255} 254={n_cost254} | '
                f'shape={self.costmap.shape}',
                throttle_duration_sec=3.0
            )
            # ─────────────────

            # 3. Edge inflation: smooth gradient near sidewalk edges
            self._inflate_edges()

            # 4. Find best forward goal
            goal = self._find_goal()
            if goal is None:
                self.get_logger().warn('No valid goal found — zero sidewalk cells in search band', throttle_duration_sec=2.0)
                # Still publish debug with no goal
                self._publish_debug(None, None)
                self._publish_stop()
                return

            self.get_logger().info(f'Goal found: row={goal[0]} col={goal[1]} cost={self.costmap[goal[0], goal[1]]}', throttle_duration_sec=1.0)

            # 5. Run A*
            start = (self.rows - 1, self.cols // 2)  # robot position
            path = self._astar(start, goal)
            if path is None or len(path) < 2:
                self.get_logger().warn(f'A* failed: path={type(path).__name__} len={len(path) if path else 0} goal=({goal[0]},{goal[1]})', throttle_duration_sec=1.0)
                self._publish_debug(None, goal)
                self._publish_stop()
                return

            # 6. Steering from path
            twist = self._steer_from_path(path)
            self.get_logger().info(
                f'A* path found: {len(path)} cells | '
                f'cmd_vel: linear={twist.linear.x:.2f} angular={twist.angular.z:.2f}',
                throttle_duration_sec=1.0
            )

            # 7. Debug visualization
            self._publish_debug(path, goal)

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    # ────────────────────────────────────────────────────────────
    # EDGE INFLATION
    # ────────────────────────────────────────────────────────────

    def _inflate_edges(self):
        """
        Apply gradient cost near sidewalk edges.
        Sidewalk cells near grass get high cost.
        Sidewalk interior (far from grass) stays cost 0.
        Uses distance transform seeded from non-sidewalk cells.
        """
        rows, cols = self.rows, self.cols
        not_sw = self.costmap != 0  # grass + obstacle
        if np.all(not_sw):
            return

        # Seed: non-sidewalk cells = distance 0
        dist = np.full((rows, cols), 255, dtype=np.uint16)
        dist[not_sw] = 0

        # Propagate: each cell takes min(neighbors+1).
        # This gives each sidewalk cell its distance to nearest grass.
        for _ in range(40):  # 40 cells = 2.0m max distance
            prev = dist.copy()
            dist[1:, :] = np.minimum(dist[1:, :], (prev[:-1, :] + 1).astype(np.uint16))
            dist[:-1, :] = np.minimum(dist[:-1, :], (prev[1:, :] + 1).astype(np.uint16))
            dist[:, 1:] = np.minimum(dist[:, 1:], (prev[:, :-1] + 1).astype(np.uint16))
            dist[:, :-1] = np.minimum(dist[:, :-1], (prev[:, 1:] + 1).astype(np.uint16))
            if np.array_equal(dist, prev):
                break

        # Cost: sidewalk cells within 40cm of grass get high cost
        # dist=0  → grass itself (already 255, stays 255)
        # dist=1  → cell adjacent to grass (5cm) → cost 255
        # dist=4  → 20cm from grass → cost ~127
        # dist=8  → 40cm from grass → cost 0
        # dist>8  → interior → cost 0
        max_edge_dist = 8  # cells = 40 cm
        sw = self.costmap == 0
        edge_cost = np.zeros((rows, cols), dtype=np.uint8)
        edge_cost[sw] = np.clip(
            255 * (1.0 - dist[sw].astype(np.float32) / max_edge_dist),
            0, 255
        ).astype(np.uint8)

        self.costmap = np.maximum(self.costmap, edge_cost)

    # ────────────────────────────────────────────────────────────
    # GOAL SEARCH
    # ────────────────────────────────────────────────────────────

    def _find_goal(self):
        """
        Scan forward rows for the first row with sidewalk cells.
        Return cell (row, col) closest to center of that row.
        Start from farthest visible row inward.
        """
        goal_r = self.goal_row
        # Search a band: rows around default goal
        search_start = min(self.rows - 2, goal_r + 5)
        search_end = max(1, goal_r - 5)

        for r in range(search_start, search_end - 1, -1):
            sw_cols = np.where(self.costmap[r, :] == 0)[0]
            if len(sw_cols) > 0:
                center = self.cols // 2
                best_c = sw_cols[np.argmin(np.abs(sw_cols - center))]
                return (r, best_c)

        return None

    # ────────────────────────────────────────────────────────────
    # A* PATH PLANNER
    # ────────────────────────────────────────────────────────────

    def _astar(self, start, goal):
        """
        Standard A* on 4-connected grid with diagonal heuristic.
        costmap values: 0=free, 1-253=some cost, 254=unknown, 255=blocked.
        Returns list of (row, col) tuples from start to goal, or None.
        """
        rows, cols = self.rows, self.cols
        costmap = self.costmap

        # Heuristic: diagonal distance
        def heuristic(a, b):
            dr = abs(a[0] - b[0])
            dc = abs(a[1] - b[1])
            return math.sqrt(dr * dr + dc * dc)

        # Fast check: goal valid?
        gr, gc = goal
        if gr < 0 or gr >= rows or gc < 0 or gc >= cols:
            return None
        if costmap[gr, gc] >= 254:  # unknown or lethal
            # Try to find nearby valid goal
            return None

        # A*
        open_set = []
        heapq.heappush(open_set, (0.0, start))
        came_from = {}
        g_score = {start: 0.0}
        f_score = {start: heuristic(start, goal)}

        max_iter = self.rows * self.cols * 2  # safety limit
        iter_count = 0

        while open_set and iter_count < max_iter:
            iter_count += 1
            _, current = heapq.heappop(open_set)

            if current == goal:
                break

            cr, cc = current
            neighbors = [
                (cr - 1, cc), (cr + 1, cc),
                (cr, cc - 1), (cr, cc + 1),
            ]

            for nr, nc in neighbors:
                if nr < 0 or nr >= rows or nc < 0 or nc >= cols:
                    continue
                cell_cost = costmap[nr, nc]
                if cell_cost >= 255:  # only confirmed lethal is blocked
                    continue

                # Unknown (254) = high cost, free (0) = low cost
                move_cost = 1.0 + (cell_cost / 255.0) * 10.0
                tent_g = g_score[current] + move_cost

                if (nr, nc) not in g_score or tent_g < g_score[(nr, nc)]:
                    came_from[(nr, nc)] = current
                    g_score[(nr, nc)] = tent_g
                    f = tent_g + heuristic((nr, nc), goal)
                    f_score[(nr, nc)] = f
                    heapq.heappush(open_set, (f, (nr, nc)))

        # Reconstruct
        if goal not in came_from and current != goal:
            return None

        path = []
        node = goal if goal in came_from or goal == start else current
        while node in came_from:
            path.append(node)
            node = came_from[node]
        path.append(start)
        path.reverse()
        return path

    # ────────────────────────────────────────────────────────────
    # STEERING
    # ────────────────────────────────────────────────────────────

    def _steer_from_path(self, path):
        """
        Compute steering angle from A* path.
        Uses a far look-ahead (~1m) so robot sees curves and intersection forks.
        """
        twist = Twist()

        if len(path) < 2:
            twist.linear.x = 0.0
            self.vel_pub.publish(twist)
            return twist

        # ── Look 1 meter ahead along path ──
        look_ahead_cells = int(1.0 / self.cell_size)  # 20 cells = 1m
        look_ahead = min(look_ahead_cells, len(path) - 1)

        start_r, start_c = path[0]
        target_r, target_c = path[look_ahead]

        dr = -(target_r - start_r)  # forward distance (cells)
        dc = target_c - start_c     # lateral offset (cells)

        self.get_logger().info(
            f'STEER: start=({start_r},{start_c}) '
            f'target=({target_r},{target_c}) look_ahead={look_ahead} '
            f'dr={dr} dc={dc}',
            throttle_duration_sec=1.0
        )

        if dr <= 0:
            twist.linear.x = 0.0
            self.vel_pub.publish(twist)
            return twist

        # ── Path angle from A* ──
        path_angle = math.atan2(dc * self.cell_size, dr * self.cell_size)

        # ── Cross-track centering ──
        # Look at row 1m ahead for sidewalk centerline
        ctr_row = max(0, self.rows - 1 - look_ahead_cells)
        cross_angle = 0.0
        row_costs = self.costmap[ctr_row, :]
        # Find cells with cost < 100 (sidewalk interior + edges)
        sw = np.where(row_costs < 100)[0]
        if len(sw) > 4:
            left = sw[0]
            right = sw[-1]
            sidewalk_center = (left + right) / 2.0
            robot_col = self.cols // 2
            cte = sidewalk_center - robot_col
            half_width = max(1, (right - left) / 2.0)
            cte_norm = cte / half_width
            cross_angle = self.kp * cte_norm * 0.4
            self.get_logger().info(
                f'CROSS: row={ctr_row} sw=[{left},{right}] '
                f'center={sidewalk_center:.1f} '
                f'cte_norm={cte_norm:.2f} cross_angle={math.degrees(cross_angle):.1f}°',
                throttle_duration_sec=1.0
            )
        else:
            self.get_logger().info(
                f'CROSS: row={ctr_row} no sidewalk found '
                f'(nonzero costs: {np.count_nonzero(row_costs < 200)}/{self.cols})',
                throttle_duration_sec=1.0
            )

        # ── Blend: path angle + cross-track ──
        blend = 0.4
        final_angle = (1.0 - blend) * path_angle + blend * cross_angle

        self.get_logger().info(
            f'ANGLE: path={math.degrees(path_angle):.1f}° '
            f'cross={math.degrees(cross_angle):.1f}° '
            f'final={math.degrees(final_angle):.1f}°',
            throttle_duration_sec=1.0
        )

        twist.linear.x = self.linear_speed
        twist.angular.z = final_angle
        self.vel_pub.publish(twist)
        return twist

    def _publish_stop(self):
        self.vel_pub.publish(Twist())

    # ────────────────────────────────────────────────────────────
    # DEBUG VISUALIZATION
    # ────────────────────────────────────────────────────────────

    def _publish_debug(self, path, goal):
        """Publish costmap as rgb8 image. Scale up to visible size for Gazebo."""
        H, W = self.rows, self.cols
        debug = np.zeros((H, W, 3), dtype=np.uint8)

        cm = self.costmap
        # Unknown (254) → dark purple
        debug[cm == 254] = (30, 0, 30)
        # Obstacle (255) → bright red
        debug[cm == 255] = (255, 0, 0)
        # Edge gradient (1-99) → orange shades
        edge = (cm > 0) & (cm < 100)
        # RGB: R=255, G=cm_value, B=0                                                                                                                                                                                                           
        debug[edge, 2] = 0        # R                                                                                                                                                                                                         
        debug[edge, 1] = cm[edge]   # G varies 0-99                                                                                                                                                                                             
        debug[edge, 0] = 255          # B
        # Sidewalk (0) → bright green
        debug[cm == 0] = (0, 200, 0)

        # Draw path in cyan
        if path:
            for r, c in path:
                if 0 <= r < H and 0 <= c < W:
                    debug[r, c] = (255, 255, 0)

        # Draw goal as yellow cross
        if goal is not None:
            gr, gc = goal
            if 0 <= gr < H and 0 <= gc < W:
                debug[max(0, gr-1):min(H, gr+2), max(0, gc-1):min(W, gc+2)] = (0, 255, 255)

        # Draw robot position in blue
        rr, rc = self.rows - 1, self.cols // 2
        debug[max(0, rr-1):min(H, rr+2), max(0, rc-1):min(W, rc+2)] = (255, 0, 0)

        # Scale up 6x for visibility (nearest-neighbor = just repeat pixels)
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


def main(args=None):
    rclpy.init(args=args)
    node = CostmapNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.vel_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
