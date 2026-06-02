#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseArray, Pose
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
import math
import bisect
from collections import defaultdict
from scipy.spatial import Voronoi
from shapely.geometry import Polygon, LineString


# =====================================================================
# INTEGRATED GEOMETRIC MATH & OPTIMIZED WINDING TEST LOGIC
# =====================================================================

def get_midpoint(p1, p2):
    return [(p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2]


def get_distance(p1, p2):
    return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)


def get_angle(x1, y1, x2, y2):
    return math.atan2(y2 - y1, x2 - x1) * 180 / math.pi


def is_point_in_polygon(pt, poly_pts):
    """Checks if a point (x, y) is inside a closed boundary using ray-casting."""
    x, y = pt
    inside = False
    n = len(poly_pts)
    if n == 0:
        return False
    p1x, p1y = poly_pts[0]
    for i in range(1, n + 1):
        p2x, p2y = poly_pts[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xints = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xints:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside


def find_better_edge(vor, v0, v1, dict_ridge_points, ridge_points_v0, path_lines=[], side_vectors=[]):
    v0_x, v0_y = vor.vertices[v0]
    v1_x, v1_y = vor.vertices[v1]
    if (v0, v1) not in side_vectors or (v1, v0) not in side_vectors:
        for rv in ridge_points_v0:
            if rv == v1:
                continue
            if len(dict_ridge_points[rv]) <= 2:
                for rrv in ridge_points_v0:
                    rrv_x, rrv_y = vor.vertices[rrv]
                    if len(dict_ridge_points[rrv]) >= 3:
                        if rrv_y > v0_y:
                            continue
                        return (rrv, v1)
    return (v0, v1)


def get_vectors_area(line, lines, dict_ridge_points, vor, side_vectors, right, range_val=960):
    v0, v1 = line
    v0_x, v0_y = vor.vertices[v0]
    v1_x, v1_y = vor.vertices[v1]
    
    ridge_points_v0 = dict_ridge_points.get(v0, [])
    ridge_points_v1 = dict_ridge_points.get(v1, [])
    
    angle_val = get_angle(v0_x, v0_y, v1_x, v1_y)
    if 80 < angle_val < 100:
        return 0, side_vectors
        
    if v0_x < v1_x:
        if v1_x - (320 * range_val / 960) > v0_x:
            return 0, side_vectors
    else:
        if v0_x - (320 * range_val / 960) > v1_x:
            return 0, side_vectors
            
    triangle_line = tuple()
    if len(ridge_points_v0) >= 3:
        is_triangle_line_found = False
        for rv in ridge_points_v0:
            rv_x, rv_y = vor.vertices[rv]
            if rv == v1:
                continue
            if right and rv_x < v0_x:
                continue
            if not right and rv_x > v0_x:
                continue
            ridge_points_rv = dict_ridge_points.get(rv, [])
            is_triangle_line_branching = False
            if len(ridge_points_rv) >= 3:
                most_distant_rv = None
                max_angle = 0
                min_angle = 180
                prev_rv = rv
                for rrv in ridge_points_rv:
                    if rrv == v0 or rrv == v1:
                        continue
                    rrv_x, rrv_y = vor.vertices[rrv]
                    rrv_angle = get_angle(v0_x, v0_y, rrv_x, rrv_y)
                    if not right and rrv_angle > max_angle:
                        max_angle = rrv_angle
                        most_distant_rv = rrv
                    if right and rrv_angle < min_angle:
                        min_angle = rrv_angle
                        most_distant_rv = rrv
                if most_distant_rv is not None:
                    rv = most_distant_rv
                    is_triangle_line_branching = True
                else:
                    continue
            if v0_x < rv_x:
                if rv_x - (320 * range_val / 960) > v0_x:
                    continue
            else:
                if v0_x - (320 * range_val / 960) > rv_x:
                    continue

            if is_triangle_line_branching:
                side_vectors.append((rv, prev_rv))
            triangle_line = (v0, rv)
            side_vectors.append(triangle_line)
            is_triangle_line_found = True
            break
        if not is_triangle_line_found:
            return 0, side_vectors
        t_x = vor.vertices[triangle_line[1]][0] - v0_x
        t_y = vor.vertices[triangle_line[1]][1] - v0_y
        v_x = v1_x - v0_x
        v_y = v1_y - v0_y
    elif len(ridge_points_v1) >= 3:
        is_triangle_line_found = False
        for rv in ridge_points_v1:
            rv_x, rv_y = vor.vertices[rv]
            if rv == v0:
                continue
            if right and rv_x < v1_x:
                continue
            if not right and rv_x > v1_x:
                continue
            ridge_points_rv = dict_ridge_points.get(rv, [])
            is_triangle_line_branching = False
            if len(ridge_points_rv) >= 3:
                most_distant_rv = None
                prev_rv = rv
                max_angle = 0
                min_angle = 180
                for rrv in ridge_points_rv:
                    if rrv == v0 or rrv == v1:
                        continue
                    rrv_x, rrv_y = vor.vertices[rrv]
                    rrv_angle = get_angle(v1_x, v1_y, rrv_x, rrv_y)
                    if not right and rrv_angle > max_angle:
                        max_angle = rrv_angle
                        most_distant_rv = rrv
                    if right and rrv_angle < min_angle:
                        min_angle = rrv_angle
                        most_distant_rv = rrv
                if most_distant_rv is not None:
                    is_triangle_line_branching = True
                    rv = most_distant_rv
                else:
                    continue
            if v1_x < rv_x:
                if rv_x - (320 * range_val / 960) > v1_x:
                    continue
            else:
                if v1_x - (320 * range_val / 960) > rv_x:
                    continue
            if is_triangle_line_branching:
                side_vectors.append((rv, prev_rv))

            triangle_line = (v1, rv)
            side_vectors.append(triangle_line)
            is_triangle_line_found = True
            break
        if not is_triangle_line_found:
            return 0, side_vectors
        t_x = vor.vertices[triangle_line[1]][0] - v1_x
        t_y = vor.vertices[triangle_line[1]][1] - v1_y
        v_x = v0_x - v1_x
        v_y = v0_y - v1_y
    else:
        return 0, side_vectors

    area = 0.5 * abs(v_y * t_x - v_x * t_y)
    return area, side_vectors


def get_path_lines(skeleton_lines, vor, dict_ridge_points, side_vectors):
    path_lines = []
    for line in skeleton_lines:
        v0, v1 = line
        ridge_points_v0 = dict_ridge_points.get(v0, [])
        ridge_points_v1 = dict_ridge_points.get(v1, [])

        if len(ridge_points_v0) >= 3 and len(ridge_points_v1) >= 3:
            continue

        if line in side_vectors or (line[1], line[0]) in side_vectors or line in path_lines or (line[1], line[0]) in path_lines:
            continue

        if len(ridge_points_v0) >= 3:
            line = find_better_edge(vor, v0, v1, dict_ridge_points, ridge_points_v0, path_lines, side_vectors)
            path_lines.append(line)
        elif len(ridge_points_v1) >= 3:
            line = find_better_edge(vor, v1, v0, dict_ridge_points, ridge_points_v1, path_lines, side_vectors)
            path_lines.append(line)
        if len(ridge_points_v0) == 1 and len(ridge_points_v1) == 1:
            path_lines.append(line)
        
    return path_lines


def get_right_lowest_and_left_lowest_line(lines, vor):
    if not lines:
        return None, None
        
    sorted_lines = sorted(
        lines,
        key=lambda l: get_midpoint(vor.vertices[l[0]], vor.vertices[l[1]])[0]
    )
    
    # Handle single line edge-case safely
    if len(sorted_lines) == 1:
        return sorted_lines[0], sorted_lines[0]

    mid_idx = len(sorted_lines) // 2
    left_lines = sorted_lines[:mid_idx]
    right_lines = sorted_lines[mid_idx:]

    # Ensure no division results in empty arrays
    if not left_lines:
        left_lines = [sorted_lines[0]]
    if not right_lines:
        right_lines = [sorted_lines[-1]]

    left_points = [(l, v) for l in left_lines for v in l]
    right_points = [(l, v) for l in right_lines for v in l]

    if not left_points:
        leftmost_line = sorted_lines[0]
    else:
        leftmost_line, _ = min(
            left_points,
            key=lambda lv: vor.vertices[lv[1]][1]
        )

    if not right_points:
        rightmost_line = sorted_lines[-1]
    else:
        rightmost_line, _ = min(
            right_points,
            key=lambda lv: vor.vertices[lv[1]][1]
        )
        
    return leftmost_line, rightmost_line


def find_straight_path(path_lines, vor, prev_coordinates, scale_factor=1.0):
    if prev_coordinates is not None:
        closest_distance = float('inf')
        possible_path = None
        possible_paths = []
        max_search_distance = 150.0 * scale_factor
        for line in path_lines:
            v0, v1 = line
            v0_x, v0_y = vor.vertices[v0]
            v1_x, v1_y = vor.vertices[v1]
            if v0_y > v1_y:
                v0_x, v0_y, v1_x, v1_y = v1_x, v1_y, v0_x, v0_y
            midpoint = get_midpoint((v0_x, v0_y), (v1_x, v1_y))
            distance = math.sqrt((midpoint[0] - prev_coordinates[0][0]) ** 2 + (midpoint[1] - prev_coordinates[0][1]) ** 2)
            if distance < closest_distance and distance < max_search_distance:
                closest_distance = distance
                possible_path = line
                possible_paths.append(line)
        
        if possible_path is not None:
            best_angle = get_angle(vor.vertices[possible_path[0]][0], vor.vertices[possible_path[0]][1], vor.vertices[possible_path[1]][0], vor.vertices[possible_path[1]][1])
            return possible_paths, possible_path, best_angle
    
    best_angle = 0
    max_height = 0
    possible_path = None
    possible_paths = []
    for line in path_lines:
        v0, v1 = line
        v0_x, v0_y = vor.vertices[v0]
        v1_x, v1_y = vor.vertices[v1]
        if v0_y > v1_y:
            v0_x, v0_y, v1_x, v1_y = v1_x, v1_y, v0_x, v0_y
        
        angle = get_angle(v0_x, v0_y, v1_x, v1_y)
        if 70 < angle < 110:
            if v1_y > max_height:
                max_height = v1_y
                possible_path = line
                best_angle = angle
                possible_paths.append(line)
    return possible_paths, possible_path, best_angle


def get_skeleton_lines(vor, poly_pts):
    dict_ridge_points = {i: list() for i in range(len(vor.vertices))}
    ridge_lines = []
    vertex_to_edge = {}

    for rv in vor.ridge_vertices:
        if -1 in rv:
            continue
        v0, v1 = vor.vertices[rv[0]], vor.vertices[rv[1]]
        
        # Fast midpoint & endpoint checks (no shapely dependency)
        if not is_point_in_polygon(v0, poly_pts):
            continue
        if not is_point_in_polygon(v1, poly_pts):
            continue
        mid = ((v0[0] + v1[0]) / 2.0, (v0[1] + v1[1]) / 2.0)
        if not is_point_in_polygon(mid, poly_pts):
            continue
            
        dict_ridge_points[rv[0]].append(rv[1])
        dict_ridge_points[rv[1]].append(rv[0])

    for rv in vor.ridge_vertices:
        if -1 in rv:
            continue
        v0, v1 = vor.vertices[rv[0]], vor.vertices[rv[1]]
        
        # Fast midpoint & endpoint checks
        if not is_point_in_polygon(v0, poly_pts):
            continue
        if not is_point_in_polygon(v1, poly_pts):
            continue
        mid = ((v0[0] + v1[0]) / 2.0, (v0[1] + v1[1]) / 2.0)
        if not is_point_in_polygon(mid, poly_pts):
            continue
            
        v0, v1 = rv

        edge0 = vertex_to_edge.get(v0)
        edge1 = vertex_to_edge.get(v1)

        if edge0 is not None and len(dict_ridge_points[v0]) < 3:
            if edge0[0] == v0:
                edge0[0] = v1
            else:
                edge0[1] = v1
            vertex_to_edge[v1] = edge0
        elif edge1 is not None and len(dict_ridge_points[v1]) < 3:
            if edge1[0] == v1:
                edge1[0] = v0
            else:
                edge1[1] = v0
            vertex_to_edge[v0] = edge1
        else:
            new_edge = [v0, v1]
            ridge_lines.append(new_edge)
            vertex_to_edge[v0] = new_edge
            vertex_to_edge[v1] = new_edge

    dict_ridge_points = {i: list() for i in range(len(vor.vertices))}
    adj = defaultdict(list)
    for a, b in ridge_lines:
        adj[a].append(b)
        adj[b].append(a)     
    junctions = {v for v in adj if len(adj[v]) != 2}
    
    skeleton_lines = []
    for a in ridge_lines:
        if a[0] in junctions and a[1] in junctions:
            pass
        elif a[0] in junctions:
            curr = a[1]
            prev = a[0]
            while curr not in junctions:
                neighbors = adj[curr]
                next_v = neighbors[0] if neighbors[0] != prev else neighbors[1]
                prev, curr = curr, next_v
            a[1] = curr
        elif a[1] in junctions:
            curr = a[0]
            prev = a[1]
            while curr not in junctions:
                neighbors = adj[curr]
                next_v = neighbors[0] if neighbors[0] != prev else neighbors[1]
                prev, curr = curr, next_v
            a[0] = curr
        else:
            continue
            
        if a[0] not in dict_ridge_points[a[1]] and a[1] not in dict_ridge_points[a[0]]:
            dict_ridge_points[a[1]].append(a[0])
            dict_ridge_points[a[0]].append(a[1])
        if a not in skeleton_lines and a[::-1] not in skeleton_lines:
            skeleton_lines.append(tuple(a))
            
    return skeleton_lines, dict_ridge_points


def interpreting_skeletons(skeleton_lines, dict_ridge_points, vor, straight_path=None, scale_factor=1.0):
    if not skeleton_lines:
        return [], 0.0, 0.0, None, [], 0.0, 0.0

    # Handle single line scenario to prevent min() errors
    if len(skeleton_lines) < 2:
        single_line = skeleton_lines[0]
        v0, v1 = single_line
        v0_x, v0_y = vor.vertices[v0]
        v1_x, v1_y = vor.vertices[v1]
        if v0_y > v1_y:
            v0_x, v0_y, v1_x, v1_y = v1_x, v1_y, v0_x, v0_y
        best_angle = get_angle(v0_x, v0_y, v1_x, v1_y)
        return skeleton_lines, best_angle, 0.0, single_line, [], 0.0, 0.0

    left_lowest_line, right_lowest_line = get_right_lowest_and_left_lowest_line(skeleton_lines, vor)
    if left_lowest_line is None or right_lowest_line is None:
        return [], 0.0, 0.0, None, [], 0.0, 0.0

    range_val = max(vor.vertices[right_lowest_line[0]][0], vor.vertices[right_lowest_line[1]][0]) - \
                min(vor.vertices[left_lowest_line[0]][0], vor.vertices[left_lowest_line[1]][0])
    
    side_vectors = []
    side_vectors.append(left_lowest_line)
    side_vectors.append(right_lowest_line)

    area_left, side_vectors = get_vectors_area(left_lowest_line, skeleton_lines, dict_ridge_points, vor, side_vectors, False, range_val)
    area_right, side_vectors = get_vectors_area(right_lowest_line, skeleton_lines, dict_ridge_points, vor, side_vectors, True, range_val)

    area_difference = area_left - area_right
    area_percentage_difference = (area_difference / max(area_left, area_right)) * 100 if max(area_left, area_right) > 0 else 0

    path_lines = get_path_lines(skeleton_lines, vor, dict_ridge_points, side_vectors)
    best_possible_path = None
    best_angle = 0.0

    if len(path_lines) == 1:
        v0, v1 = path_lines[0]
        v0_x, v0_y = vor.vertices[v0]
        v1_x, v1_y = vor.vertices[v1]
        if v0_y > v1_y:
            v0_x, v0_y, v1_x, v1_y = v1_x, v1_y, v0_x, v0_y
        best_angle = get_angle(v0_x, v0_y, v1_x, v1_y)
        best_possible_path = path_lines[0]
    else:
        _, best_possible_path, best_angle = find_straight_path(path_lines, vor, straight_path, scale_factor)
    
    return path_lines, best_angle, area_percentage_difference, best_possible_path, side_vectors, area_left, area_right


# =====================================================================
# ROS 2 NODE INTERFACE
# =====================================================================

class VoronoiPathPlanner(Node):
    def __init__(self):
        super().__init__('voronoi_path_planner')
        
        # ROS 2 Node Parameters
        self.declare_parameter('input_topic', '/camera/sidewalk_mask')
        self.declare_parameter('target_gray', 255)
        self.declare_parameter('resize_width', 960)
        self.declare_parameter('resize_height', 720)
        
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.target_gray = self.get_parameter('target_gray').get_parameter_value().integer_value
        self.resize_width = self.get_parameter('resize_width').get_parameter_value().integer_value
        self.resize_height = self.get_parameter('resize_height').get_parameter_value().integer_value
        
        # State tracking
        self.prev_best_path_coords = None  
        
        # Publishers
        self.debug_img_pub = self.create_publisher(Image, '/voronoi/debug_image', 10)
        self.angle_pub = self.create_publisher(Float32, '/voronoi/best_angle', 10)
        self.area_diff_pub = self.create_publisher(Float32, '/voronoi/area_difference', 10)
        self.best_path_pub = self.create_publisher(PoseArray, '/voronoi/best_path', 10)
        self.area_left_pub = self.create_publisher(Float32, '/voronoi/area_left', 10)
        self.area_right_pub = self.create_publisher(Float32, '/voronoi/area_right', 10)
        
        # Subscriber (Latest-only QoS pattern to completely avoid latency backlogs)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.mask_sub = self.create_subscription(
            Image,
            self.input_topic,
            self.mask_callback,
            qos_profile
        )
        
        self.get_logger().info(
            f"Optimized Voronoi Planner initialized.\n"
            f"Processing: {self.resize_width}x{self.resize_height} | Display: {self.resize_width}x{self.resize_height}"
        )

    def mask_callback(self, msg: Image):
        try:
            # 1. Decode raw image message natively
            if msg.encoding in ['mono8', '8UC1']:
                clean_mask = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
            elif msg.encoding in ['rgb8', 'bgr8']:
                raw_img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                clean_mask = cv2.cvtColor(raw_img, cv2.COLOR_BGR2GRAY)
            else:
                self.get_logger().error(f"Unsupported image encoding received: {msg.encoding}")
                return

            # 2. Rescale image to targeted standard dimensions (Optimized back to 960x720)
            w, h = self.resize_width, self.resize_height
            clean_mask = cv2.resize(clean_mask, (w, h))

            # 3. Apply Thresholding / Label Extraction logic
            if self.target_gray == 255 or self.target_gray <= 0:
                _, binary_mask = cv2.threshold(clean_mask, 127, 255, cv2.THRESH_BINARY)
            else:
                binary_mask = cv2.inRange(clean_mask, self.target_gray, self.target_gray)

            # if np.count_nonzero(binary_mask) == 0:
            #     self.publish_empty_pose_array(msg.header)
            #     self.publish_blank_debug_image(msg.header) # <-- Added safety display trigger
            #     return

             # 4. Extract external contours for boundaries
            contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            if not contours:
                return

                
            contour = max(contours, key=cv2.contourArea)
            boundary_points = contour.squeeze()

            boundary_points_cartesian = boundary_points.copy()
            boundary_points_cartesian[:, 1] = h - boundary_points_cartesian[:, 1]

            # Secure mathematical stability by downsampling boundary density
            num_pts = len(boundary_points_cartesian)
            step = min(50, max(1, num_pts // 4))
            boundary_points_ordered = boundary_points_cartesian[::step]

            if len(boundary_points_ordered) < 4:
                boundary_points_ordered = boundary_points_cartesian

            # Build polygon using a buffered boundary shape
            polygon = Polygon(boundary_points_cartesian).buffer(0)

            # Compute Voronoi Structure on the 960x720 scaled points
            vor = Voronoi(boundary_points_ordered)

            # Process Voronoi finite skeletal vectors using fast Point-in-Polygon checks
            skeleton_lines, dict_ridge_points = get_skeleton_lines(vor, boundary_points_ordered)
            # if not skeleton_lines:
            #     self.publish_empty_pose_array(msg.header)
            #     self.publish_blank_debug_image(msg.header) # <-- Added safety display trigger
            #     return

            # Compute target planning vectors
            paths, best_angle, area_percentage_diff, best_path, side_vectors, area_left, area_right = interpreting_skeletons(
                skeleton_lines, dict_ridge_points, vor, self.prev_best_path_coords
            )

            # # Retain coordinate state track history (in unscaled 960x720 space)
            # if best_path is not None:
            #     p0_cart = vor.vertices[best_path[0]]
            #     p1_cart = vor.vertices[best_path[1]]
            #     self.prev_best_path_coords = (p0_cart, p1_cart)
                
            #     # Publish physical coordinates directly (no scaling back needed)
            #     self.publish_best_path_pose_array(msg.header, p0_cart, p1_cart)
            # else:
            #     self.prev_best_path_coords = None
            #     self.publish_empty_pose_array(msg.header)
            #     self.publish_blank_debug_image(msg.header) # <-- Added safety display trigger
            #     return

            # # 5. Publish Steering Control Parameters
            angle_msg = Float32()
            angle_msg.data = float(best_angle)
            self.angle_pub.publish(angle_msg)

            area_msg = Float32()
            area_msg.data = float(area_percentage_diff)
            self.area_diff_pub.publish(area_msg)

            # Publish individual areas directly (no scale multipliers needed)
            area_left_msg = Float32()
            area_left_msg.data = float(area_left)
            self.area_left_pub.publish(area_left_msg)
            
            area_right_msg = Float32()
            area_right_msg.data = float(area_right)
            self.area_right_pub.publish(area_right_msg)

            # 6. Build and publish visual debug mapping (scaled to 0.5x of the target resize dimensions)
            dbg_w = int(self.resize_width * 0.5)
            dbg_h = int(self.resize_height * 0.5)
            debug_bgr = self.generate_debug_image(
                (dbg_h, dbg_w), boundary_points_ordered, skeleton_lines, paths, side_vectors, best_path, vor
            )
            self.publish_debug_image(msg.header, debug_bgr)

        except Exception as e:
            self.get_logger().error(f"Voronoi pipeline error: {str(e)}")

    def generate_debug_image(self, shape, boundary_points, skeleton, paths, side_vectors, best_path, vor):
        h, w = shape  # Debug canvas dimensions (e.g. 360, 480)
        canvas = np.zeros((h, w, 3), dtype=np.uint8)

        # Scale coordinates from 960x720 processing space to target debug display space
        scale_x = w / float(self.resize_width)  # 480 / 960 = 0.5
        scale_y = h / float(self.resize_height) # 360 / 720 = 0.5

        # Helper to convert Cartesian coordinate from vor.vertices back to standard Image space
        def to_img_space(pt):
            return int(pt[0] * scale_x), int(h - (pt[1] * scale_y))

        # Dynamically scale visual elements (0.5 scale factor for 0.5x display)
        line_thick = max(1, int(2 * 0.5))      
        best_line_thick = max(1, int(4 * 0.5)) 
        circle_rad = max(2, int(6 * 0.5))      

        # Draw outer boundary contour fill
        boundary_pts_img = np.array([to_img_space(p) for p in boundary_points], dtype=np.int32)
        cv2.polylines(canvas, [boundary_pts_img], isClosed=True, color=(120, 120, 120), thickness=line_thick)
        cv2.fillPoly(canvas, [boundary_pts_img], color=(30, 30, 30))

        # 1. Draw raw skeleton lines (Cyan/Blue)
        for v0, v1 in skeleton:
            p0 = to_img_space(vor.vertices[v0])
            p1 = to_img_space(vor.vertices[v1])
            cv2.line(canvas, p0, p1, (255, 100, 0), line_thick)

        # 2. Draw side vector elements (Red)
        for v0, v1 in side_vectors:
            p0 = to_img_space(vor.vertices[v0])
            p1 = to_img_space(vor.vertices[v1])
            cv2.line(canvas, p0, p1, (0, 0, 255), line_thick)

        # 3. Draw parsed candidate pathways (Green)
        for v0, v1 in paths:
            p0 = to_img_space(vor.vertices[v0])
            p1 = to_img_space(vor.vertices[v1])
            cv2.line(canvas, p0, p1, (0, 255, 0), line_thick)

        # 4. Draw targeted path selection (Yellow/Orange)
        if best_path is not None:
            p0 = to_img_space(vor.vertices[best_path[0]])
            p1 = to_img_space(vor.vertices[best_path[1]])
            cv2.line(canvas, p0, p1, (0, 255, 255), best_line_thick)
            cv2.circle(canvas, p0, circle_rad, (0, 165, 255), -1)
            cv2.circle(canvas, p1, circle_rad, (0, 165, 255), -1)

        return canvas

    def publish_debug_image(self, header, debug_bgr):
        debug_rgb = cv2.cvtColor(debug_bgr, cv2.COLOR_BGR2RGB)
        
        debug_msg = Image()
        debug_msg.header = header
        debug_msg.height = debug_rgb.shape[0]
        debug_msg.width = debug_rgb.shape[1]
        debug_msg.encoding = 'rgb8'
        debug_msg.is_bigendian = 0
        debug_msg.step = debug_rgb.shape[1] * 3
        debug_msg.data = debug_rgb.tobytes()
        
        self.debug_img_pub.publish(debug_msg)

    def publish_blank_debug_image(self, header):
        """Draws a 'NO PATH DETECTED' warning canvas to keep the display active on startup."""
        dbg_w = int(self.resize_width * 0.5)
        dbg_h = int(self.resize_height * 0.5)
        canvas = np.zeros((dbg_h, dbg_w, 3), dtype=np.uint8)
        
        cv2.putText(
            canvas, 
            "NO PATH DETECTED", 
            (int(dbg_w * 0.15), int(dbg_h * 0.5)),
            cv2.FONT_HERSHEY_SIMPLEX, 
            0.7, 
            (0, 0, 255), 
            2
        )
        self.publish_debug_image(header, canvas)

    def publish_best_path_pose_array(self, header, p0_cart, p1_cart):
        pose_array = PoseArray()
        pose_array.header = header

        pose0 = Pose()
        pose0.position.x = float(p0_cart[0])
        pose0.position.y = float(p0_cart[1])
        pose0.position.z = 0.0

        pose1 = Pose()
        pose1.position.x = float(p1_cart[0]) # <-- Fixed typo from p0_cart
        pose1.position.y = float(p1_cart[1])
        pose1.position.z = 0.0

        pose_array.poses = [pose0, pose1]
        self.best_path_pub.publish(pose_array)

    def publish_empty_pose_array(self, header):
        pose_array = PoseArray()
        pose_array.header = header
        pose_array.poses = []
        self.best_path_pub.publish(pose_array)


def main(args=None):
    rclpy.init(args=args)
    node = VoronoiPathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()