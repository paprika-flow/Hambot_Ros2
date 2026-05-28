#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseArray, Pose

import cv2
import numpy as np
import math
import bisect
from collections import defaultdict
from scipy.spatial import Voronoi
from shapely.geometry import Polygon, LineString

# =====================================================================
# INTEGRATED GEOMETRIC MATH & SKELETONIZATION LOGIC
# =====================================================================

def get_midpoint(p1, p2):
    return [(p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2]


def get_distance(p1, p2):
    return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)


def get_angle(x1, y1, x2, y2):
    return math.atan2(y2 - y1, x2 - x1) * 180 / math.pi


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
    sorted_lines = sorted(
        lines,
        key=lambda l: get_midpoint(vor.vertices[l[0]], vor.vertices[l[1]])[0]
    )
    left_lines = sorted_lines[:len(sorted_lines) // 2]
    right_lines = sorted_lines[len(sorted_lines) // 2:]

    leftmost_line, _ = min(
        ((l, v) for l in left_lines for v in l),
        key=lambda lv: vor.vertices[lv[1]][1]
    )
    rightmost_line, _ = min(
        ((l, v) for l in right_lines for v in l),
        key=lambda lv: vor.vertices[lv[1]][1]
    )
    return leftmost_line, rightmost_line


def find_straight_path(path_lines, vor, prev_coordinates):
    if prev_coordinates is not None:
        closest_distance = float('inf')
        possible_path = None
        possible_paths = []
        for line in path_lines:
            v0, v1 = line
            v0_x, v0_y = vor.vertices[v0]
            v1_x, v1_y = vor.vertices[v1]
            if v0_y > v1_y:
                v0_x, v0_y, v1_x, v1_y = v1_x, v1_y, v0_x, v0_y
            midpoint = get_midpoint((v0_x, v0_y), (v1_x, v1_y))
            distance = math.sqrt((midpoint[0] - prev_coordinates[0][0]) ** 2 + (midpoint[1] - prev_coordinates[0][1]) ** 2)
            if distance < closest_distance and distance < 150:
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


def get_skeleton_lines(vor, polygon=None):
    dict_ridge_points = {i: list() for i in range(len(vor.vertices))}
    ridge_lines = []
    vertex_to_edge = {}

    for rv in vor.ridge_vertices:
        if -1 in rv:
            continue
        v0, v1 = vor.vertices[rv[0]], vor.vertices[rv[1]]
        line = LineString([v0, v1])
        if not polygon.covers(line):
            continue
        dict_ridge_points[rv[0]].append(rv[1])
        dict_ridge_points[rv[1]].append(rv[0])

    for rv in vor.ridge_vertices:
        if -1 in rv:
            continue
        v0, v1 = vor.vertices[rv[0]], vor.vertices[rv[1]]
        line = LineString([v0, v1])
        if not polygon.covers(line):
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


def interpreting_skeletons(skeleton_lines, dict_ridge_points, vor, straight_path=None):
    if not skeleton_lines:
        return [], 0.0, 0.0, None, []

    side_vectors = []
    left_lowest_line, right_lowest_line = get_right_lowest_and_left_lowest_line(skeleton_lines, vor)

    range_val = max(vor.vertices[right_lowest_line[0]][0], vor.vertices[right_lowest_line[1]][0]) - \
                min(vor.vertices[left_lowest_line[0]][0], vor.vertices[left_lowest_line[1]][0])
    
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
        _, best_possible_path, best_angle = find_straight_path(path_lines, vor, straight_path)
    
    return path_lines, best_angle, area_percentage_difference, best_possible_path, side_vectors

# =====================================================================
# ROS 2 NODE INTERFACE
# =====================================================================

class VoronoiPathPlanner(Node):
    def __init__(self):
        super().__init__('voronoi_path_planner')
        
        # ROS 2 Node Parameters
        self.declare_parameter('input_topic', '/camera/sidewalk_mask')
        # target_gray configuration:
        # If target_gray is 255 or 0, we process it as a binary image (foreground > 127).
        # If a label is supplied (e.g. 1 or 15), we filter strictly for that exact mask label value.
        self.declare_parameter('target_gray', 255)
        self.declare_parameter('resize_width', 960)
        self.declare_parameter('resize_height', 720)
        
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.target_gray = self.get_parameter('target_gray').get_parameter_value().integer_value
        self.resize_width = self.get_parameter('resize_width').get_parameter_value().integer_value
        self.resize_height = self.get_parameter('resize_height').get_parameter_value().integer_value
        
        # State tracking to provide straight path temporal consistency
        self.prev_best_path_coords = None  # Tracks: ((x0, y0), (x1, y1))
        
        # Publishers
        self.debug_img_pub = self.create_publisher(Image, '/voronoi/debug_image', 10)
        self.angle_pub = self.create_publisher(Float32, '/voronoi/best_angle', 10)
        self.area_diff_pub = self.create_publisher(Float32, '/voronoi/area_difference', 10)
        self.best_path_pub = self.create_publisher(PoseArray, '/voronoi/best_path', 10)
        
        # Subscriber (Natively decodes image arrays to bypass cv_bridge Python/NumPy ABI mismatches) [2]
        self.mask_sub = self.create_subscription(
            Image,
            self.input_topic,
            self.mask_callback,
            10
        )
        
        self.get_logger().info(
            f"Voronoi Path Planner initialized. Subscribed to '{self.input_topic}' "
            f"(Target Filter Label: {self.target_gray})."
        )

    def mask_callback(self, msg: Image):
        try:
            # 1. Decode raw image message natively [2]
            if msg.encoding in ['mono8', '8UC1']:
                clean_mask = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
            elif msg.encoding in ['rgb8', 'bgr8']:
                raw_img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                clean_mask = cv2.cvtColor(raw_img, cv2.COLOR_BGR2GRAY)
            else:
                self.get_logger().error(f"Unsupported image encoding received: {msg.encoding}")
                return

            # 2. Rescale image to targeted standard dimensions
            target_size = (self.resize_width, self.resize_height)
            clean_mask = cv2.resize(clean_mask, target_size)

            # 3. Apply Thresholding / Label Extraction logic
            if self.target_gray == 255 or self.target_gray <= 0:
                _, binary_mask = cv2.threshold(clean_mask, 127, 255, cv2.THRESH_BINARY)
            else:
                binary_mask = cv2.inRange(clean_mask, self.target_gray, self.target_gray)

            if np.count_nonzero(binary_mask) == 0:
                self.get_logger().warning("No path pixels matching target label detected.", throttle_duration_sec=4.0)
                return

            # 4. Extract external contours for boundaries
            contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            if not contours:
                return
            contour = max(contours, key=cv2.contourArea)
            boundary_points = contour.squeeze()

            if boundary_points.ndim < 2 or len(boundary_points) < 4:
                return

            # Map image boundary coords to Cartesian space by flipping Y vertical layout
            h, w = binary_mask.shape[:2]
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

            # Compute Voronoi Structure
            vor = Voronoi(boundary_points_ordered)

            # Process Voronoi finite skeletal vectors
            skeleton_lines, dict_ridge_points = get_skeleton_lines(vor, polygon)
            if not skeleton_lines:
                return

            # Compute target planning vectors
            paths, best_angle, area_percentage_diff, best_path, side_vectors = interpreting_skeletons(
                skeleton_lines, dict_ridge_points, vor, self.prev_best_path_coords
            )

            # Retain coordinate state track history
            if best_path is not None:
                p0_cart = vor.vertices[best_path[0]]
                p1_cart = vor.vertices[best_path[1]]
                self.prev_best_path_coords = (p0_cart, p1_cart)
                
                # Publish physical coordinates
                self.publish_best_path_pose_array(msg.header, p0_cart, p1_cart)
            else:
                self.prev_best_path_coords = None

            # 5. Publish Steering Control Parameters
            angle_msg = Float32()
            angle_msg.data = float(best_angle)
            self.angle_pub.publish(angle_msg)

            area_msg = Float32()
            area_msg.data = float(area_percentage_diff)
            self.area_diff_pub.publish(area_msg)

            # 6. Build and publish visual debug mapping
            debug_bgr = self.generate_debug_image(
                (h, w), boundary_points_ordered, skeleton_lines, paths, side_vectors, best_path, vor
            )
            self.publish_debug_image(msg.header, debug_bgr)

        except Exception as e:
            self.get_logger().error(f"Voronoi pipeline error: {str(e)}")

    def generate_debug_image(self, shape, boundary_points, skeleton, paths, side_vectors, best_path, vor):
        h, w = shape
        canvas = np.zeros((h, w, 3), dtype=np.uint8)

        # Helper to convert Cartesian coordinate from vor.vertices back to standard Image space
        def to_img_space(pt):
            return int(pt[0]), int(h - pt[1])

        # Draw outer boundary contour fill
        boundary_pts_img = np.array([to_img_space(p) for p in boundary_points], dtype=np.int32)
        cv2.polylines(canvas, [boundary_pts_img], isClosed=True, color=(120, 120, 120), thickness=2)
        cv2.fillPoly(canvas, [boundary_pts_img], color=(30, 30, 30))

        # 1. Draw raw skeleton lines (Cyan/Blue)
        for v0, v1 in skeleton:
            p0 = to_img_space(vor.vertices[v0])
            p1 = to_img_space(vor.vertices[v1])
            cv2.line(canvas, p0, p1, (255, 100, 0), 1)

        # 2. Draw side vector elements (Red)
        for v0, v1 in side_vectors:
            p0 = to_img_space(vor.vertices[v0])
            p1 = to_img_space(vor.vertices[v1])
            cv2.line(canvas, p0, p1, (0, 0, 255), 2)

        # 3. Draw parsed candidate pathways (Green)
        for v0, v1 in paths:
            p0 = to_img_space(vor.vertices[v0])
            p1 = to_img_space(vor.vertices[v1])
            cv2.line(canvas, p0, p1, (0, 255, 0), 2)

        # 4. Draw targeted path selection (Yellow/Orange)
        if best_path is not None:
            p0 = to_img_space(vor.vertices[best_path[0]])
            p1 = to_img_space(vor.vertices[best_path[1]])
            cv2.line(canvas, p0, p1, (0, 255, 255), 4)
            cv2.circle(canvas, p0, 6, (0, 165, 255), -1)
            cv2.circle(canvas, p1, 6, (0, 165, 255), -1)

        return canvas

    def publish_debug_image(self, header, debug_bgr):
        # 1. Convert standard OpenCV BGR array to RGB for Gazebo compatibility
        debug_rgb = cv2.cvtColor(debug_bgr, cv2.COLOR_BGR2RGB)
        
        # 2. Build the ROS 2 Image message natively with 'rgb8' encoding
        debug_msg = Image()
        debug_msg.header = header
        debug_msg.height = debug_rgb.shape[0]
        debug_msg.width = debug_rgb.shape[1]
        debug_msg.encoding = 'rgb8'  # Upgraded to standard RGB
        debug_msg.is_bigendian = 0
        debug_msg.step = debug_rgb.shape[1] * 3
        debug_msg.data = debug_rgb.tobytes()
        
        self.debug_img_pub.publish(debug_msg)

    def publish_best_path_pose_array(self, header, p0_cart, p1_cart):
        pose_array = PoseArray()
        pose_array.header = header

        pose0 = Pose()
        pose0.position.x = float(p0_cart[0])
        pose0.position.y = float(p0_cart[1])
        pose0.position.z = 0.0

        pose1 = Pose()
        pose1.position.x = float(p1_cart[0])
        pose1.position.y = float(p1_cart[1])
        pose1.position.z = 0.0

        pose_array.poses = [pose0, pose1]
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