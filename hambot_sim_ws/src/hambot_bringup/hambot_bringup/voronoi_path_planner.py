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
import os
import joblib
from collections import defaultdict
from scipy.spatial import Voronoi
from shapely.geometry import Polygon


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
    """
    Looks for an alternative lower junction vertex (rrv) to optimize the path, 
    but stays at the original local junction (v0, v1) if that lower vertex is 
    already part of any active side vector.
    """
    v0_x, v0_y = vor.vertices[v0]
    v1_x, v1_y = vor.vertices[v1]
    
    # Only evaluate optimization if the current segment is not part of a side vector
    if (v0, v1) not in side_vectors and (v1, v0) not in side_vectors:
        for rv in ridge_points_v0:
            if rv == v1:
                continue
            if len(dict_ridge_points[rv]) <= 2:
                for rrv in ridge_points_v0:
                    rrv_x, rrv_y = vor.vertices[rrv]
                    if len(dict_ridge_points[rrv]) >= 3:
                        if rrv_y > v0_y:
                            continue
                        
                        # Guard condition: If the candidate lower point (rrv) is 
                        # already inside a side vector, stay at the original point.
                        if any(rrv in edge for edge in side_vectors):
                            continue
                            
                        return (rrv, v1)
    return (v0, v1)


def is_line_on_sidewalk(p0, p1, binary_mask, h):
    """
    Checks if a line between two Cartesian points crosses any non-sidewalk pixel (0).
    Maps Cartesian coordinates (flipped y) back to image array indices.
    """
    x0, y0 = int(round(p0[0])), int(round(h - p0[1]))
    x1, y1 = int(round(p1[0])), int(round(h - p1[1]))
    
    num_samples = max(abs(x1 - x0), abs(y1 - y0), 2) * 2
        
    x_coords = np.linspace(x0, x1, num_samples)
    y_coords = np.linspace(y0, y1, num_samples)
    
    mask_h, mask_w = binary_mask.shape
    
    for x, y in zip(x_coords, y_coords):
        col, row = int(round(x)), int(round(y))
        if 0 <= col < mask_w and 0 <= row < mask_h:
            if binary_mask[row, col] == 0:  # Touches non-sidewalk pixel
                return False
        else:
            return False  # Line strays outside the image frame boundaries completely
            
    return True


def get_vectors_area(line, lines, dict_ridge_points, vor, side_vectors, right, binary_mask, h, range_val=960):
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
                
            image_center = range_val / 2.0
            if right and rv_x < image_center:
                continue  
            if not right and rv_x > image_center:
                continue  
                
            if right and rv_x < v0_x:
                continue
            if not right and rv_x > v0_x:
                continue
                
            ridge_points_rv = dict_ridge_points.get(rv, [])
            is_triangle_line_branching = False
            resolved_rv = rv
            prev_rv = rv
            
            if len(ridge_points_rv) >= 3:
                most_distant_rv = None
                max_angle = 0
                min_angle = 180
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
                    resolved_rv = most_distant_rv
                    is_triangle_line_branching = True
                else:
                    continue

            if not is_line_on_sidewalk(vor.vertices[resolved_rv], vor.vertices[v1], binary_mask, h):
                continue

            resolved_rv_x = vor.vertices[resolved_rv][0]
            if v0_x < resolved_rv_x:
                if resolved_rv_x - (320 * range_val / 960) > v0_x:
                    continue
            else:
                if v0_x - (320 * range_val / 960) > resolved_rv_x:
                    continue

            if is_triangle_line_branching:
                side_vectors.append((resolved_rv, prev_rv))
            triangle_line = (v0, resolved_rv)
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
                
            image_center = range_val / 2.0
            if right and rv_x < image_center:
                continue
            if not right and rv_x > image_center:
                continue
                
            if right and rv_x < v1_x:
                continue
            if not right and rv_x > v1_x:
                continue
                
            ridge_points_rv = dict_ridge_points.get(rv, [])
            is_triangle_line_branching = False
            resolved_rv = rv
            prev_rv = rv
            
            if len(ridge_points_rv) >= 3:
                most_distant_rv = None
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
                    resolved_rv = most_distant_rv
                    is_triangle_line_branching = True
                else:
                    continue

            if not is_line_on_sidewalk(vor.vertices[resolved_rv], vor.vertices[v0], binary_mask, h):
                continue

            resolved_rv_x = vor.vertices[resolved_rv][0]
            if v1_x < resolved_rv_x:
                if resolved_rv_x - (320 * range_val / 960) > v1_x:
                    continue
            else:
                if v1_x - (320 * range_val / 960) > v1_x:
                    continue

            if is_triangle_line_branching:
                side_vectors.append((resolved_rv, prev_rv))

            triangle_line = (v1, resolved_rv)
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
    """
    Extracts candidate path lines. 
    - Preserves standalone lanes (like the left lane) as green paths.
    - Leaves bridges connecting multiple junctions or touching side vectors as blue.
    """
    path_lines = []
    
    # Flatten side vector edges to a set of vertices for fast connectivity lookup
    side_vertices = {v for edge in side_vectors for v in edge}
    
    for line in skeleton_lines:
        v0, v1 = line
        ridge_points_v0 = dict_ridge_points.get(v0, [])
        ridge_points_v1 = dict_ridge_points.get(v1, [])

        # Skip if the line (or its reverse) is already a side vector or already added
        if line in side_vectors or (line[1], line[0]) in side_vectors or line in path_lines or (line[1], line[0]) in path_lines:
            continue

        # 1. "connecting other path lines" -> If both endpoints are junctions, keep it blue (skip)
        if len(ridge_points_v0) >= 3 and len(ridge_points_v1) >= 3:
            continue

       
        # Optimize junction connections using find_better_edge
        if len(ridge_points_v0) >= 3 and len(ridge_points_v1) < 3:
            line = find_better_edge(vor, v0, v1, dict_ridge_points, ridge_points_v0, path_lines, side_vectors)
        elif len(ridge_points_v1) >= 3 and len(ridge_points_v0) < 3:
            line = find_better_edge(vor, v1, v0, dict_ridge_points, ridge_points_v1, path_lines, side_vectors)
        
        # Append the (potentially optimized) line to preserve connectivity
        if line not in path_lines and (line[1], line[0]) not in path_lines:
            path_lines.append(line)
        
    return path_lines


def get_right_lowest_and_left_lowest_line(lines, vor):
    if not lines:
        return None, None
        
    sorted_lines = sorted(
        lines,
        key=lambda l: get_midpoint(vor.vertices[l[0]], vor.vertices[l[1]])[0]
    )
    
    if len(sorted_lines) == 1:
        return sorted_lines[0], sorted_lines[0]

    mid_idx = len(sorted_lines) // 2
    left_lines = sorted_lines[:mid_idx]
    right_lines = sorted_lines[mid_idx:]

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


# =====================================================================
# TOPOLOGICAL GRAPH FILTER (UNION-FIND)
# =====================================================================

class DisjointSetUnion:
    """
    An optimized Disjoint Set Union (Union-Find) structure with path compression
    used to compute a Spanning Forest and eliminate Voronoi boundary cycles.
    """
    def __init__(self, n):
        self.parent = list(range(n))

    def find(self, i):
        path = []
        while self.parent[i] != i:
            path.append(i)
            i = self.parent[i]
        for node in path:
            self.parent[node] = i
        return i

    def union(self, i, j):
        root_i = self.find(i)
        root_j = self.find(j)
        if root_i != root_j:
            self.parent[root_i] = root_j
            return True
        return False


def get_skeleton_lines(vor, poly_pts):
    """
    Computes skeletal lines by filtering Voronoi vertices, extracting raw edges,
    and using a Spanning Forest to eliminate triangles and cycles before collapsing degree-2 chains.
    """
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

    # --- TOPOLOGICAL CYCLE FILTERING (SPANNING FOREST) ---
    # Sort segments descending by length to prioritize the long main path trunks
    ridge_lines_sorted = sorted(
        ridge_lines,
        key=lambda edge: get_distance(vor.vertices[edge[0]], vor.vertices[edge[1]]),
        reverse=True
    )
    
    dsu = DisjointSetUnion(len(vor.vertices))
    spanning_ridge_lines = []
    
    # Process edges and discard those that would close a loop/triangle
    for edge in ridge_lines_sorted:
        u, v = edge[0], edge[1]
        if dsu.union(u, v):
            spanning_ridge_lines.append(edge)
            
    ridge_lines = spanning_ridge_lines
    # -----------------------------------------------------

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


def interpreting_skeletons(skeleton_lines, dict_ridge_points, vor, straight_path, binary_mask, h, scale_factor=1.0):
    if not skeleton_lines:
        return [], 0.0, 0.0, None, [], 0.0, 0.0, None, None

    if len(skeleton_lines) < 2:
        single_line = skeleton_lines[0]
        v0, v1 = single_line
        v0_x, v0_y = vor.vertices[v0]
        v1_x, v1_y = vor.vertices[v1]
        if v0_y > v1_y:
            v0_x, v0_y, v1_x, v1_y = v1_x, v1_y, v0_x, v0_y
        best_angle = get_angle(v0_x, v0_y, v1_x, v1_y)
        return skeleton_lines, best_angle, 0.0, single_line, [], 0.0, 0.0, None, None

    left_lowest_line, right_lowest_line = get_right_lowest_and_left_lowest_line(skeleton_lines, vor)
    if left_lowest_line is None or right_lowest_line is None:
        return [], 0.0, 0.0, None, [], 0.0, 0.0, None, None

    pt0_l = vor.vertices[left_lowest_line[0]]
    pt1_l = vor.vertices[left_lowest_line[1]]
    left_lower_pt = pt0_l if pt0_l[1] < pt1_l[1] else pt1_l

    pt0_r = vor.vertices[right_lowest_line[0]]
    pt1_r = vor.vertices[right_lowest_line[1]]
    right_lower_pt = pt0_r if pt0_r[1] < pt1_r[1] else pt1_r

    range_val = max(vor.vertices[right_lowest_line[0]][0], vor.vertices[right_lowest_line[1]][0]) - \
                min(vor.vertices[left_lowest_line[0]][0], vor.vertices[left_lowest_line[1]][0])
    
    side_vectors = []
    side_vectors.append(left_lowest_line)
    side_vectors.append(right_lowest_line)

    area_left, side_vectors = get_vectors_area(left_lowest_line, skeleton_lines, dict_ridge_points, vor, side_vectors, False, binary_mask, h, range_val)
    area_right, side_vectors = get_vectors_area(right_lowest_line, skeleton_lines, dict_ridge_points, vor, side_vectors, True, binary_mask, h, range_val)

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
    
    return path_lines, best_angle, area_percentage_difference, best_possible_path, side_vectors, area_left, area_right, left_lower_pt, right_lower_pt

# =====================================================================
# VORONOI CLASSIFIER FEATURE EXTRACTION
# =====================================================================

def extract_feature_vector(chars_paths, chars_side_vectors, dens_l, dens_r, dens_c,
                           img_w=960, img_h=720, eps=1e-6):
    """
    Computes identical feature structure generated during offline dataset training.
    """
    features = []

    # 1. Basic Counts
    num_paths = len(chars_paths)
    num_side = len(chars_side_vectors)
    features.append(float(num_paths))
    features.append(float(num_side))

    if num_paths > 0:
        # Unpack: (distance, angle, midpoint_x, v0, v1)
        dists = np.array([p[0] for p in chars_paths], dtype=np.float64)
        angs  = np.array([p[1] for p in chars_paths], dtype=np.float64)
        midx  = np.array([p[2] for p in chars_paths], dtype=np.float64)
        
        dl_only = np.array(dens_l, dtype=np.float64)
        dr_only = np.array(dens_r, dtype=np.float64)
        dc_only = np.array(dens_c, dtype=np.float64)

        # Normalize distance and X-position
        dist_n = dists / float(img_h)
        x_n = (midx - img_w/2) / (img_w/2)  # -1 (left) to 1 (right)

        # 1. Path Length Stats
        features.extend([
            float(np.mean(dist_n)),
            float(np.max(dist_n)),
            float(np.std(dist_n)) if num_paths > 1 else 0.0
        ])

        # 2. Horizontal Distribution
        features.extend([
            float(np.mean(x_n)),
            float(np.std(x_n)) if num_paths > 1 else 0.0,
            float(np.max(x_n) - np.min(x_n)) if num_paths > 1 else 0.0
        ])

        # 3. Angular Topology (Circular representation)
        ang_rad = np.deg2rad(angs)
        sin_mean = np.mean(np.sin(ang_rad))
        cos_mean = np.mean(np.cos(ang_rad))
        avg_ang = np.rad2deg(np.arctan2(sin_mean, cos_mean))
        
        features.append(float(avg_ang))
        features.append(float(np.std(angs)) if num_paths > 1 else 0.0)

        # 4. Density/Obstacle Metrics (Set to default 0.0 matching offline fallback)
        features.extend([
            float(np.mean(dl_only)) if len(dl_only) > 0 else 0.0,
            float(np.mean(dr_only)) if len(dr_only) > 0 else 0.0,
            float(np.mean(dc_only)) if len(dc_only) > 0 else 0.0
        ])

        # 5. Branching Symmetry (Top 2 paths)
        if num_paths >= 2:
            sorted_indices = np.argsort(dists)[::-1]
            d1, d2 = dist_n[sorted_indices[0]], dist_n[sorted_indices[1]]
            x1, x2 = x_n[sorted_indices[0]], x_n[sorted_indices[1]]
            a1, a2 = angs[sorted_indices[0]], angs[sorted_indices[1]]

            features.append(float(abs(d1 - d2) / (d1 + d2 + eps)))
            ang_diff = abs(a1 - a2) % 360.0
            features.append(float(min(ang_diff, 360.0 - ang_diff)))
            features.append(float(abs(x1 - x2)))
        else:
            features.extend([0.0, 0.0, 0.0])
    else:
        features.extend([0.0] * 14)

    # 6. Side Vector Summary
    if num_side > 0:
        side_ds = np.array([s[0] for s in chars_side_vectors], dtype=np.float64)
        features.append(float(np.mean(side_ds) / img_h))
        features.append(float(np.max(side_ds) / img_h))
    else:
        features.extend([0.0, 0.0])

    return np.asarray(features, dtype=np.float32)


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
        self.declare_parameter('model_path', '')  # Loaded via parameter
        
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.target_gray = self.get_parameter('target_gray').get_parameter_value().integer_value
        self.resize_width = self.get_parameter('resize_width').get_parameter_value().integer_value
        self.resize_height = self.get_parameter('resize_height').get_parameter_value().integer_value
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        
        # Classifier Setup
        self.classifier = None
        if self.model_path:
            if os.path.exists(self.model_path):
                try:
                    self.classifier = joblib.load(self.model_path)
                    self.get_logger().info(f"Loaded Voronoi Split Classifier successfully: {self.model_path}")
                except Exception as e:
                    self.get_logger().error(f"Failed to load SVM model file from: {self.model_path}. Error: {e}")
            else:
                self.get_logger().error(f"Supplied model path does not exist: {self.model_path}")
        else:
            self.get_logger().warn("No 'model_path' parameter provided. Split probability outputs are deactivated.")

        # State tracking
        self.prev_best_path_coords = None  
        
        # Publishers
        self.debug_img_pub = self.create_publisher(Image, '/voronoi/debug_image', 10)
        self.angle_pub = self.create_publisher(Float32, '/voronoi/best_angle', 10)
        self.area_diff_pub = self.create_publisher(Float32, '/voronoi/area_difference', 10)
        self.best_path_pub = self.create_publisher(PoseArray, '/voronoi/best_path', 10)
        
        # New Topic: Publishes all candidate green path lines collectively
        self.candidate_paths_pub = self.create_publisher(PoseArray, '/voronoi/candidate_paths', 10)
        
        self.area_left_pub = self.create_publisher(Float32, '/voronoi/area_left', 10)
        self.area_right_pub = self.create_publisher(Float32, '/voronoi/area_right', 10)
        
        # Side-vector Publishers (midpoint + distance)
        self.side_mid_pub = self.create_publisher(Float32, '/voronoi/side_vector_mid_x', 10)
        self.side_dist_pub = self.create_publisher(Float32, '/voronoi/side_vector_distance', 10)
        
        # Split Probability Publisher
        self.split_prob_pub = self.create_publisher(Float32, '/voronoi/split_probability', 10)
        
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

    def publish_candidate_paths(self, header, paths, vor):
        pose_array = PoseArray()
        pose_array.header = header
        poses = []
        for line in paths:
            p0_cart = vor.vertices[line[0]]
            p1_cart = vor.vertices[line[1]]
            
            pose0 = Pose()
            pose0.position.x = float(p0_cart[0])
            pose0.position.y = float(p0_cart[1])
            pose0.position.z = 0.0
            
            pose1 = Pose()
            pose1.position.x = float(p1_cart[0])
            pose1.position.y = float(p1_cart[1])
            pose1.position.z = 0.0
            
            poses.extend([pose0, pose1])
        pose_array.poses = poses
        self.candidate_paths_pub.publish(pose_array)

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

            # 2. Rescale image to targeted standard dimensions
            w, h = self.resize_width, self.resize_height
            clean_mask = cv2.resize(clean_mask, (w, h))

            # 3. Apply Thresholding / Label Extraction logic
            if self.target_gray == 255 or self.target_gray <= 0:
                _, binary_mask = cv2.threshold(clean_mask, 127, 255, cv2.THRESH_BINARY)
            else:
                binary_mask = cv2.inRange(clean_mask, self.target_gray, self.target_gray)

             # 4. Extract external contours for boundaries
            contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            if not contours:
                self.publish_blank_debug_image(msg.header)
                self.publish_empty_pose_arrays(msg.header)
                # Publish 0.0 split probability on tracking loss
                prob_msg = Float32()
                prob_msg.data = 0.0
                self.split_prob_pub.publish(prob_msg)
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

            # Compute Voronoi Structure on the points
            vor = Voronoi(boundary_points_ordered)

            # Process Voronoi finite skeletal vectors using fast Point-in-Polygon checks
            skeleton_lines, dict_ridge_points = get_skeleton_lines(vor, boundary_points_ordered)

            # Compute target planning vectors (Passing binary mask and target height context)
            paths, best_angle, area_percentage_diff, best_path, side_vectors, area_left, area_right, left_lower_pt, right_lower_pt = interpreting_skeletons(
                skeleton_lines, dict_ridge_points, vor, self.prev_best_path_coords, binary_mask, h
            )

            # Retain coordinate state track history
            if best_path is not None:
                p0_cart = vor.vertices[best_path[0]]
                p1_cart = vor.vertices[best_path[1]]
                self.prev_best_path_coords = (p0_cart, p1_cart)
                
                # Publish physical coordinates directly
                self.publish_best_path_pose_array(msg.header, p0_cart, p1_cart)

            # Publish all green candidate lines to the new topic
            self.publish_candidate_paths(msg.header, paths, vor)

            # 5. Live Split Classification Inference
            self.compute_and_publish_split_probability(vor, paths, side_vectors, w, h)

            # 6. Publish Steering Control Parameters
            angle_msg = Float32()
            angle_msg.data = float(best_angle)
            self.angle_pub.publish(angle_msg)

            area_msg = Float32()
            area_msg.data = float(area_percentage_diff)
            self.area_diff_pub.publish(area_msg)

            # Publish individual areas directly
            area_left_msg = Float32()
            area_left_msg.data = float(area_left)
            self.area_left_pub.publish(area_left_msg)
            
            area_right_msg = Float32()
            area_right_msg.data = float(area_right)
            self.area_right_pub.publish(area_right_msg)

            # --- PUBLISH THE SIDE VECTOR BOTTOM-POINTS MIDPOINT AND DISTANCE ---
            side_mid_msg = Float32()
            side_dist_msg = Float32()
            if left_lower_pt is not None and right_lower_pt is not None:
                side_mid_msg.data = float((left_lower_pt[0] + right_lower_pt[0]) / 2.0)
                side_dist_msg.data = float(abs(right_lower_pt[0] - left_lower_pt[0]))
            else:
                side_mid_msg.data = -1.0
                side_dist_msg.data = -1.0
            self.side_mid_pub.publish(side_mid_msg)
            self.side_dist_pub.publish(side_dist_msg)

            # 7. Build and publish visual debug mapping (scaled to 0.5x of the target resize dimensions)
            dbg_w = int(self.resize_width * 0.5)
            dbg_h = int(self.resize_height * 0.5)
            debug_bgr = self.generate_debug_image(
                (dbg_h, dbg_w), boundary_points_ordered, skeleton_lines, paths, side_vectors, best_path, vor
            )
            self.publish_debug_image(msg.header, debug_bgr)

        except Exception as e:
            self.get_logger().error(f"Voronoi pipeline error: {str(e)}")
            # Fallback split publishing on error
            prob_msg = Float32()
            prob_msg.data = 0.0
            self.split_prob_pub.publish(prob_msg)

    def compute_and_publish_split_probability(self, vor, paths, side_vectors, w, h):
        prob_msg = Float32()
        
        # Changed: Publish -1.0 to signal the model did not load
        if self.classifier is None:
            prob_msg.data = -1.0
            self.split_prob_pub.publish(prob_msg)
            return

        try:
            # Reconstruct topological properties from paths list
            chars_paths = []
            for v0, v1 in paths:
                pt0 = vor.vertices[v0]
                pt1 = vor.vertices[v1]
                dist = get_distance(pt0, pt1)
                ang = get_angle(pt0[0], pt0[1], pt1[0], pt1[1])
                mid_x = (pt0[0] + pt1[0]) / 2.0
                chars_paths.append((dist, ang, mid_x, v0, v1))

            # Reconstruct topological properties from side vectors list
            chars_side = []
            for v0, v1 in side_vectors:
                pt0 = vor.vertices[v0]
                pt1 = vor.vertices[v1]
                dist = get_distance(pt0, pt1)
                ang = get_angle(pt0[0], pt0[1], pt1[0], pt1[1])
                mid_x = (pt0[0] + pt1[0]) / 2.0
                chars_side.append((dist, ang, mid_x, v0, v1))

            # Calculate the standard feature vector mapping
            features = extract_feature_vector(chars_paths, chars_side, [], [], [], img_w=w, img_h=h)
            
            # Predict probability using scikit-learn Pipeline
            features_reshaped = features.reshape(1, -1)
            probs = self.classifier.predict_proba(features_reshaped)[0]
            
            prob_msg.data = float(probs[1])
        except Exception as e:
            self.get_logger().error(f"Feature extraction or classification failure: {e}")
            prob_msg.data = -1.0 # Signal prediction failure

        self.split_prob_pub.publish(prob_msg)

    def generate_debug_image(self, shape, boundary_points, skeleton, paths, side_vectors, best_path, vor):
        h, w = shape
        canvas = np.zeros((h, w, 3), dtype=np.uint8)

        scale_x = w / float(self.resize_width)
        scale_y = h / float(self.resize_height)

        def to_img_space(pt):
            return int(pt[0] * scale_x), int(h - (pt[1] * scale_y))

        line_thick = max(1, int(2 * 0.5))      
        best_line_thick = max(1, int(4 * 0.5)) 
        circle_rad = max(2, int(6 * 0.5))      

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
        pose1.position.x = float(p1_cart[0])
        pose1.position.y = float(p1_cart[1])
        pose1.position.z = 0.0

        pose_array.poses = [pose0, pose1]
        self.best_path_pub.publish(pose_array)

    def publish_empty_pose_arrays(self, header):
        pose_array = PoseArray()
        pose_array.header = header
        pose_array.poses = []
        self.best_path_pub.publish(pose_array)
        self.candidate_paths_pub.publish(pose_array)


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