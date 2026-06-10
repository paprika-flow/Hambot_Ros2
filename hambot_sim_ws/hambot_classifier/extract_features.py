# extract_features.py
import os
import cv2
import math
import numpy as np
from pathlib import Path
from collections import defaultdict
from scipy.spatial import Voronoi
from shapely.geometry import Polygon
import concurrent.futures
from tqdm import tqdm

# =====================================================================
# GEOMETRIC MATH & SKELETONIZATION LOGIC (Identical to ROS 2 Node)
# =====================================================================

def get_midpoint(p1, p2):
    return [(p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2]

def get_distance(p1, p2):
    return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

def get_angle(x1, y1, x2, y2):
    return math.atan2(y2 - y1, x2 - x1) * 180 / math.pi

def is_point_in_polygon(pt, poly_pts):
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
    return (v0, v1)

def is_line_on_sidewalk(p0, p1, binary_mask, h):
    x0, y0 = int(round(p0[0])), int(round(h - p0[1]))
    x1, y1 = int(round(p1[0])), int(round(h - p1[1]))
    num_samples = max(abs(x1 - x0), abs(y1 - y0), 2) * 2
    x_coords = np.linspace(x0, x1, num_samples)
    y_coords = np.linspace(y0, y1, num_samples)
    mask_h, mask_w = binary_mask.shape
    for x, y in zip(x_coords, y_coords):
        col, row = int(round(x)), int(round(y))
        if 0 <= col < mask_w and 0 <= row < mask_h:
            if binary_mask[row, col] == 0:
                return False
        else:
            return False
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
                if v1_x - (320 * range_val / 960) > resolved_rv_x:
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
    sorted_lines = sorted(lines, key=lambda l: get_midpoint(vor.vertices[l[0]], vor.vertices[l[1]])[0])
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
        leftmost_line, _ = min(left_points, key=lambda lv: vor.vertices[lv[1]][1])
    if not right_points:
        rightmost_line = sorted_lines[-1]
    else:
        rightmost_line, _ = min(right_points, key=lambda lv: vor.vertices[lv[1]][1])
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
        if not is_point_in_polygon(v0, poly_pts) or not is_point_in_polygon(v1, poly_pts):
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
        if not is_point_in_polygon(v0, poly_pts) or not is_point_in_polygon(v1, poly_pts):
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
    side_vectors = [left_lowest_line, right_lowest_line]
    area_left, side_vectors = get_vectors_area(left_lowest_line, skeleton_lines, dict_ridge_points, vor, side_vectors, False, binary_mask, h, range_val)
    area_right, side_vectors = get_vectors_area(right_lowest_line, skeleton_lines, dict_ridge_points, vor, side_vectors, True, binary_mask, h, range_val)
    area_difference = area_left - area_right
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
    return path_lines, best_angle, area_difference, best_possible_path, side_vectors, area_left, area_right, left_lower_pt, right_lower_pt

# =====================================================================
# FEATURE VECTOR EXTRACTION
# =====================================================================

def extract_feature_vector(chars_paths, chars_side_vectors, dens_l, dens_r, dens_c, img_w=960, img_h=720, eps=1e-6):
    features = []
    num_paths = len(chars_paths)
    num_side = len(chars_side_vectors)
    features.append(float(num_paths))
    features.append(float(num_side))

    if num_paths > 0:
        dists = np.array([p[0] for p in chars_paths], dtype=np.float64)
        angs  = np.array([p[1] for p in chars_paths], dtype=np.float64)
        midx  = np.array([p[2] for p in chars_paths], dtype=np.float64)
        dl_only = np.array(dens_l, dtype=np.float64)
        dr_only = np.array(dens_r, dtype=np.float64)
        dc_only = np.array(dens_c, dtype=np.float64)

        dist_n = dists / float(img_h)
        x_n = (midx - img_w/2) / (img_w/2)

        features.extend([
            float(np.mean(dist_n)),
            float(np.max(dist_n)),
            float(np.std(dist_n)) if num_paths > 1 else 0.0
        ])
        features.extend([
            float(np.mean(x_n)),
            float(np.std(x_n)) if num_paths > 1 else 0.0,
            float(np.max(x_n) - np.min(x_n)) if num_paths > 1 else 0.0
        ])

        ang_rad = np.deg2rad(angs)
        sin_mean = np.mean(np.sin(ang_rad))
        cos_mean = np.mean(np.cos(ang_rad))
        avg_ang = np.rad2deg(np.arctan2(sin_mean, cos_mean))
        features.append(float(avg_ang))
        features.append(float(np.std(angs)) if num_paths > 1 else 0.0)

        features.extend([
            float(np.mean(dl_only)) if len(dl_only) > 0 else 0.0,
            float(np.mean(dr_only)) if len(dr_only) > 0 else 0.0,
            float(np.mean(dc_only)) if len(dc_only) > 0 else 0.0
        ])

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

    if num_side > 0:
        side_ds = np.array([s[0] for s in chars_side_vectors], dtype=np.float64)
        features.append(float(np.mean(side_ds) / img_h))
        features.append(float(np.max(side_ds) / img_h))
    else:
        features.extend([0.0, 0.0])

    return np.asarray(features, dtype=np.float32)

# =====================================================================
# DATASET GENERATION WORKER
# =====================================================================

_G = {}

def _init_worker(target_gray, target_size):
    global _G
    _G["target_gray"] = target_gray
    _G["target_size"] = target_size

def process_single_image(file_info):
    path = file_info['path']
    label = file_info['label']
    
    # Read the segmentation mask
    raw_img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if raw_img is None:
        return None
    
    # Resize to the planning node target resolution (960 x 720)
    w, h = _G["target_size"]
    clean_mask = cv2.resize(raw_img, (w, h))

    # Match the thresholding logic of the live node
    target_gray = _G["target_gray"]
    if target_gray == 255:
        _, binary_mask = cv2.threshold(clean_mask, 127, 255, cv2.THRESH_BINARY)
    else:
        binary_mask = cv2.inRange(clean_mask, target_gray, target_gray)

    try:
        contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not contours:
            return None

        contour = max(contours, key=cv2.contourArea)
        boundary_points = contour.squeeze()
        
        if boundary_points.ndim != 2 or len(boundary_points) < 4:
            return None

        boundary_points_cartesian = boundary_points.copy()
        boundary_points_cartesian[:, 1] = h - boundary_points_cartesian[:, 1]

        num_pts = len(boundary_points_cartesian)
        step = min(50, max(1, num_pts // 4))
        boundary_points_ordered = boundary_points_cartesian[::step]
        if len(boundary_points_ordered) < 4:
            boundary_points_ordered = boundary_points_cartesian

        vor = Voronoi(boundary_points_ordered)
        skeleton_lines, dict_ridge_points = get_skeleton_lines(vor, boundary_points_ordered)

        # Extract planning lines (Passing None to simulate start of path tracking)
        paths, best_angle, area_percentage_diff, best_path, side_vectors, area_left, area_right, left_lower_pt, right_lower_pt = interpreting_skeletons(
            skeleton_lines, dict_ridge_points, vor, None, binary_mask, h
        )

        # Map geometries to numerical properties
        chars_paths = []
        for v0, v1 in paths:
            pt0 = vor.vertices[v0]
            pt1 = vor.vertices[v1]
            dist = get_distance(pt0, pt1)
            ang = get_angle(pt0[0], pt0[1], pt1[0], pt1[1])
            mid_x = (pt0[0] + pt1[0]) / 2.0
            chars_paths.append((dist, ang, mid_x, v0, v1))

        chars_side = []
        for v0, v1 in side_vectors:
            pt0 = vor.vertices[v0]
            pt1 = vor.vertices[v1]
            dist = get_distance(pt0, pt1)
            ang = get_angle(pt0[0], pt0[1], pt1[0], pt1[1])
            mid_x = (pt0[0] + pt1[0]) / 2.0
            chars_side.append((dist, ang, mid_x, v0, v1))

        # Extract features (leaving density fields as empty list fallback matches ROS output)
        x_voronoi = extract_feature_vector(chars_paths, chars_side, [], [], [], img_w=w, img_h=h)
        
        if x_voronoi is not None:
            return {
                'x_voronoi': x_voronoi,
                'y': label,
                'file': Path(path).name
            }
    except Exception as e:
        pass
    
    return None

def build_dataset(straight_dir, split_dir, limit_per_class=1000):
    tasks = []
    
    # Class 0: Straights
    straight_files = sorted(list(Path(straight_dir).glob("*.png")))
    for f in straight_files[:limit_per_class]:
        tasks.append({'path': f, 'label': 0})
        
    # Class 1: Splits
    split_files = sorted(list(Path(split_dir).glob("*.png")))
    for f in split_files[:limit_per_class]:
        tasks.append({'path': f, 'label': 1})
    
    X, y, filenames = [], [], []
    
    print(f"Dataset summary: Found {len(tasks)} files total.")

    with concurrent.futures.ProcessPoolExecutor(
        max_workers=os.cpu_count(),
        initializer=_init_worker,
        initargs=(15, (960, 720)), # Target gray intensity 255 (binary mask), Target size 960x720
    ) as executor:
        for res in tqdm(executor.map(process_single_image, tasks), total=len(tasks), desc="Processing Voronoi Topology"):
            if res is not None:
                X.append(res['x_voronoi'])
                y.append(res['y'])
                filenames.append(res['file'])
                
    return np.array(X), np.array(y), filenames

if __name__ == "__main__":
    # Configure your dataset mask directory paths here
    STRAIGHT_DIR = "/home/ubuntu/hambot_sim_ws/processed_rosmaster_photos/mask/no_split"
    SPLIT_DIR = "/home/ubuntu/hambot_sim_ws/processed_rosmaster_photos_splits/processed_rosmaster_photos_splits/mask"
    
    X, y, filenames = build_dataset(STRAIGHT_DIR, SPLIT_DIR)
    
    np.save('X_voronoi.npy', X)
    np.save('y_local.npy', y)
    print(f"Extracted feature matrix: {X.shape}, Label vector: {y.shape}")