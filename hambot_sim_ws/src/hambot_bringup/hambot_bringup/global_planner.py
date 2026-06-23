#!/usr/bin/env python3
import os
import math
import heapq
import re
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# =====================================================================
# GEOMETRIC COMPILING FUNCTIONS & INTERSECTION TYPES
# =====================================================================
def calculate_relative_angle(prev_node, curr_node, next_node):
    v_in_x = curr_node.x - prev_node.x
    v_in_y = curr_node.y - prev_node.y
    v_out_x = next_node.x - curr_node.x
    v_out_y = next_node.y - curr_node.y
    
    angle_in = math.atan2(v_in_y, v_in_x)
    angle_out = math.atan2(v_out_y, v_out_x)
    angle_diff = angle_out - angle_in
    
    while angle_diff > math.pi: 
        angle_diff -= 2 * math.pi
    while angle_diff <= -math.pi: 
        angle_diff += 2 * math.pi
        
    return math.degrees(angle_diff)

def assign_directions_dynamically(nb_angles):
    left_nodes = []
    straight_nodes = []
    right_nodes = []
    
    for angle, node in nb_angles:
        if -30 <= angle <= 30:
            straight_nodes.append((angle, node))
        elif 30 < angle < 150:
            left_nodes.append((angle, node))
        elif -150 < angle < -30:
            right_nodes.append((angle, node))
        else:
            if angle >= 150: 
                left_nodes.append((angle, node))
            else: 
                right_nodes.append((angle, node))
                
    has_large_left = any(angle >= 60 for angle, _ in left_nodes)
    if has_large_left:
        to_move = [item for item in left_nodes if item[0] < 60]
        left_nodes = [item for item in left_nodes if item[0] >= 60]
        straight_nodes.extend(to_move)
        
    has_large_right = any(angle <= -60 for angle, _ in right_nodes)
    if has_large_right:
        to_move = [item for item in right_nodes if item[0] > -60]
        right_nodes = [item for item in right_nodes if item[0] <= -60]
        straight_nodes.extend(to_move)
        
    dirs = {}
    if len(left_nodes) == 1: 
        dirs['Left'] = left_nodes[0][1]
    elif len(left_nodes) > 1:
        left_nodes.sort(key=lambda x: x[0], reverse=True)
        for idx, (_, node) in enumerate(left_nodes, 1): 
            dirs[f'Left {idx}'] = node
            
    if len(right_nodes) == 1: 
        dirs['Right'] = right_nodes[0][1]
    elif len(right_nodes) > 1:
        right_nodes.sort(key=lambda x: x[0])
        for idx, (_, node) in enumerate(right_nodes, 1): 
            dirs[f'Right {idx}'] = node
            
    if len(straight_nodes) == 1: 
        dirs['Straight'] = straight_nodes[0][1]
    elif len(straight_nodes) > 1:
        straight_nodes.sort(key=lambda x: x[0], reverse=True)
        for idx, (_, node) in enumerate(straight_nodes, 1): 
            dirs[f'Straight {idx}'] = node
    return dirs

def get_intersection_type(dir_map):
    has_left = any(k.startswith('Left') for k in dir_map)
    has_straight = any(k.startswith('Straight') for k in dir_map)
    has_right = any(k.startswith('Right') for k in dir_map)
    
    if has_left and has_straight and has_right:
        return cross()
    elif has_left and not has_straight and has_right:
        return T()
    elif has_left and has_straight and not has_right:
        return left()
    elif not has_left and has_straight and has_right:
        return right()
    else:
        return no_intersection()

class Intersection:
    def __init__(self, left, straight, right):
        self.left = left
        self.straight = straight
        self.right = right

class no_intersection(Intersection):
    def __init__(self):
        super().__init__(False, False, False)

class cross(Intersection):
    def __init__(self):
        super().__init__(True, True, True)

class T(Intersection):
    def __init__(self):
        super().__init__(True, False, True)

class left(Intersection):
    def __init__(self):
        super().__init__(True, True, False)

class right(Intersection):
    def __init__(self):
        super().__init__(False, True, True)

# =====================================================================
# CORE DATA STRUCTURES
# =====================================================================
class NodeClass:
    def __init__(self, name, x, y, nodes=None):
        self.name = name
        self.x = x
        self.y = y
        self.nodes = nodes if nodes is not None else []

# Map alias to comply with sdftodataStruc.py structure cleanly
NodeObj = NodeClass

class Edges:
    def __init__(self, node1, node2, distance):
        self.node1 = node1
        self.node2 = node2
        self.dist = distance
        self.type_intersection1 = no_intersection() 
        self.type_intersection2 = no_intersection()
        self.directions = {node1.name: {}, node2.name: {}}

class Map:
    def __init__(self):
        self.nodes_amount = 0
        self.all_nodes = []
        self.edges = dict()

    def add_node(self, x, y, nodes=None):
        node = NodeObj(f"{self.nodes_amount}", x, y, nodes)
        self.all_nodes.append(node)
        self.nodes_amount += 1
        return node

    def change_node(self, node, node_to_be_added):
        self.all_nodes[int(node.name)].nodes.append(node_to_be_added)

    def add_segment(self, node1: NodeObj, node2: NodeObj, distance: float = None):
        idx1 = int(node1.name)
        while len(self.all_nodes) <= idx1:
            self.all_nodes.append(None)
        if self.all_nodes[idx1] is None:
            self.all_nodes[idx1] = node1
            self.nodes_amount = max(self.nodes_amount, idx1 + 1)

        if node2 not in node1.nodes:
            self.change_node(node1, node2)

        idx2 = int(node2.name)
        while len(self.all_nodes) <= idx2:
            self.all_nodes.append(None)
        if self.all_nodes[idx2] is None:
            self.all_nodes[idx2] = node2
            self.nodes_amount = max(self.nodes_amount, idx2 + 1)

        if node1 not in node2.nodes:
            self.change_node(node2, node1)

        calculated_dist = math.dist((node1.x, node1.y), (node2.x, node2.y))
        self.edges[node1, node2] = Edges(node1, node2, calculated_dist)

    def compile_map(self):
        for edge in list(self.edges.values()):
            n1, n2 = edge.node1, edge.node2
            
            edge.directions[n1.name] = {}
            edge.directions[n2.name] = {}
            
            neighbors1 = [nb for nb in n1.nodes if nb != n2]
            if neighbors1:
                nb_angles = [(calculate_relative_angle(n2, n1, nb), nb) for nb in neighbors1]
                edge.directions[n1.name] = assign_directions_dynamically(nb_angles)
                    
            neighbors2 = [nb for nb in n2.nodes if nb != n1]
            if neighbors2:
                nb_angles = [(calculate_relative_angle(n1, n2, nb), nb) for nb in neighbors2]
                edge.directions[n2.name] = assign_directions_dynamically(nb_angles)
            
            edge.type_intersection1 = get_intersection_type(edge.directions[n1.name])
            edge.type_intersection2 = get_intersection_type(edge.directions[n2.name])

# =====================================================================
# GEOMETRIC UTILITIES & SDF PARSERS (SDF TO DATA STRUCTURE LOGIC)
# =====================================================================
def point_to_segment_distance_and_projection(px, py, ax, ay, bx, by):
    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay
    
    ab_len_sq = abx**2 + aby**2
    if ab_len_sq == 0:
        return math.hypot(px - ax, py - ay), 0.0
        
    t = (apx * abx + apy * aby) / ab_len_sq
    t_clamped = max(0.0, min(1.0, t))
    
    proj_x = ax + t_clamped * abx
    proj_y = ay + t_clamped * aby
    
    dist = math.hypot(px - proj_x, py - proj_y)
    return dist, t_clamped

def extract_node_id(frame_name, existing_ids):
    match = re.search(r'\d+', frame_name)
    if match:
        val = int(match.group())
        if val in existing_ids:
            new_val = max(existing_ids) + 1
            return new_val
        return val
    new_val = max(existing_ids) + 1 if existing_ids else 0
    return new_val

def parse_sdf(sdf_str):
    root = ET.fromstring(sdf_str)
    world = root.find('world')
    
    # 1. Extract raw Frames (Waypoints)
    frames = []
    for frame in world.findall('frame'):
        name = frame.attrib['name']
        pose_text = frame.find('pose').text.strip()
        parts = [float(val) for val in pose_text.split()]
        x, y = parts[0], parts[1]
        frames.append({'name': name, 'x': x, 'y': y})
        
    # 2. Extract Raw Sidewalk Links
    segments = []
    sidewalk_model = None
    for model in world.findall('model'):
        if model.attrib.get('name') == 'sidewalk_network':
            sidewalk_model = model
            break
            
    if sidewalk_model is not None:
        for link in sidewalk_model.findall('link'):
            link_name = link.attrib['name']
            pose_elem = link.find('pose')
            pose_parts = [float(v) for v in pose_elem.text.strip().split()]
            link_x, link_y, _, _, _, yaw = pose_parts
            
            collision = link.find('collision')
            box_geom = collision.find('geometry').find('box')
            size_parts = [float(v) for v in box_geom.find('size').text.strip().split()]
            length, width, height = size_parts
            
            ax, ay = link_x, link_y
            bx = link_x + length * math.cos(yaw)
            by = link_y + length * math.sin(yaw)
            
            segments.append({
                'name': link_name,
                'ax': ax, 'ay': ay,
                'bx': bx, 'by': by,
                'length': length,
                'width': width
            })
            
    return frames, segments

def clean_and_map_nodes(frames):
    cleaned_frames = []
    name_mapping = {}
    
    for f in frames:
        merged = False
        for cf in cleaned_frames:
            dist = math.hypot(f['x'] - cf['x'], f['y'] - cf['y'])
            if dist < 1.0:
                name_mapping[f['name']] = cf['name']
                merged = True
                break
        if not merged:
            cleaned_frames.append(f)
            name_mapping[f['name']] = f['name']
            
    existing_ids = set()
    node_id_map = {}
    node_coords = {}
    
    for cf in cleaned_frames:
        n_id = extract_node_id(cf['name'], existing_ids)
        existing_ids.add(n_id)
        node_id_map[cf['name']] = n_id
        node_coords[n_id] = (cf['x'], cf['y'], cf['name'])
        
    for orig_name, target_name in name_mapping.items():
        if orig_name not in node_id_map:
            target_id = node_id_map[target_name]
            node_id_map[orig_name] = target_id
            
    return node_id_map, node_coords

def associate_nodes_to_segments(node_coords, segments):
    segment_nodes = {seg['name']: [] for seg in segments}
    
    for n_id, (nx, ny, name) in node_coords.items():
        for seg in segments:
            dist, t = point_to_segment_distance_and_projection(
                nx, ny, seg['ax'], seg['ay'], seg['bx'], seg['by']
            )
            threshold = (seg['width'] / 2.0) + 0.3 # 0.9m
            if dist <= threshold:
                segment_nodes[seg['name']].append((t, n_id))
                
    return segment_nodes

def populate_map_from_sdf(my_map, node_coords, segment_nodes, extra_connections=None):
    node_objects = {}
    for n_id, (x, y, original_name) in node_coords.items():
        node = NodeObj(str(n_id), x, y)
        node_objects[n_id] = node
        
        idx = int(node.name)
        while len(my_map.all_nodes) <= idx:
            my_map.all_nodes.append(None)
        my_map.all_nodes[idx] = node
        my_map.nodes_amount = max(my_map.nodes_amount, idx + 1)
        
    for seg_name, nodes_list in segment_nodes.items():
        if len(nodes_list) < 2:
            continue
        nodes_list.sort(key=lambda x: x[0])
        
        for i in range(len(nodes_list) - 1):
            n1_id = nodes_list[i][1]
            n2_id = nodes_list[i+1][1]
            node1 = node_objects[n1_id]
            node2 = node_objects[n2_id]
            
            if node1 != node2:
                my_map.add_segment(node1, node2)
                
    if extra_connections:
        for n1_id, n2_id in extra_connections:
            if n1_id in node_objects and n2_id in node_objects:
                node1 = node_objects[n1_id]
                node2 = node_objects[n2_id]
                my_map.add_segment(node1, node2)


# =====================================================================
# DIJKSTRA ROUTING MECHANIC
# =====================================================================
def dijkstra(road_map, start_node, end_node):
    distances = {node: float('inf') for node in road_map.all_nodes if node is not None}
    distances[start_node] = 0
    pq = [(0, start_node.name, start_node)]
    previous_nodes = {node: None for node in road_map.all_nodes if node is not None}
    
    while pq:
        current_distance, _, current_node = heapq.heappop(pq)
        if current_node == end_node: 
            break
        if current_distance > distances[current_node]: 
            continue
            
        for neighbor in current_node.nodes:
            edge = road_map.edges.get((current_node, neighbor)) or road_map.edges.get((neighbor, current_node))
            if edge is None: 
                continue
            
            distance = current_distance + edge.dist
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(pq, (distance, neighbor.name, neighbor))
                
    path = []
    current = end_node
    while current is not None:
        path.append(current)
        current = previous_nodes[current]
    path.reverse()
    return (path, distances[end_node]) if distances[end_node] != float('inf') else (None, float('inf'))


# =====================================================================
# GLOBAL COORDINATOR NODE
# =====================================================================
class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('global_planner')
        
        self.declare_parameter('world_name', 'campus_sidewalk')
        self.world_name = self.get_parameter('world_name').value

        self.start_node_str = self.declare_flexible_node_parameter('start_node', 0)
        self.end_node_str = self.declare_flexible_node_parameter('end_node', 19)
        
        # Build Map Structure (Calculated strictly once during setup)
        self.map = Map()
        self.load_sdf_geometry()
        
        # Compute Path (Calculated strictly once during setup)
        self.path = []
        self.turn_sequence = []  
        self.current_turn_index = 0
        self.compute_global_plan()
        
        # ROS Communications
        self.target_dir_pub = self.create_publisher(String, '/global_planner/target_direction', 10)
        self.dest_type_pub = self.create_publisher(String, '/global_planner/destination_type', 10)
        self.feedback_sub = self.create_subscription(
            String, '/global_planner/turn_completed', self.feedback_callback, 10
        )
        
        # Dispatch the first target layout immediately on startup
        self.publish_current_target()
        
        # Background heartbeat safety timer (Reduced to 3.0s to minimize CPU load & DDS traffic)
        self.timer = self.create_timer(3.0, self.publish_current_target)
        self.get_logger().info("Event-driven high-efficiency global planner initialized.")

    def declare_flexible_node_parameter(self, name, default_val_int):
        try:
            self.declare_parameter(name, int(default_val_int))
            val = self.get_parameter(name).value
            return str(val)
        except Exception:
            try:
                self.declare_parameter(name, str(default_val_int))
                val = self.get_parameter(name).value
                return str(val)
            except Exception:
                return str(default_val_int)

    def load_sdf_geometry(self):
        pkg_bringup = get_package_share_directory('hambot_bringup')
        world_path = os.path.join(pkg_bringup, 'worlds', f"{self.world_name}.sdf")
        
        if not os.path.exists(world_path):
            self.get_logger().error(f"SDF World path does not exist: {world_path}")
            return
            
        try:
            with open(world_path, 'r', encoding='utf-8') as f:
                sdf_string = f.read()
        except Exception as e:
            self.get_logger().error(f"Failed to read SDF file: {str(e)}")
            return
            
        # Compile layout frames, links, projection configurations and duplicates
        frames, segments = parse_sdf(sdf_string)
        node_id_map, node_coords = clean_and_map_nodes(frames)
        segment_nodes = associate_nodes_to_segments(node_coords, segments)
        
        # Geometrically forced connections for curvilinear boundaries
        edge_overrides = [(10, 2), (4, 11), (11, 12)]
        
        # Build map with connection overrides and dynamic calculations
        populate_map_from_sdf(self.map, node_coords, segment_nodes, extra_connections=edge_overrides)
        self.map.compile_map()
        
        self.get_logger().info(f"SDF compiled using projection pipeline. Created {len([n for n in self.map.all_nodes if n])} topological nodes.")

    def compute_global_plan(self):
        try:
            start_node_obj = self.map.all_nodes[int(self.start_node_str)]
            end_node_obj = self.map.all_nodes[int(self.end_node_str)]
        except (ValueError, IndexError):
            self.get_logger().error("Routing failed: Node IDs invalid or non-existent in the SDF structure.")
            return
            
        path, dist = dijkstra(self.map, start_node_obj, end_node_obj)
        if not path:
            self.get_logger().error(f"No path could be calculated between Node {self.start_node_str} and Node {self.end_node_str}.")
            return
            
        self.path = path
        path_names = [n.name for n in path]
        self.get_logger().info(f"Dijkstra computed global trajectory: {path_names} (Distance: {dist:.2f}m)")
        
        self.turn_sequence = []
        for i in range(1, len(path) - 1):
            prev_node = path[i-1]
            current_node = path[i]
            next_node = path[i+1]
            
            edge = self.map.edges.get((prev_node, current_node)) or self.map.edges.get((current_node, prev_node))
            if edge is not None:
                dir_map = edge.directions.get(current_node.name, {})
                selected_turn = 'straight'
                for direction, target_node in dir_map.items():
                    if target_node == next_node:
                        direction_lower = direction.lower()
                        if 'left' in direction_lower: 
                            selected_turn = 'left'
                        elif 'right' in direction_lower: 
                            selected_turn = 'right'
                        else: 
                            selected_turn = 'straight'
                        break
                self.turn_sequence.append((current_node.name, selected_turn))
            else:
                self.turn_sequence.append((current_node.name, 'straight'))
                
        self.get_logger().info(f"Sequence of intermediate relative turn decisions: {self.turn_sequence}")

    def feedback_callback(self, msg: String):
        if msg.data == "reached_destination":
            self.get_logger().info("COORDINATION_UPDATE: Destination reached by navigator. Shutting down global planner.")
            rclpy.shutdown()
            return

        if msg.data == "completed":
            if self.current_turn_index < len(self.turn_sequence):
                node_name, turn_dir = self.turn_sequence[self.current_turn_index]
                self.get_logger().info(f"COORDINATION_UPDATE: Split action '{turn_dir.upper()}' at Node {node_name} completed.")
                
                self.current_turn_index += 1
                self.publish_current_target()
                
                if self.current_turn_index < len(self.turn_sequence):
                    next_node, next_dir = self.turn_sequence[self.current_turn_index]
                    self.get_logger().info(f"COORDINATION_UPDATE: Advancing plan. Target is now: '{next_dir.upper()}' for Node {next_node}.")
                else:
                    self.get_logger().info("COORDINATION_UPDATE: Final turn cleared. Direct corridor path remaining to destination.")
            else:
                self.get_logger().warn("COORDINATION_UPDATE: Received clearance signal, but no pending tasks are remaining.")

    def publish_current_target(self):
        msg = String()
        dest_type_msg = String()
        
        if not self.path or len(self.path) < 2:
            msg.data = "stop"
            self.target_dir_pub.publish(msg)
            dest_type_msg.data = "none"
            self.dest_type_pub.publish(dest_type_msg)
            return

        is_final_edge = (self.current_turn_index >= len(self.turn_sequence))

        # Classify the destination target node type
        if is_final_edge:
            end_node_obj = self.path[-1]
            if len(end_node_obj.nodes) <= 1:
                dest_type_msg.data = "end_node"
            else:
                dest_type_msg.data = "split_node"
        else:
            dest_type_msg.data = "none"

        if self.current_turn_index < len(self.turn_sequence):
            _, turn_dir = self.turn_sequence[self.current_turn_index]
            msg.data = turn_dir
            
            k = self.current_turn_index
            if k < len(self.path) - 1:
                node_a = self.path[k]
                node_b = self.path[k+1]
                self.get_logger().info(
                    f"TRACKER: Robot is traversing edge {node_a.name} <-> {node_b.name}. Next turn decision at Node {node_b.name} is '{turn_dir.upper()}'",
                    throttle_duration_sec=5.0
                )
        else:
            node_a = self.path[-2]
            node_b = self.path[-1]
            self.get_logger().info(
                f"TRACKER: Robot is on the final straight section {node_a.name} <-> {node_b.name} approaching target.",
                throttle_duration_sec=5.0
            )
            msg.data = "straight"
            
        self.target_dir_pub.publish(msg)
        self.dest_type_pub.publish(dest_type_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
    
'''
ros2 launch hambot_bringup voronoi_launch.py \
  world_name:=the_map \
  start_point:=1 \
  start_node:=0 \
  end_node:=19
'''