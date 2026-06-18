#!/usr/bin/env python3
import os
import math
import heapq
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# =====================================================================
# GEOMETRIC COMPILING FUNCTIONS & CLASS WRAPPERS
# =====================================================================
def calculate_relative_angle(prev_node, curr_node, next_node):
    v_in_x = curr_node.x - prev_node.x
    v_in_y = curr_node.y - prev_node.y
    v_out_x = next_node.x - curr_node.x
    v_out_y = next_node.y - curr_node.y
    
    angle_in = math.atan2(v_in_y, v_in_x)
    angle_out = math.atan2(v_out_y, v_out_x)
    angle_diff = angle_out - angle_in
    
    while angle_diff > math.pi: angle_diff -= 2 * math.pi
    while angle_diff <= -math.pi: angle_diff += 2 * math.pi
    return math.degrees(angle_diff)

def assign_directions_dynamically(nb_angles):
    left_nodes, straight_nodes, right_nodes = [], [], []
    for angle, node in nb_angles:
        if -30 <= angle <= 30:
            straight_nodes.append((angle, node))
        elif 30 < angle < 150:
            left_nodes.append((angle, node))
        elif -150 < angle < -30:
            right_nodes.append((angle, node))
        else:
            if angle >= 150: left_nodes.append((angle, node))
            else: right_nodes.append((angle, node))
                
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
    if len(left_nodes) == 1: dirs['Left'] = left_nodes[0][1]
    elif len(left_nodes) > 1:
        left_nodes.sort(key=lambda x: x[0], reverse=True)
        for idx, (_, node) in enumerate(left_nodes, 1): dirs[f'Left {idx}'] = node
            
    if len(right_nodes) == 1: dirs['Right'] = right_nodes[0][1]
    elif len(right_nodes) > 1:
        right_nodes.sort(key=lambda x: x[0])
        for idx, (_, node) in enumerate(right_nodes, 1): dirs[f'Right {idx}'] = node
            
    if len(straight_nodes) == 1: dirs['Straight'] = straight_nodes[0][1]
    elif len(straight_nodes) > 1:
        straight_nodes.sort(key=lambda x: x[0], reverse=True)
        for idx, (_, node) in enumerate(straight_nodes, 1): dirs[f'Straight {idx}'] = node
    return dirs

class NodeClass:
    def __init__(self, name, x, y, nodes=None):
        self.name = name
        self.x = x
        self.y = y
        self.nodes = nodes if nodes is not None else []

class Edges:
    def __init__(self, node1, node2, distance):
        self.node1 = node1
        self.node2 = node2
        self.dist = distance
        self.directions = {node1.name: {}, node2.name: {}}

class Map:
    def __init__(self):
        self.nodes_amount = 0
        self.all_nodes = []
        self.edges = dict()

    def add_node(self, name, x, y):
        idx = int(name)
        while len(self.all_nodes) <= idx:
            self.all_nodes.append(None)
        node = NodeClass(name, x, y)
        self.all_nodes[idx] = node
        self.nodes_amount = max(self.nodes_amount, idx + 1)
        return node

    def add_segment(self, node1, node2):
        if node2 not in node1.nodes: node1.nodes.append(node2)
        if node1 not in node2.nodes: node2.nodes.append(node1)
        dist = math.dist((node1.x, node1.y), (node2.x, node2.y))
        self.edges[node1, node2] = Edges(node1, node2, dist)

    def compile_map(self):
        for edge in self.edges.values():
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
        if current_node == end_node: break
        if current_distance > distances[current_node]: continue
            
        for neighbor in current_node.nodes:
            edge = road_map.edges.get((current_node, neighbor)) or road_map.edges.get((neighbor, current_node))
            if edge is None: continue
            
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
        self.declare_parameter('start_node', '0')
        self.declare_parameter('end_node', '19')
        
        self.world_name = self.get_parameter('world_name').value
        self.start_node_str = self.get_parameter('start_node').value
        self.end_node_str = self.get_parameter('end_node').value
        
        # Build Map Structure Dynamically from SDF
        self.map = Map()
        self.load_sdf_geometry()
        
        # Compute Path
        self.path = []
        self.turn_sequence = []  # List of tuples: (at_node_name, direction_to_take)
        self.current_turn_index = 0
        self.compute_global_plan()
        
        # ROS Communications
        self.target_dir_pub = self.create_publisher(String, '/global_planner/target_direction', 10)
        self.feedback_sub = self.create_subscription(
            String, '/global_planner/turn_completed', self.feedback_callback, 10
        )
        
        # Start repeat action timer to ensure robust dynamic reception
        self.timer = self.create_timer(1.0, self.publish_current_target)
        self.get_logger().info("Global topological pathplanner coordinator online.")

    def load_sdf_geometry(self):
        pkg_bringup = get_package_share_directory('hambot_bringup')
        world_path = os.path.join(pkg_bringup, 'worlds', f"{self.world_name}.sdf")
        
        if not os.path.exists(world_path):
            self.get_logger().error(f"SDF World path does not exist: {world_path}")
            return
            
        tree = ET.parse(world_path)
        root = tree.getroot()
        world = root.find('world')
        
        # 1. Parse Waypoints
        raw_frames = []
        for frame in world.findall('frame'):
            name = frame.attrib['name']
            pose_text = frame.find('pose').text.strip().split()
            x, y = float(pose_text[0]), float(pose_text[1])
            raw_frames.append({'name': name, 'x': x, 'y': y})
            
        # 2. Cleanup Spatially Duplicate Waypoints
        cleaned_frames = []
        for f in raw_frames:
            merged = False
            for cf in cleaned_frames:
                if math.hypot(f['x'] - cf['x'], f['y'] - cf['y']) < 1.0:
                    merged = True
                    break
            if not merged:
                cleaned_frames.append(f)
                
        # 3. Add Unique Nodes to Graph
        node_id_mapping = {}
        import re
        for f in cleaned_frames:
            match = re.search(r'\d+', f['name'])
            idx_str = match.group() if match else f['name']
            node_id_mapping[f['name']] = idx_str
            self.map.add_node(idx_str, f['x'], f['y'])
            
        # 4. Extract Sidewalk Corridor Links
        sidewalk_model = None
        for model in world.findall('model'):
            if model.attrib.get('name') == 'sidewalk_network':
                sidewalk_model = model
                break
                
        if sidewalk_model is None:
            self.get_logger().error("Model 'sidewalk_network' not found inside the specified SDF world.")
            return
            
        segments = []
        for link in sidewalk_model.findall('link'):
            link_name = link.attrib['name']
            pose_parts = [float(v) for v in link.find('pose').text.strip().split()]
            lx, ly, _, _, _, yaw = pose_parts
            
            size_parts = [float(v) for v in link.find('collision').find('geometry').find('box').find('size').text.strip().split()]
            length, width = size_parts[0], size_parts[1]
            
            ax, ay = lx, ly
            bx = lx + length * math.cos(yaw)
            by = ly + length * math.sin(yaw)
            
            segments.append({'name': link_name, 'ax': ax, 'ay': ay, 'bx': bx, 'by': by, 'width': width})
            
        # 5. Project Nodes onto Corridor Geometry to build Interconnections
        segment_nodes_map = {seg['name']: [] for seg in segments}
        for node in self.map.all_nodes:
            if node is None: continue
            for seg in segments:
                # Segment Projection
                abx, aby = seg['bx'] - seg['ax'], seg['by'] - seg['ay']
                apx, apy = node.x - seg['ax'], node.y - seg['ay']
                ab_len_sq = abx**2 + aby**2
                
                t = (apx * abx + apy * aby) / ab_len_sq if ab_len_sq > 0 else 0.0
                t_clamped = max(0.0, min(1.0, t))
                
                proj_x = seg['ax'] + t_clamped * abx
                proj_y = seg['ay'] + t_clamped * aby
                dist = math.hypot(node.x - proj_x, node.y - proj_y)
                
                if dist <= (seg['width'] / 2.0 + 0.3):
                    segment_nodes_map[seg['name']].append((t_clamped, node))
                    
        # 6. Establish Segment Edges sequentially
        for nodes_list in segment_nodes_map.values():
            if len(nodes_list) < 2: continue
            nodes_list.sort(key=lambda item: item[0])
            for i in range(len(nodes_list) - 1):
                n1, n2 = nodes_list[i][1], nodes_list[i+1][1]
                if n1 != n2:
                    self.map.add_segment(n1, n2)
                    
        self.map.compile_map()
        self.get_logger().info(f"SDF compiled. Created {len([n for n in self.map.all_nodes if n])} topological nodes.")

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
        
        # Translate Node Steps to relative Turn Sequences
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
                        if 'left' in direction_lower: selected_turn = 'left'
                        elif 'right' in direction_lower: selected_turn = 'right'
                        else: selected_turn = 'straight'
                        break
                self.turn_sequence.append((current_node.name, selected_turn))
            else:
                self.turn_sequence.append((current_node.name, 'straight'))
                
        self.get_logger().info(f"Sequence of intermediate relative turn decisions: {self.turn_sequence}")

    def feedback_callback(self, msg: String):
        if msg.data == "completed":
            if self.current_turn_index < len(self.turn_sequence):
                node_name, turn_dir = self.turn_sequence[self.current_turn_index]
                self.get_logger().info(f"COORDINATION: Split action '{turn_dir.upper()}' at Node {node_name} completed.")
                
                # Advance step index
                self.current_turn_index += 1
                
                if self.current_turn_index < len(self.turn_sequence):
                    next_node, next_dir = self.turn_sequence[self.current_turn_index]
                    self.get_logger().info(f"COORDINATION: Setting action direction to '{next_dir.upper()}' for Node {next_node}.")
                else:
                    self.get_logger().info("COORDINATION: Final turn cleared. Direct corridor path remaining to destination.")
            else:
                self.get_logger().warn("COORDINATION: Received clearance signal, but no pending tasks are remaining.")

    def publish_current_target(self):
        msg = String()
        if self.current_turn_index < len(self.turn_sequence):
            _, turn_dir = self.turn_sequence[self.current_turn_index]
            msg.data = turn_dir
        else:
            # Check if we have completed all tasks on the route list
            if len(self.path) > 0:
                # Arrived at final straight section or destination, hold centering and prepare to end
                msg.data = "straight"
            else:
                msg.data = "stop"
        self.target_dir_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


'''
ros2 launch hambot_bringup voronoi_launch.py \
  world_name:=campus_sidewalk \
  start_point:=1 \
  start_node:=0 \
  end_node:=19
'''