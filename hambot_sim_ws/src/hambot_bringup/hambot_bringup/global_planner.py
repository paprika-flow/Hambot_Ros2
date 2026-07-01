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
    
    while angle_diff > math.pi: 
        angle_diff -= 2 * math.pi
    while angle_diff <= -math.pi: 
        angle_diff += 2 * math.pi
    return math.degrees(angle_diff)

def assign_directions_dynamically(nb_angles):
    left_nodes, straight_nodes, right_nodes = [], [], []
    for angle, node in nb_angles:
        if -20 <= angle <= 20:
            straight_nodes.append((angle, node))
        elif 20 < angle:
            left_nodes.append((angle, node))
        elif angle < -20:
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

class NodeClass:
    def __init__(self, name, x, y, original_name=None, nodes=None):
        self.name = name
        self.x = x
        self.y = y
        self.original_name = original_name if original_name is not None else name
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
        self.nodes_by_name = {}  # Safe dictionary lookup for alphanumeric node names
        self.edges = dict()

    def add_node(self, name, x, y, original_name=None):
        # Handle non-integer names (like 'p1') safely by mapping them to an auto-assigned list index
        if not name.isdigit():
            idx = len(self.all_nodes)
            self.all_nodes.append(None)
        else:
            idx = int(name)
            while len(self.all_nodes) <= idx:
                self.all_nodes.append(None)
                
        node = NodeClass(name, x, y, original_name)
        self.all_nodes[idx] = node
        self.nodes_by_name[name] = node
        self.nodes_amount = max(self.nodes_amount, idx + 1)
        return node

    def add_segment(self, node1, node2):
        if node2 not in node1.nodes: 
            node1.nodes.append(node2)
        if node1 not in node2.nodes: 
            node2.nodes.append(node1)
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

        self.start_node_str = self.declare_flexible_node_parameter('start_node', '0')
        self.end_node_str = self.declare_flexible_node_parameter('end_node', '19')
        
        # Build Map Structure (Calculated strictly once during setup)
        self.map = Map()
        self.plazas = {}  # Holds plaza structure mapping
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

    def declare_flexible_node_parameter(self, name, default_val):
        # 1. Try to declare as integer first to match standard parameters file overrides
        try:
            self.declare_parameter(name, int(default_val))
            val = self.get_parameter(name).value
            return str(val)
        except Exception:
            # 2. If integer conversion fails or type mismatch occurs, fallback to string
            try:
                self.declare_parameter(name, str(default_val))
                val = self.get_parameter(name).value
                return str(val)
            except Exception:
                return str(default_val)

    def load_sdf_geometry(self):
        pkg_bringup = get_package_share_directory('hambot_bringup')
        world_path = os.path.join(pkg_bringup, 'worlds', f"{self.world_name}.sdf")
        
        if not os.path.exists(world_path):
            self.get_logger().error(f"SDF World path does not exist: {world_path}")
            return
            
        tree = ET.parse(world_path)
        root = tree.getroot()
        world = root.find('world')
        
        raw_frames = []
        for frame in world.findall('frame'):
            name = frame.attrib['name']
            pose_text = frame.find('pose').text.strip().split()
            x, y = float(pose_text[0]), float(pose_text[1])
            raw_frames.append({'name': name, 'x': x, 'y': y})
            
        cleaned_frames = []
        for f in raw_frames:
            merged = False
            for cf in cleaned_frames:
                if math.hypot(f['x'] - cf['x'], f['y'] - cf['y']) < 1.0:
                    merged = True
                    break
            if not merged:
                cleaned_frames.append(f)
                
        node_id_mapping = {}
        for f in cleaned_frames:
            # SPECIAL p* PLAZA IDENTIFICATION RULE:
            # If the frame name represents a plaza waypoint, identify it as "p*" 
            # (where * is the number) to differentiate it as a specialized hub node.
            if "plaza" in f['name'].lower():
                match = re.search(r'\d+', f['name'])
                idx_str = f"p{match.group()}" if match else "p1"
            else:
                match = re.search(r'\d+', f['name'])
                idx_str = match.group() if match else f['name']
                
            node_id_mapping[f['name']] = idx_str
            self.map.add_node(idx_str, f['x'], f['y'], original_name=f['name'])
            
        sidewalk_model = None
        for model in world.findall('model'):
            if model.attrib.get('name') == 'sidewalk_network':
                sidewalk_model = model
                break
                
        if sidewalk_model is None:
            self.get_logger().error("Model 'sidewalk_network' not found inside the specified world.")
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
            
        segment_nodes_map = {seg['name']: [] for seg in segments}
        for node in self.map.all_nodes:
            if node is None: 
                continue
            for seg in segments:
                abx, aby = seg['bx'] - seg['ax'], seg['by'] - seg['ay']
                apx, apy = node.x - seg['ax'], node.y - seg['ay']
                ab_len_sq = abx**2 + aby**2
                
                t = (apx * abx + apy * aby) / ab_len_sq if ab_len_sq > 0 else 0.0
                t_clamped = max(0.0, min(1.0, t))
                
                proj_x = seg['ax'] + t_clamped * abx
                proj_y = seg['ay'] + t_clamped * aby
                dist = math.hypot(node.x - proj_x, node.y - proj_y)
                
                tolerance = 3.5 if seg['width'] >= 1.5 else 0.3
                if dist <= (seg['width'] / 2.0 + tolerance):
                    segment_nodes_map[seg['name']].append((t_clamped, node))
                    
        # RULE 1: Connect 1D corridors sequentially, but connect plazas/wide areas as hub-and-spoke
        for seg_name, nodes_list in segment_nodes_map.items():
            if len(nodes_list) < 2: 
                continue
                
            seg = next(s for s in segments if s['name'] == seg_name)
            seg_width = seg.get('width', 0.9)
            
            if seg_width >= 1.5:
                # Wide Area/Plaza: Identify the special p* plaza hub node
                plaza_center = None
                for _, node in nodes_list:
                    if str(node.name).startswith('p'):
                        plaza_center = node
                        break
                
                # Fallback to the closest centroid node if naming doesn't contain 'p'
                if not plaza_center:
                    center_x = (seg['ax'] + seg['bx']) / 2.0
                    center_y = (seg['ay'] + seg['by']) / 2.0
                    min_dist = float('inf')
                    for _, node in nodes_list:
                        d = math.hypot(node.x - center_x, node.y - center_y)
                        if d < min_dist:
                            min_dist = d
                            plaza_center = node
                        
                if plaza_center:
                    portals = [n for _, n in nodes_list if n != plaza_center]
                    self.plazas[plaza_center.name] = {
                        'center': plaza_center,
                        'portals': portals
                    }
                    # Link portals strictly to the central hub node (spokes)
                    for portal in portals:
                        self.map.add_segment(plaza_center, portal)
            else:
                # Narrow Corridor: Connect nodes sequentially along the projection axis
                nodes_list.sort(key=lambda item: item[0])
                for i in range(len(nodes_list) - 1):
                    n1, n2 = nodes_list[i][1], nodes_list[i+1][1]
                    if n1 != n2:
                        self.map.add_segment(n1, n2)

        # RULE 2: Bridge nodes separated by empty curved/turning segment chains (Segment BFS)
        touching_segments = {seg['name']: [] for seg in segments}
        for seg1 in segments:
            for seg2 in segments:
                if seg1['name'] == seg2['name']:
                    continue
                ep1 = [(seg1['ax'], seg1['ay']), (seg1['bx'], seg1['by'])]
                ep2 = [(seg2['ax'], seg2['ay']), (seg2['bx'], seg2['by'])]
                
                is_touching = False
                for p1 in ep1:
                    for p2 in ep2:
                        if math.hypot(p1[0] - p2[0], p1[1] - p2[1]) < 1.0:
                            is_touching = True
                            break
                    if is_touching:
                        break
                if is_touching:
                    touching_segments[seg1['name']].append(seg2['name'])

        segs_with_nodes = [s_name for s_name, nodes in segment_nodes_map.items() if len(nodes) > 0]
        connections_to_make = set()
        
        for start_seg in segs_with_nodes:
            queue = [(start_seg, [start_seg])]
            visited = {start_seg}
            
            while queue:
                curr_seg, path = queue.pop(0)
                
                if curr_seg != start_seg and curr_seg in segs_with_nodes:
                    pair = tuple(sorted([start_seg, curr_seg]))
                    connections_to_make.add(pair)
                    continue
                    
                for neighbor in touching_segments[curr_seg]:
                    if neighbor not in visited:
                        visited.add(neighbor)
                        queue.append((neighbor, path + [neighbor]))
                        
        for segA, segB in connections_to_make:
            nodesA = segment_nodes_map[segA]
            nodesB = segment_nodes_map[segB]
            
            best_pair = None
            min_d = float('inf')
            for _, n1 in nodesA:
                for _, n2 in nodesB:
                    if n1 != n2:
                        d = math.hypot(n1.x - n2.x, n1.y - n2.y)
                        if d < min_d:
                            min_d = d
                            best_pair = (n1, n2)
            if best_pair:
                self.map.add_segment(best_pair[0], best_pair[1])

        # PORTAL DISCONNECTION SANITIZATION PASS:
        # Enforces a strict Hub-and-Spoke structure. If any two nodes connected to each other 
        # belong to the same plaza center, break their connection. This forces Dijkstra to route 
        # physically through the center 'p*' node, triggering the plaza's geometric relative turn rules.
        for plaza_name, plaza_data in self.plazas.items():
            portals = plaza_data['portals']
            for i in range(len(portals)):
                for j in range(i + 1, len(portals)):
                    p1 = portals[i]
                    p2 = portals[j]
                    if p2 in p1.nodes:
                        p1.nodes.remove(p2)
                    if p1 in p2.nodes:
                        p2.nodes.remove(p1)
                    self.map.edges.pop((p1, p2), None)
                    self.map.edges.pop((p2, p1), None)

        self.map.compile_map()
        self.get_logger().info(f"SDF compiled. Created {len([n for n in self.map.all_nodes if n])} topological nodes.")

        # =====================================================================
        # DEBUG VERIFICATION: PRINT MAP DATA STRUCTURE
        # =====================================================================
        print("\n" + "="*60)
        print("          VERIFYING TOPOLOGICAL MAP DATA STRUCTURE")
        print("="*60)
        print("--- TOPOLOGICAL NODES LIST ---")
        for n in self.map.all_nodes:
            if n is None: 
                continue
            degree = len(n.nodes)
            node_type = "Dead-End" if degree == 1 else f"{degree}-way Intersection"
            neighbors = ", ".join([nb.original_name for nb in n.nodes])
            print(f"  Node {n.original_name:10s} | Position: ({n.x:6.2f}, {n.y:6.2f}) | {node_type:20s} | Neighbors: [{neighbors}]")
            
        print("\n--- EDGE DIRECTIONS & PERSPECTIVES ---")
        for edge in self.map.edges.values():
            n1, n2 = edge.node1, edge.node2
            print(f"\nEdge ({n1.original_name} <-> {n2.original_name}) | Distance: {edge.dist:.2f} meters")
            
            # Print directions from both travel perspectives
            for curr, approach in [(n1, n2), (n2, n1)]:
                dir_map = edge.directions.get(curr.name, {})
                if dir_map:
                    perspective_str = " | ".join([f"{direction} -> Node {target.original_name}" for direction, target in dir_map.items()])
                    print(f"  * At Node {curr.original_name} (approaching from Node {approach.original_name}): {perspective_str}")
                else:
                    print(f"  * At Node {curr.original_name} (approaching from Node {approach.original_name}): Dead End (No connections)")
        print("\n" + "="*60 + "\n")

    def generate_plaza_instruction(self, plaza_name, entry_node, exit_node):
        """
        Generates a semantic "N-th left/right" roundabout instruction for traversing a plaza,
        and returns the correct command direction for the voronoi_navigator (e.g. plaza_right_2).
        """
        plaza_data = self.plazas.get(plaza_name)
        if not plaza_data:
            return f"proceed straight towards {exit_node.original_name}", "straight"
            
        center = plaza_data['center']
        portals = plaza_data['portals']
        
        # 1. Compute absolute angles from plaza center to each portal
        theta_entry_node = math.atan2(entry_node.y - center.y, entry_node.x - center.x)
        theta_exit_node = math.atan2(exit_node.y - center.y, exit_node.x - center.x)
        
        # 2. Determine if the exit is to the left or right of our entry heading
        # Entry heading is the vector pointing from entry_node to center
        dx_heading = center.x - entry_node.x
        dy_heading = center.y - entry_node.y
        theta_entry_heading = math.atan2(dy_heading, dx_heading)
        
        delta_theta = theta_exit_node - theta_entry_heading
        delta_theta = (delta_theta + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]
        
        # If relative angle is negative, we go counter-clockwise (to the right)
        # If relative angle is positive, we go clockwise (to the left)
        is_left = (delta_theta >= 0.0)
        direction_str = "left" if is_left else "right"
        
        # 3. Gather other portals and compute their angles relative to the entry node
        portal_ranks = []
        for portal in portals:
            if portal.name == entry_node.name:
                continue
                
            theta_p = math.atan2(portal.y - center.y, portal.x - center.x)
            
            if is_left:
                # Clockwise (Left) order: angle decreases, so (theta_entry_node - theta_p) mod 2pi
                rel_angle = (theta_entry_node - theta_p) % (2 * math.pi)
            else:
                # Counter-Clockwise (Right) order: angle increases, so (theta_p - theta_entry_node) mod 2pi
                rel_angle = (theta_p - theta_entry_node) % (2 * math.pi)
                
            portal_ranks.append((portal, rel_angle))
            
        # Sort ascending to order them as we encounter them driving around the perimeter
        portal_ranks.sort(key=lambda x: x[1])
        
        # 4. Find the exit rank of our target exit_node
        target_rank = -1
        for idx, (portal, _) in enumerate(portal_ranks):
            if portal.name == exit_node.name:
                target_rank = idx + 1
                break
                
        if target_rank == -1:
            return f"proceed straight towards {exit_node.original_name}", "straight"
            
        ordinals = {1: "first", 2: "second", 3: "third", 4: "fourth"}
        rank_str = ordinals.get(target_rank, f"{target_rank}th")
        
        command_str = f"plaza_{direction_str}_{target_rank}"
        instruction_str = f"take the {rank_str} {direction_str} exit"
        
        return instruction_str, command_str

    def compute_global_plan(self):
        # Look up start and end nodes securely using the dictionary lookup table
        start_node_obj = self.map.nodes_by_name.get(self.start_node_str)
        if start_node_obj is None:
            try:
                start_node_obj = self.map.all_nodes[int(self.start_node_str)]
            except (ValueError, IndexError):
                pass
                
        end_node_obj = self.map.nodes_by_name.get(self.end_node_str)
        if end_node_obj is None:
            try:
                end_node_obj = self.map.all_nodes[int(self.end_node_str)]
            except (ValueError, IndexError):
                pass
                
        if start_node_obj is None or end_node_obj is None:
            self.get_logger().error(f"Routing failed: Start '{self.start_node_str}' or End '{self.end_node_str}' invalid.")
            return
            
        path, dist = dijkstra(self.map, start_node_obj, end_node_obj)
        if not path:
            self.get_logger().error(f"No path could be calculated between start and end node.")
            return
            
        self.path = path
        path_names = [n.original_name for n in path]
        self.get_logger().info(f"Dijkstra computed global trajectory: {path_names} (Distance: {dist:.2f}m)")
        
        self.turn_sequence = []
        instructions_summary = []
        
        # 1. Starting Segment Instruction
        if len(path) >= 2:
            # Check if the first transition enters a plaza
            if len(path) >= 3 and path[1].name in self.plazas:
                # First node is the entry portal to a plaza!
                # We handle this in the loop below, so we don't need a separate step 1 proceed straight
                pass
            else:
                instructions_summary.append(f"Proceed straight from {path[0].original_name} to {path[1].original_name}")
            
        i = 1
        while i < len(path) - 1:
            prev_node = path[i-1]
            current_node = path[i]
            next_node = path[i+1]
            
            # PLAZA ENTRY LOOK-AHEAD RULE:
            # If the next node in the path is a plaza center, then the current node is the entry portal.
            # We must assign the plaza instruction and "plaza_direction_rank" command directly to 
            # the current node (the entry portal) and skip any intermediate "straight to plaza center" instructions.
            if next_node.name in self.plazas and i + 2 < len(path):
                plaza_name = next_node.name
                entry_node = current_node
                exit_node = path[i+2]
                
                instruction_str, command_str = self.generate_plaza_instruction(plaza_name, entry_node, exit_node)
                instructions_summary.append(
                    f"At entry portal {entry_node.original_name}, "
                    f"{instruction_str.upper()} (Command: {command_str.upper()}) and exit towards {exit_node.original_name}"
                )
                self.turn_sequence.append((entry_node.name, command_str))
                
                # Advance i by 3 to skip both the plaza center node itself and its exit portal step
                # and the exit to the next node
                i += 3
                continue
                
            # If current_node is a plaza center node (which could happen if we start at the plaza or pathing fallback),
            # we handle it defensively, but otherwise we bypass it.
            if current_node.name in self.plazas:
                i += 1
                continue
                
            # Standard corridor directional logic
            edge = self.map.edges.get((prev_node, current_node)) or self.map.edges.get((current_node, prev_node))
            selected_turn = 'straight'
            if edge is not None:
                dir_map = edge.directions.get(current_node.name, {})
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
            instructions_summary.append(
                f"At Node {current_node.original_name}, turn {selected_turn.upper()} towards {next_node.original_name}"
            )
            self.turn_sequence.append((current_node.name, selected_turn))
            i += 1
            
        # 2. Final Approach Instruction
        if len(path) >= 2:
            # Only add final approach if the last edge wasn't already covered in the plaza skip
            last_sequence_node_name = self.turn_sequence[-1][0] if self.turn_sequence else None
            if last_sequence_node_name != path[-2].name:
                instructions_summary.append(
                    f"Finally, proceed straight from {path[-2].original_name} to {path[-1].original_name} to reach target destination."
                )
            
        # 3. Print the comprehensive route checklist on startup
        print("\n" + "="*75)
        print("                 COMPREHENSIVE GLOBAL ROUTE ITINERARY")
        print("="*75)
        for idx, step in enumerate(instructions_summary, 1):
            print(f"  Step {idx:2d}: {step}")
        print("="*75 + "\n")
                
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
                
                # 1. Advance step index
                self.current_turn_index += 1
                
                # 2. IMMEDIATELY dispatch next target target to eliminate polling latency
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

        # Check destination node type if we are on the final edge
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
                    f"TRACKER: Robot is traversing edge {node_a.original_name} <-> {node_b.original_name}. Next turn decision at Node {node_b.original_name} is '{turn_dir.upper()}'",
                    throttle_duration_sec=5.0
                )
        else:
            node_a = self.path[-2]
            node_b = self.path[-1]
            self.get_logger().info(
                f"TRACKER: Robot is on the final straight section {node_a.original_name} <-> {node_b.original_name} approaching target.",
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