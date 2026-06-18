import math
import heapq

# --- 1. Helper Functions for Geometry ---

def calculate_relative_angle(prev_node, curr_node, next_node):
    """
    Calculates the relative angle (in degrees) from prev_node through curr_node to next_node
    using 2D vector angles on a standard Cartesian plane.
    """
    v_in_x = curr_node.x - prev_node.x
    v_in_y = curr_node.y - prev_node.y
    
    v_out_x = next_node.x - curr_node.x
    v_out_y = next_node.y - curr_node.y
    
    angle_in = math.atan2(v_in_y, v_in_x)
    angle_out = math.atan2(v_out_y, v_out_x)
    
    angle_diff = angle_out - angle_in
    
    # Normalize angle difference to [-pi, pi]
    while angle_diff > math.pi:
        angle_diff -= 2 * math.pi
    while angle_diff <= -math.pi:
        angle_diff += 2 * math.pi
        
    return math.degrees(angle_diff)

def assign_directions_dynamically(nb_angles):
    """
    Dynamically sorts and assigns directional labels to adjacent nodes based on
    relative angles and local congestion (hierarchical shifting).
    """
    left_nodes = []
    straight_nodes = []
    right_nodes = []
    
    for angle, node in nb_angles:
        # Initial categorization based on standard sectors
        if -30 <= angle <= 30:
            straight_nodes.append((angle, node))
        elif 30 < angle < 150:
            left_nodes.append((angle, node))
        elif -150 < angle < -30:
            right_nodes.append((angle, node))
        else:
            # Handle U-turns / hard sharp turns near +/-180 degrees
            if angle >= 150:
                left_nodes.append((angle, node))
            else:
                right_nodes.append((angle, node))
                
    # Hierarchical Shift Rule: If there is a highly-left branch (>= 60 degrees),
    # any softer-left branch (< 60 degrees) gets shifted into the Straight category.
    has_large_left = any(angle >= 60 for angle, _ in left_nodes)
    if has_large_left:
        to_move = [item for item in left_nodes if item[0] < 60]
        left_nodes = [item for item in left_nodes if item[0] >= 60]
        straight_nodes.extend(to_move)
        
    # Symmetric rule for right-leaning branches
    has_large_right = any(angle <= -60 for angle, _ in right_nodes)
    if has_large_right:
        to_move = [item for item in right_nodes if item[0] > -60]
        right_nodes = [item for item in right_nodes if item[0] <= -60]
        straight_nodes.extend(to_move)
        
    dirs = {}
    
    # 1. Map Lefts
    if len(left_nodes) == 1:
        dirs['Left'] = left_nodes[0][1]
    elif len(left_nodes) > 1:
        # Sort descending (leftmost first)
        left_nodes.sort(key=lambda x: x[0], reverse=True)
        for idx, (_, node) in enumerate(left_nodes, 1):
            dirs[f'Left {idx}'] = node
            
    # 2. Map Rights
    if len(right_nodes) == 1:
        dirs['Right'] = right_nodes[0][1]
    elif len(right_nodes) > 1:
        # Sort ascending (rightmost first)
        right_nodes.sort(key=lambda x: x[0])
        for idx, (_, node) in enumerate(right_nodes, 1):
            dirs[f'Right {idx}'] = node
            
    # 3. Map Straights
    if len(straight_nodes) == 1:
        dirs['Straight'] = straight_nodes[0][1]
    elif len(straight_nodes) > 1:
        # Sort descending (from left to right)
        straight_nodes.sort(key=lambda x: x[0], reverse=True)
        for idx, (_, node) in enumerate(straight_nodes, 1):
            dirs[f'Straight {idx}'] = node
            
    return dirs

def get_intersection_type(dir_map):
    """Factory to return the correct legacy Intersection class."""
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


# --- 2. Data Structure Definitions ---

class Intersection:
    def __init__(self, left, straight, right):
        self.left = left
        self.straight = straight
        self.right = right
        
    def print_intersection(self):
        directions = []
        if self.left:
            directions.append("Left")
        if self.straight:
            directions.append("Straight")
        if self.right:
            directions.append("Right")
        print(" ".join(directions))

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

class Node:
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
        self.type_intersection1 = no_intersection() 
        self.type_intersection2 = no_intersection()
        
        self.directions = {
            node1.name: {},
            node2.name: {}
        }
        
    def _print_node_perspective(self, current_node, approach_node, type_intersection):
        print(f"  At Node {current_node.name} (approaching from Node {approach_node.name}):")
        dir_map = self.directions.get(current_node.name, {})
        
        if not dir_map:
            print("    Dead End (no intersection)")
            return
            
        def sort_key(k):
            if k.startswith('Left'): return (0, k)
            if k.startswith('Straight'): return (1, k)
            if k.startswith('Right'): return (2, k)
            return (3, k)
            
        for k in sorted(dir_map.keys(), key=sort_key):
            print(f"    {k} -> Node {dir_map[k].name}")
        
    def print_1(self):
        self._print_node_perspective(self.node1, self.node2, self.type_intersection1)
        
    def print_2(self):
        self._print_node_perspective(self.node2, self.node1, self.type_intersection2)

class Map:
    def __init__(self):
        self.nodes_amount = 0
        self.all_nodes = []
        self.edges = dict()

    def add_node(self, x, y, nodes=None):
        node = Node(f"{self.nodes_amount}", x, y, nodes)
        self.all_nodes.append(node)
        self.nodes_amount += 1
        return node

    def change_node(self, node, node_to_be_added):
        self.all_nodes[int(node.name)].nodes.append(node_to_be_added)
    
    def print_all(self):
        print("Nodes: ")
        for n in self.all_nodes:
            if n is None:
                continue
            degree = len(n.nodes)
            node_type = "Dead-End" if degree == 1 else f"{degree}-way Intersection"
            print(f"    {n.name} (x:{n.x}, y:{n.y}) [{node_type}] - Connected to: ", end="")
            for nn in n.nodes:
                print(f"{nn.name}", end=' ')
            print()
        print("\nEdges: ")
        for key, value in self.edges.items():
            print(f"Edge between Node {key[0].name} and Node {key[1].name}:")
            value.print_1()
            value.print_2()
            print(f"  Distance: {value.dist:.2f}\n")

    def add_segment(self, node1: Node, node2: Node, distance: int = None):
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

        # Automatically calculate and assign euclidean distance
        calculated_dist = math.dist((node1.x, node1.y), (node2.x, node2.y))
        self.edges[node1, node2] = Edges(node1, node2, calculated_dist)

    def compile_map(self):
        """
        Loops through all edges to automatically populate relative direction 
        dictionaries and detect physical intersection shapes.
        """
        for edge in self.edges.values():
            n1, n2 = edge.node1, edge.node2
            
            # Clear and rebuild direction mappings
            edge.directions[n1.name] = {}
            edge.directions[n2.name] = {}
            
            # Evaluate turns at node1 (approaching from node2)
            neighbors1 = [nb for nb in n1.nodes if nb != n2]
            if neighbors1:
                nb_angles = [(calculate_relative_angle(n2, n1, nb), nb) for nb in neighbors1]
                edge.directions[n1.name] = assign_directions_dynamically(nb_angles)
                    
            # Evaluate turns at node2 (approaching from node1)
            neighbors2 = [nb for nb in n2.nodes if nb != n1]
            if neighbors2:
                nb_angles = [(calculate_relative_angle(n1, n2, nb), nb) for nb in neighbors2]
                edge.directions[n2.name] = assign_directions_dynamically(nb_angles)
            
            # Automatically assign correct Intersection class shapes
            edge.type_intersection1 = get_intersection_type(edge.directions[n1.name])
            edge.type_intersection2 = get_intersection_type(edge.directions[n2.name])


# --- 3. Routing Engine ---

def dijkstra(road_map: Map, start_node: Node, end_node: Node):
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
    
    if distances[end_node] == float('inf'):
        return None, float('inf')
        
    return path, distances[end_node]


def get_route_instructions(road_map, path):
    if not path:
        return ["No path found."]
    
    instructions = []
    instructions.append(f"Start at Node {path[0].name}.")
    
    for i in range(1, len(path)):
        prev_node = path[i-2] if i >= 2 else None
        current_node = path[i-1]
        next_node = path[i]
        
        if prev_node is None:
            instructions.append(f"Proceed straight to Node {next_node.name}.")
        else:
            edge = road_map.edges.get((prev_node, current_node)) or road_map.edges.get((current_node, prev_node))
            if edge is None:
                instructions.append(f"At Node {current_node.name}, continue to Node {next_node.name}.")
                continue
            
            dir_map = edge.directions.get(current_node.name, {})
            turn = None
            for direction, target_node in dir_map.items():
                if target_node == next_node:
                    turn = direction
                    break
            
            if turn:
                instructions.append(f"At Node {current_node.name}, turn {turn} to Node {next_node.name}.")
            else:
                instructions.append(f"At Node {current_node.name}, continue to Node {next_node.name}.")
                
    instructions.append(f"Arrive at Node {path[-1].name}.")
    return instructions


# Setup 11 Nodes
my_map = Map()
n0 = my_map.add_node(1, 3)
n1 = my_map.add_node(0, 3)
n2 = my_map.add_node(1, 4)
n3 = my_map.add_node(1, 1)
n4 = my_map.add_node(0, 1)
n5 = my_map.add_node(2, 1)
n6 = my_map.add_node(1, 0)
n7 = my_map.add_node(0, 0)
n8 = my_map.add_node(2, 0)
n9 = my_map.add_node(3, 0)
n10 = my_map.add_node(3, 1)  # Fixed coordinates: No longer passes over node 5
n11 = my_map.add_node(4,0)

# Set up Segments
my_map.add_segment(n0, n1)
my_map.add_segment(n0, n2)
my_map.add_segment(n0, n3)
my_map.add_segment(n3, n4)
my_map.add_segment(n3, n5)
my_map.add_segment(n3, n6)
my_map.add_segment(n6, n7)
my_map.add_segment(n6, n8)
my_map.add_segment(n8, n9)
my_map.add_segment(n8, n10)


# Re-compile Layout
my_map.compile_map()


# --- VERIFICATION TEST 1: Standard Configuration (Without 5-to-8 Connection) ---
print("================= TEST 1: NO CONNECTION BETWEEN 5 AND 8 =================")
edge_6_8 = my_map.edges.get((n6, n8)) or my_map.edges.get((n8, n6))
edge_6_8.print_2()  # At Node 8 approaching from Node 6

# --- VERIFICATION TEST 2: Dynamic Update (Connecting Node 5 to Node 8) ---
print("\n================= TEST 2: ADDING SEGMENT BETWEEN 5 AND 8 =================")
my_map.add_segment(n0, n10)
my_map.add_segment(n9, n10)
my_map.add_segment(n9, n11)
my_map.compile_map()  # Recalculate intersections and directions dynamically
my_map.print_all()

edge_6_8_updated = my_map.edges.get((n6, n8)) or my_map.edges.get((n8, n6))
edge_6_8_updated.print_2()  # At Node 8 approaching from Node 6 (Updated)


def run_test(start_node, end_node):
    print(f"\n================= ROUTING FROM {start_node.name} TO {end_node.name} =================")
    path, dist = dijkstra(my_map, start_node, end_node)
    if path:
        instructions = get_route_instructions(my_map, path)
        for index, step in enumerate(instructions, 1):
            print(f"  {index}. {step}")
        print(f"  Total Distance: {dist}")
    else:
        print("  No route found.")

# Test 1: Route from Node 1 (Top-Left Dead-end) to Node 9 (Far-Right Dead-end)
run_test(n1, n9)

# Test 2: Route from Node 7 (Bottom-Left Dead-end) to Node 10 (Mid-Right Dead-end)
run_test(n2, n8)