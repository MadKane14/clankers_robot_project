import xml.etree.ElementTree as ET
import math
import os
import json
import re

def calculate_distance(p1, p2):
    """Calculates Euclidean distance between two (x, y) tuples."""
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def extract_spawn_from_launch(launch_path):
    """Dynamically extracts the robot's initial x, y spawn coordinates from the launch file."""
    print(f"Parsing Launch File for spawn coords: {launch_path}")
    x, y = 0.0, 0.0
    try:
        with open(launch_path, 'r') as f:
            content = f.read()
            # Regex to find 'x': '-8.0' and 'y': '2.0' inside the launch dictionary
            x_match = re.search(r"'x':\s*'([^']+)'", content)
            y_match = re.search(r"'y':\s*'([^']+)'", content)
            
            if x_match: x = float(x_match.group(1))
            if y_match: y = float(y_match.group(1))
            print(f"-> Extracted start coordinates: x={x}, y={y}")
    except FileNotFoundError:
        print("-> Launch file not found, defaulting to (0.0, 0.0)")
        
    return x, y

def generate_topological_map(sdf_path, start_x, start_y):
    print(f"Parsing SDF File for objects: {sdf_path}")
    tree = ET.parse(sdf_path)
    root = tree.getroot()

    nodes = {}
    nodes["start"] = (start_x, start_y)

    for model in root.findall('.//model'):
        name = model.get('name')
        if name and ('capsule' in name or 'cylinder' in name):
            pose_elem = model.find('pose')
            if pose_elem is not None:
                pose_values = pose_elem.text.strip().split()
                x = float(pose_values[0])
                y = float(pose_values[1])
                
                node_type = "charger" if 'capsule' in name else "delivery"
                node_name = f"{node_type}_{name}"
                nodes[node_name] = (x, y)
                print(f"-> Found {node_name} at ({x}, {y})")

    graph = {}
    for node_a, coords_a in nodes.items():
        graph[node_a] = {}
        for node_b, coords_b in nodes.items():
            if node_a != node_b:
                distance = calculate_distance(coords_a, coords_b)
                graph[node_a][node_b] = round(distance, 2)

    return nodes, graph

if __name__ == '__main__':
    # Use os.path.expanduser to automatically resolve '~' to '/home/madkane'
    base_dir = os.path.expanduser('~/ros2_ws/src/clankers_robot_project')
    world_file_path = os.path.join(base_dir, 'my_robot_simulation', 'worlds', 'custom_world1.sdf')
    launch_file_path = os.path.join(base_dir, 'my_robot_simulation', 'launch', 'simulation.launch.py')
    
    output_json_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'topological_map.json')
    
    # 1. Get the real starting coordinates
    start_x, start_y = extract_spawn_from_launch(launch_file_path)
    
    try:
        # 2. Generate the map using those real coordinates
        nodes, graph = generate_topological_map(world_file_path, start_x, start_y)
        
        map_data = {
            "nodes": nodes,
            "graph": graph
        }
        
        with open(output_json_path, 'w') as json_file:
            json.dump(map_data, json_file, indent=4)
            
        print(f"\nSUCCESS: Topological map saved to {output_json_path}")
            
    except FileNotFoundError:
        print(f"ERROR: Could not find the SDF file at {world_file_path}")