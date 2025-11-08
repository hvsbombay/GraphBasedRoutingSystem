#!/usr/bin/env python3
"""
Test case generator for Phase 1 of the Graph Routing System
Generates simple synthetic graphs and queries for testing
"""

import json
import random
import math

def generate_simple_graph(num_nodes=10, num_edges=20):
    """Generate a simple connected graph"""
    
    # Generate nodes in a grid-like pattern around Mumbai coordinates
    base_lat = 19.070
    base_lon = 72.870
    
    nodes = []
    poi_types = ["Restaurant", "Hospital", "School", "Mall", "Park"]
    
    for i in range(num_nodes):
        lat = base_lat + (i % 5) * 0.01
        lon = base_lon + (i // 5) * 0.01
        
        # Randomly assign 0-2 POIs to each node
        num_pois = random.randint(0, 2)
        pois = random.sample(poi_types, min(num_pois, len(poi_types)))
        
        nodes.append({
            "id": i,
            "lat": lat,
            "lon": lon,
            "pois": pois
        })
    
    # Generate edges
    edges = []
    edge_id = 1000
    road_types = ["primary", "secondary", "tertiary", "local", "expressway"]
    
    # Create a connected graph first (spanning tree)
    for i in range(1, num_nodes):
        u = random.randint(0, i-1)
        v = i
        
        # Calculate approximate distance
        lat1, lon1 = nodes[u]["lat"], nodes[u]["lon"]
        lat2, lon2 = nodes[v]["lat"], nodes[v]["lon"]
        
        dlat = (lat2 - lat1) * 111000  # meters
        dlon = (lon2 - lon1) * 111000 * math.cos(math.radians((lat1 + lat2) / 2))
        length = math.sqrt(dlat**2 + dlon**2)
        
        # Average speed 10-20 m/s
        avg_speed = random.uniform(10, 20)
        avg_time = length / avg_speed
        
        edges.append({
            "id": edge_id,
            "u": u,
            "v": v,
            "length": round(length, 2),
            "average_time": round(avg_time, 2),
            "oneway": random.choice([True, False]),
            "road_type": random.choice(road_types)
        })
        edge_id += 1
    
    # Add random edges
    for _ in range(num_edges - (num_nodes - 1)):
        u = random.randint(0, num_nodes - 1)
        v = random.randint(0, num_nodes - 1)
        
        if u == v:
            continue
        
        # Calculate distance
        lat1, lon1 = nodes[u]["lat"], nodes[u]["lon"]
        lat2, lon2 = nodes[v]["lat"], nodes[v]["lon"]
        
        dlat = (lat2 - lat1) * 111000
        dlon = (lon2 - lon1) * 111000 * math.cos(math.radians((lat1 + lat2) / 2))
        length = math.sqrt(dlat**2 + dlon**2)
        
        avg_speed = random.uniform(10, 20)
        avg_time = length / avg_speed
        
        edges.append({
            "id": edge_id,
            "u": u,
            "v": v,
            "length": round(length, 2),
            "average_time": round(avg_time, 2),
            "oneway": random.choice([True, False]),
            "road_type": random.choice(road_types)
        })
        edge_id += 1
    
    graph = {
        "meta": {
            "id": "test_graph_1",
            "nodes": num_nodes,
            "description": "Synthetic test graph for Phase 1"
        },
        "nodes": nodes,
        "edges": edges
    }
    
    return graph

def generate_phase1_queries(num_nodes=10):
    """Generate Phase 1 test queries"""
    
    queries = []
    query_id = 1
    
    # Shortest path query (distance mode)
    queries.append({
        "type": "shortest_path",
        "id": query_id,
        "source": 0,
        "target": num_nodes - 1,
        "mode": "distance"
    })
    query_id += 1
    
    # Shortest path query (time mode)
    queries.append({
        "type": "shortest_path",
        "id": query_id,
        "source": 1,
        "target": num_nodes - 2,
        "mode": "time"
    })
    query_id += 1
    
    # Shortest path with constraints
    queries.append({
        "type": "shortest_path",
        "id": query_id,
        "source": 0,
        "target": num_nodes - 1,
        "mode": "distance",
        "constraints": {
            "forbidden_nodes": [2, 3],
            "forbidden_road_types": ["local"]
        }
    })
    query_id += 1
    
    # KNN query - Euclidean
    queries.append({
        "type": "knn",
        "id": query_id,
        "poi": "restaurant",
        "query_point": {
            "lat": 19.075,
            "lon": 72.875
        },
        "k": 3,
        "metric": "euclidean"
    })
    query_id += 1
    
    # KNN query - Shortest path
    queries.append({
        "type": "knn",
        "id": query_id,
        "poi": "hospital",
        "query_point": {
            "lat": 19.080,
            "lon": 72.880
        },
        "k": 2,
        "metric": "shortest_path"
    })
    query_id += 1
    
    # Dynamic update - remove edge
    queries.append({
        "type": "remove_edge",
        "id": query_id,
        "edge_id": 1001
    })
    query_id += 1
    
    # Query after edge removal
    queries.append({
        "type": "shortest_path",
        "id": query_id,
        "source": 0,
        "target": num_nodes - 1,
        "mode": "distance"
    })
    query_id += 1
    
    # Modify edge - restore
    queries.append({
        "type": "modify_edge",
        "id": query_id,
        "edge_id": 1001,
        "patch": {
            "length": 150.0
        }
    })
    query_id += 1
    
    # Query after edge modification
    queries.append({
        "type": "shortest_path",
        "id": query_id,
        "source": 0,
        "target": num_nodes - 1,
        "mode": "distance"
    })
    query_id += 1
    
    return {
        "meta": {"id": "phase1_test_queries"},
        "events": queries
    }

if __name__ == "__main__":
    # Generate graph
    print("Generating test graph...")
    graph = generate_simple_graph(num_nodes=15, num_edges=30)
    
    with open("tests/test_graph_1.json", "w") as f:
        json.dump(graph, f, indent=2)
    
    print(f"Generated graph with {len(graph['nodes'])} nodes and {len(graph['edges'])} edges")
    
    # Generate queries
    print("Generating test queries...")
    queries = generate_phase1_queries(num_nodes=15)
    
    with open("tests/test_queries_1.json", "w") as f:
        json.dump(queries, f, indent=2)
    
    print(f"Generated {len(queries['events'])} queries")
    print("\nTest files created:")
    print("  - tests/test_graph_1.json")
    print("  - tests/test_queries_1.json")
    print("\nRun with: ./phase1 tests/test_graph_1.json tests/test_queries_1.json tests/output_1.json")
