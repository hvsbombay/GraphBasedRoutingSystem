#!/usr/bin/env python3
"""
Test case generator for Phase 1 of the Graph Routing System.
Generates JSON inputs that comply with the latest ProjectChanged.md spec.
"""

import json
import random
import math
from typing import Dict, List, Set, Tuple

ALLOWED_POIS = ["restaurant", "petrol station", "hospital", "pharmacy", "hotel", "atm"]
ROAD_TYPES = ["primary", "secondary", "tertiary", "local", "expressway"]


def _compute_length(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Approximate great-circle distance in meters for small regions."""

    dlat = (lat2 - lat1) * 111_000.0
    dlon = (lon2 - lon1) * 111_000.0 * math.cos(math.radians((lat1 + lat2) / 2.0))
    return math.hypot(dlat, dlon)


def _generate_speed_profile(base_speed: float) -> List[float]:
    """Return a 96-slot speed profile around the supplied base speed (m/s)."""

    profile = []
    for slot in range(96):
        fluctuation = random.uniform(-0.25, 0.25)
        slot_speed = max(5.0, base_speed * (1.0 + fluctuation))
        profile.append(round(slot_speed, 2))
    return profile


def generate_simple_graph(num_nodes: int = 10, num_edges: int = 20) -> Dict[str, object]:
    """Generate a simple connected graph that respects the project specification."""

    base_lat = 19.070
    base_lon = 72.870

    nodes: List[Dict[str, object]] = []
    for node_id in range(num_nodes):
        lat = base_lat + (node_id % 5) * 0.01
        lon = base_lon + (node_id // 5) * 0.01

        poi_count = random.randint(0, 2)
        pois = random.sample(ALLOWED_POIS, poi_count)

        nodes.append({
            "id": node_id,
            "lat": round(lat, 6),
            "lon": round(lon, 6),
            "pois": pois,
        })

    edges: List[Dict[str, object]] = []
    edge_id = 1000
    seen_pairs: Set[Tuple[int, int]] = set()

    def add_edge(u: int, v: int) -> None:
        nonlocal edge_id
        length = _compute_length(nodes[u]["lat"], nodes[u]["lon"], nodes[v]["lat"], nodes[v]["lon"])
        base_speed = random.uniform(8.0, 22.0)
        avg_time = length / base_speed if base_speed > 0 else length / 10.0
        road_type = random.choice(ROAD_TYPES)

        is_oneway = random.random() < 0.3

        edge: Dict[str, object] = {
            "id": edge_id,
            "u": u,
            "v": v,
            "length": round(length, 2),
            "average_time": round(avg_time, 2),
            "oneway": is_oneway,
            "road_type": road_type,
        }

        # Attach a 96-slot profile for a subset of edges (Phase 1 requirement).
        if road_type in {"primary", "expressway"}:
            edge["speed_profile"] = _generate_speed_profile(base_speed)

        edges.append(edge)
        seen_pairs.add((min(u, v), max(u, v)))
        edge_id += 1

    # Spanning tree ensures connectivity.
    for node_id in range(1, num_nodes):
        parent = random.randint(0, node_id - 1)
        add_edge(parent, node_id)

    # Add remaining edges while respecting the simple-graph constraint.
    attempts = 0
    target_edges = max(num_edges, num_nodes - 1)
    while len(edges) < target_edges and attempts < num_nodes * num_nodes:
        u = random.randint(0, num_nodes - 1)
        v = random.randint(0, num_nodes - 1)
        attempts += 1

        if u == v:
            continue
        if (min(u, v), max(u, v)) in seen_pairs:
            continue

        add_edge(u, v)

    graph = {
        "meta": {
            "id": "test_graph_1",
            "nodes": num_nodes,
            "description": "Synthetic test graph for Phase 1",
        },
        "nodes": nodes,
        "edges": edges,
    }

    return graph

def generate_phase1_queries(num_nodes: int = 10) -> Dict[str, object]:
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
