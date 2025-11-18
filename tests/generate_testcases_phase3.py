#!/usr/bin/env python3
"""
Test case generator for Phase 3: Delivery Scheduling
Generates realistic delivery scenarios with varying complexity
"""

import json
import random
from typing import List, Dict, Tuple

# Configuration
ALLOWED_ROAD_TYPES = ["primary", "secondary", "tertiary", "local", "expressway"]
ALLOWED_POIS = ["restaurant", "petrol station", "hospital", "pharmacy", "hotel", "atm"]

def generate_simple_graph() -> Dict:
    """Generate a simple graph for delivery scheduling tests"""
    nodes = []
    edges = []
    
    # Create 20 nodes in a grid-like pattern
    num_nodes = 20
    for i in range(num_nodes):
        lat = 19.0 + (i // 5) * 0.01  # Grid layout
        lon = 72.8 + (i % 5) * 0.01
        
        # Add some POIs (restaurants are common pickup points)
        pois = []
        if random.random() < 0.4:
            pois.append(random.choice(ALLOWED_POIS))
        
        nodes.append({
            "id": i,
            "lat": lat,
            "lon": lon,
            "pois": pois
        })
    
    # Create edges to form connected graph
    edge_id = 2000
    
    # Grid connections (horizontal and vertical)
    for i in range(num_nodes):
        row = i // 5
        col = i % 5
        
        # Horizontal edge
        if col < 4:
            length = random.uniform(800, 1200)
            avg_time = length / random.uniform(8, 12)
            edges.append({
                "id": edge_id,
                "u": i,
                "v": i + 1,
                "length": round(length, 2),
                "average_time": round(avg_time, 2),
                "oneway": False,
                "road_type": random.choice(["primary", "secondary", "local"])
            })
            edge_id += 1
        
        # Vertical edge
        if row < 3:
            length = random.uniform(800, 1200)
            avg_time = length / random.uniform(8, 12)
            edges.append({
                "id": edge_id,
                "u": i,
                "v": i + 5,
                "length": round(length, 2),
                "average_time": round(avg_time, 2),
                "oneway": False,
                "road_type": random.choice(["primary", "secondary", "local"])
            })
            edge_id += 1
    
    # Add some diagonal shortcuts
    for _ in range(5):
        u = random.randint(0, num_nodes - 2)
        v = random.randint(u + 1, num_nodes - 1)
        if u != v:
            length = random.uniform(1000, 1500)
            avg_time = length / random.uniform(10, 15)
            edges.append({
                "id": edge_id,
                "u": u,
                "v": v,
                "length": round(length, 2),
                "average_time": round(avg_time, 2),
                "oneway": random.choice([True, False]),
                "road_type": random.choice(["tertiary", "local"])
            })
            edge_id += 1
    
    return {
        "meta": {
            "id": "phase3_delivery_graph",
            "nodes": num_nodes,
            "description": "Graph for delivery scheduling tests"
        },
        "nodes": nodes,
        "edges": edges
    }


def generate_delivery_queries(num_nodes: int, num_scenarios: int = 5) -> List[Dict]:
    """Generate delivery scheduling queries with varying complexity"""
    queries = []
    
    # Scenario 1: Single driver, few orders
    orders = []
    for i in range(3):
        pickup = random.randint(1, num_nodes - 1)
        dropoff = random.randint(1, num_nodes - 1)
        while dropoff == pickup:
            dropoff = random.randint(1, num_nodes - 1)
        
        orders.append({
            "order_id": i + 1,
            "pickup": pickup,
            "dropoff": dropoff
        })
    
    queries.append({
        "orders": orders,
        "fleet": {
            "num_delievery_guys": 1,
            "depot_node": 0
        }
    })
    
    # Scenario 2: Multiple drivers, moderate orders
    orders = []
    for i in range(8):
        pickup = random.randint(1, num_nodes - 1)
        dropoff = random.randint(1, num_nodes - 1)
        while dropoff == pickup:
            dropoff = random.randint(1, num_nodes - 1)
        
        orders.append({
            "order_id": i + 1,
            "pickup": pickup,
            "dropoff": dropoff
        })
    
    queries.append({
        "orders": orders,
        "fleet": {
            "num_delievery_guys": 3,
            "depot_node": 0
        }
    })
    
    # Scenario 3: Many orders, few drivers (stress test)
    orders = []
    for i in range(15):
        pickup = random.randint(1, num_nodes - 1)
        dropoff = random.randint(1, num_nodes - 1)
        while dropoff == pickup:
            dropoff = random.randint(1, num_nodes - 1)
        
        orders.append({
            "order_id": i + 1,
            "pickup": pickup,
            "dropoff": dropoff
        })
    
    queries.append({
        "orders": orders,
        "fleet": {
            "num_delievery_guys": 2,
            "depot_node": 0
        }
    })
    
    # Scenario 4: Balanced scenario
    orders = []
    for i in range(10):
        pickup = random.randint(1, num_nodes - 1)
        dropoff = random.randint(1, num_nodes - 1)
        while dropoff == pickup:
            dropoff = random.randint(1, num_nodes - 1)
        
        orders.append({
            "order_id": i + 1,
            "pickup": pickup,
            "dropoff": dropoff
        })
    
    queries.append({
        "orders": orders,
        "fleet": {
            "num_delievery_guys": 4,
            "depot_node": 0
        }
    })
    
    # Scenario 5: Single order (edge case)
    queries.append({
        "orders": [
            {
                "order_id": 1,
                "pickup": 5,
                "dropoff": 15
            }
        ],
        "fleet": {
            "num_delievery_guys": 1,
            "depot_node": 0
        }
    })
    
    return queries


def main():
    """Generate and save Phase 3 test cases"""
    print("🚀 Generating Phase 3 delivery scheduling test cases...")
    
    # Generate graph
    graph = generate_simple_graph()
    num_nodes = len(graph["nodes"])
    
    # Save graph
    with open("tests/test_graph_phase3.json", "w") as f:
        json.dump(graph, f, indent=2)
    print(f"✅ Created test_graph_phase3.json: {num_nodes} nodes, {len(graph['edges'])} edges")
    
    # Generate queries
    queries = generate_delivery_queries(num_nodes)
    
    # Save queries
    with open("tests/test_queries_phase3.json", "w") as f:
        json.dump(queries, f, indent=2)
    print(f"✅ Created test_queries_phase3.json: {len(queries)} delivery scenarios")
    
    # Print summary
    print("\n📊 Test Scenarios Summary:")
    for i, query in enumerate(queries, 1):
        num_orders = len(query["orders"])
        num_drivers = query["fleet"]["num_delievery_guys"]
        print(f"   Scenario {i}: {num_orders} orders, {num_drivers} driver(s)")
    
    print("\n✨ Phase 3 test cases generated successfully!")


if __name__ == "__main__":
    main()
