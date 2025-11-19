#!/usr/bin/env python3
"""Generate a richer set of Phase 3 delivery scheduling testcases.
Creates a denser graph and multiple scenarios ranging from simple to extreme loads.
"""

import json
import random
from typing import Dict, List

random.seed(42)

ALLOWED_ROAD_TYPES = [
    "primary",
    "secondary",
    "tertiary",
    "local",
    "expressway",
]
ALLOWED_POIS = [
    "restaurant",
    "petrol station",
    "hospital",
    "pharmacy",
    "hotel",
    "atm",
]


def generate_complex_graph(rows: int = 8, cols: int = 8) -> Dict:
    """Generate a denser grid graph with random shortcuts and one-way edges."""
    nodes: List[Dict] = []
    edges: List[Dict] = []
    node_count = rows * cols

    for r in range(rows):
        for c in range(cols):
            node_id = r * cols + c
            lat = 19.0 + r * 0.004 + random.uniform(-0.0005, 0.0005)
            lon = 72.8 + c * 0.004 + random.uniform(-0.0005, 0.0005)
            pois = []
            if random.random() < 0.5:
                pois.append(random.choice(ALLOWED_POIS))
            if random.random() < 0.15:
                pois.append(random.choice(ALLOWED_POIS))

            nodes.append(
                {
                    "id": node_id,
                    "lat": round(lat, 6),
                    "lon": round(lon, 6),
                    "pois": pois,
                }
            )

            # Connect to right neighbor
            if c < cols - 1:
                length = random.uniform(600, 1500)
                speed = random.uniform(8, 18)
                avg_time = length / speed
                edges.append(
                    {
                        "id": len(edges) + 3000,
                        "u": node_id,
                        "v": node_id + 1,
                        "length": round(length, 2),
                        "average_time": round(avg_time, 2),
                        "oneway": random.choice([False, False, True]),
                        "road_type": random.choice(ALLOWED_ROAD_TYPES),
                    }
                )

            # Connect to down neighbor
            if r < rows - 1:
                length = random.uniform(500, 1400)
                speed = random.uniform(7, 16)
                avg_time = length / speed
                edges.append(
                    {
                        "id": len(edges) + 3000,
                        "u": node_id,
                        "v": node_id + cols,
                        "length": round(length, 2),
                        "average_time": round(avg_time, 2),
                        "oneway": random.choice([False, True]),
                        "road_type": random.choice(ALLOWED_ROAD_TYPES),
                    }
                )

    # Add extra random shortcuts to increase connectivity
    for _ in range(node_count // 2):
        u = random.randint(0, node_count - 1)
        v = random.randint(0, node_count - 1)
        if u == v:
            continue
        length = random.uniform(400, 2000)
        speed = random.uniform(10, 25)
        avg_time = length / speed
        edges.append(
            {
                "id": len(edges) + 3000,
                "u": u,
                "v": v,
                "length": round(length, 2),
                "average_time": round(avg_time, 2),
                "oneway": random.choice([True, False]),
                "road_type": random.choice(ALLOWED_ROAD_TYPES),
            }
        )

    return {
        "meta": {
            "id": "phase3_delivery_graph_complex",
            "nodes": node_count,
            "description": "Complex graph for delivery scheduling validation",
        },
        "nodes": nodes,
        "edges": edges,
    }


def random_orders(num_orders: int, num_nodes: int, start_id: int = 1) -> List[Dict]:
    """Generate a list of orders ensuring pickup != dropoff."""
    orders = []
    for idx in range(num_orders):
        pickup = random.randint(1, num_nodes - 1)
        dropoff = random.randint(1, num_nodes - 1)
        while dropoff == pickup:
            dropoff = random.randint(1, num_nodes - 1)
        orders.append(
            {
                "order_id": start_id + idx,
                "pickup": pickup,
                "dropoff": dropoff,
            }
        )
    return orders


def generate_complex_queries(num_nodes: int) -> List[Dict]:
    """Create a mix of simple, balanced, and stress scenarios."""
    return [
        {
            "name": "baseline",
            "orders": random_orders(5, num_nodes),
            "fleet": {"num_delievery_guys": 2, "depot_node": 0},
        },
        {
            "name": "balanced-medium",
            "orders": random_orders(20, num_nodes, start_id=100),
            "fleet": {"num_delievery_guys": 4, "depot_node": 0},
        },
        {
            "name": "load-heavy",
            "orders": random_orders(35, num_nodes, start_id=500),
            "fleet": {"num_delievery_guys": 3, "depot_node": 0},
        },
        {
            "name": "driver-limited",
            "orders": random_orders(40, num_nodes, start_id=1000),
            "fleet": {"num_delievery_guys": 2, "depot_node": 0},
        },
        {
            "name": "high-fleet",
            "orders": random_orders(30, num_nodes, start_id=2000),
            "fleet": {"num_delievery_guys": 6, "depot_node": 0},
        },
    ]


def main() -> None:
    print("🚀 Generating complex Phase 3 delivery scheduling testcases...")
    graph = generate_complex_graph()
    num_nodes = len(graph["nodes"])

    graph_path = "tests/test_graph_phase3_complex.json"
    queries_path = "tests/test_queries_phase3_complex.json"

    with open(graph_path, "w", encoding="utf-8") as f:
        json.dump(graph, f, indent=2)
    print(f"✅ Wrote {graph_path}: {num_nodes} nodes, {len(graph['edges'])} edges")

    queries = generate_complex_queries(num_nodes)
    with open(queries_path, "w", encoding="utf-8") as f:
        json.dump(queries, f, indent=2)
    print(f"✅ Wrote {queries_path}: {len(queries)} scenarios")

    print("\n📊 Scenario summary:")
    for idx, scenario in enumerate(queries, start=1):
        print(
            f"  {idx}. {scenario['name']}: {len(scenario['orders'])} orders, "
            f"{scenario['fleet']['num_delievery_guys']} driver(s)"
        )

    print("\n✨ Complex Phase 3 testcases generated successfully!")


if __name__ == "__main__":
    main()
