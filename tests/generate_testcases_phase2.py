#!/usr/bin/env python3
"""
Test case generator for Phase 2 of the Graph Routing System
"""

import json
import random
import math

def generate_phase2_queries(num_nodes=15):
    """Generate Phase 2 test queries"""
    
    queries = []
    query_id = 1
    
    # K shortest paths (exact) query
    queries.append({
        "type": "k_shortest_paths",
        "id": query_id,
        "source": 0,
        "target": num_nodes - 1,
        "k": 5,
        "mode": "distance"
    })
    query_id += 1
    
    # Another k shortest paths query
    queries.append({
        "type": "k_shortest_paths",
        "id": query_id,
        "source": 2,
        "target": num_nodes - 3,
        "k": 3,
        "mode": "distance"
    })
    query_id += 1
    
    # K shortest paths (heuristic) query
    queries.append({
        "type": "k_shortest_paths_heuristic",
        "id": query_id,
        "source": 0,
        "target": num_nodes - 1,
        "k": 5,
        "mode": "distance",
        "overlap_threshold": 30.0
    })
    query_id += 1
    
    # Another heuristic query with different overlap threshold
    queries.append({
        "type": "k_shortest_paths_heuristic",
        "id": query_id,
        "source": 1,
        "target": num_nodes - 2,
        "k": 4,
        "mode": "distance",
        "overlap_threshold": 40.0
    })
    query_id += 1
    
    # Approximate shortest path (batch) query
    batch_queries = []
    for i in range(10):
        src = random.randint(0, num_nodes - 1)
        tgt = random.randint(0, num_nodes - 1)
        if src != tgt:
            batch_queries.append({
                "source": src,
                "target": tgt
            })
    
    queries.append({
        "type": "approx_shortest_path",
        "id": query_id,
        "queries": batch_queries,
        "mode": "distance",
        "time_budget_ms": 100,
        "acceptable_error_pct": 5.0
    })
    query_id += 1
    
    # Another batch query with tighter time budget
    batch_queries_2 = []
    for i in range(20):
        src = random.randint(0, num_nodes - 1)
        tgt = random.randint(0, num_nodes - 1)
        if src != tgt:
            batch_queries_2.append({
                "source": src,
                "target": tgt
            })
    
    queries.append({
        "type": "approx_shortest_path",
        "id": query_id,
        "queries": batch_queries_2,
        "mode": "distance",
        "time_budget_ms": 50,
        "acceptable_error_pct": 10.0
    })
    query_id += 1
    
    return {
        "meta": {
            "id": "phase2_test_queries"
        },
        "events": queries
    }

if __name__ == "__main__":
    print("Generating Phase 2 test queries...")
    queries = generate_phase2_queries(num_nodes=15)
    
    with open("tests/test_queries_phase2.json", "w") as f:
        json.dump(queries, f, indent=2)
    
    print(f"Generated {len(queries['events'])} Phase 2 queries")
    print("\nTest file created:")
    print("  - tests/test_queries_phase2.json")
    print("\nRun with: ./phase2 tests/test_graph_1.json tests/test_queries_phase2.json tests/output_phase2.json")
