#!/usr/bin/env python3
import json
import subprocess
import time
import os
import sys
import random
import math

# Add tests directory to path to import generator
sys.path.append(os.path.join(os.path.dirname(__file__), 'tests'))
from generate_testcases import generate_simple_graph

def generate_benchmark_data(num_nodes=1000, num_edges=3000, num_queries=50):
    print(f"Generating benchmark graph with {num_nodes} nodes and {num_edges} edges...")
    graph_data = generate_simple_graph(num_nodes, num_edges)
    
    graph_file = "benchmark_graph.json"
    with open(graph_file, "w") as f:
        json.dump(graph_data, f, indent=2)
        
    print(f"Generating {num_queries} random queries...")
    pairs = []
    for i in range(num_queries):
        src = random.randint(0, num_nodes - 1)
        tgt = random.randint(0, num_nodes - 1)
        while src == tgt:
            tgt = random.randint(0, num_nodes - 1)
        pairs.append((src, tgt))
            
    # Phase 1 Queries (Individual shortest_path events)
    p1_queries = []
    for i, (src, tgt) in enumerate(pairs):
        p1_queries.append({
            "type": "shortest_path",
            "id": i + 1,
            "source": src,
            "target": tgt,
            "mode": "distance"
        })
        
    p1_file = "bench_queries_p1.json"
    with open(p1_file, "w") as f:
        json.dump({"meta": {"id": "p1_bench"}, "events": p1_queries}, f, indent=2)
        
    # Phase 2 Queries (Single batch approx_shortest_path event)
    p2_batch = [{"source": src, "target": tgt} for src, tgt in pairs]
    p2_queries = [{
        "type": "approx_shortest_path",
        "id": 999,
        "queries": p2_batch,
        "mode": "distance",
        "time_budget_ms": 60000, # Generous budget to ensure all run
        "acceptable_error_pct": 20.0
    }]
    
    p2_file = "bench_queries_p2.json"
    with open(p2_file, "w") as f:
        json.dump({"meta": {"id": "p2_bench"}, "events": p2_queries}, f, indent=2)
        
    return graph_file, p1_file, p2_file, pairs

def run_phase(executable, graph_file, query_file, output_file):
    print(f"Running {executable}...")
    start_time = time.time()
    
    # Run the command
    cmd = [f"./{executable}", graph_file, query_file, output_file]
    if os.name == 'nt': # Windows
        cmd = [f"{executable}.exe", graph_file, query_file, output_file]
        
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        return time.time() - start_time
    except subprocess.CalledProcessError as e:
        print(f"Error running {executable}: {e}")
        print(e.stdout)
        print(e.stderr)
        return 0.0

def analyze_results(p1_out, p2_out, pairs):
    # Load P1 Results
    with open(p1_out) as f:
        p1_data = json.load(f)
    p1_results_list = p1_data.get("results", []) if isinstance(p1_data, dict) else p1_data
    
    # Map P1 results by ID
    p1_map = {}
    for res in p1_results_list:
        if "id" in res and "minimum_distance" in res:
            p1_map[res["id"]] = res["minimum_distance"]
            
    # Load P2 Results
    with open(p2_out) as f:
        p2_data = json.load(f)
    p2_results_list = p2_data.get("results", []) if isinstance(p2_data, dict) else p2_data
    
    # Extract P2 distances from the single batch result
    p2_dists = []
    if p2_results_list and "distances" in p2_results_list[0]:
        p2_dists = p2_results_list[0]["distances"]
        
    # Map P2 results by (source, target) - assuming order is preserved but let's be safe
    # Actually, the output list order matches input list order usually, but let's use a map key
    p2_map = {}
    for d in p2_dists:
        key = (d["source"], d["target"])
        p2_map[key] = d["approx_shortest_distance"]
        
    total_error = 0.0
    count = 0
    
    print("\n" + "="*70)
    print(f"{'ID':<5} | {'Source':<8} | {'Target':<8} | {'Exact':<10} | {'Approx':<10} | {'Error %':<10}")
    print("-" * 70)
    
    for i, (src, tgt) in enumerate(pairs):
        qid = i + 1
        d1 = p1_map.get(qid, -1)
        d2 = p2_map.get((src, tgt), -1)
        
        if d1 == -1 or d2 == -1:
            continue
            
        if d1 == 0:
            error = 0.0
        else:
            error = abs(d2 - d1) / d1 * 100.0
            
        total_error += error
        count += 1
        
        if count <= 15: # Print first 15
            print(f"{qid:<5} | {src:<8} | {tgt:<8} | {d1:<10.2f} | {d2:<10.2f} | {error:<10.2f}")
            
    avg_error = total_error / count if count > 0 else 0.0
    print("-" * 70)
    return avg_error

def main():
    # 1. Generate Data
    graph_file, p1_file, p2_file, pairs = generate_benchmark_data(num_nodes=1000, num_edges=3000, num_queries=100)
    
    # 2. Run Phase 1 (Exact)
    p1_time = run_phase("phase1", graph_file, p1_file, "bench_p1.json")
    
    # 3. Run Phase 2 (Approx)
    p2_time = run_phase("phase2", graph_file, p2_file, "bench_p2.json")
    
    # 4. Analyze
    avg_error = analyze_results("bench_p1.json", "bench_p2.json", pairs)
    
    print("\n" + "="*40)
    print("BENCHMARK RESULTS (1000 Nodes, 100 Queries)")
    print("="*40)
    print(f"Phase 1 (Exact) Total Time : {p1_time:.4f} s")
    print(f"Phase 2 (Approx) Total Time: {p2_time:.4f} s")
    print(f"Speedup                    : {p1_time / p2_time:.2f}x")
    print(f"Average Path Error         : {avg_error:.2f}%")
    print("="*40)
    
    # Cleanup
    # os.remove(graph_file)
    # os.remove(p1_file)
    # os.remove(p2_file)
    # os.remove("bench_p1.json")
    # os.remove("bench_p2.json")

if __name__ == "__main__":
    main()
