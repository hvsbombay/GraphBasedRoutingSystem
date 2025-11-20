#!/usr/bin/env python3
"""
Phase 3 Benchmark Runner - Demonstrates VRP algorithm performance
Compares different heuristics and problem scales
"""
import json
import subprocess
import time
import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), 'tests'))

def run_phase3(graph_file, query_file, output_file):
    """Run Phase 3 and return execution time."""
    cmd = ["./phase3.exe", graph_file, query_file, output_file] if os.name == 'nt' else \
          ["./phase3", graph_file, query_file, output_file]
    
    start = time.time()
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, check=True, timeout=300)
        elapsed = time.time() - start
        return elapsed, True
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
        return time.time() - start, False

def analyze_results(output_file):
    """Extract metrics from Phase 3 output."""
    with open(output_file) as f:
        data = json.load(f)
    
    results = []
    for query_result in data:
        if "metrics" in query_result and "total_delivery_time_s" in query_result["metrics"]:
            total_time = query_result["metrics"]["total_delivery_time_s"]
            num_drivers = len(query_result.get("assignments", []))
            total_orders = sum(len(d.get("order_ids", [])) for d in query_result.get("assignments", []))
            proc_time = query_result.get("processing_time", 0)
            
            results.append({
                "total_delivery_time": total_time,
                "num_drivers": num_drivers,
                "num_orders": total_orders,
                "processing_time_ms": proc_time
            })
    
    return results

def print_benchmark_results(scenarios):
    """Pretty print benchmark results."""
    print("\n" + "="*80)
    print("PHASE 3 VRP BENCHMARK RESULTS")
    print("="*80)
    print(f"{'Scenario':<25} | {'Orders':<8} | {'Drivers':<8} | {'Proc Time':<12} | {'Quality':<12}")
    print("-"*80)
    
    for scenario in scenarios:
        print(f"{scenario['name']:<25} | {scenario['orders']:<8} | {scenario['drivers']:<8} | "
              f"{scenario['proc_time']:<12.2f} | {scenario['quality']:<12.1f}")
    
    print("="*80)
    print("\nKey Insights:")
    print("- Adaptive algorithm selection based on problem size")
    print("- Small problems (<10 orders): Multiple algorithms + Simulated Annealing")
    print("- Medium problems (10-30 orders): Clarke-Wright Savings + Or-opt")
    print("- Large problems (>30 orders): Cluster-First Route-Second + 2-opt")
    print("="*80)

def main():
    print("Phase 3 VRP Benchmarking Suite\n")
    
    scenarios = []
    
    # Scenario 1: Small problem (existing test)
    print("Scenario 1: Small Problem (5 orders, 2 drivers)...")
    elapsed, success = run_phase3(
        "tests/test_graph_phase3.json",
        "tests/test_queries_phase3.json",
        "bench_p3_small.json"
    )
    if success:
        results = analyze_results("bench_p3_small.json")
        if results:
            scenarios.append({
                "name": "Small (5 orders)",
                "orders": results[0]["num_orders"],
                "drivers": results[0]["num_drivers"],
                "proc_time": results[0]["processing_time_ms"],
                "quality": results[0]["total_delivery_time"]
            })
    
    # Scenario 2: Medium problem
    print("Scenario 2: Medium Problem (15 orders, 3 drivers)...")
    
    # Generate medium test case
    queries_data = {
        "meta": {"id": "medium_bench"},
        "events": [{
            "type": "delivery_scheduling",
            "orders": [{"order_id": i, "pickup": i*4, "dropoff": i*4+1} for i in range(15)],
            "fleet": {"num_delievery_guys": 3, "depot_node": 0}
        }]
    }
    
    with open("bench_queries_medium.json", "w") as f:
        json.dump(queries_data, f)
    
    elapsed, success = run_phase3(
        "tests/test_graph_phase3_complex.json",
        "bench_queries_medium.json",
        "bench_p3_medium.json"
    )
    if success:
        results = analyze_results("bench_p3_medium.json")
        if results:
            scenarios.append({
                "name": "Medium (15 orders)",
                "orders": results[0]["num_orders"],
                "drivers": results[0]["num_drivers"],
                "proc_time": results[0]["processing_time_ms"],
                "quality": results[0]["total_delivery_time"]
            })
    
    # Scenario 3: Large problem (existing complex test)
    print("Scenario 3: Large Problem (20 orders, 5 drivers)...")
    elapsed, success = run_phase3(
        "tests/test_graph_phase3_complex.json",
        "tests/test_queries_phase3_complex.json",
        "bench_p3_large.json"
    )
    if success:
        results = analyze_results("bench_p3_large.json")
        if results:
            scenarios.append({
                "name": "Large (20 orders)",
                "orders": results[0]["num_orders"],
                "drivers": results[0]["num_drivers"],
                "proc_time": results[0]["processing_time_ms"],
                "quality": results[0]["total_delivery_time"]
            })
    
    # Print results
    print_benchmark_results(scenarios)
    
    print("\n✅ Advanced Algorithms Implemented:")
    print("   1. Greedy Assignment (baseline)")
    print("   2. Clarke-Wright Savings Algorithm")
    print("   3. Simulated Annealing (metaheuristic)")
    print("   4. Cluster-First Route-Second (spatial)")
    print("   5. 2-opt Local Search")
    print("   6. Or-opt Move (segment relocation)")
    print("   7. Adaptive Strategy Selection\n")

if __name__ == "__main__":
    main()
