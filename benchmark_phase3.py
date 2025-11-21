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
    
    results_list = []
    if isinstance(data, dict) and "results" in data:
        results_list = data["results"]
    elif isinstance(data, list):
        results_list = data
    
    results = []
    for query_result in results_list:
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
        quality = scenario['quality']
        if quality is None:
            quality_str = "Inf"
        else:
            quality_str = f"{quality:.1f}"
            
        print(f"{scenario['name']:<25} | {scenario['orders']:<8} | {scenario['drivers']:<8} | "
              f"{scenario['proc_time']:<12.2f} | {quality_str:<12}")
    
    print("="*80)
    print("\nKey Insights:")
    print("- Parallel Portfolio Strategy: Runs 3 advanced algorithms concurrently")
    print("- Genetic Algorithm (GA): Global exploration with population evolution")
    print("- Simulated Annealing (SA): Local optima escape")
    print("- Cluster-First Route-Second: Spatial decomposition")
    print("- The system automatically picks the best result from all parallel threads")
    print("="*80)

def main():
    print("Phase 3 VRP Benchmarking Suite (Parallel Portfolio Mode)\n")
    
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
    
    # Generate medium test case (List of query objects)
    queries_data = [
        {
            "orders": [{"order_id": i, "pickup": i*4, "dropoff": i*4+1} for i in range(15)],
            "fleet": {"num_delievery_guys": 3, "depot_node": 0}
        }
    ]
    
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
    print("Scenario 3: Large Problem (Complex Test Case)...")
    elapsed, success = run_phase3(
        "tests/test_graph_phase3_complex.json",
        "tests/test_queries_phase3_complex.json",
        "bench_p3_large.json"
    )
    if success:
        results = analyze_results("bench_p3_large.json")
        if len(results) > 1:
            # Use the second query which is larger/more complex
            r = results[1]
            scenarios.append({
                "name": "Large (Complex)",
                "orders": r["num_orders"],
                "drivers": r["num_drivers"],
                "proc_time": r["processing_time_ms"],
                "quality": r["total_delivery_time"]
            })
        elif results:
             scenarios.append({
                "name": "Large (Complex)",
                "orders": results[0]["num_orders"],
                "drivers": results[0]["num_drivers"],
                "proc_time": results[0]["processing_time_ms"],
                "quality": results[0]["total_delivery_time"]
            })
    
    # Scenario 4: Massive Problem (10k nodes)
    print("Scenario 4: Massive Problem (10k nodes, 100 orders)...")
    # We assume the files are already generated by generate_large_benchmark.py
    if os.path.exists("benchmark_graph_10k.json"):
        elapsed, success = run_phase3(
            "benchmark_graph_10k.json",
            "benchmark_queries_10k.json",
            "output_10k.json"
        )
        if success:
            results = analyze_results("output_10k.json")
            if results:
                scenarios.append({
                    "name": "Massive (10k nodes)",
                    "orders": results[0]["num_orders"],
                    "drivers": results[0]["num_drivers"],
                    "proc_time": results[0]["processing_time_ms"],
                    "quality": results[0]["total_delivery_time"]
                })
    
    # Print results
    print_benchmark_results(scenarios)
    
    print("\n✅ Advanced Algorithms Implemented:")
    print("   1. Parallel Portfolio Optimization (NEW)")
    print("   2. Genetic Algorithm with Order Crossover (NEW)")
    print("   3. Simulated Annealing (Metaheuristic)")
    print("   4. Clarke-Wright Savings Algorithm")
    print("   5. Cluster-First Route-Second")
    print("   6. 2-opt & Or-opt Local Search")
    print("   7. Multi-threaded Execution Strategy\n")

if __name__ == "__main__":
    main()
