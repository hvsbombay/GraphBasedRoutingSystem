# Graph-Based Routing System - CS293 Project

## Project Status

### ✅ Completed
- **Phase 1**: FULLY IMPLEMENTED AND TESTED
  - Graph data structure with dynamic updates
  - Shortest path (distance & time modes)
  - Time-dependent routing with speed profiles
  - Path constraints (forbidden nodes/road types)
  - KNN queries (Euclidean & shortest path metrics)
  - Dynamic graph updates (remove/modify edges)

### 🚧 In Progress
- **Phase 2**: Structure created, implementation pending
  - K shortest paths (exact)
  - K shortest paths (heuristic with diversity)
  - Approximate shortest paths (batch queries)

- **Phase 3**: Structure created, implementation pending
  - Delivery scheduling (TSP variant)
  - Pickup/dropoff constraints
  - Multi-driver optimization

## Quick Start

### Build All Phases
```bash
make all          # Build all three phases
make phase1       # Build only Phase 1
make phase2       # Build only Phase 2
make phase3       # Build only Phase 3
```

### Clean Build Artifacts
```bash
make clean        # Clean all
make clean-phase1 # Clean Phase 1 only
```

### Generate Test Cases
```bash
python tests/generate_testcases.py
```

### Run Phase 1
```bash
./phase1.exe tests/test_graph_1.json tests/test_queries_1.json tests/output_1.json
```

## Project Structure

```
CS293_Project/
├── common/
│   ├── json.hpp          # nlohmann/json library
│   ├── Graph.hpp/cpp     # Core graph data structure
│   └── ...
├── Phase-1/
│   ├── main.cpp          # Phase 1 driver
│   ├── ShortestPath.hpp/cpp
│   ├── KNN.hpp/cpp
│   └── ...
├── Phase-2/
│   └── main.cpp          # Phase 2 driver (WIP)
├── Phase-3/
│   └── main.cpp          # Phase 3 driver (WIP)
├── tests/
│   ├── generate_testcases.py
│   ├── test_graph_1.json
│   ├── test_queries_1.json
│   └── output_1.json
├── Makefile
└── README.md
```

## Phase 1 Features

### Implemented Query Types

1. **Shortest Path Queries**
   - Distance minimization
   - Time minimization (with time-dependent speed profiles)
   - Forbidden nodes constraint
   - Forbidden road types constraint
   - Handles 96 × 15-minute speed profile slots

2. **KNN Queries**
   - Euclidean distance metric
   - Shortest path distance metric
   - POI-based search

3. **Dynamic Graph Updates**
   - Remove edge (soft delete, can be restored)
   - Modify edge (update properties or restore deleted edges)

### Algorithms Used

- **Dijkstra's Algorithm**: Standard shortest path
- **Time-Dependent Dijkstra**: Handles speed profiles with 15-min slots
- **Priority Queue Optimization**: O((E + V) log V) complexity
- **POI Indexing**: Fast lookups by POI type

## Input/Output Format

### Graph JSON Format
```json
{
  "meta": {"id": "...", "nodes": 15, "description": "..."},
  "nodes": [
    {"id": 0, "lat": 19.07, "lon": 72.87, "pois": ["Restaurant"]}
  ],
  "edges": [
    {
      "id": 1001,
      "u": 0, "v": 1,
      "length": 100.0,
      "average_time": 5.5,
      "speed_profile": [40, 42, ...],
      "oneway": false,
      "road_type": "primary"
    }
  ]
}
```

### Query JSON Format
```json
{
  "events": [
    {
      "type": "shortest_path",
      "id": 1,
      "source": 0,
      "target": 14,
      "mode": "distance",
      "constraints": {
        "forbidden_nodes": [2, 3],
        "forbidden_road_types": ["local"]
      }
    }
  ]
}
```

### Output JSON Format
```json
[
  {
    "id": 1,
    "possible": true,
    "minimum_distance": 6537.79,
    "path": [0, 2, 3, 13, 14],
    "processing_time": 0.0
  }
]
```

## Development Timeline

- **Nov 4, 2025**: Phase 1 implementation completed ✅
- **Nov 5-7, 2025**: Phase 2 implementation
- **Nov 8-9, 2025**: Testing and optimization (Phases 1 & 2)
- **Nov 10, 2025**: **DEADLINE - Phases 1 & 2**
- **Nov 11-20, 2025**: Phase 3 implementation
- **Nov 21, 2025**: Final testing and report
- **Nov 22, 2025**: **DEADLINE - Phase 3**

## Testing

Current test suite includes:
- 15-node connected graph
- 29 edges with realistic properties
- 9 diverse query types
- Dynamic update scenarios

To create additional test cases, modify `tests/generate_testcases.py`.

## Next Steps

1. **Phase 2 Implementation** (Priority: HIGH)
   - Implement Yen's K-Shortest Paths algorithm
   - Design path diversity heuristics
   - Optimize batch query processing with time budgets

2. **Enhanced Testing**
   - Generate larger graphs (100-1000 nodes)
   - Stress test with edge cases
   - Benchmark performance

3. **Phase 3 Implementation**
   - Research TSP heuristics (2-opt, Christofides, genetic algorithms)
   - Implement pickup/dropoff constraints
   - Compare total vs max delivery time objectives

## Notes

- All code follows modular design principles
- Clear separation between data structures and algorithms
- Extensive comments for maintainability
- Windows-compatible Makefile
- JSON library: nlohmann/json v3.11.3

## AI Assistance Log

This project utilizes AI assistance (GitHub Copilot). All interactions are logged for the report as per project requirements.
