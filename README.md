# Graph-Based Routing System - CS293 Project

## Project Status (Updated: Nov 7, 2025)

### ✅ Completed & Updated to New Specification
- **Phase 1**: FULLY UPDATED ✅
  - Output format changed to `{meta: {...}, results: [...]}`
  - Try-catch error handling for all queries
  - Correct field names: `minimum_time` or `minimum_distance` (not both)
  - `remove_edge` returns false if already deleted
  - `modify_edge` logic updated per specification

- **Phase 2**: FULLY UPDATED ✅
  - Output format changed to `{meta: {...}, results: [...]}`
  - Try-catch error handling for all queries
  - `approx_shortest_path` processes queries ONE BY ONE with time budget check
  - Output field changed to `approx_shortest_distance`
  - Heuristic k-shortest paths updated with `overlap_threshold` parameter
  - Edge overlap calculation (not node overlap)

### 🚧 Pending
- **Phase 3**: Structure created, implementation pending
  - TSP delivery scheduling
  - Can ignore speed profiles, use average_time only
  - Pickup/dropoff constraints
  - Multi-driver optimization

## Key Changes from Original Specification

### Output Format
**OLD:** Direct array `[{result1}, {result2}, ...]`  
**NEW:** Object with meta and results:
```json
{
  "meta": { "id": "qset1" },
  "results": [
    {"id": 1, "possible": true, "minimum_distance": 123, "path": [...], "processing_time": 0.5},
    {"id": 2, "nodes": [...], "processing_time": 0.3}
  ]
}
```

### Error Handling
**NEW REQUIREMENT:** All queries must be wrapped in try-catch blocks. If a query fails, return error result and continue processing remaining queries.

### Phase 2 Changes
- **Approximate shortest paths**: Must process queries **one by one**, checking time budget before each query. If budget exceeded, reject remaining queries.
- **Output field**: Changed from `distance` to `approx_shortest_distance`
- **Heuristic k-shortest paths**: Uses `overlap_threshold` (percentage) to calculate penalties

### Dynamic Edge Updates
- `remove_edge`: Returns `false` if edge already deleted
- `modify_edge`: 
  - If edge deleted: restore with patch (or previous values if patch empty)
  - If edge exists and patch empty: return `false`
  - If edge doesn't exist: return `false`

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
├── Phase-1/                      # Phase 1: Shortest Paths & KNN
│   ├── main.cpp                  # Driver program
│   ├── Graph.hpp/cpp             # Graph data structure
│   ├── ShortestPath.hpp/cpp      # Dijkstra algorithms
│   ├── KNN.hpp/cpp               # K-nearest neighbors
│   └── json.hpp                  # JSON library
├── Phase-2/                      # Phase 2: K-Shortest Paths & Approximate
│   ├── main.cpp                  # Driver program
│   ├── Graph.hpp/cpp             # Graph data structure
│   ├── KShortestPaths.hpp/cpp    # Yen's algorithm & heuristics
│   ├── ApproxShortestPath.hpp/cpp # A* with landmarks (ALT)
│   └── json.hpp                  # JSON library
├── Phase-3/                      # Phase 3: TSP Delivery Scheduling
│   ├── main.cpp                  # Driver program (skeleton)
│   ├── Graph.hpp/cpp             # Graph data structure
│   └── json.hpp                  # JSON library
├── tests/
│   ├── generate_testcases.py     # Phase 1 test generator
│   └── generate_testcases_phase2.py # Phase 2 test generator
├── Makefile                      # Build system (Windows-compatible)
├── SampleDriver.cpp              # Reference implementation
└── README.md
```

**Note**: Each phase is self-contained with all necessary files. No shared `common/` directory as per project specification.

## Implemented Features

### Phase 1: Shortest Paths & KNN

**Query Types:**
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

**Algorithms:**
- Dijkstra's Algorithm (distance mode)
- Time-Dependent Dijkstra (speed profiles)
- Priority Queue: O((E + V) log V) complexity
- POI Indexing: O(1) lookups by type

### Phase 2: Multiple Paths & Batch Processing

**Query Types:**
1. **K-Shortest Paths (Exact)**
   - Yen's algorithm for finding K distinct paths
   - Guarantees non-overlapping paths

2. **K-Shortest Paths (Heuristic)**
   - Path diversity optimization
   - Overlap and deviation penalties
   - Faster than exact for large K

3. **Approximate Shortest Paths (Batch)**
   - Processes multiple queries with time budgets
   - Landmark-based heuristics (ALT algorithm)
   - Iteration limits for guaranteed termination

**Algorithms:**
- Yen's K-Shortest Paths: O(K × N(E + V log V))
- A* with Landmarks (ALT): O(E + V log V) with precomputed landmarks
- Heuristic Path Scoring: Overlap ratio + distance deviation
- Batch Processing: Time budget management (e.g., 1000ms for 20 queries)

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

- **Nov 4, 2025**: ✅ Phase 1 & 2 initial implementation completed
- **Nov 7, 2025**: ✅ **MAJOR UPDATE** - All code updated to match ProjectChanged.md specification
  - Output format changed to `{meta, results}`
  - Try-catch error handling added
  - Field names corrected
  - Approximate query processing fixed
- **Nov 8-9, 2025**: Phase 3 implementation (TSP delivery scheduling)
- **Nov 10, 2025**: **DEADLINE - Phases 1 & 2** ⏰
- **Nov 11-21, 2025**: Phase 3 testing, optimization, and report writing
- **Nov 22, 2025**: **DEADLINE - Phase 3** ⏰

## Grading Information

### Phase 1 (5 marks)
- **70% weightage**: Accuracy under reasonable execution time
- **30% weightage**: Execution time (relatively graded, linear scale between max/min times)
- Focus on correct and highly efficient algorithms

### Phase 2 (7 marks)
- Similar balance: Accuracy (70%) + Efficiency (30%)
- Heuristic evaluation: Relative grading and threshold-based assessment
- Quantitative results and qualitative heuristics matter

### Phase 3 (8 marks)
- Partially relatively graded
- Major marks for:
  - Depth of exploration and experimentation
  - Originality of ideas
  - Quality and comprehensiveness of report

**Total: 20 marks**

## Testing

Current test suite includes:
- 15-node connected graph
- 29 edges with realistic properties
- 9 diverse query types
- Dynamic update scenarios

To create additional test cases, modify `tests/generate_testcases.py`.

## Next Steps

1. **Phase 3 Implementation** (Priority: URGENT - 6 days to deadline)
   - Implement TSP delivery scheduling algorithms
   - Greedy nearest neighbor baseline
   - 2-opt local search optimization
   - Handle pickup/dropoff constraints
   - Multiple delivery driver assignment
   - Compare total vs max delivery time objectives

2. **Enhanced Testing**
   - Generate larger graphs (1,000-10,000 nodes)
   - Real-world map extraction from OpenStreetMap
   - Stress test with diverse delivery scenarios
   - Performance benchmarking

3. **Project Report**
   - Document implementation details
   - Time and space complexity analysis
   - Real-world test case analysis
   - Include AI interaction logs

## Notes

- All code follows modular design principles
- Clear separation between data structures and algorithms
- Extensive comments for maintainability
- Windows-compatible Makefile
- JSON library: nlohmann/json v3.11.3

## AI Assistance Log

This project utilizes AI assistance (GitHub Copilot). All interactions are logged for the report as per project requirements.
