#ifndef APPROX_SHORTEST_PATH_HPP
#define APPROX_SHORTEST_PATH_HPP

#include "Graph.hpp"
#include <vector>
#include <unordered_map>

using namespace std;

class ApproxShortestPath {
private:
    const Graph& graph;
    
    // Landmark-based distance estimation (ALT algorithm)
    vector<int> landmarks;
    unordered_map<int, unordered_map<int, double>> landmark_distances;
    
    void selectLandmarks(int count = 8);
    void computeLandmarkDistances();
    double getLowerBound(int node, int target);
    
    // Bidirectional A* search
    double bidirectionalAStar(int source, int target);
    
public:
    ApproxShortestPath(const Graph& g);
    
    // Initialize approximation structures
    void initialize();
    
    // Single approximate shortest path query
    double findApproxPath(int source, int target, double time_budget_ms, double acceptable_error_pct);
    
    // Batch approximate shortest paths (deprecated - use individual queries)
    vector<double> findApproxPaths(const vector<pair<int, int>>& queries,
                                  double time_budget_ms,
                                  double acceptable_error_pct);
};

#endif // APPROX_SHORTEST_PATH_HPP
