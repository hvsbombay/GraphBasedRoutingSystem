#ifndef K_SHORTEST_PATHS_HPP
#define K_SHORTEST_PATHS_HPP

#include "Graph.hpp"
#include <vector>
#include <string>
#include <set>

using namespace std;

struct PathInfo {
    vector<int> path;
    double length;
    double penalty; // For heuristic version
    
    PathInfo() : length(0.0), penalty(0.0) {}
    PathInfo(const vector<int>& p, double l) : path(p), length(l), penalty(0.0) {}
};

class KShortestPaths {
private:
    const Graph& graph;
    
    // Helper: Find shortest path avoiding specific edges
    PathInfo findShortestPath(int source, int target, const string& mode,
                             const set<pair<int, int>>& forbidden_edges);
    
    // Calculate path overlap (for heuristic)
    double calculateOverlap(const vector<int>& path1, const vector<int>& path2);
    
    // Calculate deviation from shortest path (for heuristic)
    double calculateDeviation(double path_length, double shortest_length);
    
public:
    KShortestPaths(const Graph& g) : graph(g) {}
    
    // Yen's algorithm for k shortest paths
    vector<PathInfo> findKShortestPaths(int source, int target, int k, const string& mode);
    
    // Heuristic k shortest paths with diversity
    vector<PathInfo> findKShortestPathsHeuristic(int source, int target, int k,
                                                 double overlap_threshold);
};

#endif // K_SHORTEST_PATHS_HPP
