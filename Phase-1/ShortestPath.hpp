#ifndef SHORTEST_PATH_HPP
#define SHORTEST_PATH_HPP

#include "Graph.hpp"
#include <vector>
#include <string>
#include <set>

using namespace std;

struct PathResult {
    bool possible;
    double distance;
    vector<int> path;
    
    PathResult() : possible(false), distance(0.0) {}
};

class ShortestPath {
private:
    const Graph& graph;
    
public:
    ShortestPath(const Graph& g) : graph(g) {}
    
    // Standard shortest path with constraints
    PathResult findPath(int source, int target, const string& mode,
                       const set<int>& forbidden_nodes = {},
                       const set<string>& forbidden_road_types = {});
    
    // Time-dependent shortest path (for speed profiles)
    PathResult findTimeDependentPath(int source, int target,
                                    const set<int>& forbidden_nodes = {},
                                    const set<string>& forbidden_road_types = {});
};

#endif // SHORTEST_PATH_HPP
