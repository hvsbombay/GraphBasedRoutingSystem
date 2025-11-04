#include "KShortestPaths.hpp"
#include <queue>
#include <algorithm>
#include <limits>
#include <unordered_set>

using namespace std;

PathInfo KShortestPaths::findShortestPath(int source, int target, const string& mode,
                                         const set<pair<int, int>>& forbidden_edges) {
    PathInfo result;
    
    if (!graph.hasNode(source) || !graph.hasNode(target)) {
        return result;
    }
    
    // Dijkstra's algorithm
    unordered_map<int, double> dist;
    unordered_map<int, int> parent;
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<>> pq;
    
    dist[source] = 0.0;
    pq.push({0.0, source});
    
    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        
        if (d > dist[u]) continue;
        
        if (u == target) {
            // Reconstruct path
            vector<int> path;
            int curr = target;
            while (curr != source) {
                path.push_back(curr);
                curr = parent[curr];
            }
            path.push_back(source);
            reverse(path.begin(), path.end());
            
            result.path = path;
            result.length = dist[target];
            return result;
        }
        
        // Explore neighbors
        for (const auto& [v, edge_id] : graph.getNeighbors(u)) {
            // Check if this edge is forbidden
            if (forbidden_edges.count({u, v})) continue;
            
            double weight = graph.getEdgeWeight(edge_id, mode, 0);
            double new_dist = dist[u] + weight;
            
            if (dist.find(v) == dist.end() || new_dist < dist[v]) {
                dist[v] = new_dist;
                parent[v] = u;
                pq.push({new_dist, v});
            }
        }
    }
    
    return result; // No path found
}

vector<PathInfo> KShortestPaths::findKShortestPaths(int source, int target, int k, const string& mode) {
    vector<PathInfo> result;
    
    // Yen's algorithm
    // A: List of shortest paths found
    // B: Candidates for next shortest path
    
    // Find the shortest path
    PathInfo shortest = findShortestPath(source, target, mode, {});
    
    if (shortest.path.empty()) {
        return result; // No path exists
    }
    
    result.push_back(shortest);
    
    // Priority queue for candidate paths
    auto cmp = [](const PathInfo& a, const PathInfo& b) {
        return a.length > b.length;
    };
    priority_queue<PathInfo, vector<PathInfo>, decltype(cmp)> candidates(cmp);
    
    for (int k_iter = 1; k_iter < k; k_iter++) {
        const PathInfo& prev_path = result.back();
        
        // For each node in the previous path except the target
        for (size_t i = 0; i < prev_path.path.size() - 1; i++) {
            int spur_node = prev_path.path[i];
            vector<int> root_path(prev_path.path.begin(), prev_path.path.begin() + i + 1);
            
            // Edges to be removed
            set<pair<int, int>> forbidden_edges;
            
            // For each path in A that shares the same root path
            for (const auto& p : result) {
                if (p.path.size() > i) {
                    bool same_root = true;
                    for (size_t j = 0; j <= i; j++) {
                        if (p.path[j] != root_path[j]) {
                            same_root = false;
                            break;
                        }
                    }
                    
                    if (same_root && p.path.size() > i + 1) {
                        // Remove the edge from spur_node to next node in this path
                        forbidden_edges.insert({p.path[i], p.path[i + 1]});
                    }
                }
            }
            
            // Also remove nodes in root path (except spur_node) by forbidding all their edges
            for (size_t j = 0; j < i; j++) {
                int node = root_path[j];
                for (const auto& [neighbor, edge_id] : graph.getNeighbors(node)) {
                    forbidden_edges.insert({node, neighbor});
                }
            }
            
            // Find shortest path from spur_node to target with forbidden edges
            PathInfo spur_path = findShortestPath(spur_node, target, mode, forbidden_edges);
            
            if (!spur_path.path.empty()) {
                // Combine root path and spur path
                PathInfo total_path;
                total_path.path = root_path;
                total_path.path.insert(total_path.path.end(), 
                                      spur_path.path.begin() + 1, 
                                      spur_path.path.end());
                
                // Calculate total length
                total_path.length = 0.0;
                for (size_t j = 0; j < total_path.path.size() - 1; j++) {
                    int u = total_path.path[j];
                    int v = total_path.path[j + 1];
                    
                    // Find edge between u and v
                    for (const auto& [neighbor, edge_id] : graph.getNeighbors(u)) {
                        if (neighbor == v) {
                            total_path.length += graph.getEdgeWeight(edge_id, mode, 0);
                            break;
                        }
                    }
                }
                
                candidates.push(total_path);
            }
        }
        
        if (candidates.empty()) {
            break; // No more paths
        }
        
        // Add the best candidate to result
        PathInfo best_candidate = candidates.top();
        candidates.pop();
        
        // Check for duplicates
        bool is_duplicate = false;
        for (const auto& p : result) {
            if (p.path == best_candidate.path) {
                is_duplicate = true;
                break;
            }
        }
        
        if (!is_duplicate) {
            result.push_back(best_candidate);
        } else {
            k_iter--; // Don't count duplicates
        }
    }
    
    return result;
}

double KShortestPaths::calculateOverlap(const vector<int>& path1, const vector<int>& path2) {
    unordered_set<int> nodes1(path1.begin(), path1.end());
    int overlap_count = 0;
    
    for (int node : path2) {
        if (nodes1.count(node)) {
            overlap_count++;
        }
    }
    
    // Normalize by the size of the smaller path
    return (double)overlap_count / min(path1.size(), path2.size());
}

double KShortestPaths::calculateDeviation(double path_length, double shortest_length) {
    if (shortest_length == 0.0) return 0.0;
    return (path_length - shortest_length) / shortest_length;
}

vector<PathInfo> KShortestPaths::findKShortestPathsHeuristic(int source, int target, int k,
                                                             double overlap_penalty,
                                                             double distance_penalty) {
    // First, find many candidate paths using Yen's algorithm
    int candidate_count = min(k * 3, 50); // Get more candidates
    vector<PathInfo> candidates = findKShortestPaths(source, target, candidate_count, "distance");
    
    if (candidates.empty()) {
        return vector<PathInfo>();
    }
    
    double shortest_length = candidates[0].length;
    
    // Calculate penalties for each candidate
    for (size_t i = 1; i < candidates.size(); i++) {
        double overlap_score = 0.0;
        
        // Calculate overlap with the shortest path
        overlap_score = calculateOverlap(candidates[0].path, candidates[i].path);
        
        // Calculate deviation from shortest path
        double deviation = calculateDeviation(candidates[i].length, shortest_length);
        
        // Combined penalty: lower is better
        candidates[i].penalty = overlap_penalty * overlap_score + distance_penalty * deviation;
    }
    
    // Sort by penalty (ascending - lower penalty is better)
    sort(candidates.begin() + 1, candidates.end(), 
         [](const PathInfo& a, const PathInfo& b) {
             return a.penalty < b.penalty;
         });
    
    // Return top k
    vector<PathInfo> result;
    for (int i = 0; i < min(k, (int)candidates.size()); i++) {
        result.push_back(candidates[i]);
    }
    
    return result;
}
