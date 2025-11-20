#include "ApproxShortestPath.hpp"
#include <queue>
#include <algorithm>
#include <limits>
#include <random>
#include <chrono>
#include <cmath>

using namespace std;

ApproxShortestPath::ApproxShortestPath(const Graph& g) : graph(g) {}

void ApproxShortestPath::initialize() {
    selectLandmarks();
    computeLandmarkDistances();
}

void ApproxShortestPath::selectLandmarks(int count) {
    // Select landmarks using farthest-first strategy
    landmarks.clear();
    
    const auto& nodes = graph.getNodes();
    if (nodes.empty()) return;
    
    // Start with a random node
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> dis(0, nodes.size() - 1);
    
    auto it = nodes.begin();
    advance(it, dis(gen));
    landmarks.push_back(it->first);
    
    // Select remaining landmarks as farthest from existing ones
    for (int i = 1; i < count && i < (int)nodes.size(); i++) {
        int farthest_node = -1;
        double max_min_dist = -1;
        
        for (const auto& [node_id, node] : nodes) {
            if (find(landmarks.begin(), landmarks.end(), node_id) != landmarks.end()) {
                continue;
            }
            
            // Find minimum distance to any landmark
            double min_dist = numeric_limits<double>::max();
            for (int landmark : landmarks) {
                const Node* lm_node = graph.getNode(landmark);
                if (lm_node) {
                    double dist = Graph::euclideanDistance(node, *lm_node);
                    min_dist = min(min_dist, dist);
                }
            }
            
            if (min_dist > max_min_dist) {
                max_min_dist = min_dist;
                farthest_node = node_id;
            }
        }
        
        if (farthest_node != -1) {
            landmarks.push_back(farthest_node);
        }
    }
}

void ApproxShortestPath::computeLandmarkDistances() {
    // Run Dijkstra from each landmark to all nodes
    for (int landmark : landmarks) {
        unordered_map<int, double> dist;
        priority_queue<pair<double, int>, vector<pair<double, int>>, greater<>> pq;
        
        dist[landmark] = 0.0;
        pq.push({0.0, landmark});
        
        while (!pq.empty()) {
            auto [d, u] = pq.top();
            pq.pop();
            
            if (d > dist[u]) continue;
            
            for (const auto& [v, edge_id] : graph.getNeighbors(u)) {
                double weight = graph.getEdgeWeight(edge_id, "distance", 0);
                double new_dist = dist[u] + weight;
                
                if (dist.find(v) == dist.end() || new_dist < dist[v]) {
                    dist[v] = new_dist;
                    pq.push({new_dist, v});
                }
            }
        }
        
        landmark_distances[landmark] = dist;
    }
}

double ApproxShortestPath::getLowerBound(int node, int target) {
    // ALT heuristic: max over all landmarks of |d(landmark, target) - d(landmark, node)|
    double max_bound = 0.0;
    
    for (int landmark : landmarks) {
        if (landmark_distances.find(landmark) != landmark_distances.end()) {
            double dist_to_node = landmark_distances[landmark].count(node) 
                                 ? landmark_distances[landmark][node] 
                                 : 1e9;
            double dist_to_target = landmark_distances[landmark].count(target) 
                                   ? landmark_distances[landmark][target] 
                                   : 1e9;
            
            double bound = abs(dist_to_target - dist_to_node);
            max_bound = max(max_bound, bound);
        }
    }
    
    return max_bound;
}

double ApproxShortestPath::bidirectionalAStar(int source, int target, double weight_factor) {
    // Use Weighted A* for speed
    if (source == target) return 0.0;
    
    if (!graph.hasNode(source) || !graph.hasNode(target)) {
        return -1.0;
    }
    
    unordered_map<int, double> dist;
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<>> pq;
    
    dist[source] = 0.0;
    pq.push({0.0, source});
    
    int iterations = 0;
    const int max_iterations = 100000; // Increased limit
    
    while (!pq.empty() && iterations < max_iterations) {
        iterations++;
        
        auto [d, u] = pq.top();
        pq.pop();
        
        double h = getLowerBound(u, target);
        if (d > dist[u] + h * weight_factor + 1e-9) continue;
        
        if (u == target) {
            return dist[target];
        }
        
        for (const auto& [v, edge_id] : graph.getNeighbors(u)) {
            double weight = graph.getEdgeWeight(edge_id, "distance", 0);
            double new_dist = dist[u] + weight;
            
            if (dist.find(v) == dist.end() || new_dist < dist[v]) {
                dist[v] = new_dist;
                double heuristic = getLowerBound(v, target);
                // Weighted A*
                pq.push({new_dist + heuristic * weight_factor, v}); 
            }
        }
    }
    
    return dist.count(target) ? dist[target] : -1.0;
}

double ApproxShortestPath::findApproxPath(int source, int target, double time_budget_ms, double acceptable_error_pct) {
    // Single query with time budget check
    auto start_time = chrono::high_resolution_clock::now();
    
    // Calculate weight factor based on acceptable error
    // e.g., 5% error -> weight 1.05
    double weight = 1.0 + (acceptable_error_pct / 100.0);
    
    // Use weighted A* with landmarks
    double distance = bidirectionalAStar(source, target, weight);
    
    auto end_time = chrono::high_resolution_clock::now();
    double elapsed = chrono::duration<double, milli>(end_time - start_time).count();
    
    // If we're within time budget, return the result
    if (elapsed <= time_budget_ms) {
        return distance;
    }
    
    return distance;
}

vector<double> ApproxShortestPath::findApproxPaths(const vector<pair<int, int>>& queries,
                                                   double time_budget_ms,
                                                   double acceptable_error_pct) {
    vector<double> results;
    
    auto start_time = chrono::high_resolution_clock::now();
    double weight = 1.0 + (acceptable_error_pct / 100.0);
    
    for (const auto& [source, target] : queries) {
        // Check remaining time
        auto current_time = chrono::high_resolution_clock::now();
        double elapsed = chrono::duration<double, milli>(current_time - start_time).count();
        
        if (elapsed >= time_budget_ms * 0.95) {
            // Running out of time, use simple heuristic or ALT lower bound
            double lb = getLowerBound(source, target);
            if (lb > 0) results.push_back(lb);
            else {
                const Node* s_node = graph.getNode(source);
                const Node* t_node = graph.getNode(target);
                if (s_node && t_node) {
                    results.push_back(Graph::euclideanDistance(*s_node, *t_node));
                } else {
                    results.push_back(-1.0);
                }
            }
            continue;
        }
        
        // Use weighted A* with landmarks
        double distance = bidirectionalAStar(source, target, weight);
        results.push_back(distance);
    }
    
    return results;
}
