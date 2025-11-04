#include "ShortestPath.hpp"
#include <queue>
#include <limits>
#include <algorithm>

using namespace std;

PathResult ShortestPath::findPath(int source, int target, const string& mode,
                                  const set<int>& forbidden_nodes,
                                  const set<string>& forbidden_road_types) {
    PathResult result;
    
    // Check if source or target are forbidden
    if (forbidden_nodes.count(source) || forbidden_nodes.count(target)) {
        return result;
    }
    
    // Check if nodes exist
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
            result.possible = true;
            result.distance = dist[target];
            
            int curr = target;
            while (curr != source) {
                result.path.push_back(curr);
                curr = parent[curr];
            }
            result.path.push_back(source);
            reverse(result.path.begin(), result.path.end());
            
            return result;
        }
        
        // Explore neighbors
        for (const auto& [v, edge_id] : graph.getNeighbors(u)) {
            // Skip forbidden nodes
            if (forbidden_nodes.count(v)) continue;
            
            const Edge* edge = graph.getEdge(edge_id);
            if (!edge) continue;
            
            // Skip forbidden road types
            if (forbidden_road_types.count(edge->road_type)) continue;
            
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

PathResult ShortestPath::findTimeDependentPath(int source, int target,
                                               const set<int>& forbidden_nodes,
                                               const set<string>& forbidden_road_types) {
    PathResult result;
    
    // Check if source or target are forbidden
    if (forbidden_nodes.count(source) || forbidden_nodes.count(target)) {
        return result;
    }
    
    // Check if nodes exist
    if (!graph.hasNode(source) || !graph.hasNode(target)) {
        return result;
    }
    
    // Time-dependent Dijkstra
    // State: (time, node_id)
    unordered_map<int, double> dist; // Best time to reach each node
    unordered_map<int, int> parent;
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<>> pq;
    
    dist[source] = 0.0;
    pq.push({0.0, source});
    
    while (!pq.empty()) {
        auto [current_time, u] = pq.top();
        pq.pop();
        
        if (current_time > dist[u]) continue;
        
        if (u == target) {
            // Reconstruct path
            result.possible = true;
            result.distance = dist[target];
            
            int curr = target;
            while (curr != source) {
                result.path.push_back(curr);
                curr = parent[curr];
            }
            result.path.push_back(source);
            reverse(result.path.begin(), result.path.end());
            
            return result;
        }
        
        // Explore neighbors
        for (const auto& [v, edge_id] : graph.getNeighbors(u)) {
            // Skip forbidden nodes
            if (forbidden_nodes.count(v)) continue;
            
            const Edge* edge = graph.getEdge(edge_id);
            if (!edge) continue;
            
            // Skip forbidden road types
            if (forbidden_road_types.count(edge->road_type)) continue;
            
            // Calculate time slot (15-minute intervals)
            int time_slot = static_cast<int>(current_time / 900.0) % 96; // 900 seconds = 15 minutes
            
            double travel_time;
            if (!edge->speed_profile.empty()) {
                double speed = edge->speed_profile[time_slot];
                if (speed > 0) {
                    travel_time = edge->length / speed;
                    
                    // If travel takes more than 15 minutes, account for speed changes
                    double remaining_distance = edge->length;
                    double accumulated_time = 0.0;
                    int current_slot = time_slot;
                    
                    while (remaining_distance > 0 && accumulated_time < 86400) { // Max 1 day
                        double slot_speed = edge->speed_profile[current_slot % 96];
                        if (slot_speed <= 0) slot_speed = edge->length / edge->average_time;
                        
                        double distance_in_slot = slot_speed * 900.0; // Distance covered in 15 min
                        
                        if (distance_in_slot >= remaining_distance) {
                            accumulated_time += remaining_distance / slot_speed;
                            remaining_distance = 0;
                        } else {
                            accumulated_time += 900.0;
                            remaining_distance -= distance_in_slot;
                            current_slot++;
                        }
                    }
                    
                    travel_time = accumulated_time;
                } else {
                    travel_time = edge->average_time;
                }
            } else {
                travel_time = edge->average_time;
            }
            
            double new_time = current_time + travel_time;
            
            if (dist.find(v) == dist.end() || new_time < dist[v]) {
                dist[v] = new_time;
                parent[v] = u;
                pq.push({new_time, v});
            }
        }
    }
    
    return result; // No path found
}
