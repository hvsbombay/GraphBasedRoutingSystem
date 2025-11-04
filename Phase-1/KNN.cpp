#include "KNN.hpp"
#include <queue>
#include <algorithm>
#include <limits>

using namespace std;

vector<int> KNN::findKNearestEuclidean(double query_lat, double query_lon,
                                       const string& poi_type, int k) {
    vector<int> poi_nodes = graph.getNodesByPOI(poi_type);
    
    if (poi_nodes.empty()) {
        return vector<int>();
    }
    
    // Calculate distances and sort
    vector<pair<double, int>> distances;
    
    for (int node_id : poi_nodes) {
        const Node* node = graph.getNode(node_id);
        if (node) {
            double dist = Graph::euclideanDistance(query_lat, query_lon, node->lat, node->lon);
            distances.push_back({dist, node_id});
        }
    }
    
    // Sort by distance
    sort(distances.begin(), distances.end());
    
    // Take top k
    vector<int> result;
    for (int i = 0; i < min(k, (int)distances.size()); i++) {
        result.push_back(distances[i].second);
    }
    
    return result;
}

vector<int> KNN::findKNearestShortestPath(double query_lat, double query_lon,
                                         const string& poi_type, int k) {
    vector<int> poi_nodes = graph.getNodesByPOI(poi_type);
    
    if (poi_nodes.empty()) {
        return vector<int>();
    }
    
    // Find nearest node to query point
    int nearest_node = -1;
    double min_euclidean = numeric_limits<double>::max();
    
    for (const auto& [node_id, node] : graph.getNodes()) {
        double dist = Graph::euclideanDistance(query_lat, query_lon, node.lat, node.lon);
        if (dist < min_euclidean) {
            min_euclidean = dist;
            nearest_node = node_id;
        }
    }
    
    if (nearest_node == -1) {
        return vector<int>();
    }
    
    // Run Dijkstra from nearest node to find shortest paths to all POI nodes
    unordered_map<int, double> dist;
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<>> pq;
    
    dist[nearest_node] = 0.0;
    pq.push({0.0, nearest_node});
    
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
    
    // Collect distances to POI nodes
    vector<pair<double, int>> poi_distances;
    
    for (int poi_node : poi_nodes) {
        if (dist.find(poi_node) != dist.end()) {
            poi_distances.push_back({dist[poi_node], poi_node});
        }
    }
    
    // Sort by distance
    sort(poi_distances.begin(), poi_distances.end());
    
    // Take top k
    vector<int> result;
    for (int i = 0; i < min(k, (int)poi_distances.size()); i++) {
        result.push_back(poi_distances[i].second);
    }
    
    return result;
}
