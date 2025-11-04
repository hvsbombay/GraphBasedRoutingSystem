#include "Graph.hpp"
#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;

Graph::Graph() {}

void Graph::addNode(const Node& node) {
    nodes[node.id] = node;
    
    // Index POIs
    for (const auto& poi : node.pois) {
        string poi_lower = poi;
        transform(poi_lower.begin(), poi_lower.end(), poi_lower.begin(), ::tolower);
        poi_index[poi_lower].insert(node.id);
    }
}

void Graph::addEdge(const Edge& edge) {
    edges[edge.id] = edge;
}

void Graph::finalize() {
    buildAdjacencyList();
}

void Graph::buildAdjacencyList() {
    adj.clear();
    
    for (const auto& [edge_id, edge] : edges) {
        if (edge.deleted) continue;
        
        adj[edge.u].push_back({edge.v, edge_id});
        
        // If not one-way, add reverse edge
        if (!edge.oneway) {
            adj[edge.v].push_back({edge.u, edge_id});
        }
    }
}

void Graph::rebuildAdjacencyList() {
    buildAdjacencyList();
}

bool Graph::removeEdge(int edge_id) {
    if (edges.find(edge_id) != edges.end()) {
        edges[edge_id].deleted = true;
        deleted_edges[edge_id] = edges[edge_id]; // Store for potential restoration
        rebuildAdjacencyList();
        return true;
    }
    return false;
}

bool Graph::modifyEdge(int edge_id, const Edge& patch) {
    // Case 1: Edge exists and is deleted -> restore with patch
    if (edges.find(edge_id) != edges.end() && edges[edge_id].deleted) {
        Edge& edge = edges[edge_id];
        edge.deleted = false;
        
        // Apply patch values
        if (patch.length > 0) edge.length = patch.length;
        if (patch.average_time > 0) edge.average_time = patch.average_time;
        if (!patch.speed_profile.empty()) edge.speed_profile = patch.speed_profile;
        if (!patch.road_type.empty()) edge.road_type = patch.road_type;
        // oneway is kept from original unless explicitly patched
        
        rebuildAdjacencyList();
        return true;
    }
    
    // Case 2: Edge exists and is not deleted -> modify
    if (edges.find(edge_id) != edges.end() && !edges[edge_id].deleted) {
        Edge& edge = edges[edge_id];
        
        // Apply patch values
        if (patch.length > 0) edge.length = patch.length;
        if (patch.average_time > 0) edge.average_time = patch.average_time;
        if (!patch.speed_profile.empty()) edge.speed_profile = patch.speed_profile;
        if (!patch.road_type.empty()) edge.road_type = patch.road_type;
        
        rebuildAdjacencyList();
        return true;
    }
    
    // Case 3: Edge was in deleted_edges and not in edges -> restore
    if (deleted_edges.find(edge_id) != deleted_edges.end()) {
        Edge restored = deleted_edges[edge_id];
        restored.deleted = false;
        
        // Apply patch values
        if (patch.length > 0) restored.length = patch.length;
        if (patch.average_time > 0) restored.average_time = patch.average_time;
        if (!patch.speed_profile.empty()) restored.speed_profile = patch.speed_profile;
        if (!patch.road_type.empty()) restored.road_type = patch.road_type;
        
        edges[edge_id] = restored;
        rebuildAdjacencyList();
        return true;
    }
    
    return false;
}

const Node* Graph::getNode(int node_id) const {
    auto it = nodes.find(node_id);
    return (it != nodes.end()) ? &it->second : nullptr;
}

const Edge* Graph::getEdge(int edge_id) const {
    auto it = edges.find(edge_id);
    return (it != edges.end() && !it->second.deleted) ? &it->second : nullptr;
}

const vector<pair<int, int>>& Graph::getNeighbors(int node_id) const {
    static const vector<pair<int, int>> empty;
    auto it = adj.find(node_id);
    return (it != adj.end()) ? it->second : empty;
}

vector<int> Graph::getNodesByPOI(const string& poi_type) const {
    string poi_lower = poi_type;
    transform(poi_lower.begin(), poi_lower.end(), poi_lower.begin(), ::tolower);
    
    auto it = poi_index.find(poi_lower);
    if (it != poi_index.end()) {
        return vector<int>(it->second.begin(), it->second.end());
    }
    return vector<int>();
}

double Graph::getEdgeWeight(int edge_id, const string& mode, int time_slot) const {
    const Edge* edge = getEdge(edge_id);
    if (!edge) return 1e9; // Invalid edge
    
    if (mode == "distance") {
        return edge->length;
    } else if (mode == "time") {
        // Use speed profile if available
        if (!edge->speed_profile.empty() && time_slot >= 0 && time_slot < 96) {
            double speed = edge->speed_profile[time_slot];
            if (speed > 0) {
                return edge->length / speed; // time = distance / speed
            }
        }
        // Fallback to average time
        return edge->average_time;
    }
    
    return edge->length; // Default to distance
}

double Graph::euclideanDistance(const Node& a, const Node& b) {
    return euclideanDistance(a.lat, a.lon, b.lat, b.lon);
}

double Graph::euclideanDistance(double lat1, double lon1, double lat2, double lon2) {
    // Simple Euclidean distance (works for small areas)
    // For more accuracy, use Haversine formula
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    
    // Approximate: 1 degree latitude ~ 111 km, 1 degree longitude ~ 111 km * cos(lat)
    double lat_km = dlat * 111000.0; // meters
    double lon_km = dlon * 111000.0 * cos((lat1 + lat2) / 2.0 * M_PI / 180.0);
    
    return sqrt(lat_km * lat_km + lon_km * lon_km);
}
