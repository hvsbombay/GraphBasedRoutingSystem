#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>
#include <unordered_map>
#include <string>
#include <set>
#include <cmath>

using namespace std;

struct Node {
    int id;
    double lat;
    double lon;
    vector<string> pois;
    
    Node() : id(-1), lat(0.0), lon(0.0) {}
    Node(int id, double lat, double lon, vector<string> pois = {})
        : id(id), lat(lat), lon(lon), pois(pois) {}
};

struct Edge {
    int id;
    int u;
    int v;
    double length;
    double average_time;
    vector<double> speed_profile; // 96 slots for 15-min intervals
    bool oneway;
    string road_type;
    bool deleted;
    
    Edge() : id(-1), u(-1), v(-1), length(0.0), average_time(0.0), 
             oneway(false), deleted(false) {}
};

class Graph {
private:
    unordered_map<int, Node> nodes;
    unordered_map<int, Edge> edges;
    unordered_map<int, Edge> deleted_edges; // Store deleted edges for restoration
    
    // Adjacency list: node_id -> vector of (neighbor_id, edge_id)
    unordered_map<int, vector<pair<int, int>>> adj;
    
    // POI index: poi_type -> set of node_ids
    unordered_map<string, set<int>> poi_index;
    
    void buildAdjacencyList();
    void rebuildAdjacencyList();
    
public:
    Graph();
    
    // Initialization
    void addNode(const Node& node);
    void addEdge(const Edge& edge);
    void finalize(); // Call after loading all nodes and edges
    
    // Dynamic updates
    bool removeEdge(int edge_id);
    bool modifyEdge(int edge_id, const Edge& patch, bool has_patch);
    
    // Queries
    const Node* getNode(int node_id) const;
    const Edge* getEdge(int edge_id) const;
    const vector<pair<int, int>>& getNeighbors(int node_id) const;
    
    // POI queries
    vector<int> getNodesByPOI(const string& poi_type) const;
    
    // Utility
    int getNodeCount() const { return nodes.size(); }
    int getEdgeCount() const { return edges.size(); }
    bool hasNode(int node_id) const { return nodes.find(node_id) != nodes.end(); }
    bool hasEdge(int edge_id) const { return edges.find(edge_id) != edges.end(); }
    
    // Get edge weight based on mode
    double getEdgeWeight(int edge_id, const string& mode, int time_slot = 0) const;
    
    // Euclidean distance
    static double euclideanDistance(const Node& a, const Node& b);
    static double euclideanDistance(double lat1, double lon1, double lat2, double lon2);
    
    // Get all nodes (for iteration)
    const unordered_map<int, Node>& getNodes() const { return nodes; }
    const unordered_map<int, Edge>& getEdges() const { return edges; }
};

#endif // GRAPH_HPP
