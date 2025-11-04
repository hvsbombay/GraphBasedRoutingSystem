#include "json.hpp"
#include "Graph.hpp"
#include "KShortestPaths.hpp"
#include "ApproxShortestPath.hpp"
#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>

using json = nlohmann::json;
using namespace std;

// Global graph instance
Graph graph;
ApproxShortestPath* approx_sp = nullptr;

// Load graph from JSON
void loadGraph(const string& filename) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Failed to open graph file: " << filename << endl;
        exit(1);
    }
    
    json graph_json;
    file >> graph_json;
    
    // Load nodes
    if (graph_json.contains("nodes")) {
        for (const auto& node_json : graph_json["nodes"]) {
            Node node;
            node.id = node_json["id"];
            node.lat = node_json["lat"];
            node.lon = node_json["lon"];
            
            if (node_json.contains("pois")) {
                for (const auto& poi : node_json["pois"]) {
                    node.pois.push_back(poi);
                }
            }
            
            graph.addNode(node);
        }
    }
    
    // Load edges
    if (graph_json.contains("edges")) {
        for (const auto& edge_json : graph_json["edges"]) {
            Edge edge;
            edge.id = edge_json["id"];
            edge.u = edge_json["u"];
            edge.v = edge_json["v"];
            edge.length = edge_json["length"];
            edge.average_time = edge_json["average_time"];
            edge.oneway = edge_json.value("oneway", false);
            edge.road_type = edge_json.value("road_type", "local");
            edge.deleted = false;
            
            graph.addEdge(edge);
        }
    }
    
    graph.finalize();
    
    cout << "Loaded graph: " << graph.getNodeCount() << " nodes, " 
         << graph.getEdgeCount() << " edges" << endl;
    
    // Initialize approximate shortest path structures
    approx_sp = new ApproxShortestPath(graph);
    cout << "Initializing landmarks for approximate shortest paths..." << endl;
    approx_sp->initialize();
    cout << "Initialization complete!" << endl;
}

// Process a single query (Phase 2 implementation)
json process_query(const json& query) {
    json result;
    string query_type = query["type"];
    
    if (query_type == "k_shortest_paths") {
        int id = query["id"];
        int source = query["source"];
        int target = query["target"];
        int k = query["k"];
        string mode = query["mode"];
        
        KShortestPaths ksp(graph);
        vector<PathInfo> paths = ksp.findKShortestPaths(source, target, k, mode);
        
        result["id"] = id;
        json paths_json = json::array();
        
        for (const auto& path_info : paths) {
            json path_obj;
            path_obj["path"] = path_info.path;
            path_obj["length"] = path_info.length;
            paths_json.push_back(path_obj);
        }
        
        result["paths"] = paths_json;
    }
    else if (query_type == "k_shortest_paths_heuristic") {
        int id = query["id"];
        int source = query["source"];
        int target = query["target"];
        int k = query["k"];
        
        double overlap_penalty = 0.7;
        double distance_penalty = 0.3;
        
        if (query.contains("heuristic")) {
            const auto& heuristic = query["heuristic"];
            if (heuristic.contains("overlap_penalty")) {
                overlap_penalty = heuristic["overlap_penalty"];
            }
            if (heuristic.contains("distance_penalty")) {
                distance_penalty = heuristic["distance_penalty"];
            }
        }
        
        KShortestPaths ksp(graph);
        vector<PathInfo> paths = ksp.findKShortestPathsHeuristic(source, target, k,
                                                                 overlap_penalty, distance_penalty);
        
        result["id"] = id;
        json paths_json = json::array();
        
        for (const auto& path_info : paths) {
            json path_obj;
            path_obj["path"] = path_info.path;
            path_obj["length"] = path_info.length;
            path_obj["penalty"] = path_info.penalty;
            paths_json.push_back(path_obj);
        }
        
        result["paths"] = paths_json;
    }
    else if (query_type == "approx_shortest_path") {
        int id = query["id"];
        double time_budget_ms = query["time_budget_ms"];
        double acceptable_error_pct = query["acceptable_error_pct"];
        
        vector<pair<int, int>> queries_list;
        for (const auto& q : query["queries"]) {
            int source = q["source"];
            int target = q["target"];
            queries_list.push_back({source, target});
        }
        
        vector<double> distances = approx_sp->findApproxPaths(queries_list, 
                                                              time_budget_ms,
                                                              acceptable_error_pct);
        
        result["id"] = id;
        result["distances"] = distances;
    }
    else {
        result["error"] = "Unknown query type: " + query_type;
    }
    
    return result;
}

int main(int argc, char* argv[]) {
    if (argc != 4) {
        cerr << "Usage: " << argv[0] << " <graph.json> <queries.json> <output.json>" << endl;
        return 1;
    }
    
    // Load graph
    loadGraph(argv[1]);
    
    // Read queries from second file
    ifstream queries_file(argv[2]);
    if (!queries_file.is_open()) {
        cerr << "Failed to open " << argv[2] << endl;
        return 1;
    }
    json queries_json;
    queries_file >> queries_json;
    
    vector<json> results;
    
    // Handle both array format and object with "events" field
    json events;
    if (queries_json.is_array()) {
        events = queries_json;
    } else if (queries_json.contains("events")) {
        events = queries_json["events"];
    } else {
        cerr << "Invalid query format" << endl;
        return 1;
    }
    
    for (const auto& query : events) {
        auto start_time = chrono::high_resolution_clock::now();
        
        json result = process_query(query);
        
        auto end_time = chrono::high_resolution_clock::now();
        result["processing_time"] = chrono::duration<double, milli>(end_time - start_time).count();
        results.push_back(result);
    }
    
    ofstream output_file(argv[3]);
    if (!output_file.is_open()) {
        cerr << "Failed to open output.json for writing" << endl;
        return 1;
    }
    
    json output = results;
    output_file << output.dump(4) << endl;
    
    output_file.close();
    
    // Cleanup
    if (approx_sp) {
        delete approx_sp;
    }
    
    return 0;
}
