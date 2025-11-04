#include "json.hpp"
#include "Graph.hpp"
#include "ShortestPath.hpp"
#include "KNN.hpp"
#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>

using json = nlohmann::json;
using namespace std;

// Global graph instance
Graph graph;

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
            
            if (edge_json.contains("speed_profile")) {
                for (const auto& speed : edge_json["speed_profile"]) {
                    edge.speed_profile.push_back(speed);
                }
            }
            
            graph.addEdge(edge);
        }
    }
    
    graph.finalize();
    
    cout << "Loaded graph: " << graph.getNodeCount() << " nodes, " 
         << graph.getEdgeCount() << " edges" << endl;
}

// Process a single query
json process_query(const json& query) {
    json result;
    string query_type = query["type"];
    
    if (query_type == "remove_edge") {
        int edge_id = query["edge_id"];
        graph.removeEdge(edge_id);
        result["done"] = true;
    }
    else if (query_type == "modify_edge") {
        int edge_id = query["edge_id"];
        Edge patch;
        
        if (query.contains("patch")) {
            const auto& patch_json = query["patch"];
            
            if (patch_json.contains("length")) {
                patch.length = patch_json["length"];
            }
            if (patch_json.contains("average_time")) {
                patch.average_time = patch_json["average_time"];
            }
            if (patch_json.contains("speed_profile")) {
                for (const auto& speed : patch_json["speed_profile"]) {
                    patch.speed_profile.push_back(speed);
                }
            }
            if (patch_json.contains("road_type")) {
                patch.road_type = patch_json["road_type"];
            }
        }
        
        graph.modifyEdge(edge_id, patch);
        result["done"] = true;
    }
    else if (query_type == "shortest_path") {
        int id = query["id"];
        int source = query["source"];
        int target = query["target"];
        string mode = query["mode"];
        
        set<int> forbidden_nodes;
        set<string> forbidden_road_types;
        
        if (query.contains("constraints")) {
            const auto& constraints = query["constraints"];
            
            if (constraints.contains("forbidden_nodes")) {
                for (const auto& node : constraints["forbidden_nodes"]) {
                    forbidden_nodes.insert(node.get<int>());
                }
            }
            
            if (constraints.contains("forbidden_road_types")) {
                for (const auto& road_type : constraints["forbidden_road_types"]) {
                    forbidden_road_types.insert(road_type.get<string>());
                }
            }
        }
        
        ShortestPath sp(graph);
        PathResult path_result;
        
        if (mode == "time") {
            path_result = sp.findTimeDependentPath(source, target, forbidden_nodes, forbidden_road_types);
        } else {
            path_result = sp.findPath(source, target, mode, forbidden_nodes, forbidden_road_types);
        }
        
        result["id"] = id;
        result["possible"] = path_result.possible;
        
        if (path_result.possible) {
            if (mode == "time") {
                result["minimum_time"] = path_result.distance;
            } else {
                result["minimum_distance"] = path_result.distance;
            }
            result["path"] = path_result.path;
        }
    }
    else if (query_type == "knn") {
        int id = query["id"];
        string poi = query["poi"];
        double query_lat = query["query_point"]["lat"];
        double query_lon = query["query_point"]["lon"];
        int k = query["k"];
        string metric = query["metric"];
        
        KNN knn(graph);
        vector<int> nodes;
        
        if (metric == "euclidean") {
            nodes = knn.findKNearestEuclidean(query_lat, query_lon, poi, k);
        } else if (metric == "shortest_path") {
            nodes = knn.findKNearestShortestPath(query_lat, query_lon, poi, k);
        }
        
        result["id"] = id;
        result["nodes"] = nodes;
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
    return 0;
}
