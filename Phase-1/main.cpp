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

// Process a single query with try-catch as per new specification
json process_query(const json& query) {
    json result;
    
    try {
        string query_type = query["type"];
        
        if (query_type == "remove_edge") {
            int edge_id = query["edge_id"];
            int id = query["id"];
            bool success = graph.removeEdge(edge_id);
            result["id"] = id;
            result["done"] = success;
        }
        else if (query_type == "modify_edge") {
            int edge_id = query["edge_id"];
            int id = query["id"];
            Edge patch;
            bool has_patch = false;
            
            if (query.contains("patch") && !query["patch"].empty()) {
                has_patch = true;
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
            
            bool success = graph.modifyEdge(edge_id, patch, has_patch);
            result["id"] = id;
            result["done"] = success;
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
    
    } catch (const exception& e) {
        // Gracefully handle errors as per new specification
        result["error"] = string("Exception: ") + e.what();
        if (query.contains("id")) {
            result["id"] = query["id"];
        }
    } catch (...) {
        // Catch any other exceptions
        result["error"] = "Unknown exception occurred";
        if (query.contains("id")) {
            result["id"] = query["id"];
        }
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
    
    // Extract meta and events as per new specification
    json meta;
    json events;
    
    if (queries_json.contains("meta")) {
        meta = queries_json["meta"];
    }
    
    if (queries_json.contains("events")) {
        events = queries_json["events"];
    } else if (queries_json.is_array()) {
        // Fallback for old format
        events = queries_json;
    } else {
        cerr << "Invalid query format - missing 'events' field" << endl;
        return 1;
    }
    
    vector<json> results;
    
    // Process each query one by one with try-catch
    for (const auto& query : events) {
        auto start_time = chrono::high_resolution_clock::now();
        
        json result = process_query(query);
        
        auto end_time = chrono::high_resolution_clock::now();
        result["processing_time"] = chrono::duration<double, milli>(end_time - start_time).count();
        results.push_back(result);
    }
    
    // Write output with new format: {meta: {...}, results: [...]}
    ofstream output_file(argv[3]);
    if (!output_file.is_open()) {
        cerr << "Failed to open output.json for writing" << endl;
        return 1;
    }
    
    json output;
    if (!meta.is_null()) {
        output["meta"] = meta;
    }
    output["results"] = results;
    
    output_file << output.dump(4) << endl;
    
    output_file.close();
    return 0;
}
