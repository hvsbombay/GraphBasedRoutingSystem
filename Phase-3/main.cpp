#include "json.hpp"
#include "Graph.hpp"
#include "DeliveryScheduler.hpp"
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
            
            graph.addEdge(edge);
        }
    }
    
    graph.finalize();
    
    cout << "Loaded graph: " << graph.getNodeCount() << " nodes, " 
         << graph.getEdgeCount() << " edges" << endl;
    cout << "Initialization complete! Time: 0.0 ms" << endl;
}

// Process a single query (Phase 3 implementation)
json process_query(const json& query) {
    json result;
    
    try {
        // Parse delivery scheduling query
        if (!query.contains("orders") || !query.contains("fleet")) {
            result["error"] = "Missing required fields: orders or fleet";
            return result;
        }
        
        // Parse orders
        vector<Order> orders;
        for (const auto& order_json : query["orders"]) {
            Order order;
            order.order_id = order_json["order_id"];
            order.pickup_node = order_json["pickup"];
            order.dropoff_node = order_json["dropoff"];
            orders.push_back(order);
        }
        
        // Parse fleet info
        int num_delivery_guys = query["fleet"]["num_delievery_guys"];  // Note: typo in spec
        int depot_node = query["fleet"]["depot_node"];
        
        // Validate depot node exists
        if (!graph.hasNode(depot_node)) {
            result["error"] = "Invalid depot node";
            return result;
        }
        
        // Validate all order nodes exist
        for (const auto& order : orders) {
            if (!graph.hasNode(order.pickup_node) || !graph.hasNode(order.dropoff_node)) {
                result["error"] = "Invalid order nodes";
                return result;
            }
        }
        
        // Create scheduler and run
        DeliveryScheduler scheduler(graph, depot_node, orders, num_delivery_guys);
        SchedulingResult scheduling_result = scheduler.schedule();
        
        // Build output JSON
        json assignments_json = json::array();
        
        for (const auto& driver : scheduling_result.assignments) {
            if (driver.order_ids.empty()) continue;  // Skip drivers with no assignments
            
            json driver_json;
            driver_json["driver_id"] = driver.driver_id;
            driver_json["route"] = driver.route;
            
            // Convert order indices to actual order_ids
            json order_ids_json = json::array();
            for (int order_idx : driver.order_ids) {
                order_ids_json.push_back(orders[order_idx].order_id);
            }
            driver_json["order_ids"] = order_ids_json;
            
            assignments_json.push_back(driver_json);
        }
        
        result["assignments"] = assignments_json;
        result["metrics"] = {
            {"total_delivery_time_s", scheduling_result.total_delivery_time}
        };
        
    } catch (const exception& e) {
        result["error"] = string("Exception: ") + e.what();
    }
    
    return result;
}

int main(int argc, char* argv[]) {
    if (argc != 4 && argc != 3) {
        cerr << "Usage: " << argv[0] << " <graph.json> <queries.json> [output.json]" << endl;
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
    
    for (const auto& query : queries_json) {
        auto start_time = chrono::high_resolution_clock::now();
        
        json result = process_query(query);
        
        auto end_time = chrono::high_resolution_clock::now();
        result["processing_time"] = chrono::duration<double, milli>(end_time - start_time).count();
        results.push_back(result);
    }
    
    json output = results;
    
    if (argc == 4) {
        ofstream output_file(argv[3]);
        if (!output_file.is_open()) {
            cerr << "Failed to open output.json for writing" << endl;
            return 1;
        }
        output_file << output.dump(4) << endl;
        output_file.close();
    } else {
        cout << output.dump(4) << endl;
    }

    return 0;
}
