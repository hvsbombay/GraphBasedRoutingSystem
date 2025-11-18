#include "DeliveryScheduler.hpp"
#include <queue>
#include <cmath>
#include <algorithm>
#include <limits>
#include <set>

using namespace std;

DeliveryScheduler::DeliveryScheduler(const Graph& g, int depot, const vector<Order>& orders, int num_drivers)
    : graph(g), depot_node(depot), orders(orders), num_drivers(num_drivers) {}

// Dijkstra's algorithm to find shortest path
pair<double, vector<int>> DeliveryScheduler::dijkstra(int source, int target) {
    const double INF = numeric_limits<double>::infinity();
    unordered_map<int, double> dist;
    unordered_map<int, int> parent;
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
    
    // Initialize
    for (const auto& [node_id, node] : graph.getNodes()) {
        dist[node_id] = INF;
    }
    dist[source] = 0.0;
    pq.push({0.0, source});
    parent[source] = -1;
    
    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        
        if (d > dist[u]) continue;
        if (u == target) break;  // Early termination
        
        const auto& neighbors = graph.getNeighbors(u);
        for (const auto& [v, edge_id] : neighbors) {
            double weight = graph.getEdgeWeight(edge_id, "time", 0);
            
            if (dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                parent[v] = u;
                pq.push({dist[v], v});
            }
        }
    }
    
    // Reconstruct path
    vector<int> path;
    if (dist[target] == INF) {
        return {INF, path};  // No path exists
    }
    
    int curr = target;
    while (curr != -1) {
        path.push_back(curr);
        curr = parent[curr];
    }
    reverse(path.begin(), path.end());
    
    return {dist[target], path};
}

// Get shortest path with caching
pair<double, vector<int>> DeliveryScheduler::getShortestPath(int from, int to) {
    if (from == to) {
        return {0.0, {from}};
    }
    
    // Check cache
    if (path_cache.count(from) && path_cache[from].count(to)) {
        return path_cache[from][to];
    }
    
    // Compute and cache
    auto result = dijkstra(from, to);
    path_cache[from][to] = result;
    
    return result;
}

// Calculate total time for a route
double DeliveryScheduler::calculateRouteTime(const vector<int>& route) {
    if (route.size() < 2) return 0.0;
    
    double total_time = 0.0;
    for (size_t i = 0; i < route.size() - 1; i++) {
        auto [time, path] = getShortestPath(route[i], route[i + 1]);
        if (time == numeric_limits<double>::infinity()) {
            return time;  // Unreachable
        }
        total_time += time;
    }
    
    return total_time;
}

// Validate that pickup precedes dropoff for all orders
bool DeliveryScheduler::validateRoute(const vector<int>& route, const vector<int>& order_ids) {
    for (int order_id : order_ids) {
        const Order& order = orders[order_id];
        
        // Find positions of pickup and dropoff in route
        int pickup_pos = -1, dropoff_pos = -1;
        for (size_t i = 0; i < route.size(); i++) {
            if (route[i] == order.pickup_node && pickup_pos == -1) {
                pickup_pos = i;
            }
            if (route[i] == order.dropoff_node && dropoff_pos == -1) {
                dropoff_pos = i;
            }
        }
        
        // Pickup must come before dropoff
        if (pickup_pos == -1 || dropoff_pos == -1 || pickup_pos >= dropoff_pos) {
            return false;
        }
    }
    
    return true;
}

// Nearest neighbor TSP heuristic
vector<int> DeliveryScheduler::nearestNeighborTSP(const vector<int>& nodes, int start) {
    if (nodes.empty()) return {start};
    
    vector<int> route;
    set<int> unvisited(nodes.begin(), nodes.end());
    
    int current = start;
    route.push_back(current);
    unvisited.erase(current);
    
    while (!unvisited.empty()) {
        int nearest = -1;
        double min_dist = numeric_limits<double>::infinity();
        
        for (int node : unvisited) {
            auto [dist, path] = getShortestPath(current, node);
            if (dist < min_dist) {
                min_dist = dist;
                nearest = node;
            }
        }
        
        if (nearest == -1) break;  // No reachable nodes
        
        route.push_back(nearest);
        unvisited.erase(nearest);
        current = nearest;
    }
    
    return route;
}

// 2-opt optimization to improve route (precedence-preserving version)
void DeliveryScheduler::twoOptOptimization(vector<int>& route) {
    if (route.size() < 4) return;  // Need at least 4 nodes for 2-opt
    
    bool improved = true;
    int max_iterations = 100;
    int iteration = 0;
    
    while (improved && iteration < max_iterations) {
        improved = false;
        iteration++;
        
        for (size_t i = 1; i < route.size() - 2; i++) {
            for (size_t j = i + 1; j < route.size() - 1; j++) {
                // Calculate current distance
                double current_dist = 0.0;
                auto [d1, p1] = getShortestPath(route[i-1], route[i]);
                auto [d2, p2] = getShortestPath(route[j], route[j+1]);
                current_dist = d1 + d2;
                
                // Calculate new distance after swap
                double new_dist = 0.0;
                auto [d3, p3] = getShortestPath(route[i-1], route[j]);
                auto [d4, p4] = getShortestPath(route[i], route[j+1]);
                new_dist = d3 + d4;
                
                if (new_dist < current_dist - 1e-6) {
                    // Tentatively reverse the segment between i and j
                    reverse(route.begin() + i, route.begin() + j + 1);
                    improved = true;
                    
                    // Note: We don't validate here because we don't have order_ids in this function
                    // Validation will happen at the caller level
                }
            }
        }
    }
}

// Build valid route ensuring pickup before dropoff
// Uses interleaved strategy: can deliver orders as soon as picked up
vector<int> DeliveryScheduler::buildValidRoute(const vector<int>& order_ids, int start_node) {
    vector<int> route;
    route.push_back(start_node);
    
    if (order_ids.empty()) {
        return route;
    }
    
    // Track which orders have been picked up but not delivered
    set<int> picked_up;
    set<int> delivered;
    
    int current = start_node;
    
    // Continue until all orders are delivered
    while (delivered.size() < order_ids.size()) {
        int next_node = -1;
        double min_dist = numeric_limits<double>::infinity();
        bool is_pickup = false;
        int selected_order = -1;
        
        // Option 1: Pick up a new order (if not all picked up)
        for (int order_id : order_ids) {
            if (picked_up.count(order_id) == 0) {
                int pickup = orders[order_id].pickup_node;
                auto [dist, path] = getShortestPath(current, pickup);
                
                if (dist < min_dist) {
                    min_dist = dist;
                    next_node = pickup;
                    is_pickup = true;
                    selected_order = order_id;
                }
            }
        }
        
        // Option 2: Deliver a picked up order
        // Apply slight bias toward deliveries to reduce carried load
        for (int order_id : order_ids) {
            if (picked_up.count(order_id) > 0 && delivered.count(order_id) == 0) {
                int dropoff = orders[order_id].dropoff_node;
                auto [dist, path] = getShortestPath(current, dropoff);
                
                // Bias: prefer deliveries when they're competitive (within 20% of best pickup)
                double biased_dist = dist * 0.8;
                
                if (biased_dist < min_dist) {
                    min_dist = dist;  // Use actual distance for route cost
                    next_node = dropoff;
                    is_pickup = false;
                    selected_order = order_id;
                }
            }
        }
        
        if (next_node == -1 || selected_order == -1) {
            break;  // No valid move (shouldn't happen)
        }
        
        route.push_back(next_node);
        current = next_node;
        
        if (is_pickup) {
            picked_up.insert(selected_order);
        } else {
            delivered.insert(selected_order);
        }
    }
    
    return route;
}

// Greedy assignment strategy
SchedulingResult DeliveryScheduler::greedyAssignment() {
    SchedulingResult result;
    
    // Initialize drivers
    vector<Driver> drivers;
    for (int i = 0; i < num_drivers; i++) {
        drivers.push_back(Driver(i));
        drivers[i].route.push_back(depot_node);
    }
    
    // Sort orders by distance from depot (closest first)
    vector<int> order_indices(orders.size());
    for (size_t i = 0; i < orders.size(); i++) {
        order_indices[i] = i;
    }
    
    sort(order_indices.begin(), order_indices.end(), [&](int a, int b) {
        auto [dist_a, path_a] = getShortestPath(depot_node, orders[a].pickup_node);
        auto [dist_b, path_b] = getShortestPath(depot_node, orders[b].pickup_node);
        return dist_a < dist_b;
    });
    
    // Assign orders to drivers greedily
    for (int order_idx : order_indices) {
        // Find driver with minimum incremental time
        int best_driver = 0;
        double min_incremental_time = numeric_limits<double>::infinity();
        
        for (int d = 0; d < num_drivers; d++) {
            vector<int> temp_order_ids = drivers[d].order_ids;
            temp_order_ids.push_back(order_idx);
            
            vector<int> temp_route = buildValidRoute(temp_order_ids, depot_node);
            double new_time = calculateRouteTime(temp_route);
            double incremental = new_time - drivers[d].total_time;
            
            if (incremental < min_incremental_time) {
                min_incremental_time = incremental;
                best_driver = d;
            }
        }
        
        // Assign to best driver
        drivers[best_driver].order_ids.push_back(order_idx);
        drivers[best_driver].route = buildValidRoute(drivers[best_driver].order_ids, depot_node);
        drivers[best_driver].total_time = calculateRouteTime(drivers[best_driver].route);
    }
    
    // Optimize each driver's route with 2-opt (with feasibility checks)
    for (auto& driver : drivers) {
        if (driver.route.size() > 3) {
            // Save original route in case optimization breaks constraints
            vector<int> original_route = driver.route;
            double original_time = driver.total_time;
            
            // Try to optimize
            twoOptOptimization(driver.route);
            double new_time = calculateRouteTime(driver.route);
            
            // Validate that optimization didn't break pickup-before-dropoff
            if (validateRoute(driver.route, driver.order_ids)) {
                // Keep the improvement
                driver.total_time = new_time;
            } else {
                // Revert to original route
                driver.route = original_route;
                driver.total_time = original_time;
            }
        }
    }    
    result.assignments = drivers;
    result.total_delivery_time = calculateTotalDeliveryTime(drivers);
    
    return result;
}

// Calculate total delivery time (sum of completion times)
double DeliveryScheduler::calculateTotalDeliveryTime(const vector<Driver>& drivers) {
    double total = 0.0;
    
    for (const auto& driver : drivers) {
        if (driver.route.size() < 2) continue;
        
        double current_time = 0.0;
        
        // Track when each order is delivered
        for (size_t i = 1; i < driver.route.size(); i++) {
            auto [time, path] = getShortestPath(driver.route[i-1], driver.route[i]);
            current_time += time;
            
            // Check if this is a dropoff for any order
            for (int order_id : driver.order_ids) {
                if (orders[order_id].dropoff_node == driver.route[i]) {
                    total += current_time;
                }
            }
        }
    }
    
    return total;
}

// Main scheduling function
SchedulingResult DeliveryScheduler::schedule() {
    if (orders.empty()) {
        return SchedulingResult();
    }
    
    // Use greedy assignment strategy
    SchedulingResult result = greedyAssignment();
    
    return result;
}

// Alternative: minimize maximum delivery time
SchedulingResult DeliveryScheduler::scheduleMinMax() {
    // For now, use same algorithm - can be extended with load balancing
    return schedule();
}
