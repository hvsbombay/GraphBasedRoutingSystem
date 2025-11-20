#include "DeliveryScheduler.hpp"
#include <queue>
#include <cmath>
#include <algorithm>
#include <limits>
#include <set>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

// Clarke-Wright Savings Algorithm for VRP
SchedulingResult DeliveryScheduler::savingsAlgorithm() {
    SchedulingResult result;
    
    // Start with each order on its own route from depot
    vector<Driver> drivers;
    for (size_t i = 0; i < orders.size() && i < (size_t)num_drivers; i++) {
        Driver driver(i);
        driver.order_ids.push_back(i);
        driver.route = buildValidRoute(driver.order_ids, depot_node);
        driver.total_time = calculateRouteTime(driver.route);
        drivers.push_back(driver);
    }
    
    // Calculate savings for merging routes
    struct Saving {
        int order_i, order_j;
        double savings;
        bool operator<(const Saving& other) const {
            return savings > other.savings;  // Descending order
        }
    };
    
    vector<Saving> savings_list;
    for (size_t i = 0; i < orders.size(); i++) {
        for (size_t j = i + 1; j < orders.size(); j++) {
            auto [d_depot_i, p1] = getShortestPath(depot_node, orders[i].pickup_node);
            auto [d_i_depot, p2] = getShortestPath(orders[i].dropoff_node, depot_node);
            auto [d_depot_j, p3] = getShortestPath(depot_node, orders[j].pickup_node);
            auto [d_j_depot, p4] = getShortestPath(orders[j].dropoff_node, depot_node);
            auto [d_i_j, p5] = getShortestPath(orders[i].dropoff_node, orders[j].pickup_node);
            
            double savings = (d_depot_i + d_i_depot) + (d_depot_j + d_j_depot) - (d_depot_i + d_i_j + d_j_depot);
            if (savings > 0) {
                savings_list.push_back({(int)i, (int)j, savings});
            }
        }
    }
    
    sort(savings_list.begin(), savings_list.end());
    
    // Merge routes based on savings (simplified version)
    // Track which driver has which order
    vector<int> order_to_driver(orders.size());
    for (size_t i = 0; i < drivers.size(); i++) {
        for (int order_id : drivers[i].order_ids) {
            order_to_driver[order_id] = i;
        }
    }
    
    for (const auto& saving : savings_list) {
        if (drivers.size() >= (size_t)num_drivers) break;
        
        int driver_i = order_to_driver[saving.order_i];
        int driver_j = order_to_driver[saving.order_j];
        
        if (driver_i != driver_j) {
            // Merge driver_j into driver_i
            drivers[driver_i].order_ids.insert(drivers[driver_i].order_ids.end(),
                                              drivers[driver_j].order_ids.begin(),
                                              drivers[driver_j].order_ids.end());
            drivers[driver_i].route = buildValidRoute(drivers[driver_i].order_ids, depot_node);
            drivers[driver_i].total_time = calculateRouteTime(drivers[driver_i].route);
            
            // Update order mappings
            for (int order_id : drivers[driver_j].order_ids) {
                order_to_driver[order_id] = driver_i;
            }
            
            // Remove driver_j
            drivers.erase(drivers.begin() + driver_j);
            
            // Update driver indices
            for (int& d : order_to_driver) {
                if (d > driver_j) d--;
            }
        }
    }
    
    result.assignments = drivers;
    result.total_delivery_time = calculateTotalDeliveryTime(drivers);
    return result;
}

// Simulated Annealing for local optimization
SchedulingResult DeliveryScheduler::simulatedAnnealing(SchedulingResult initial, int max_iterations) {
    SchedulingResult current = initial;
    SchedulingResult best = current;
    
    double temperature = 1000.0;
    double cooling_rate = 0.995;
    double min_temperature = 1.0;
    
    for (int iter = 0; iter < max_iterations && temperature > min_temperature; iter++) {
        SchedulingResult neighbor = perturbSolution(current);
        
        double current_cost = calculateCost(current);
        double neighbor_cost = calculateCost(neighbor);
        double delta = neighbor_cost - current_cost;
        
        // Accept if better, or with probability based on temperature
        if (delta < 0 || (rand() / (double)RAND_MAX) < exp(-delta / temperature)) {
            current = neighbor;
            
            if (neighbor_cost < calculateCost(best)) {
                best = neighbor;
            }
        }
        
        temperature *= cooling_rate;
    }
    
    return best;
}

// Perturb solution for simulated annealing
SchedulingResult DeliveryScheduler::perturbSolution(const SchedulingResult& current) {
    SchedulingResult neighbor = current;
    
    if (neighbor.assignments.size() < 2) return neighbor;
    
    // Random move: swap an order between two drivers
    int driver1 = rand() % neighbor.assignments.size();
    int driver2 = rand() % neighbor.assignments.size();
    
    while (driver1 == driver2 && neighbor.assignments.size() > 1) {
        driver2 = rand() % neighbor.assignments.size();
    }
    
    if (!neighbor.assignments[driver1].order_ids.empty()) {
        int order_idx = rand() % neighbor.assignments[driver1].order_ids.size();
        int order_id = neighbor.assignments[driver1].order_ids[order_idx];
        
        // Move order from driver1 to driver2
        neighbor.assignments[driver1].order_ids.erase(
            neighbor.assignments[driver1].order_ids.begin() + order_idx);
        neighbor.assignments[driver2].order_ids.push_back(order_id);
        
        // Rebuild routes
        neighbor.assignments[driver1].route = buildValidRoute(
            neighbor.assignments[driver1].order_ids, depot_node);
        neighbor.assignments[driver1].total_time = calculateRouteTime(
            neighbor.assignments[driver1].route);
        
        neighbor.assignments[driver2].route = buildValidRoute(
            neighbor.assignments[driver2].order_ids, depot_node);
        neighbor.assignments[driver2].total_time = calculateRouteTime(
            neighbor.assignments[driver2].route);
        
        neighbor.total_delivery_time = calculateTotalDeliveryTime(neighbor.assignments);
    }
    
    return neighbor;
}

// Calculate solution cost (total delivery time)
double DeliveryScheduler::calculateCost(const SchedulingResult& solution) {
    return solution.total_delivery_time;
}

// Cluster-First Route-Second approach
SchedulingResult DeliveryScheduler::clusterFirstRouteSecond() {
    // Cluster orders spatially, then solve TSP for each cluster
    vector<vector<int>> clusters = clusterOrders(num_drivers);
    
    SchedulingResult result;
    vector<Driver> drivers;
    
    for (int d = 0; d < num_drivers && d < (int)clusters.size(); d++) {
        Driver driver(d);
        driver.order_ids = clusters[d];
        driver.route = buildValidRoute(driver.order_ids, depot_node);
        driver.total_time = calculateRouteTime(driver.route);
        drivers.push_back(driver);
    }
    
    result.assignments = drivers;
    result.total_delivery_time = calculateTotalDeliveryTime(drivers);
    return result;
}

// Simple K-means clustering for orders
vector<vector<int>> DeliveryScheduler::clusterOrders(int num_clusters) {
    vector<vector<int>> clusters(num_clusters);
    
    if (orders.empty()) return clusters;
    
    // Use pickup locations for clustering
    vector<pair<double, double>> pickup_coords;
    for (const auto& order : orders) {
        const Node* node = graph.getNode(order.pickup_node);
        if (node) {
            pickup_coords.push_back({node->lat, node->lon});
        }
    }
    
    // Simple spatial assignment based on angle from depot
    const Node* depot = graph.getNode(depot_node);
    if (!depot) return clusters;
    
    for (size_t i = 0; i < orders.size(); i++) {
        const Node* pickup = graph.getNode(orders[i].pickup_node);
        if (!pickup) continue;
        
        double angle = atan2(pickup->lat - depot->lat, pickup->lon - depot->lon);
        int cluster_id = (int)((angle + M_PI) / (2 * M_PI) * num_clusters) % num_clusters;
        clusters[cluster_id].push_back(i);
    }
    
    return clusters;
}

// Or-opt move: remove and reinsert a segment
bool DeliveryScheduler::orOptMove(vector<int>& route, const vector<int>& order_ids) {
    if (route.size() < 5) return false;  // Need sufficient nodes
    
    double original_time = calculateRouteTime(route);
    vector<int> best_route = route;
    double best_time = original_time;
    
    // Try removing segments of size 1-3 and reinserting elsewhere
    for (int seg_size = 1; seg_size <= 3 && seg_size < (int)route.size() - 2; seg_size++) {
        for (size_t i = 1; i < route.size() - seg_size; i++) {
            // Extract segment
            vector<int> segment(route.begin() + i, route.begin() + i + seg_size);
            vector<int> temp_route = route;
            temp_route.erase(temp_route.begin() + i, temp_route.begin() + i + seg_size);
            
            // Try inserting at each position
            for (size_t j = 1; j < temp_route.size(); j++) {
                vector<int> new_route = temp_route;
                new_route.insert(new_route.begin() + j, segment.begin(), segment.end());
                
                if (validateRoute(new_route, order_ids)) {
                    double new_time = calculateRouteTime(new_route);
                    if (new_time < best_time) {
                        best_time = new_time;
                        best_route = new_route;
                    }
                }
            }
        }
    }
    
    if (best_time < original_time - 1e-6) {
        route = best_route;
        return true;
    }
    
    return false;
}

// Adaptive strategy: choose algorithm based on problem size
SchedulingResult DeliveryScheduler::adaptiveSchedule() {
    int total_orders = orders.size();
    
    SchedulingResult result;
    
    // Small problems: try multiple algorithms and pick best
    if (total_orders <= 10) {
        SchedulingResult greedy = greedyAssignment();
        SchedulingResult savings = savingsAlgorithm();
        SchedulingResult cluster = clusterFirstRouteSecond();
        
        // Apply simulated annealing to best initial solution
        if (greedy.total_delivery_time <= savings.total_delivery_time &&
            greedy.total_delivery_time <= cluster.total_delivery_time) {
            result = simulatedAnnealing(greedy, 500);
        } else if (savings.total_delivery_time <= cluster.total_delivery_time) {
            result = simulatedAnnealing(savings, 500);
        } else {
            result = simulatedAnnealing(cluster, 500);
        }
    }
    // Medium problems: use savings + local search
    else if (total_orders <= 30) {
        result = savingsAlgorithm();
        
        // Apply or-opt to each route
        for (auto& driver : result.assignments) {
            orOptMove(driver.route, driver.order_ids);
            driver.total_time = calculateRouteTime(driver.route);
        }
        result.total_delivery_time = calculateTotalDeliveryTime(result.assignments);
    }
    // Large problems: cluster-first for speed
    else {
        result = clusterFirstRouteSecond();
        
        // Light optimization with 2-opt
        for (auto& driver : result.assignments) {
            if (driver.route.size() > 3) {
                twoOptOptimization(driver.route);
                if (validateRoute(driver.route, driver.order_ids)) {
                    driver.total_time = calculateRouteTime(driver.route);
                }
            }
        }
        result.total_delivery_time = calculateTotalDeliveryTime(result.assignments);
    }
    
    return result;
}

// Main scheduling function
SchedulingResult DeliveryScheduler::schedule() {
    if (orders.empty()) {
        return SchedulingResult();
    }
    
    // Use adaptive strategy to select best algorithm
    SchedulingResult result = adaptiveSchedule();
    
    return result;
}

// Alternative: minimize maximum delivery time
SchedulingResult DeliveryScheduler::scheduleMinMax() {
    // For now, use same algorithm - can be extended with load balancing
    return schedule();
}
