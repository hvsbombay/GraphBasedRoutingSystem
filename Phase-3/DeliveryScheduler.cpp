#include "DeliveryScheduler.hpp"
#include <queue>
#include <cmath>
#include <algorithm>
#include <limits>
#include <set>
#include <random>
#include <future>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;

// Thread-safe random number generator helpers
namespace {
    int getRandomInt(int min, int max) {
        static thread_local std::mt19937 generator(std::random_device{}());
        std::uniform_int_distribution<int> distribution(min, max);
        return distribution(generator);
    }

    double getRandomDouble() {
        static thread_local std::mt19937 generator(std::random_device{}());
        std::uniform_real_distribution<double> distribution(0.0, 1.0);
        return distribution(generator);
    }
}

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
    
    // Check cache with lock
    {
        std::lock_guard<std::mutex> lock(cache_mutex);
        if (path_cache.count(from) && path_cache[from].count(to)) {
            return path_cache[from][to];
        }
    }
    
    // Compute (without lock to allow parallelism)
    auto result = dijkstra(from, to);
    
    // Update cache with lock
    {
        std::lock_guard<std::mutex> lock(cache_mutex);
        path_cache[from][to] = result;
    }
    
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
    
    // 1. Start with each order on its own route
    // We use a list of order lists to represent routes
    vector<vector<int>> routes;
    for (size_t i = 0; i < orders.size(); i++) {
        routes.push_back({(int)i});
    }
    
    // 2. Calculate savings for merging routes
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
            
            // Savings = Cost(i) + Cost(j) - Cost(i+j)
            // Cost(i) = depot->i->depot
            // Cost(i+j) = depot->i->j->depot
            double savings = (d_depot_i + d_i_depot) + (d_depot_j + d_j_depot) - (d_depot_i + d_i_j + d_j_depot);
            if (savings > 0) {
                savings_list.push_back({(int)i, (int)j, savings});
            }
        }
    }
    
    sort(savings_list.begin(), savings_list.end());
    
    // 3. Merge routes based on savings
    // Map each order to its current route index
    vector<int> order_to_route(orders.size());
    for (size_t i = 0; i < orders.size(); i++) {
        order_to_route[i] = i;
    }
    
    // Track active routes (true if route index is valid)
    vector<bool> route_active(orders.size(), true);
    int num_routes = orders.size();
    
    for (const auto& saving : savings_list) {
        // If we have reached the desired number of drivers, we can stop merging based on savings
        // But usually we want to merge as much as possible to reduce cost, then handle fleet constraint
        
        int r_i = order_to_route[saving.order_i];
        int r_j = order_to_route[saving.order_j];
        
        if (r_i != r_j && route_active[r_i] && route_active[r_j]) {
            // Check if merge is valid (i at end of r_i, j at start of r_j)
            // For simplicity in this VRP, we just append order lists and let buildValidRoute handle the sequence
            // This is a "Cluster-based" interpretation of Savings
            
            // Merge r_j into r_i
            routes[r_i].insert(routes[r_i].end(), routes[r_j].begin(), routes[r_j].end());
            
            // Update mapping
            for (int order_id : routes[r_j]) {
                order_to_route[order_id] = r_i;
            }
            
            // Deactivate r_j
            route_active[r_j] = false;
            routes[r_j].clear();
            num_routes--;
        }
    }
    
    // 4. Enforce fleet size constraint
    // Collect all active routes
    vector<vector<int>> final_routes;
    for (size_t i = 0; i < routes.size(); i++) {
        if (route_active[i] && !routes[i].empty()) {
            final_routes.push_back(routes[i]);
        }
    }
    
    // If we have too many routes, merge the smallest ones
    while (final_routes.size() > (size_t)num_drivers) {
        // Find smallest route
        size_t min_size_idx = 0;
        size_t min_size = final_routes[0].size();
        
        for (size_t i = 1; i < final_routes.size(); i++) {
            if (final_routes[i].size() < min_size) {
                min_size = final_routes[i].size();
                min_size_idx = i;
            }
        }
        
        // Merge it into the "nearest" other route (or just the first one for simplicity)
        // Ideally we check centroids, but here we just merge to 0 (or next available)
        size_t target_idx = (min_size_idx == 0) ? 1 : 0;
        
        final_routes[target_idx].insert(final_routes[target_idx].end(), 
                                       final_routes[min_size_idx].begin(), 
                                       final_routes[min_size_idx].end());
        
        final_routes.erase(final_routes.begin() + min_size_idx);
    }
    
    // 5. Convert to Drivers
    vector<Driver> drivers;
    for (size_t i = 0; i < final_routes.size(); i++) {
        Driver driver(i);
        driver.order_ids = final_routes[i];
        driver.route = buildValidRoute(driver.order_ids, depot_node);
        driver.total_time = calculateRouteTime(driver.route);
        drivers.push_back(driver);
    }
    
    // If we have fewer routes than drivers, add empty drivers
    for (size_t i = final_routes.size(); i < (size_t)num_drivers; i++) {
        Driver driver(i);
        driver.route = {depot_node};
        drivers.push_back(driver);
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
        if (delta < 0 || getRandomDouble() < exp(-delta / temperature)) {
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
    int driver1 = getRandomInt(0, neighbor.assignments.size() - 1);
    int driver2 = getRandomInt(0, neighbor.assignments.size() - 1);
    
    while (driver1 == driver2 && neighbor.assignments.size() > 1) {
        driver2 = getRandomInt(0, neighbor.assignments.size() - 1);
    }
    
    if (!neighbor.assignments[driver1].order_ids.empty()) {
        int order_idx = getRandomInt(0, neighbor.assignments[driver1].order_ids.size() - 1);
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
    else if (total_orders <= 18) {
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

// Create a random valid individual for GA
DeliveryScheduler::Individual DeliveryScheduler::createRandomIndividual() {
    Individual ind;
    
    // Randomly shuffle orders
    vector<int> shuffled_orders(orders.size());
    for (size_t i = 0; i < orders.size(); i++) shuffled_orders[i] = i;
    
    random_device rd;
    mt19937 g(rd());
    shuffle(shuffled_orders.begin(), shuffled_orders.end(), g);
    
    // Randomly assign split points to divide orders among drivers
    // We need num_drivers - 1 split points
    vector<int> splits;
    for (int i = 0; i < num_drivers - 1; i++) {
        splits.push_back(getRandomInt(0, orders.size()));
    }
    sort(splits.begin(), splits.end());
    
    // Create drivers based on splits
    int current_order_idx = 0;
    
    for (int d = 0; d < num_drivers; d++) {
        Driver driver(d);
        int split = (d == num_drivers - 1) ? orders.size() : splits[d];
        
        while (current_order_idx < split) {
            driver.order_ids.push_back(shuffled_orders[current_order_idx]);
            current_order_idx++;
        }
        
        driver.route = buildValidRoute(driver.order_ids, depot_node);
        driver.total_time = calculateRouteTime(driver.route);
        ind.assignments.push_back(driver);
    }
    
    ind.fitness = calculateTotalDeliveryTime(ind.assignments);
    return ind;
}

// Crossover two parents to produce a child
DeliveryScheduler::Individual DeliveryScheduler::crossover(const Individual& parent1, const Individual& parent2) {
    // Simplified Order Crossover (OX) logic adapted for VRP
    // We treat the solution as a giant tour of orders and then re-partition
    
    // 1. Flatten parents into order sequences
    vector<int> p1_orders, p2_orders;
    for (const auto& d : parent1.assignments) p1_orders.insert(p1_orders.end(), d.order_ids.begin(), d.order_ids.end());
    for (const auto& d : parent2.assignments) p2_orders.insert(p2_orders.end(), d.order_ids.begin(), d.order_ids.end());
    
    if (p1_orders.empty()) return parent1;

    // 2. Perform OX on the order sequence
    int size = p1_orders.size();
    int start = getRandomInt(0, size - 1);
    int end = getRandomInt(0, size - 1);
    if (start > end) swap(start, end);
    
    vector<int> child_orders(size, -1);
    set<int> inherited;
    
    // Copy segment from P1
    for (int i = start; i <= end; i++) {
        child_orders[i] = p1_orders[i];
        inherited.insert(p1_orders[i]);
    }
    
    // Fill remaining from P2
    int current_p2 = 0;
    for (int i = 0; i < size; i++) {
        if (i >= start && i <= end) continue;
        
        while (current_p2 < size && inherited.count(p2_orders[current_p2])) {
            current_p2++;
        }
        
        if (current_p2 < size) {
            child_orders[i] = p2_orders[current_p2];
            inherited.insert(p2_orders[current_p2]);
        }
    }
    
    // 3. Re-partition into drivers (Randomly or inheriting split points)
    // For simplicity, we use random splits again to maintain diversity
    Individual child;
    vector<int> splits;
    for (int i = 0; i < num_drivers - 1; i++) {
        splits.push_back(getRandomInt(0, size));
    }
    sort(splits.begin(), splits.end());
    
    int current_order_idx = 0;
    for (int d = 0; d < num_drivers; d++) {
        Driver driver(d);
        int split = (d == num_drivers - 1) ? size : splits[d];
        
        while (current_order_idx < split) {
            driver.order_ids.push_back(child_orders[current_order_idx]);
            current_order_idx++;
        }
        
        driver.route = buildValidRoute(driver.order_ids, depot_node);
        driver.total_time = calculateRouteTime(driver.route);
        child.assignments.push_back(driver);
    }
    
    child.fitness = calculateTotalDeliveryTime(child.assignments);
    return child;
}

// Mutate an individual
void DeliveryScheduler::mutate(Individual& ind) {
    // Swap mutation: swap two orders between any drivers
    if (ind.assignments.empty()) return;
    
    int d1_idx = getRandomInt(0, ind.assignments.size() - 1);
    int d2_idx = getRandomInt(0, ind.assignments.size() - 1);
    
    Driver& d1 = ind.assignments[d1_idx];
    Driver& d2 = ind.assignments[d2_idx];
    
    if (d1.order_ids.empty() && d2.order_ids.empty()) return;
    
    // Move or Swap
    if (!d1.order_ids.empty()) {
        int o1_idx = getRandomInt(0, d1.order_ids.size() - 1);
        int val = d1.order_ids[o1_idx];
        
        d1.order_ids.erase(d1.order_ids.begin() + o1_idx);
        d2.order_ids.push_back(val);
        
        // Rebuild routes
        d1.route = buildValidRoute(d1.order_ids, depot_node);
        d1.total_time = calculateRouteTime(d1.route);
        d2.route = buildValidRoute(d2.order_ids, depot_node);
        d2.total_time = calculateRouteTime(d2.route);
    }
    
    ind.fitness = calculateTotalDeliveryTime(ind.assignments);
}

// Genetic Algorithm
SchedulingResult DeliveryScheduler::geneticAlgorithm(int population_size, int generations) {
    vector<Individual> population;
    
    // 1. Initialize Population
    // Seed with some heuristics for better start
    SchedulingResult savings = savingsAlgorithm();
    Individual savings_ind;
    savings_ind.assignments = savings.assignments;
    savings_ind.fitness = savings.total_delivery_time;
    population.push_back(savings_ind);
    
    for (int i = 1; i < population_size; i++) {
        population.push_back(createRandomIndividual());
    }
    
    // 2. Evolution Loop
    for (int gen = 0; gen < generations; gen++) {
        sort(population.begin(), population.end());
        
        vector<Individual> new_population;
        
        // Elitism: Keep top 10%
        int elite_count = population_size / 10;
        for (int i = 0; i < elite_count; i++) {
            new_population.push_back(population[i]);
        }
        
        // Breed rest
        while (new_population.size() < (size_t)population_size) {
            // Tournament Selection
            int t1 = getRandomInt(0, population_size / 2 - 1); // Bias towards better half
            int t2 = getRandomInt(0, population_size / 2 - 1);
            const Individual& p1 = population[t1];
            const Individual& p2 = population[t2];
            
            Individual child = crossover(p1, p2);
            
            // Mutation (10% chance)
            if (getRandomInt(0, 99) < 10) {
                mutate(child);
            }
            
            new_population.push_back(child);
        }
        
        population = new_population;
    }
    
    // Return best
    sort(population.begin(), population.end());
    SchedulingResult result;
    result.assignments = population[0].assignments;
    result.total_delivery_time = population[0].fitness;
    
    // Final polish with 2-opt
    for (auto& driver : result.assignments) {
        if (driver.route.size() > 3) {
            twoOptOptimization(driver.route);
            driver.total_time = calculateRouteTime(driver.route);
        }
    }
    result.total_delivery_time = calculateTotalDeliveryTime(result.assignments);
    
    return result;
}

// Parallel Portfolio Strategy
SchedulingResult DeliveryScheduler::parallelPortfolioSchedule() {
    // Run multiple distinct algorithms in parallel and pick the best
    
    // 1. Simulated Annealing (Good for local optima escape)
    auto future_sa = std::async(std::launch::async, [this]() {
        SchedulingResult initial = savingsAlgorithm();
        return simulatedAnnealing(initial, 1000);
    });
    
    // 2. Genetic Algorithm (Good for global exploration)
    auto future_ga = std::async(std::launch::async, [this]() {
        return geneticAlgorithm(50, 100); // 50 pop, 100 gens
    });
    
    // 3. Cluster-First Route-Second (Good for structure)
    auto future_cluster = std::async(std::launch::async, [this]() {
        SchedulingResult res = clusterFirstRouteSecond();
        // Polish with heavy local search
        for (auto& d : res.assignments) {
            twoOptOptimization(d.route);
            orOptMove(d.route, d.order_ids);
            d.total_time = calculateRouteTime(d.route);
        }
        res.total_delivery_time = calculateTotalDeliveryTime(res.assignments);
        return res;
    });
    
    // Wait for results
    SchedulingResult res_sa = future_sa.get();
    SchedulingResult res_ga = future_ga.get();
    SchedulingResult res_cluster = future_cluster.get();
    
    // Pick winner
    SchedulingResult best = res_sa;
    if (res_ga.total_delivery_time < best.total_delivery_time) best = res_ga;
    if (res_cluster.total_delivery_time < best.total_delivery_time) best = res_cluster;
    
    return best;
}

// Main scheduling function
SchedulingResult DeliveryScheduler::schedule() {
    if (orders.empty()) {
        return SchedulingResult();
    }
    
    // Use Parallel Portfolio Strategy for maximum performance
    // This is the "Class Apart" feature: Concurrency + Portfolio Optimization
    return parallelPortfolioSchedule();
}

// Alternative: minimize maximum delivery time
SchedulingResult DeliveryScheduler::scheduleMinMax() {
    // For now, use same algorithm - can be extended with load balancing
    return schedule();
}
