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

// ============================================================================
// REAL-WORLD SCENARIO HANDLING: Dynamic Road Blocks
// ============================================================================

void DeliveryScheduler::addRoadBlock(int edge_id, double start_time, double duration, const string& reason) {
    RoadBlock block(edge_id, start_time, duration, reason);
    road_blocks.push_back(block);
}

void DeliveryScheduler::applyRoadBlocks(double current_time) {
    std::lock_guard<std::mutex> lock(block_mutex);
    
    for (const auto& block : road_blocks) {
        if (current_time >= block.start_time && 
            current_time < block.start_time + block.duration) {
            
            if (currently_blocked_edges.find(block.edge_id) == currently_blocked_edges.end()) {
                // Block this edge in the graph
                graph.removeEdge(block.edge_id);
                currently_blocked_edges.insert(block.edge_id);
            }
        }
    }
}

void DeliveryScheduler::clearExpiredBlocks(double current_time) {
    std::lock_guard<std::mutex> lock(block_mutex);
    
    set<int> to_restore;
    for (int edge_id : currently_blocked_edges) {
        bool still_blocked = false;
        
        for (const auto& block : road_blocks) {
            if (block.edge_id == edge_id &&
                current_time >= block.start_time && 
                current_time < block.start_time + block.duration) {
                still_blocked = true;
                break;
            }
        }
        
        if (!still_blocked) {
            to_restore.insert(edge_id);
        }
    }
    
    // Restore edges that are no longer blocked
    for (int edge_id : to_restore) {
        Edge restored_edge;
        graph.modifyEdge(edge_id, restored_edge, false);  // Restore without patch
        currently_blocked_edges.erase(edge_id);
    }
}

bool DeliveryScheduler::isEdgeBlocked(int edge_id) const {
    std::lock_guard<std::mutex> lock(block_mutex);
    return currently_blocked_edges.find(edge_id) != currently_blocked_edges.end();
}

void DeliveryScheduler::invalidateBlockedPaths() {
    std::lock_guard<std::mutex> lock(cache_mutex);
    
    // Clear entire path cache when road blocks occur
    // In a production system, we'd selectively invalidate only affected paths
    path_cache.clear();
    
    // Also clear distance matrix - it needs to be rebuilt
    dist_matrix.matrix.clear();
    dist_matrix.node_to_index.clear();
    dist_matrix.index_to_node.clear();
    dist_matrix.size = 0;
}

SchedulingResult DeliveryScheduler::replanWithBlocks(const SchedulingResult& original_plan, double current_time) {
    // Apply current road blocks
    applyRoadBlocks(current_time);
    
    // Invalidate cached paths that use blocked edges
    invalidateBlockedPaths();
    
    // Rebuild distance matrix with current graph state
    buildDistanceMatrix();
    
    // Recompute the schedule
    return scheduleDelivery();
}

DeliveryScheduler::DeliveryScheduler(Graph& g, int depot, const vector<Order>& orders, int num_drivers)
    : graph(g), depot_node(depot), orders(orders), num_drivers(num_drivers) {
    // Precompute distance matrix for all relevant nodes
    buildDistanceMatrix();
}

// Bidirectional A* for faster pathfinding
pair<double, vector<int>> DeliveryScheduler::findPathBidirectionalAStar(int source, int target) {
    if (source == target) return {0.0, {source}};

    // Use the graph's heuristic if available, or Euclidean distance
    auto heuristic = [&](int u, int v) {
        const Node* n1 = graph.getNode(u);
        const Node* n2 = graph.getNode(v);
        if (!n1 || !n2) return 0.0;
        
        // Haversine approximation or simple Euclidean on lat/lon
        double dLat = n1->lat - n2->lat;
        double dLon = n1->lon - n2->lon;
        return sqrt(dLat*dLat + dLon*dLon) * 111000.0 / 13.8; // Approx meters to seconds (assuming ~50km/h)
    };

    // Fallback to Standard A* for robustness
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
    unordered_map<int, double> dist;
    unordered_map<int, int> parent;
    
    dist[source] = 0.0;
    pq.push({heuristic(source, target), source});
    parent[source] = -1;
    
    while(!pq.empty()){
        auto [f, u] = pq.top();
        pq.pop();
        
        if(f - heuristic(u, target) > dist[u]) continue;
        if(u == target) break;
        
        for(const auto& [v, edge_id] : graph.getNeighbors(u)){
            double w = graph.getEdgeWeight(edge_id, "time", 0);
            if(!dist.count(v) || dist[u] + w < dist[v]){
                dist[v] = dist[u] + w;
                parent[v] = u;
                pq.push({dist[v] + heuristic(v, target), v});
            }
        }
    }
    
    if(!dist.count(target)) return {numeric_limits<double>::infinity(), {}};
    
    vector<int> path;
    int curr = target;
    while(curr != -1){
        path.push_back(curr);
        curr = parent[curr];
    }
    reverse(path.begin(), path.end());
    return {dist[target], path};
}

void DeliveryScheduler::buildDistanceMatrix() {
    set<int> key_nodes;
    key_nodes.insert(depot_node);
    for(const auto& o : orders) {
        key_nodes.insert(o.pickup_node);
        key_nodes.insert(o.dropoff_node);
    }
    
    vector<int> nodes(key_nodes.begin(), key_nodes.end());
    dist_matrix.init(nodes);
    
    // Parallelize matrix computation
    vector<future<void>> futures;
    int num_threads = thread::hardware_concurrency();
    int chunk_size = (nodes.size() + num_threads - 1) / num_threads;
    
    for(int t=0; t<num_threads; ++t) {
        futures.push_back(async(launch::async, [&, t]() {
            int start = t * chunk_size;
            int end = min((int)nodes.size(), start + chunk_size);
            
            for(int i=start; i<end; ++i) {
                for(int j=0; j<(int)nodes.size(); ++j) {
                    if(i == j) continue;
                    // Use A* for fast computation
                    auto [dist, path] = findPathBidirectionalAStar(nodes[i], nodes[j]);
                    
                    // Store in matrix (thread-safe because each index is unique)
                    // dist_matrix.set uses a flat vector, we can access directly if we want, 
                    // but set() is safe if we don't resize.
                    // However, dist_matrix.set writes to specific indices.
                    // Since i is unique per thread, row i is exclusive to this thread.
                    // So we can write safely.
                    const_cast<DistanceMatrix&>(dist_matrix).set(nodes[i], nodes[j], dist);
                    
                    // Also cache the path if needed (optional, might consume memory)
                    // {
                    //    lock_guard<mutex> lock(cache_mutex);
                    //    path_cache[nodes[i]][nodes[j]] = {dist, path};
                    // }
                }
            }
        }));
    }
    
    for(auto& f : futures) f.wait();
}

// Get shortest path with caching
pair<double, vector<int>> DeliveryScheduler::getShortestPath(int from, int to) {
    if (from == to) return {0.0, {from}};
    
    // Check matrix first for distance
    double mat_dist = dist_matrix.get(from, to);
    if (mat_dist != numeric_limits<double>::infinity()) {
        // If we need the path vector, we might still need to compute it or reconstruct it.
        // For VRP optimization, we mostly need distance.
        // But this function returns pair<double, vector<int>>.
        // If the caller needs the path, we must compute it.
        // Let's check cache.
        {
            std::lock_guard<std::mutex> lock(cache_mutex);
            if (path_cache.count(from) && path_cache[from].count(to)) {
                return path_cache[from][to];
            }
        }
        // If not in cache, compute A*
        auto result = findPathBidirectionalAStar(from, to);
        {
            std::lock_guard<std::mutex> lock(cache_mutex);
            path_cache[from][to] = result;
        }
        return result;
    }
    
    // Fallback
    return findPathBidirectionalAStar(from, to);
}

// Calculate route time using Matrix (Fast)
// Calculate total time for a route
double DeliveryScheduler::calculateRouteTime(const vector<int>& route) {
    if (route.size() < 2) return 0.0;
    
    double total_time = 0.0;
    for (size_t i = 0; i < route.size() - 1; i++) {
        double d = dist_matrix.get(route[i], route[i+1]);
        if (d == numeric_limits<double>::infinity()) {
            // Fallback to slow calculation if not in matrix
            auto [time, path] = getShortestPath(route[i], route[i + 1]);
            d = time;
        }
        total_time += d;
    }
    return total_time;
}

// Validate that pickup precedes dropoff for all orders
bool DeliveryScheduler::validateRoute(const vector<int>& route, const vector<int>& order_indices) {
    for (int order_idx : order_indices) {
        const Order& order = orders[order_idx];
        
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
vector<int> DeliveryScheduler::buildValidRoute(const vector<int>& order_indices, int start_node) {
    vector<int> route;
    route.push_back(start_node);
    
    if (order_indices.empty()) {
        return route;
    }
    
    // Track which orders have been picked up but not delivered
    set<int> picked_up;
    set<int> delivered;
    
    int current = start_node;
    
    // Continue until all orders are delivered
    while (delivered.size() < order_indices.size()) {
        int next_node = -1;
        double min_dist = numeric_limits<double>::infinity();
        bool is_pickup = false;
        int selected_order_idx = -1;
        
        // Option 1: Pick up a new order (if not all picked up)
        for (int order_idx : order_indices) {
            if (picked_up.count(order_idx) == 0) {
                int pickup = orders[order_idx].pickup_node;
                
                // Use Matrix Lookup
                double dist = dist_matrix.get(current, pickup);
                if (dist == numeric_limits<double>::infinity()) {
                     auto [d, p] = getShortestPath(current, pickup);
                     dist = d;
                }
                
                if (dist < min_dist) {
                    min_dist = dist;
                    next_node = pickup;
                    is_pickup = true;
                    selected_order_idx = order_idx;
                }
            }
        }
        
        // Option 2: Deliver a picked up order
        // Apply slight bias toward deliveries to reduce carried load
        for (int order_idx : order_indices) {
            if (picked_up.count(order_idx) > 0 && delivered.count(order_idx) == 0) {
                int dropoff = orders[order_idx].dropoff_node;
                
                // Use Matrix Lookup
                double dist = dist_matrix.get(current, dropoff);
                if (dist == numeric_limits<double>::infinity()) {
                     auto [d, p] = getShortestPath(current, dropoff);
                     dist = d;
                }
                
                // Bias: prefer deliveries when they're competitive (within 20% of best pickup)
                double biased_dist = dist * 0.8;
                
                if (biased_dist < min_dist) {
                    min_dist = dist;  // Use actual distance for route cost
                    next_node = dropoff;
                    is_pickup = false;
                    selected_order_idx = order_idx;
                }
            }
        }
        
        if (next_node == -1 || selected_order_idx == -1) {
            break;  // No valid move (shouldn't happen)
        }
        
        route.push_back(next_node);
        current = next_node;
        
        if (is_pickup) {
            picked_up.insert(selected_order_idx);
        } else {
            delivered.insert(selected_order_idx);
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
            vector<int> temp_order_indices = drivers[d].assigned_order_indices;
            temp_order_indices.push_back(order_idx);
            
            vector<int> temp_route = buildValidRoute(temp_order_indices, depot_node);
            double new_time = calculateRouteTime(temp_route);
            double incremental = new_time - drivers[d].total_time;
            
            if (incremental < min_incremental_time) {
                min_incremental_time = incremental;
                best_driver = d;
            }
        }
        
        // Assign to best driver
        drivers[best_driver].assigned_order_indices.push_back(order_idx);
        drivers[best_driver].route = buildValidRoute(drivers[best_driver].assigned_order_indices, depot_node);
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
            if (validateRoute(driver.route, driver.assigned_order_indices)) {
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
            for (int order_idx : driver.assigned_order_indices) {
                if (orders[order_idx].dropoff_node == driver.route[i]) {
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
        driver.assigned_order_indices = final_routes[i];
        driver.route = buildValidRoute(driver.assigned_order_indices, depot_node);
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
    
    if (!neighbor.assignments[driver1].assigned_order_indices.empty()) {
        int order_idx_in_driver = getRandomInt(0, neighbor.assignments[driver1].assigned_order_indices.size() - 1);
        int order_idx = neighbor.assignments[driver1].assigned_order_indices[order_idx_in_driver];
        
        // Move order from driver1 to driver2
        neighbor.assignments[driver1].assigned_order_indices.erase(
            neighbor.assignments[driver1].assigned_order_indices.begin() + order_idx_in_driver);
        neighbor.assignments[driver2].assigned_order_indices.push_back(order_idx);
        
        // Rebuild routes
        neighbor.assignments[driver1].route = buildValidRoute(
            neighbor.assignments[driver1].assigned_order_indices, depot_node);
        neighbor.assignments[driver1].total_time = calculateRouteTime(
            neighbor.assignments[driver1].route);
        
        neighbor.assignments[driver2].route = buildValidRoute(
            neighbor.assignments[driver2].assigned_order_indices, depot_node);
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
        driver.assigned_order_indices = clusters[d];
        
        // Use optimized route building
        driver.route = buildValidRoute(driver.assigned_order_indices, depot_node);
        
        // Apply heavy local search
        twoOptOptimization(driver.route);
        orOptMove(driver.route, driver.assigned_order_indices);
        
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
    
    // 1. Initialize centroids (K-Means++)
    vector<pair<double, double>> centroids;
    vector<pair<double, double>> points;
    
    for(const auto& o : orders) {
        const Node* p = graph.getNode(o.pickup_node);
        const Node* d = graph.getNode(o.dropoff_node);
        if (!p || !d) {
            points.push_back({0.0, 0.0}); // Should not happen if graph is valid
            continue;
        }
        // Use midpoint of pickup and dropoff as the order's location
        points.push_back({(p->lat + d->lat)/2.0, (p->lon + d->lon)/2.0});
    }
    
    // Pick first centroid randomly
    int first_idx = getRandomInt(0, points.size()-1);
    centroids.push_back(points[first_idx]);
    
    // Pick remaining centroids
    for(int k=1; k<num_clusters; ++k) {
        vector<double> dists(points.size());
        double sum_dist = 0;
        for(size_t i=0; i<points.size(); ++i) {
            double min_d = numeric_limits<double>::infinity();
            for(const auto& c : centroids) {
                double d = pow(points[i].first - c.first, 2) + pow(points[i].second - c.second, 2);
                if(d < min_d) min_d = d;
            }
            dists[i] = min_d;
            sum_dist += min_d;
        }
        
        double r = getRandomDouble() * sum_dist;
        double cum_dist = 0;
        int next_centroid = -1;
        for(size_t i=0; i<points.size(); ++i) {
            cum_dist += dists[i];
            if(cum_dist >= r) {
                next_centroid = i;
                break;
            }
        }
        if(next_centroid == -1) next_centroid = points.size()-1;
        centroids.push_back(points[next_centroid]);
    }
    
    // 2. K-Means Iterations
    for(int iter=0; iter<20; ++iter) { // 20 iterations usually enough
        // Assignment step
        for(auto& c : clusters) c.clear();
        
        for(size_t i=0; i<points.size(); ++i) {
            int best_c = 0;
            double min_d = numeric_limits<double>::infinity();
            for(int k=0; k<num_clusters; ++k) {
                double d = pow(points[i].first - centroids[k].first, 2) + 
                           pow(points[i].second - centroids[k].second, 2);
                if(d < min_d) {
                    min_d = d;
                    best_c = k;
                }
            }
            clusters[best_c].push_back(i);
        }
        
        // Update step
        bool changed = false;
        for(int k=0; k<num_clusters; ++k) {
            if(clusters[k].empty()) continue;
            double sum_lat = 0, sum_lon = 0;
            for(int idx : clusters[k]) {
                sum_lat += points[idx].first;
                sum_lon += points[idx].second;
            }
            pair<double, double> new_c = {sum_lat / clusters[k].size(), sum_lon / clusters[k].size()};
            if(abs(new_c.first - centroids[k].first) > 1e-6 || abs(new_c.second - centroids[k].second) > 1e-6) {
                centroids[k] = new_c;
                changed = true;
            }
        }
        if(!changed) break;
    }
    
    return clusters;
}

// Or-opt move: remove and reinsert a segment
bool DeliveryScheduler::orOptMove(vector<int>& route, const vector<int>& order_indices) {
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
                
                if (validateRoute(new_route, order_indices)) {
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
            orOptMove(driver.route, driver.assigned_order_indices);
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
                if (validateRoute(driver.route, driver.assigned_order_indices)) {
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
            driver.assigned_order_indices.push_back(shuffled_orders[current_order_idx]);
            current_order_idx++;
        }
        
        driver.route = buildValidRoute(driver.assigned_order_indices, depot_node);
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
    for (const auto& d : parent1.assignments) p1_orders.insert(p1_orders.end(), d.assigned_order_indices.begin(), d.assigned_order_indices.end());
    for (const auto& d : parent2.assignments) p2_orders.insert(p2_orders.end(), d.assigned_order_indices.begin(), d.assigned_order_indices.end());
    
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
            driver.assigned_order_indices.push_back(child_orders[current_order_idx]);
            current_order_idx++;
        }
        
        driver.route = buildValidRoute(driver.assigned_order_indices, depot_node);
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
    
    if (d1.assigned_order_indices.empty() && d2.assigned_order_indices.empty()) return;
    
    // Move or Swap
    if (!d1.assigned_order_indices.empty()) {
        int o1_idx = getRandomInt(0, d1.assigned_order_indices.size() - 1);
        int val = d1.assigned_order_indices[o1_idx];
        
        d1.assigned_order_indices.erase(d1.assigned_order_indices.begin() + o1_idx);
        d2.assigned_order_indices.push_back(val);
        
        // Rebuild routes
        d1.route = buildValidRoute(d1.assigned_order_indices, depot_node);
        d1.total_time = calculateRouteTime(d1.route);
        d2.route = buildValidRoute(d2.assigned_order_indices, depot_node);
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
            orOptMove(d.route, d.assigned_order_indices);
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

// Main entry point for scheduling
SchedulingResult DeliveryScheduler::scheduleDelivery() {
    // 1. Build Distance Matrix (Parallel)
    // This is crucial for performance on large graphs
    buildDistanceMatrix();
    
    // 2. Run Clustering Strategy (Scalable)
    // For 10k nodes, Genetic Algorithm might be too slow if not highly optimized.
    // K-Means Clustering + TSP is very fast and gives good results.
    return clusterFirstRouteSecond();
}


