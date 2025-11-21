#ifndef DELIVERY_SCHEDULER_HPP
#define DELIVERY_SCHEDULER_HPP

#include "Graph.hpp"
#include <vector>
#include <unordered_map>
#include <set>
#include <limits>
#include <algorithm>
#include <random>
#include <mutex>

using namespace std;

// Represents a delivery order with pickup and dropoff locations
struct Order {
    int order_id;
    int pickup_node;
    int dropoff_node;
    
    Order() : order_id(-1), pickup_node(-1), dropoff_node(-1) {}
    Order(int id, int pickup, int dropoff) 
        : order_id(id), pickup_node(pickup), dropoff_node(dropoff) {}
};

// Represents a delivery driver with their assigned route
struct Driver {
    int driver_id;
    vector<int> route;  // Sequence of nodes to visit
    vector<int> assigned_order_indices;  // Indices of orders in the 'orders' vector assigned to this driver
    double total_time;  // Total time for this driver's route
    
    Driver() : driver_id(-1), total_time(0.0) {}
    Driver(int id) : driver_id(id), total_time(0.0) {}
};

// Result of delivery scheduling
struct SchedulingResult {
    vector<Driver> assignments;
    double total_delivery_time;  // Sum of all order completion times
    double max_delivery_time;    // Maximum individual order completion time
    
    SchedulingResult() : total_delivery_time(0.0), max_delivery_time(0.0) {}
};

// Real-world event that blocks an edge
struct RoadBlock {
    int edge_id;
    double start_time;  // When the block starts (in seconds from simulation start)
    double duration;    // How long the block lasts (in seconds)
    string reason;      // "accident", "construction", "flooding", etc.
    
    RoadBlock() : edge_id(-1), start_time(0.0), duration(0.0), reason("") {}
    RoadBlock(int id, double start, double dur, string r) 
        : edge_id(id), start_time(start), duration(dur), reason(r) {}
};

class DeliveryScheduler {
private:
    Graph& graph;  // Changed to non-const for dynamic edge modification
    int depot_node;
    vector<Order> orders;
    int num_drivers;
    
    // Precomputed shortest paths cache (using Dijkstra)
    mutable std::mutex cache_mutex;
    unordered_map<int, unordered_map<int, pair<double, vector<int>>>> path_cache;
    
    // --- NEW: Real-World Scenario Handling ---
    vector<RoadBlock> road_blocks;  // Simulated road blocks
    set<int> currently_blocked_edges;  // Edges currently unavailable
    mutable std::mutex block_mutex;
    
    // --- NEW: Fast Distance Matrix for Large Scale ---
    struct DistanceMatrix {
        unordered_map<int, int> node_to_index;
        vector<int> index_to_node;
        vector<double> matrix; // Flat 2D array
        int size;

        void init(const vector<int>& nodes) {
            size = nodes.size();
            index_to_node = nodes;
            for(int i=0; i<size; ++i) node_to_index[nodes[i]] = i;
            matrix.assign(size * size, numeric_limits<double>::infinity());
            for(int i=0; i<size; ++i) matrix[i*size + i] = 0.0;
        }

        void set(int u, int v, double dist) {
            if(node_to_index.count(u) && node_to_index.count(v)) {
                int i = node_to_index[u];
                int j = node_to_index[v];
                matrix[i*size + j] = dist;
            }
        }

        double get(int u, int v) const {
            if(u == v) return 0.0;
            if(node_to_index.count(u) && node_to_index.count(v)) {
                int i = node_to_index.at(u);
                int j = node_to_index.at(v);
                return matrix[i*size + j];
            }
            return numeric_limits<double>::infinity();
        }
    } dist_matrix;
    
    // --- NEW: Real-World Scenario Methods (Private) ---
    // Apply road blocks at given simulation time
    void applyRoadBlocks(double current_time);
    
    // Remove expired road blocks
    void clearExpiredBlocks(double current_time);
    
    // Check if an edge is currently blocked
    bool isEdgeBlocked(int edge_id) const;
    
    // Invalidate cache entries that use blocked edges
    void invalidateBlockedPaths();

    // Initialize distance matrix for all key points (depot, pickups, dropoffs)
    void buildDistanceMatrix();
    
    // Compute shortest path between two nodes (with caching)
    pair<double, vector<int>> getShortestPath(int from, int to);
    
    // Compute shortest path using Bidirectional A* (Optimized)
    pair<double, vector<int>> findPathBidirectionalAStar(int source, int target);
    
    // Calculate route time for a sequence of nodes using Matrix
    double calculateRouteTime(const vector<int>& route);
    
    // Validate that pickup precedes dropoff for all orders in route
    bool validateRoute(const vector<int>& route, const vector<int>& order_indices);
    
    // Greedy assignment: assign orders to drivers one by one
    SchedulingResult greedyAssignment();
    
    // Nearest neighbor heuristic for TSP
    vector<int> nearestNeighborTSP(const vector<int>& nodes, int start);
    
    // 2-opt local search optimization
    void twoOptOptimization(vector<int>& route);
    
    // Calculate total delivery time for all orders
    double calculateTotalDeliveryTime(const vector<Driver>& drivers);
    
    // Merge driver routes if beneficial
    void mergeRoutes(vector<Driver>& drivers);
    
    // Build route from order assignments ensuring pickup before dropoff
    vector<int> buildValidRoute(const vector<int>& order_indices, int start_node);
    
    // Advanced algorithms
    SchedulingResult savingsAlgorithm();  // Clarke-Wright Savings
    SchedulingResult simulatedAnnealing(SchedulingResult initial, int max_iterations = 1000);
    SchedulingResult clusterFirstRouteSecond();  // Spatial clustering approach
    
    // Helper for simulated annealing
    SchedulingResult perturbSolution(const SchedulingResult& current);
    double calculateCost(const SchedulingResult& solution);
    
    // Clustering helper
    vector<vector<int>> clusterOrders(int num_clusters);
    
    // Or-opt move (remove and reinsert segment)
    bool orOptMove(vector<int>& route, const vector<int>& order_indices);
    
    // Adaptive strategy selector based on problem size
    SchedulingResult adaptiveSchedule();
    
    // Genetic Algorithm implementation
    struct Individual {
        vector<Driver> assignments;
        double fitness; // Total delivery time
        
        bool operator<(const Individual& other) const {
            return fitness < other.fitness; // Lower time is better
        }
    };
    
    SchedulingResult geneticAlgorithm(int population_size, int generations);
    Individual crossover(const Individual& parent1, const Individual& parent2);
    void mutate(Individual& ind);
    Individual createRandomIndividual();
    
    // Parallel Portfolio Strategy
    SchedulingResult parallelPortfolioSchedule();

public:
    DeliveryScheduler(Graph& g, int depot, const vector<Order>& orders, int num_drivers);
    
    // Main scheduling algorithm
    SchedulingResult schedule();
    
    // Alternative: minimize maximum delivery time
    SchedulingResult scheduleMinMax();
    
    // Main entry point for scheduling
    SchedulingResult scheduleDelivery();
    
    // --- PUBLIC: Real-World Scenario Management ---
    // Add a road block event (simulates accident, construction, etc.)
    void addRoadBlock(int edge_id, double start_time, double duration, const string& reason);
    
    // Recompute route when road conditions change
    SchedulingResult replanWithBlocks(const SchedulingResult& original_plan, double current_time);

};

#endif // DELIVERY_SCHEDULER_HPP
