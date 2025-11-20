#ifndef DELIVERY_SCHEDULER_HPP
#define DELIVERY_SCHEDULER_HPP

#include "Graph.hpp"
#include <vector>
#include <unordered_map>
#include <set>
#include <limits>
#include <algorithm>
#include <random>

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
    vector<int> order_ids;  // Orders assigned to this driver
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

class DeliveryScheduler {
private:
    const Graph& graph;
    int depot_node;
    vector<Order> orders;
    int num_drivers;
    
    // Precomputed shortest paths cache (using Dijkstra)
    unordered_map<int, unordered_map<int, pair<double, vector<int>>>> path_cache;
    
    // Compute shortest path between two nodes (with caching)
    pair<double, vector<int>> getShortestPath(int from, int to);
    
    // Compute shortest path using Dijkstra's algorithm
    pair<double, vector<int>> dijkstra(int source, int target);
    
    // Calculate route time for a sequence of nodes
    double calculateRouteTime(const vector<int>& route);
    
    // Validate that pickup precedes dropoff for all orders in route
    bool validateRoute(const vector<int>& route, const vector<int>& order_ids);
    
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
    vector<int> buildValidRoute(const vector<int>& order_ids, int start_node);
    
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
    bool orOptMove(vector<int>& route, const vector<int>& order_ids);
    
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
    DeliveryScheduler(const Graph& g, int depot, const vector<Order>& orders, int num_drivers);
    
    // Main scheduling algorithm
    SchedulingResult schedule();
    
    // Alternative: minimize maximum delivery time
    SchedulingResult scheduleMinMax();
};

#endif // DELIVERY_SCHEDULER_HPP
