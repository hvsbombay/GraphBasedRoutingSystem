#ifndef KNN_HPP
#define KNN_HPP

#include "Graph.hpp"
#include <vector>
#include <string>

using namespace std;

class KNN {
private:
    const Graph& graph;
    
public:
    KNN(const Graph& g) : graph(g) {}
    
    // Find k nearest POIs based on Euclidean distance
    vector<int> findKNearestEuclidean(double query_lat, double query_lon,
                                     const string& poi_type, int k);
    
    // Find k nearest POIs based on shortest path distance
    vector<int> findKNearestShortestPath(double query_lat, double query_lon,
                                        const string& poi_type, int k);
};

#endif // KNN_HPP
