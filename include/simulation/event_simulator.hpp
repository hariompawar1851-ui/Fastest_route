#pragma once

#include "graph/graph.hpp"
#include <vector>
#include <random>

namespace dynamic_route {

struct PerturbationEvent {
    NodeId from{0};
    NodeId to{0};
    double old_weight{1.0};
    double new_weight{1.0};
    bool is_blockage{false};
};

class EventSimulator {
public:
    explicit EventSimulator(std::uint32_t seed = 42);

    // Creates an N x M 2D grid graph with Euclidean distances
    static Graph create_grid_graph(std::size_t rows, std::size_t cols, double spacing = 1.0);

    // Simulates an accident / blockage on a specific edge
    PerturbationEvent simulate_edge_blockage(Graph& graph, NodeId from, NodeId to);

    // Simulates congestion by multiplying weight by a factor
    PerturbationEvent simulate_congestion(Graph& graph, NodeId from, NodeId to, double factor);

    // Generates a random perturbation across the graph
    PerturbationEvent generate_random_event(Graph& graph);

private:
    std::mt19937 rng_;
};

} // namespace dynamic_route
