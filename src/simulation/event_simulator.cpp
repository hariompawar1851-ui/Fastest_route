#include "simulation/event_simulator.hpp"
#include <stdexcept>

namespace dynamic_route {

EventSimulator::EventSimulator(std::uint32_t seed) : rng_(seed) {}

Graph EventSimulator::create_grid_graph(std::size_t rows, std::size_t cols, double spacing) {
    Graph graph;
    for (std::size_t r = 0; r < rows; ++r) {
        for (std::size_t c = 0; c < cols; ++c) {
            graph.add_node(static_cast<double>(c) * spacing,
                           static_cast<double>(r) * spacing,
                           "N_" + std::to_string(r) + "_" + std::to_string(c));
        }
    }

    auto get_id = [cols](std::size_t r, std::size_t c) {
        return r * cols + c;
    };

    for (std::size_t r = 0; r < rows; ++r) {
        for (std::size_t c = 0; c < cols; ++c) {
            NodeId u = get_id(r, c);
            if (c + 1 < cols) {
                NodeId v = get_id(r, c + 1);
                graph.add_undirected_edge(u, v, spacing);
            }
            if (r + 1 < rows) {
                NodeId v = get_id(r + 1, c);
                graph.add_undirected_edge(u, v, spacing);
            }
        }
    }

    return graph;
}

PerturbationEvent EventSimulator::simulate_edge_blockage(Graph& graph, NodeId from, NodeId to) {
    auto current_w = graph.get_edge_weight(from, to).value_or(1.0);
    graph.set_edge_blocked(from, to, true);
    return PerturbationEvent{from, to, current_w, INF_WEIGHT, true};
}

PerturbationEvent EventSimulator::simulate_congestion(Graph& graph, NodeId from, NodeId to, double factor) {
    auto current_w = graph.get_edge_weight(from, to).value_or(1.0);
    double new_w = current_w * factor;
    graph.update_edge_weight(from, to, new_w);
    return PerturbationEvent{from, to, current_w, new_w, false};
}

PerturbationEvent EventSimulator::generate_random_event(Graph& graph) {
    if (graph.num_nodes() < 2) {
        throw std::runtime_error("Graph too small for random events");
    }

    std::uniform_int_distribution<NodeId> node_dist(0, graph.num_nodes() - 1);
    for (int attempts = 0; attempts < 100; ++attempts) {
        NodeId u = node_dist(rng_);
        const auto& edges = graph.get_outgoing_edges(u);
        if (edges.empty()) continue;

        std::uniform_int_distribution<std::size_t> edge_dist(0, edges.size() - 1);
        const auto& chosen_edge = edges[edge_dist(rng_)];
        NodeId v = chosen_edge.to;

        std::bernoulli_distribution block_dist(0.3);
        if (block_dist(rng_)) {
            return simulate_edge_blockage(graph, u, v);
        } else {
            std::uniform_real_distribution<double> factor_dist(2.0, 5.0);
            return simulate_congestion(graph, u, v, factor_dist(rng_));
        }
    }

    return PerturbationEvent{};
}

} // namespace dynamic_route
