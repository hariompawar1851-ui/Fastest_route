#pragma once

#include "graph/graph.hpp"
#include <vector>

namespace dynamic_route {

struct SearchResult {
    bool path_found{false};
    std::vector<NodeId> path{};
    double total_cost{INF_WEIGHT};
    std::size_t nodes_visited{0};
    double duration_micros{0.0};
};

class Dijkstra {
public:
    static SearchResult find_shortest_path(const Graph& graph, NodeId source, NodeId target);
};

} // namespace dynamic_route
