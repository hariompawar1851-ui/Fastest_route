#pragma once

#include "graph/graph.hpp"
#include "algorithms/dijkstra.hpp"

namespace dynamic_route {

class AStar {
public:
    static SearchResult find_shortest_path(const Graph& graph, NodeId source, NodeId target);
};

} // namespace dynamic_route
