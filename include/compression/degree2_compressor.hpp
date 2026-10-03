#pragma once

#include "graph/graph.hpp"
#include <vector>
#include <unordered_map>

namespace dynamic_route {

// Optional Extension: Degree-2 Topological Chain Compressor
struct MacroEdge {
    NodeId from{0};
    NodeId to{0};
    double aggregated_weight{0.0};
    std::vector<NodeId> internal_path{};
};

class Degree2Compressor {
public:
    static Graph compress(const Graph& raw_graph, std::vector<MacroEdge>& macro_edges);
    static std::vector<NodeId> unpack_path(const std::vector<NodeId>& compressed_path,
                                           const std::vector<MacroEdge>& macro_edges);
};

} // namespace dynamic_route
