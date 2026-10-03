#include "compression/degree2_compressor.hpp"

namespace dynamic_route {

Graph Degree2Compressor::compress(const Graph& raw_graph, std::vector<MacroEdge>& macro_edges) {
    macro_edges.clear();
    Graph compressed = raw_graph;
    // Optional Extension: scan for degree-2 non-junction vertices
    // In Phase 0 skeleton, we establish the interface and passthrough
    return compressed;
}

std::vector<NodeId> Degree2Compressor::unpack_path(const std::vector<NodeId>& compressed_path,
                                                   const std::vector<MacroEdge>& /*macro_edges*/) {
    // Return unpacked full geometry
    return compressed_path;
}

} // namespace dynamic_route
