#pragma once

#include "graph/graph.hpp"
#include "algorithms/dijkstra.hpp"
#include <vector>

namespace dynamic_route {

// Optional Extension: Ramalingam-Reps Dynamic Single-Source Shortest Path Tree
class RamalingamReps {
public:
    RamalingamReps(const Graph& graph, NodeId source);

    void initialize();
    void on_edge_increase(NodeId u, NodeId v, double old_w, double new_w);
    void on_edge_decrease(NodeId u, NodeId v, double old_w, double new_w);

    double get_distance(NodeId u) const;
    std::vector<NodeId> get_path(NodeId target) const;

private:
    const Graph& graph_;
    NodeId source_;
    std::vector<double> dist_;
    std::vector<NodeId> parent_;
};

} // namespace dynamic_route
