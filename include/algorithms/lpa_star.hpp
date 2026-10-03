#pragma once

#include "graph/graph.hpp"
#include "algorithms/dijkstra.hpp"
#include <vector>
#include <set>
#include <utility>

namespace dynamic_route {

using LPAKey = std::pair<double, double>;

class LifelongPlanningAStar {
public:
    LifelongPlanningAStar(const Graph& graph, NodeId source, NodeId target);

    SearchResult compute_shortest_path();
    void on_edge_changed(NodeId from, NodeId to, double new_weight);

    double get_g_value(NodeId u) const;
    double get_rhs_value(NodeId u) const;

private:
    LPAKey calculate_key(NodeId u) const;
    void update_vertex(NodeId u);
    double heuristic(NodeId u) const;

    const Graph& graph_;
    NodeId source_;
    NodeId target_;

    std::vector<double> g_;
    std::vector<double> rhs_;

    // Set of (Key, NodeId) acts as an addressable min-priority queue
    std::set<std::pair<LPAKey, NodeId>> open_list_;
    std::vector<bool> in_open_list_;

    std::size_t nodes_expanded_total_{0};
};

} // namespace dynamic_route
