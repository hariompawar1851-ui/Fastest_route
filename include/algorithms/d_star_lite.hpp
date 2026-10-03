#pragma once

#include "graph/graph.hpp"
#include "algorithms/dijkstra.hpp"
#include "algorithms/lpa_star.hpp"   // brings in LPAKey
#include <vector>
#include <set>
#include <utility>

namespace dynamic_route {

class DStarLite {
public:
    DStarLite(const Graph& graph, NodeId start, NodeId goal);

    SearchResult compute_shortest_path();
    void on_edge_changed(NodeId from, NodeId to, double new_weight);
    void update_start(NodeId new_start);

    double get_g_value(NodeId u) const;
    double get_rhs_value(NodeId u) const;

private:
    LPAKey calculate_key(NodeId u) const;
    void update_vertex(NodeId u);
    double heuristic(NodeId u, NodeId v) const;

    const Graph& graph_;
    NodeId start_;
    NodeId goal_;
    NodeId last_start_;

    double km_{0.0};

    std::vector<double> g_;
    std::vector<double> rhs_;

    std::set<std::pair<LPAKey, NodeId>> open_list_;
};

} // namespace dynamic_route
