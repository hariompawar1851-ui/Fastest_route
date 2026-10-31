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

    // --- Phase 3 diagnostics ---------------------------------------------
    /// Number of vertices currently waiting in the priority queue.
    std::size_t queue_size() const noexcept { return open_list_.size(); }
    /// Cumulative vertex expansions across every compute_shortest_path() call.
    std::size_t total_expansions() const noexcept { return nodes_expanded_total_; }
    /// True iff the queue holds exactly the locally inconsistent vertices
    /// (g != rhs) and the bookkeeping vectors agree with the set.
    bool queue_invariant_holds() const;

private:
    LPAKey calculate_key(NodeId u) const;
    void update_vertex(NodeId u);
    double heuristic(NodeId u) const;

    const Graph& graph_;
    NodeId source_;
    NodeId target_;

    std::vector<double> g_;
    std::vector<double> rhs_;

    // Set of (Key, NodeId) acts as an addressable min-priority queue.
    // queued_key_[u] remembers the key u was inserted with so that removal
    // is an O(log n) erase of the exact element rather than a linear scan.
    std::set<std::pair<LPAKey, NodeId>> open_list_;
    std::vector<bool> in_open_list_;
    std::vector<LPAKey> queued_key_;

    std::size_t nodes_expanded_total_{0};
};

} // namespace dynamic_route
