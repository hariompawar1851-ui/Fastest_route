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

    // --- Phase 3: moving-agent state & diagnostics -------------------------
    NodeId current_start() const noexcept { return start_; }
    NodeId goal() const noexcept { return goal_; }
    double km() const noexcept { return km_; }
    std::size_t queue_size() const noexcept { return open_list_.size(); }
    /// Cumulative vertex expansions across every compute_shortest_path() call.
    std::size_t total_expansions() const noexcept { return total_expansions_; }
    /// True iff the queue holds exactly the locally inconsistent vertices
    /// (g != rhs) and the bookkeeping vectors agree with the set.
    bool queue_invariant_holds() const;

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

    // Indexed-queue bookkeeping: lets update_vertex erase in O(log n).
    std::vector<bool> in_open_list_;
    std::vector<LPAKey> queued_key_;

    std::size_t total_expansions_{0};
};

} // namespace dynamic_route
