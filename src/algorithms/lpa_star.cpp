#include "algorithms/lpa_star.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>

namespace dynamic_route {

namespace {
inline bool is_consistent(double g, double rhs) noexcept {
    if (g == rhs) return true;  // Handles INF == INF correctly without NaN arithmetic
    return std::abs(g - rhs) <= 1e-9;
}

inline bool is_inconsistent(double g, double rhs) noexcept {
    if (g == rhs) return false;
    return std::abs(g - rhs) > 1e-9;
}
} // anonymous namespace

LifelongPlanningAStar::LifelongPlanningAStar(const Graph& graph, NodeId source, NodeId target)
    : graph_(graph), source_(source), target_(target) {
    const std::size_t n = graph_.num_nodes();
    g_.assign(n, INF_WEIGHT);
    rhs_.assign(n, INF_WEIGHT);
    in_open_list_.assign(n, false);
    queued_key_.assign(n, LPAKey{0.0, 0.0});

    rhs_[source_] = 0.0;
    queued_key_[source_] = calculate_key(source_);
    open_list_.insert({queued_key_[source_], source_});
    in_open_list_[source_] = true;
}

double LifelongPlanningAStar::heuristic(NodeId u) const {
    return graph_.euclidean_distance(u, target_);
}

LPAKey LifelongPlanningAStar::calculate_key(NodeId u) const {
    double min_val = std::min(g_[u], rhs_[u]);
    return {min_val + heuristic(u), min_val};
}

void LifelongPlanningAStar::update_vertex(NodeId u) {
    if (u != source_) {
        double min_rhs = INF_WEIGHT;
        for (const auto& edge : graph_.get_incoming_edges(u)) {
            double w = edge.effective_weight();
            if (std::isinf(w)) continue;
            NodeId p = edge.from;
            if (g_[p] < INF_WEIGHT && g_[p] + w < min_rhs) {
                min_rhs = g_[p] + w;
            }
        }
        rhs_[u] = min_rhs;
    }

    if (in_open_list_[u]) {
        // O(log n): erase the exact (key, node) element we inserted earlier.
        open_list_.erase({queued_key_[u], u});
        in_open_list_[u] = false;
    }

    if (is_inconsistent(g_[u], rhs_[u])) {
        queued_key_[u] = calculate_key(u);
        open_list_.insert({queued_key_[u], u});
        in_open_list_[u] = true;
    }
}

SearchResult LifelongPlanningAStar::compute_shortest_path() {
    auto start_time = std::chrono::high_resolution_clock::now();
    SearchResult result;
    std::size_t expansions = 0;

    while (!open_list_.empty()) {
        auto top_it = open_list_.begin();
        LPAKey top_key = top_it->first;
        LPAKey target_key = calculate_key(target_);

        if (!(top_key < target_key) && is_consistent(rhs_[target_], g_[target_])) {
            break;
        }

        NodeId u = top_it->second;
        open_list_.erase(top_it);
        in_open_list_[u] = false;
        ++expansions;

        if (g_[u] > rhs_[u]) {
            // Overconsistent
            g_[u] = rhs_[u];
            for (const auto& edge : graph_.get_outgoing_edges(u)) {
                update_vertex(edge.to);
            }
        } else {
            // Underconsistent
            g_[u] = INF_WEIGHT;
            update_vertex(u);
            for (const auto& edge : graph_.get_outgoing_edges(u)) {
                update_vertex(edge.to);
            }
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    result.duration_micros = std::chrono::duration<double, std::micro>(end_time - start_time).count();
    nodes_expanded_total_ += expansions;
    result.nodes_visited = expansions;

    if (g_[target_] < INF_WEIGHT) {
        result.path_found = true;
        result.total_cost = g_[target_];

        // Backtrack path from target to source
        NodeId curr = target_;
        result.path.push_back(curr);
        const std::size_t n = graph_.num_nodes();
        while (curr != source_) {
            NodeId best_pred = curr;
            double best_cost = INF_WEIGHT;
            for (const auto& edge : graph_.get_incoming_edges(curr)) {
                double w = edge.effective_weight();
                if (std::isinf(w)) continue;
                NodeId p = edge.from;
                if (g_[p] + w < best_cost) {
                    best_cost = g_[p] + w;
                    best_pred = p;
                }
            }
            if (best_pred == curr || best_cost >= INF_WEIGHT || result.path.size() > n) {
                result.path_found = false;
                result.path.clear();
                break;
            }
            curr = best_pred;
            result.path.push_back(curr);
        }
        std::reverse(result.path.begin(), result.path.end());
    }

    return result;
}

void LifelongPlanningAStar::on_edge_changed(NodeId from, NodeId to, double /*new_weight*/) {
    (void)from;
    update_vertex(to);
}

double LifelongPlanningAStar::get_g_value(NodeId u) const {
    return (u < g_.size()) ? g_[u] : INF_WEIGHT;
}

double LifelongPlanningAStar::get_rhs_value(NodeId u) const {
    return (u < rhs_.size()) ? rhs_[u] : INF_WEIGHT;
}

bool LifelongPlanningAStar::queue_invariant_holds() const {
    std::size_t flagged = 0;
    for (NodeId u = 0; u < g_.size(); ++u) {
        const bool inconsistent = is_inconsistent(g_[u], rhs_[u]);
        if (inconsistent != in_open_list_[u]) return false;
        if (in_open_list_[u]) {
            ++flagged;
            if (open_list_.find({queued_key_[u], u}) == open_list_.end()) return false;
        }
    }
    return flagged == open_list_.size();
}

} // namespace dynamic_route
