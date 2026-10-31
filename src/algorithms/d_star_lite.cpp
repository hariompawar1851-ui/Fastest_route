#include "algorithms/d_star_lite.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>

namespace dynamic_route {

DStarLite::DStarLite(const Graph& graph, NodeId start, NodeId goal)
    : graph_(graph), start_(start), goal_(goal), last_start_(start) {
    const std::size_t n = graph_.num_nodes();
    g_.assign(n, INF_WEIGHT);
    rhs_.assign(n, INF_WEIGHT);
    in_open_list_.assign(n, false);
    queued_key_.assign(n, LPAKey{0.0, 0.0});

    rhs_[goal_] = 0.0;
    queued_key_[goal_] = calculate_key(goal_);
    open_list_.insert({queued_key_[goal_], goal_});
    in_open_list_[goal_] = true;
}

double DStarLite::heuristic(NodeId u, NodeId v) const {
    return graph_.euclidean_distance(u, v);
}

LPAKey DStarLite::calculate_key(NodeId u) const {
    double min_val = std::min(g_[u], rhs_[u]);
    return {min_val + heuristic(start_, u) + km_, min_val};
}

void DStarLite::update_vertex(NodeId u) {
    if (u != goal_) {
        double min_rhs = INF_WEIGHT;
        for (const auto& edge : graph_.get_outgoing_edges(u)) {
            double w = edge.effective_weight();
            if (std::isinf(w)) continue;
            NodeId s = edge.to;
            if (g_[s] < INF_WEIGHT && w + g_[s] < min_rhs) {
                min_rhs = w + g_[s];
            }
        }
        rhs_[u] = min_rhs;
    }

    // O(log n): erase the exact (key, node) element we inserted earlier.
    if (in_open_list_[u]) {
        open_list_.erase({queued_key_[u], u});
        in_open_list_[u] = false;
    }

    if (std::abs(g_[u] - rhs_[u]) > 1e-9) {
        queued_key_[u] = calculate_key(u);
        open_list_.insert({queued_key_[u], u});
        in_open_list_[u] = true;
    }
}

SearchResult DStarLite::compute_shortest_path() {
    auto start_time = std::chrono::high_resolution_clock::now();
    SearchResult result;
    std::size_t expansions = 0;

    while (!open_list_.empty()) {
        auto top_it = open_list_.begin();
        LPAKey k_old = top_it->first;
        NodeId u = top_it->second;

        LPAKey k_start = calculate_key(start_);
        if (!(k_old < k_start) && std::abs(rhs_[start_] - g_[start_]) <= 1e-9) {
            break;
        }

        open_list_.erase(top_it);
        in_open_list_[u] = false;
        LPAKey k_new = calculate_key(u);

        if (k_old < k_new) {
            queued_key_[u] = k_new;
            open_list_.insert({k_new, u});
            in_open_list_[u] = true;
        } else if (g_[u] > rhs_[u]) {
            g_[u] = rhs_[u];
            ++expansions;
            for (const auto& edge : graph_.get_incoming_edges(u)) {
                update_vertex(edge.from);
            }
        } else {
            g_[u] = INF_WEIGHT;
            ++expansions;
            update_vertex(u);
            for (const auto& edge : graph_.get_incoming_edges(u)) {
                update_vertex(edge.from);
            }
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    result.duration_micros = std::chrono::duration<double, std::micro>(end_time - start_time).count();
    result.nodes_visited = expansions;
    total_expansions_ += expansions;

    if (g_[start_] < INF_WEIGHT) {
        result.path_found = true;
        result.total_cost = g_[start_];

        // Trace forward from start to goal
        NodeId curr = start_;
        result.path.push_back(curr);
        while (curr != goal_) {
            NodeId best_succ = curr;
            double best_cost = INF_WEIGHT;
            for (const auto& edge : graph_.get_outgoing_edges(curr)) {
                double w = edge.effective_weight();
                if (std::isinf(w)) continue;
                NodeId s = edge.to;
                if (w + g_[s] < best_cost) {
                    best_cost = w + g_[s];
                    best_succ = s;
                }
            }
            if (best_succ == curr || best_cost >= INF_WEIGHT) {
                result.path_found = false;
                result.path.clear();
                break;
            }
            curr = best_succ;
            result.path.push_back(curr);
        }
    }

    return result;
}

void DStarLite::on_edge_changed(NodeId from, NodeId to, double /*new_weight*/) {
    (void)to;
    update_vertex(from);
}

void DStarLite::update_start(NodeId new_start) {
    km_ += heuristic(last_start_, new_start);
    last_start_ = new_start;
    start_ = new_start;
}

double DStarLite::get_g_value(NodeId u) const {
    return (u < g_.size()) ? g_[u] : INF_WEIGHT;
}

double DStarLite::get_rhs_value(NodeId u) const {
    return (u < rhs_.size()) ? rhs_[u] : INF_WEIGHT;
}

bool DStarLite::queue_invariant_holds() const {
    std::size_t flagged = 0;
    for (NodeId u = 0; u < g_.size(); ++u) {
        const bool inconsistent = std::abs(g_[u] - rhs_[u]) > 1e-9;
        if (inconsistent != in_open_list_[u]) return false;
        if (in_open_list_[u]) {
            ++flagged;
            if (open_list_.find({queued_key_[u], u}) == open_list_.end()) return false;
        }
    }
    return flagged == open_list_.size();
}

} // namespace dynamic_route
