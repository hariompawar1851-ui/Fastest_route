#include "algorithms/lpa_star.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>

namespace dynamic_route {

LifelongPlanningAStar::LifelongPlanningAStar(const Graph& graph, NodeId source, NodeId target)
    : graph_(graph), source_(source), target_(target) {
    const std::size_t n = graph_.num_nodes();
    g_.assign(n, INF_WEIGHT);
    rhs_.assign(n, INF_WEIGHT);
    in_open_list_.assign(n, false);

    rhs_[source_] = 0.0;
    open_list_.insert({calculate_key(source_), source_});
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
        for (auto it = open_list_.begin(); it != open_list_.end(); ++it) {
            if (it->second == u) {
                open_list_.erase(it);
                in_open_list_[u] = false;
                break;
            }
        }
    }

    if (std::abs(g_[u] - rhs_[u]) > 1e-9) {
        open_list_.insert({calculate_key(u), u});
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

        if (!(top_key < target_key) && std::abs(rhs_[target_] - g_[target_]) <= 1e-9) {
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
            if (best_pred == curr || best_cost >= INF_WEIGHT) {
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

} // namespace dynamic_route
