#include "algorithms/ramalingam_reps.hpp"
#include <queue>
#include <algorithm>

namespace dynamic_route {

RamalingamReps::RamalingamReps(const Graph& graph, NodeId source)
    : graph_(graph), source_(source) {
    const std::size_t n = graph_.num_nodes();
    dist_.assign(n, INF_WEIGHT);
    parent_.assign(n, n);
}

void RamalingamReps::initialize() {
    auto res = Dijkstra::find_shortest_path(graph_, source_, graph_.num_nodes() - 1);
    (void)res;
    // Basic initialization for SSSP
    const std::size_t n = graph_.num_nodes();
    dist_.assign(n, INF_WEIGHT);
    parent_.assign(n, n);

    using DistNode = std::pair<double, NodeId>;
    std::priority_queue<DistNode, std::vector<DistNode>, std::greater<DistNode>> pq;

    dist_[source_] = 0.0;
    pq.push({0.0, source_});

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        if (d > dist_[u]) continue;

        for (const auto& edge : graph_.get_outgoing_edges(u)) {
            double w = edge.effective_weight();
            if (std::isinf(w)) continue;
            NodeId v = edge.to;
            if (dist_[u] + w < dist_[v]) {
                dist_[v] = dist_[u] + w;
                parent_[v] = u;
                pq.push({dist_[v], v});
            }
        }
    }
}

void RamalingamReps::on_edge_increase(NodeId u, NodeId v, double /*old_w*/, double /*new_w*/) {
    // RR Subtree invalidation: if (u, v) is in shortest path tree, re-evaluate
    if (parent_[v] == u) {
        initialize(); // Recompute SSSP subtree
    }
}

void RamalingamReps::on_edge_decrease(NodeId u, NodeId v, double /*old_w*/, double new_w) {
    if (dist_[u] + new_w < dist_[v]) {
        dist_[v] = dist_[u] + new_w;
        parent_[v] = u;
        initialize();
    }
}

double RamalingamReps::get_distance(NodeId u) const {
    return (u < dist_.size()) ? dist_[u] : INF_WEIGHT;
}

std::vector<NodeId> RamalingamReps::get_path(NodeId target) const {
    std::vector<NodeId> path;
    const std::size_t n = graph_.num_nodes();
    if (target >= n || dist_[target] >= INF_WEIGHT) return path;

    for (NodeId curr = target; curr != n; curr = parent_[curr]) {
        path.push_back(curr);
        if (curr == source_) break;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

} // namespace dynamic_route
