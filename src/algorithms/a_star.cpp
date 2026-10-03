#include "algorithms/a_star.hpp"
#include <queue>
#include <chrono>
#include <algorithm>

namespace dynamic_route {

SearchResult AStar::find_shortest_path(const Graph& graph, NodeId source, NodeId target) {
    auto start_time = std::chrono::high_resolution_clock::now();

    SearchResult result;
    const std::size_t n = graph.num_nodes();
    if (source >= n || target >= n) return result;

    std::vector<double> g_score(n, INF_WEIGHT);
    std::vector<NodeId> parent(n, n);
    std::vector<bool> closed(n, false);

    using FNode = std::pair<double, NodeId>;
    std::priority_queue<FNode, std::vector<FNode>, std::greater<FNode>> open_set;

    g_score[source] = 0.0;
    double h_start = graph.euclidean_distance(source, target);
    open_set.push({h_start, source});

    while (!open_set.empty()) {
        auto [f, u] = open_set.top();
        open_set.pop();

        if (closed[u]) continue;
        closed[u] = true;
        ++result.nodes_visited;

        if (u == target) break;

        for (const auto& edge : graph.get_outgoing_edges(u)) {
            double weight = edge.effective_weight();
            if (std::isinf(weight)) continue;

            NodeId v = edge.to;
            double tentative_g = g_score[u] + weight;
            if (tentative_g < g_score[v]) {
                g_score[v] = tentative_g;
                parent[v] = u;
                double f_score = tentative_g + graph.euclidean_distance(v, target);
                open_set.push({f_score, v});
            }
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    result.duration_micros = std::chrono::duration<double, std::micro>(end_time - start_time).count();

    if (g_score[target] < INF_WEIGHT) {
        result.path_found = true;
        result.total_cost = g_score[target];
        for (NodeId curr = target; curr != n; curr = parent[curr]) {
            result.path.push_back(curr);
            if (curr == source) break;
        }
        std::reverse(result.path.begin(), result.path.end());
    }

    return result;
}

} // namespace dynamic_route
