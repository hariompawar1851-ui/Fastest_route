#include "algorithms/dijkstra.hpp"
#include <queue>
#include <chrono>
#include <algorithm>

namespace dynamic_route {

SearchResult Dijkstra::find_shortest_path(const Graph& graph, NodeId source, NodeId target) {
    auto start_time = std::chrono::high_resolution_clock::now();

    SearchResult result;
    const std::size_t n = graph.num_nodes();
    if (source >= n || target >= n) return result;

    std::vector<double> dist(n, INF_WEIGHT);
    std::vector<NodeId> parent(n, n);
    std::vector<bool> visited(n, false);

    // Min-heap ordered by tentative distance
    using DistNode = std::pair<double, NodeId>;
    std::priority_queue<DistNode, std::vector<DistNode>, std::greater<DistNode>> pq;

    dist[source] = 0.0;
    pq.push({0.0, source});

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();

        if (visited[u]) continue;
        visited[u] = true;
        ++result.nodes_visited;

        if (u == target) break;

        for (const auto& edge : graph.get_outgoing_edges(u)) {
            double weight = edge.effective_weight();
            if (std::isinf(weight)) continue;

            NodeId v = edge.to;
            if (dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                parent[v] = u;
                pq.push({dist[v], v});
            }
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    result.duration_micros = std::chrono::duration<double, std::micro>(end_time - start_time).count();

    if (dist[target] < INF_WEIGHT) {
        result.path_found = true;
        result.total_cost = dist[target];
        for (NodeId curr = target; curr != n; curr = parent[curr]) {
            result.path.push_back(curr);
            if (curr == source) break;
        }
        std::reverse(result.path.begin(), result.path.end());
    }

    return result;
}

} // namespace dynamic_route
