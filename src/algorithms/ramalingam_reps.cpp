#include "algorithms/ramalingam_reps.hpp"
#include <algorithm>
#include <cmath>

namespace dynamic_route {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

RamalingamReps::RamalingamReps(const Graph& graph, NodeId source)
    : graph_(graph), source_(source), spt_(graph.num_nodes()) {}

// ---------------------------------------------------------------------------
// initialize() — full Dijkstra sweep to build the initial SPT
// ---------------------------------------------------------------------------

void RamalingamReps::initialize() {
    const std::size_t n = graph_.num_nodes();
    spt_ = ShortestPathTree(n);
    spt_.set_source(source_);

    // Min-heap: (tentative_dist, node)
    using DistNode = std::pair<double, NodeId>;
    std::priority_queue<DistNode, std::vector<DistNode>, std::greater<DistNode>> pq;
    pq.push({0.0, source_});

    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();

        // Skip if we already found a shorter path to u
        if (d > spt_.dist(u)) continue;

        for (const auto& edge : graph_.get_outgoing_edges(u)) {
            double w = edge.effective_weight();
            if (std::isinf(w)) continue;

            NodeId v      = edge.to;
            double new_d  = spt_.dist(u) + w;
            if (new_d < spt_.dist(v)) {
                spt_.set_dist_and_parent(v, new_d, u);
                pq.push({new_d, v});
            }
        }
    }
}

// ---------------------------------------------------------------------------
// on_edge_increase() — Ramalingam-Reps incremental SPT repair for cost increase
// ---------------------------------------------------------------------------

void RamalingamReps::on_edge_increase(NodeId u, NodeId v,
                                       double /*old_w*/, double /*new_w*/) {
    nodes_invalidated_last_ = 0;
    nodes_relaxed_last_     = 0;

    // ---------------------------------------------------------------
    // Fast-path: if (u,v) is NOT on the SPT, nothing changes.
    // The current parent of v must be u for this edge to matter.
    // ---------------------------------------------------------------
    if (spt_.parent(v) != u) return;

    // ---------------------------------------------------------------
    // Step 1: Collect the SPT subtree rooted at v via BFS over children.
    //         These are the vertices that lose their shortest path via (u,v).
    // ---------------------------------------------------------------
    std::vector<NodeId> subtree = spt_.collect_subtree(v);
    nodes_invalidated_last_ = subtree.size();

    // Build a fast membership set for O(1) subtree lookups
    std::unordered_set<NodeId> in_subtree(subtree.begin(), subtree.end());

    // ---------------------------------------------------------------
    // Step 2: Invalidate the entire subtree (set dist=INF, parent=INVALID)
    //         spt_.invalidate_subtree also clears child links within S.
    // ---------------------------------------------------------------
    spt_.invalidate_subtree(v);

    // ---------------------------------------------------------------
    // Step 3: Boundary-seed priority queue.
    //         For every vertex s in the subtree, examine all incoming
    //         edges (p → s) where p is NOT in the subtree.
    //         If dist[p] < INF (p is in the surviving SPT):
    //           push (dist[p] + w(p,s), s, p) into the priority queue.
    //
    //         This is the "frontier" through which the disconnected
    //         subtree may reconnect to the surviving tree.
    // ---------------------------------------------------------------
    // Priority queue entry: (new_tentative_dist, vertex, predecessor)
    using Entry = std::tuple<double, NodeId, NodeId>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;

    for (NodeId s : subtree) {
        for (const auto& edge : graph_.get_incoming_edges(s)) {
            NodeId p = edge.from;
            double w = edge.effective_weight();
            if (std::isinf(w)) continue;
            if (in_subtree.count(p)) continue;    // p is also disconnected
            if (!spt_.in_tree(p))   continue;     // p has no finite dist

            pq.push({spt_.dist(p) + w, s, p});
        }
    }

    // ---------------------------------------------------------------
    // Step 4: Dijkstra-style relaxation, restricted to the invalidated
    //         subtree vertices.
    //         We only update vertices whose dist is still INF (i.e.,
    //         still in the disconnected subtree).
    // ---------------------------------------------------------------
    while (!pq.empty()) {
        auto [cost, s, pred] = pq.top(); pq.pop();

        // Skip if s was already reconnected via a cheaper path
        if (spt_.in_tree(s)) continue;

        // Reconnect s into the SPT
        spt_.set_dist_and_parent(s, cost, pred);
        ++nodes_relaxed_last_;

        // Propagate to unresolved subtree neighbours of s
        for (const auto& edge : graph_.get_outgoing_edges(s)) {
            NodeId t = edge.to;
            double w = edge.effective_weight();
            if (std::isinf(w)) continue;
            if (!in_subtree.count(t)) continue;  // t not in subtree → skip
            if (spt_.in_tree(t))      continue;  // already reconnected

            pq.push({cost + w, t, s});
        }
    }
    // Vertices remaining in the subtree with dist==INF are truly unreachable.
}

// ---------------------------------------------------------------------------
// on_edge_decrease() — incremental SPT repair for cost decrease / new road
// ---------------------------------------------------------------------------

void RamalingamReps::on_edge_decrease(NodeId u, NodeId v,
                                       double /*old_w*/, double new_w) {
    nodes_invalidated_last_ = 0;
    nodes_relaxed_last_     = 0;

    // If this edge offers a genuine improvement to dist[v], propagate it.
    double new_dist_v = spt_.dist(u) + new_w;
    if (new_dist_v >= spt_.dist(v)) return;  // No improvement needed

    // Seed a forward Dijkstra relaxation from v
    using DistNode = std::pair<double, NodeId>;
    std::priority_queue<DistNode, std::vector<DistNode>, std::greater<DistNode>> pq;

    spt_.set_dist_and_parent(v, new_dist_v, u);
    pq.push({new_dist_v, v});
    ++nodes_relaxed_last_;

    while (!pq.empty()) {
        auto [d, curr] = pq.top(); pq.pop();

        // Stale entry guard
        if (d > spt_.dist(curr)) continue;

        for (const auto& edge : graph_.get_outgoing_edges(curr)) {
            double w = edge.effective_weight();
            if (std::isinf(w)) continue;

            NodeId next = edge.to;
            double candidate = spt_.dist(curr) + w;
            if (candidate < spt_.dist(next)) {
                spt_.set_dist_and_parent(next, candidate, curr);
                pq.push({candidate, next});
                ++nodes_relaxed_last_;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Query interface
// ---------------------------------------------------------------------------

double RamalingamReps::get_distance(NodeId u) const {
    return spt_.dist(u);
}

std::vector<NodeId> RamalingamReps::get_path(NodeId target) const {
    std::vector<NodeId> path;
    const std::size_t n = graph_.num_nodes();
    if (target >= n || !spt_.in_tree(target)) return path;

    for (NodeId curr = target; curr != INVALID_NODE; curr = spt_.parent(curr)) {
        path.push_back(curr);
        if (curr == source_) break;
        if (path.size() > n) { path.clear(); break; }  // cycle guard
    }
    std::reverse(path.begin(), path.end());
    return path;
}

} // namespace dynamic_route
