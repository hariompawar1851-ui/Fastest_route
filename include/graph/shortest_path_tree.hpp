#pragma once

#include "graph/node.hpp"
#include "graph/edge.hpp"

#include <vector>
#include <queue>
#include <stack>
#include <functional>
#include <algorithm>
#include <limits>

namespace dynamic_route {

/**
 * @brief Shortest Path Tree (SPT) Data Structure
 *
 * Maintains an explicit rooted tree over the graph where each vertex u
 * stores:
 *   - dist[u]   : tentative shortest distance from the source
 *   - parent[u] : predecessor in the SPT (NodeId == INVALID_NODE if root)
 *   - children[u]: forward adjacency list of the directed SPT
 *
 * This structure is the core of the Ramalingam-Reps incremental SSSP:
 * when an edge on the SPT is modified, only the subtree rooted at the
 * affected vertex is invalidated and searched for reconnection edges.
 *
 * Phase 1 Contribution:
 *   Previously, the RR implementation fell back to full Dijkstra on every
 *   update. Phase 1 replaces that with a proper subtree-aware BFS scan
 *   bounded to O(|δ| + |neighbors(δ)| log |neighbors(δ)|).
 */

/// Sentinel value for "no parent" (source node or disconnected)
static constexpr NodeId INVALID_NODE = std::numeric_limits<NodeId>::max();

class ShortestPathTree {
public:
    /// Construct an empty SPT for a graph with `n` vertices.
    explicit ShortestPathTree(std::size_t n);

    // ------------------------------------------------------------------
    // Access
    // ------------------------------------------------------------------

    /// Distance estimate from source to u. Returns INF if disconnected.
    double dist(NodeId u) const noexcept;

    /// SPT parent of u. Returns INVALID_NODE for the source or if not reached.
    NodeId parent(NodeId u) const noexcept;

    /// Direct SPT children of u.
    const std::vector<NodeId>& children(NodeId u) const noexcept;

    std::size_t size() const noexcept { return dist_.size(); }

    // ------------------------------------------------------------------
    // Modification
    // ------------------------------------------------------------------

    /// Set source (root) and reset the tree.
    void set_source(NodeId src);

    /// Set distance + parent for vertex u, and wire up the SPT child link.
    void set_dist_and_parent(NodeId u, double d, NodeId par);

    /// Remove vertex u from the SPT (set dist = INF, parent = INVALID).
    void invalidate(NodeId u);

    /// Fully reset all vertices to INF / INVALID.
    void reset();

    // ------------------------------------------------------------------
    // Subtree operations (Ramalingam-Reps Phase 1)
    // ------------------------------------------------------------------

    /**
     * @brief Collect all vertices in the SPT subtree rooted at `root`
     *        using BFS over the children adjacency list.
     *
     * These are the vertices whose dist/parent may need to change when
     * the edge leading into `root` is modified (increased or blocked).
     *
     * Complexity: O(|subtree|)
     */
    std::vector<NodeId> collect_subtree(NodeId root) const;

    /**
     * @brief Invalidate (disconnect) the entire SPT subtree rooted at `root`.
     *
     * Sets dist[v] = INF and parent[v] = INVALID for every v in the subtree,
     * and clears the children links of all those vertices.
     *
     * Complexity: O(|subtree|)
     */
    void invalidate_subtree(NodeId root);

    /**
     * @brief Check whether vertex u is currently in the SPT
     *        (i.e., dist[u] < INF).
     */
    bool in_tree(NodeId u) const noexcept;

    // ------------------------------------------------------------------
    // Internal helpers
    // ------------------------------------------------------------------

private:
    void remove_child_link(NodeId par, NodeId child);

    std::vector<double>             dist_;
    std::vector<NodeId>             parent_;
    std::vector<std::vector<NodeId>> children_;
    NodeId                          source_{INVALID_NODE};
};

// -------------------------------------------------------------------------
// Inline implementations
// -------------------------------------------------------------------------

inline ShortestPathTree::ShortestPathTree(std::size_t n)
    : dist_(n, INF_WEIGHT), parent_(n, INVALID_NODE), children_(n) {}

inline double ShortestPathTree::dist(NodeId u) const noexcept {
    return (u < dist_.size()) ? dist_[u] : INF_WEIGHT;
}

inline NodeId ShortestPathTree::parent(NodeId u) const noexcept {
    return (u < parent_.size()) ? parent_[u] : INVALID_NODE;
}

inline const std::vector<NodeId>& ShortestPathTree::children(NodeId u) const noexcept {
    static const std::vector<NodeId> empty;
    return (u < children_.size()) ? children_[u] : empty;
}

inline void ShortestPathTree::set_source(NodeId src) {
    source_ = src;
    reset();
    if (src < dist_.size()) dist_[src] = 0.0;
}

inline void ShortestPathTree::reset() {
    std::fill(dist_.begin(), dist_.end(), INF_WEIGHT);
    std::fill(parent_.begin(), parent_.end(), INVALID_NODE);
    for (auto& c : children_) c.clear();
    if (source_ != INVALID_NODE && source_ < dist_.size())
        dist_[source_] = 0.0;
}

inline void ShortestPathTree::set_dist_and_parent(NodeId u, double d, NodeId par) {
    // Remove old child link if parent changed
    if (parent_[u] != INVALID_NODE && parent_[u] != par) {
        remove_child_link(parent_[u], u);
    }
    dist_[u]   = d;
    parent_[u] = par;
    if (par != INVALID_NODE) {
        auto& ch = children_[par];
        if (std::find(ch.begin(), ch.end(), u) == ch.end()) {
            ch.push_back(u);
        }
    }
}

inline void ShortestPathTree::invalidate(NodeId u) {
    if (u >= dist_.size()) return;
    if (parent_[u] != INVALID_NODE) {
        remove_child_link(parent_[u], u);
    }
    dist_[u]   = INF_WEIGHT;
    parent_[u] = INVALID_NODE;
    children_[u].clear();
}

inline bool ShortestPathTree::in_tree(NodeId u) const noexcept {
    return u < dist_.size() && dist_[u] < INF_WEIGHT;
}

inline std::vector<NodeId> ShortestPathTree::collect_subtree(NodeId root) const {
    std::vector<NodeId> subtree;
    if (root >= children_.size()) return subtree;

    std::queue<NodeId> bfs;
    bfs.push(root);
    while (!bfs.empty()) {
        NodeId v = bfs.front(); bfs.pop();
        subtree.push_back(v);
        for (NodeId c : children_[v]) {
            bfs.push(c);
        }
    }
    return subtree;
}

inline void ShortestPathTree::invalidate_subtree(NodeId root) {
    if (root >= dist_.size()) return;

    // Disconnect root from its SPT parent first
    if (parent_[root] != INVALID_NODE) {
        remove_child_link(parent_[root], root);
        parent_[root] = INVALID_NODE;
    }

    // BFS over children to clear the full subtree
    std::queue<NodeId> bfs;
    bfs.push(root);
    while (!bfs.empty()) {
        NodeId v = bfs.front(); bfs.pop();
        dist_[v]   = INF_WEIGHT;
        parent_[v] = INVALID_NODE;
        for (NodeId c : children_[v]) {
            bfs.push(c);
        }
        children_[v].clear();
    }
}

inline void ShortestPathTree::remove_child_link(NodeId par, NodeId child) {
    if (par >= children_.size()) return;
    auto& ch = children_[par];
    ch.erase(std::remove(ch.begin(), ch.end(), child), ch.end());
}

} // namespace dynamic_route
