#pragma once

#include "graph/graph.hpp"
#include "graph/shortest_path_tree.hpp"
#include "algorithms/dijkstra.hpp"

#include <vector>
#include <queue>
#include <set>
#include <unordered_set>

namespace dynamic_route {

/**
 * @brief Ramalingam–Reps (RR) Incremental Single-Source Shortest Path
 *
 * =====================================================================
 * ALGORITHM OVERVIEW (Phase 1 — Real Implementation)
 * =====================================================================
 *
 * Original paper:
 *   Ramalingam, G. & Reps, T. (1996).
 *   "An Incremental Algorithm for a Generalization of the Shortest-Path
 *   Problem." ACM TOPLAS 18(6):1020–1046.
 *
 * The key insight is that after a graph change, only a subset of vertices
 * δ ⊆ V actually require a distance update. The RR algorithm restricts
 * all computation to δ and its immediate frontier.
 *
 * -------------------------------------------------------------------
 * DATA STRUCTURES maintained by this class:
 * -------------------------------------------------------------------
 *   spt_       : ShortestPathTree — stores dist[], parent[], children[]
 *   source_    : fixed root of the SPT
 *
 * -------------------------------------------------------------------
 * EDGE WEIGHT INCREASE / BLOCKAGE  (on_edge_increase)
 * -------------------------------------------------------------------
 *   Case 1 — Edge (u,v) is NOT on the SPT (parent[v] ≠ u):
 *     The edge is not used by any shortest path → no SPT change needed.
 *
 *   Case 2 — Edge (u,v) IS on the SPT (parent[v] == u):
 *     Step 1: Collect subtree S rooted at v using BFS over SPT children.
 *     Step 2: Invalidate S — set dist[s] = ∞ and parent[s] = INVALID ∀s ∈ S.
 *     Step 3: Build boundary priority queue Q with all edges from
 *             outside S into S that are not blocked:
 *             For each s ∈ S, for each incoming edge (p, s) where p ∉ S:
 *               push (dist[p] + w(p,s), s, p) into Q.
 *     Step 4: Dijkstra-style relaxation restricted to S:
 *             While Q not empty:
 *               Pop (cost, v, pred).
 *               If dist[v] ≤ cost: skip (already relaxed via cheaper path).
 *               Set dist[v] = cost, parent[v] = pred.
 *               For each outgoing edge (v, w) where w ∈ S (i.e., dist[w] = ∞):
 *                 Push (dist[v] + w(v,w), w, v) into Q.
 *
 *   Complexity: O(|S| + |E(boundary)| log |S|)
 *               When change is local: |S| ≪ |V|
 *
 * -------------------------------------------------------------------
 * EDGE WEIGHT DECREASE / NEW ROAD  (on_edge_decrease)
 * -------------------------------------------------------------------
 *   If dist[u] + new_weight < dist[v]:
 *     This is a potential improvement. Run a forward Dijkstra relaxation
 *     seeded at v with dist[v] = dist[u] + new_weight, propagating
 *     only to vertices that actually improve. This is bounded by the
 *     overconsistent subtree size.
 *
 * =====================================================================
 */

class RamalingamReps {
public:
    /**
     * @param graph   The road network (passed by reference; must outlive this object).
     * @param source  Fixed source node for the Single-Source SPT.
     */
    RamalingamReps(const Graph& graph, NodeId source);

    /**
     * @brief Build the initial SPT via a full Dijkstra sweep.
     *        Must be called once before any on_edge_* updates.
     */
    void initialize();

    /**
     * @brief Update the SPT when edge (u→v) cost INCREASES (or is blocked).
     *
     * @param u      Edge tail
     * @param v      Edge head
     * @param old_w  Previous effective weight (before the change)
     * @param new_w  New effective weight (INF_WEIGHT if blocked)
     *
     * This is the core RR incremental algorithm for cost increases:
     *   1. If (u,v) not on SPT → no-op (fast path).
     *   2. Collect + invalidate the SPT subtree rooted at v.
     *   3. Boundary-seed priority queue with non-subtree edges into subtree.
     *   4. Dijkstra-style relaxation restricted to the invalidated subtree.
     *
     * Metrics tracking:
     *   nodes_invalidated_last_  : how many vertices were cut from the SPT
     *   nodes_relaxed_last_      : how many vertices were re-relaxed
     */
    void on_edge_increase(NodeId u, NodeId v, double old_w, double new_w);

    /**
     * @brief Update the SPT when edge (u→v) cost DECREASES (new road / improved).
     *
     * @param u      Edge tail
     * @param v      Edge head
     * @param old_w  Previous effective weight
     * @param new_w  New effective weight (must be < old_w)
     *
     * Uses forward relaxation seeded at v, propagating improvements
     * through overconsistent subtree. Complexity O(|overconsistent| log n).
     */
    void on_edge_decrease(NodeId u, NodeId v, double old_w, double new_w);

    // ------------------------------------------------------------------
    // Query interface
    // ------------------------------------------------------------------

    /// Shortest distance from source to u (-1 / INF if unreachable).
    double get_distance(NodeId u) const;

    /// Reconstruct path from source → target using the SPT parent array.
    std::vector<NodeId> get_path(NodeId target) const;

    /// Read-only access to the maintained SPT.
    const ShortestPathTree& spt() const noexcept { return spt_; }

    // ------------------------------------------------------------------
    // Diagnostic / benchmark counters (set by last update call)
    // ------------------------------------------------------------------

    /// Number of SPT vertices invalidated by the last on_edge_increase call.
    std::size_t nodes_invalidated_last() const noexcept { return nodes_invalidated_last_; }

    /// Number of vertices re-relaxed during the last SPT repair.
    std::size_t nodes_relaxed_last()     const noexcept { return nodes_relaxed_last_; }

private:
    const Graph&      graph_;
    NodeId            source_;
    ShortestPathTree  spt_;

    std::size_t nodes_invalidated_last_{0};
    std::size_t nodes_relaxed_last_{0};
};

} // namespace dynamic_route
