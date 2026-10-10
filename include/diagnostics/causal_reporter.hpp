#pragma once

// =============================================================================
// DynamicRoute — Diagnostics: CausalReporter
// =============================================================================
// A thin instrumentation wrapper around any incremental shortest-path
// algorithm.  It intercepts every on_edge_changed() call to:
//   1. Time the update with nanosecond precision.
//   2. Capture diagnostic counters (nodes_relaxed, nodes_invalidated for RR).
//   3. Run compute_shortest_path() and record the result.
//   4. Optionally compare against Dijkstra ground truth.
//   5. Append a complete AuditEntry to the attached AuditLogger.
//
// Design: zero-overhead when ENABLE_AUDIT is false (compile-time flag).
//
// Usage example:
//   AuditLogger logger;
//   CausalReporter<LifelongPlanningAStar> rep(lpa, graph, src, tgt, logger, "LPA*");
//   rep.on_edge_changed(u, v, new_w);   // logs automatically
//   auto result = rep.compute_shortest_path();
// =============================================================================

#include "diagnostics/audit_log.hpp"
#include "algorithms/dijkstra.hpp"
#include "graph/graph.hpp"
#include "graph/path_validator.hpp"

#include <string>
#include <cmath>

namespace dynamic_route {

// ---------------------------------------------------------------------------
// DiagnosticCounters — extracted from algorithm objects via ADL helpers
// ---------------------------------------------------------------------------

// Default: algorithm has no invalidation/relaxation counters (LPA*, D*Lite)
template <typename AlgoT>
struct AlgoCounters {
    static std::size_t invalidated(const AlgoT&) { return 0; }
    static std::size_t relaxed    (const AlgoT&) { return 0; }
    static std::size_t expanded   (const AlgoT&) { return 0; }
};

// Specialisation for RamalingamReps (has nodes_invalidated_last / nodes_relaxed_last)
// Forward-declare to avoid circular include; actual specialisation is in the .cpp
// that uses CausalReporter<RamalingamReps>.  We use a SFINAE helper instead:

// SFINAE: detect nodes_invalidated_last()
template <typename T, typename = void>
struct has_invalidated_counter : std::false_type {};

template <typename T>
struct has_invalidated_counter<T,
    std::void_t<decltype(std::declval<const T&>().nodes_invalidated_last())>>
    : std::true_type {};

// SFINAE: detect nodes_relaxed_last()
template <typename T, typename = void>
struct has_relaxed_counter : std::false_type {};

template <typename T>
struct has_relaxed_counter<T,
    std::void_t<decltype(std::declval<const T&>().nodes_relaxed_last())>>
    : std::true_type {};

// ---------------------------------------------------------------------------
// CausalReporter<AlgoT>
// ---------------------------------------------------------------------------

/**
 * @brief Instrumentation wrapper for any incremental shortest-path algorithm.
 *
 * @tparam AlgoT  Must expose:
 *   - void on_edge_changed(NodeId from, NodeId to, double new_weight)
 *   - SearchResult compute_shortest_path()
 *   Optionally:
 *   - std::size_t nodes_invalidated_last() const
 *   - std::size_t nodes_relaxed_last() const
 */
template <typename AlgoT>
class CausalReporter {
public:
    /**
     * @param algo       The wrapped algorithm (must outlive this object).
     * @param graph      Graph reference for Dijkstra ground-truth checks.
     * @param source     Source node.
     * @param target     Target node.
     * @param logger     AuditLogger to append entries to.
     * @param algo_name  Human-readable algorithm label (e.g. "LPA*").
     * @param check_dijkstra  If true, validate every result against Dijkstra.
     */
    CausalReporter(AlgoT&        algo,
                   const Graph&  graph,
                   NodeId        source,
                   NodeId        target,
                   AuditLogger&  logger,
                   std::string   algo_name,
                   bool          check_dijkstra = true)
        : algo_(algo),
          graph_(graph),
          source_(source),
          target_(target),
          logger_(logger),
          algo_name_(std::move(algo_name)),
          check_dijkstra_(check_dijkstra)
    {}

    // ------------------------------------------------------------------
    // on_edge_changed  (times the update, dispatches to the real algo)
    // ------------------------------------------------------------------

    void on_edge_changed(NodeId from, NodeId to, double new_weight) {
        // Get old weight before the change is applied
        auto old_w_opt = graph_.get_edge_weight(from, to);
        double old_w = old_w_opt.value_or(INF_WEIGHT);

        // Build partial audit entry
        pending_.step            = step_counter_++;
        pending_.algorithm       = algo_name_;
        pending_.edge_change.from        = from;
        pending_.edge_change.to          = to;
        pending_.edge_change.old_weight  = old_w;
        pending_.edge_change.new_weight  = new_weight;
        pending_.edge_change.is_blockage = std::isinf(new_weight);

        // Time the update
        {
            ScopedTimer timer(pending_.repair_time_us);
            algo_.on_edge_changed(from, to, new_weight);
        }

        // Harvest diagnostic counters (SFINAE dispatch)
        pending_.nodes_invalidated = get_invalidated(algo_);
        pending_.nodes_relaxed     = get_relaxed(algo_);
    }

    // ------------------------------------------------------------------
    // compute_shortest_path  (runs query, records result, logs entry)
    // ------------------------------------------------------------------

    SearchResult compute_shortest_path() {
        SearchResult res = algo_.compute_shortest_path();

        pending_.path_found    = res.path_found;
        pending_.final_cost    = res.total_cost;
        pending_.path          = res.path;
        pending_.nodes_expanded = res.nodes_visited;

        // Optionally validate against Dijkstra
        if (check_dijkstra_) {
            auto vr = PathValidator::compare_with_dijkstra(
                          graph_, source_, target_, res);
            pending_.correct    = vr.ok;
            pending_.cost_delta = vr.cost_delta;
        } else {
            pending_.correct    = true;
            pending_.cost_delta = 0.0;
        }

        logger_.record(pending_);
        pending_ = AuditEntry{};  // reset for next step
        return res;
    }

private:
    AlgoT&       algo_;
    const Graph& graph_;
    NodeId       source_;
    NodeId       target_;
    AuditLogger& logger_;
    std::string  algo_name_;
    bool         check_dijkstra_;
    int          step_counter_{0};
    AuditEntry   pending_;

    // -- SFINAE helpers for diagnostic counters --

    template <typename T>
    static std::size_t get_invalidated(const T& a) {
        if constexpr (has_invalidated_counter<T>::value)
            return a.nodes_invalidated_last();
        else
            return 0;
    }

    template <typename T>
    static std::size_t get_relaxed(const T& a) {
        if constexpr (has_relaxed_counter<T>::value)
            return a.nodes_relaxed_last();
        else
            return 0;
    }
};

// ---------------------------------------------------------------------------
// RRCausalReporter — variant for RamalingamReps (different update API)
// ---------------------------------------------------------------------------
// RR exposes on_edge_increase / on_edge_decrease instead of on_edge_changed,
// and its "query" is get_distance(target) not compute_shortest_path().
// This wrapper adapts it to the same AuditLogger interface.

class RamalingamReps;  // forward declaration

class RRCausalReporter {
public:
    RRCausalReporter(RamalingamReps& rr,
                     Graph&          graph,
                     NodeId          source,
                     NodeId          target,
                     AuditLogger&    logger,
                     bool            check_dijkstra = true);

    /**
     * @brief Call after updating the graph and calling rr.on_edge_increase/decrease.
     *        Records timing, counters, and result.
     *
     * @param from       Edge tail
     * @param to         Edge head
     * @param old_weight Weight before change
     * @param new_weight Weight after change
     */
    void record_step(NodeId from, NodeId to,
                     double old_weight, double new_weight);

private:
    RamalingamReps& rr_;
    Graph&          graph_;
    NodeId          source_;
    NodeId          target_;
    AuditLogger&    logger_;
    bool            check_dijkstra_;
    int             step_counter_{0};
};

} // namespace dynamic_route
