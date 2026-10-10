#pragma once

// =============================================================================
// DynamicRoute — Advanced Traffic Simulation: TrafficScenario
// =============================================================================
// Provides three realistic traffic perturbation scenario types:
//
//   1. CongestionWave   — A blockage propagates step-by-step along a
//                         pre-defined ordered road corridor (e.g. motorway
//                         accident backing up traffic towards the city).
//
//   2. DisasterZone     — A rectangular geographic region simultaneously
//                         degrades ALL contained edges by a congestion
//                         multiplier (e.g. flooding, road works zone, event).
//                         Optionally restores all edges after a TTL.
//
//   3. RecoveryEvent    — Tracks previously perturbed edges and restores them
//                         to their original weights after a configurable TTL
//                         (time-to-live in perturbation steps).
//
// All scenario types operate on a mutable Graph reference and record the
// exact PerturbationEvents they generate, so they can be replayed via the
// ScenarioRunner or fed directly into incremental algorithms.
// =============================================================================

#include "graph/graph.hpp"
#include "simulation/event_simulator.hpp"

#include <vector>
#include <deque>
#include <string>
#include <random>
#include <cstdint>

namespace dynamic_route {

// ---------------------------------------------------------------------------
// CongestionWave
// ---------------------------------------------------------------------------

/**
 * @brief A congestion wave that propagates along an ordered road corridor.
 *
 * The corridor is a sequence of node IDs [n0, n1, n2, ..., nk].
 * At each step(), the next edge (ni → ni+1) is blocked (weight set to INF),
 * simulating a backup from an accident that closes roads progressively.
 *
 * Example: motorway M25 with 6 junctions — each call to step() closes the
 * next segment, triggering route repair in affected algorithms.
 */
class CongestionWave {
public:
    /**
     * @param graph      The road network (mutated by step()).
     * @param corridor   Ordered sequence of NodeIds forming the congested road.
     * @param congestion_factor  Weight multiplier for congested (but not yet
     *                           fully blocked) edges. Default 5.0 (×5 slowdown).
     *                           Set to INF_WEIGHT to fully block each segment.
     */
    CongestionWave(Graph&                    graph,
                   std::vector<NodeId>       corridor,
                   double                    congestion_factor = INF_WEIGHT);

    /**
     * @brief Advance the wave by one segment.
     *
     * Blocks or congests the next corridor edge and returns the event.
     * Returns a zeroed event with from==to if the wave has fully propagated.
     *
     * @returns PerturbationEvent describing the change.
     */
    PerturbationEvent step();

    /// True when all corridor segments have been applied.
    bool finished() const noexcept;

    /// Number of remaining steps.
    std::size_t remaining() const noexcept;

    /// Undo all applied steps (restore original weights).
    void reset();

    /// The corridor this wave operates on.
    const std::vector<NodeId>& corridor() const noexcept { return corridor_; }

private:
    Graph&              graph_;
    std::vector<NodeId> corridor_;
    double              congestion_factor_;
    std::size_t         current_step_{0};

    // Saved original weights for reset()
    std::vector<double> original_weights_;
};

// ---------------------------------------------------------------------------
// DisasterZone
// ---------------------------------------------------------------------------

/**
 * @brief A geographic disaster zone that degrades all edges within a
 *        bounding rectangle simultaneously.
 *
 * The bounding box is defined in the same coordinate space as node positions
 * (x_min, y_min, x_max, y_max).  All directed edges whose BOTH endpoints lie
 * within the zone are degraded.
 *
 * Typical scenarios:
 *   - Flash flood: multiply weights by 8×, or block entirely.
 *   - Road works zone: multiply by 3× for the duration.
 *   - Major event (stadium): multiply by 2× in a radius around the venue.
 *
 * After apply(), the zone can be cleared with clear() which restores weights.
 */
class DisasterZone {
public:
    /**
     * @param graph            The road network.
     * @param x_min, y_min     Bottom-left of the bounding box.
     * @param x_max, y_max     Top-right of the bounding box.
     * @param multiplier       Weight multiplier (use INF_WEIGHT to block all).
     * @param name             Human-readable label (e.g. "Flood Zone A").
     */
    DisasterZone(Graph&       graph,
                 double       x_min, double y_min,
                 double       x_max, double y_max,
                 double       multiplier,
                 std::string  name = "DisasterZone");

    /**
     * @brief Apply the disaster — degrade all edges within the zone.
     *
     * @returns List of PerturbationEvents (one per affected edge).
     */
    std::vector<PerturbationEvent> apply();

    /**
     * @brief Clear the disaster — restore all originally affected edges.
     *
     * @returns List of recovery PerturbationEvents.
     */
    std::vector<PerturbationEvent> clear();

    /// Whether the zone has been applied (and not yet cleared).
    bool is_active() const noexcept { return active_; }

    /// Name label for logging.
    const std::string& name() const noexcept { return name_; }

    /// Number of edges affected by the last apply().
    std::size_t affected_edge_count() const noexcept { return saved_weights_.size(); }

private:
    struct SavedEdge {
        NodeId from;
        NodeId to;
        double original_weight;
    };

    Graph&      graph_;
    double      x_min_, y_min_, x_max_, y_max_;
    double      multiplier_;
    std::string name_;
    bool        active_{false};

    std::vector<SavedEdge> saved_weights_;

    bool node_in_zone(NodeId u) const;
};

// ---------------------------------------------------------------------------
// RecoveryEvent
// ---------------------------------------------------------------------------

/**
 * @brief Tracks perturbed edges and restores them after a TTL.
 *
 * Each time record_perturbation() is called with a PerturbationEvent, the
 * recovery engine stores the original weight.  When tick() is called,
 * the TTL counts down.  When it reaches zero, all tracked edges are
 * restored to their original weights and the corresponding recovery events
 * are returned.
 *
 * Usage pattern:
 *   RecoveryEvent recovery(graph, ttl_steps=10);
 *   // After each perturbation:
 *   recovery.record_perturbation(event);
 *   // After each time step:
 *   auto restored = recovery.tick(graph);
 *   // Feed restored events back into algorithms.
 */
class RecoveryEvent {
public:
    /**
     * @param graph     The road network.
     * @param ttl       Number of perturbation steps before automatic recovery.
     */
    explicit RecoveryEvent(Graph& graph, int ttl = 10);

    /// Record a perturbation so it will be recovered after TTL steps.
    void record_perturbation(const PerturbationEvent& event);

    /**
     * @brief Advance the TTL counter by one step.
     *
     * Edges whose TTL has expired are restored to their original weights.
     *
     * @returns List of recovery PerturbationEvents (empty if nothing recovered).
     */
    std::vector<PerturbationEvent> tick();

    /// Number of pending recoveries.
    std::size_t pending_count() const noexcept { return pending_.size(); }

    /// Clear all pending recoveries without restoring.
    void clear_pending() { pending_.clear(); current_tick_ = 0; }

private:
    struct PendingRecovery {
        NodeId from;
        NodeId to;
        double original_weight;
        int    recover_at_tick;   ///< restore when current_tick_ == this value
        bool   was_blocked;
    };

    Graph& graph_;
    int    ttl_;
    int    current_tick_{0};

    std::deque<PendingRecovery> pending_;
};

// ---------------------------------------------------------------------------
// CongestionWaveGenerator  (random corridor builder)
// ---------------------------------------------------------------------------

/**
 * @brief Utility to auto-generate a random corridor from the graph for
 *        a CongestionWave scenario.
 *
 * Performs a random walk of `length` steps from a random starting node,
 * following existing edges.  Returns an empty vector if no corridor of
 * the requested length can be found.
 */
std::vector<NodeId> generate_random_corridor(const Graph& graph,
                                              std::size_t  length,
                                              std::uint32_t seed = 42);

} // namespace dynamic_route
