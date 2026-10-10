#pragma once

// =============================================================================
// DynamicRoute — ScenarioRunner
// =============================================================================
// Orchestrates a multi-event traffic simulation timeline.
//
// The timeline is a sequence of timed scenario events that can be of any type
// (CongestionWave steps, DisasterZone apply/clear, random perturbations, and
// RecoveryEvent ticks).  After each event is applied to the graph, the runner
// notifies all registered incremental algorithms and records an AuditEntry.
//
// Typical usage:
//   ScenarioRunner runner(graph, source, target);
//   runner.add_congestion_wave(wave, start_tick=0, step_interval=1);
//   runner.add_disaster_zone(zone, apply_tick=5, clear_tick=15);
//   runner.add_recovery_tracker(ttl=8);
//   runner.register_lpa(lpa, logger, "LPA*");
//   runner.run(20 /*total ticks*/);
//   auto report = runner.report();
// =============================================================================

#include "graph/graph.hpp"
#include "simulation/traffic_scenario.hpp"
#include "diagnostics/audit_log.hpp"
#include "algorithms/dijkstra.hpp"
#include "algorithms/lpa_star.hpp"
#include "algorithms/d_star_lite.hpp"
#include "algorithms/ramalingam_reps.hpp"
#include "graph/path_validator.hpp"

#include <vector>
#include <functional>
#include <string>
#include <memory>

namespace dynamic_route {

// ---------------------------------------------------------------------------
// ScenarioRunnerReport — summary produced after run()
// ---------------------------------------------------------------------------

struct ScenarioRunnerReport {
    int    total_ticks{0};
    int    total_events{0};        ///< number of edge changes applied
    int    total_recoveries{0};    ///< number of TTL-triggered edge restores

    // Per-algorithm correctness
    struct AlgoStats {
        std::string name;
        int         steps_verified{0};  ///< incremental == Dijkstra
        int         steps_total{0};
        double      avg_repair_us{0.0};
        double      max_repair_us{0.0};
    };
    std::vector<AlgoStats> algo_stats;

    bool all_correct() const {
        for (const auto& a : algo_stats)
            if (a.steps_verified != a.steps_total) return false;
        return true;
    }
};

// ---------------------------------------------------------------------------
// ScenarioRunner
// ---------------------------------------------------------------------------

class ScenarioRunner {
public:
    /**
     * @param graph   Road network (mutated during run).
     * @param source  Source node for all algorithms and Dijkstra checks.
     * @param target  Target node.
     */
    ScenarioRunner(Graph& graph, NodeId source, NodeId target);

    // ------------------------------------------------------------------
    // Scenario registration
    // ------------------------------------------------------------------

    /**
     * @brief Schedule a CongestionWave to propagate starting at `start_tick`,
     *        advancing one corridor segment every `step_interval` ticks.
     */
    void add_congestion_wave(CongestionWave& wave,
                             int start_tick      = 0,
                             int step_interval   = 1);

    /**
     * @brief Schedule a DisasterZone: apply at `apply_tick`, clear at `clear_tick`.
     *        Set clear_tick = -1 to never clear automatically.
     */
    void add_disaster_zone(DisasterZone& zone,
                           int apply_tick,
                           int clear_tick = -1);

    /**
     * @brief Attach a RecoveryEvent tracker.
     *        Its tick() is called every simulation tick.
     */
    void add_recovery_tracker(RecoveryEvent& recovery);

    /**
     * @brief Add N random perturbation steps uniformly distributed across
     *        [start_tick, end_tick].
     */
    void add_random_perturbations(int n_events,
                                  int start_tick, int end_tick,
                                  std::uint32_t seed = 42);

    // ------------------------------------------------------------------
    // Algorithm registration
    // ------------------------------------------------------------------

    void register_lpa  (LifelongPlanningAStar& lpa,   AuditLogger& log, const std::string& name = "LPA*");
    void register_dstar(DStarLite&             dstar, AuditLogger& log, const std::string& name = "D*Lite");
    void register_rr   (RamalingamReps&        rr,    AuditLogger& log, const std::string& name = "RR");

    // ------------------------------------------------------------------
    // Execution
    // ------------------------------------------------------------------

    /**
     * @brief Run the simulation for `total_ticks` steps.
     *
     * At each tick:
     *   1. Apply all scheduled scenario events for this tick.
     *   2. Notify all registered algorithms of each edge change.
     *   3. Query each algorithm and record an AuditEntry.
     *   4. Tick all RecoveryEvent trackers; apply any restored edges.
     *
     * @returns A summary report.
     */
    ScenarioRunnerReport run(int total_ticks);

private:
    Graph&  graph_;
    NodeId  source_;
    NodeId  target_;

    // ---- Scenario schedule entries ----
    struct WaveEntry {
        CongestionWave* wave;
        int start_tick;
        int step_interval;
        int last_stepped_tick{-1};
    };

    struct ZoneEntry {
        DisasterZone* zone;
        int apply_tick;
        int clear_tick;
    };

    struct RandomEntry {
        int tick;
        NodeId from;
        NodeId to;
    };

    std::vector<WaveEntry>      wave_entries_;
    std::vector<ZoneEntry>      zone_entries_;
    std::vector<RecoveryEvent*> recovery_trackers_;
    std::vector<RandomEntry>    random_events_;

    // ---- Registered algorithm callbacks ----
    struct AlgoEntry {
        std::string   name;
        AuditLogger*  log;
        // Unified callbacks
        std::function<void(NodeId, NodeId, double)> on_change;
        std::function<SearchResult()>               query;
    };
    std::vector<AlgoEntry> algos_;

    // ---- Internal helpers ----
    void notify_and_record(const PerturbationEvent& event, int tick);
};

} // namespace dynamic_route
