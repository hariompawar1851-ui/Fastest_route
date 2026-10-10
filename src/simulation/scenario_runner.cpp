#include "simulation/scenario_runner.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <numeric>

namespace dynamic_route {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

ScenarioRunner::ScenarioRunner(Graph& graph, NodeId source, NodeId target)
    : graph_(graph), source_(source), target_(target)
{}

// ---------------------------------------------------------------------------
// Scenario registration
// ---------------------------------------------------------------------------

void ScenarioRunner::add_congestion_wave(CongestionWave& wave,
                                         int start_tick,
                                         int step_interval) {
    wave_entries_.push_back({&wave, start_tick, step_interval, -1});
}

void ScenarioRunner::add_disaster_zone(DisasterZone& zone,
                                        int apply_tick,
                                        int clear_tick) {
    zone_entries_.push_back({&zone, apply_tick, clear_tick});
}

void ScenarioRunner::add_recovery_tracker(RecoveryEvent& recovery) {
    recovery_trackers_.push_back(&recovery);
}

void ScenarioRunner::add_random_perturbations(int n_events,
                                               int start_tick, int end_tick,
                                               std::uint32_t seed) {
    if (n_events <= 0 || start_tick > end_tick) return;

    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> tick_dist(start_tick, end_tick);
    std::uniform_int_distribution<NodeId> node_dist(
        0, static_cast<NodeId>(graph_.num_nodes() - 1));

    for (int i = 0; i < n_events; ++i) {
        NodeId from = node_dist(rng);
        // Find a valid outgoing edge
        const auto& edges = graph_.get_outgoing_edges(from);
        if (edges.empty()) { --i; continue; }
        std::uniform_int_distribution<std::size_t> edge_pick(0, edges.size() - 1);
        NodeId to = edges[edge_pick(rng)].to;
        random_events_.push_back({tick_dist(rng), from, to});
    }

    // Sort by tick so they're applied in order
    std::sort(random_events_.begin(), random_events_.end(),
              [](const RandomEntry& a, const RandomEntry& b){
                  return a.tick < b.tick;
              });
}

// ---------------------------------------------------------------------------
// Algorithm registration
// ---------------------------------------------------------------------------

void ScenarioRunner::register_lpa(LifelongPlanningAStar& lpa,
                                   AuditLogger& log,
                                   const std::string& name) {
    AlgoEntry entry;
    entry.name = name;
    entry.log  = &log;
    entry.on_change = [&lpa](NodeId f, NodeId t, double w) {
        lpa.on_edge_changed(f, t, w);
    };
    entry.query = [&lpa]() -> SearchResult {
        return lpa.compute_shortest_path();
    };
    algos_.push_back(std::move(entry));
}

void ScenarioRunner::register_dstar(DStarLite& dstar,
                                     AuditLogger& log,
                                     const std::string& name) {
    AlgoEntry entry;
    entry.name = name;
    entry.log  = &log;
    entry.on_change = [&dstar](NodeId f, NodeId t, double w) {
        dstar.on_edge_changed(f, t, w);
    };
    entry.query = [&dstar]() -> SearchResult {
        return dstar.compute_shortest_path();
    };
    algos_.push_back(std::move(entry));
}

void ScenarioRunner::register_rr(RamalingamReps& rr,
                                  AuditLogger& log,
                                  const std::string& name) {
    AlgoEntry entry;
    entry.name = name;
    entry.log  = &log;
    // RR has on_edge_increase / decrease; we need old weight to dispatch
    entry.on_change = [&rr, &graph = graph_](NodeId f, NodeId t, double new_w) {
        auto old_opt = graph.get_edge_weight(f, t);
        double old_w = old_opt.value_or(INF_WEIGHT);
        if (new_w >= old_w)
            rr.on_edge_increase(f, t, old_w, new_w);
        else
            rr.on_edge_decrease(f, t, old_w, new_w);
    };
    // RR query: build a SearchResult from get_distance / get_path
    entry.query = [&rr, tgt = target_]() -> SearchResult {
        SearchResult res;
        double dist = rr.get_distance(tgt);
        res.path_found  = !std::isinf(dist);
        res.total_cost  = dist;
        res.path        = rr.get_path(tgt);
        res.nodes_visited = 0;
        return res;
    };
    algos_.push_back(std::move(entry));
}

// ---------------------------------------------------------------------------
// notify_and_record  — apply one edge change to all algorithms + log
// ---------------------------------------------------------------------------

void ScenarioRunner::notify_and_record(const PerturbationEvent& event, int /*tick*/) {
    for (auto& algo : algos_) {
        AuditEntry e;

        // Time the update
        double repair_us = 0.0;
        {
            ScopedTimer timer(repair_us);
            algo.on_change(event.from, event.to, event.new_weight);
        }

        // Query
        SearchResult res = algo.query();

        // Validate
        auto vr = PathValidator::compare_with_dijkstra(graph_, source_, target_, res);

        e.algorithm              = algo.name;
        e.edge_change.from       = event.from;
        e.edge_change.to         = event.to;
        e.edge_change.old_weight = event.old_weight;
        e.edge_change.new_weight = event.new_weight;
        e.edge_change.is_blockage = event.is_blockage;
        e.repair_time_us         = repair_us;
        e.path_found             = res.path_found;
        e.final_cost             = res.total_cost;
        e.path                   = res.path;
        e.nodes_expanded         = res.nodes_visited;
        e.correct                = vr.ok;
        e.cost_delta             = vr.cost_delta;

        // step counter is set from log size for simplicity
        e.step = static_cast<int>(algo.log->size());

        algo.log->record(std::move(e));
    }
}

// ---------------------------------------------------------------------------
// run
// ---------------------------------------------------------------------------

ScenarioRunnerReport ScenarioRunner::run(int total_ticks) {
    ScenarioRunnerReport report;
    report.total_ticks = total_ticks;

    // Random events iterator
    std::size_t rand_idx = 0;

    // We also need an EventSimulator for random weight changes when
    // random_events_ contains (from, to) pairs but no new weight yet.
    // We use a fixed rng to generate the new weights.
    std::mt19937 weight_rng(99999);
    std::uniform_real_distribution<double> factor_dist(1.5, 6.0);
    std::bernoulli_distribution block_prob(0.25);

    for (int tick = 0; tick < total_ticks; ++tick) {
        std::vector<PerturbationEvent> tick_events;

        // ---- 1. Congestion waves ----
        for (auto& we : wave_entries_) {
            if (tick < we.start_tick) continue;
            if (we.wave->finished()) continue;

            int steps_due = (tick - we.start_tick) / we.step_interval;
            int steps_done = we.last_stepped_tick < 0
                             ? 0
                             : (we.last_stepped_tick - we.start_tick) / we.step_interval + 1;

            if (steps_due > steps_done && !we.wave->finished()) {
                auto ev = we.wave->step();
                if (ev.from != ev.to || ev.from != 0) {  // valid event guard
                    tick_events.push_back(ev);
                    we.last_stepped_tick = tick;
                }
            }
        }

        // ---- 2. Disaster zones: apply ----
        for (auto& ze : zone_entries_) {
            if (tick == ze.apply_tick) {
                auto evs = ze.zone->apply();
                for (auto& e : evs) tick_events.push_back(e);
            }
            if (ze.clear_tick >= 0 && tick == ze.clear_tick) {
                auto evs = ze.zone->clear();
                for (auto& e : evs) tick_events.push_back(e);
            }
        }

        // ---- 3. Scheduled random perturbations ----
        while (rand_idx < random_events_.size() &&
               random_events_[rand_idx].tick == tick) {
            NodeId from = random_events_[rand_idx].from;
            NodeId to   = random_events_[rand_idx].to;
            ++rand_idx;

            auto cur_w_opt = graph_.get_edge_weight(from, to);
            if (!cur_w_opt.has_value()) continue;
            double cur_w = *cur_w_opt;

            double new_w;
            bool   is_block = block_prob(weight_rng);
            if (is_block) {
                new_w = INF_WEIGHT;
                graph_.set_edge_blocked(from, to, true);
            } else {
                new_w = cur_w * factor_dist(weight_rng);
                graph_.update_edge_weight(from, to, new_w);
            }

            tick_events.push_back(PerturbationEvent{from, to, cur_w, new_w, is_block});
        }

        // ---- 4. Notify all algorithms of every event this tick ----
        for (const auto& ev : tick_events) {
            notify_and_record(ev, tick);
        }
        report.total_events += static_cast<int>(tick_events.size());

        // ---- 5. Recovery ticks ----
        for (auto* rec : recovery_trackers_) {
            auto restored = rec->tick();
            for (const auto& ev : restored) {
                notify_and_record(ev, tick);
            }
            report.total_recoveries += static_cast<int>(restored.size());
        }
    }

    // ---- Build per-algorithm stats from audit logs ----
    for (const auto& algo : algos_) {
        auto stats = algo.log->compute_stats(graph_.num_nodes());
        ScenarioRunnerReport::AlgoStats as;
        as.name            = algo.name;
        as.steps_verified  = static_cast<int>(stats.correct_steps);
        as.steps_total     = static_cast<int>(stats.total_steps);
        as.avg_repair_us   = stats.avg_repair_us;
        as.max_repair_us   = stats.max_repair_us;
        report.algo_stats.push_back(as);
    }

    return report;
}

} // namespace dynamic_route
