#include "diagnostics/causal_reporter.hpp"
#include "algorithms/ramalingam_reps.hpp"
#include "algorithms/dijkstra.hpp"
#include "graph/path_validator.hpp"

#include <cmath>

namespace dynamic_route {

RRCausalReporter::RRCausalReporter(RamalingamReps& rr,
                                   Graph&          graph,
                                   NodeId          source,
                                   NodeId          target,
                                   AuditLogger&    logger,
                                   bool            check_dijkstra)
    : rr_(rr),
      graph_(graph),
      source_(source),
      target_(target),
      logger_(logger),
      check_dijkstra_(check_dijkstra)
{}

void RRCausalReporter::record_step(NodeId from, NodeId to,
                                    double old_weight, double new_weight)
{
    AuditEntry e;
    e.step      = step_counter_++;
    e.algorithm = "RR";

    e.edge_change.from        = from;
    e.edge_change.to          = to;
    e.edge_change.old_weight  = old_weight;
    e.edge_change.new_weight  = new_weight;
    e.edge_change.is_blockage = std::isinf(new_weight);

    // Time the incremental update
    {
        ScopedTimer timer(e.repair_time_us);
        if (new_weight >= old_weight) {
            rr_.on_edge_increase(from, to, old_weight, new_weight);
        } else {
            rr_.on_edge_decrease(from, to, old_weight, new_weight);
        }
    }

    // Harvest RR-specific diagnostic counters
    e.nodes_invalidated = rr_.nodes_invalidated_last();
    e.nodes_relaxed     = rr_.nodes_relaxed_last();
    e.nodes_expanded    = 0;  // RR does not count PQ pops separately

    // Query distance
    double dist = rr_.get_distance(target_);
    auto   path = rr_.get_path(target_);

    e.path_found = !std::isinf(dist);
    e.final_cost = dist;
    e.path       = path;

    // Validate against Dijkstra ground truth
    if (check_dijkstra_) {
        auto ref = Dijkstra::find_shortest_path(graph_, source_, target_);
        if (ref.path_found != e.path_found) {
            e.correct    = false;
            e.cost_delta = std::abs(dist - ref.total_cost);
        } else if (ref.path_found) {
            e.cost_delta = std::abs(dist - ref.total_cost);
            e.correct    = (e.cost_delta < 1e-6);
        } else {
            e.correct    = true;
            e.cost_delta = 0.0;
        }
    } else {
        e.correct    = true;
        e.cost_delta = 0.0;
    }

    logger_.record(std::move(e));
}

} // namespace dynamic_route
