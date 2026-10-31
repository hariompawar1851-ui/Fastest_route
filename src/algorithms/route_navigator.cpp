#include "algorithms/route_navigator.hpp"
#include "algorithms/dijkstra.hpp"

#include <cmath>

namespace dynamic_route {

RouteNavigator::RouteNavigator(Graph& graph, NodeId start, NodeId goal)
    : graph_(graph), start_(start), goal_(goal) {}

NavigationReport RouteNavigator::run(const EventProvider& provider,
                                     std::size_t max_steps,
                                     bool verify) {
    NavigationReport report;

    const std::size_t n = graph_.num_nodes();
    if (start_ >= n || goal_ >= n) {
        report.blocked = true;
        return report;
    }
    if (max_steps == 0) max_steps = 4 * n;

    auto cross_check = [&](NodeId position, const SearchResult& inc) {
        auto ref = Dijkstra::find_shortest_path(graph_, position, goal_);
        report.baseline_expansions += ref.nodes_visited;
        ++report.verified_steps;
        bool ok = (inc.path_found == ref.path_found);
        if (ok && inc.path_found) {
            ok = std::abs(inc.total_cost - ref.total_cost) < 1e-6;
        }
        if (!ok) ++report.mismatches;
    };

    DStarLite dstar(graph_, start_, goal_);
    SearchResult plan = dstar.compute_shortest_path();
    report.initial_expansions = plan.nodes_visited;
    report.trajectory.push_back(start_);
    if (verify) cross_check(start_, plan);

    if (!plan.path_found) {
        report.blocked = true;
        return report;
    }
    report.initial_cost = plan.total_cost;

    NodeId position = start_;
    std::size_t step = 0;

    while (position != goal_) {
        if (step >= max_steps) {
            report.step_limit_hit = true;
            break;
        }
        if (plan.path.size() < 2) {          // defensive: plan must advance
            report.blocked = true;
            break;
        }

        // ---- 1. Drive one hop along the current plan --------------------
        const NodeId next = plan.path[1];
        const Edge* edge = graph_.get_edge(position, next);
        if (edge == nullptr || std::isinf(edge->effective_weight())) {
            report.blocked = true;           // plan invalid: should never happen
            break;
        }

        NavigationStep rec;
        rec.step = step;
        rec.from = position;
        rec.to = next;
        rec.edge_cost = edge->effective_weight();

        report.travelled_cost += rec.edge_cost;
        position = next;
        report.trajectory.push_back(position);
        ++report.moves;

        // ---- 2. Tell D* Lite the agent moved (updates km) ---------------
        dstar.update_start(position);

        // ---- 3. Let the world change, forward edge updates --------------
        if (provider) {
            auto events = provider(step, graph_, position);
            for (const auto& ev : events) {
                dstar.on_edge_changed(ev.from, ev.to, ev.new_weight);
            }
            rec.events_applied = events.size();
            report.total_events += events.size();
            if (!events.empty()) ++report.replans;
        }

        // ---- 4. Repair the inconsistent region only ---------------------
        plan = dstar.compute_shortest_path();
        rec.expansions = plan.nodes_visited;
        rec.repair_us = plan.duration_micros;
        rec.remaining_cost = plan.path_found ? plan.total_cost : INF_WEIGHT;
        report.incremental_expansions += plan.nodes_visited;
        if (verify) cross_check(position, plan);

        report.steps.push_back(rec);
        ++step;

        if (position == goal_) break;
        if (!plan.path_found) {
            report.blocked = true;
            break;
        }
    }

    report.reached_goal = (position == goal_);
    return report;
}

} // namespace dynamic_route
