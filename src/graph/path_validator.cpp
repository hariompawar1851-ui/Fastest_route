#include "graph/path_validator.hpp"
#include "simulation/event_simulator.hpp"
#include <functional>
#include <random>

namespace dynamic_route {

// ---------------------------------------------------------------------------
// validate_walk
// ---------------------------------------------------------------------------

ValidationResult PathValidator::validate_walk(const Graph& graph,
                                               const std::vector<NodeId>& path,
                                               double reported_cost,
                                               double tol) {
    ValidationResult r;

    if (path.empty()) {
        r.ok = false;
        r.path_is_valid_walk = false;
        r.error_message = "Path is empty";
        return r;
    }

    // Walk the path and sum up actual edge weights
    double actual_cost = 0.0;
    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
        NodeId u = path[i];
        NodeId v = path[i + 1];
        auto w_opt = graph.get_edge_weight(u, v);
        if (!w_opt.has_value() || std::isinf(*w_opt)) {
            r.ok = false;
            r.path_is_valid_walk = false;
            r.error_message = "No unblocked edge between nodes "
                + std::to_string(u) + " → " + std::to_string(v);
            return r;
        }
        actual_cost += *w_opt;
    }

    r.incremental_cost = reported_cost;
    r.reference_cost   = actual_cost;
    r.cost_delta       = std::abs(actual_cost - reported_cost);

    if (r.cost_delta > tol) {
        r.ok = false;
        r.error_message = "Cost mismatch: reported=" + std::to_string(reported_cost)
                        + " actual=" + std::to_string(actual_cost);
    }

    return r;
}

// ---------------------------------------------------------------------------
// compare_with_dijkstra
// ---------------------------------------------------------------------------

ValidationResult PathValidator::compare_with_dijkstra(const Graph& graph,
                                                        NodeId source,
                                                        NodeId target,
                                                        const SearchResult& incremental_result,
                                                        double tol) {
    ValidationResult r;

    // Ground truth
    SearchResult ref = Dijkstra::find_shortest_path(graph, source, target);

    r.reference_cost   = ref.total_cost;
    r.incremental_cost = incremental_result.total_cost;
    r.cost_delta       = std::abs(ref.total_cost - incremental_result.total_cost);

    if (ref.path_found != incremental_result.path_found) {
        r.ok = false;
        r.error_message = "path_found flag mismatch: Dijkstra="
            + std::to_string(ref.path_found)
            + " incremental=" + std::to_string(incremental_result.path_found);
        return r;
    }

    if (ref.path_found && r.cost_delta > tol) {
        r.ok = false;
        r.error_message = "Cost mismatch: Dijkstra=" + std::to_string(ref.total_cost)
                        + " incremental=" + std::to_string(incremental_result.total_cost)
                        + " delta=" + std::to_string(r.cost_delta);
    }

    return r;
}

// ---------------------------------------------------------------------------
// stress_test
// ---------------------------------------------------------------------------

int PathValidator::stress_test(
    Graph& graph,
    NodeId source,
    NodeId target,
    int    n_steps,
    std::uint32_t seed,
    std::function<void(NodeId, NodeId, double)> update_fn,
    std::function<SearchResult()>               query_fn)
{
    EventSimulator sim(seed);
    int passed = 0;

    for (int step = 0; step < n_steps; ++step) {
        auto event = sim.generate_random_event(graph);
        update_fn(event.from, event.to, event.new_weight);

        auto inc_result = query_fn();
        auto vr = compare_with_dijkstra(graph, source, target, inc_result);
        if (vr.ok) ++passed;
    }

    return passed;
}

} // namespace dynamic_route
