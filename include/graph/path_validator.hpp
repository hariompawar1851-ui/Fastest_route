#pragma once

#include "graph/graph.hpp"
#include "algorithms/dijkstra.hpp"
#include <vector>
#include <string>
#include <cstdint>
#include <functional>
#include <cmath>

namespace dynamic_route {

/**
 * @brief PathValidator — Route Correctness & Invariant Checker (Phase 1)
 *
 * Provides a suite of validation functions used by the automated test suite
 * and the benchmarking harness to verify that incremental algorithms
 * (LPA*, D* Lite, Ramalingam-Reps) produce exactly the same minimum-cost
 * routes as the ground-truth full Dijkstra recomputation.
 *
 * The Zero-Tolerance Correctness guarantee:
 *   For every graph state and every source/target pair tested,
 *   cost(incremental_path) == cost(dijkstra_path)
 *   within floating-point tolerance (1e-9).
 *
 * This class is stateless — all methods are static.
 */

struct ValidationResult {
    bool   ok{true};
    double incremental_cost{0.0};
    double reference_cost{0.0};
    double cost_delta{0.0};
    bool   path_is_valid_walk{true};
    std::string error_message{};
};

class PathValidator {
public:
    /**
     * @brief Check that a path forms a valid walk in the given graph.
     *
     * A valid walk requires:
     *   (a) The path is non-empty.
     *   (b) Every consecutive pair (path[i], path[i+1]) has an
     *       unblocked directed edge in the graph.
     *   (c) The sum of edge weights equals reported_cost (within tol).
     *
     * @param graph         The road network
     * @param path          Sequence of NodeIds from source to target
     * @param reported_cost The cost claimed by the algorithm
     * @param tol           Floating-point tolerance (default 1e-9)
     */
    static ValidationResult validate_walk(const Graph& graph,
                                          const std::vector<NodeId>& path,
                                          double reported_cost,
                                          double tol = 1e-9);

    /**
     * @brief Compare an incremental algorithm's result against Dijkstra.
     *
     * Runs Dijkstra on the CURRENT graph state (after perturbations) and
     * checks that incremental_result matches on:
     *   - path_found flag
     *   - total_cost (within tol)
     *
     * @param graph              Graph in its current state
     * @param source             Source node
     * @param target             Target node
     * @param incremental_result Result from LPA* / D* Lite / RR
     * @param tol                Floating-point tolerance
     */
    static ValidationResult compare_with_dijkstra(const Graph& graph,
                                                   NodeId source,
                                                   NodeId target,
                                                   const SearchResult& incremental_result,
                                                   double tol = 1e-9);

    /**
     * @brief Run N random perturbations and verify incremental == Dijkstra
     *        at every step.
     *
     * Internally calls generate_random_event() on the graph, notifies the
     * algorithm via the provided callback, then calls compare_with_dijkstra.
     *
     * Returns the number of steps where correctness was confirmed.
     *
     * @param graph        Graph (mutated by perturbations)
     * @param source       Source node
     * @param target       Target node
     * @param n_steps      Number of random perturbation steps
     * @param seed         RNG seed for reproducibility
     * @param update_fn    Callback: (from, to, new_weight) to notify the
     *                     incremental algorithm of each edge change.
     * @param query_fn     Callback: () → SearchResult from the incremental algo.
     */
    static int stress_test(
        Graph& graph,
        NodeId source,
        NodeId target,
        int    n_steps,
        std::uint32_t seed,
        std::function<void(NodeId, NodeId, double)> update_fn,
        std::function<SearchResult()>               query_fn
    );
};

} // namespace dynamic_route
