#pragma once

#include "graph/graph.hpp"
#include "algorithms/d_star_lite.hpp"
#include "simulation/event_simulator.hpp"

#include <cstddef>
#include <functional>
#include <vector>

namespace dynamic_route {

/**
 * @brief Moving-agent route following with D* Lite (Phase 3)
 *
 * Simulates a vehicle that physically drives along the current best route
 * while the road network keeps changing underneath it. At every hop:
 *
 *   1. The agent moves one edge along the current plan.
 *   2. D* Lite is told about the new start (km += h(last, new)), which keeps
 *      all previously queued keys valid without re-sorting the queue.
 *   3. An optional event provider mutates the graph; each changed edge is
 *      forwarded to D* Lite via on_edge_changed().
 *   4. compute_shortest_path() repairs only the inconsistent region.
 *
 * Optionally every step is cross-checked against a from-scratch Dijkstra run
 * from the agent's *current* position, which doubles as the ground-truth
 * oracle and as the "full recomputation" baseline for expansion counts.
 *
 * Reference: Koenig & Likhachev, "D* Lite", AAAI 2002.
 */

struct NavigationStep {
    std::size_t step{0};            ///< hop index (0-based)
    NodeId      from{0};            ///< position before the hop
    NodeId      to{0};              ///< position after the hop
    double      edge_cost{0.0};     ///< weight of the traversed edge
    std::size_t events_applied{0};  ///< graph events applied after this hop
    std::size_t expansions{0};      ///< D* Lite expansions for the repair
    double      remaining_cost{0.0};///< plan cost-to-go after the repair
    double      repair_us{0.0};     ///< wall-clock time of the repair
};

struct NavigationReport {
    bool   reached_goal{false};
    bool   blocked{false};           ///< goal became unreachable / no initial route
    bool   step_limit_hit{false};
    double initial_cost{INF_WEIGHT}; ///< optimal cost from the start at t = 0
    double travelled_cost{0.0};      ///< total weight of the edges driven

    std::vector<NodeId>         trajectory;  ///< visited nodes, start..final
    std::vector<NavigationStep> steps;

    std::size_t moves{0};
    std::size_t replans{0};                  ///< hops after which >=1 event fired
    std::size_t total_events{0};
    std::size_t initial_expansions{0};       ///< cost of the very first search
    std::size_t incremental_expansions{0};   ///< sum of repair expansions
    std::size_t baseline_expansions{0};      ///< Dijkstra-per-hop (verify mode)
    std::size_t verified_steps{0};
    std::size_t mismatches{0};               ///< D* Lite != Dijkstra (must be 0)
};

class RouteNavigator {
public:
    /// Called after every hop. May mutate the graph and must return the list
    /// of events it applied so they can be forwarded to the planner.
    using EventProvider = std::function<std::vector<PerturbationEvent>(
        std::size_t step, Graph& graph, NodeId position)>;

    RouteNavigator(Graph& graph, NodeId start, NodeId goal);

    /**
     * @param provider      Optional dynamic-event source (nullptr = static world).
     * @param max_steps     Hop limit (0 = 4 * |V| safety cap).
     * @param verify        Cross-check every repair against Dijkstra.
     */
    NavigationReport run(const EventProvider& provider = nullptr,
                         std::size_t max_steps = 0,
                         bool verify = false);

private:
    Graph& graph_;
    NodeId start_;
    NodeId goal_;
};

} // namespace dynamic_route
