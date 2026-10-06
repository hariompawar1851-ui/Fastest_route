#include "graph/graph.hpp"
#include "algorithms/dijkstra.hpp"
#include "algorithms/a_star.hpp"
#include "algorithms/lpa_star.hpp"
#include "algorithms/d_star_lite.hpp"
#include "simulation/event_simulator.hpp"

#include <iostream>
#include <iomanip>

int main() {
    std::cout << "====================================================\n";
    std::cout << " ⚡ DynamicRoute: Incremental Shortest-Path Engine ⚡\n";
    std::cout << "====================================================\n\n";

    // 1. Create a 10x10 grid road network (100 nodes, 360 directed edges)
    const std::size_t rows = 10;
    const std::size_t cols = 10;
    dynamic_route::Graph graph = dynamic_route::EventSimulator::create_grid_graph(rows, cols, 1.0);

    dynamic_route::NodeId start = 0;
    dynamic_route::NodeId goal = rows * cols - 1;

    std::cout << "🌐 Road Network Created: " << graph.num_nodes() << " nodes, "
              << graph.num_edges() << " edges.\n";
    std::cout << "📍 Route: Node " << start << " -> Node " << goal << "\n\n";

    // 2. Initial Route Computation
    std::cout << "--- Initial Search ---\n";
    auto dijkstra_init = dynamic_route::Dijkstra::find_shortest_path(graph, start, goal);
    std::cout << "Dijkstra: Cost = " << dijkstra_init.total_cost
              << ", Visited = " << dijkstra_init.nodes_visited
              << ", Latency = " << dijkstra_init.duration_micros << " us\n";

    dynamic_route::LifelongPlanningAStar lpa(graph, start, goal);
    auto lpa_init = lpa.compute_shortest_path();
    std::cout << "LPA*    : Cost = " << lpa_init.total_cost
              << ", Visited = " << lpa_init.nodes_visited
              << ", Latency = " << lpa_init.duration_micros << " us\n\n";

    // 3. Simulate a Dynamic Traffic Incident (Block a road on the current fastest path)
    if (lpa_init.path.size() >= 3) {
        dynamic_route::NodeId block_u = lpa_init.path[1];
        dynamic_route::NodeId block_v = lpa_init.path[2];

        std::cout << "🚨 Dynamic Incident: Road between Node " << block_u
                  << " and Node " << block_v << " is BLOCKED (accident)!\n\n";

        graph.set_edge_blocked(block_u, block_v, true);
        lpa.on_edge_changed(block_u, block_v, dynamic_route::INF_WEIGHT);

        // 4. Recovery: Recomputed Dijkstra vs Incremental LPA*
        std::cout << "--- Route Recovery Comparison ---\n";
        auto dijkstra_recompute = dynamic_route::Dijkstra::find_shortest_path(graph, start, goal);
        auto lpa_repair = lpa.compute_shortest_path();

        std::cout << std::fixed << std::setprecision(2);
        std::cout << "Full Dijkstra Recalculation:\n"
                  << "  - New Cost     : " << dijkstra_recompute.total_cost << "\n"
                  << "  - Nodes Visited: " << dijkstra_recompute.nodes_visited << "\n"
                  << "  - Time Taken   : " << dijkstra_recompute.duration_micros << " us\n\n";

        std::cout << "Incremental LPA* Repair:\n"
                  << "  - New Cost     : " << lpa_repair.total_cost << "\n"
                  << "  - Nodes Visited: " << lpa_repair.nodes_visited << "\n"
                  << "  - Time Taken   : " << lpa_repair.duration_micros << " us\n\n";

        bool costs_match = (std::abs(dijkstra_recompute.total_cost - lpa_repair.total_cost) < 1e-6);
        std::cout << "🎯 Correctness Verification: "
                  << (costs_match ? "PASSED (Costs strictly identical!)" : "FAILED")
                  << "\n";

        if (dijkstra_recompute.nodes_visited > 0) {
            double node_reduction = (1.0 - static_cast<double>(lpa_repair.nodes_visited) / static_cast<double>(dijkstra_recompute.nodes_visited)) * 100.0;
            std::cout << "⚡ Node Expansion Reduction: " << node_reduction << "%\n";
        }
    }

    std::cout << "\n====================================================\n";
    return 0;
}
