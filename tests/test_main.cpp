#include "graph/graph.hpp"
#include "algorithms/dijkstra.hpp"
#include "algorithms/a_star.hpp"
#include "algorithms/lpa_star.hpp"
#include "algorithms/d_star_lite.hpp"
#include "simulation/event_simulator.hpp"

#include <cassert>
#include <iostream>
#include <cmath>

void test_graph_construction() {
    std::cout << "[Test 1] Graph Construction & Edge Operations... ";
    dynamic_route::Graph g(5);
    assert(g.num_nodes() == 5);
    g.add_directed_edge(0, 1, 2.5);
    g.add_directed_edge(1, 2, 3.0);
    assert(g.num_edges() == 2);
    assert(g.get_edge_weight(0, 1).value() == 2.5);
    assert(g.get_edge_weight(1, 2).value() == 3.0);

    g.update_edge_weight(0, 1, 5.0);
    assert(g.get_edge_weight(0, 1).value() == 5.0);

    g.set_edge_blocked(0, 1, true);
    assert(std::isinf(g.get_edge_weight(0, 1).value()));

    g.set_edge_blocked(0, 1, false);
    assert(g.get_edge_weight(0, 1).value() == 5.0);
    std::cout << "PASSED\n";
}

void test_static_algorithms() {
    std::cout << "[Test 2] Static Dijkstra and A* Equivalence... ";
    auto g = dynamic_route::EventSimulator::create_grid_graph(5, 5, 1.0);
    auto dijkstra_res = dynamic_route::Dijkstra::find_shortest_path(g, 0, 24);
    auto astar_res = dynamic_route::AStar::find_shortest_path(g, 0, 24);

    assert(dijkstra_res.path_found);
    assert(astar_res.path_found);
    assert(std::abs(dijkstra_res.total_cost - astar_res.total_cost) < 1e-6);
    assert(std::abs(dijkstra_res.total_cost - 8.0) < 1e-6); // Manhattan distance for 5x5 is (4 + 4) = 8
    std::cout << "PASSED (Cost = " << dijkstra_res.total_cost << ")\n";
}

void test_lpa_star_initial() {
    std::cout << "[Test 3] LPA* Initial Search Matches Dijkstra... ";
    auto g = dynamic_route::EventSimulator::create_grid_graph(6, 6, 1.0);
    auto dijkstra_res = dynamic_route::Dijkstra::find_shortest_path(g, 0, 35);

    dynamic_route::LifelongPlanningAStar lpa(g, 0, 35);
    auto lpa_res = lpa.compute_shortest_path();

    assert(lpa_res.path_found);
    assert(std::abs(lpa_res.total_cost - dijkstra_res.total_cost) < 1e-6);
    std::cout << "PASSED\n";
}

void test_d_star_lite_initial() {
    std::cout << "[Test 4] D* Lite Initial Search Matches Dijkstra... ";
    auto g = dynamic_route::EventSimulator::create_grid_graph(6, 6, 1.0);
    auto dijkstra_res = dynamic_route::Dijkstra::find_shortest_path(g, 0, 35);

    dynamic_route::DStarLite dstar(g, 0, 35);
    auto dstar_res = dstar.compute_shortest_path();

    assert(dstar_res.path_found);
    assert(std::abs(dstar_res.total_cost - dijkstra_res.total_cost) < 1e-6);
    std::cout << "PASSED\n";
}

void test_incremental_invariants_under_perturbations() {
    std::cout << "[Test 5] Invariant Test: Incremental == Scratch Dijkstra across 10 Perturbations... ";
    const std::size_t rows = 8;
    const std::size_t cols = 8;
    auto g = dynamic_route::EventSimulator::create_grid_graph(rows, cols, 1.0);
    dynamic_route::EventSimulator sim(12345);

    dynamic_route::NodeId start = 0;
    dynamic_route::NodeId goal = rows * cols - 1;

    dynamic_route::LifelongPlanningAStar lpa(g, start, goal);
    lpa.compute_shortest_path();

    for (int step = 0; step < 10; ++step) {
        auto event = sim.generate_random_event(g);
        lpa.on_edge_changed(event.from, event.to, event.new_weight);

        auto lpa_res = lpa.compute_shortest_path();
        auto dijkstra_res = dynamic_route::Dijkstra::find_shortest_path(g, start, goal);

        assert(lpa_res.path_found == dijkstra_res.path_found);
        if (lpa_res.path_found) {
            assert(std::abs(lpa_res.total_cost - dijkstra_res.total_cost) < 1e-6);
        }
    }
    std::cout << "PASSED\n";
}

int main() {
    std::cout << "=========================================\n";
    std::cout << "🧪 Running DynamicRoute Test Suite 🧪\n";
    std::cout << "=========================================\n";

    test_graph_construction();
    test_static_algorithms();
    test_lpa_star_initial();
    test_d_star_lite_initial();
    test_incremental_invariants_under_perturbations();

    std::cout << "\n✅ ALL UNIT & INVARIANT TESTS PASSED!\n";
    std::cout << "=========================================\n";
    return 0;
}
