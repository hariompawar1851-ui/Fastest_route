// =============================================================================
// DynamicRoute — Complete Test Suite  (Phase 0 + Phase 1)
// =============================================================================
// Tests 1-5   : Phase 0 — baseline correctness of all static + incremental algorithms
// Tests 6-10  : Phase 1 — ShortestPathTree, Ramalingam-Reps (real impl),
//                          GraphIO generators, PathValidator, stress-test harness
// =============================================================================

#include "graph/graph.hpp"
#include "graph/shortest_path_tree.hpp"
#include "graph/graph_io.hpp"
#include "graph/path_validator.hpp"
#include "algorithms/dijkstra.hpp"
#include "algorithms/a_star.hpp"
#include "algorithms/lpa_star.hpp"
#include "algorithms/d_star_lite.hpp"
#include "algorithms/ramalingam_reps.hpp"
#include "simulation/event_simulator.hpp"

#include <cassert>
#include <iostream>
#include <cmath>

// ---------------------------------------------------------------------------
// PHASE 0 TESTS
// ---------------------------------------------------------------------------

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
    auto astar_res    = dynamic_route::AStar::find_shortest_path(g, 0, 24);

    assert(dijkstra_res.path_found);
    assert(astar_res.path_found);
    assert(std::abs(dijkstra_res.total_cost - astar_res.total_cost) < 1e-6);
    assert(std::abs(dijkstra_res.total_cost - 8.0) < 1e-6);
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
    std::cout << "[Test 5] Invariant: LPA* == Dijkstra over 10 Perturbations... ";
    const std::size_t rows = 8, cols = 8;
    auto g = dynamic_route::EventSimulator::create_grid_graph(rows, cols, 1.0);
    dynamic_route::EventSimulator sim(12345);

    dynamic_route::NodeId start = 0;
    dynamic_route::NodeId goal  = rows * cols - 1;

    dynamic_route::LifelongPlanningAStar lpa(g, start, goal);
    lpa.compute_shortest_path();

    for (int step = 0; step < 10; ++step) {
        auto event = sim.generate_random_event(g);
        lpa.on_edge_changed(event.from, event.to, event.new_weight);

        auto lpa_res      = lpa.compute_shortest_path();
        auto dijkstra_res = dynamic_route::Dijkstra::find_shortest_path(g, start, goal);

        assert(lpa_res.path_found == dijkstra_res.path_found);
        if (lpa_res.path_found) {
            assert(std::abs(lpa_res.total_cost - dijkstra_res.total_cost) < 1e-6);
        }
    }
    std::cout << "PASSED\n";
}

// ---------------------------------------------------------------------------
// PHASE 1 TESTS
// ---------------------------------------------------------------------------

void test_shortest_path_tree() {
    std::cout << "[Test 6] ShortestPathTree — SPT construction & subtree ops... ";

    // Build a simple graph: 0->1 (2.0), 1->2 (3.0), 0->2 (10.0)
    dynamic_route::Graph g(4);
    g.add_directed_edge(0, 1, 2.0);
    g.add_directed_edge(1, 2, 3.0);
    g.add_directed_edge(0, 2, 10.0);
    g.add_directed_edge(2, 3, 1.0);

    // Build SPT via RamalingamReps (uses its initialize() internally)
    dynamic_route::RamalingamReps rr(g, 0);
    rr.initialize();

    // Check distances
    assert(std::abs(rr.get_distance(0) - 0.0)  < 1e-9);
    assert(std::abs(rr.get_distance(1) - 2.0)  < 1e-9);
    assert(std::abs(rr.get_distance(2) - 5.0)  < 1e-9);  // via 0->1->2
    assert(std::abs(rr.get_distance(3) - 6.0)  < 1e-9);  // via 0->1->2->3

    // Check SPT parent structure
    const auto& spt = rr.spt();
    assert(spt.parent(1) == 0);
    assert(spt.parent(2) == 1);
    assert(spt.parent(3) == 2);

    // Collect subtree rooted at node 1: should contain {1, 2, 3}
    auto subtree = spt.collect_subtree(1);
    assert(subtree.size() == 3);

    std::cout << "PASSED\n";
}

void test_ramalingam_reps_edge_increase() {
    std::cout << "[Test 7] Ramalingam-Reps: incremental repair on edge cost increase... ";

    // 5-node chain: 0->1 (1), 1->2 (1), 2->3 (1), 3->4 (1)
    // Plus a bypass: 0->3 (5)
    dynamic_route::Graph g(5);
    g.add_directed_edge(0, 1, 1.0);
    g.add_directed_edge(1, 2, 1.0);
    g.add_directed_edge(2, 3, 1.0);
    g.add_directed_edge(3, 4, 1.0);
    g.add_directed_edge(0, 3, 5.0);

    dynamic_route::RamalingamReps rr(g, 0);
    rr.initialize();

    // Initial: shortest path to 4 is 0->1->2->3->4 = 4.0
    assert(std::abs(rr.get_distance(4) - 4.0) < 1e-9);

    // Block edge 1->2 (increase to INF)
    g.set_edge_blocked(1, 2, true);
    rr.on_edge_increase(1, 2, 1.0, dynamic_route::INF_WEIGHT);

    // After repair: path must go 0->3 (cost 5) -> 3->4 (cost 1) = 6.0
    // (the direct chain 0->1->2->3->4 is broken)
    double rr_dist = rr.get_distance(4);
    auto   ref     = dynamic_route::Dijkstra::find_shortest_path(g, 0, 4);

    assert(ref.path_found);
    assert(std::abs(rr_dist - ref.total_cost) < 1e-9);

    // Verify that the repair was LOCAL (not full recompute)
    // At least nodes 2,3,4 should have been invalidated (subtree of node 2)
    assert(rr.nodes_invalidated_last() >= 2);

    std::cout << "PASSED (dist=" << rr_dist
              << ", invalidated=" << rr.nodes_invalidated_last()
              << ", relaxed="     << rr.nodes_relaxed_last() << ")\n";
}

void test_ramalingam_reps_edge_decrease() {
    std::cout << "[Test 8] Ramalingam-Reps: incremental improvement on edge cost decrease... ";

    // 3-node graph: 0->1 (10), 0->2 (1), 1->2 (10)
    dynamic_route::Graph g(3);
    g.add_directed_edge(0, 1, 10.0);
    g.add_directed_edge(0, 2,  1.0);
    g.add_directed_edge(1, 2, 10.0);

    dynamic_route::RamalingamReps rr(g, 0);
    rr.initialize();

    // Initial: dist[1]=10, dist[2]=1
    assert(std::abs(rr.get_distance(1) - 10.0) < 1e-9);
    assert(std::abs(rr.get_distance(2) -  1.0) < 1e-9);

    // Decrease edge 0->1 to 0.5
    g.update_edge_weight(0, 1, 0.5);
    rr.on_edge_decrease(0, 1, 10.0, 0.5);

    // Now dist[1] should be 0.5
    assert(std::abs(rr.get_distance(1) - 0.5) < 1e-9);

    // Verify against Dijkstra
    auto ref = dynamic_route::Dijkstra::find_shortest_path(g, 0, 1);
    assert(std::abs(rr.get_distance(1) - ref.total_cost) < 1e-9);

    std::cout << "PASSED\n";
}

void test_graph_io_generators() {
    std::cout << "[Test 9] GraphIO: Grid & Random Geometric Graph generators... ";

    // Grid graph
    auto grid = dynamic_route::GraphIO::make_grid(4, 4, 1.0);
    assert(grid.num_nodes() == 16);
    // 4x4 grid: 2*(4-1)*4 = 24 directed pairs → 48 directed edges (bidirectional)
    assert(grid.num_edges() == 48);

    // Check a specific edge exists: node 0 → node 1 (horizontal)
    assert(grid.get_edge_weight(0, 1).has_value());
    assert(std::abs(grid.get_edge_weight(0, 1).value() - 1.0) < 1e-9);

    // Random Geometric Graph (small, dense radius to ensure connectivity)
    auto rgg = dynamic_route::GraphIO::make_random_geometric(50, 0.4, 42);
    assert(rgg.num_nodes() == 50);
    assert(rgg.num_edges() > 0);

    // Verify a Dijkstra search works on the RGG
    auto res = dynamic_route::Dijkstra::find_shortest_path(rgg, 0, 49);
    // With radius=0.4 on 50 nodes, highly likely to be connected
    // (just check the algo runs without crash; connectivity not guaranteed for all seeds)
    (void)res;

    std::cout << "PASSED (grid edges=" << grid.num_edges()
              << ", RGG edges=" << rgg.num_edges() << ")\n";
}

void test_path_validator() {
    std::cout << "[Test 10] PathValidator: walk validation & Dijkstra comparison... ";

    auto g = dynamic_route::GraphIO::make_grid(5, 5, 1.0);
    auto dres = dynamic_route::Dijkstra::find_shortest_path(g, 0, 24);
    assert(dres.path_found);

    // validate_walk on a correct path
    auto vr = dynamic_route::PathValidator::validate_walk(
        g, dres.path, dres.total_cost);
    assert(vr.ok);
    assert(vr.path_is_valid_walk);

    // compare_with_dijkstra — should match itself
    auto cr = dynamic_route::PathValidator::compare_with_dijkstra(g, 0, 24, dres);
    assert(cr.ok);
    assert(std::abs(cr.cost_delta) < 1e-9);

    // Stress test with PathValidator harness using LPA*
    dynamic_route::LifelongPlanningAStar lpa(g, 0, 24);
    lpa.compute_shortest_path();

    int passed_steps = dynamic_route::PathValidator::stress_test(
        g, 0, 24, 5, /*seed=*/7777,
        [&](dynamic_route::NodeId from, dynamic_route::NodeId to, double w) {
            lpa.on_edge_changed(from, to, w);
        },
        [&]() { return lpa.compute_shortest_path(); }
    );
    assert(passed_steps == 5);

    std::cout << "PASSED (stress_test: " << passed_steps << "/5 steps verified)\n";
}

void test_metadata_and_o1_lookup() {
    std::cout << "[Test 11] Graph Metadata & O(1) Edge Lookup... ";
    dynamic_route::Graph g;
    auto n0 = g.add_node(0.0, 0.0, "Junction_A", "roundabout");
    auto n1 = g.add_node(10.0, 0.0, "Junction_B", "intersection");

    assert(g.get_node(n0).node_type == "roundabout");
    assert(g.get_node(n1).node_type == "intersection");

    g.add_directed_edge(n0, n1, 4.5, "motorway", 250.0, 100.0);

    const auto* edge_ptr = g.get_edge(n0, n1);
    assert(edge_ptr != nullptr);
    assert(edge_ptr->road_type == "motorway");
    assert(edge_ptr->capacity == 250.0);
    assert(edge_ptr->max_speed == 100.0);
    assert(std::abs(edge_ptr->effective_weight() - 4.5) < 1e-9);
    (void)edge_ptr;

    // O(1) weight update
    bool updated = g.update_edge_weight(n0, n1, 2.0);
    assert(updated);
    assert(std::abs(g.get_edge_weight(n0, n1).value() - 2.0) < 1e-9);
    (void)updated;

    // Non-existent edge query returns nullopt / nullptr
    assert(!g.get_edge_weight(n1, n0).has_value());
    assert(g.get_edge(n1, n0) == nullptr);

    std::cout << "PASSED\n";
}

void test_graph_io_file_roundtrip() {
    std::cout << "[Test 12] GraphIO: File Export & Load Round-Trip... ";
    auto g_orig = dynamic_route::GraphIO::make_grid(3, 3, 2.0);
    const std::string tmp_file = "/tmp/dynamic_route_test_roundtrip.txt";

    bool saved = dynamic_route::GraphIO::save_edge_list(g_orig, tmp_file);
    assert(saved);
    (void)saved;

    auto g_loaded = dynamic_route::GraphIO::load_edge_list(tmp_file);
    assert(g_loaded.num_nodes() == g_orig.num_nodes());
    assert(g_loaded.num_edges() == g_orig.num_edges());

    // Verify shortest path is identical on loaded graph
    auto d_orig   = dynamic_route::Dijkstra::find_shortest_path(g_orig, 0, 8);
    auto d_loaded = dynamic_route::Dijkstra::find_shortest_path(g_loaded, 0, 8);
    assert(d_orig.path_found && d_loaded.path_found);
    assert(std::abs(d_orig.total_cost - d_loaded.total_cost) < 1e-9);

    std::remove(tmp_file.c_str());
    std::cout << "PASSED\n";
}

void test_ramalingam_reps_path_reconstruction() {
    std::cout << "[Test 13] Ramalingam-Reps: Path Reconstruction & Walk Validity... ";
    auto g = dynamic_route::GraphIO::make_grid(4, 4, 1.0);
    dynamic_route::RamalingamReps rr(g, 0);
    rr.initialize();

    auto path_init = rr.get_path(15);
    assert(!path_init.empty());
    assert(path_init.front() == 0);
    assert(path_init.back() == 15);

    auto vr_init = dynamic_route::PathValidator::validate_walk(g, path_init, rr.get_distance(15));
    assert(vr_init.ok);

    // Block an edge along the current path
    if (path_init.size() >= 3) {
        auto u = path_init[1];
        auto v = path_init[2];
        g.set_edge_blocked(u, v, true);
        rr.on_edge_increase(u, v, 1.0, dynamic_route::INF_WEIGHT);

        auto path_repaired = rr.get_path(15);
        assert(!path_repaired.empty());
        auto vr_repaired = dynamic_route::PathValidator::validate_walk(g, path_repaired, rr.get_distance(15));
        assert(vr_repaired.ok);

        auto d_check = dynamic_route::Dijkstra::find_shortest_path(g, 0, 15);
        assert(std::abs(rr.get_distance(15) - d_check.total_cost) < 1e-9);
    }
    std::cout << "PASSED\n";
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main() {
    std::cout << "============================================================\n";
    std::cout << "  🧪 DynamicRoute — Full Test Suite (Phase 0 + Phase 1) 🧪\n";
    std::cout << "============================================================\n\n";

    std::cout << "--- Phase 0: Baseline Correctness ---\n";
    test_graph_construction();
    test_static_algorithms();
    test_lpa_star_initial();
    test_d_star_lite_initial();
    test_incremental_invariants_under_perturbations();

    std::cout << "\n--- Phase 1: Core DSA Engine ---\n";
    test_shortest_path_tree();
    test_ramalingam_reps_edge_increase();
    test_ramalingam_reps_edge_decrease();
    test_graph_io_generators();
    test_path_validator();
    test_metadata_and_o1_lookup();
    test_graph_io_file_roundtrip();
    test_ramalingam_reps_path_reconstruction();

    std::cout << "\n✅ ALL 13 TESTS PASSED!\n";
    std::cout << "============================================================\n";
    return 0;
}
