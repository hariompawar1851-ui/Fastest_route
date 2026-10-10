// =============================================================================
// DynamicRoute — Complete Test Suite  (Phase 0 + Phase 1 + Phase 2)
// =============================================================================
// Tests 1–5   : Phase 0 — baseline correctness of all static + incremental algorithms
// Tests 6–13  : Phase 1 — ShortestPathTree, Ramalingam-Reps (real impl),
//                          GraphIO generators, PathValidator, stress-test harness
// Tests 14–19 : Phase 2, Part 1 — large-scale invariant stress testing
// Tests 20–22 : Phase 2, Part 2 — explainability & diagnostics engine
// Tests 23–26 : Phase 2, Part 3 — advanced traffic simulator
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
#include "simulation/traffic_scenario.hpp"
#include "simulation/scenario_runner.hpp"
#include "diagnostics/audit_log.hpp"
#include "diagnostics/causal_reporter.hpp"

#include <cassert>
#include <iostream>
#include <cmath>
#include <string>
#include <numeric>
#include <vector>
#include <cstdio>

// ===========================================================================
// Phase 2 Part 1 helpers — generic stress driver
// ===========================================================================

namespace {

template <typename AlgoT>
int run_stress(dynamic_route::Graph& graph,
               AlgoT& algo,
               dynamic_route::NodeId source,
               dynamic_route::NodeId target,
               int n_steps,
               std::uint32_t seed,
               const std::string& label)
{
    dynamic_route::EventSimulator sim(seed);
    int passed = 0;

    for (int step = 0; step < n_steps; ++step) {
        auto event = sim.generate_random_event(graph);
        algo.on_edge_changed(event.from, event.to, event.new_weight);

        auto inc = algo.compute_shortest_path();
        auto ref = dynamic_route::Dijkstra::find_shortest_path(graph, source, target);

        bool ok = (inc.path_found == ref.path_found);
        if (ok && inc.path_found)
            ok = (std::abs(inc.total_cost - ref.total_cost) < 1e-6);

        if (ok) {
            ++passed;
        } else {
            std::cerr << "\n  [FAIL " << label << " step " << step
                      << "] inc=" << inc.total_cost
                      << " ref=" << ref.total_cost
                      << " inc_found=" << inc.path_found
                      << " ref_found=" << ref.path_found << "\n";
        }
    }
    return passed;
}

int run_rr_stress(dynamic_route::Graph& graph,
                  dynamic_route::RamalingamReps& rr,
                  dynamic_route::NodeId source,
                  dynamic_route::NodeId target,
                  int n_steps,
                  std::uint32_t seed,
                  std::vector<std::size_t>* invalidated_counts = nullptr)
{
    dynamic_route::EventSimulator sim(seed);
    int passed = 0;

    for (int step = 0; step < n_steps; ++step) {
        auto event = sim.generate_random_event(graph);

        if (event.new_weight >= event.old_weight)
            rr.on_edge_increase(event.from, event.to, event.old_weight, event.new_weight);
        else
            rr.on_edge_decrease(event.from, event.to, event.old_weight, event.new_weight);

        if (invalidated_counts)
            invalidated_counts->push_back(rr.nodes_invalidated_last());

        double rr_dist = rr.get_distance(target);
        auto   ref     = dynamic_route::Dijkstra::find_shortest_path(graph, source, target);

        bool ok;
        if (!ref.path_found)
            ok = std::isinf(rr_dist);
        else
            ok = (std::abs(rr_dist - ref.total_cost) < 1e-6);

        if (ok) {
            ++passed;
        } else {
            std::cerr << "\n  [FAIL RR step " << step
                      << "] rr_dist=" << rr_dist
                      << " ref=" << ref.total_cost << "\n";
        }
    }
    return passed;
}

// Phase 2 Part 3 helper: verify incremental result matches Dijkstra
static void check_against_dijkstra(const dynamic_route::Graph& g,
                                   dynamic_route::NodeId src,
                                   dynamic_route::NodeId tgt,
                                   bool inc_found,
                                   double inc_cost,
                                   int step)
{
    auto ref = dynamic_route::Dijkstra::find_shortest_path(g, src, tgt);
    if (ref.path_found != inc_found) {
        std::cerr << "[FAIL step " << step << "] path_found mismatch\n";
        assert(false);
    }
    if (inc_found) {
        double delta = std::abs(inc_cost - ref.total_cost);
        if (delta >= 1e-6) {
            std::cerr << "[FAIL step " << step << "] cost delta=" << delta
                      << " inc=" << inc_cost << " ref=" << ref.total_cost << "\n";
            assert(false);
        }
    }
}

} // anonymous namespace

// ===========================================================================
// PHASE 0 TESTS  (1–5)
// ===========================================================================

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
        if (lpa_res.path_found)
            assert(std::abs(lpa_res.total_cost - dijkstra_res.total_cost) < 1e-6);
    }
    std::cout << "PASSED\n";
}

// ===========================================================================
// PHASE 1 TESTS  (6–13)
// ===========================================================================

void test_shortest_path_tree() {
    std::cout << "[Test 6] ShortestPathTree — SPT construction & subtree ops... ";

    dynamic_route::Graph g(4);
    g.add_directed_edge(0, 1, 2.0);
    g.add_directed_edge(1, 2, 3.0);
    g.add_directed_edge(0, 2, 10.0);
    g.add_directed_edge(2, 3, 1.0);

    dynamic_route::RamalingamReps rr(g, 0);
    rr.initialize();

    assert(std::abs(rr.get_distance(0) - 0.0) < 1e-9);
    assert(std::abs(rr.get_distance(1) - 2.0) < 1e-9);
    assert(std::abs(rr.get_distance(2) - 5.0) < 1e-9);   // via 0->1->2
    assert(std::abs(rr.get_distance(3) - 6.0) < 1e-9);   // via 0->1->2->3

    const auto& spt = rr.spt();
    assert(spt.parent(1) == 0);
    assert(spt.parent(2) == 1);
    assert(spt.parent(3) == 2);

    auto subtree = spt.collect_subtree(1);
    assert(subtree.size() == 3);

    std::cout << "PASSED\n";
}

void test_ramalingam_reps_edge_increase() {
    std::cout << "[Test 7] Ramalingam-Reps: incremental repair on edge cost increase... ";

    dynamic_route::Graph g(5);
    g.add_directed_edge(0, 1, 1.0);
    g.add_directed_edge(1, 2, 1.0);
    g.add_directed_edge(2, 3, 1.0);
    g.add_directed_edge(3, 4, 1.0);
    g.add_directed_edge(0, 3, 5.0);

    dynamic_route::RamalingamReps rr(g, 0);
    rr.initialize();

    assert(std::abs(rr.get_distance(4) - 4.0) < 1e-9);

    g.set_edge_blocked(1, 2, true);
    rr.on_edge_increase(1, 2, 1.0, dynamic_route::INF_WEIGHT);

    double rr_dist = rr.get_distance(4);
    auto   ref     = dynamic_route::Dijkstra::find_shortest_path(g, 0, 4);

    assert(ref.path_found);
    assert(std::abs(rr_dist - ref.total_cost) < 1e-9);
    assert(rr.nodes_invalidated_last() >= 2);

    std::cout << "PASSED (dist=" << rr_dist
              << ", invalidated=" << rr.nodes_invalidated_last()
              << ", relaxed="     << rr.nodes_relaxed_last() << ")\n";
}

void test_ramalingam_reps_edge_decrease() {
    std::cout << "[Test 8] Ramalingam-Reps: incremental improvement on edge cost decrease... ";

    dynamic_route::Graph g(3);
    g.add_directed_edge(0, 1, 10.0);
    g.add_directed_edge(0, 2,  1.0);
    g.add_directed_edge(1, 2, 10.0);

    dynamic_route::RamalingamReps rr(g, 0);
    rr.initialize();

    assert(std::abs(rr.get_distance(1) - 10.0) < 1e-9);
    assert(std::abs(rr.get_distance(2) -  1.0) < 1e-9);

    g.update_edge_weight(0, 1, 0.5);
    rr.on_edge_decrease(0, 1, 10.0, 0.5);

    assert(std::abs(rr.get_distance(1) - 0.5) < 1e-9);

    auto ref = dynamic_route::Dijkstra::find_shortest_path(g, 0, 1);
    assert(std::abs(rr.get_distance(1) - ref.total_cost) < 1e-9);

    std::cout << "PASSED\n";
}

void test_graph_io_generators() {
    std::cout << "[Test 9] GraphIO: Grid & Random Geometric Graph generators... ";

    auto grid = dynamic_route::GraphIO::make_grid(4, 4, 1.0);
    assert(grid.num_nodes() == 16);
    assert(grid.num_edges() == 48);
    assert(grid.get_edge_weight(0, 1).has_value());
    assert(std::abs(grid.get_edge_weight(0, 1).value() - 1.0) < 1e-9);

    auto rgg = dynamic_route::GraphIO::make_random_geometric(50, 0.4, 42);
    assert(rgg.num_nodes() == 50);
    assert(rgg.num_edges() > 0);

    auto res = dynamic_route::Dijkstra::find_shortest_path(rgg, 0, 49);
    (void)res;

    std::cout << "PASSED (grid edges=" << grid.num_edges()
              << ", RGG edges=" << rgg.num_edges() << ")\n";
}

void test_path_validator() {
    std::cout << "[Test 10] PathValidator: walk validation & Dijkstra comparison... ";

    auto g = dynamic_route::GraphIO::make_grid(5, 5, 1.0);
    auto dres = dynamic_route::Dijkstra::find_shortest_path(g, 0, 24);
    assert(dres.path_found);

    auto vr = dynamic_route::PathValidator::validate_walk(g, dres.path, dres.total_cost);
    assert(vr.ok);
    assert(vr.path_is_valid_walk);

    auto cr = dynamic_route::PathValidator::compare_with_dijkstra(g, 0, 24, dres);
    assert(cr.ok);
    assert(std::abs(cr.cost_delta) < 1e-9);

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

    bool updated = g.update_edge_weight(n0, n1, 2.0);
    assert(updated);
    assert(std::abs(g.get_edge_weight(n0, n1).value() - 2.0) < 1e-9);
    (void)updated;

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
    assert(path_init.back()  == 15);

    auto vr_init = dynamic_route::PathValidator::validate_walk(g, path_init, rr.get_distance(15));
    assert(vr_init.ok);

    if (path_init.size() >= 3) {
        auto u = path_init[1];
        auto v = path_init[2];
        g.set_edge_blocked(u, v, true);
        rr.on_edge_increase(u, v, 1.0, dynamic_route::INF_WEIGHT);

        auto path_repaired = rr.get_path(15);
        assert(!path_repaired.empty());
        auto vr_repaired = dynamic_route::PathValidator::validate_walk(
            g, path_repaired, rr.get_distance(15));
        assert(vr_repaired.ok);

        auto d_check = dynamic_route::Dijkstra::find_shortest_path(g, 0, 15);
        assert(std::abs(rr.get_distance(15) - d_check.total_cost) < 1e-9);
    }
    std::cout << "PASSED\n";
}

// ===========================================================================
// PHASE 2, PART 1 TESTS  (14–19) — Large-Scale Invariant Stress Testing
// ===========================================================================

void test14_lpa_star_1000_perturbations_grid30() {
    std::cout << "[Test 14] LPA* — 1,000 perturbations on 30x30 grid (900 nodes)... ";
    std::cout.flush();

    auto g = dynamic_route::GraphIO::make_grid(30, 30, 1.0);
    assert(g.num_nodes() == 900);

    dynamic_route::NodeId source = 0, target = 899;
    dynamic_route::LifelongPlanningAStar lpa(g, source, target);
    lpa.compute_shortest_path();

    constexpr int N = 1000;
    int passed = run_stress(g, lpa, source, target, N, 100, "LPA*");

    assert(passed == N);
    std::cout << "PASSED (" << passed << "/" << N << " steps verified)\n";
}

void test15_d_star_lite_1000_perturbations_grid30() {
    std::cout << "[Test 15] D* Lite — 1,000 perturbations on 30x30 grid (900 nodes)... ";
    std::cout.flush();

    auto g = dynamic_route::GraphIO::make_grid(30, 30, 1.0);
    assert(g.num_nodes() == 900);

    dynamic_route::NodeId source = 0, target = 899;
    dynamic_route::DStarLite dstar(g, source, target);
    dstar.compute_shortest_path();

    constexpr int N = 1000;
    int passed = run_stress(g, dstar, source, target, N, 200, "D*Lite");

    assert(passed == N);
    std::cout << "PASSED (" << passed << "/" << N << " steps verified)\n";
}

void test16_rr_500_perturbations_grid20() {
    std::cout << "[Test 16] Ramalingam-Reps — 500 perturbations on 20x20 grid (400 nodes)... ";
    std::cout.flush();

    auto g = dynamic_route::GraphIO::make_grid(20, 20, 1.0);
    assert(g.num_nodes() == 400);

    dynamic_route::NodeId source = 0, target = 399;
    dynamic_route::RamalingamReps rr(g, source);
    rr.initialize();

    constexpr int N = 500;
    int passed = run_rr_stress(g, rr, source, target, N, 300);

    assert(passed == N);
    std::cout << "PASSED (" << passed << "/" << N << " steps verified)\n";
}

void test17_lpa_star_rgg_path_walk_500() {
    std::cout << "[Test 17] LPA* — 500 perturbations on RGG 200 nodes + path walk validation... ";
    std::cout.flush();

    auto g = dynamic_route::GraphIO::make_random_geometric(200, 0.18, 42);
    assert(g.num_nodes() == 200);

    {
        auto init = dynamic_route::Dijkstra::find_shortest_path(g, 0, 199);
        if (!init.path_found)
            g = dynamic_route::GraphIO::make_random_geometric(200, 0.25, 99);
    }

    dynamic_route::NodeId source = 0, target = 199;
    dynamic_route::LifelongPlanningAStar lpa(g, source, target);
    lpa.compute_shortest_path();

    dynamic_route::EventSimulator sim(400);
    int passed = 0;
    constexpr int N = 500;

    for (int step = 0; step < N; ++step) {
        auto event = sim.generate_random_event(g);
        lpa.on_edge_changed(event.from, event.to, event.new_weight);

        auto inc = lpa.compute_shortest_path();
        auto ref = dynamic_route::Dijkstra::find_shortest_path(g, source, target);

        bool cost_ok = (inc.path_found == ref.path_found);
        if (cost_ok && inc.path_found)
            cost_ok = (std::abs(inc.total_cost - ref.total_cost) < 1e-6);

        bool walk_ok = true;
        if (inc.path_found && !inc.path.empty()) {
            auto vr = dynamic_route::PathValidator::validate_walk(g, inc.path, inc.total_cost);
            walk_ok = vr.ok && vr.path_is_valid_walk;
        }

        if (cost_ok && walk_ok) ++passed;
    }

    assert(passed == N);
    std::cout << "PASSED (" << passed << "/" << N << " steps with walk validation)\n";
}

void test18_three_way_agreement_grid50() {
    std::cout << "[Test 18] Three-way agreement (LPA* == D*Lite == RR == Dijkstra)"
              << " on 50x50 grid, 200 steps... ";
    std::cout.flush();

    auto g_lpa  = dynamic_route::GraphIO::make_grid(50, 50, 1.0);
    auto g_dstr = dynamic_route::GraphIO::make_grid(50, 50, 1.0);
    auto g_rr   = dynamic_route::GraphIO::make_grid(50, 50, 1.0);
    assert(g_lpa.num_nodes() == 2500);

    dynamic_route::NodeId source = 0, target = 2499;

    dynamic_route::LifelongPlanningAStar lpa(g_lpa, source, target);
    lpa.compute_shortest_path();

    dynamic_route::DStarLite dstar(g_dstr, source, target);
    dstar.compute_shortest_path();

    dynamic_route::RamalingamReps rr(g_rr, source);
    rr.initialize();

    constexpr int N    = 200;
    constexpr std::uint32_t SEED = 500;
    int passed = 0;

    {
        dynamic_route::EventSimulator sim_lpa(SEED);
        dynamic_route::EventSimulator sim_dstr(SEED);
        dynamic_route::EventSimulator sim_rr(SEED);

        for (int step = 0; step < N; ++step) {
            auto ev_lpa  = sim_lpa.generate_random_event(g_lpa);
            auto ev_dstr = sim_dstr.generate_random_event(g_dstr);
            auto ev_rr   = sim_rr.generate_random_event(g_rr);

            lpa.on_edge_changed(ev_lpa.from, ev_lpa.to, ev_lpa.new_weight);
            dstar.on_edge_changed(ev_dstr.from, ev_dstr.to, ev_dstr.new_weight);

            if (ev_rr.new_weight >= ev_rr.old_weight)
                rr.on_edge_increase(ev_rr.from, ev_rr.to, ev_rr.old_weight, ev_rr.new_weight);
            else
                rr.on_edge_decrease(ev_rr.from, ev_rr.to, ev_rr.old_weight, ev_rr.new_weight);

            auto lpa_res  = lpa.compute_shortest_path();
            auto dstr_res = dstar.compute_shortest_path();
            double rr_dist = rr.get_distance(target);

            auto ref = dynamic_route::Dijkstra::find_shortest_path(g_lpa, source, target);

            bool lpa_ok = (lpa_res.path_found == ref.path_found);
            if (lpa_ok && lpa_res.path_found)
                lpa_ok = (std::abs(lpa_res.total_cost - ref.total_cost) < 1e-6);

            bool dstr_ok = (dstr_res.path_found == ref.path_found);
            if (dstr_ok && dstr_res.path_found)
                dstr_ok = (std::abs(dstr_res.total_cost - ref.total_cost) < 1e-6);

            bool rr_ok = ref.path_found
                ? (std::abs(rr_dist - ref.total_cost) < 1e-6)
                : std::isinf(rr_dist);

            if (lpa_ok && dstr_ok && rr_ok) {
                ++passed;
            } else {
                std::cerr << "\n  [FAIL step " << step
                          << "] ref=" << ref.total_cost
                          << " lpa=" << lpa_res.total_cost
                          << " dstar=" << dstr_res.total_cost
                          << " rr=" << rr_dist << "\n";
            }
        }
    }

    assert(passed == N);
    std::cout << "PASSED (" << passed << "/" << N
              << " steps — all 3 algorithms agree with Dijkstra)\n";
}

void test19_rr_locality_ratio_sparse300() {
    std::cout << "[Test 19] RR locality ratio < 15% on sparse random graph (300 nodes)... ";
    std::cout.flush();

    auto g = dynamic_route::GraphIO::make_random_sparse(300, 2400, 1.0, 10.0, 77);
    assert(g.num_nodes() == 300);

    {
        auto init = dynamic_route::Dijkstra::find_shortest_path(g, 0, 299);
        if (!init.path_found)
            g = dynamic_route::GraphIO::make_random_sparse(300, 4000, 1.0, 10.0, 88);
    }

    dynamic_route::RamalingamReps rr(g, 0);
    rr.initialize();

    constexpr int N = 300;
    std::vector<std::size_t> invalidated_counts;
    invalidated_counts.reserve(static_cast<std::size_t>(N));

    int passed = run_rr_stress(g, rr, 0, 299, N, 600, &invalidated_counts);

    assert(passed == N);

    double sum_inv = 0.0;
    for (auto c : invalidated_counts) sum_inv += static_cast<double>(c);
    double avg_inv  = sum_inv / static_cast<double>(N);
    double locality = avg_inv / static_cast<double>(g.num_nodes());

    assert(locality < 0.15);

    std::cout << "PASSED (" << passed << "/" << N << " correct, "
              << "avg invalidated=" << avg_inv
              << " / " << g.num_nodes()
              << " nodes = " << (locality * 100.0) << "%)\n";
}

// ===========================================================================
// PHASE 2, PART 2 TESTS  (20–22) — Explainability & Diagnostics Engine
// ===========================================================================

void test20_causal_reporter_lpa_entries() {
    std::cout << "[Test 20] CausalReporter<LPA*> — entry correctness over 50 steps... ";
    std::cout.flush();

    auto g = dynamic_route::GraphIO::make_grid(10, 10, 1.0);
    dynamic_route::NodeId src = 0, tgt = 99;

    dynamic_route::LifelongPlanningAStar lpa(g, src, tgt);
    lpa.compute_shortest_path();

    dynamic_route::AuditLogger logger;
    dynamic_route::CausalReporter<dynamic_route::LifelongPlanningAStar> reporter(
        lpa, g, src, tgt, logger, "LPA*", /*check_dijkstra=*/true
    );

    dynamic_route::EventSimulator sim(1001);
    constexpr int N = 50;

    for (int i = 0; i < N; ++i) {
        auto ev = sim.generate_random_event(g);
        reporter.on_edge_changed(ev.from, ev.to, ev.new_weight);
        reporter.compute_shortest_path();
    }

    assert(logger.size() == static_cast<std::size_t>(N));

    for (std::size_t i = 0; i < logger.size(); ++i) {
        const auto& e = logger[i];
        assert(e.algorithm == "LPA*");
        assert(e.step == static_cast<int>(i));
        assert(e.correct);
        assert(e.cost_delta < 1e-6);
        assert(e.edge_change.from < g.num_nodes());
        assert(e.edge_change.to   < g.num_nodes());
        if (e.path_found) {
            assert(!e.path.empty());
            assert(e.path.front() == src);
            assert(e.path.back()  == tgt);
        }
        assert(e.repair_time_us >= 0.0);
    }

    std::cout << "PASSED (" << N << "/" << N << " entries correct, all fields valid)\n";
}

void test21_ndjson_roundtrip() {
    std::cout << "[Test 21] NDJSON round-trip (save -> load -> verify) over 30 steps... ";
    std::cout.flush();

    auto g = dynamic_route::GraphIO::make_grid(8, 8, 1.0);
    dynamic_route::NodeId src = 0, tgt = 63;

    dynamic_route::DStarLite dstar(g, src, tgt);
    dstar.compute_shortest_path();

    dynamic_route::AuditLogger logger_orig;
    dynamic_route::CausalReporter<dynamic_route::DStarLite> reporter(
        dstar, g, src, tgt, logger_orig, "D*Lite", true
    );

    dynamic_route::EventSimulator sim(2002);
    constexpr int N = 30;

    for (int i = 0; i < N; ++i) {
        auto ev = sim.generate_random_event(g);
        reporter.on_edge_changed(ev.from, ev.to, ev.new_weight);
        reporter.compute_shortest_path();
    }

    const std::string tmp_path = "/tmp/dynamic_route_audit_test.ndjson";
    bool saved = logger_orig.save_ndjson(tmp_path);
    assert(saved);
    (void)saved;

    auto logger_loaded = dynamic_route::AuditLogger::load_ndjson(tmp_path);
    assert(logger_loaded.size() == logger_orig.size());

    for (std::size_t i = 0; i < logger_orig.size(); ++i) {
        const auto& orig   = logger_orig[i];
        const auto& loaded = logger_loaded[i];
        (void)loaded;

        assert(loaded.step      == orig.step);
        assert(loaded.algorithm == orig.algorithm);
        assert(loaded.edge_change.from == orig.edge_change.from);
        assert(loaded.edge_change.to   == orig.edge_change.to);
        assert(loaded.edge_change.is_blockage == orig.edge_change.is_blockage);
        assert(std::abs(loaded.edge_change.old_weight - orig.edge_change.old_weight) < 1e-9
               || (std::isinf(orig.edge_change.old_weight) && std::isinf(loaded.edge_change.old_weight)));
        assert(std::abs(loaded.edge_change.new_weight - orig.edge_change.new_weight) < 1e-9
               || (std::isinf(orig.edge_change.new_weight) && std::isinf(loaded.edge_change.new_weight)));
        assert(loaded.path_found == orig.path_found);
        assert(loaded.correct    == orig.correct);
        assert(std::abs(loaded.cost_delta - orig.cost_delta) < 1e-9);
        if (orig.path_found) {
            assert(std::abs(loaded.final_cost - orig.final_cost) < 1e-9);
            assert(loaded.path.size() == orig.path.size());
            for (std::size_t j = 0; j < orig.path.size(); ++j)
                assert(loaded.path[j] == orig.path[j]);
        }
    }

    std::remove(tmp_path.c_str());
    std::cout << "PASSED (" << N << " entries round-tripped without data loss)\n";
}

void test22_aggregate_stats_and_rr_reporter() {
    std::cout << "[Test 22] Aggregate stats (correct_steps, repair_us, RR locality) over 100 steps... ";
    std::cout.flush();

    auto g = dynamic_route::GraphIO::make_grid(15, 15, 1.0);
    dynamic_route::NodeId src = 0, tgt = 224;

    dynamic_route::RamalingamReps rr(g, src);
    rr.initialize();

    dynamic_route::AuditLogger logger;
    dynamic_route::RRCausalReporter rr_rep(rr, g, src, tgt, logger, /*check_dijkstra=*/true);

    dynamic_route::EventSimulator sim(3003);
    constexpr int N = 100;

    for (int i = 0; i < N; ++i) {
        auto ev = sim.generate_random_event(g);
        double old_w = g.get_edge_weight(ev.from, ev.to).value_or(1.0);
        rr_rep.record_step(ev.from, ev.to, old_w, ev.new_weight);
    }

    assert(logger.size() == static_cast<std::size_t>(N));

    auto stats = logger.compute_stats(g.num_nodes());
    assert(stats.total_steps   == static_cast<std::size_t>(N));
    assert(stats.correct_steps == static_cast<std::size_t>(N));
    assert(stats.avg_repair_us >= 0.0);
    assert(stats.max_repair_us >= stats.avg_repair_us);

    double locality = stats.avg_nodes_invalidated / static_cast<double>(g.num_nodes());
    assert(locality < 0.5);

    for (std::size_t i = 0; i < logger.size(); ++i) {
        assert(logger[i].repair_time_us >= 0.0);
        assert(logger[i].algorithm == "RR");
        assert(logger[i].correct);
    }

    std::cout << "PASSED (all " << N << " correct, avg_repair="
              << stats.avg_repair_us << "us, locality="
              << (locality * 100.0) << "%)\n";
}

// ===========================================================================
// PHASE 2, PART 3 TESTS  (23–26) — Advanced Traffic Simulator
// ===========================================================================

void test23_congestion_wave() {
    std::cout << "[Test 23] CongestionWave — 6-segment corridor on 15x15 grid... ";
    std::cout.flush();

    auto g = dynamic_route::GraphIO::make_grid(15, 15, 1.0);
    dynamic_route::NodeId src = 0, tgt = 224;

    std::vector<dynamic_route::NodeId> corridor = {0, 1, 2, 3, 4, 5, 6};
    dynamic_route::CongestionWave wave(g, corridor, dynamic_route::INF_WEIGHT);

    assert(!wave.finished());
    assert(wave.remaining() == 6);

    dynamic_route::LifelongPlanningAStar lpa(g, src, tgt);
    lpa.compute_shortest_path();

    int step = 0;
    while (!wave.finished()) {
        auto ev = wave.step();
        assert(ev.is_blockage);
        lpa.on_edge_changed(ev.from, ev.to, ev.new_weight);
        auto res = lpa.compute_shortest_path();
        check_against_dijkstra(g, src, tgt, res.path_found, res.total_cost, step);
        ++step;
    }

    assert(wave.finished());
    assert(wave.remaining() == 0);
    assert(step == 6);

    wave.reset();
    auto ref_reset = dynamic_route::Dijkstra::find_shortest_path(g, src, tgt);
    assert(ref_reset.path_found);
    assert(std::abs(ref_reset.total_cost - 28.0) < 1e-6);

    std::cout << "PASSED (6/6 wave steps correct, reset verified)\n";
}

void test24_disaster_zone() {
    std::cout << "[Test 24] DisasterZone — apply + clear on 20x20 grid with D* Lite... ";
    std::cout.flush();

    auto g = dynamic_route::GraphIO::make_grid(20, 20, 1.0);
    dynamic_route::NodeId src = 0, tgt = 399;

    dynamic_route::DStarLite dstar(g, src, tgt);
    dstar.compute_shortest_path();

    dynamic_route::DisasterZone zone(g, 8.0, 8.0, 12.0, 12.0, 8.0, "CentreFlood");

    {
        auto ref = dynamic_route::Dijkstra::find_shortest_path(g, src, tgt);
        assert(ref.path_found);
        assert(std::abs(ref.total_cost - 38.0) < 1e-6);
    }

    auto apply_events = zone.apply();
    assert(zone.is_active());
    assert(zone.affected_edge_count() > 0);
    std::cout << "\n    [Zone applied: " << zone.affected_edge_count() << " edges degraded 8x] ";

    for (const auto& ev : apply_events)
        dstar.on_edge_changed(ev.from, ev.to, ev.new_weight);
    auto res_during = dstar.compute_shortest_path();

    auto ref_during = dynamic_route::Dijkstra::find_shortest_path(g, src, tgt);
    assert(res_during.path_found == ref_during.path_found);
    if (res_during.path_found) {
        assert(std::abs(res_during.total_cost - ref_during.total_cost) < 1e-6);
        assert(res_during.total_cost >= 38.0);
    }

    auto clear_events = zone.clear();
    assert(!zone.is_active());

    for (const auto& ev : clear_events)
        dstar.on_edge_changed(ev.from, ev.to, ev.new_weight);
    auto res_after = dstar.compute_shortest_path();

    auto ref_after = dynamic_route::Dijkstra::find_shortest_path(g, src, tgt);
    assert(res_after.path_found == ref_after.path_found);
    if (res_after.path_found) {
        assert(std::abs(res_after.total_cost - ref_after.total_cost) < 1e-6);
        assert(std::abs(res_after.total_cost - 38.0) < 1e-6);
    }

    std::cout << "PASSED (apply: cost=" << res_during.total_cost
              << ", clear: cost=" << res_after.total_cost << ")\n";
}

void test25_recovery_event_ttl() {
    std::cout << "[Test 25] RecoveryEvent — TTL=5, 20 perturbations, auto-restore... ";
    std::cout.flush();

    auto g = dynamic_route::GraphIO::make_grid(12, 12, 1.0);
    dynamic_route::NodeId src = 0, tgt = 143;

    dynamic_route::LifelongPlanningAStar lpa(g, src, tgt);
    lpa.compute_shortest_path();

    constexpr int TTL = 5;
    dynamic_route::RecoveryEvent recovery(g, TTL);

    dynamic_route::EventSimulator sim(4004);
    constexpr int N_PERTURB = 20;
    int step = 0;

    for (int i = 0; i < N_PERTURB; ++i) {
        auto ev = sim.generate_random_event(g);
        recovery.record_perturbation(ev);

        lpa.on_edge_changed(ev.from, ev.to, ev.new_weight);
        auto res = lpa.compute_shortest_path();
        check_against_dijkstra(g, src, tgt, res.path_found, res.total_cost, step);
        ++step;
    }

    int total_restored = 0;
    for (int tick = 0; tick < TTL + 2; ++tick) {
        auto restored = recovery.tick();
        for (const auto& ev : restored) {
            lpa.on_edge_changed(ev.from, ev.to, ev.new_weight);
            total_restored++;
        }
        if (!restored.empty()) {
            auto res = lpa.compute_shortest_path();
            check_against_dijkstra(g, src, tgt, res.path_found, res.total_cost, step);
            ++step;
        }
    }

    assert(recovery.pending_count() == 0);
    assert(total_restored > 0);

    auto ref_final = dynamic_route::Dijkstra::find_shortest_path(g, src, tgt);
    assert(ref_final.path_found);

    std::cout << "PASSED (" << total_restored << " edges auto-restored after TTL="
              << TTL << " ticks)\n";
}

void test26_scenario_runner_composite() {
    std::cout << "[Test 26] ScenarioRunner — 30-tick composite scenario (wave + zone + random + recovery)... ";
    std::cout.flush();

    auto g_lpa   = dynamic_route::GraphIO::make_grid(15, 15, 1.0);
    auto g_dstar  = dynamic_route::GraphIO::make_grid(15, 15, 1.0);
    auto g_rr     = dynamic_route::GraphIO::make_grid(15, 15, 1.0);

    dynamic_route::NodeId src = 0, tgt = 224;

    dynamic_route::LifelongPlanningAStar lpa(g_lpa, src, tgt);
    lpa.compute_shortest_path();

    dynamic_route::DStarLite dstar(g_dstar, src, tgt);
    dstar.compute_shortest_path();

    dynamic_route::RamalingamReps rr(g_rr, src);
    rr.initialize();

    std::vector<dynamic_route::NodeId> corridor = {0,1,2,3,4,5,6,7};
    dynamic_route::CongestionWave wave(g_lpa, corridor, dynamic_route::INF_WEIGHT);

    dynamic_route::DisasterZone zone(g_lpa, 5.0, 5.0, 9.0, 9.0, 4.0, "CentreZone");
    dynamic_route::RecoveryEvent recovery(g_lpa, 6);

    dynamic_route::AuditLogger log_lpa, log_dstar, log_rr;
    dynamic_route::ScenarioRunner runner(g_lpa, src, tgt);

    runner.add_congestion_wave(wave,   2, 2);
    runner.add_disaster_zone  (zone,   8, 20);
    runner.add_recovery_tracker(recovery);
    runner.add_random_perturbations(10, 0, 29, 5005);
    runner.register_lpa(lpa, log_lpa, "LPA*");

    auto report = runner.run(30);

    assert(!report.algo_stats.empty());
    for (const auto& as : report.algo_stats) {
        (void)as;
        assert(as.steps_verified == as.steps_total);
    }

    assert(report.total_ticks  == 30);
    assert(report.total_events >  0);

    // Independent 3-way agreement on a fresh grid
    {
        auto gx = dynamic_route::GraphIO::make_grid(10, 10, 1.0);
        dynamic_route::NodeId s2 = 0, t2 = 99;

        dynamic_route::LifelongPlanningAStar lpa2(gx, s2, t2);
        lpa2.compute_shortest_path();
        dynamic_route::DStarLite dstar2(gx, s2, t2);
        dstar2.compute_shortest_path();
        dynamic_route::RamalingamReps rr2(gx, s2);
        rr2.initialize();

        dynamic_route::EventSimulator sim2(6006);
        for (int i = 0; i < 50; ++i) {
            auto ev = sim2.generate_random_event(gx);

            lpa2.on_edge_changed(ev.from, ev.to, ev.new_weight);
            dstar2.on_edge_changed(ev.from, ev.to, ev.new_weight);

            double old_w = gx.get_edge_weight(ev.from, ev.to).value_or(ev.new_weight);
            if (ev.new_weight >= old_w)
                rr2.on_edge_increase(ev.from, ev.to, old_w, ev.new_weight);
            else
                rr2.on_edge_decrease(ev.from, ev.to, old_w, ev.new_weight);

            auto lpa_res   = lpa2.compute_shortest_path();
            auto dstar_res = dstar2.compute_shortest_path();
            auto ref       = dynamic_route::Dijkstra::find_shortest_path(gx, s2, t2);
            double rr_d    = rr2.get_distance(t2);
            (void)rr_d;

            assert((lpa_res.path_found == ref.path_found) &&
                   (!ref.path_found || std::abs(lpa_res.total_cost - ref.total_cost) < 1e-6));
            assert((dstar_res.path_found == ref.path_found) &&
                   (!ref.path_found || std::abs(dstar_res.total_cost - ref.total_cost) < 1e-6));
            assert(ref.path_found
                   ? (std::abs(rr_d - ref.total_cost) < 1e-6)
                   : std::isinf(rr_d));
        }
    }

    std::cout << "PASSED (runner: " << report.total_events << " events, "
              << report.total_recoveries << " recoveries; "
              << "3-way agreement verified over 50 steps)\n";
}

// ===========================================================================
// main
// ===========================================================================

int main() {
    std::cout << "============================================================\n";
    std::cout << "  DynamicRoute - Complete Test Suite (Phase 0 + 1 + 2)\n";
    std::cout << "============================================================\n\n";

    // ---- Phase 0 ----
    std::cout << "--- Phase 0: Baseline Correctness ---\n";
    test_graph_construction();
    test_static_algorithms();
    test_lpa_star_initial();
    test_d_star_lite_initial();
    test_incremental_invariants_under_perturbations();

    // ---- Phase 1 ----
    std::cout << "\n--- Phase 1: Core DSA Engine ---\n";
    test_shortest_path_tree();
    test_ramalingam_reps_edge_increase();
    test_ramalingam_reps_edge_decrease();
    test_graph_io_generators();
    test_path_validator();
    test_metadata_and_o1_lookup();
    test_graph_io_file_roundtrip();
    test_ramalingam_reps_path_reconstruction();

    // ---- Phase 2, Part 1 ----
    std::cout << "\n--- Phase 2, Part 1: Large-Scale Invariant Stress Testing ---\n";
    test14_lpa_star_1000_perturbations_grid30();
    test15_d_star_lite_1000_perturbations_grid30();
    test16_rr_500_perturbations_grid20();
    test17_lpa_star_rgg_path_walk_500();
    test18_three_way_agreement_grid50();
    test19_rr_locality_ratio_sparse300();

    // ---- Phase 2, Part 2 ----
    std::cout << "\n--- Phase 2, Part 2: Explainability & Diagnostics Engine ---\n";
    test20_causal_reporter_lpa_entries();
    test21_ndjson_roundtrip();
    test22_aggregate_stats_and_rr_reporter();

    // ---- Phase 2, Part 3 ----
    std::cout << "\n--- Phase 2, Part 3: Advanced Traffic Simulator ---\n";
    test23_congestion_wave();
    test24_disaster_zone();
    test25_recovery_event_ttl();
    test26_scenario_runner_composite();

    std::cout << "\n============================================================\n";
    std::cout << " ALL 26 TESTS PASSED!\n";
    std::cout << "============================================================\n";
    return 0;
}
