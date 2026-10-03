#include "graph/graph.hpp"
#include "algorithms/dijkstra.hpp"
#include "algorithms/a_star.hpp"
#include "algorithms/lpa_star.hpp"
#include "algorithms/d_star_lite.hpp"
#include "simulation/event_simulator.hpp"

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <cstring>
#include <numeric>

// ---------------------------------------------------------------------------
// Utility: print a separator line
// ---------------------------------------------------------------------------
static void separator(char c = '-', int w = 70) {
    std::cout << std::string(static_cast<std::size_t>(w), c) << '\n';
}

// ---------------------------------------------------------------------------
// Benchmark: single-query latency on a grid of size rows x cols
// ---------------------------------------------------------------------------
struct BenchResult {
    std::string label;
    double       cost{0.0};
    std::size_t  nodes_visited{0};
    double       time_us{0.0};
};

static BenchResult run_dijkstra(const dynamic_route::Graph& g,
                                dynamic_route::NodeId src,
                                dynamic_route::NodeId dst,
                                const std::string& tag)
{
    auto r = dynamic_route::Dijkstra::find_shortest_path(g, src, dst);
    return {tag, r.total_cost, r.nodes_visited, r.duration_micros};
}

static BenchResult run_astar(const dynamic_route::Graph& g,
                             dynamic_route::NodeId src,
                             dynamic_route::NodeId dst,
                             const std::string& tag)
{
    auto r = dynamic_route::AStar::find_shortest_path(g, src, dst);
    return {tag, r.total_cost, r.nodes_visited, r.duration_micros};
}

static BenchResult run_lpa(dynamic_route::LifelongPlanningAStar& lpa,
                           const std::string& tag)
{
    auto r = lpa.compute_shortest_path();
    return {tag, r.total_cost, r.nodes_visited, r.duration_micros};
}

static BenchResult run_dstar(dynamic_route::DStarLite& dstar,
                             const std::string& tag)
{
    auto r = dstar.compute_shortest_path();
    return {tag, r.total_cost, r.nodes_visited, r.duration_micros};
}

static void print_row(const BenchResult& b) {
    std::cout << std::left  << std::setw(32) << b.label
              << std::right << std::setw(10) << std::fixed << std::setprecision(2) << b.cost
              << std::setw(14) << b.nodes_visited
              << std::setw(16) << std::setprecision(3) << b.time_us
              << " µs\n";
}

// ---------------------------------------------------------------------------
// Smoke test: fast correctness + latency check (used in CI)
// ---------------------------------------------------------------------------
static int smoke_test() {
    std::cout << "\n🔬 Smoke Test Mode (CI validation)\n";
    separator();

    auto g = dynamic_route::EventSimulator::create_grid_graph(10, 10, 1.0);
    dynamic_route::NodeId src = 0;
    dynamic_route::NodeId dst = 99;

    auto d = dynamic_route::Dijkstra::find_shortest_path(g, src, dst);
    dynamic_route::LifelongPlanningAStar lpa(g, src, dst);
    auto lpa_res = lpa.compute_shortest_path();

    if (!d.path_found || !lpa_res.path_found) {
        std::cerr << "❌ SMOKE TEST FAILED: path not found\n";
        return 1;
    }
    if (std::abs(d.total_cost - lpa_res.total_cost) > 1e-6) {
        std::cerr << "❌ SMOKE TEST FAILED: cost mismatch (Dijkstra="
                  << d.total_cost << ", LPA*=" << lpa_res.total_cost << ")\n";
        return 1;
    }
    std::cout << "✅ Smoke test PASSED — Cost=" << d.total_cost << "\n";
    separator();
    return 0;
}

// ---------------------------------------------------------------------------
// Full benchmark suite
// ---------------------------------------------------------------------------
static int full_benchmark() {
    std::cout << "\n";
    separator('=');
    std::cout << "  ⚡ DynamicRoute — Performance Benchmark Suite\n";
    separator('=');

    const std::vector<std::pair<std::size_t, std::size_t>> grid_sizes = {
        {10, 10},   // 100 nodes
        {20, 20},   // 400 nodes
        {40, 40},   // 1 600 nodes
        {80, 80},   // 6 400 nodes
        {150, 150}, // 22 500 nodes
    };

    std::cout << "\n";
    std::cout << std::left << std::setw(32) << "Algorithm"
              << std::right << std::setw(10) << "Cost"
              << std::setw(14) << "Nodes Visited"
              << std::setw(17) << "Latency\n";
    separator();

    for (auto [rows, cols] : grid_sizes) {
        auto g = dynamic_route::EventSimulator::create_grid_graph(rows, cols, 1.0);
        dynamic_route::NodeId src = 0;
        dynamic_route::NodeId dst = rows * cols - 1;
        std::string tag = std::to_string(rows) + "x" + std::to_string(cols)
                          + " grid (" + std::to_string(rows * cols) + " nodes)";

        std::cout << "\n  Grid: " << tag << "\n";
        separator('-', 70);

        // --- Static algorithms ---
        print_row(run_dijkstra(g, src, dst, "  Dijkstra"));
        print_row(run_astar   (g, src, dst, "  A*"));

        // --- Incremental initial search ---
        dynamic_route::LifelongPlanningAStar lpa(g, src, dst);
        dynamic_route::DStarLite             dstar(g, src, dst);
        print_row(run_lpa  (lpa,   "  LPA* (initial)"));
        print_row(run_dstar(dstar, "  D* Lite (initial)"));

        // --- Dynamic perturbation + incremental repair ---
        dynamic_route::EventSimulator sim(42);
        constexpr int NUM_EVENTS = 5;
        double lpa_total_us   = 0.0;
        double full_total_us  = 0.0;
        std::size_t lpa_nodes = 0;
        std::size_t full_nodes = 0;
        int matched = 0;

        for (int i = 0; i < NUM_EVENTS; ++i) {
            auto evt = sim.generate_random_event(g);
            lpa.on_edge_changed(evt.from, evt.to, evt.new_weight);

            auto lpa_r  = lpa.compute_shortest_path();
            auto full_r = dynamic_route::Dijkstra::find_shortest_path(g, src, dst);

            lpa_total_us  += lpa_r.duration_micros;
            full_total_us += full_r.duration_micros;
            lpa_nodes     += lpa_r.nodes_visited;
            full_nodes    += full_r.nodes_visited;
            if (lpa_r.path_found == full_r.path_found &&
                (!lpa_r.path_found ||
                 std::abs(lpa_r.total_cost - full_r.total_cost) < 1e-6))
            {
                ++matched;
            }
        }

        BenchResult lpa_avg  {"  LPA* repair (avg/event)",
                               0.0, lpa_nodes  / NUM_EVENTS, lpa_total_us  / NUM_EVENTS};
        BenchResult full_avg {"  Dijkstra re-calc (avg)",
                               0.0, full_nodes / NUM_EVENTS, full_total_us / NUM_EVENTS};
        print_row(full_avg);
        print_row(lpa_avg);

        double speedup = (lpa_total_us > 0.0)
                       ? full_total_us / lpa_total_us
                       : 1.0;
        double node_red = (full_nodes > 0)
                        ? (1.0 - static_cast<double>(lpa_nodes) /
                                  static_cast<double>(full_nodes)) * 100.0
                        : 0.0;

        std::cout << "  → Speedup: "  << std::fixed << std::setprecision(2) << speedup << "x"
                  << "  |  Node expansion reduction: " << node_red << "%"
                  << "  |  Correctness: " << matched << "/" << NUM_EVENTS << "\n";
    }

    separator('=');
    std::cout << "\n✅ Benchmark complete.\n\n";
    return 0;
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    bool smoke = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--smoke-test") == 0) {
            smoke = true;
        }
    }

    if (smoke) return smoke_test();
    return full_benchmark();
}
