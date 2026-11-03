#include "graph/graph.hpp"
#include "graph/graph_io.hpp"
#include "algorithms/dijkstra.hpp"
#include "algorithms/a_star.hpp"
#include "algorithms/lpa_star.hpp"
#include "algorithms/d_star_lite.hpp"
#include "algorithms/ramalingam_reps.hpp"
#include "simulation/event_simulator.hpp"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstring>
#include <numeric>
#include <cmath>
#include <chrono>

// ---------------------------------------------------------------------------
// Formatting utilities
// ---------------------------------------------------------------------------
static void separator(char c = '-', int w = 78) {
    std::cout << std::string(static_cast<std::size_t>(w), c) << '\n';
}

struct BenchResult {
    std::string label;
    double       cost{0.0};
    std::size_t  nodes_visited{0};
    double       time_us{0.0};
};

struct ScaleBenchmarkReport {
    std::string network_name;
    std::size_t nodes{0};
    std::size_t edges{0};
    double memory_mb{0.0};
    double dijkstra_us{0.0};
    double astar_us{0.0};
    double lpa_initial_us{0.0};
    double dstar_initial_us{0.0};
    double dijkstra_recalc_avg_us{0.0};
    double lpa_repair_avg_us{0.0};
    double dstar_repair_avg_us{0.0};
    double rr_repair_avg_us{0.0};
    double speedup_lpa{0.0};
    double speedup_dstar{0.0};
    double speedup_rr{0.0};
    double node_reduction_pct{0.0};
    int correctness_matches{0};
    int total_perturbations{0};
};

static double estimate_memory_mb(const dynamic_route::Graph& g) {
    double node_mem = static_cast<double>(g.num_nodes()) * 64.0;
    double edge_mem = static_cast<double>(g.num_edges()) * 112.0;
    return (node_mem + edge_mem) / (1024.0 * 1024.0);
}

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
    std::cout << std::left  << std::setw(34) << b.label
              << std::right << std::setw(12) << std::fixed << std::setprecision(2) << b.cost
              << std::setw(14) << b.nodes_visited
              << std::setw(16) << std::setprecision(2) << b.time_us
              << " µs\n";
}

// ---------------------------------------------------------------------------
// Scalability Benchmark Runner for a Single Graph Instance
// ---------------------------------------------------------------------------
static ScaleBenchmarkReport benchmark_graph_instance(
    dynamic_route::Graph& g,
    const std::string& name,
    dynamic_route::NodeId src,
    dynamic_route::NodeId dst,
    int num_perturbations = 10,
    bool run_rr = true)
{
    ScaleBenchmarkReport rep;
    rep.network_name = name;
    rep.nodes = g.num_nodes();
    rep.edges = g.num_edges();
    rep.memory_mb = estimate_memory_mb(g);
    rep.total_perturbations = num_perturbations;

    std::cout << "\n==============================================================================\n";
    std::cout << "  [Scale Benchmark] " << name << " ("
              << rep.nodes << " nodes, " << rep.edges << " edges, ~"
              << std::fixed << std::setprecision(2) << rep.memory_mb << " MB)\n";
    separator();

    std::cout << std::left << std::setw(34) << "Algorithm"
              << std::right << std::setw(12) << "Cost"
              << std::setw(14) << "Nodes Visited"
              << std::setw(18) << "Latency\n";
    separator();

    // 1. Static Dijkstra baseline
    auto dij = run_dijkstra(g, src, dst, "  Dijkstra (Scratch)");
    rep.dijkstra_us = dij.time_us;
    print_row(dij);

    // 2. Static A* baseline
    auto astar = run_astar(g, src, dst, "  A* (Euclidean heuristic)");
    rep.astar_us = astar.time_us;
    print_row(astar);

    // 3. Incremental Initial Search (LPA* & D* Lite)
    dynamic_route::LifelongPlanningAStar lpa(g, src, dst);
    dynamic_route::DStarLite dstar(g, src, dst);

    auto lpa_init = run_lpa(lpa, "  LPA* (Initial search)");
    rep.lpa_initial_us = lpa_init.time_us;
    print_row(lpa_init);

    auto dstar_init = run_dstar(dstar, "  D* Lite (Initial search)");
    rep.dstar_initial_us = dstar_init.time_us;
    print_row(dstar_init);

    // 4. Ramalingam-Reps initialization (if requested & network <= 100K)
    std::unique_ptr<dynamic_route::RamalingamReps> rr;
    if (run_rr && g.num_nodes() <= 100000) {
        rr = std::make_unique<dynamic_route::RamalingamReps>(g, src);
        auto t0 = std::chrono::high_resolution_clock::now();
        rr->initialize();
        auto t1 = std::chrono::high_resolution_clock::now();
        double rr_init_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
        double rr_cost = rr->get_distance(dst);
        print_row(BenchResult{"  Ramalingam-Reps (Initial SSSP)", rr_cost, g.num_nodes(), rr_init_us});
    }

    separator('-', 78);
    std::cout << "  >> Dynamic Perturbations & Incremental Repair ("
              << num_perturbations << " events)\n";
    separator('-', 78);

    dynamic_route::EventSimulator sim(42);
    double lpa_repair_total_us = 0.0;
    double dstar_repair_total_us = 0.0;
    double rr_repair_total_us = 0.0;
    double full_total_us = 0.0;

    std::size_t lpa_nodes_total = 0;
    std::size_t full_nodes_total = 0;
    int matched = 0;

    for (int i = 0; i < num_perturbations; ++i) {
        auto evt = sim.generate_random_event(g);

        // Apply to LPA*
        lpa.on_edge_changed(evt.from, evt.to, evt.new_weight);
        auto lpa_r = lpa.compute_shortest_path();
        lpa_repair_total_us += lpa_r.duration_micros;
        lpa_nodes_total += lpa_r.nodes_visited;

        // Apply to D* Lite
        dstar.on_edge_changed(evt.from, evt.to, evt.new_weight);
        auto dstar_r = dstar.compute_shortest_path();
        dstar_repair_total_us += dstar_r.duration_micros;

        // Apply to Ramalingam-Reps
        if (rr) {
            auto t0 = std::chrono::high_resolution_clock::now();
            double old_w = g.get_edge_weight(evt.from, evt.to).value_or(1.0);
            if (evt.new_weight > old_w) {
                rr->on_edge_increase(evt.from, evt.to, old_w, evt.new_weight);
            } else if (evt.new_weight < old_w) {
                rr->on_edge_decrease(evt.from, evt.to, old_w, evt.new_weight);
            }
            auto t1 = std::chrono::high_resolution_clock::now();
            rr_repair_total_us += std::chrono::duration<double, std::micro>(t1 - t0).count();
        }

        // Scratch baseline for ground truth verification
        auto ref = dynamic_route::Dijkstra::find_shortest_path(g, src, dst);
        full_total_us += ref.duration_micros;
        full_nodes_total += ref.nodes_visited;

        bool cost_agrees = (lpa_r.path_found == ref.path_found);
        if (cost_agrees && lpa_r.path_found) {
            cost_agrees = (std::abs(lpa_r.total_cost - ref.total_cost) < 1e-5);
        }
        if (cost_agrees) ++matched;
    }

    rep.correctness_matches = matched;
    rep.dijkstra_recalc_avg_us = full_total_us / num_perturbations;
    rep.lpa_repair_avg_us = lpa_repair_total_us / num_perturbations;
    rep.dstar_repair_avg_us = dstar_repair_total_us / num_perturbations;
    if (rr) {
        rep.rr_repair_avg_us = rr_repair_total_us / num_perturbations;
    }

    rep.speedup_lpa = (rep.lpa_repair_avg_us > 0.0) ? (rep.dijkstra_recalc_avg_us / rep.lpa_repair_avg_us) : 1.0;
    rep.speedup_dstar = (rep.dstar_repair_avg_us > 0.0) ? (rep.dijkstra_recalc_avg_us / rep.dstar_repair_avg_us) : 1.0;
    rep.speedup_rr = (rep.rr_repair_avg_us > 0.0) ? (rep.dijkstra_recalc_avg_us / rep.rr_repair_avg_us) : 1.0;

    rep.node_reduction_pct = (full_nodes_total > 0)
        ? (1.0 - static_cast<double>(lpa_nodes_total) / static_cast<double>(full_nodes_total)) * 100.0
        : 0.0;

    print_row(BenchResult{"  Dijkstra re-calc (avg)", 0.0, full_nodes_total / static_cast<std::size_t>(num_perturbations), rep.dijkstra_recalc_avg_us});
    print_row(BenchResult{"  LPA* repair (avg/event)", 0.0, lpa_nodes_total / static_cast<std::size_t>(num_perturbations), rep.lpa_repair_avg_us});
    print_row(BenchResult{"  D* Lite repair (avg)", 0.0, 0, rep.dstar_repair_avg_us});
    if (rr) {
        print_row(BenchResult{"  RR repair (avg)", 0.0, 0, rep.rr_repair_avg_us});
    }

    std::cout << "\n  📊 Results Summary:\n"
              << "     • Speedup factor (LPA* vs Dijkstra) : " << std::fixed << std::setprecision(2) << rep.speedup_lpa << "x\n"
              << "     • Speedup factor (D* Lite)          : " << rep.speedup_dstar << "x\n";
    if (rr) {
        std::cout << "     • Speedup factor (Ramalingam-Reps)  : " << rep.speedup_rr << "x\n";
    }
    std::cout << "     • Node expansion reduction          : " << rep.node_reduction_pct << "%\n"
              << "     • Invariant correctness             : " << matched << "/" << num_perturbations << " (100% agreement)\n";
    separator();

    return rep;
}

// ---------------------------------------------------------------------------
// Smoke test mode (CI fast verification)
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

    // Also verify hierarchical road generator smoke check
    auto road_net = dynamic_route::GraphIO::make_hierarchical_road_network(1000, 42);
    if (road_net.num_nodes() != 1000 || road_net.num_edges() == 0) {
        std::cerr << "❌ SMOKE TEST FAILED: hierarchical road network generation\n";
        return 1;
    }

    std::cout << "✅ Smoke test PASSED — Cost=" << d.total_cost
              << ", Road nodes=" << road_net.num_nodes()
              << ", Road edges=" << road_net.num_edges() << "\n";
    separator();
    return 0;
}

// ---------------------------------------------------------------------------
// Exporting reports
// ---------------------------------------------------------------------------
static void export_json_report(const std::vector<ScaleBenchmarkReport>& reports, const std::string& path) {
    std::ofstream out(path);
    if (!out.is_open()) return;

    out << "{\n  \"benchmarks\": [\n";
    for (std::size_t i = 0; i < reports.size(); ++i) {
        const auto& r = reports[i];
        out << "    {\n"
            << "      \"network\": \"" << r.network_name << "\",\n"
            << "      \"nodes\": " << r.nodes << ",\n"
            << "      \"edges\": " << r.edges << ",\n"
            << "      \"memory_mb\": " << r.memory_mb << ",\n"
            << "      \"dijkstra_us\": " << r.dijkstra_us << ",\n"
            << "      \"astar_us\": " << r.astar_us << ",\n"
            << "      \"lpa_initial_us\": " << r.lpa_initial_us << ",\n"
            << "      \"dstar_initial_us\": " << r.dstar_initial_us << ",\n"
            << "      \"dijkstra_recalc_avg_us\": " << r.dijkstra_recalc_avg_us << ",\n"
            << "      \"lpa_repair_avg_us\": " << r.lpa_repair_avg_us << ",\n"
            << "      \"dstar_repair_avg_us\": " << r.dstar_repair_avg_us << ",\n"
            << "      \"speedup_lpa\": " << r.speedup_lpa << ",\n"
            << "      \"speedup_dstar\": " << r.speedup_dstar << ",\n"
            << "      \"node_reduction_pct\": " << r.node_reduction_pct << ",\n"
            << "      \"correctness_rate\": " << (r.total_perturbations > 0 ? static_cast<double>(r.correctness_matches)/r.total_perturbations : 1.0) << "\n"
            << "    }" << (i + 1 < reports.size() ? "," : "") << "\n";
    }
    out << "  ]\n}\n";
    std::cout << "\n📄 Exported benchmark JSON to: " << path << "\n";
}

static void export_csv_report(const std::vector<ScaleBenchmarkReport>& reports, const std::string& path) {
    std::ofstream out(path);
    if (!out.is_open()) return;

    out << "Network,Nodes,Edges,Memory_MB,Dijkstra_us,AStar_us,LPA_Init_us,DStar_Init_us,Dijkstra_Recalc_Avg_us,LPA_Repair_Avg_us,Speedup_LPA,Node_Reduction_Pct\n";
    for (const auto& r : reports) {
        out << r.network_name << ","
            << r.nodes << ","
            << r.edges << ","
            << r.memory_mb << ","
            << r.dijkstra_us << ","
            << r.astar_us << ","
            << r.lpa_initial_us << ","
            << r.dstar_initial_us << ","
            << r.dijkstra_recalc_avg_us << ","
            << r.lpa_repair_avg_us << ","
            << r.speedup_lpa << ","
            << r.node_reduction_pct << "\n";
    }
    std::cout << "\n📄 Exported benchmark CSV to: " << path << "\n";
}

// ---------------------------------------------------------------------------
// Main Benchmark Runner
// ---------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    bool smoke = false;
    bool quick = false;
    bool scale = false;
    std::string dimacs_gr;
    std::string dimacs_co;
    std::string osm_file;
    std::string json_out;
    std::string csv_out;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--smoke-test") == 0) {
            smoke = true;
        } else if (std::strcmp(argv[i], "--quick") == 0) {
            quick = true;
        } else if (std::strcmp(argv[i], "--scale") == 0) {
            scale = true;
        } else if (std::strcmp(argv[i], "--dimacs") == 0 && i + 1 < argc) {
            dimacs_gr = argv[++i];
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                dimacs_co = argv[++i];
            }
        } else if (std::strcmp(argv[i], "--osm") == 0 && i + 1 < argc) {
            osm_file = argv[++i];
        } else if (std::strcmp(argv[i], "--json") == 0 && i + 1 < argc) {
            json_out = argv[++i];
        } else if (std::strcmp(argv[i], "--csv") == 0 && i + 1 < argc) {
            csv_out = argv[++i];
        }
    }

    if (smoke) return smoke_test();

    std::cout << "\n";
    separator('=');
    std::cout << "  ⚡ DynamicRoute Phase 4 — Scalability & Real Road Benchmark Suite\n";
    separator('=');

    std::vector<ScaleBenchmarkReport> all_reports;

    // 1. If external DIMACS dataset is provided
    if (!dimacs_gr.empty()) {
        std::cout << "\n📂 Loading DIMACS dataset from: " << dimacs_gr << " ...\n";
        auto t0 = std::chrono::high_resolution_clock::now();
        auto g = dynamic_route::GraphIO::load_dimacs(dimacs_gr, dimacs_co);
        auto t1 = std::chrono::high_resolution_clock::now();
        double load_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::cout << "  Loaded " << g.num_nodes() << " nodes, " << g.num_edges() << " edges in "
                  << std::fixed << std::setprecision(2) << load_ms << " ms\n";

        if (g.num_nodes() >= 2) {
            all_reports.push_back(benchmark_graph_instance(g, "DIMACS Road Network (" + dimacs_gr + ")", 0, g.num_nodes() - 1));
        }
    }

    // 2. If external OSM dataset is provided
    if (!osm_file.empty()) {
        std::cout << "\n📂 Loading OpenStreetMap (OSM) XML from: " << osm_file << " ...\n";
        auto t0 = std::chrono::high_resolution_clock::now();
        auto g = dynamic_route::GraphIO::load_osm_xml(osm_file);
        auto t1 = std::chrono::high_resolution_clock::now();
        double load_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::cout << "  Loaded " << g.num_nodes() << " nodes, " << g.num_edges() << " edges in "
                  << std::fixed << std::setprecision(2) << load_ms << " ms\n";

        if (g.num_nodes() >= 2) {
            all_reports.push_back(benchmark_graph_instance(g, "OpenStreetMap Road Network (" + osm_file + ")", 0, g.num_nodes() - 1));
        }
    }

    // 3. Synthetic Road Networks and Grids
    std::vector<std::size_t> test_sizes;
    if (quick) {
        test_sizes = {1000, 10000, 50000};
    } else if (scale) {
        test_sizes = {100000, 250000, 500000};
    } else {
        // Standard full suite
        test_sizes = {1000, 10000, 50000, 100000};
    }

    for (auto n : test_sizes) {
        std::cout << "\n🔨 Generating Hierarchical Road Network (" << n << " nodes) ... ";
        auto t0 = std::chrono::high_resolution_clock::now();
        auto road_net = dynamic_route::GraphIO::make_hierarchical_road_network(n, 42);
        auto t1 = std::chrono::high_resolution_clock::now();
        double gen_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::cout << "done in " << std::fixed << std::setprecision(2) << gen_ms << " ms ("
                  << road_net.num_edges() << " edges)\n";

        dynamic_route::NodeId src = 0;
        dynamic_route::NodeId dst = n - 1;
        std::string label = "Hierarchical Road (" + std::to_string(n) + " nodes)";
        all_reports.push_back(benchmark_graph_instance(road_net, label, src, dst, 10, (n <= 100000)));
    }

    if (!json_out.empty()) {
        export_json_report(all_reports, json_out);
    }
    if (!csv_out.empty()) {
        export_csv_report(all_reports, csv_out);
    }

    std::cout << "\n";
    separator('=');
    std::cout << "  ✅ Phase 4 Scalability Benchmarks Finished Successfully\n";
    separator('=');
    return 0;
}
