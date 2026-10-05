#include "graph/graph_io.hpp"

#include <random>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <algorithm>

namespace dynamic_route {

// ---------------------------------------------------------------------------
// Grid Graph Generator
// ---------------------------------------------------------------------------

Graph GraphIO::make_grid(std::size_t rows, std::size_t cols, double spacing) {
    Graph graph;

    // Add all nodes with Cartesian coordinates
    for (std::size_t r = 0; r < rows; ++r) {
        for (std::size_t c = 0; c < cols; ++c) {
            graph.add_node(
                static_cast<double>(c) * spacing,
                static_cast<double>(r) * spacing,
                "N_" + std::to_string(r) + "_" + std::to_string(c)
            );
        }
    }

    auto get_id = [cols](std::size_t r, std::size_t c) -> NodeId {
        return r * cols + c;
    };

    // Add horizontal and vertical edges (bidirectional)
    for (std::size_t r = 0; r < rows; ++r) {
        for (std::size_t c = 0; c < cols; ++c) {
            NodeId u = get_id(r, c);
            if (c + 1 < cols) {
                graph.add_undirected_edge(u, get_id(r, c + 1), spacing);
            }
            if (r + 1 < rows) {
                graph.add_undirected_edge(u, get_id(r + 1, c), spacing);
            }
        }
    }

    return graph;
}

// ---------------------------------------------------------------------------
// Random Geometric Graph Generator
// ---------------------------------------------------------------------------

Graph GraphIO::make_random_geometric(std::size_t n, double radius,
                                     std::uint32_t seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> coord_dist(0.0, 1.0);

    Graph graph;

    // Place nodes uniformly in [0,1]²
    for (std::size_t i = 0; i < n; ++i) {
        double x = coord_dist(rng);
        double y = coord_dist(rng);
        graph.add_node(x, y, "V" + std::to_string(i));
    }

    // Add bidirectional edges between pairs within the connection radius.
    // Edge weight = Euclidean distance (preserves the metric structure
    // needed by admissible heuristics in A* / LPA* / D* Lite).
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            double d = graph.euclidean_distance(
                static_cast<NodeId>(i), static_cast<NodeId>(j));
            if (d <= radius) {
                graph.add_undirected_edge(
                    static_cast<NodeId>(i),
                    static_cast<NodeId>(j),
                    d);
            }
        }
    }

    return graph;
}

// ---------------------------------------------------------------------------
// Random Sparse Graph Generator  (Erdős–Rényi G(n, m) model)
// ---------------------------------------------------------------------------

Graph GraphIO::make_random_sparse(std::size_t n, std::size_t m,
                                  double min_w, double max_w,
                                  std::uint32_t seed) {
    std::mt19937 rng(seed);
    std::uniform_int_distribution<NodeId> node_dist(0,
        static_cast<NodeId>(n - 1));
    std::uniform_real_distribution<double> weight_dist(min_w, max_w);

    Graph graph;
    for (std::size_t i = 0; i < n; ++i) {
        graph.add_node(0.0, 0.0, "V" + std::to_string(i));
    }

    std::size_t added = 0;
    const std::size_t max_attempts = m * 10;
    for (std::size_t attempt = 0; attempt < max_attempts && added < m; ++attempt) {
        NodeId u = node_dist(rng);
        NodeId v = node_dist(rng);
        if (u == v) continue;
        // Skip if edge already exists (simple check via weight query)
        if (graph.get_edge_weight(u, v).has_value()) continue;

        graph.add_directed_edge(u, v, weight_dist(rng));
        ++added;
    }

    return graph;
}

// ---------------------------------------------------------------------------
// File I/O — load_edge_list
// ---------------------------------------------------------------------------

Graph GraphIO::load_edge_list(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("GraphIO: cannot open file: " + filepath);
    }

    // First pass: determine number of nodes needed
    std::vector<std::tuple<NodeId, NodeId, double>> edges;
    NodeId max_node = 0;
    std::string line;

    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        // Handle optional DIMACS header: "p sp <n> <m>"
        if (line[0] == 'p') continue;
        // DIMACS arc line: "a from to weight"
        if (line[0] == 'a') {
            std::istringstream iss(line.substr(1));
            NodeId u, v; double w;
            if (iss >> u >> v >> w) {
                // DIMACS uses 1-indexed nodes
                edges.emplace_back(u - 1, v - 1, w);
                max_node = std::max(max_node, std::max(u - 1, v - 1));
            }
            continue;
        }
        // Plain edge-list: "u v weight"
        std::istringstream iss(line);
        NodeId u, v; double w;
        if (iss >> u >> v >> w) {
            edges.emplace_back(u, v, w);
            max_node = std::max(max_node, std::max(u, v));
        }
    }

    Graph graph(max_node + 1);
    for (auto& [u, v, w] : edges) {
        graph.add_directed_edge(u, v, w);
    }
    return graph;
}

// ---------------------------------------------------------------------------
// File I/O — save_edge_list
// ---------------------------------------------------------------------------

bool GraphIO::save_edge_list(const Graph& graph, const std::string& filepath) {
    std::ofstream file(filepath);
    if (!file.is_open()) return false;

    file << "# DynamicRoute edge-list export\n";
    file << "# p sp " << graph.num_nodes() << " " << graph.num_edges() << "\n";

    for (NodeId u = 0; u < graph.num_nodes(); ++u) {
        for (const auto& edge : graph.get_outgoing_edges(u)) {
            if (!edge.is_blocked) {
                file << u << " " << edge.to << " " << edge.weight << "\n";
            }
        }
    }
    return true;
}

} // namespace dynamic_route
