#pragma once

#include "graph/graph.hpp"
#include <string>
#include <cstdint>
#include <vector>

namespace dynamic_route {

/**
 * @brief Graph I/O and Synthetic Graph Generators (Phase 1)
 *
 * Provides:
 *   1. Grid graph generator — aligned rectangular road network.
 *   2. Random Geometric Graph (RGG) generator — nodes placed randomly in
 *      a 2D unit square; edges added between pairs within radius r.
 *      Used in scalability benchmarks.
 *   3. File-based loaders:
 *        - load_edge_list() : simple "u v weight" text format
 *        - save_edge_list() : write graph back to the same format
 *
 * The Random Geometric Graph model was chosen because it closely
 * mirrors the sparse, planar-like structure of real road networks:
 *   - Average degree ≈ constant as n grows (controlled by radius r).
 *   - Short-range connectivity mimics urban / suburban topology.
 *   - References:
 *       Penrose, M. (2003). Random Geometric Graphs. Oxford Univ. Press.
 */

class GraphIO {
public:
    // ------------------------------------------------------------------
    // Synthetic Graph Generators
    // ------------------------------------------------------------------

    /**
     * @brief Create an M×N grid road network.
     *
     * Each intersection (r, c) becomes NodeId = r*cols + c.
     * Horizontal and vertical edges are bidirectional with the given spacing.
     *
     * Complexity: O(rows * cols)
     *
     * @param rows     Number of grid rows
     * @param cols     Number of grid columns
     * @param spacing  Weight of each edge (e.g., road length in metres)
     */
    static Graph make_grid(std::size_t rows, std::size_t cols, double spacing = 1.0);

    /**
     * @brief Create a Random Geometric Graph (RGG).
     *
     * Nodes are placed uniformly at random in [0, 1]². A bidirectional
     * edge is added between every pair (u, v) whose Euclidean distance
     * is ≤ radius. Edge weight = Euclidean distance between u and v.
     *
     * Choosing radius ≈ sqrt(ln(n) / (π * n)) guarantees connectivity
     * with high probability (Gupta & Kumar, 2000).
     *
     * @param n       Number of nodes
     * @param radius  Connection radius in [0, 1] space
     * @param seed    RNG seed for reproducibility
     */
    static Graph make_random_geometric(std::size_t n, double radius,
                                       std::uint32_t seed = 42);

    /**
     * @brief Create a random sparse graph (Erdős–Rényi G(n, m) model).
     *
     * Generates exactly `m` directed edges between distinct random endpoints
     * with random weights in [min_w, max_w].
     *
     * @param n      Number of vertices
     * @param m      Number of directed edges to generate
     * @param min_w  Minimum edge weight
     * @param max_w  Maximum edge weight
     * @param seed   RNG seed
     */
    static Graph make_random_sparse(std::size_t n, std::size_t m,
                                    double min_w = 1.0, double max_w = 10.0,
                                    std::uint32_t seed = 42);

    // ------------------------------------------------------------------
    // File I/O  (simple edge-list format)
    // ------------------------------------------------------------------

    /**
     * @brief Load a graph from a plain-text edge-list file.
     *
     * File format (one edge per line):
     *   <from_id>  <to_id>  <weight>
     *   # Lines starting with '#' are treated as comments.
     *   # The first non-comment line may optionally be:
     *   #   p sp <num_nodes> <num_edges>
     *
     * Compatible with the DIMACS shortest-path challenge format.
     *
     * @param filepath  Path to the edge-list file
     * @returns         Loaded Graph object (may be empty on error)
     */
    static Graph load_edge_list(const std::string& filepath);

    /**
     * @brief Save a graph to a plain-text edge-list file.
     *
     * @param graph     Graph to serialise
     * @param filepath  Destination file path
     * @returns         true on success
     */
    static bool save_edge_list(const Graph& graph, const std::string& filepath);
};

} // namespace dynamic_route
