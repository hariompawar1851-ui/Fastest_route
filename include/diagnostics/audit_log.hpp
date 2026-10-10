#pragma once

// =============================================================================
// DynamicRoute — Diagnostics: AuditLog
// =============================================================================
// Records a structured causal audit entry for every edge perturbation step.
// Each entry captures:
//   - The edge that changed (from, to, old weight, new weight)
//   - The algorithm that processed it
//   - How many vertices were invalidated and re-relaxed
//   - Wall-clock repair time in microseconds
//   - The resulting shortest path and its cost
//
// Entries can be serialised to newline-delimited JSON (one JSON object per line)
// for offline analysis, visualisation, or regression replay.
// =============================================================================

#include "graph/graph.hpp"
#include "algorithms/dijkstra.hpp"  // SearchResult

#include <string>
#include <vector>
#include <cstdint>
#include <ostream>
#include <fstream>
#include <chrono>

namespace dynamic_route {

// ---------------------------------------------------------------------------
// AuditEntry — one record per perturbation step
// ---------------------------------------------------------------------------

struct EdgeChange {
    NodeId from{0};
    NodeId to{0};
    double old_weight{0.0};
    double new_weight{0.0};
    bool   is_blockage{false};   ///< true when new_weight == INF_WEIGHT
};

struct AuditEntry {
    int         step{0};
    std::string algorithm;       ///< "LPA*", "D*Lite", "RR", "Dijkstra", etc.

    EdgeChange  edge_change;

    // Algorithm metrics
    std::size_t nodes_invalidated{0}; ///< vertices cut from the SPT (RR only; 0 for LPA*/D*)
    std::size_t nodes_relaxed{0};     ///< vertices re-relaxed during repair
    std::size_t nodes_expanded{0};    ///< priority-queue pops during the query
    double      repair_time_us{0.0};  ///< wall-clock time for on_edge_changed() call

    // Result
    bool               path_found{false};
    double             final_cost{0.0};
    std::vector<NodeId> path;          ///< reconstructed path (source → target)

    // Correctness flag (set by AuditLogger after Dijkstra comparison)
    bool   correct{true};
    double cost_delta{0.0};    ///< |incremental_cost − dijkstra_cost|
};

// ---------------------------------------------------------------------------
// AuditLogger — accumulates entries and serialises them
// ---------------------------------------------------------------------------

class AuditLogger {
public:
    AuditLogger() = default;

    /// Append an entry (moves it in).
    void record(AuditEntry entry);

    /// Number of entries recorded so far.
    std::size_t size() const noexcept { return entries_.size(); }

    /// Access a specific entry (0-indexed).
    const AuditEntry& operator[](std::size_t i) const { return entries_[i]; }

    /// Iterate entries
    auto begin() const { return entries_.begin(); }
    auto end()   const { return entries_.end(); }

    // ------------------------------------------------------------------
    // Serialisation
    // ------------------------------------------------------------------

    /**
     * @brief Write all entries as newline-delimited JSON to the given stream.
     *
     * Format: one JSON object per line (NDJSON / JSON Lines).
     * INF weights are written as the string "INF" (JSON has no Infinity literal).
     */
    void write_ndjson(std::ostream& out) const;

    /**
     * @brief Save NDJSON to a file path.
     * @returns true on success.
     */
    bool save_ndjson(const std::string& filepath) const;

    /**
     * @brief Load NDJSON from a file and return a new AuditLogger.
     *
     * Only the scalar fields are parsed (path is reconstructed as a list of
     * integers). Returns an empty logger on parse error.
     */
    static AuditLogger load_ndjson(const std::string& filepath);

    // ------------------------------------------------------------------
    // Aggregate statistics
    // ------------------------------------------------------------------

    struct Stats {
        std::size_t total_steps{0};
        std::size_t correct_steps{0};
        double      avg_repair_us{0.0};
        double      max_repair_us{0.0};
        double      avg_nodes_relaxed{0.0};
        double      avg_nodes_invalidated{0.0};
        double      avg_locality_ratio{0.0}; ///< avg_invalidated / graph_nodes
        std::size_t graph_nodes{0};
    };

    Stats compute_stats(std::size_t graph_node_count = 0) const;

    /// Clear all entries.
    void clear() { entries_.clear(); }

private:
    std::vector<AuditEntry> entries_;

    /// Escape a string for JSON output.
    static std::string json_escape(const std::string& s);

    /// Serialise a single entry to a JSON object string.
    std::string entry_to_json(const AuditEntry& e) const;

    /// Parse a single JSON line into an AuditEntry.
    static bool parse_json_line(const std::string& line, AuditEntry& out);
};

// ---------------------------------------------------------------------------
// Timer — RAII nanosecond timer used by CausalReporter
// ---------------------------------------------------------------------------

class ScopedTimer {
public:
    explicit ScopedTimer(double& out_us)
        : out_us_(out_us),
          start_(std::chrono::high_resolution_clock::now()) {}

    ~ScopedTimer() {
        auto end = std::chrono::high_resolution_clock::now();
        out_us_ = static_cast<double>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(end - start_).count()
        ) / 1000.0;
    }

private:
    double& out_us_;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_;
};

} // namespace dynamic_route
