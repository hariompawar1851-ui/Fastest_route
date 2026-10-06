#pragma once

#include "graph/node.hpp"
#include "graph/edge.hpp"

#include <vector>
#include <unordered_map>
#include <optional>
#include <cmath>
#include <cstdint>
#include <utility>

namespace dynamic_route {

class Graph {
public:
    Graph() = default;
    explicit Graph(std::size_t num_nodes);

    NodeId add_node(double x = 0.0, double y = 0.0,
                    const std::string& label = "",
                    const std::string& node_type = "intersection");
    bool has_node(NodeId id) const noexcept;
    const Node& get_node(NodeId id) const;

    void add_directed_edge(NodeId from, NodeId to, double weight,
                           const std::string& road_type = "standard",
                           double capacity = 100.0,
                           double max_speed = 50.0);
    void add_undirected_edge(NodeId u, NodeId v, double weight,
                             const std::string& road_type = "standard",
                             double capacity = 100.0,
                             double max_speed = 50.0);

    bool update_edge_weight(NodeId from, NodeId to, double new_weight);
    bool set_edge_blocked(NodeId from, NodeId to, bool blocked);

    const std::vector<Edge>& get_outgoing_edges(NodeId u) const;
    const std::vector<Edge>& get_incoming_edges(NodeId u) const;

    std::optional<double> get_edge_weight(NodeId from, NodeId to) const;
    const Edge* get_edge(NodeId from, NodeId to) const;

    std::size_t num_nodes() const noexcept;
    std::size_t num_edges() const noexcept;

    double euclidean_distance(NodeId u, NodeId v) const;

private:
    static constexpr std::uint64_t edge_key(NodeId u, NodeId v) noexcept {
        return (static_cast<std::uint64_t>(u) << 32) | static_cast<std::uint64_t>(v);
    }

    std::vector<Node> nodes_;
    std::vector<std::vector<Edge>> forward_adj_;  // Outgoing edges from u
    std::vector<std::vector<Edge>> backward_adj_; // Incoming edges to u
    // O(1) Edge lookup: maps (from, to) -> pair(forward_idx, backward_idx)
    std::unordered_map<std::uint64_t, std::pair<std::size_t, std::size_t>> edge_indices_;
    std::size_t edge_count_{0};
};

} // namespace dynamic_route
