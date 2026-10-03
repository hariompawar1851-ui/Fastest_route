#pragma once

#include "graph/node.hpp"
#include "graph/edge.hpp"

#include <vector>
#include <unordered_map>
#include <optional>
#include <cmath>

namespace dynamic_route {

class Graph {
public:
    Graph() = default;
    explicit Graph(std::size_t num_nodes);

    NodeId add_node(double x = 0.0, double y = 0.0, const std::string& label = "");
    bool has_node(NodeId id) const noexcept;
    const Node& get_node(NodeId id) const;

    void add_directed_edge(NodeId from, NodeId to, double weight);
    void add_undirected_edge(NodeId u, NodeId v, double weight);

    bool update_edge_weight(NodeId from, NodeId to, double new_weight);
    bool set_edge_blocked(NodeId from, NodeId to, bool blocked);

    const std::vector<Edge>& get_outgoing_edges(NodeId u) const;
    const std::vector<Edge>& get_incoming_edges(NodeId u) const;

    std::optional<double> get_edge_weight(NodeId from, NodeId to) const;

    std::size_t num_nodes() const noexcept;
    std::size_t num_edges() const noexcept;

    double euclidean_distance(NodeId u, NodeId v) const;

private:
    std::vector<Node> nodes_;
    std::vector<std::vector<Edge>> forward_adj_;  // Outgoing edges from u
    std::vector<std::vector<Edge>> backward_adj_; // Incoming edges to u
    std::size_t edge_count_{0};
};

} // namespace dynamic_route
