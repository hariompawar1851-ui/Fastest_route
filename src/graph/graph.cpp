#include "graph/graph.hpp"
#include <stdexcept>
#include <cmath>

namespace dynamic_route {

Graph::Graph(std::size_t num_nodes) {
    nodes_.reserve(num_nodes);
    forward_adj_.resize(num_nodes);
    backward_adj_.resize(num_nodes);
    for (std::size_t i = 0; i < num_nodes; ++i) {
        nodes_.push_back(Node{i, Coordinates{0.0, 0.0}, ""});
    }
}

NodeId Graph::add_node(double x, double y, const std::string& label) {
    NodeId id = nodes_.size();
    nodes_.push_back(Node{id, Coordinates{x, y}, label});
    forward_adj_.emplace_back();
    backward_adj_.emplace_back();
    return id;
}

bool Graph::has_node(NodeId id) const noexcept {
    return id < nodes_.size();
}

const Node& Graph::get_node(NodeId id) const {
    if (!has_node(id)) {
        throw std::out_of_range("Node ID out of range");
    }
    return nodes_[id];
}

void Graph::add_directed_edge(NodeId from, NodeId to, double weight) {
    if (!has_node(from) || !has_node(to)) {
        throw std::out_of_range("Invalid endpoint for edge");
    }
    forward_adj_[from].push_back(Edge{from, to, weight, false});
    backward_adj_[to].push_back(Edge{from, to, weight, false});
    ++edge_count_;
}

void Graph::add_undirected_edge(NodeId u, NodeId v, double weight) {
    add_directed_edge(u, v, weight);
    add_directed_edge(v, u, weight);
}

bool Graph::update_edge_weight(NodeId from, NodeId to, double new_weight) {
    if (!has_node(from) || !has_node(to)) return false;

    bool found = false;
    for (auto& edge : forward_adj_[from]) {
        if (edge.to == to) {
            edge.weight = new_weight;
            edge.is_blocked = (new_weight >= INF_WEIGHT);
            found = true;
            break;
        }
    }
    for (auto& edge : backward_adj_[to]) {
        if (edge.from == from) {
            edge.weight = new_weight;
            edge.is_blocked = (new_weight >= INF_WEIGHT);
            break;
        }
    }
    return found;
}

bool Graph::set_edge_blocked(NodeId from, NodeId to, bool blocked) {
    if (!has_node(from) || !has_node(to)) return false;

    bool found = false;
    for (auto& edge : forward_adj_[from]) {
        if (edge.to == to) {
            edge.is_blocked = blocked;
            found = true;
            break;
        }
    }
    for (auto& edge : backward_adj_[to]) {
        if (edge.from == from) {
            edge.is_blocked = blocked;
            break;
        }
    }
    return found;
}

const std::vector<Edge>& Graph::get_outgoing_edges(NodeId u) const {
    if (!has_node(u)) throw std::out_of_range("Node ID out of range");
    return forward_adj_[u];
}

const std::vector<Edge>& Graph::get_incoming_edges(NodeId u) const {
    if (!has_node(u)) throw std::out_of_range("Node ID out of range");
    return backward_adj_[u];
}

std::optional<double> Graph::get_edge_weight(NodeId from, NodeId to) const {
    if (!has_node(from) || !has_node(to)) return std::nullopt;
    for (const auto& edge : forward_adj_[from]) {
        if (edge.to == to) {
            return edge.effective_weight();
        }
    }
    return std::nullopt;
}

std::size_t Graph::num_nodes() const noexcept {
    return nodes_.size();
}

std::size_t Graph::num_edges() const noexcept {
    return edge_count_;
}

double Graph::euclidean_distance(NodeId u, NodeId v) const {
    if (!has_node(u) || !has_node(v)) return 0.0;
    const auto& c1 = nodes_[u].coords;
    const auto& c2 = nodes_[v].coords;
    return std::hypot(c1.x - c2.x, c1.y - c2.y);
}

} // namespace dynamic_route
