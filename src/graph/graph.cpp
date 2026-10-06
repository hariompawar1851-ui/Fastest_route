#include "graph/graph.hpp"
#include <stdexcept>
#include <cmath>

namespace dynamic_route {

Graph::Graph(std::size_t num_nodes) {
    nodes_.reserve(num_nodes);
    forward_adj_.resize(num_nodes);
    backward_adj_.resize(num_nodes);
    for (std::size_t i = 0; i < num_nodes; ++i) {
        nodes_.push_back(Node{i, Coordinates{0.0, 0.0}, "", "intersection"});
    }
}

NodeId Graph::add_node(double x, double y, const std::string& label, const std::string& node_type) {
    NodeId id = nodes_.size();
    nodes_.push_back(Node{id, Coordinates{x, y}, label, node_type});
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

void Graph::add_directed_edge(NodeId from, NodeId to, double weight,
                               const std::string& road_type,
                               double capacity,
                               double max_speed) {
    if (!has_node(from) || !has_node(to)) {
        throw std::out_of_range("Invalid endpoint for edge");
    }
    auto key = edge_key(from, to);
    auto it = edge_indices_.find(key);
    if (it != edge_indices_.end()) {
        auto [f_idx, b_idx] = it->second;
        bool blocked = (weight >= INF_WEIGHT);
        forward_adj_[from][f_idx].weight = weight;
        forward_adj_[from][f_idx].is_blocked = blocked;
        forward_adj_[from][f_idx].road_type = road_type;
        forward_adj_[from][f_idx].capacity = capacity;
        forward_adj_[from][f_idx].max_speed = max_speed;

        backward_adj_[to][b_idx].weight = weight;
        backward_adj_[to][b_idx].is_blocked = blocked;
        backward_adj_[to][b_idx].road_type = road_type;
        backward_adj_[to][b_idx].capacity = capacity;
        backward_adj_[to][b_idx].max_speed = max_speed;
        return;
    }

    std::size_t f_idx = forward_adj_[from].size();
    forward_adj_[from].push_back(Edge{from, to, weight, (weight >= INF_WEIGHT), road_type, capacity, max_speed});

    std::size_t b_idx = backward_adj_[to].size();
    backward_adj_[to].push_back(Edge{from, to, weight, (weight >= INF_WEIGHT), road_type, capacity, max_speed});

    edge_indices_[key] = {f_idx, b_idx};
    ++edge_count_;
}

void Graph::add_undirected_edge(NodeId u, NodeId v, double weight,
                                 const std::string& road_type,
                                 double capacity,
                                 double max_speed) {
    add_directed_edge(u, v, weight, road_type, capacity, max_speed);
    add_directed_edge(v, u, weight, road_type, capacity, max_speed);
}

bool Graph::update_edge_weight(NodeId from, NodeId to, double new_weight) {
    if (!has_node(from) || !has_node(to)) return false;

    auto it = edge_indices_.find(edge_key(from, to));
    if (it == edge_indices_.end()) return false;

    auto [f_idx, b_idx] = it->second;
    bool blocked = (new_weight >= INF_WEIGHT);
    forward_adj_[from][f_idx].weight = new_weight;
    forward_adj_[from][f_idx].is_blocked = blocked;

    backward_adj_[to][b_idx].weight = new_weight;
    backward_adj_[to][b_idx].is_blocked = blocked;
    return true;
}

bool Graph::set_edge_blocked(NodeId from, NodeId to, bool blocked) {
    if (!has_node(from) || !has_node(to)) return false;

    auto it = edge_indices_.find(edge_key(from, to));
    if (it == edge_indices_.end()) return false;

    auto [f_idx, b_idx] = it->second;
    forward_adj_[from][f_idx].is_blocked = blocked;
    backward_adj_[to][b_idx].is_blocked = blocked;
    return true;
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

    auto it = edge_indices_.find(edge_key(from, to));
    if (it == edge_indices_.end()) return std::nullopt;

    return forward_adj_[from][it->second.first].effective_weight();
}

const Edge* Graph::get_edge(NodeId from, NodeId to) const {
    if (!has_node(from) || !has_node(to)) return nullptr;

    auto it = edge_indices_.find(edge_key(from, to));
    if (it == edge_indices_.end()) return nullptr;

    return &forward_adj_[from][it->second.first];
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
