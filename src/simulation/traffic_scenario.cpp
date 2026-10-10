#include "simulation/traffic_scenario.hpp"
#include "graph/graph.hpp"

#include <cmath>
#include <stdexcept>
#include <algorithm>

namespace dynamic_route {

// =============================================================================
// CongestionWave
// =============================================================================

CongestionWave::CongestionWave(Graph&              graph,
                               std::vector<NodeId> corridor,
                               double              congestion_factor)
    : graph_(graph),
      corridor_(std::move(corridor)),
      congestion_factor_(congestion_factor)
{
    // Save original weights for all corridor edges so reset() works
    original_weights_.reserve(corridor_.size() > 0 ? corridor_.size() - 1 : 0);
    for (std::size_t i = 0; i + 1 < corridor_.size(); ++i) {
        NodeId u = corridor_[i];
        NodeId v = corridor_[i + 1];
        auto w = graph_.get_edge_weight(u, v);
        original_weights_.push_back(w.value_or(1.0));
    }
}

PerturbationEvent CongestionWave::step() {
    if (finished()) return PerturbationEvent{};

    NodeId u = corridor_[current_step_];
    NodeId v = corridor_[current_step_ + 1];
    double old_w = original_weights_[current_step_];

    bool is_blockage = std::isinf(congestion_factor_);

    if (is_blockage) {
        graph_.set_edge_blocked(u, v, true);
    } else {
        double new_w = old_w * congestion_factor_;
        graph_.update_edge_weight(u, v, new_w);
    }

    double new_w = is_blockage ? INF_WEIGHT
                               : old_w * congestion_factor_;

    ++current_step_;
    return PerturbationEvent{u, v, old_w, new_w, is_blockage};
}

bool CongestionWave::finished() const noexcept {
    return corridor_.size() < 2 ||
           current_step_ >= corridor_.size() - 1;
}

std::size_t CongestionWave::remaining() const noexcept {
    if (corridor_.size() < 2) return 0;
    std::size_t total = corridor_.size() - 1;
    return (current_step_ >= total) ? 0 : total - current_step_;
}

void CongestionWave::reset() {
    // Restore all applied segments
    for (std::size_t i = 0; i < current_step_ && i < original_weights_.size(); ++i) {
        NodeId u = corridor_[i];
        NodeId v = corridor_[i + 1];
        graph_.set_edge_blocked(u, v, false);
        graph_.update_edge_weight(u, v, original_weights_[i]);
    }
    current_step_ = 0;
}

// =============================================================================
// DisasterZone
// =============================================================================

DisasterZone::DisasterZone(Graph&       graph,
                           double       x_min, double y_min,
                           double       x_max, double y_max,
                           double       multiplier,
                           std::string  name)
    : graph_(graph),
      x_min_(x_min), y_min_(y_min),
      x_max_(x_max), y_max_(y_max),
      multiplier_(multiplier),
      name_(std::move(name))
{}

bool DisasterZone::node_in_zone(NodeId u) const {
    const auto& node = graph_.get_node(u);
    return node.coords.x >= x_min_ && node.coords.x <= x_max_ &&
           node.coords.y >= y_min_ && node.coords.y <= y_max_;
}

std::vector<PerturbationEvent> DisasterZone::apply() {
    if (active_) return {};  // already applied

    saved_weights_.clear();
    std::vector<PerturbationEvent> events;

    const std::size_t n = graph_.num_nodes();
    for (NodeId u = 0; u < n; ++u) {
        if (!node_in_zone(u)) continue;
        for (const auto& edge : graph_.get_outgoing_edges(u)) {
            NodeId v = edge.to;
            if (!node_in_zone(v)) continue;
            if (edge.is_blocked) continue;

            double old_w = edge.weight;
            double new_w;
            bool   block = std::isinf(multiplier_);

            if (block) {
                new_w = INF_WEIGHT;
                graph_.set_edge_blocked(u, v, true);
            } else {
                new_w = old_w * multiplier_;
                graph_.update_edge_weight(u, v, new_w);
            }

            saved_weights_.push_back({u, v, old_w});
            events.push_back(PerturbationEvent{u, v, old_w, new_w, block});
        }
    }

    active_ = true;
    return events;
}

std::vector<PerturbationEvent> DisasterZone::clear() {
    if (!active_) return {};

    std::vector<PerturbationEvent> events;

    for (const auto& saved : saved_weights_) {
        // Current effective weight
        auto cur_opt = graph_.get_edge_weight(saved.from, saved.to);
        double cur_w = cur_opt.value_or(INF_WEIGHT);

        // Unblock if it was blocked
        graph_.set_edge_blocked(saved.from, saved.to, false);
        graph_.update_edge_weight(saved.from, saved.to, saved.original_weight);

        events.push_back(PerturbationEvent{
            saved.from, saved.to,
            cur_w, saved.original_weight,
            false
        });
    }

    saved_weights_.clear();
    active_ = false;
    return events;
}

// =============================================================================
// RecoveryEvent
// =============================================================================

RecoveryEvent::RecoveryEvent(Graph& graph, int ttl)
    : graph_(graph), ttl_(ttl)
{}

void RecoveryEvent::record_perturbation(const PerturbationEvent& event) {
    // Don't re-record if this exact edge already has a pending recovery
    // (a second perturbation on the same edge updates the recovery target)
    for (auto& p : pending_) {
        if (p.from == event.from && p.to == event.to) {
            p.recover_at_tick = current_tick_ + ttl_;
            return;
        }
    }

    pending_.push_back({
        event.from,
        event.to,
        event.old_weight,
        current_tick_ + ttl_,
        event.is_blockage
    });
}

std::vector<PerturbationEvent> RecoveryEvent::tick() {
    ++current_tick_;
    std::vector<PerturbationEvent> restored;

    auto it = pending_.begin();
    while (it != pending_.end()) {
        if (it->recover_at_tick <= current_tick_) {
            // Restore this edge
            double cur_w = graph_.get_edge_weight(it->from, it->to)
                               .value_or(INF_WEIGHT);

            graph_.set_edge_blocked(it->from, it->to, false);
            graph_.update_edge_weight(it->from, it->to, it->original_weight);

            restored.push_back(PerturbationEvent{
                it->from, it->to,
                cur_w, it->original_weight,
                false
            });
            it = pending_.erase(it);
        } else {
            ++it;
        }
    }

    return restored;
}

// =============================================================================
// generate_random_corridor
// =============================================================================

std::vector<NodeId> generate_random_corridor(const Graph& graph,
                                              std::size_t  length,
                                              std::uint32_t seed)
{
    if (graph.num_nodes() < 2 || length < 2) return {};

    std::mt19937 rng(seed);
    std::uniform_int_distribution<NodeId> node_dist(0,
        static_cast<NodeId>(graph.num_nodes() - 1));

    constexpr int MAX_ATTEMPTS = 200;

    for (int attempt = 0; attempt < MAX_ATTEMPTS; ++attempt) {
        std::vector<NodeId> corridor;
        NodeId cur = node_dist(rng);
        corridor.push_back(cur);

        bool stuck = false;
        while (corridor.size() < length) {
            const auto& out = graph.get_outgoing_edges(cur);
            if (out.empty()) { stuck = true; break; }

            // Pick a random outgoing edge that isn't already in the corridor
            std::vector<const Edge*> candidates;
            for (const auto& e : out) {
                bool already_in = false;
                for (NodeId n : corridor) {
                    if (n == e.to) { already_in = true; break; }
                }
                if (!already_in && !e.is_blocked) candidates.push_back(&e);
            }

            if (candidates.empty()) { stuck = true; break; }

            std::uniform_int_distribution<std::size_t> pick(0, candidates.size() - 1);
            cur = candidates[pick(rng)]->to;
            corridor.push_back(cur);
        }

        if (!stuck && corridor.size() == length) return corridor;
    }

    return {};  // Could not build a corridor of the requested length
}

} // namespace dynamic_route
