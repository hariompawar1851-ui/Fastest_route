#pragma once

#include "graph/node.hpp"
#include <limits>

namespace dynamic_route {

constexpr double INF_WEIGHT = std::numeric_limits<double>::infinity();

struct Edge {
    NodeId from{0};
    NodeId to{0};
    double weight{1.0};
    bool is_blocked{false};
    std::string road_type{"standard"}; // e.g. "motorway", "primary", "secondary", "residential"
    double capacity{100.0};             // vehicles/minute or flow capacity
    double max_speed{50.0};             // km/h

    double effective_weight() const noexcept {
        return is_blocked ? INF_WEIGHT : weight;
    }
};

} // namespace dynamic_route
