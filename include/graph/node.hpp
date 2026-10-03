#pragma once

#include <cstddef>
#include <string>

namespace dynamic_route {

using NodeId = std::size_t;

struct Coordinates {
    double x{0.0};
    double y{0.0};
};

struct Node {
    NodeId id{0};
    Coordinates coords{0.0, 0.0};
    std::string label{""};

    bool operator==(const Node& other) const noexcept {
        return id == other.id;
    }
};

} // namespace dynamic_route
