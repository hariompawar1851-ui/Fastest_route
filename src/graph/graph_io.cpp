#include "graph/graph_io.hpp"

#include <random>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <algorithm>
#include <iomanip>
#include <unordered_map>

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

// ---------------------------------------------------------------------------
// Hierarchical Road Network Generator (Phase 4)
// ---------------------------------------------------------------------------

Graph GraphIO::make_hierarchical_road_network(std::size_t num_nodes, std::uint32_t seed) {
    if (num_nodes == 0) return Graph(0);
    if (num_nodes == 1) {
        Graph g;
        g.add_node(0.0, 0.0, "Hub_0", "intersection");
        return g;
    }

    std::mt19937 rng(seed);
    std::size_t cols = static_cast<std::size_t>(std::ceil(std::sqrt(static_cast<double>(num_nodes))));
    if (cols < 2) cols = 2;
    std::size_t rows = (num_nodes + cols - 1) / cols;

    Graph graph;
    std::uniform_real_distribution<double> jitter_dist(-5.0, 5.0);
    std::uniform_real_distribution<double> prob_dist(0.0, 1.0);

    const double base_spacing = 60.0; // 60 meters between street junctions

    // Add nodes with spatial coordinates
    for (std::size_t i = 0; i < num_nodes; ++i) {
        std::size_t r = i / cols;
        std::size_t c = i % cols;
        double x = static_cast<double>(c) * base_spacing + jitter_dist(rng);
        double y = static_cast<double>(r) * base_spacing + jitter_dist(rng);

        bool is_highway_hub = (r % 10 == 0 && c % 10 == 0);
        std::string ntype = is_highway_hub ? "highway_interchange" : "intersection";
        std::string label = "V" + std::to_string(i);
        graph.add_node(x, y, label, ntype);
    }

    auto get_node_id = [cols, num_nodes](std::size_t r, std::size_t c) -> std::optional<NodeId> {
        std::size_t idx = r * cols + c;
        if (idx < num_nodes) return idx;
        return std::nullopt;
    };

    // Add edges with hierarchy
    for (std::size_t r = 0; r < rows; ++r) {
        for (std::size_t c = 0; c < cols; ++c) {
            auto u_opt = get_node_id(r, c);
            if (!u_opt) continue;
            NodeId u = *u_opt;

            bool on_motorway_row = (r % 10 == 0);
            bool on_motorway_col = (c % 10 == 0);
            bool on_secondary_row = (r % 4 == 0);
            bool on_secondary_col = (c % 4 == 0);

            // Horizontal neighbor (r, c+1)
            auto v_h_opt = get_node_id(r, c + 1);
            if (v_h_opt && (c + 1 < cols)) {
                NodeId v_h = *v_h_opt;
                double dist = graph.euclidean_distance(u, v_h);
                if (dist < 1.0) dist = 1.0;

                std::string rtype = "residential";
                double max_speed = 35.0;
                double capacity = 60.0;
                double keep_prob = 0.85;

                if (on_motorway_row) {
                    rtype = "motorway";
                    max_speed = 110.0;
                    capacity = 250.0;
                    keep_prob = 1.0; // Highways are unbroken
                } else if (on_secondary_row) {
                    rtype = "secondary";
                    max_speed = 60.0;
                    capacity = 120.0;
                    keep_prob = 0.95;
                }

                if (prob_dist(rng) < keep_prob) {
                    bool oneway = (!on_motorway_row && prob_dist(rng) < 0.08);
                    if (oneway) {
                        graph.add_directed_edge(u, v_h, dist, rtype, capacity, max_speed);
                    } else {
                        graph.add_undirected_edge(u, v_h, dist, rtype, capacity, max_speed);
                    }
                }
            }

            // Vertical neighbor (r+1, c)
            auto v_v_opt = get_node_id(r + 1, c);
            if (v_v_opt) {
                NodeId v_v = *v_v_opt;
                double dist = graph.euclidean_distance(u, v_v);
                if (dist < 1.0) dist = 1.0;

                std::string rtype = "residential";
                double max_speed = 35.0;
                double capacity = 60.0;
                double keep_prob = 0.85;

                if (on_motorway_col) {
                    rtype = "motorway";
                    max_speed = 110.0;
                    capacity = 250.0;
                    keep_prob = 1.0;
                } else if (on_secondary_col) {
                    rtype = "secondary";
                    max_speed = 60.0;
                    capacity = 120.0;
                    keep_prob = 0.95;
                }

                if (prob_dist(rng) < keep_prob) {
                    bool oneway = (!on_motorway_col && prob_dist(rng) < 0.08);
                    if (oneway) {
                        graph.add_directed_edge(u, v_v, dist, rtype, capacity, max_speed);
                    } else {
                        graph.add_undirected_edge(u, v_v, dist, rtype, capacity, max_speed);
                    }
                }
            }
        }
    }

    return graph;
}

// ---------------------------------------------------------------------------
// DIMACS Challenge Format (Phase 4)
// ---------------------------------------------------------------------------

Graph GraphIO::load_dimacs(const std::string& gr_filepath, const std::string& co_filepath) {
    std::ifstream gr(gr_filepath);
    if (!gr.is_open()) {
        throw std::runtime_error("GraphIO: cannot open DIMACS arc file: " + gr_filepath);
    }

    std::vector<std::tuple<NodeId, NodeId, double>> edges;
    std::vector<std::tuple<NodeId, double, double>> coords_in_gr;
    NodeId max_node = 0;
    bool found_node = false;
    std::size_t header_nodes = 0;

    std::string line;
    while (std::getline(gr, line)) {
        if (line.empty() || line[0] == 'c') continue;

        if (line[0] == 'p') {
            std::istringstream iss(line);
            std::string p, type;
            std::size_t n = 0, m = 0;
            if (iss >> p >> type >> n >> m) {
                header_nodes = n;
            }
            continue;
        }

        if (line[0] == 'a') {
            std::istringstream iss(line.substr(1));
            long long u, v;
            double w;
            if (iss >> u >> v >> w) {
                if (u > 0 && v > 0) {
                    NodeId from = static_cast<NodeId>(u - 1);
                    NodeId to = static_cast<NodeId>(v - 1);
                    edges.emplace_back(from, to, w);
                    max_node = std::max(max_node, std::max(from, to));
                    found_node = true;
                }
            }
            continue;
        }

        if (line[0] == 'v') {
            std::istringstream iss(line.substr(1));
            long long id;
            double x, y;
            if (iss >> id >> x >> y) {
                if (id > 0) {
                    NodeId nid = static_cast<NodeId>(id - 1);
                    coords_in_gr.emplace_back(nid, x, y);
                    max_node = std::max(max_node, nid);
                    found_node = true;
                }
            }
            continue;
        }
    }

    std::size_t total_nodes = std::max(header_nodes, found_node ? (max_node + 1) : 0);
    Graph graph(total_nodes);

    for (const auto& [nid, x, y] : coords_in_gr) {
        if (nid < total_nodes) {
            graph.set_node_coordinates(nid, x, y);
        }
    }

    for (const auto& [u, v, w] : edges) {
        if (u < total_nodes && v < total_nodes) {
            graph.add_directed_edge(u, v, w);
        }
    }

    // Read separate .co file if provided
    if (!co_filepath.empty()) {
        std::ifstream co(co_filepath);
        if (co.is_open()) {
            std::string coline;
            while (std::getline(co, coline)) {
                if (coline.empty() || coline[0] == 'c' || coline[0] == 'p') continue;
                if (coline[0] == 'v') {
                    std::istringstream iss(coline.substr(1));
                    long long id;
                    double x, y;
                    if (iss >> id >> x >> y) {
                        if (id > 0) {
                            NodeId nid = static_cast<NodeId>(id - 1);
                            if (nid < graph.num_nodes()) {
                                graph.set_node_coordinates(nid, x, y);
                            }
                        }
                    }
                }
            }
        }
    }

    return graph;
}

bool GraphIO::save_dimacs(const Graph& graph, const std::string& gr_filepath, const std::string& co_filepath) {
    std::ofstream gr(gr_filepath);
    if (!gr.is_open()) return false;

    gr << "c DynamicRoute DIMACS road network export\n";
    gr << "p sp " << graph.num_nodes() << " " << graph.num_edges() << "\n";
    for (NodeId u = 0; u < graph.num_nodes(); ++u) {
        for (const auto& edge : graph.get_outgoing_edges(u)) {
            if (!edge.is_blocked) {
                gr << "a " << (u + 1) << " " << (edge.to + 1) << " " << edge.weight << "\n";
            }
        }
    }

    if (!co_filepath.empty()) {
        std::ofstream co(co_filepath);
        if (!co.is_open()) return false;

        co << "c DynamicRoute DIMACS coordinates export\n";
        co << "p aux sp co " << graph.num_nodes() << "\n";
        for (NodeId u = 0; u < graph.num_nodes(); ++u) {
            const auto& c = graph.get_node(u).coords;
            co << "v " << (u + 1) << " " << std::fixed << std::setprecision(6) << c.x << " " << c.y << "\n";
        }
    }

    return true;
}

// ---------------------------------------------------------------------------
// OpenStreetMap (OSM) XML Format (Phase 4)
// ---------------------------------------------------------------------------

namespace {

static std::string extract_xml_attr(const std::string& line, const std::string& attr) {
    std::string pattern1 = attr + "=\"";
    auto pos = line.find(pattern1);
    if (pos != std::string::npos) {
        pos += pattern1.size();
        auto end_pos = line.find('"', pos);
        if (end_pos != std::string::npos) {
            return line.substr(pos, end_pos - pos);
        }
    }
    std::string pattern2 = attr + "='";
    pos = line.find(pattern2);
    if (pos != std::string::npos) {
        pos += pattern2.size();
        auto end_pos = line.find('\'', pos);
        if (end_pos != std::string::npos) {
            return line.substr(pos, end_pos - pos);
        }
    }
    return "";
}

} // namespace

Graph GraphIO::load_osm_xml(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("GraphIO: cannot open OSM XML file: " + filepath);
    }

    struct RawNode {
        double lat{0.0};
        double lon{0.0};
        bool used{false};
    };

    struct RawWay {
        std::vector<long long> nd_refs;
        std::string highway;
        std::string oneway;
        double max_speed{50.0};
    };

    std::unordered_map<long long, RawNode> raw_nodes;
    std::vector<RawWay> raw_ways;

    bool in_way = false;
    RawWay current_way;

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;

        // Node element
        if (line.find("<node ") != std::string::npos) {
            std::string id_str = extract_xml_attr(line, "id");
            std::string lat_str = extract_xml_attr(line, "lat");
            std::string lon_str = extract_xml_attr(line, "lon");
            if (!id_str.empty() && !lat_str.empty() && !lon_str.empty()) {
                try {
                    long long id = std::stoll(id_str);
                    double lat = std::stod(lat_str);
                    double lon = std::stod(lon_str);
                    raw_nodes[id] = RawNode{lat, lon, false};
                } catch (...) {}
            }
            continue;
        }

        // Way begin
        if (line.find("<way ") != std::string::npos) {
            in_way = true;
            current_way = RawWay{};
            continue;
        }

        if (in_way) {
            if (line.find("<nd ") != std::string::npos) {
                std::string ref_str = extract_xml_attr(line, "ref");
                if (!ref_str.empty()) {
                    try {
                        current_way.nd_refs.push_back(std::stoll(ref_str));
                    } catch (...) {}
                }
            } else if (line.find("<tag ") != std::string::npos) {
                std::string k = extract_xml_attr(line, "k");
                std::string v = extract_xml_attr(line, "v");
                if (k == "highway") {
                    current_way.highway = v;
                } else if (k == "oneway") {
                    current_way.oneway = v;
                } else if (k == "maxspeed") {
                    try {
                        current_way.max_speed = std::stod(v);
                    } catch (...) {}
                }
            } else if (line.find("</way>") != std::string::npos) {
                in_way = false;
                if (!current_way.highway.empty() &&
                    current_way.highway != "footway" &&
                    current_way.highway != "steps" &&
                    current_way.highway != "cycleway" &&
                    current_way.nd_refs.size() >= 2)
                {
                    for (auto ref : current_way.nd_refs) {
                        auto it = raw_nodes.find(ref);
                        if (it != raw_nodes.end()) {
                            it->second.used = true;
                        }
                    }
                    raw_ways.push_back(current_way);
                }
            }
        }
    }

    // Map referenced OSM nodes to compact NodeId
    std::unordered_map<long long, NodeId> id_map;
    id_map.reserve(raw_nodes.size());

    double sum_lat = 0.0;
    double sum_lon = 0.0;
    std::size_t used_count = 0;

    for (const auto& [id, n] : raw_nodes) {
        if (n.used) {
            sum_lat += n.lat;
            sum_lon += n.lon;
            ++used_count;
        }
    }

    double ref_lat = (used_count > 0) ? (sum_lat / static_cast<double>(used_count)) : 0.0;
    double ref_lon = (used_count > 0) ? (sum_lon / static_cast<double>(used_count)) : 0.0;
    constexpr double PI = 3.14159265358979323846;
    constexpr double EARTH_RADIUS = 6371000.0;
    double cos_lat = std::cos(ref_lat * (PI / 180.0));

    std::vector<long long> sorted_ids;
    sorted_ids.reserve(raw_nodes.size());
    for (const auto& [id, n] : raw_nodes) {
        if (n.used) sorted_ids.push_back(id);
    }
    std::sort(sorted_ids.begin(), sorted_ids.end());

    Graph graph;
    for (auto id : sorted_ids) {
        const auto& n = raw_nodes[id];
        double x = EARTH_RADIUS * (n.lon - ref_lon) * (PI / 180.0) * cos_lat;
        double y = EARTH_RADIUS * (n.lat - ref_lat) * (PI / 180.0);
        NodeId nid = graph.add_node(x, y, "osm_" + std::to_string(id), "intersection");
        id_map[id] = nid;
    }

    // Add edges
    for (const auto& way : raw_ways) {
        for (std::size_t i = 0; i + 1 < way.nd_refs.size(); ++i) {
            auto it1 = id_map.find(way.nd_refs[i]);
            auto it2 = id_map.find(way.nd_refs[i + 1]);
            if (it1 != id_map.end() && it2 != id_map.end()) {
                NodeId u = it1->second;
                NodeId v = it2->second;
                double dist = graph.euclidean_distance(u, v);
                if (dist < 0.1) dist = 0.1;

                bool is_oneway = (way.oneway == "yes" || way.oneway == "true" || way.oneway == "1");
                bool is_reverse = (way.oneway == "-1");

                if (is_reverse) {
                    graph.add_directed_edge(v, u, dist, way.highway, 100.0, way.max_speed);
                } else if (is_oneway) {
                    graph.add_directed_edge(u, v, dist, way.highway, 100.0, way.max_speed);
                } else {
                    graph.add_undirected_edge(u, v, dist, way.highway, 100.0, way.max_speed);
                }
            }
        }
    }

    return graph;
}

bool GraphIO::save_osm_xml(const Graph& graph, const std::string& filepath) {
    std::ofstream file(filepath);
    if (!file.is_open()) return false;

    constexpr double PI = 3.14159265358979323846;
    constexpr double EARTH_RADIUS = 6371000.0;
    double ref_lat = 37.7749;
    double ref_lon = -122.4194;
    double cos_lat = std::cos(ref_lat * (PI / 180.0));

    file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    file << "<osm version=\"0.6\" generator=\"DynamicRoute\">\n";

    for (NodeId u = 0; u < graph.num_nodes(); ++u) {
        const auto& c = graph.get_node(u).coords;
        double lat = ref_lat + (c.y / EARTH_RADIUS) * (180.0 / PI);
        double lon = ref_lon + (c.x / (EARTH_RADIUS * cos_lat)) * (180.0 / PI);
        file << "  <node id=\"" << (u + 1) << "\" lat=\"" << std::fixed << std::setprecision(7)
             << lat << "\" lon=\"" << lon << "\"/>\n";
    }

    std::size_t way_id = 1;
    for (NodeId u = 0; u < graph.num_nodes(); ++u) {
        for (const auto& edge : graph.get_outgoing_edges(u)) {
            if (u <= edge.to) {
                file << "  <way id=\"" << way_id++ << "\">\n";
                file << "    <nd ref=\"" << (u + 1) << "\"/>\n";
                file << "    <nd ref=\"" << (edge.to + 1) << "\"/>\n";
                file << "    <tag k=\"highway\" v=\"" << edge.road_type << "\"/>\n";
                file << "    <tag k=\"maxspeed\" v=\"" << static_cast<int>(edge.max_speed) << "\"/>\n";
                file << "  </way>\n";
            }
        }
    }

    file << "</osm>\n";
    return true;
}

} // namespace dynamic_route
