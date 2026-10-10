#include "diagnostics/audit_log.hpp"

#include <sstream>
#include <fstream>
#include <cmath>
#include <numeric>
#include <algorithm>

namespace dynamic_route {

// ---------------------------------------------------------------------------
// record
// ---------------------------------------------------------------------------

void AuditLogger::record(AuditEntry entry) {
    entries_.push_back(std::move(entry));
}

// ---------------------------------------------------------------------------
// json_escape  (minimal: handle backslash and double-quote)
// ---------------------------------------------------------------------------

std::string AuditLogger::json_escape(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (char c : s) {
        if (c == '"')  { out += "\\\""; }
        else if (c == '\\') { out += "\\\\"; }
        else if (c == '\n') { out += "\\n"; }
        else if (c == '\r') { out += "\\r"; }
        else if (c == '\t') { out += "\\t"; }
        else { out += c; }
    }
    return out;
}

// ---------------------------------------------------------------------------
// entry_to_json
// ---------------------------------------------------------------------------

std::string AuditLogger::entry_to_json(const AuditEntry& e) const {
    std::ostringstream o;

    // Weight helper: INF → "INF" string, else numeric
    auto weight_str = [](double w) -> std::string {
        if (std::isinf(w)) return "\"INF\"";
        std::ostringstream ws;
        ws << w;
        return ws.str();
    };

    o << "{"
      << "\"step\":" << e.step << ","
      << "\"algorithm\":\"" << json_escape(e.algorithm) << "\","
      << "\"edge_change\":{"
        << "\"from\":"       << e.edge_change.from      << ","
        << "\"to\":"         << e.edge_change.to        << ","
        << "\"old_w\":"      << weight_str(e.edge_change.old_weight) << ","
        << "\"new_w\":"      << weight_str(e.edge_change.new_weight) << ","
        << "\"is_blockage\":" << (e.edge_change.is_blockage ? "true" : "false")
      << "},"
      << "\"nodes_invalidated\":"  << e.nodes_invalidated << ","
      << "\"nodes_relaxed\":"      << e.nodes_relaxed     << ","
      << "\"nodes_expanded\":"     << e.nodes_expanded    << ","
      << "\"repair_time_us\":"     << e.repair_time_us    << ","
      << "\"path_found\":"         << (e.path_found ? "true" : "false") << ","
      << "\"final_cost\":"         << weight_str(e.final_cost) << ","
      << "\"path\":[";

    for (std::size_t i = 0; i < e.path.size(); ++i) {
        if (i > 0) o << ",";
        o << e.path[i];
    }

    o << "],"
      << "\"correct\":"     << (e.correct ? "true" : "false") << ","
      << "\"cost_delta\":"  << e.cost_delta
      << "}";

    return o.str();
}

// ---------------------------------------------------------------------------
// write_ndjson
// ---------------------------------------------------------------------------

void AuditLogger::write_ndjson(std::ostream& out) const {
    for (const auto& e : entries_) {
        out << entry_to_json(e) << "\n";
    }
}

// ---------------------------------------------------------------------------
// save_ndjson
// ---------------------------------------------------------------------------

bool AuditLogger::save_ndjson(const std::string& filepath) const {
    std::ofstream f(filepath);
    if (!f.is_open()) return false;
    write_ndjson(f);
    return true;
}

// ---------------------------------------------------------------------------
// parse_json_line  (minimal hand-rolled parser for our specific schema)
// ---------------------------------------------------------------------------

// Extract value for a key in a flat JSON object string.
// Returns empty string if key not found.
static std::string extract_field(const std::string& json, const std::string& key) {
    std::string needle = "\"" + key + "\":";
    auto pos = json.find(needle);
    if (pos == std::string::npos) return "";
    pos += needle.size();

    // Skip whitespace
    while (pos < json.size() && json[pos] == ' ') ++pos;
    if (pos >= json.size()) return "";

    char first = json[pos];
    if (first == '"') {
        // String value
        ++pos;
        std::string val;
        while (pos < json.size() && json[pos] != '"') {
            if (json[pos] == '\\' && pos + 1 < json.size()) {
                ++pos;
                val += json[pos];
            } else {
                val += json[pos];
            }
            ++pos;
        }
        return val;
    } else {
        // Number / bool / null
        auto end = json.find_first_of(",}", pos);
        if (end == std::string::npos) end = json.size();
        return json.substr(pos, end - pos);
    }
}

// Parse an array of integers from a JSON array string like "[0,3,7,11]"
static std::vector<NodeId> parse_id_array(const std::string& json, const std::string& key) {
    std::string needle = "\"" + key + "\":[";
    auto pos = json.find(needle);
    if (pos == std::string::npos) return {};
    pos += needle.size();

    std::vector<NodeId> result;
    while (pos < json.size() && json[pos] != ']') {
        auto comma = json.find_first_of(",]", pos);
        if (comma == std::string::npos) break;
        std::string num = json.substr(pos, comma - pos);
        if (!num.empty()) {
            try {
                result.push_back(static_cast<NodeId>(std::stoull(num)));
            } catch (...) {}
        }
        if (json[comma] == ']') break;
        pos = comma + 1;
    }
    return result;
}

bool AuditLogger::parse_json_line(const std::string& line, AuditEntry& out) {
    if (line.empty() || line[0] != '{') return false;

    try {
        out.step      = std::stoi(extract_field(line, "step"));
        out.algorithm = extract_field(line, "algorithm");

        auto old_w_str = extract_field(line, "old_w");
        auto new_w_str = extract_field(line, "new_w");
        out.edge_change.old_weight = (old_w_str == "INF") ? INF_WEIGHT : std::stod(old_w_str);
        out.edge_change.new_weight = (new_w_str == "INF") ? INF_WEIGHT : std::stod(new_w_str);
        out.edge_change.from       = static_cast<NodeId>(std::stoull(extract_field(line, "from")));
        out.edge_change.to         = static_cast<NodeId>(std::stoull(extract_field(line, "to")));
        out.edge_change.is_blockage = (extract_field(line, "is_blockage") == "true");

        out.nodes_invalidated = static_cast<std::size_t>(std::stoull(extract_field(line, "nodes_invalidated")));
        out.nodes_relaxed     = static_cast<std::size_t>(std::stoull(extract_field(line, "nodes_relaxed")));
        out.nodes_expanded    = static_cast<std::size_t>(std::stoull(extract_field(line, "nodes_expanded")));
        out.repair_time_us    = std::stod(extract_field(line, "repair_time_us"));
        out.path_found        = (extract_field(line, "path_found") == "true");

        auto cost_str = extract_field(line, "final_cost");
        out.final_cost = (cost_str == "INF") ? INF_WEIGHT : std::stod(cost_str);

        out.path       = parse_id_array(line, "path");
        out.correct    = (extract_field(line, "correct") == "true");
        out.cost_delta = std::stod(extract_field(line, "cost_delta"));
        return true;
    } catch (...) {
        return false;
    }
}

// ---------------------------------------------------------------------------
// load_ndjson
// ---------------------------------------------------------------------------

AuditLogger AuditLogger::load_ndjson(const std::string& filepath) {
    AuditLogger logger;
    std::ifstream f(filepath);
    if (!f.is_open()) return logger;

    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] != '{') continue;
        AuditEntry entry;
        if (parse_json_line(line, entry)) {
            logger.record(std::move(entry));
        }
    }
    return logger;
}

// ---------------------------------------------------------------------------
// compute_stats
// ---------------------------------------------------------------------------

AuditLogger::Stats AuditLogger::compute_stats(std::size_t graph_node_count) const {
    Stats s;
    s.total_steps  = entries_.size();
    s.graph_nodes  = graph_node_count;
    if (entries_.empty()) return s;

    double sum_repair = 0.0;
    double sum_relaxed = 0.0;
    double sum_invalidated = 0.0;
    double max_repair = 0.0;

    for (const auto& e : entries_) {
        if (e.correct) ++s.correct_steps;
        sum_repair       += e.repair_time_us;
        sum_relaxed      += static_cast<double>(e.nodes_relaxed);
        sum_invalidated  += static_cast<double>(e.nodes_invalidated);
        if (e.repair_time_us > max_repair) max_repair = e.repair_time_us;
    }

    double n = static_cast<double>(entries_.size());
    s.avg_repair_us          = sum_repair / n;
    s.max_repair_us          = max_repair;
    s.avg_nodes_relaxed      = sum_relaxed / n;
    s.avg_nodes_invalidated  = sum_invalidated / n;

    if (graph_node_count > 0) {
        s.avg_locality_ratio = s.avg_nodes_invalidated
                               / static_cast<double>(graph_node_count);
    }
    return s;
}

} // namespace dynamic_route
