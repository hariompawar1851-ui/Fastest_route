# Phase 3 — Indexed-Queue & Moving-Agent D* Lite: Changes, New Files & Code Explanation

> **Status:** ✅ Complete — All 31 tests pass (Tests 27–31 are Phase 3)

---

## 1. What Was Changed / Added in Phase 3

### Problem with Phase 2

Phase 2 left two gaps in the incremental search engine:

1. **Priority queue correctness** — `update_vertex()` removed nodes from `std::set<pair<LPAKey, NodeId>>` by scanning for any matching node, which could erase a stale entry with the wrong key. It also had **O(n) worst-case complexity** in the naive form.
2. **No moving-agent support** — D* Lite was designed for a vehicle that physically drives, but Phase 2 only used it with a fixed start. The `km_` accumulator (essential for correct key comparisons after the agent moves) was present but `update_start()` was not wired into a navigation loop.

Phase 3 fixes both gaps and adds a full moving-agent simulation layer (`RouteNavigator`).

---

## 2. New / Modified Files

| File | Type | Phase 3 Action |
|---|---|---|
| `include/algorithms/lpa_star.hpp` | **Updated** | `queue_invariant_holds()`, `total_expansions()`, `queue_size()`; `in_open_list_[]` + `queued_key_[]` bookkeeping vectors |
| `src/algorithms/lpa_star.cpp` | **Updated** | Indexed erase in `update_vertex()`; `nodes_expanded_total_` accumulator; `queue_invariant_holds()` implementation |
| `include/algorithms/d_star_lite.hpp` | **Updated** | `update_start()`, `km()`, `current_start()`, `goal()`, `queue_invariant_holds()`, `total_expansions()`; same bookkeeping vectors |
| `src/algorithms/d_star_lite.cpp` | **Updated** | Indexed erase in `update_vertex()`; `update_start()` accumulates `km_`; termination condition uses live `start_` |
| `include/algorithms/route_navigator.hpp` | **New** | `NavigationStep`, `NavigationReport` structs; `RouteNavigator` class with `EventProvider` callback |
| `src/algorithms/route_navigator.cpp` | **New** | Full moving-agent simulation: hop-by-hop loop, optional Dijkstra cross-check, per-step stat recording |
| `CMakeLists.txt` | **Updated** | `src/algorithms/route_navigator.cpp` added to `dynamic_route_core` static library |
| `tests/test_main.cpp` | **Updated** | Tests 27–31 added; main changed from "ALL 26" to "ALL 31"; Phase 3 section header added |

---

## 3. File-by-File Explanation

---

### 3.1 `include/algorithms/lpa_star.hpp` + `src/algorithms/lpa_star.cpp` — UPDATED

#### The Indexed-Queue Fix

**Before Phase 3** — `update_vertex(u)` called `open_list_.erase(u)`, which the implementation had to do with a linear scan (finding any `pair<key, u>` in the set):

```cpp
// OLD — O(n) naive removal
for (auto it = open_list_.begin(); it != open_list_.end(); ++it) {
    if (it->second == u) { open_list_.erase(it); break; }
}
```

**After Phase 3** — two new side-tables are added as class members:

```cpp
std::vector<bool>    in_open_list_;   // is u currently in the set?
std::vector<LPAKey>  queued_key_;     // the key u was inserted with
```

`update_vertex(u)` now removes `u` in `O(log n)`:

```cpp
void LifelongPlanningAStar::update_vertex(NodeId u) {
    // ... recompute rhs[u] ...

    if (in_open_list_[u]) {
        open_list_.erase({queued_key_[u], u});   // O(log n) exact erase
        in_open_list_[u] = false;
    }

    if (std::abs(g_[u] - rhs_[u]) > 1e-9) {
        queued_key_[u] = calculate_key(u);
        open_list_.insert({queued_key_[u], u});
        in_open_list_[u] = true;
    }
}
```

#### Diagnostics Added

| Method | Returns | Description |
|---|---|---|
| `queue_size()` | `size_t` | Current number of inconsistent vertices in the queue |
| `total_expansions()` | `size_t` | Cumulative PQ pops across all `compute_shortest_path()` calls |
| `queue_invariant_holds()` | `bool` | Deep consistency check — see below |

#### `queue_invariant_holds()` Implementation

```cpp
bool LifelongPlanningAStar::queue_invariant_holds() const {
    std::size_t flagged = 0;
    for (NodeId u = 0; u < g_.size(); ++u) {
        const bool inconsistent = std::abs(g_[u] - rhs_[u]) > 1e-9;
        if (inconsistent != in_open_list_[u]) return false;  // mismatch
        if (in_open_list_[u]) {
            ++flagged;
            // The recorded key must actually exist in the set
            if (open_list_.find({queued_key_[u], u}) == open_list_.end())
                return false;
        }
    }
    return flagged == open_list_.size();  // no phantom entries
}
```

This verifies three properties simultaneously:
1. Every inconsistent vertex has a queue entry.
2. Every consistent vertex is absent from the queue.
3. Every bookkeeping entry points to a real element in the set.

---

### 3.2 `include/algorithms/d_star_lite.hpp` + `src/algorithms/d_star_lite.cpp` — UPDATED

#### Same Indexed-Queue Fix

Identical bookkeeping (`in_open_list_[]`, `queued_key_[]`) and the same `O(log n)` erase pattern. D* Lite's `update_vertex()` scans **outgoing** edges (backward search from goal), but the queue mechanics are the same.

#### Moving-Agent API — `update_start(new_start)`

D* Lite searches **backward** from goal to the agent's current position. When the agent moves, the start shifts. Naively re-keying the queue would cost `O(n log n)`. Instead, D* Lite accumulates a scalar key modifier `km_`:

```cpp
void DStarLite::update_start(NodeId new_start) {
    km_ += heuristic(last_start_, new_start);  // accumulate travel distance
    last_start_ = new_start;
    start_      = new_start;
}
```

`calculate_key(u)` adds `km_` to the first key component:

```cpp
LPAKey DStarLite::calculate_key(NodeId u) const {
    double min_val = std::min(g_[u], rhs_[u]);
    return { min_val + heuristic(start_, u) + km_,   // k1 + km offset
             min_val };                               // k2 unchanged
}
```

This means all existing queue entries remain valid — no re-keying needed — with **O(1) cost per agent move**.

#### New Accessors

| Method | Returns | Description |
|---|---|---|
| `current_start()` | `NodeId` | Current agent position (start of backward search) |
| `goal()` | `NodeId` | Fixed goal node |
| `km()` | `double` | Accumulated key modifier |
| `queue_size()` | `size_t` | Inconsistent vertices in queue |
| `total_expansions()` | `size_t` | Cumulative PQ pops |
| `queue_invariant_holds()` | `bool` | Same 3-property invariant as LPA* |

---

### 3.3 `include/algorithms/route_navigator.hpp` + `src/algorithms/route_navigator.cpp` — NEW

#### Purpose

`RouteNavigator` implements the full **moving-agent navigation loop** described in:

> Koenig, S. & Likhachev, M. (2002). *D* Lite*. AAAI-02.

It simulates a vehicle that physically drives hop-by-hop along the D* Lite plan while an optional `EventProvider` mutates the graph at each step.

#### `NavigationStep` — Per-Hop Record

```cpp
struct NavigationStep {
    std::size_t step;           // hop index (0-based)
    NodeId      from;           // position before the hop
    NodeId      to;             // position after the hop
    double      edge_cost;      // weight of the traversed edge
    std::size_t events_applied; // graph events applied after this hop
    std::size_t expansions;     // D* Lite expansions for the repair
    double      remaining_cost; // plan cost-to-go after repair
    double      repair_us;      // wall-clock time of the repair
};
```

#### `NavigationReport` — Full-Run Summary

| Field | Type | Description |
|---|---|---|
| `reached_goal` | bool | Agent arrived at goal |
| `blocked` | bool | Goal became unreachable |
| `step_limit_hit` | bool | Safety cap triggered |
| `initial_cost` | double | Optimal cost from start at t=0 |
| `travelled_cost` | double | Total weight of edges driven |
| `trajectory` | `vector<NodeId>` | Visited nodes, start to final |
| `steps` | `vector<NavigationStep>` | Per-hop records |
| `moves` | size_t | Total hops taken |
| `replans` | size_t | Hops where ≥1 event fired |
| `total_events` | size_t | Sum of graph mutations applied |
| `initial_expansions` | size_t | D* Lite expansions at t=0 |
| `incremental_expansions` | size_t | Sum of repair expansions |
| `baseline_expansions` | size_t | Dijkstra-from-scratch (verify mode) |
| `verified_steps` | size_t | Steps cross-checked |
| `mismatches` | size_t | D* Lite ≠ Dijkstra (must be 0) |

#### `EventProvider` Callback

```cpp
using EventProvider = std::function<std::vector<PerturbationEvent>(
    std::size_t step, Graph& graph, NodeId position)>;
```

Called after each hop. May mutate the graph and **must return a list of the events it applied** so they can be forwarded to the D* Lite planner. Passing `nullptr` gives a static-world simulation.

#### `run()` Loop — Step by Step

```cpp
NavigationReport RouteNavigator::run(const EventProvider& provider,
                                     std::size_t max_steps,
                                     bool verify) {
    DStarLite dstar(graph_, start_, goal_);
    SearchResult plan = dstar.compute_shortest_path();   // initial search
    // ...

    while (position != goal_) {
        // 1. Drive one hop
        NodeId next = plan.path[1];
        report.travelled_cost += edge->effective_weight();
        position = next;

        // 2. Tell D* Lite the agent moved
        dstar.update_start(position);

        // 3. Apply world events
        if (provider) {
            auto events = provider(step, graph_, position);
            for (const auto& ev : events)
                dstar.on_edge_changed(ev.from, ev.to, ev.new_weight);
        }

        // 4. Incremental repair
        plan = dstar.compute_shortest_path();

        // 5. Optional Dijkstra cross-check
        if (verify) cross_check(position, plan);
    }
}
```

**Safety cap:** `max_steps` defaults to `4 * |V|` — guarantees termination even if the graph has no path.

**Verify mode:** Runs `Dijkstra::find_shortest_path(graph_, position, goal_)` at every hop and compares costs. Each discrepancy increments `report.mismatches`. All Phase 3 tests use `verify=true`.

---

## 4. Test Results

```
============================================================
  DynamicRoute - Complete Test Suite (Phase 0 + 1 + 2 + 3)
============================================================

--- Phase 3: Indexed Queue & Moving-Agent D* Lite ---
[Test 27] Indexed queue: LPA* & D* Lite invariant + Dijkstra agreement (300 steps)...
          PASSED (LPA* exp=598, D* Lite exp=716)
[Test 28] RouteNavigator - static world follows optimal route (15x15 grid)...
          PASSED (28 hops, cost=28, repair expansions=0)
[Test 29] RouteNavigator - random dynamics, per-hop D* Lite == Dijkstra (3 seeds, 20x20)...
          PASSED (117 steps verified; 3 reached, 0 blocked)
[Test 30] RouteNavigator - forced detour + unreachable goal...
          PASSED (detour cost within [10,12]; unreachable chain detected)
[Test 31] RouteNavigator - D* Lite repairs < Dijkstra-per-hop (40x40)...
          PASSED (D* Lite 1600 vs Dijkstra 74258 expansions, 97.8% fewer)

 ALL 31 TESTS PASSED!
```

---

## 5. Benchmark Results (Phase 3 Efficiency — Test 31)

| Metric | Value |
|---|---|
| Graph | Grid 40×40 (1,600 nodes) |
| D* Lite total expansions (initial + incremental) | 1,600 |
| Dijkstra-per-hop baseline expansions | 74,258 |
| **Expansion reduction** | **97.8%** |

D* Lite repaired the route using **47× fewer vertex expansions** than running Dijkstra from scratch at every hop — on a grid where a new random event fires every 3 hops.

---

## 6. What Remains for Future Phases

| Phase | Remaining Work |
|---|---|
| **Phase 4** | Scalability benchmarks on OSM / DIMACS real road networks (100K–500K nodes) |
| **Phase 5 (Optional)** | Ramalingam–Reps SSSP engine (advanced extensions) |
| **Phase 6 (Optional)** | Degree-2 topological compression pipeline |
| **Phase 7 (Optional)** | OSM road graph importer + sophisticated traffic models |
| **Phase 8 (Optional)** | Memory/cache profiling (CSR layout) + interactive visualizer |
