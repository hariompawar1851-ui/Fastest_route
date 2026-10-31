# Phase 3 — Code Explanation & Implementation Reference

> **Status:** ✅ Complete — All 5 Phase 3 tests passing (Tests 27–31, total 31/31)

---

## Table of Contents

1. [Overview & Goals](#1-overview--goals)
2. [Indexed Priority Queue Upgrade (LPA* & D* Lite)](#2-indexed-priority-queue-upgrade-lpa--d-lite)
3. [D* Lite Moving-Agent API](#3-d-lite-moving-agent-api)
4. [RouteNavigator — Moving-Agent Navigation Layer](#4-routenavigator--moving-agent-navigation-layer)
5. [Test Suite — Tests 27–31](#5-test-suite--tests-2731)
6. [CMakeLists.txt Changes](#6-cmakeliststxt-changes)
7. [Test Summary Table](#7-test-summary-table)
8. [Key Design Decisions](#8-key-design-decisions)
9. [Efficiency Results](#9-efficiency-results)

---

## 1. Overview & Goals

Phase 3 extended DynamicRoute (which completed Phase 2 with 26 passing tests) across two focused areas:

| Area | Focus | Tests |
|---|---|---|
| **Indexed Queue** | O(log n) update_vertex for LPA* & D* Lite; queue invariant checker | 27 |
| **Moving-Agent Navigation** | D* Lite update_start + km_; RouteNavigator simulation loop | 28–31 |

All 5 new tests (27–31) pass. The total project test suite is now **31 tests across all phases**.

---

## 2. Indexed Priority Queue Upgrade (LPA* & D* Lite)

### 2.1 The Problem

Both `LifelongPlanningAStar` and `DStarLite` maintain an open list as
`std::set<pair<LPAKey, NodeId>>`. In `update_vertex(u)`, nodes need to be
**removed and re-inserted** whenever their key changes.

The set is keyed by `(LPAKey, NodeId)`. In Phase 0–2, there was no efficient
way to find and erase a specific `(key, u)` pair given only the node `u` — so
removal required a linear scan or a `find_if`, making `update_vertex()` **O(n)**
in the worst case.

Beyond performance, there was a subtle correctness risk: if a stale `(old_key, u)`
was left in the set and `u` was re-inserted with a `(new_key, u)`, both entries
would coexist, violating the single-entry invariant.

### 2.2 The Solution — Two Bookkeeping Side-Tables

Two new member vectors are added to both classes:

```cpp
std::vector<bool>    in_open_list_;   // is vertex u currently in the set?
std::vector<LPAKey>  queued_key_;     // the key u was last inserted with
```

**Initialization (constructor):**
```cpp
in_open_list_.assign(n, false);
queued_key_.assign(n, LPAKey{0.0, 0.0});

// Insert source (LPA*) or goal (D* Lite)
queued_key_[seed] = calculate_key(seed);
open_list_.insert({queued_key_[seed], seed});
in_open_list_[seed] = true;
```

**`update_vertex(u)` — Phase 3 version (LPA*):**

```cpp
void LifelongPlanningAStar::update_vertex(NodeId u) {
    // Step 1: Recompute rhs[u]
    if (u != source_) {
        double min_rhs = INF_WEIGHT;
        for (const auto& edge : graph_.get_incoming_edges(u)) {
            double w = edge.effective_weight();
            if (std::isinf(w)) continue;
            if (g_[edge.from] + w < min_rhs)
                min_rhs = g_[edge.from] + w;
        }
        rhs_[u] = min_rhs;
    }

    // Step 2: Remove old entry if present — O(log n)
    if (in_open_list_[u]) {
        open_list_.erase({queued_key_[u], u});  // exact (key, node) erase
        in_open_list_[u] = false;
    }

    // Step 3: Re-insert if inconsistent
    if (std::abs(g_[u] - rhs_[u]) > 1e-9) {
        queued_key_[u] = calculate_key(u);
        open_list_.insert({queued_key_[u], u});
        in_open_list_[u] = true;
    }
}
```

**Why `std::set::erase({queued_key_[u], u})` is O(log n):**
`std::set` is a balanced BST. Erasing by value (the exact `pair<LPAKey, NodeId>`)
uses the BST ordering to locate and remove the element in `O(log n)`. Since
`queued_key_[u]` records exactly what was inserted, the pair is guaranteed to exist.

### 2.3 `compute_shortest_path()` — Expansion Tracking

```cpp
SearchResult LifelongPlanningAStar::compute_shortest_path() {
    std::size_t expansions = 0;
    while (!open_list_.empty()) {
        // ... termination check ...
        NodeId u = top_it->second;
        open_list_.erase(top_it);
        in_open_list_[u] = false;
        ++expansions;
        // ... overconsistent / underconsistent handling ...
    }
    nodes_expanded_total_ += expansions;   // cumulative counter
    result.nodes_visited = expansions;
    // ...
}
```

`total_expansions()` returns `nodes_expanded_total_` — the sum across **all**
`compute_shortest_path()` calls since construction. This allows Test 27 to verify
that both algorithms actually did work (expansions > 0) after 300 perturbations.

### 2.4 `queue_invariant_holds()` — Deep Correctness Checker

The core LPA*/D* Lite invariant is:
> **A vertex is in the open list if and only if it is locally inconsistent (g ≠ rhs).**

`queue_invariant_holds()` verifies this plus the bookkeeping correctness:

```cpp
bool LifelongPlanningAStar::queue_invariant_holds() const {
    std::size_t flagged = 0;
    for (NodeId u = 0; u < g_.size(); ++u) {
        const bool inconsistent = std::abs(g_[u] - rhs_[u]) > 1e-9;

        // Property 1 & 2: presence flag matches actual consistency state
        if (inconsistent != in_open_list_[u]) return false;

        if (in_open_list_[u]) {
            ++flagged;
            // Property 3: bookkeeping key actually exists in the set
            if (open_list_.find({queued_key_[u], u}) == open_list_.end())
                return false;
        }
    }
    // Property 4: no phantom entries (set has exactly as many as flagged)
    return flagged == open_list_.size();
}
```

**Four properties checked simultaneously:**

| # | Property | What it catches |
|---|---|---|
| 1 | Every inconsistent vertex is in the list | Missing queue entries (vertex would never be repaired) |
| 2 | Every consistent vertex is absent | Phantom entries (stale nodes bloating the queue) |
| 3 | Bookkeeping key resolves to a real set element | Key mismatch (stale `queued_key_` after a re-insert without removing old entry) |
| 4 | `flagged_count == open_list_.size()` | No ghost entries not reflected in `in_open_list_[]` |

### 2.5 D* Lite — Identical Mechanics, Mirrored Direction

D* Lite's `update_vertex()` differs only in that it scans **outgoing** edges
(backward search from goal uses successors, not predecessors):

```cpp
void DStarLite::update_vertex(NodeId u) {
    if (u != goal_) {
        double min_rhs = INF_WEIGHT;
        for (const auto& edge : graph_.get_outgoing_edges(u)) {  // outgoing!
            // ...
        }
        rhs_[u] = min_rhs;
    }
    // Same indexed erase/insert as LPA* ...
}
```

All bookkeeping vectors, erase/insert pattern, and `queue_invariant_holds()` are
structurally identical.

---

## 3. D* Lite Moving-Agent API

### 3.1 Motivation — The Key Modifier Problem

D* Lite is inherently designed for a **mobile agent**: it searches backward from
the goal and is re-rooted at the agent's position after each move. When the agent
moves from node `last` to node `curr`, the heuristic estimate `h(start, u)` for
every vertex `u` changes — because `start` changed.

Naively updating every key in the queue would cost `O(|queue| · log |queue|)` —
equivalent to rebuilding the queue from scratch.

### 3.2 The `km_` Solution (Koenig & Likhachev, 2002)

Instead of updating any keys, a single scalar accumulator `km_` is maintained.
Every time the agent moves, `km_` grows by the travel distance:

```cpp
void DStarLite::update_start(NodeId new_start) {
    km_ += heuristic(last_start_, new_start);   // h(last, curr)
    last_start_ = new_start;
    start_      = new_start;
}
```

`calculate_key(u)` always adds the current `km_` to the primary key component:

```cpp
LPAKey DStarLite::calculate_key(NodeId u) const {
    double min_val = std::min(g_[u], rhs_[u]);
    return {
        min_val + heuristic(start_, u) + km_,  // k1: f-value + global offset
        min_val                                 // k2: g-value tie-breaker
    };
}
```

**Why this is correct:**

Before moving: `k1(u) = min(g,rhs) + h(old_start, u)`
After moving (old start → new start):
- True new k1 = `min(g,rhs) + h(new_start, u)`
- Using `km_` trick: `min(g,rhs) + h(new_start, u) + km_` where `km_` now includes `h(old_start, new_start)`

By the triangle inequality (`h(old, u) ≤ h(old, new) + h(new, u)`), the accumulated
`km_` correctly upper-bounds the key shift, preserving queue order validity.

**Cost: O(1) per agent move** — no queue entries are touched.

### 3.3 Accessors Added

```cpp
NodeId current_start() const noexcept { return start_; }
NodeId goal()          const noexcept { return goal_;  }
double km()            const noexcept { return km_;    }
```

These expose agent state for `RouteNavigator` and for test assertions.

---

## 4. RouteNavigator — Moving-Agent Navigation Layer

### 4.1 `NavigationStep` Struct

One record is created per hop:

```cpp
struct NavigationStep {
    std::size_t step;           // hop index (0-based)
    NodeId      from;           // position before the hop
    NodeId      to;             // position after the hop
    double      edge_cost;      // weight of the traversed edge
    std::size_t events_applied; // graph events applied this hop
    std::size_t expansions;     // D* Lite PQ pops for the repair
    double      remaining_cost; // plan cost-to-go after repair
    double      repair_us;      // wall-clock repair time (µs)
};
```

### 4.2 `NavigationReport` Struct

Aggregated statistics for the full navigation run:

```cpp
struct NavigationReport {
    bool   reached_goal;          // arrived at goal
    bool   blocked;               // goal became unreachable
    bool   step_limit_hit;        // safety cap hit
    double initial_cost;          // optimal cost at t=0
    double travelled_cost;        // sum of edge weights driven

    std::vector<NodeId>          trajectory;   // node sequence driven
    std::vector<NavigationStep>  steps;        // per-hop records

    std::size_t moves;                // total hops
    std::size_t replans;              // hops with ≥1 event
    std::size_t total_events;         // total mutations applied
    std::size_t initial_expansions;   // expansions for first search
    std::size_t incremental_expansions; // sum of repair expansions
    std::size_t baseline_expansions;  // Dijkstra-from-scratch (verify)
    std::size_t verified_steps;       // hops cross-checked
    std::size_t mismatches;           // D* Lite ≠ Dijkstra (must be 0)
};
```

### 4.3 `EventProvider` — The World Mutation Callback

```cpp
using EventProvider = std::function<std::vector<PerturbationEvent>(
    std::size_t step, Graph& graph, NodeId position)>;
```

The callback is called **after** each hop. It receives:
- The current step index (for deterministic event scheduling).
- A mutable reference to the live graph (to apply blockages, weight changes).
- The agent's current position (for proximity-based event logic).

It must return the list of `PerturbationEvent`s it applied. These are forwarded
to `dstar.on_edge_changed()` so the planner stays in sync with the graph.

Passing `nullptr` gives a static-world run (Tests 28).

### 4.4 `run()` — The Full Navigation Loop

```cpp
NavigationReport RouteNavigator::run(const EventProvider& provider,
                                     std::size_t max_steps,
                                     bool verify) {
    // --- Setup ---
    if (max_steps == 0) max_steps = 4 * graph_.num_nodes();  // safety cap

    DStarLite dstar(graph_, start_, goal_);
    SearchResult plan = dstar.compute_shortest_path();        // initial search
    report.initial_expansions = plan.nodes_visited;
    report.trajectory.push_back(start_);
    if (verify) cross_check(start_, plan);                    // t=0 verify

    if (!plan.path_found) { report.blocked = true; return report; }
    report.initial_cost = plan.total_cost;

    NodeId position = start_;
    while (position != goal_) {
        if (step >= max_steps) { report.step_limit_hit = true; break; }
        if (plan.path.size() < 2) { report.blocked = true; break; }

        // ---- 1. Drive one hop ----
        const NodeId next = plan.path[1];
        const Edge* edge  = graph_.get_edge(position, next);
        report.travelled_cost += edge->effective_weight();
        position = next;
        report.trajectory.push_back(position);
        ++report.moves;

        // ---- 2. Advance D* Lite start ----
        dstar.update_start(position);         // O(1) — updates km_

        // ---- 3. Apply world events ----
        if (provider) {
            auto events = provider(step, graph_, position);
            for (const auto& ev : events)
                dstar.on_edge_changed(ev.from, ev.to, ev.new_weight);
            rec.events_applied = events.size();
            report.total_events += events.size();
            if (!events.empty()) ++report.replans;
        }

        // ---- 4. Incremental repair ----
        plan = dstar.compute_shortest_path();
        rec.expansions = plan.nodes_visited;
        report.incremental_expansions += plan.nodes_visited;

        // ---- 5. Optional cross-check ----
        if (verify) cross_check(position, plan);

        report.steps.push_back(rec);
        if (!plan.path_found) { report.blocked = true; break; }
    }
    report.reached_goal = (position == goal_);
    return report;
}
```

### 4.5 `cross_check()` Lambda — Dijkstra Oracle

```cpp
auto cross_check = [&](NodeId position, const SearchResult& inc) {
    auto ref = Dijkstra::find_shortest_path(graph_, position, goal_);
    report.baseline_expansions += ref.nodes_visited;
    ++report.verified_steps;
    bool ok = (inc.path_found == ref.path_found);
    if (ok && inc.path_found)
        ok = std::abs(inc.total_cost - ref.total_cost) < 1e-6;
    if (!ok) ++report.mismatches;
};
```

`baseline_expansions` accumulates Dijkstra's node visits across all hops.
This is the fair comparison denominator for Test 31's efficiency assertion.

---

## 5. Test Suite — Tests 27–31

### Test 27 — `test27_indexed_queue_invariants()`

**Scenario:** 300 random perturbations on a 20×20 grid (400 nodes, seed=7).

**What it checks:**
1. `queue_invariant_holds()` returns `true` after the initial search.
2. `queue_invariant_holds()` returns `true` after every `on_edge_changed()` + `compute_shortest_path()`.
3. `lpa.compute_shortest_path().total_cost == Dijkstra.total_cost` at every step.
4. `dstar.compute_shortest_path().total_cost == Dijkstra.total_cost` at every step.
5. `lpa.total_expansions() > 0` and `dstar.total_expansions() > 0` (algorithms actually worked).

**Why this is hard to fake:** The invariant checker verifies 4 independent properties of
internal state per vertex. Any off-by-one in the bookkeeping vectors would be caught.

```
PASSED (LPA* exp=598, D* Lite exp=716)
```

---

### Test 28 — `test28_navigator_static_world()`

**Scenario:** 15×15 grid, `RouteNavigator(g, 0, 224)`, no event provider.

**What it checks (exhaustive list):**
```cpp
assert(rep.reached_goal);
assert(!rep.blocked);
assert(std::abs(rep.initial_cost  - 28.0) < 1e-6);  // Manhattan distance = 28
assert(std::abs(rep.travelled_cost - 28.0) < 1e-6);  // same in static world
assert(rep.moves == 28);
assert(rep.trajectory.size() == 29);                 // 28 hops + start
assert(rep.trajectory.front() == 0 && rep.trajectory.back() == 224);
assert(rep.replans == 0 && rep.total_events == 0);   // static: no events
assert(rep.mismatches == 0);
assert(rep.verified_steps == 29);                    // initial + per-hop
```

Also verifies the trivial `start == goal` case:
```cpp
RouteNavigator same(g, 5, 5);
auto rep0 = same.run();
assert(rep0.reached_goal && rep0.moves == 0 && rep0.travelled_cost == 0.0);
```

```
PASSED (28 hops, cost=28, repair expansions=0)
```

Note: `repair expansions=0` — in a static world, the plan never becomes inconsistent,
so `compute_shortest_path()` returns immediately without touching the queue.

---

### Test 29 — `test29_navigator_random_dynamics()`

**Scenario:** 3 independent runs on 20×20 grid with seeds {11, 22, 33}, one random
event per hop.

**What it checks per seed:**
```cpp
assert(rep.mismatches == 0);             // D* Lite == Dijkstra at every hop
assert(!rep.step_limit_hit);             // terminated normally
assert(rep.reached_goal != rep.blocked); // exactly one terminal state
assert(rep.trajectory.front() == 0);
assert(rep.steps.size() == rep.moves);
assert(rep.trajectory.size() == rep.moves + 1);
// Every hop used a real edge:
for (size_t i = 1; i < rep.trajectory.size(); ++i)
    assert(g.get_edge(rep.trajectory[i-1], rep.trajectory[i]) != nullptr);
```

**Cumulative result:** 117 verified steps across 3 seeds, 3 reached goal, 0 blocked.

```
PASSED (117 steps verified; 3 reached, 0 blocked)
```

---

### Test 30 — `test30_navigator_detour_and_unreachable()`

Two sub-scenarios:

**Sub-scenario A — Forced Detour (6×6 grid, goal=35):**
- At step 0, the `EventProvider` blocks the 2nd and 3rd nodes on the optimal path.
- D* Lite must find an alternative route around the blockage.
- Assertions:
```cpp
assert(rep.reached_goal);
assert(rep.mismatches == 0);
assert(rep.total_events == 2 && rep.replans == 1);
assert(rep.travelled_cost >= 10.0 - 1e-6);   // detour costs at most +2
assert(rep.travelled_cost <= 12.0 + 1e-6);
```

**Sub-scenario B — Unreachable Goal (chain 0-1-2-3-4):**
- At step 0, edges (3,4) and (4,3) are blocked, making node 4 unreachable from anywhere in the chain.
- Agent is at 0, goal is 4. After moving to 1, it detects `!plan.path_found`.
- Assertions:
```cpp
assert(!rep.reached_goal);
assert(rep.blocked);
assert(rep.mismatches == 0);
assert(rep.moves == 1);   // one hop then blocked
```

```
PASSED (detour cost within [10,12]; unreachable chain detected)
```

---

### Test 31 — `test31_navigator_efficiency()`

**Scenario:** 40×40 grid (1,600 nodes), goal at 1599, random event every 3 hops (seed=2024).

**Core assertion — efficiency:**
```cpp
const std::size_t incremental = rep.initial_expansions + rep.incremental_expansions;
assert(rep.baseline_expansions > 0);
assert(incremental < rep.baseline_expansions);  // D* Lite MUST beat Dijkstra
```

Where:
- `incremental` = D* Lite initial search + sum of all repair expansions.
- `baseline_expansions` = sum of Dijkstra-from-scratch expansions at every hop (verify mode).

**Result:**
```
D* Lite: 1,600 expansions   (initial + all repairs combined)
Dijkstra: 74,258 expansions (per-hop from scratch, summed)
Reduction: 97.8%
```

D* Lite explored **46× fewer vertices** than re-running Dijkstra at each hop.

```
PASSED (D* Lite 1600 vs Dijkstra 74258 expansions, 97.8% fewer)
```

---

## 6. CMakeLists.txt Changes

`route_navigator.cpp` was added to the core static library:

```cmake
add_library(dynamic_route_core STATIC
    ...
    src/algorithms/route_navigator.cpp   # ← Phase 3 addition
    ...
)
```

The comment on `run_tests` was updated from "Phase 0 + Phase 1 + Phase 2" to "Phase 0 + Phase 1 + Phase 2 + Phase 3".

---

## 7. Test Summary Table

| Test | Phase | Name | Algorithm | Graph | Key Assertion | Status |
|---|---|---|---|---|---|---|
| 27 | P3 | Indexed queue invariant | LPA* + D\* Lite | Grid 20×20 | `queue_invariant_holds()` true at every step | PASS |
| 28 | P3 | Static world navigation | D\* Lite (RouteNavigator) | Grid 15×15 | cost==28, moves==28, mismatches==0 | PASS |
| 29 | P3 | Random dynamics | D\* Lite (RouteNavigator) | Grid 20×20 | mismatches==0, 3 seeds | PASS |
| 30 | P3 | Detour + unreachable | D\* Lite (RouteNavigator) | Grid 6×6 + chain-5 | Detour ∈[10,12]; chain blocked | PASS |
| 31 | P3 | Efficiency vs Dijkstra | D\* Lite (RouteNavigator) | Grid 40×40 | incremental < baseline (97.8% fewer) | PASS |

---

## 8. Key Design Decisions

### Why two bookkeeping vectors instead of a `std::unordered_map<NodeId, LPAKey>`?

The graph has a fixed, known node count `n`. Two `std::vector`s of size `n` use
contiguous memory and support **O(1) random access** by NodeId, with no hashing
overhead. A map would add hash computation and dynamic allocation.

### Why accumulate `km_` instead of re-keying the queue on every move?

Re-keying `|queue|` entries after each agent hop would cost `O(|queue| log |queue|)` —
potentially `O(n log n)` if most of the graph is inconsistent. The `km_` accumulator
reduces this to `O(1)` per move. The correctness follows from the triangle inequality
on the admissible heuristic.

### Why does `cross_check()` use a lambda (closure) rather than a method?

`cross_check()` captures `report`, `goal_`, and `graph_` by reference. A lambda
avoids passing them as parameters at every call site and keeps the verification
logic co-located with the loop. Since `NavigationReport` is modified in place, the
closure captures the live reference.

### Why is `start == goal` handled as a trivial special case?

If `start == goal`, D* Lite returns a 1-node path with cost 0 immediately (since
`rhs[goal] = 0 = g[goal]` after the first compute). The loop condition
`position != goal_` is false from the start, so the loop body never executes.
This is correct by construction — no special-casing needed in the loop.

### Why does Test 31 compare `incremental < baseline` instead of a fixed ratio?

A fixed ratio (e.g., "must be < 10%") would be brittle — the ratio depends on
graph topology, event frequency, and how many hops reach the goal vs. get blocked.
Asserting `incremental < baseline` (strict inequality) is the fundamental
correctness guarantee of incremental search: it should always be faster than
recomputing from scratch.

---

## 9. Efficiency Results

### Benchmark — Test 31 (40×40 Grid, Event Every 3 Hops)

| Metric | D* Lite (Incremental) | Dijkstra (Per-Hop) | Ratio |
|---|---|---|---|
| Total vertex expansions | **1,600** | **74,258** | **97.8% fewer** |

### Benchmark — Phase 3 effect on Phase 2 benchmarks (no regression)

All Phase 0–2 benchmark results are unchanged. The indexed-queue change is a pure
performance and correctness upgrade with no impact on output costs.

### Build & Run

```bash
# Build (WSL)
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4

# Run all 31 tests
./bin/run_tests

# Run performance benchmarks
./bin/run_benchmarks

# CTest integration
ctest --test-dir build --output-on-failure
```
