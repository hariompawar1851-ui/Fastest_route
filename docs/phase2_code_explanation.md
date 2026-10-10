# Phase 2 — Code Explanation & Implementation Reference

> **Status:** ✅ Complete — All 13 Phase 2 tests passing (Tests 14–26)

---

## Table of Contents

1. [Overview & Goals](#1-overview--goals)
2. [Part 1 — Large-Scale Invariant Stress Testing](#2-part-1--large-scale-invariant-stress-testing)
3. [Part 2 — Explainability & Diagnostics Engine](#3-part-2--explainability--diagnostics-engine)
4. [Part 3 — Advanced Traffic Simulator](#4-part-3--advanced-traffic-simulator)
5. [CMakeLists.txt Changes](#5-cmakeliststxt-changes)
6. [Test Summary Table](#6-test-summary-table)
7. [Key Design Decisions](#7-key-design-decisions)
8. [JSON Audit Log Schema](#8-json-audit-log-schema)

---

## 1. Overview & Goals

Phase 2 extended the DynamicRoute engine (which completed Phase 1 with 13 passing tests) across three sub-phases:

| Sub-Phase | Focus | Tests |
|---|---|---|
| **Part 1** | Large-scale invariant stress testing — prove correctness at real scale | 14–19 |
| **Part 2** | Explainability & Diagnostics Engine — JSON causal audit logging | 20–22 |
| **Part 3** | Advanced Traffic Simulator — realistic perturbation scenarios | 23–26 |

All 13 new tests (14–26) pass. The total project test suite is now **26 tests across all phases**.

---

## 2. Part 1 — Large-Scale Invariant Stress Testing

### File: `tests/test_phase2_part1.cpp`

**Core invariant under test:**
```
cost(incremental_algo) == cost(full_Dijkstra_recompute)   at every perturbation step
```

### Helper: `run_stress<AlgoT>()` (lines 61–96)

```cpp
template <typename AlgoT>
int run_stress(Graph& graph, AlgoT& algo, NodeId source, NodeId target,
               int n_steps, uint32_t seed, const string& label)
```

A **generic template function** that drives any algorithm (LPA\*, D\* Lite) through `n_steps` random perturbations. At each step:

1. `EventSimulator::generate_random_event(graph)` picks a random existing edge and assigns a random new weight (may be INF = blockage).
2. `algo.on_edge_changed(from, to, new_weight)` notifies the incremental algorithm.
3. `algo.compute_shortest_path()` returns the incremental result.
4. `Dijkstra::find_shortest_path(graph, source, target)` computes ground truth from scratch.
5. `|inc.total_cost − ref.total_cost| < 1e-6` is checked; if violated, a diagnostic is printed.

Returns the count of steps where the invariant held.

**Why a template?** Both `LifelongPlanningAStar` and `DStarLite` share the same API surface (`on_edge_changed` + `compute_shortest_path`), so one template covers both without code duplication.

### Helper: `run_rr_stress()` (lines 105–150)

```cpp
int run_rr_stress(Graph& graph, RamalingamReps& rr, NodeId source, NodeId target,
                  int n_steps, uint32_t seed, vector<size_t>* invalidated_counts)
```

A **non-template variant** for `RamalingamReps` because RR has a different API:
- Uses `on_edge_increase` / `on_edge_decrease` (not a single `on_edge_changed`)
- Queries `rr.get_distance(target)` rather than `compute_shortest_path()`
- Optionally records `rr.nodes_invalidated_last()` for locality ratio analysis

### Tests 14–19 Summary

| Test | Algorithm | Graph | Steps | Key Assertion |
|---|---|---|---|---|
| **Test 14** | LPA\* | Grid 30x30 (900 nodes) | 1,000 | `cost(LPA*) == cost(Dijkstra)` at every step |
| **Test 15** | D\* Lite | Grid 30x30 (900 nodes) | 1,000 | Same invariant for D\* Lite |
| **Test 16** | RR | Grid 20x20 (400 nodes) | 500 | Same invariant for RR |
| **Test 17** | LPA\* | RGG 200 nodes (r=0.18) | 500 | Cost invariant + `PathValidator::validate_walk` on every result |
| **Test 18** | All 3 | Grid 50x50 (2,500 nodes) | 200 | Three-way: LPA\* == D\*Lite == RR == Dijkstra simultaneously |
| **Test 19** | RR | Sparse random 300 nodes | 300 | Cost invariant + `avg_invalidated / num_nodes < 0.15` (15%) |

### Test 17 — Walk Validation Detail

Beyond cost matching, Test 17 calls `PathValidator::validate_walk()` which checks that every consecutive pair `(path[i], path[i+1])` is a real edge in the graph and that the summed edge weights equal `inc.total_cost`.

### Test 18 — Three-Way Sync Detail

Three independent graph copies (`g_lpa`, `g_dstr`, `g_rr`) are kept in lockstep by seeding three `EventSimulator` objects with the **same seed**. Because the graphs are initially identical, the same random edge sequence is produced on all three.

### Test 19 — RR Locality Ratio

The Ramalingam-Reps algorithm's core value is **bounded local repair**. Test 19 measures:

```
locality = avg_invalidated / num_nodes
```

Actual observed result: **~0.23%** — the algorithm explores less than 0.25% of the graph per perturbation on a sparse 300-node graph.

---

## 3. Part 2 — Explainability & Diagnostics Engine

### 3.1 `include/diagnostics/audit_log.hpp` + `src/diagnostics/audit_log.cpp`

#### `EdgeChange` Struct

Records what changed: which edge, from what weight to what weight. `is_blockage` is derived: `true` when `new_weight == std::numeric_limits<double>::infinity()`.

#### `AuditEntry` Struct

The fundamental unit of the diagnostics system — one entry per perturbation step per algorithm:

| Field | Type | Description |
|---|---|---|
| `step` | int | Zero-indexed perturbation step |
| `algorithm` | string | "LPA*", "D*Lite", "RR", "Dijkstra" |
| `edge_change` | EdgeChange | What edge changed and by how much |
| `nodes_invalidated` | size_t | Subtree vertices cut from SPT (RR only) |
| `nodes_relaxed` | size_t | Vertices re-relaxed during repair (RR only) |
| `nodes_expanded` | size_t | PQ pops during query |
| `repair_time_us` | double | Wall-clock time for incremental update (microseconds) |
| `path_found` | bool | Whether a finite-cost path exists |
| `final_cost` | double | Shortest path cost |
| `path` | vector<NodeId> | Source to target path |
| `correct` | bool | Verified against Dijkstra |
| `cost_delta` | double | `|incremental_cost - dijkstra_cost|` |

#### `AuditLogger` Class

Accumulates `AuditEntry` objects and provides serialization:

| Method | Purpose |
|---|---|
| `record(AuditEntry)` | Append an entry |
| `operator[](i)` | Random access |
| `write_ndjson(ostream&)` | Serialize all entries as Newline-Delimited JSON |
| `save_ndjson(filepath)` | Write NDJSON to disk |
| `load_ndjson(filepath)` | Parse NDJSON from disk into a new AuditLogger |
| `compute_stats(node_count)` | Aggregate: avg repair time, locality ratio, correctness count |

**Why NDJSON?** JSON has no native infinity literal, so INF weights are encoded as the string `"INF"`. NDJSON (one JSON object per line) is streaming-friendly, easy to grep, and directly ingestible by tools like `jq` or Python.

#### `ScopedTimer` — RAII Nanosecond Timer

```cpp
class ScopedTimer {
    explicit ScopedTimer(double& out_us);
    ~ScopedTimer();  // records elapsed nanoseconds / 1000.0 into out_us
};
```

Uses `chrono::high_resolution_clock`. Timing starts at construction and written to `out_us` on destruction. Used by `CausalReporter` to time every `on_edge_changed()` call.

#### `compute_stats()` — Locality Ratio

`avg_locality_ratio = avg_nodes_invalidated / graph_node_count`

This is the key metric proving RR's local repair efficiency — a value near 0 means RR repairs the SPT by touching almost no nodes, vs. Dijkstra which always visits 100%.

---

### 3.2 `include/diagnostics/causal_reporter.hpp` + `src/diagnostics/causal_reporter.cpp`

#### Template `CausalReporter<AlgoT>`

A **thin instrumentation wrapper** around any incremental algorithm exposing:
- `void on_edge_changed(NodeId, NodeId, double)`
- `SearchResult compute_shortest_path()`

The wrapper intercepts two calls:
1. **`on_edge_changed()`** — times via `ScopedTimer`, harvests diagnostic counters via SFINAE
2. **`compute_shortest_path()`** — calls algorithm, validates against Dijkstra, appends `AuditEntry`

#### SFINAE Counter Detection

Not all algorithms expose `nodes_invalidated_last()`. The reporter detects this at **compile time** using SFINAE void_t:

```cpp
template <typename T, typename = void>
struct has_invalidated_counter : std::false_type {};

template <typename T>
struct has_invalidated_counter<T,
    std::void_t<decltype(std::declval<const T&>().nodes_invalidated_last())>>
    : std::true_type {};
```

Then `get_invalidated()` uses `if constexpr` to call it only when it exists. This means `CausalReporter<LifelongPlanningAStar>` returns 0 with zero overhead, while a future algorithm that implements the counter would use it automatically.

#### `RRCausalReporter` — Concrete RR Wrapper

Because `RamalingamReps` has a different API (separate increase/decrease functions, `get_distance()` not `compute_shortest_path()`), a dedicated non-template class is used:

```cpp
void RRCausalReporter::record_step(NodeId from, NodeId to,
                                    double old_weight, double new_weight)
{
    // Time the update (dispatch to increase or decrease)
    { ScopedTimer timer(e.repair_time_us);
      if (new_weight >= old_weight) rr_.on_edge_increase(from, to, old_weight, new_weight);
      else                          rr_.on_edge_decrease(from, to, old_weight, new_weight); }

    // Harvest RR-specific counters
    e.nodes_invalidated = rr_.nodes_invalidated_last();
    e.nodes_relaxed     = rr_.nodes_relaxed_last();

    // Query result
    double dist = rr_.get_distance(target_);
    auto   path = rr_.get_path(target_);

    // Validate against Dijkstra, then append to logger
}
```

### Tests 20–22 Summary

| Test | What is tested |
|---|---|
| **Test 20** | `CausalReporter<LPA*>` over 50 steps — every entry has `correct=true`, `cost_delta < 1e-6`, valid path, non-negative `repair_time_us` |
| **Test 21** | Save 30 D\*Lite entries to NDJSON file, reload, verify every scalar field round-trips exactly |
| **Test 22** | `RRCausalReporter` over 100 steps — `stats.correct_steps == 100`, locality ratio < 50% (typically < 5%) |

---

## 4. Part 3 — Advanced Traffic Simulator

### 4.1 `CongestionWave` (traffic_scenario.hpp/.cpp)

**Models:** A motorway accident that progressively blocks corridor segments in sequence.

- Stores `current_step_` to track progress through the corridor
- Saves `original_weights_` so `reset()` can restore all segments
- `step()` either calls `set_edge_blocked(u, v, true)` for full blockage, or `update_edge_weight(u, v, old_w * factor)` for partial congestion
- Returns a `PerturbationEvent` so calling code can notify algorithms

### 4.2 `DisasterZone` (traffic_scenario.hpp/.cpp)

**Models:** A geographic area (flash flood, road works) where all edges within a bounding box are simultaneously degraded.

- `node_in_zone(u)` checks `x in [x_min, x_max] AND y in [y_min, y_max]`
- `apply()` iterates all edges, saves original weights for affected ones, and multiplies by the given factor
- `clear()` restores all saved weights — algorithm receives recovery `PerturbationEvent` list

### 4.3 `RecoveryEvent` (traffic_scenario.hpp/.cpp)

**Models:** Temporary perturbations that auto-recover after a configurable TTL (time-to-live in ticks).

```cpp
struct PendingRecovery {
    NodeId from, to;
    double original_weight;
    int    recover_at_tick;   // restore when current_tick_ == this
    bool   was_blocked;
};
deque<PendingRecovery> pending_;
```

- `record_perturbation(event)` sets `recover_at_tick = current_tick_ + ttl_`
- `tick()` increments `current_tick_`, drains expired recoveries from the front, restores edges, returns recovery events

### 4.4 `ScenarioRunner` (scenario_runner.hpp/.cpp)

A **timeline orchestrator** combining all scenario types into a single simulation.

**Registration API:**
```cpp
runner.add_congestion_wave(wave, start_tick, step_interval);
runner.add_disaster_zone(zone, apply_tick, clear_tick);
runner.add_recovery_tracker(recovery);
runner.add_random_perturbations(n_events, start_tick, end_tick, seed);
runner.register_lpa(lpa, logger, "LPA*");
```

**`run(total_ticks)` loop — at each tick t:**
1. Wave step if `t >= start_tick && (t-start) % interval == 0`
2. Zone apply if `t == apply_tick`
3. Zone clear if `t == clear_tick`
4. Apply pre-generated random events at tick t
5. `notify_and_record()` dispatches to all registered algorithms
6. `recovery.tick()` — apply any auto-restored edges

Returns a `ScenarioRunnerReport` with total events, recoveries, and per-algorithm correctness stats.

### Tests 23–26 Summary

| Test | Scenario | Algorithm | Key Assertion |
|---|---|---|---|
| **Test 23** | CongestionWave: 6-segment corridor, 15x15 grid | LPA\* | All 6 steps correct; after `reset()`, Dijkstra cost == 28.0 |
| **Test 24** | DisasterZone: center 5x5 block x8, 20x20 grid | D\* Lite | During: cost >= 38; after `clear()`: cost == 38 |
| **Test 25** | RecoveryEvent: TTL=5, 20 perturbations, 12x12 grid | LPA\* | After TTL+2 ticks: `pending_count() == 0`, all edges restored |
| **Test 26** | Composite: wave + zone + random + recovery, 30 ticks | LPA\* + 3-way | 100% correctness; 3-way agreement verified over 50 additional steps |

---

## 5. CMakeLists.txt Changes

Three new test executables added and compiled from `dynamic_route_core`:

```cmake
add_executable(run_tests_phase2_part1 tests/test_phase2_part1.cpp)
add_executable(run_tests_phase2_part2 tests/test_phase2_part2.cpp)
add_executable(run_tests_phase2_part3 tests/test_phase2_part3.cpp)

add_test(NAME Phase2Part1StressTests      COMMAND run_tests_phase2_part1)
add_test(NAME Phase2Part2DiagnosticsTests COMMAND run_tests_phase2_part2)
add_test(NAME Phase2Part3TrafficSimTests  COMMAND run_tests_phase2_part3)
```

All new source files (`audit_log.cpp`, `causal_reporter.cpp`, `traffic_scenario.cpp`, `scenario_runner.cpp`) are compiled into the shared `dynamic_route_core` static library.

---

## 6. Test Summary Table

| Test | Sub-Phase | Name | Algorithm | Graph | Steps | Status |
|---|---|---|---|---|---|---|
| 14 | P2.1 | LPA\* 1K perturbations | LPA\* | Grid 30x30 | 1,000 | PASS |
| 15 | P2.1 | D\* Lite 1K perturbations | D\* Lite | Grid 30x30 | 1,000 | PASS |
| 16 | P2.1 | RR 500 perturbations | RR | Grid 20x20 | 500 | PASS |
| 17 | P2.1 | LPA\* RGG + walk validation | LPA\* | RGG 200 nodes | 500 | PASS |
| 18 | P2.1 | Three-way agreement | LPA\* + D\*L + RR | Grid 50x50 | 200 | PASS |
| 19 | P2.1 | RR locality < 15% | RR | Sparse 300 nodes | 300 | PASS (~0.23%) |
| 20 | P2.2 | CausalReporter<LPA\*> entries | LPA\* | Grid 10x10 | 50 | PASS |
| 21 | P2.2 | NDJSON round-trip | D\* Lite | Grid 8x8 | 30 | PASS |
| 22 | P2.2 | Aggregate stats + RR reporter | RR | Grid 15x15 | 100 | PASS |
| 23 | P2.3 | CongestionWave | LPA\* | Grid 15x15 | 6 | PASS |
| 24 | P2.3 | DisasterZone apply/clear | D\* Lite | Grid 20x20 | 2 | PASS |
| 25 | P2.3 | RecoveryEvent TTL=5 | LPA\* | Grid 12x12 | 20+7 | PASS |
| 26 | P2.3 | ScenarioRunner composite | LPA\* + 3-way | Grid 15x15/10x10 | 30+50 | PASS |

---

## 7. Key Design Decisions

### Why template `CausalReporter<AlgoT>` vs. a virtual base class?

A virtual interface would require all algorithms to inherit it, adding coupling and vtable overhead. The template is zero-overhead — the compiler specializes at compile time. SFINAE handles optional counter methods without runtime dispatch.

### Why a separate `RRCausalReporter` instead of a template specialization?

`RamalingamReps` has a fundamentally different API (two update functions, `get_distance()` not `compute_shortest_path()`). A concrete class is simpler and more readable than a partial template specialization.

### Why NDJSON over standard JSON?

1. **Streaming** — write/read one entry at a time without buffering the entire log
2. **Grep-friendly** — each line is self-contained: `grep '"algorithm":"RR"'` extracts RR steps
3. **INF encoding** — `"INF"` string sentinel is round-trippable; JSON has no infinity literal
4. **Zero dependencies** — the hand-rolled parser handles our fixed schema with no external libraries

### Why three graph copies in Tests 18 and 26?

Each incremental algorithm holds a reference to its graph and tracks live state. Sharing one graph between LPA\*, D\*Lite, and RR would not isolate their internal states. Separate copies guarantee that each algorithm's invariant is tested independently with identical perturbation sequences.

### Why linear scan in `DisasterZone::apply()`?

At the scale of test graphs (up to 2,500 nodes), linear scan is fast. The design is correct-by-construction. A spatial index (k-d tree) would be a Phase 5 optimization for 100k+ node networks.

---

## 8. JSON Audit Log Schema

Every NDJSON line emitted by `AuditLogger::write_ndjson()`:

```json
{
  "step": 42,
  "algorithm": "LPA*",
  "edge_change": {
    "from": 5, "to": 12,
    "old_w": 1.0, "new_w": "INF",
    "is_blockage": true
  },
  "nodes_invalidated": 0,
  "nodes_relaxed": 0,
  "nodes_expanded": 7,
  "repair_time_us": 2.35,
  "path_found": true,
  "final_cost": 14.5,
  "path": [0, 3, 7, 11, 14],
  "correct": true,
  "cost_delta": 0.0
}
```

| Field | Type | Notes |
|---|---|---|
| `step` | int | Zero-indexed step number |
| `algorithm` | string | "LPA\*", "D\*Lite", "RR" |
| `edge_change.old_w` / `new_w` | float or "INF" | INF encoded as string sentinel |
| `is_blockage` | bool | true when new_w == INF |
| `nodes_invalidated` | int | SPT vertices cut (RR only; 0 for LPA\*/D\*L) |
| `nodes_relaxed` | int | Vertices re-relaxed during repair (RR only) |
| `nodes_expanded` | int | Priority queue pops during query |
| `repair_time_us` | float | Wall-clock time for incremental update |
| `correct` | bool | true if `cost_delta < 1e-6` vs Dijkstra |
| `cost_delta` | float | Absolute difference from Dijkstra ground truth |

---

## Build & Run Commands

```bash
# Build all targets (from project root)
cmake --build build -j4

# Run Phase 2 tests individually
./bin/run_tests_phase2_part1   # Tests 14-19
./bin/run_tests_phase2_part2   # Tests 20-22
./bin/run_tests_phase2_part3   # Tests 23-26

# Run complete suite (all 26 tests)
./bin/run_tests && ./bin/run_tests_phase2_part1 && ./bin/run_tests_phase2_part2 && ./bin/run_tests_phase2_part3

# Or via CTest
cd build && ctest --verbose
```
