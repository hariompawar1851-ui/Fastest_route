# Phase 3 — Indexed-Queue Incremental Search & Moving-Agent D* Lite

> **Status:** ✅ Complete — All 5 Tests Passing (Tests 27–31)

---

## Overview

Phase 3 upgrades the incremental search core and adds a **moving-agent navigation layer** on top of D* Lite. It builds directly on the verified Phase 2 engine (26/26 tests passing) and is driven by two goals:

1. **Robustness** — replace the naive linear-scan priority queue with an *indexed* queue that guarantees `O(log n)` erases, and add a `queue_invariant_holds()` checker that can verify correctness at runtime.
2. **Real navigation** — implement `RouteNavigator`, a vehicle simulation that physically moves along the current route while the road network mutates, using D* Lite to repair only the affected region at each hop.

---

## Deliverables

| File | Type | Description |
|---|---|---|
| `include/algorithms/lpa_star.hpp` | **Updated** | Added `queue_invariant_holds()`, `total_expansions()`, `queue_size()` diagnostics; indexed bookkeeping vectors |
| `src/algorithms/lpa_star.cpp` | **Updated** | `update_vertex()` now erases by exact `(key, node)` pair in `O(log n)`; expansion counter |
| `include/algorithms/d_star_lite.hpp` | **Updated** | Moving-agent API: `update_start()`, `km()`, `current_start()`, `goal()`, invariant checker |
| `src/algorithms/d_star_lite.cpp` | **Updated** | Indexed-queue erase; `update_start()` accumulates `km_` |
| `include/algorithms/route_navigator.hpp` | **New** | `RouteNavigator` class, `NavigationReport`, `NavigationStep` structs |
| `src/algorithms/route_navigator.cpp` | **New** | Moving-agent simulation loop: hop → update_start → apply events → repair |
| `CMakeLists.txt` | **Updated** | `route_navigator.cpp` added to `dynamic_route_core` |
| `tests/test_main.cpp` | **Updated** | Tests 27–31 added (Phase 3 section) |

---

## Sub-Task 1 — Indexed Priority Queue (LPA* & D* Lite)

**Problem:** The previous implementation removed vertices from `std::set<pair<LPAKey, NodeId>>` via a linear scan — `O(n)` worst case.

**Solution:** Maintain two bookkeeping side-tables:
- `in_open_list_[u]` — bool: is vertex u currently queued?
- `queued_key_[u]` — the key used when u was last inserted.

On removal, erase the exact `{queued_key_[u], u}` pair — guaranteed `O(log n)`.

**Queue Invariant:**
```
queue_invariant_holds() returns true iff:
  ∀u:  (g[u] ≠ rhs[u])  ⟺  (u ∈ open_list_)
       if in list: open_list_.find({queued_key_[u], u}) succeeds
  AND: flagged_count == open_list_.size()
```

---

## Sub-Task 2 — D* Lite Moving-Agent API (`update_start` + `km_`)

When the agent moves from `last_start_` to `new_start`, all heuristic estimates shift. Instead of re-keying the whole queue, a scalar accumulator `km_` is maintained:

```
update_start(new_start):
    km_ += h(last_start_, new_start)
    last_start_ = new_start
    start_ = new_start
```

All future `calculate_key(u)` calls add `km_` to the first component — **O(1) overhead per move**, queue stays valid.

---

## Sub-Task 3 — RouteNavigator

Per-hop loop:
1. **Move** one edge along `plan.path[1]`.
2. **`dstar.update_start(new_position)`** — advance the D* Lite start.
3. **Apply events** via `provider(step, graph, position)`, forward each to `dstar.on_edge_changed()`.
4. **`dstar.compute_shortest_path()`** — incremental repair only.
5. **Verify (optional)** — cross-check against from-scratch Dijkstra.

---

## Tests Added (27–31)

| Test | Name | Graph | Steps | Key Assertion |
|---|---|---|---|---|
| **27** | Indexed-queue invariant + Dijkstra agreement | Grid 20×20 (400 nodes) | 300 | `queue_invariant_holds()` true every step; costs == Dijkstra |
| **28** | Static world: optimal route followed exactly | Grid 15×15 (225 nodes) | 28 hops | `reached_goal`, cost==28.0, replans==0, mismatches==0 |
| **29** | Random dynamics: D* Lite == Dijkstra per hop | Grid 20×20 (400 nodes) | 3 seeds | `mismatches==0`; exactly one terminal state per run |
| **30** | Forced detour + unreachable goal | Grid 6×6 + chain-5 | 1 event each | Detour ∈ [10, 12]; chain `blocked` after 1 move |
| **31** | D* Lite expansions < Dijkstra-per-hop baseline | Grid 40×40 (1,600 nodes) | Full traversal | `incremental < baseline`; ≥ 95% reduction observed |

---

## Build & Run

```bash
# Configure + build (WSL / Linux)
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4

# Run all 31 tests
./bin/run_tests

# Run benchmarks
./bin/run_benchmarks
```

---

## Completion Criteria

| Criterion | Status |
|---|---|
| `queue_invariant_holds()` passes every step in Test 27 | ✅ |
| `RouteNavigator` reaches goal in static world with zero mismatches (Test 28) | ✅ |
| D* Lite == Dijkstra at every hop, 3 seeds (Test 29) | ✅ |
| Detour cost in bounds; chain unreachable detected (Test 30) | ✅ |
| Incremental expansions < Dijkstra baseline (Test 31) | ✅ |
| **ALL 31 TESTS PASS** | ✅ |
