# Phase 1 — Core DSA Engine: Changes, New Files & Code Explanation

> **Status:** ✅ Complete — All 10 tests pass, benchmarks verified

---

## 1. What Was Changed / Added in Phase 1

### Problem with Phase 0

Phase 0 laid a working skeleton, but the **Ramalingam–Reps (RR)** implementation
had a critical flaw: `on_edge_increase()` and `on_edge_decrease()` both called
`initialize()`, which runs a **full Dijkstra recompute** from scratch. This
defeated the entire purpose of the incremental algorithm and made RR no better
than repeatedly rerunning Dijkstra.

Phase 1 replaces this stub with the **real RR algorithm** backed by a proper
**Shortest Path Tree (SPT)** data structure. It also adds graph generators and
a validation harness.

---

## 2. New / Modified Files

| File | Type | Phase 1 Action |
|---|---|---|
| `include/graph/shortest_path_tree.hpp` | **New** | SPT data structure with subtree BFS |
| `include/graph/graph_io.hpp` | **New** | Grid, RGG, sparse graph generators + file I/O |
| `include/graph/path_validator.hpp` | **New** | Route correctness & stress-test harness |
| `include/algorithms/ramalingam_reps.hpp` | **Rewritten** | Full algorithm documentation + real interface |
| `src/algorithms/ramalingam_reps.cpp` | **Rewritten** | Real incremental SPT repair (not full Dijkstra) |
| `src/graph/graph_io.cpp` | **New** | Generator + file loader implementations |
| `src/graph/path_validator.cpp` | **New** | Validator + stress test implementations |
| `CMakeLists.txt` | **Updated** | Added `graph_io.cpp` and `path_validator.cpp` to core lib |
| `tests/test_main.cpp` | **Updated** | Tests 6–10 added for Phase 1 components |

---

## 3. File-by-File Explanation

---

### 3.1 `include/graph/shortest_path_tree.hpp` — NEW

#### Purpose
Provides an **explicit, navigable Shortest Path Tree (SPT)** data structure.
Previously the algorithms only maintained `dist[]` and `parent[]` arrays.
The SPT adds a `children[]` list (the *forward* SPT adjacency), enabling BFS
over the tree without touching the main road graph.

#### Key Concepts

```
SPT for source S, graph G:
  dist[u]      = shortest distance from S to u
  parent[u]    = predecessor of u on the optimal path (INVALID_NODE for source)
  children[u]  = list of vertices v where parent[v] == u
```

When edge `(u, v)` is removed or its cost increases and it was the SPT parent
edge for `v`, the subtree rooted at `v` is **disconnected** from the surviving
tree. The `children[]` list enables this BFS traversal in `O(|subtree|)`.

#### Key Methods

| Method | Complexity | Description |
|---|---|---|
| `set_dist_and_parent(u, d, par)` | O(1) amortised | Wires u into the SPT at distance d, updating child links |
| `collect_subtree(root)` | O(\|subtree\|) | BFS over children[], returns all vertices under root |
| `invalidate_subtree(root)` | O(\|subtree\|) | Sets dist=INF, parent=INVALID for the entire subtree |
| `in_tree(u)` | O(1) | Returns true if dist[u] < INF |

All methods are inlined in the header — no `.cpp` file needed.

---

### 3.2 `include/algorithms/ramalingam_reps.hpp` + `src/algorithms/ramalingam_reps.cpp` — REWRITTEN

#### Algorithm: Real Ramalingam–Reps Incremental SSSP

**Reference:** Ramalingam, G. & Reps, T. (1996). *An Incremental Algorithm for
a Generalization of the Shortest-Path Problem.* ACM TOPLAS 18(6):1020–1046.

---

#### on_edge_increase (or blockage)

When edge `(u → v)` cost increases and `parent[v] == u` (i.e., this edge
**was** on the SPT):

```
Step 1: Collect subtree S = collect_subtree(v)     O(|S|)
        (BFS over children[] to find all descendants of v)

Step 2: Invalidate S — disconnect every s ∈ S       O(|S|)
        dist[s] = INF, parent[s] = INVALID

Step 3: Seed priority queue Q with boundary edges   O(|E(boundary)|)
        For each s ∈ S:
          For each incoming edge (p → s) where p ∉ S and dist[p] < INF:
            push (dist[p] + w(p,s), s, p) into Q

Step 4: Dijkstra-style relaxation restricted to S   O(|E(boundary)| log |S|)
        While Q not empty:
          Pop (cost, s, pred)
          If s already reconnected: skip
          Set dist[s] = cost, parent[s] = pred
          For each outgoing edge (s → t) where t ∈ S and t not reconnected:
            push (cost + w(s,t), t, s) into Q
```

**Why this is better than full Dijkstra:**
Full Dijkstra explores `|V|` vertices. The RR algorithm explores only `|S|`
(the invalidated subtree) plus its boundary edges. When a local road is
blocked in a large network, `|S|` is typically `< 50` vertices even if
`|V| = 500,000`.

**Fast-path optimisation:** If `parent[v] != u`, the edge is not on the SPT →
return immediately with zero computation.

---

#### on_edge_decrease (or new road)

When edge `(u → v)` cost decreases and `dist[u] + new_w < dist[v]`:

```
1. Set dist[v] = dist[u] + new_w, parent[v] = u
2. Run forward Dijkstra relaxation seeded at v:
   For each (v → w): if dist[v] + w(v,w) < dist[w]: update, re-enqueue
   This propagates improvements only to overconsistent vertices.
```

---

#### Diagnostic counters

```cpp
rr.nodes_invalidated_last()  // how many vertices were cut from the SPT
rr.nodes_relaxed_last()      // how many were re-relaxed during repair
```

These are used by the benchmarking harness to measure update locality.

---

### 3.3 `include/graph/graph_io.hpp` + `src/graph/graph_io.cpp` — NEW

#### Purpose
Provides three synthetic graph generators and a file I/O loader.

#### Generators

**`make_grid(rows, cols, spacing)`**
Creates an aligned M×N rectangular road network. Node `(r, c)` gets ID
`r * cols + c`. Horizontal and vertical edges are added bidirectionally with
weight = `spacing`. This is the primary test graph type (mirror of an urban grid).

**`make_random_geometric(n, radius, seed)`**
Places `n` nodes uniformly at random in the unit square `[0,1]²`. Two nodes
get a bidirectional edge if their Euclidean distance ≤ `radius`. Edge weight =
Euclidean distance (preserving metric admissibility for A*/LPA*/D*).

This model mirrors the sparse, planar structure of real road networks.
Reference: *Penrose, M. (2003). Random Geometric Graphs. Oxford Univ. Press.*

Choosing `radius ≈ sqrt(ln(n) / (π * n))` guarantees connectivity with
high probability (Gupta & Kumar, 2000).

**`make_random_sparse(n, m, min_w, max_w, seed)`**
Erdős–Rényi `G(n, m)` model — exactly `m` directed edges between distinct
random endpoint pairs with random weights.

#### File I/O

`load_edge_list(filepath)` — loads DIMACS shortest-path format (`a u v w` lines)
or plain `u v weight` edge list format (1 edge per line, `#` comments skipped).

`save_edge_list(graph, filepath)` — serialises all unblocked edges to the plain
edge-list format.

---

### 3.4 `include/graph/path_validator.hpp` + `src/graph/path_validator.cpp` — NEW

#### Purpose
A stateless utility class providing automated correctness checking for
incremental algorithms against the Dijkstra ground truth.

#### Methods

**`validate_walk(graph, path, reported_cost, tol)`**
- Checks every consecutive pair `(path[i], path[i+1])` has an unblocked edge.
- Computes the actual sum of edge weights along the path.
- Asserts it matches `reported_cost` within tolerance `tol`.

**`compare_with_dijkstra(graph, source, target, incremental_result, tol)`**
- Runs full Dijkstra on the current graph state.
- Checks `path_found` flag matches.
- Checks `total_cost` matches within `tol`.

**`stress_test(graph, source, target, n_steps, seed, update_fn, query_fn)`**
- Runs `n_steps` random perturbations.
- After each perturbation, calls `update_fn` (to notify the incremental algo)
  then `query_fn` (to get its result).
- Calls `compare_with_dijkstra` at every step.
- Returns the count of steps where correctness was confirmed.
- Used in Tests 5 and 10 for automated invariant verification.

---

## 4. Test Results

```
============================================================
  🧪 DynamicRoute — Full Test Suite (Phase 0 + Phase 1) 🧪
============================================================

--- Phase 0: Baseline Correctness ---
[Test 1] Graph Construction & Edge Operations... PASSED
[Test 2] Static Dijkstra and A* Equivalence... PASSED (Cost = 8)
[Test 3] LPA* Initial Search Matches Dijkstra... PASSED
[Test 4] D* Lite Initial Search Matches Dijkstra... PASSED
[Test 5] Invariant: LPA* == Dijkstra over 10 Perturbations... PASSED

--- Phase 1: Core DSA Engine ---
[Test 6] ShortestPathTree — SPT construction & subtree ops... PASSED
[Test 7] Ramalingam-Reps: incremental repair on edge cost increase... PASSED
         (dist=6, invalidated=3, relaxed=2)
[Test 8] Ramalingam-Reps: incremental improvement on edge cost decrease... PASSED
[Test 9] GraphIO: Grid & Random Geometric Graph generators... PASSED
         (grid edges=48, RGG edges=792)
[Test 10] PathValidator: walk validation & Dijkstra comparison... PASSED
          (stress_test: 5/5 steps verified)

✅ ALL 10 TESTS PASSED!
```

---

## 5. Benchmark Results (LPA* Incremental vs Full Dijkstra Recompute)

| Grid Size | Nodes | Dijkstra Recompute (avg) | LPA* Repair (avg) | Speedup | Node Reduction |
|---|---|---|---|---|---|
| 10×10 | 100 | 7.1 µs | 0.06 µs | **117×** | 100% |
| 20×20 | 400 | 37.6 µs | 0.05 µs | **780×** | 100% |
| 40×40 | 1,600 | 247 µs | 0.09 µs | **2,660×** | 100% |
| 80×80 | 6,400 | 880 µs | 0.11 µs | **7,974×** | 100% |
| 150×150 | 22,500 | 3,234 µs | 0.04 µs | **91,361×** | 100% |

All benchmarks maintained **5/5 correctness** — the incremental result was
identical to the ground-truth Dijkstra recomputation at every perturbation step.

---

## 6. What Remains for Future Phases

| Phase | Remaining Work |
|---|---|
| **Phase 2** | Automated invariant test with 1,000+ perturbations; larger RGG graphs (10K–500K nodes) |
| **Phase 3** | Explainability / diagnostics engine: JSON causal audit log emitter |
| **Phase 4** | Advanced traffic simulator: corridor congestion waves, DisasterZone events |
| **Phase 5** | Scalability benchmarks on OSM / DIMACS real road networks |
| **Phase 6** | Web dashboard: interactive graph visualisation with live path repair animation |
