# Phase 2 — Expanded Correctness, Diagnostics & Advanced Simulation

> **Status:** ✅ Complete — All 3 Parts Done (Tests 14–26 passing)

---

## Overview

Phase 2 extends DynamicRoute across three focused sub-phases, building on the
fully validated Phase 1 engine (13/13 tests passing):

---

## Part 1 — Large-Scale Invariant Stress Testing  ✅ Complete

**Goal:** Prove algorithm correctness at real scale (1,000+ perturbations,
large Random Geometric Graphs up to ~500 nodes in fast CI).

### Deliverables

| File | Type | Description |
|---|---|---|
| `tests/test_main.cpp` | **Updated** | All 26 tests (Phases 0, 1, and 2 Parts 1–3) consolidated into single suite |
| `docs/phase2_plan.md` | **Updated** | This roadmap |
| `CMakeLists.txt` | **Updated** | Single test target `run_tests` |

### Tests Added

| Test | Algorithm | Graph | Perturbations | Invariant Checked |
|---|---|---|---|---|
| 14 | LPA* | Grid 30×30 (900 nodes) | 1,000 | cost(LPA*) == cost(Dijkstra) at every step |
| 15 | D* Lite | Grid 30×30 (900 nodes) | 1,000 | cost(D*Lite) == cost(Dijkstra) at every step |
| 16 | RR | Grid 20×20 (400 nodes) | 500 | cost(RR) == cost(Dijkstra) at every step |
| 17 | LPA* | RGG 200 nodes | 500 | cost(LPA*) == cost(Dijkstra) + path walk valid |
| 18 | All 3 | Grid 50×50 (2,500 nodes) | 200 | Three-way: LPA* == D*Lite == RR == Dijkstra |
| 19 | RR | Sparse random 300 nodes | 300 | Locality ratio: avg invalidated < 15% of V |

### Build & Run

```bash
cd build
cmake ..
cmake --build .
./bin/run_tests
```

---

## Part 2 — Explainability & Diagnostics Engine  ✅ Complete

**Goal:** Add a JSON causal audit log emitter recording per-perturbation:
- Which edge changed and by how much
- Which vertices were invalidated / relaxed
- Wall-clock time for the repair
- The final reconstructed path

### Planned Deliverables

| File | Type | Description |
|---|---|---|
| `include/diagnostics/audit_log.hpp` | **New** | `AuditEntry` struct + `AuditLogger` class |
| `src/diagnostics/audit_log.cpp` | **New** | JSON serialisation of audit entries |
| `include/diagnostics/causal_reporter.hpp` | **New** | Wraps any algorithm with logging hooks |
| `tests/test_main.cpp` | **Updated** | Tests 20–22: audit log correctness |
| `docs/phase2_part2_design.md` | **New** | Design spec and JSON schema |

### JSON Output Schema (draft)

```json
{
  "step": 42,
  "edge_change": { "from": 5, "to": 12, "old_w": 1.0, "new_w": "INF" },
  "algorithm": "LPA*",
  "nodes_invalidated": 7,
  "nodes_relaxed": 4,
  "repair_time_us": 2.3,
  "final_cost": 14.5,
  "path": [0, 3, 7, 11, 14]
}
```

---

## Part 3 — Advanced Traffic Simulator  ✅ Complete

**Goal:** Replace the simple random-edge perturbation generator with a realistic
traffic scenario engine:

1. **Corridor Congestion Waves** — blockage propagates along a road sequence.
2. **DisasterZone Events** — a rectangular region simultaneously degrades all edges.
3. **Recovery Events** — blocked/degraded edges restore after a configurable TTL.

### Planned Deliverables

| File | Type | Description |
|---|---|---|
| `include/simulation/traffic_scenario.hpp` | **New** | `CongestionWave`, `DisasterZone`, `RecoveryEvent` |
| `src/simulation/traffic_scenario.cpp` | **New** | Scenario generators + TTL recovery engine |
| `include/simulation/scenario_runner.hpp` | **New** | Multi-event timeline orchestrator |
| `src/simulation/scenario_runner.cpp` | **New** | Timeline playback + correctness logging |
| `tests/test_main.cpp` | **Updated** | Tests 23–26: traffic scenario verification |
| `docs/phase2_part3_design.md` | **New** | Traffic scenario design spec |

---

## Completion Criteria

| Part | Criterion |
|---|---|
| **Part 1** ✅ | All 6 tests pass; RR locality ratio < 15% verified |
| **Part 2** ✅ | JSON audit log emitted + round-tripped for 3 algorithms; Tests 20-22 passing |
| **Part 3** ✅ | All 3 scenario types tested with multi-algo agreement; Tests 23-26 passing |
