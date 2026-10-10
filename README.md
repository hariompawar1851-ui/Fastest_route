# ⚡ DynamicRoute

> **Adaptive Shortest-Path & Route Recovery Engine for Dynamic Graphs**

[![C++20](https://img.shields.io/badge/C%2B%2B-20-blue.svg)](https://en.cppreference.com/w/cpp/20)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Build Status](https://img.shields.io/badge/Build-Passing-brightgreen.svg)]()
[![Status](https://img.shields.io/badge/Status-Active%20Development-orange.svg)]()

---

## 📌 Overview

Standard shortest-path engines recalculate Dijkstra or $A^*$ across entire road networks whenever travel conditions fluctuate (lane accidents, construction delays, sudden congestion). For large graphs, full recomputation incurs an unnecessary cost of $O(|E| + |V| \log |V|)$ per query, even when the disturbance is strictly local.

**DynamicRoute** eliminates full recomputations using **Incremental Heuristic Search** ($D^*$ Lite and Lifelong Planning $A^*$ / LPA*). By maintaining search tree states and one-step lookahead values, the engine restricts updates strictly to the affected perturbation region (the $\delta$-subgraph), achieving fast route recovery with mathematical optimality guaranteed.

---

## ⚡ Core Features

- **Incremental Route Repair ($D^*$ Lite / LPA\*):** Reuses distance values from previous iterations, only updating inconsistent vertices ($g(u) \neq rhs(u)$) affected by edge weight changes or road blockages.
- **Dual Adjacency Graph:** Directed weighted graph with forward and backward adjacency for $O(1)$ neighbor lookup and instant edge cost modification.
- **Baseline Verification Engine:** Built-in standard Dijkstra and $A^*$ implementations to automatically verify correctness ($\text{cost}_{\text{incremental}} \equiv \text{cost}_{\text{Dijkstra}}$).
- **Performance & Diagnostic Metrics:** Records execution latency ($\mu s$), node expansion counts, and speedup ratios against full recalculation.
- **Modular Architecture:** Clean modern C++20 design with CMake build configuration and automated test suite.

---

## 🧩 Optional & Advanced Extensions

The repository is architected to support optional modular extensions:

- 🔹 **Ramalingam–Reps (RR) Algorithm *(Optional)*:** Bounded incremental Single-Source Shortest Path (SSSP) tree maintenance.
- 🔹 **Degree-2 Chain Compression *(Optional)*:** Topological reduction contracting non-branching road corridors into macro-edges, shrinking graph size by 30%–50%.
- 🔹 **OSM Road Graph Ingestion *(Optional)*:** Parser to import real-world road networks from OpenStreetMap data.
- 🔹 **Memory & Cache Profiling *(Optional)*:** Compressed Sparse Row (CSR) representation and CPU cache locality benchmarking.
- 🔹 **Advanced Event Simulation *(Optional)*:** Sophisticated traffic scenarios such as multi-road congestion waves and incident zones.
- 🔹 **Interactive Visualizer *(Optional)*:** Web/GUI interface for real-time graph state and route repair visualization.

---

## 📊 Algorithmic Complexity Comparison

| Algorithm | Paradigm | Query Latency ($s \to t$) | Dynamic Edge Weight Increase / Block | Dynamic Edge Weight Decrease / Clear | Nodes Visited in Local Change |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Dijkstra (Scratch)** | Greedy Static | $O(|E| + |V| \log |V|)$ | Recompute full graph | Recompute full graph | Full graph exploration ($|V|$) |
| **$A^*$ (Scratch)** | Heuristic Static | $O(|E|)$ average | Recompute full graph | Recompute full graph | Goal-directed cone |
| **$D^*$ Lite / LPA\* *(Core)*** | Incremental Heuristic | Pre-computed Tree Lookup ($O(1)$) | $O(\|\delta\| + |\delta| \log |\delta|)$ | $O(\|\delta\| + |\delta| \log |\delta|)$ | **Only inconsistent nodes ($\|\delta\| \ll |V|$)** |
| **Ramalingam-Reps *(Optional)*** | Dynamic SSSP Tree | Pre-computed Tree Lookup ($O(1)$) | $O(\|\delta\| + |\delta| \log |\delta|)$ | $O(\|\delta\| + |\delta| \log |\delta|)$ | **Only affected subtree & boundary** |
| **Degree-2 + $D^*$ Lite *(Optional)*** | Compressed Incremental | Compressed Tree Lookup ($O(1)$) | $O(\|\delta_c\| + |\delta_c| \log |\delta_c|)$ | $O(\|\delta_c\| + |\delta_c| \log |\delta_c|)$ | **Only affected macro-nodes ($\|\delta_c\| \ll \|\delta\|$)** |

*Note: $\|\delta\|$ represents the number of edges incident to vertices whose distance or shortest-path parent changes. In local road changes, $|\delta|$ is typically a tiny fraction of $|V|$.*

---

## 🏗️ Architecture

```
                  Dynamic Graph Event
        (Edge Weight Increase / Decrease / Block)
                          │
                          ▼
            Update affected edge weight w(u, v)
            Update one-step lookahead rhs(v)
                          │
                          ▼
               Is v inconsistent?
                (g(v) != rhs(v))
                  /           \
                No             Yes
                /               \
         [No action]      [Insert v into Priority Queue]
                                │
                                ▼
                  Process Inconsistent Frontier:
           ┌────────────────────────────────────────┐
           │ Overconsistent: g(u) > rhs(u)          │
           │   -> Set g(u) = rhs(u)                 │
           │   -> Propagate improvement to succ(u)  │
           ├────────────────────────────────────────┤
           │ Underconsistent: g(u) < rhs(u)         │
           │   -> Set g(u) = ∞                      │
           │   -> Re-evaluate u and all succ(u)     │
           │   -> Find best alternative predecessor │
           └────────────────────────────────────────┘
                                │
                                ▼
                     Goal Reached / Locally Consistent
                                │
                                ▼
                      Optimal Route Extracted
                     + Performance Metrics Logged
```

---

## 📁 Repository Layout

```text
dynamic-route/
├── docs/
│   ├── research_work.md           # Research blueprint and algorithmic specifications
│   └── WORKFLOW_AND_RESEARCH.md   # Mathematical derivations and literature review
├── include/
│   ├── graph/                     # Adjacency representations, node & edge definitions
│   ├── algorithms/                # Dijkstra, A*, D* Lite / LPA*, RR (optional)
│   ├── simulation/                # Dynamic edge perturbation generators
│   └── compression/               # Degree-2 contraction & unpacker (optional)
├── src/                           # Core implementations & CLI main
├── tests/                         # Automated invariant tests (Incremental == Dijkstra)
├── benchmarks/                    # Latency and node-expansion benchmarking suite
├── CMakeLists.txt                 # Modern CMake build configuration
├── .gitignore
└── README.md
```

---

## 🚀 Getting Started

### Prerequisites

- Modern C++ compiler supporting **C++20** (GCC 11+, Clang 13+, or MSVC 2019+)
- **CMake** (v3.20 or newer)
- **Git**

### Build

```bash
# Clone the repository
git clone https://github.com/<your-username>/dynamic-route.git
cd dynamic-route

# Configure and build
mkdir build && cd build
cmake ..
cmake --build .
```

### Run

```bash
# Run the interactive CLI & simulation demo
./bin/dynamic_route

# Run automated correctness verification tests
./bin/run_tests

# Run benchmark comparisons
./bin/run_benchmarks
```

---

## 🗺️ Project Roadmap

### Core Phases
- [x] **Phase 0:** Project specification, algorithmic research, and system design.
- [x] **Phase 1:** Core graph engine with forward/backward adjacency, $O(1)$ edge modification, SPT, RR, GraphIO, PathValidator (13/13 tests passing).
- [x] **Phase 2, Part 1:** Large-scale invariant stress testing — 1,000+ perturbations on grids and RGGs; three-way LPA\* == D\* Lite == RR == Dijkstra agreement; RR locality ratio verified < 15% (6/6 tests passing). See [`docs/phase2_plan.md`](docs/phase2_plan.md).
- [x] **Phase 2, Part 2:** Explainability & Diagnostics Engine — JSON causal audit log per perturbation (`AuditLogger`, `CausalReporter`, NDJSON serialization, Tests 20–22 passing).
- [x] **Phase 2, Part 3:** Advanced Traffic Simulator — corridor congestion waves, DisasterZone events, TTL recovery engine, `ScenarioRunner` (Tests 23–26 passing).
- [ ] **Phase 3:** Incremental Heuristic Search ($D^*$ Lite / Lifelong Planning $A^*$) — advanced extensions.
- [ ] **Phase 4:** Scalability benchmarks on OSM / DIMACS real road networks.

### Optional Extension Phases
- [ ] **Phase 5 (Optional):** Ramalingam–Reps (RR) dynamic Single-Source Shortest Path engine.
- [ ] **Phase 6 (Optional):** Degree-2 topological compression pipeline and path unpacker.
- [ ] **Phase 7 (Optional):** OSM road graph importer and sophisticated traffic incident models.
- [ ] **Phase 8 (Optional):** Memory/cache profiling (CSR layout) and interactive visualizer.

---

## 📚 In-Depth Documentation

- 📖 **[Research & Algorithmic Blueprint](docs/research_work.md):** Detailed algorithms, consistency states, and benchmark plans.
- 🔬 **[Workflow & Research Foundations](docs/WORKFLOW_AND_RESEARCH.md):** Mathematical foundations, literature review, and complexity analysis.

---

## 📜 License

Distributed under the MIT License. See [LICENSE](LICENSE) for more information.
