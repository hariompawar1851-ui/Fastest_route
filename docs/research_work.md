# DynamicRoute: Research & Engineering Blueprint

> **Adaptive Shortest-Path & Route Recovery Engine for Dynamic Graphs**

[![C++20](https://img.shields.io/badge/C%2B%2B-20-blue.svg)](https://en.cppreference.com/w/cpp/20)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](../LICENSE)
[![Build Status](https://img.shields.io/badge/Build-Passing-brightgreen.svg)]()
[![Status](https://img.shields.io/badge/Status-Active%20Development-orange.svg)]()

---

## 1. Executive Summary & Problem Formulation

Standard shortest-path algorithms assume a static graph $G = (V, E)$. When edge costs change—such as road blockages, construction delays, lane accidents, or sudden congestion—standard engines rerun Dijkstra or $A^*$ across the entire graph. For large networks, this incurs an unnecessary computational cost of $O(|E| + |V| \log |V|)$ per query, even when the perturbation only affects a localized section of the road network.

**DynamicRoute** addresses this challenge through **Incremental Heuristic Search**. Instead of recomputing shortest paths from scratch, the system maintains vertex distance estimates and one-step lookahead values, isolating graph repairs strictly to the affected perturbation region (the $\delta$-subgraph).

### Core Research Focus
- **Incremental Heuristic Search ($D^*$ Lite / Lifelong Planning $A^*$):** Focus updates exclusively on vertices whose distance estimates become inconsistent with their lookahead values toward the target goal.
- **Dual Adjacency Graph Representation:** Maintain forward and backward edge lists to allow $O(1)$ predecessor and successor queries during dynamic edge weight modifications.
- **Ground-Truth Correctness Verification:** Automated invariant verification ensuring that the incremental path cost matches a scratch-recomputed Dijkstra path on every modification.
- **Empirical Profiling:** Measure wall-clock latency ($\mu s$), vertex visitation count, priority queue operations, and speedup ratios against full recomputation.

---

## 2. Research Foundations

The core engine is grounded in peer-reviewed research in incremental search and dynamic graph theory:

### 2.1 Core Foundations

1. **Koenig & Likhachev (2002, 2005)**
   * **Title:** *D\* Lite* (AAAI 2002) and *Lifelong Planning A\** (Artificial Intelligence Journal, 2004).
   * **Venue:** AAAI / Journal of Artificial Intelligence.
   * **Core Contribution:** Introduces the concept of **one-step lookahead values ($rhs$)** alongside current distance estimates ($g$). Vertices are classified into consistency states:
     - *Locally Consistent:* $g(u) = rhs(u)$
     - *Locally Overconsistent:* $g(u) > rhs(u)$ (distance can be decreased)
     - *Locally Underconsistent:* $g(u) < rhs(u)$ (distance must increase or find an alternative predecessor)
   * Only inconsistent vertices are placed into the priority queue, enabling fast repair toward a goal.

2. **Frigioni, Marchetti-Spaccamela, & Nanni (2000)**
   * **Title:** *Fully Dynamic Algorithms for Shortest Paths in Directed Graphs*
   * **Venue:** ACM Transactions on Algorithms / Theoretical Computer Science.
   * **Core Contribution:** Theoretical properties of dynamic shortest-path trees under arbitrary edge deletions, insertions, weight increases, and weight decreases in general directed graphs.

### 2.2 Optional & Extension Literature

3. **Ramalingam & Reps (1996) *(Optional Extension)***
   * **Title:** *An Incremental Algorithm for a Generalization of the Shortest-Path Problem*
   * **Venue:** ACM Transactions on Programming Languages and Systems (TOPLAS), 18(6), 1020–1046.
   * **Core Contribution:** The foundational **RR-Algorithm** for bounded dynamic Single-Source Shortest Paths (SSSP) tree maintenance, bounding updates to the $\delta$-subgraph: $O(\|\delta\| + |\delta| \log |\delta|)$.

---

## 3. Algorithmic Specifications

### 3.1 Graph Representation & Lookahead States
In a directed weighted graph $G = (V, E)$, for a given source $S$, we maintain for each vertex $u \in V$:
* $g(u)$: The current known shortest-path distance estimate from $S$ to $u$.
* $rhs(u)$: The one-step lookahead value defined as:
  $$rhs(u) = \begin{cases} 0 & \text{if } u = S \\ \min_{v \in \text{Pred}(u)} \big(g(v) + w(v, u)\big) & \text{otherwise} \end{cases}$$
* $parent(u)$: The current predecessor on the shortest path tree:
  $$parent(u) = \arg\min_{v \in \text{Pred}(u)} \big(g(v) + w(v, u)\big)$$

### 3.2 Consistency States & Priority Queue
A vertex is:
* **Locally Consistent:** $g(u) = rhs(u)$ (optimal based on current predecessor distances).
* **Locally Overconsistent:** $g(u) > rhs(u)$ (a shorter path is available; vertex needs relaxation).
* **Locally Underconsistent:** $g(u) < rhs(u)$ (an edge cost increased or road blocked; current path is invalid).

The priority queue stores **only inconsistent vertices**, keyed by a lexicographical 2-tuple:
$$k(u) = \big[k_1(u), \; k_2(u)\big]$$
where:
$$k_1(u) = \min(g(u), rhs(u)) + h(u, \text{Goal})$$
$$k_2(u) = \min(g(u), rhs(u))$$
and $h(u, \text{Goal})$ is an admissible heuristic function (e.g., Euclidean distance for planar graphs, or $0$ for SSSP).

### 3.3 Dynamic Repair Procedure (LPA* / D* Lite)

```
When an edge (u, v) changes weight to w_new(u, v):
1. Update w(u, v) = w_new(u, v).
2. Update rhs(v) based on min_{p in Pred(v)} (g(p) + w(p, v)).
3. Update vertex v in Priority Queue:
   - If g(v) != rhs(v), insert or update v with key k(v).
   - If g(v) == rhs(v), remove v from Priority Queue.
4. While (Priority Queue min key < k(Goal) OR rhs(Goal) != g(Goal)):
   - Pop vertex u with smallest key.
   - If g(u) > rhs(u) [Overconsistent]:
       Set g(u) = rhs(u).
       For each s in Succ(u), re-evaluate rhs(s) and update s in Priority Queue.
   - Else [Underconsistent]:
       Set g(u) = infinity.
       Re-evaluate rhs(u) and for each s in Succ(u), re-evaluate rhs(s).
       Update u and succ(u) in Priority Queue.
5. Extract reconstructed shortest path by back-tracing from Goal to S.
```

---

## 4. Optional & Advanced Extension Modules

DynamicRoute is designed with modular interfaces to support optional experimental extensions:

### 4.1 Ramalingam–Reps (RR) Algorithm *(Optional)*
* Implements tree-based incremental Single-Source Shortest Path (SSSP) maintenance.
* Separates updates into *phase 1* (identifying the affected subtree via DFS/BFS) and *phase 2* (recalculating distances for boundary and internal vertices via priority queue).
* Useful for all-pairs or single-source applications where no specific single destination goal is designated.

### 4.2 Degree-2 Topological Graph Compression *(Optional)*
* Identifies non-branching vertex chains where $\text{in-deg}(v) = \text{out-deg}(v) = 1$.
* Replaces chains $u \leftrightarrow v_1 \leftrightarrow \dots \leftrightarrow w$ with a macro-edge $(u, w)$ with aggregated weight $\sum w_i$.
* Reduces $|V|$ and $|E|$ by 30%–50% for road networks while preserving 100% path optimality.
* Includes a lightweight path unpacker to expand macro-edges back into full coordinate sequences.

### 4.3 OSM Road Graph Ingestion *(Optional)*
* Ingests real-world road networks from OpenStreetMap XML/PBF formats into the dual-adjacency graph structure.
* Parses node latitude/longitude coordinates and road classification speeds.

### 4.4 Memory & Cache Experiments *(Optional)*
* Evaluates memory locality by comparing standard `std::vector` adjacency structures with flat Compressed Sparse Row (CSR) representations.
* Profiles CPU L1/L2 cache misses during high-frequency node relaxation.

### 4.5 Sophisticated Event Models *(Optional)*
* Simulates advanced traffic phenomena such as:
  - Multi-edge corridor congestion waves (propagating slow traffic along arterial routes).
  - Geographic incident exclusion zones (disabling all edges within a specified radius).

### 4.6 Advanced Visualization *(Optional)*
* Graphical user interface / web visualization displaying the road network, live edge weight perturbations, and incremental search frontier expansion.

---

## 5. Comparative Complexity Matrix

| Algorithm | Paradigm | Query Latency ($s \to t$) | Dynamic Edge Weight Increase / Block | Dynamic Edge Weight Decrease / Clear | Nodes Visited in Local Change |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Dijkstra (Scratch)** | Greedy Static | $O(|E| + |V| \log |V|)$ | Recompute full graph | Recompute full graph | Full graph exploration ($|V|$) |
| **$A^*$ (Scratch)** | Heuristic Static | $O(|E|)$ average | Recompute full graph | Recompute full graph | Goal-directed cone |
| **$D^*$ Lite / LPA\* *(Core)*** | Incremental Heuristic | Pre-computed Tree Lookup ($O(1)$) | $O(\|\delta\| + |\delta| \log |\delta|)$ | $O(\|\delta\| + |\delta| \log |\delta|)$ | **Only inconsistent nodes ($\|\delta\| \ll |V|$)** |
| **Ramalingam-Reps *(Optional)*** | Dynamic SSSP Tree | Pre-computed Tree Lookup ($O(1)$) | $O(\|\delta\| + |\delta| \log |\delta|)$ | $O(\|\delta\| + |\delta| \log |\delta|)$ | **Only affected subtree & boundary** |
| **Degree-2 + $D^*$ Lite *(Optional)*** | Compressed Incremental | Compressed Tree Lookup ($O(1)$) | $O(\|\delta_c\| + |\delta_c| \log |\delta_c|)$ | $O(\|\delta_c\| + |\delta_c| \log |\delta_c|)$ | **Only affected macro-nodes ($\|\delta_c\| \ll \|\delta\|$)** |

---

## 6. Project Architecture & Directory Layout

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
├── src/
│   ├── graph/
│   ├── algorithms/
│   ├── simulation/
│   ├── compression/               # (optional)
│   └── main.cpp                   # Application entry point & interactive CLI
├── tests/                         # Automated invariant tests (Incremental == Dijkstra)
├── benchmarks/                    # Scalability and speedup benchmarks
├── CMakeLists.txt                 # Modern CMake build configuration
├── .gitignore
└── README.md
```

---

## 7. Phased Implementation Roadmap

### Core Phases
- [x] **Phase 0:** Project specification, algorithmic research, and system design.
- [ ] **Phase 1:** Core graph engine with forward/backward adjacency and $O(1)$ edge modification.
- [ ] **Phase 2:** Baseline Dijkstra and $A^*$ implementations for ground-truth verification.
- [ ] **Phase 3:** Incremental Heuristic Search ($D^*$ Lite / Lifelong Planning $A^*$).
- [ ] **Phase 4:** Automated invariant test suite (`cost(Incremental) == cost(Dijkstra)`) and benchmarks.

### Optional Extension Phases
- [ ] **Phase 5 (Optional):** Ramalingam–Reps (RR) dynamic Single-Source Shortest Path engine.
- [ ] **Phase 6 (Optional):** Degree-2 topological compression pipeline and path unpacker.
- [ ] **Phase 7 (Optional):** OSM road graph importer and sophisticated traffic incident models.
- [ ] **Phase 8 (Optional):** Memory/cache profiling (CSR layout) and interactive visualizer.
