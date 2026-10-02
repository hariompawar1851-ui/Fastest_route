# DynamicRoute: Technical Architecture, Literature Review & Project Workflow

This document provides the complete academic foundation, algorithmic specifications, and multi-phase implementation roadmap for **DynamicRoute**.

---

## 1. Executive Summary & Vision

Traditional shortest-path systems treat the road network as a static graph $G = (V, E)$. When road conditions fluctuate—such as sudden congestion, traffic light delays, construction, or road blockages—standard systems rerun algorithms like Dijkstra or $A^*$ from scratch. On large networks, this incurs an unnecessary computational cost of $O(|E| + |V| \log |V|)$ per query, even when the modification is localized and only affects a tiny fraction of paths.

To elevate this project from a standard classroom Dijkstra implementation into a **novel, research-grade systems project**, we combine **Dynamic Graph Theory**, **Incremental Heuristic Search**, and an **Explainable Routing & Diagnostics Engine**:

1. **Incremental Subgraph Repair vs. Global Recalculation:** Instead of full recomputation, we maintain a **Shortest Path Tree (SPT)** and incrementally repair only the affected perturbation region (the $\delta$-subgraph), achieving update complexity bounded by the size of the change rather than the size of the entire graph.
2. **Algorithmic Duality:** We implement and compare:
   * **Ramalingam–Reps (RR) Incremental SSSP Algorithm** (Dynamic Shortest Path Tree maintenance).
   * **$D^*$ Lite / Lifelong Planning $A^*$ (LPA\*)** (Incremental heuristic search widely used in robotics and autonomous navigation).
   * **Baseline Recomputed Dijkstra / $A^*$** (Ground-truth verification).
3. **Reasoning & Explainability Engine ("Why did my route change?"):** The system tracks and logs the causal chain of decisions:
   * *Root Cause:* "Edge $(u, v)$ capacity dropped by 70% due to congestion."
   * *Impact:* "Current route invalidated at junction $u$; 14 downstream vertices invalidated."
   * *Alternative Exploration:* "Frontier evaluated 3 candidate bypasses; bypass via node $k$ selected."
   * *Efficiency Metric:* "Explored 28 nodes vs 45,000 in full Dijkstra (99.94% reduction in vertex visits)."
4. **Decoupled Architecture Ready for Web Visualization:** The core engine is built in high-performance C++ with a clean serialization layer (JSON / WebSockets / WebAssembly ready) so that a real-time web visualization can be hooked in seamlessly.

---

## 2. Theoretical Grounding: Research Papers Used

This project is built on peer-reviewed algorithms from dynamic graph theory and incremental search:

### 2.1 Primary Research Papers

1. **Ramalingam & Reps (1996)**
   * **Title:** *An Incremental Algorithm for a Generalization of the Shortest-Path Problem*
   * **Venue:** ACM Transactions on Programming Languages and Systems (TOPLAS), 18(6), 1020–1046. (Early version in STOC 1991).
   * **Core Contribution:** The foundational **RR-Algorithm** for bounded dynamic Single-Source Shortest Paths (SSSP). When an edge weight increases or decreases, the algorithm restricts its operations strictly to the set of vertices $\delta$ whose distance or parent pointer actually changes.
   * **Complexity:** $O(|\delta| + ||\delta|| \log ||\delta||)$, where $|\delta||$ is the number of edges incident to modified vertices. When changes are local, $|\delta|| \ll |E|$.

2. **Koenig & Likhachev (2002, 2005)**
   * **Title:** *D\* Lite* (AAAI 2002) and *Lifelong Planning A\** (Artificial Intelligence Journal, 2004).
   * **Venue:** AAAI / Journal of Artificial Intelligence.
   * **Core Contribution:** Introduces the concept of **one-step lookahead values ($rhs$)** alongside current distance estimates ($g$). Vertices are classified as *locally consistent* ($g = rhs$), *locally overconsistent* ($g > rhs$), or *locally underconsistent* ($g < rhs$). Only inconsistent vertices are placed in a priority queue. It combines the goal-directed efficiency of $A^*$ with incremental edge updates.

3. **Frigioni, Marchetti-Spaccamela, & Nanni (2000)**
   * **Title:** *Fully Dynamic Algorithms for Shortest Paths in Directed Graphs*
   * **Venue:** ACM Transactions on Algorithms / Theoretical Computer Science.
   * **Core Contribution:** Formalized dynamic Shortest Path Tree (SPT) properties under edge deletions, insertions, weight increases, and weight decreases in general directed graphs with non-negative edge weights.

4. **Dibbelt, Strasser, & Wagner (2016)** *(Inspiration for large-scale road networks)*
   * **Title:** *Customizable Contraction Hierarchies (CCH)*
   * **Venue:** ACM Journal of Experimental Algorithmics (JEA).
   * **Core Contribution:** Separates the metric-independent metric topology from dynamic live weights, showing how production map engines handle live traffic updates.

---

## 3. Algorithmic Deep Dive: How the Core Engine Works

### 3.1 The Shortest Path Tree (SPT) Representation
In our graph $G = (V, E)$, for a given source $S$, we maintain:
* $g(u)$: The current known shortest distance from $S$ to $u$.
* $rhs(u)$: One-step lookahead value defined as:
  $$rhs(u) = \begin{cases} 0 & \text{if } u = S \\ \min_{v \in \text{Pred}(u)} \big(g(v) + w(v, u)\big) & \text{otherwise} \end{cases}$$
* $parent(u)$: The predecessor node on the shortest path tree:
  $$parent(u) = \arg\min_{v \in \text{Pred}(u)} \big(g(v) + w(v, u)\big)$$
* $\text{Children}(u)$: Adjacency list of the directed Shortest Path Tree $T$.

### 3.2 Consistency States & Priority Queue
A vertex is:
* **Locally Consistent:** $g(u) = rhs(u)$ (no change needed).
* **Locally Overconsistent:** $g(u) > rhs(u)$ (a shorter path was discovered; vertex needs relaxation).
* **Locally Underconsistent:** $g(u) < rhs(u)$ (a road cost increased or was blocked; old path is no longer valid, distance must increase or find an alternative).

The priority queue stores **only inconsistent nodes**, keyed by:
$$k(u) = \big[\min(g(u), rhs(u)) + h(u, \text{Goal}), \;\; \min(g(u), rhs(u))\big]$$
*(where $h(u, \text{Goal})$ is the admissible heuristic distance to the goal, or $0$ for Dijkstra-style SSSP).*

---

## 4. End-to-End System Workflow

The project is structured into 6 sequential phases:

- **Phase 1: Core Graph & Dynamic Topological Engine**
  - Forward/Backward Adjacency Lists with O(1) edge lookup
  - Shortest Path Tree (SPT) with parent/children pointers
  - Node metadata (coordinates, road types, capacities)

- **Phase 2: Algorithm Implementation & Verification**
  - Baseline Dijkstra & A* (Static Ground Truth)
  - Ramalingam-Reps (RR) Dynamic SSSP
  - D* Lite / LPA* (Incremental Heuristic Search)
  - Automated Invariant Checker (verifies dynamic output == baseline)

- **Phase 3: Explainability & Reasoning Engine**
  - Subgraph perturbation tracer (counts invalidated vs repaired nodes)
  - Decision log generator ("Why Edge X was bypassed for Edge Y")
  - Real-time computational efficiency audit (Ops counter)

- **Phase 4: Dynamic Event Simulator & Road Generators**
  - Grid, Geometric random, and OpenStreetMap (OSM) road graph loaders
  - Synthetic perturbation engine (Blockages, Congestion surges, Disaster zones)

- **Phase 5: Comprehensive Benchmark & Profiling Suite**
  - Scaling tests: 1,000 to 500,000 nodes
  - Metrics: Wall-clock time, nodes explored, queue updates, memory
  - Speedup factor: Dynamic repair time vs Full recalculation time

- **Phase 6: Interface & Visualization Layer (Web Ready)**
  - Headless C++ JSON state dumper / REST / WebSocket server
  - Interactive UI (Web dashboard with canvas/SVG road network)
