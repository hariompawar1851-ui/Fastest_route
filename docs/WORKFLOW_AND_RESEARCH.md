# DynamicRoute: Technical Architecture, Literature Review & Project Workflow

This document provides the academic foundation, algorithmic specifications, mathematical formulations, and multi-phase implementation roadmap for **DynamicRoute**.

---

## 1. Executive Summary & Vision

Traditional shortest-path systems treat the road network as a static graph $G = (V, E)$. When travel conditions fluctuate—such as sudden congestion, traffic light delays, lane accidents, or road blockages—standard systems rerun Dijkstra or $A^*$ from scratch across the entire network. On large networks, this incurs an unnecessary computational cost of $O(|E| + |V| \log |V|)$ per query, even when the modification is strictly localized and only affects a tiny fraction of paths.

**DynamicRoute** addresses this challenge through **Incremental Heuristic Search**:

1. **Incremental Subgraph Repair vs. Global Recalculation:** Instead of full recomputation, the engine maintains distance estimates and one-step lookahead values, isolating repairs strictly to the affected perturbation region (the $\delta$-subgraph).
2. **Core Algorithm: Lifelong Planning $A^*$ (LPA\*) / $D^*$ Lite:** Combines the goal-directed efficiency of $A^*$ with incremental edge updates. By tracking locally inconsistent vertices ($g(u) \neq rhs(u)$), the search frontier only repairs paths relevant to the destination.
3. **Dual Adjacency Graph Engine:** Maintains forward and backward adjacency lists, enabling $O(1)$ predecessor and successor lookups during dynamic edge cost modifications.
4. **Automated Correctness Verification:** Establishes rigorous invariant testing ensuring that incremental shortest paths are mathematically identical in cost to scratch-recomputed Dijkstra paths on every modification.
5. **Modular Architecture for Optional Extensions:** Core routing is kept clean, focused, and lightweight, while providing modular interfaces for optional extensions such as topological compression, the Ramalingam-Reps algorithm, OSM map loaders, and cache experiments.

---

## 2. Theoretical Grounding: Research Papers Used

### 2.1 Core Foundations

1. **Koenig & Likhachev (2002, 2005)**
   * **Title:** *D\* Lite* (AAAI 2002) and *Lifelong Planning A\** (Artificial Intelligence Journal, 2004).
   * **Venue:** AAAI / Journal of Artificial Intelligence.
   * **Core Contribution:** Introduces the concept of **one-step lookahead values ($rhs$)** alongside current distance estimates ($g$). Vertices are classified into consistency states:
     - *Locally Consistent:* $g(u) = rhs(u)$ (no change needed).
     - *Locally Overconsistent:* $g(u) > rhs(u)$ (a shorter path was discovered; vertex needs relaxation).
     - *Locally Underconsistent:* $g(u) < rhs(u)$ (an edge cost increased or was blocked; old path is invalid).
   * Only inconsistent vertices are placed into the priority queue, enabling fast repair toward a goal.

2. **Frigioni, Marchetti-Spaccamela, & Nanni (2000)**
   * **Title:** *Fully Dynamic Algorithms for Shortest Paths in Directed Graphs*
   * **Venue:** ACM Transactions on Algorithms / Theoretical Computer Science.
   * **Core Contribution:** Formalized dynamic Shortest Path Tree (SPT) properties under edge deletions, insertions, weight increases, and weight decreases in general directed graphs with non-negative edge weights.

### 2.2 Optional Extension Literature

3. **Ramalingam & Reps (1996) *(Optional Extension)***
   * **Title:** *An Incremental Algorithm for a Generalization of the Shortest-Path Problem*
   * **Venue:** ACM Transactions on Programming Languages and Systems (TOPLAS), 18(6), 1020–1046.
   * **Core Contribution:** The foundational **RR-Algorithm** for bounded dynamic Single-Source Shortest Paths (SSSP). When an edge weight increases or decreases, the algorithm restricts its operations strictly to the set of vertices $\delta$ whose distance or parent pointer actually changes: $O(|\delta| + ||\delta|| \log ||\delta||)$.

---

## 3. Algorithmic Deep Dive & Mathematical Formulations

### 3.1 Graph Representation & Lookahead States

In our graph $G = (V, E)$, for a given source $S$, we maintain:
* $g(u)$: The current known shortest distance estimate from $S$ to $u$.
* $rhs(u)$: One-step lookahead value defined as:
  $$rhs(u) = \begin{cases} 0 & \text{if } u = S \\ \min_{v \in \text{Pred}(u)} \big(g(v) + w(v, u)\big) & \text{otherwise} \end{cases}$$
* $parent(u)$: The predecessor node on the shortest path tree:
  $$parent(u) = \arg\min_{v \in \text{Pred}(u)} \big(g(v) + w(v, u)\big)$$
* $\text{Pred}(u)$ and $\text{Succ}(u)$: Directed predecessor and successor adjacency sets.

### 3.2 Consistency States & Priority Queue

A vertex is:
* **Locally Consistent:** $g(u) = rhs(u)$.
* **Locally Overconsistent:** $g(u) > rhs(u)$.
* **Locally Underconsistent:** $g(u) < rhs(u)$.

The priority queue stores **only inconsistent vertices**, keyed by a 2-tuple:
$$k(u) = \big[k_1(u), \; k_2(u)\big]$$
where:
$$k_1(u) = \min(g(u), rhs(u)) + h(u, \text{Goal})$$
$$k_2(u) = \min(g(u), rhs(u))$$
and $h(u, \text{Goal})$ is an admissible heuristic estimate (e.g. Euclidean distance, or $0$ for SSSP).

Keys are ordered lexicographically:
$$k(u) < k(v) \iff k_1(u) < k_1(v) \lor \big(k_1(u) = k_1(v) \land k_2(u) < k_2(v)\big)$$

### 3.3 Dynamic Repair Procedure ($D^*$ Lite / LPA\*)

When an edge $(u, v)$ undergoes a weight change $w(u, v) \leftarrow w_{\text{new}}(u, v)$:
1. Update directed edge cost $w(u, v)$.
2. Update one-step lookahead value for $v$:
   $$rhs(v) = \min_{p \in \text{Pred}(v)} \big(g(p) + w(p, v)\big)$$
3. Update $v$ in the priority queue:
   - If $g(v) \neq rhs(v)$, insert or update $v$ with key $k(v)$.
   - If $g(v) = rhs(v)$, remove $v$ from the queue.
4. **ComputeShortestPath() Loop:**
   While the priority queue's minimum key is smaller than $k(\text{Goal})$ or $rhs(\text{Goal}) \neq g(\text{Goal})$:
   - Pop vertex $u$ with the minimum key.
   - **Case 1 (Overconsistent, $g(u) > rhs(u)$):**
     - Set $g(u) = rhs(u)$.
     - For every successor $s \in \text{Succ}(u)$: update $rhs(s)$ and update $s$ in the priority queue.
   - **Case 2 (Underconsistent, $g(u) < rhs(u)$):**
     - Set $g(u) = \infty$.
     - Re-evaluate $rhs(u)$ and update $u$ in the priority queue.
     - For every successor $s \in \text{Succ}(u)$: update $rhs(s)$ and update $s$ in the priority queue.
5. Reconstruct the optimal path by following parent pointers from $\text{Goal}$ back to $S$.

---

## 4. Optional & Advanced Extension Specifications

The following modules are architected as optional extensions that can be independently enabled or studied:

### 4.1 Optional: Lossless Topological Compression (Degree-2 Chain Contraction)
* **Concept:** Road networks frequently have non-branching road segments:
  $$\text{Chain: } u_0 \longleftrightarrow u_1 \longleftrightarrow u_2 \dots \longleftrightarrow u_k$$
  where intermediate vertices have $\text{deg}(u_i) = 2$.
* **Compression:** Contract chains into a macro-edge $(u_0, u_k)$ with weight $\sum_{i=0}^{k-1} w(u_i, u_{i+1})$.
* **Speedup:** Reduces $|V|$ and $|E|$ by 30%–50% while preserving exact shortest paths.
* **Path Unpacker:** Recursively expands macro-edges back into full coordinate/edge sequences for visualization and navigation.

### 4.2 Optional: Ramalingam–Reps (RR) Algorithm
* Implements dynamic Single-Source Shortest Path (SSSP) tree maintenance without heuristic goal-direction.
* Separates repair into an invalidation phase (identifying affected subtree) and a priority queue re-evaluation phase.
* Serves as an algorithmic baseline to compare against LPA* / $D^*$ Lite.

### 4.3 Optional: Real-World OSM Road Graph Ingestion
* Ingests real-world street networks from OpenStreetMap XML/PBF data formats.
* Normalizes node coordinates and computes edge travel times from road classifications and length.

### 4.4 Optional: Memory & Cache Experiments (CSR Layout)
* Evaluates memory locality by comparing pointer-based adjacency lists with a flat **Compressed Sparse Row (CSR)** layout:
  - `row_offsets`: contiguous array of size $|V| + 1$.
  - `column_indices`: contiguous array of target vertices.
  - `edge_weights`: contiguous array of weights.
* Measures CPU L1/L2 cache misses during high-frequency node relaxation.

### 4.5 Optional: Advanced Traffic Simulation Scenarios
* Generates realistic dynamic disturbance events:
  - Multi-road corridor congestion waves (propagating congestion along consecutive edges).
  - Geographic exclusion zones (simulating road closures within a radius).

### 4.6 Optional: Interactive Visualization Layer
* Graphical/Web visualization rendering the road network, live edge weight perturbations, and incremental search frontier expansion.

---

## 5. End-to-End Project Workflow & Roadmap

### Core Implementation Workflow
- **Phase 1: Core Graph Engine**
  - Directed weighted graph with forward & backward adjacency lists.
  - Efficient edge insertion, deletion, and $O(1)$ weight updates.
  - Synthetic grid and random planar graph generators.

- **Phase 2: Baseline Static Algorithms & Diagnostics**
  - Standard Dijkstra implementation (greedy ground truth).
  - Standard $A^*$ with Euclidean / Manhattan distance heuristics.
  - Basic metrics instrumentation: wall-clock time ($\mu s$), node visits, path cost.

- **Phase 3: Incremental Heuristic Search ($D^*$ Lite / LPA\*)**
  - Priority queue with 2-tuple lexicographical keys.
  - Lookahead value ($rhs$) and consistency state management.
  - Dynamic edge update and `ComputeShortestPath()` repair loop.

- **Phase 4: Automated Verification & Benchmarking Suite**
  - Invariant testing: $\text{cost}(D^*\text{ Lite}) \equiv \text{cost}(\text{Dijkstra})$ across edge modifications and blockages.
  - Scalability benchmarks on graphs from 1,000 to 100,000 nodes.
  - Speedup ratios: dynamic repair vs full recalculation.

### Optional Extension Phases
- **Phase 5 (Optional):** Ramalingam–Reps (RR) dynamic SSSP engine implementation and comparison.
- **Phase 6 (Optional):** Degree-2 topological compression pipeline and path unpacker.
- **Phase 7 (Optional):** OSM road network importer and sophisticated congestion models.
- **Phase 8 (Optional):** Memory/cache locality experiments (CSR layout) and interactive visualization.
