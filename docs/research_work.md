# DynamicRoute: Comprehensive Algorithmic Blueprint & Research Guide

> **Adaptive Shortest-Path & Route Recovery Engine for Dynamic Graphs**

[![C++20](https://img.shields.io/badge/C%2B%2B-20-blue.svg)](https://en.cppreference.com/w/cpp/20)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](../LICENSE)
[![Build Status](https://img.shields.io/badge/Build-Passing-brightgreen.svg)]()
[![Status](https://img.shields.io/badge/Status-Active%20Development-orange.svg)]()

---

## 1. Executive Summary & Problem Formulation

Standard shortest-path algorithms assume a static graph $G = (V, E)$. When edge costs change—such as road blockages, construction delays, lane accidents, or sudden congestion—standard engines rerun Dijkstra or $A^*$ across the entire graph. For large networks, this incurs an unnecessary computational cost of $O(|E| + |V| \log |V|)$ per query, even when the perturbation only affects a localized section of the road network.

**DynamicRoute** eliminates global recalculation using **Incremental Heuristic Search**. Instead of recomputing shortest paths from scratch, the system maintains vertex distance estimates and one-step lookahead values, isolating graph repairs strictly to the affected perturbation region (the $\delta$-subgraph).

This document provides a **complete, mathematically rigorous, and pedagogical explanation** of all algorithms used in the project, detailing how they work, why they were chosen, and how they handle dynamic graph perturbations.

---

## 2. Taxonomy: Which Algorithms Are Used & Why?

The algorithms in **DynamicRoute** are organized into **Core Production Algorithms** and **Optional Extension Modules**:

```
                                DynamicRoute Algorithms
                                           │
         ┌─────────────────────────────────┴─────────────────────────────────┐
         ▼                                                                   ▼
   CORE ALGORITHMS                                                  OPTIONAL EXTENSIONS
  ┌──────────────────────────────────────────────┐                 ┌──────────────────────────────────────────┐
  │ 1. Lifelong Planning A* (LPA*)               │                 │ 5. Ramalingam–Reps (RR) Algorithm        │
  │    - Incremental forward heuristic search    │                 │    - Dynamic SSSP tree maintenance       │
  │                                              │                 │                                          │
  │ 2. D* Lite                                   │                 │ 6. Degree-2 Topological Compression      │
  │    - Incremental backward search for mobile  │                 │    - Non-branching corridor contraction  │
  │      navigation with key modifier k_m        │                 │                                          │
  │                                              │                 │ 7. Cache-Conscious CSR Layout            │
  │ 3. Baseline Static Dijkstra                  │                 │    - Contiguous array memory experiments │
  │    - Ground-truth uniform-cost baseline      │                 │                                          │
  │                                              │                 │ 8. Advanced Traffic Incident Simulator   │
  │ 4. Baseline Static A*                        │                 │    - Corridor congestion & wave models   │
  │    - Goal-directed heuristic baseline        │                 └──────────────────────────────────────────┘
  └──────────────────────────────────────────────┘
```

---

## 3. Core Algorithm 1: Lifelong Planning $A^*$ (LPA\*)

*Primary Paper:* Koenig, S., & Likhachev, M. (2004). *Lifelong Planning A\**. Artificial Intelligence Journal.

### 3.1 Intuition & The Core Concept

Imagine calculating a path from your home ($S$) to your office ($Goal$) across a city of 100,000 intersections. When a tree falls across a single residential street 5 km away, $99.9\%$ of your shortest-path calculations remain identical. Standard Dijkstra or $A^*$ throws away all 100,000 computed distances and starts completely over from $S$.

**Lifelong Planning $A^*$ (LPA\*)** retains distance values from previous searches. When road costs change, LPA* identifies only the subset of nodes whose distance values have become **inconsistent** with their neighbors, places them in a priority queue, and propagates corrections outward until the shortest path to $Goal$ is once again guaranteed optimal.

---

### 3.2 Fundamental Variables & State Values

For every vertex $u \in V$, LPA* maintains three primary quantities:

1. **$g(u)$ — Current Distance Estimate:**
   * Represents the path cost from source $S$ to $u$ computed during the previous search iteration.
   * Initialized to $\infty$ for all vertices except $g(S) = 0$.

2. **$rhs(u)$ — One-Step Lookahead Value:**
   * Represents what the distance to $u$ *should be* based on the current $g$-values of its immediate predecessors:
   $$rhs(u) = \begin{cases} 0 & \text{if } u = S \\ \min_{v \in \text{Pred}(u)} \big(g(v) + w(v, u)\big) & \text{otherwise} \end{cases}$$
   * If all predecessors have accurate $g$-values and no edges changed, then $g(u) = rhs(u)$.

3. **$h(u, \text{Goal})$ — Admissible Heuristic:**
   * An estimate of the remaining distance from $u$ to $Goal$ (e.g., straight-line Euclidean distance).
   * Must be **admissible** ($h(u, \text{Goal}) \le c^*(u, \text{Goal})$) and **consistent** ($h(u) \le w(u, v) + h(v)$) to guarantee optimality.

---

### 3.3 Consistency States: The Core Engine of Incremental Search

At any moment in time, comparing $g(u)$ with $rhs(u)$ categorizes vertex $u$ into one of three distinct **consistency states**:

| State | Condition | Meaning | Required Action |
| :--- | :--- | :--- | :--- |
| **Locally Consistent** | $g(u) = rhs(u)$ | The current distance $g(u)$ matches the optimal one-step predecessor lookahead. Node is stable. | **Do nothing.** Vertex does not enter the priority queue. |
| **Locally Overconsistent** | $g(u) > rhs(u)$ | A **shorter path** to $u$ has been discovered (e.g., an edge cost decreased or a predecessor was relaxed). | **Lower $g(u)$ to match $rhs(u)$**, then propagate this improvement to successors $\text{Succ}(u)$. |
| **Locally Underconsistent** | $g(u) < rhs(u)$ | The path through $u$ has become **invalid or more expensive** (e.g., an incoming edge was blocked or congested). | **Set $g(u) = \infty$**, re-evaluate $u$ and all successors, and find the best alternative predecessor. |

> **Crucial Invariant:** Only vertices that are **locally inconsistent** ($g(u) \neq rhs(u)$) are stored in the Priority Queue. Consistent vertices require zero computational work!

---

### 3.4 Priority Queue Key Structure & Lexicographical Ordering

Inconsistent vertices are stored in a priority queue $U$, sorted by a 2-tuple key $k(u) = [k_1(u), \; k_2(u)]$:

$$k_1(u) = \min\big(g(u), \; rhs(u)\big) + h(u, \text{Goal})$$
$$k_2(u) = \min\big(g(u), \; rhs(u)\big)$$

#### Why a 2-tuple key?
1. **$k_1(u)$ (Primary Key):** Corresponds to the $f$-value in classic $A^*$ ($f(u) = g(u) + h(u)$). It directs search expansion toward the goal, prioritizing nodes that lie along the most promising trajectory.
2. **$k_2(u)$ (Secondary Tie-Breaker):** Corresponds to the accumulated cost $g(u)$. When two nodes have identical $f$-estimates, $k_2$ prioritizes nodes closer to the source (or deeper along the path), avoiding useless lateral expansions.

#### Lexicographical Ordering Rule:
$$k(u) < k(v) \iff \Big(k_1(u) < k_1(v)\Big) \;\lor\; \Big(k_1(u) = k_1(v) \;\land\; k_2(u) < k_2(v)\Big)$$

---

### 3.5 Detailed Mechanics: How LPA* Runs Step-by-Step

LPA* operates via two core functions: `UpdateVertex(u)` and `ComputeShortestPath()`.

```
Algorithm: Lifelong Planning A* (LPA*)
─────────────────────────────────────────────────────────────────────────────
Procedure Initialize():
  1:  U = EmptyPriorityQueue()
  2:  For every vertex u in V:
  3:      g(u) = infinity
  4:      rhs(u) = infinity
  5:  rhs(S) = 0
  6:  U.Insert(S, [h(S, Goal), 0])

Procedure UpdateVertex(u):
  7:  If u != S:
  8:      rhs(u) = min_{p in Pred(u)} (g(p) + w(p, u))
  9:  If u in U:
 10:      U.Remove(u)
 11:  If g(u) != rhs(u):
 12:      k(u) = [min(g(u), rhs(u)) + h(u, Goal),  min(g(u), rhs(u))]
 13:      U.Insert(u, k(u))

Procedure ComputeShortestPath():
 14:  While (U.TopKey() < CalculateKey(Goal) OR rhs(Goal) != g(Goal)):
 15:      u = U.Pop()
 16:      If g(u) > rhs(u):                     // Overconsistent case
 17:          g(u) = rhs(u)
 18:          For each s in Succ(u):
 19:              UpdateVertex(s)
 20:      Else:                                 // Underconsistent case
 21:          g(u) = infinity
 22:          UpdateVertex(u)
 23:          For each s in Succ(u):
 24:              UpdateVertex(s)
─────────────────────────────────────────────────────────────────────────────
```

---

### 3.6 Concrete Walkthrough Example: Handling a Dynamic Road Blockage

Let us trace what happens when an edge cost changes in a simple 4-node network:

```
        (3)
    A ───────> B
    │          │
 (1)│       (2)│
    ▼          ▼
    C ───────> D (Goal)
        (4)
```

1. **Initial Search (from Source A to Goal D):**
   * Path $A \to C \to D$ has cost $1 + 4 = 5$.
   * Path $A \to B \to D$ has cost $3 + 2 = 5$.
   * Suppose $A \to B \to D$ was chosen: $g(A)=0$, $g(B)=3$, $g(C)=1$, $g(D)=5$. All nodes are locally consistent ($g = rhs$). Priority queue is empty.

2. **Perturbation Event:**
   * Road $(B, D)$ suffers a blockage: weight increases from $2 \to \infty$.

3. **Step 1 — Immediate Delta Update:**
   * Edge $(B, D)$ is modified.
   * `UpdateVertex(D)` is invoked:
     $$rhs(D) = \min\big(g(B) + w(B, D), \; g(C) + w(C, D)\big) = \min(3 + \infty, \; 1 + 4) = 5$$
   * Since $g(D) = 5$ and $rhs(D) = 5$, node $D$ is actually consistent through bypass node $C$!
   * Parent pointer of $D$ instantly switches from $B \to C$ in $O(1)$ operations without expanding any other nodes.

4. **Suppose Road $(B, D)$ was the *only* path to $D$:**
   * Then $rhs(D) = \infty$, while $g(D) = 5$.
   * $g(D) < rhs(D) \implies D$ is **underconsistent**.
   * $D$ is placed into the queue. It pops, sets $g(D) = \infty$, re-evaluates its predecessors, and finds the next best available alternative route.

---

## 4. Core Algorithm 2: $D^*$ Lite

*Primary Paper:* Koenig, S., & Likhachev, M. (2002). *$D^*$ Lite*. Eighteenth National Conference on Artificial Intelligence (AAAI-02).

### 4.1 Relationship to LPA* & Motivation

While LPA* plans forward from $S \to Goal$, in dynamic real-world navigation, an agent moves along the planned route:
$$S = v_0 \longrightarrow v_1 \longrightarrow v_2 \longrightarrow \dots \longrightarrow Goal$$

When an agent reaches $v_2$ and discovers a road blockage ahead at $v_5$:
* In LPA*, the start has changed ($v_2$ is now the new start). Changing the start node invalidates all $h(u, \text{Goal})$ calculations relative to the start, requiring re-keying the entire queue!
* **$D^*$ Lite's Brilliant Inversion:** **Search backwards from $Goal$ to the Agent's current position.**
  - Lookahead values $g(u)$ and $rhs(u)$ represent distances from $u \to Goal$ (instead of $S \to u$).
  - Heuristic $h(u, S_{\text{curr}})$ estimates distance from the agent's current position to $u$.

---

### 4.2 The Key Modifier $k_m$

When the agent moves from position $S_{\text{last}}$ to $S_{\text{curr}}$, all heuristic estimates change by up to $h(S_{\text{last}}, S_{\text{curr}})$. Recomputing the keys for thousands of priority queue entries would be costly.

$D^*$ Lite introduces a scalar accumulator **$k_m$ (Key Modifier)**:

```
When agent moves from S_last to S_curr:
    k_m = k_m + h(S_last, S_curr)
    S_last = S_curr
```

Instead of modifying every entry in the queue, when calculating new keys for newly inconsistent nodes:
$$k(u) = \big[\min(g(u), rhs(u)) + h(S_{\text{curr}}, u) + k_m, \;\; \min(g(u), rhs(u))\big]$$

When comparing the top of the queue against the current agent position, $k_m$ acts as a global offset, ensuring mathematical correctness with **$O(1)$ overhead** when moving!

---

### 4.3 Pseudocode for $D^*$ Lite

```
Algorithm: D* Lite
─────────────────────────────────────────────────────────────────────────────
Procedure CalculateKey(u):
  Return [min(g(u), rhs(u)) + h(S_curr, u) + k_m,  min(g(u), rhs(u))]

Procedure Initialize():
  U = EmptyPriorityQueue()
  k_m = 0
  For every vertex u in V:
      g(u) = rhs(u) = infinity
  rhs(Goal) = 0
  U.Insert(Goal, [h(S_start, Goal), 0])

Procedure UpdateVertex(u):
  If u != Goal:
      rhs(u) = min_{s in Succ(u)} (w(u, s) + g(s))
  If u in U:
      U.Remove(u)
  If g(u) != rhs(u):
      U.Insert(u, CalculateKey(u))

Procedure ComputeShortestPath():
  While (U.TopKey() < CalculateKey(S_curr) OR rhs(S_curr) != g(S_curr)):
      k_old = U.TopKey()
      u = U.Pop()
      k_new = CalculateKey(u)
      If k_old < k_new:                         // Lazy key update
          U.Insert(u, k_new)
      Else If g(u) > rhs(u):                    // Overconsistent
          g(u) = rhs(u)
          For each p in Pred(u):
              UpdateVertex(p)
      Else:                                     // Underconsistent
          g(u) = infinity
          UpdateVertex(u)
          For each p in Pred(u):
              UpdateVertex(p)

Main Navigation Loop:
  Initialize()
  ComputeShortestPath()
  While S_curr != Goal:
      If g(S_curr) == infinity:
          Return "No path exists"
      S_curr = argmin_{s in Succ(S_curr)} (w(S_curr, s) + g(s))
      MoveTo(S_curr)
      Scan for changed edge costs
      If any edge costs changed:
          k_m = k_m + h(S_last, S_curr)
          S_last = S_curr
          For each directed edge (u, v) with changed cost w_old to w_new:
              w(u, v) = w_new
              UpdateVertex(u)
          ComputeShortestPath()
─────────────────────────────────────────────────────────────────────────────
```

---

## 5. Core Baselines: Static Dijkstra & Static $A^*$

To scientifically validate that DynamicRoute's incremental algorithms produce strictly optimal routes, two classical static baselines are implemented:

### 5.1 Baseline Dijkstra (Uniform-Cost Ground Truth)
* **Mechanics:** Begins at source $S$, maintains tentative distances $d(v)$, and repeatedly extracts the vertex $u$ with the lowest $d(u)$ using a min-heap priority queue. Relaxes all outgoing edges $(u, v)$:
  $$d(v) \leftarrow \min\big(d(v), \; d(u) + w(u, v)\big)$$
* **Role in DynamicRoute:** Serves as the **mathematical ground truth**. On any dynamic event, Dijkstra can recompute the entire graph from scratch. DynamicRoute's automated test suite asserts:
  $$\text{PathCost}(D^*\text{ Lite}) \equiv \text{PathCost}(\text{Scratch Dijkstra})$$

### 5.2 Baseline $A^*$ (Heuristic Ground Truth)
* **Mechanics:** Extends Dijkstra by ordering the priority queue by $f(u) = g(u) + h(u, \text{Goal})$. Guides the search in a directed ellipse/cone toward the target.
* **Role in DynamicRoute:** Serves as the **speedup benchmark baseline**. Measures how many fewer node visits $D^*$ Lite / LPA* requires compared to a clean static $A^*$ run.

---

## 6. Optional Extension 1: Ramalingam–Reps (RR) Algorithm

*Primary Paper:* Ramalingam, G., & Reps, T. (1996). *An Incremental Algorithm for a Generalization of the Shortest-Path Problem*. ACM TOPLAS.

### 6.1 Purpose & Distinction from LPA* / $D^*$ Lite

While LPA* and $D^*$ Lite are **goal-directed single-pair** algorithms, the **Ramalingam–Reps (RR)** algorithm maintains a **Single-Source Shortest Path (SSSP) Tree** to *all* reachable vertices in the graph without using a goal-directed heuristic.

### 6.2 Two-Phase Repair Mechanics

When an edge weight increases or a road is blocked:

```
                    Edge (u, v) Weight Increases
                                │
                                ▼
        ┌─────────────────────────────────────────────────┐
        │ Phase 1: Invalidation Identification            │
        │ - Traverse the Shortest Path Tree starting at v │
        │ - Identify all downstream descendants whose     │
        │   shortest paths relied on (u, v)               │
        │ - Mark these vertices as "Invalidated" (dist=∞) │
        └─────────────────────────────────────────────────┘
                                │
                                ▼
        ┌─────────────────────────────────────────────────┐
        │ Phase 2: Priority-Queue Frontier Reconstruction │
        │ - Collect boundary vertices adjacent to the     │
        │   invalidated subtree that have valid paths     │
        │ - Insert boundary nodes into Priority Queue     │
        │ - Relax frontier outward to find new bypasses   │
        └─────────────────────────────────────────────────┘
                                │
                                ▼
                      Restored SSSP Tree
```

* **Complexity:** Strictly bounded by $O(\|\delta\| + |\delta| \log |\delta|)$, where $\delta$ is the set of vertices whose distance or parent pointer actually changes.
* **When to Use:** Ideal for fleet routing or one-to-all dispatch centers where distances from a central depot to all customer locations must be kept current simultaneously.

---

## 7. Optional Extension 2: Degree-2 Topological Compression

### 7.1 Intuition: Road Corridor Elimination

In transportation and road networks, up to $50\%$ of all vertices in OpenStreetMap data do not represent intersections. Instead, they represent curves, bends, speed limit changes, or survey points where:
$$\text{in-degree}(v) = 1 \quad \text{and} \quad \text{out-degree}(v) = 1$$

Running graph search across these intermediate vertices wastes priority queue operations.

```
Raw Corridor:
    (u) ──[w1]──> (v1) ──[w2]──> (v2) ──[w3]──> (w)

Compressed Macro-Edge:
    (u) ─────────────────[w1 + w2 + w3]────────> (w)
        (Metadata: Path Sequence = [u, v1, v2, w])
```

### 7.2 Mechanics of Topological Contraction:
1. **Detection:** Scan graph for all vertices $v$ where $\text{deg}(v) = 2$ and $v$ is not marked as a protected origin/destination or landmark.
2. **Replacement:** Replace chain $\langle u, v, w \rangle$ with a shortcut macro-edge $(u, w)$ with aggregated travel time $w(u, v) + w(v, w)$.
3. **$O(1)$ Event Mapping:** A hash table maps raw internal edge IDs to the encompassing macro-edge. When an accident occurs on $(v_1, v_2)$, the weight of macro-edge $(u, w)$ is updated in $O(1)$.
4. **$O(1)$ Path Unpacker:** Once the shortest path is found over the compressed graph $G_c$, each macro-edge is unpacked into its original constituent GPS coordinates for navigation.

---

## 8. Algorithmic Comparison & Decision Matrix

| Dimension | Standard Dijkstra | Standard $A^*$ | Ramalingam–Reps (RR) | Lifelong Planning $A^*$ (LPA\*) | $D^*$ Lite |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Category** | Core Baseline | Core Baseline | Optional Extension | Core Algorithm | Core Algorithm |
| **Search Direction** | Forward ($S \to V$) | Forward ($S \to T$) | Forward ($S \to V$) | Forward ($S \to T$) | Backward ($T \to S_{\text{curr}}$) |
| **Heuristic Used?** | No ($h = 0$) | Yes ($h(u, T)$) | No ($h = 0$) | Yes ($h(u, T)$) | Yes ($h(S_{\text{curr}}, u)$) |
| **Scope** | One-to-All | Point-to-Point | One-to-All (Tree) | Point-to-Point | Point-to-Point (Mobile) |
| **Static Query Time** | $O(\|E\| + \|V\|\log\|V\|)$ | $O(\|E\|)$ average | $O(\|E\| + \|V\|\log\|V\|)$ | $O(\|E\|)$ average | $O(\|E\|)$ average |
| **Dynamic Perturbation Cost** | Full Recompute | Full Recompute | $O(\|\delta\| + \|\delta\|\log\|\delta\|)$ | $O(\|\delta\| + \|\delta\|\log\|\delta\|)$ | $O(\|\delta\| + \|\delta\|\log\|\delta\|)$ |
| **Robot/Agent Movement** | Restart from new pos | Restart from new pos | Restart if source moves | Requires re-keying | **$O(1)$ key modifier ($k_m$)** |
| **Optimal For...** | Ground-truth verification | Static benchmarks | Dispatch / Fleet SSSP | Static source/target | Real-time driving navigation |

---

## 9. Invariant Verification & Correctness Proof Strategy

To ensure that incremental search does not introduce subtle distance corruption, the test suite implements an automated **Ground-Truth Invariant Checker**:

```
                       Dynamic Graph Generator
                                  │
                                  ▼
                     Apply Random Perturbation
                  (Weight delta / Blockage / Clear)
                                  │
                  ┌───────────────┴───────────────┐
                  ▼                               ▼
       DynamicRoute Engine               Ground-Truth Baseline
       (D* Lite / LPA*)                 (Scratch Dijkstra)
                  │                               │
                  ▼                               ▼
            Cost_incremental                Cost_dijkstra
                  │                               │
                  └───────────────┬───────────────┘
                                  │
                                  ▼
                 ASSERT(Cost_incremental == Cost_dijkstra)
                                  │
                                  ▼
                   Verification Log: 100% Match
```

Any discrepancy between incremental route costs and ground-truth Dijkstra triggers immediate invariant failure, identifying the exact vertex and edge responsible for state corruption.
