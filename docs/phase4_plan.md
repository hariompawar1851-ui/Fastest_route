# Phase 4 — Scalability Benchmarks on OSM & DIMACS Road Networks

> **Status:** ✅ Complete — All 5 Phase 4 Tests Passing (Tests 32–36, 36/36 Total)

---

## 1. Overview & Goals

Phase 4 elevates DynamicRoute from evaluating small synthetic grids to **stress-testing and profiling dynamic shortest-path repair across real-world road networks scaling from 1,000 to 500,000 nodes**.

Phase 4 focuses on four foundational objectives:

1. **DIMACS 9th Implementation Challenge Parser & Serializer:**
   Support the industry-standard `.gr` (arc/graph) and `.co` (coordinate) formats for large road networks, reading and persisting planar Euclidean geometry.
2. **OpenStreetMap (OSM) XML Importer & Exporter:**
   A zero-dependency, streaming XML parser for `.osm` networks that converts geographic spherical coordinates $(\text{lat}, \text{lon})$ to metric Cartesian $(x, y)$ planes, maps OSM ways to routable directed/undirected edges, and captures road hierarchy speeds and oneway constraints.
3. **High-Performance Hierarchical Road Network Generator:**
   A deterministic generator producing realistic multi-tier urban/regional topologies (motorways, secondaries, residentials) up to 500,000 vertices in sub-second execution time.
4. **Scalability Benchmarking & Profiling Suite:**
   Extensive head-to-head empirical comparisons between scratch baselines (Dijkstra, $A^*$) and incremental engines (LPA\*, $D^*$ Lite, Ramalingam-Reps) measuring wall-clock speedups, search space reduction, memory footprint, and invariant correctness across 100K–500K nodes.

---

## 2. Deliverables Summary

| File | Type | Description |
|---|---|---|
| `include/graph/graph.hpp` | **Updated** | Added `set_node_coordinates(id, x, y)` and `set_node_metadata(id, label, type)` for post-construction geometry updates |
| `src/graph/graph.cpp` | **Updated** | Implementation of coordinate and metadata modification methods |
| `include/graph/graph_io.hpp` | **Updated** | Added `load_dimacs()`, `save_dimacs()`, `load_osm_xml()`, `save_osm_xml()`, `make_hierarchical_road_network()` |
| `src/graph/graph_io.cpp` | **Updated** | Zero-dependency streaming parsers, spherical metric projections, and multi-tier road generation logic |
| `benchmarks/benchmark_main.cpp` | **Updated** | Complete scalability benchmark harness with `--quick`, `--scale`, `--dimacs`, `--osm`, `--json`, `--csv` CLI flags |
| `tests/test_main.cpp` | **Updated** | Tests 32–36 added (DIMACS roundtrip, OSM projection, Hierarchical generator, 100K-node invariant, RouteNavigator on large network) |
| `docs/phase4_plan.md` | **New** | Phase 4 architectural design and specification |
| `docs/phase4_changes.md` | **New** | Detailed report of changes, empirical benchmarks, and speedup curves |
| `docs/phase4_code_explanation.md` | **New** | Deep-dive code walkthrough of all Phase 4 implementations |

---

## 3. Technical Specifications

### 3.1 DIMACS 9th Implementation Challenge Format
The DIMACS shortest path challenge defines:
- **Graph (`.gr`) files:**
  - Comments: `c <text>`
  - Problem line: `p sp <num_nodes> <num_edges>`
  - Arc lines: `a <u> <v> <weight>` (1-indexed nodes, directed)
- **Coordinate (`.co`) files:**
  - Comments: `c <text>`
  - Problem line: `p aux sp co <num_nodes>`
  - Vertex coordinates: `v <id> <x> <y>` (1-indexed nodes)

`GraphIO::load_dimacs()` parses either separate `.gr` and `.co` files or single combined files. `GraphIO::save_dimacs()` exports graphs back to the standard format.

### 3.2 OpenStreetMap (OSM) XML Format & Projection
Real OSM road data is distributed in XML (`.osm`). `GraphIO::load_osm_xml()` performs:
1. **Node Extraction:** Extracts `id`, `lat`, and `lon` attributes for all `<node>` elements.
2. **Way Filtering:** Scans `<way>` elements, identifying routable vehicle roads via `<tag k="highway" v="..."/>` (e.g. `motorway`, `trunk`, `primary`, `secondary`, `residential`), filtering out non-vehicular ways (`footway`, `steps`, `cycleway`).
3. **Topological Compaction:** Unreferenced nodes are excluded, mapping active OSM 64-bit IDs to compact contiguous `NodeId` values $0 \dots |V|-1$.
4. **Equirectangular Metric Projection:**
   Translates spherical $(\text{lat}, \text{lon})$ to metric $(x, y)$ in meters relative to the network midpoint:
   $$x = R \cdot (\text{lon} - \text{lon}_0) \cdot \frac{\pi}{180.0} \cdot \cos\left(\text{lat}_0 \cdot \frac{\pi}{180.0}\right)$$
   $$y = R \cdot (\text{lat} - \text{lat}_0) \cdot \frac{\pi}{180.0}$$
   where $R = 6,371,000 \text{ m}$.
5. **Directionality & Speed Limits:** Supports `oneway` tags (`yes`, `1`, `-1`) and extracts `maxspeed` limits.

### 3.3 Hierarchical Road Network Generator
To enable automated reproducible benchmarks up to 500,000 nodes without external downloads:
- **Spatial Grid with Geometric Jitter:** Distributes $N$ intersections across a 2D bounding box with realistic metric spacing ($\approx 60 \text{ m}$).
- **Road Hierarchy:**
  - *Motorways:* High speed ($110 \text{ km/h}$), capacity $250$, $100\%$ connected backbone on every 10th arterial row/column.
  - *Secondary Collectors:* Medium speed ($60 \text{ km/h}$), capacity $120$, $95\%$ connectivity on every 4th corridor.
  - *Residential Streets:* Urban speed ($35 \text{ km/h}$), capacity $60$, with $15\%$ natural cul-de-sac / dead-end omissions and $8\%$ one-way configurations.
- **Average Degree:** Strictly matches real road networks ($\approx 2.8\text{--}3.4$).
- **Throughput:** Capable of synthesizing 100K nodes in $\approx 90 \text{ ms}$ and 500K nodes in $\approx 790 \text{ ms}$.

---

## 4. Phase 4 Verification Tests (32–36)

| Test | Name | Graph | Key Verification |
|---|---|---|---|
| **32** | DIMACS IO & Coordinates | 100 nodes | Roundtrip `.gr` + `.co`, exact coordinate preservation, Dijkstra == A* equivalence |
| **33** | OSM XML Parser & Metric Projection | 4 nodes, 5 edges | XML attribute parsing, metric Cartesian projection, oneway enforcement, roundtrip save |
| **34** | Hierarchical Road Generator | 2,000 nodes | Average degree $\in [2.5, 4.0]$, multi-tier road classification (motorway, secondary, residential) |
| **35** | Scalability Invariant & Speedup | 100,000 nodes | 20 dynamic edge perturbations; 100% cost agreement with Dijkstra; $\ge 95\%$ expansion reduction |
| **36** | RouteNavigator on Large Road Network | 10,000 nodes | Moving-agent D* Lite traversal with localized congestion waves; zero path mismatches across 198 steps |

---

## 5. Build and Execution

```bash
# Run all 36 automated tests
./bin/run_tests

# Run smoke test benchmark
./bin/run_benchmarks --smoke-test

# Run quick scalability benchmarks (1K, 10K, 50K nodes)
./bin/run_benchmarks --quick

# Run full scalability suite (100K, 250K, 500K nodes) and export metrics
./bin/run_benchmarks --scale --json benchmarks_scale.json --csv benchmarks_scale.csv
```

