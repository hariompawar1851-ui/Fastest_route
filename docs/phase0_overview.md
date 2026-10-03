# Phase 0 – Project Specification, Research, and System Design

## Goals of Phase 0
- **Define the problem domain** – compute shortest‑path routes on dynamic road‑network graphs.
- **Research core algorithms** – Dijkstra, A*, Lifelong Planning A* (LPA*), D*‑Lite, and optional extensions (Ramalingam‑Reps, degree‑2 compression).
- **Design a clean API** for a `Graph` type supporting:
  - Node addition with coordinates and optional label.
  - Edge creation (directed/undirected) with mutable weight and blocked‑edge flag.
  - Query helpers (`has_node`, `get_edge_weight`, `set_edge_blocked`, etc.).
- **Select a build system** – CMake with C++20, static library `dynamic_route_core`, and three executables (`dynamic_route`, `run_tests`, `run_benchmarks`).
- **Provide a CI pipeline** (GitHub Actions) that builds the project, runs the invariant test suite, and executes a smoke‑test benchmark.
- **Implement all core algorithms** using the fresh `Graph` API:
  - `Dijkstra::find_shortest_path`
  - `AStar::find_shortest_path`
  - `LifelongPlanningAStar` (LPA*)
  - `DStarLite`
- **Create a deterministic demo** (`src/main.cpp`) that builds a grid graph, runs each algorithm, simulates a dynamic incident, and reports latency and node‑expansion statistics.
- **Write an invariant test suite** (`tests/test_main.cpp`) that ensures the incremental algorithms produce identical costs to a fresh Dijkstra run across many random perturbations.
- **Add a benchmark harness** (`benchmarks/benchmark_main.cpp`) that measures latency and node‑expansion reduction for increasingly large grids (100–22 500 nodes) and supports a CI‑friendly `--smoke-test` mode.

## What Was Actually Delivered
- **CMake build configuration** (`CMakeLists.txt`).
- **GitHub Actions workflow** (`.github/workflows/ci.yml`).
- **Fully‑typed header files** for graph, nodes, edges, and each algorithm.
- **Complete source implementations** for all algorithms and the simulation utilities.
- **Executable binaries**:
  - `./bin/dynamic_route` – interactive demo.
  - `./bin/run_tests` – invariant verification suite.
  - `./bin/run_benchmarks` – performance benchmark (full + smoke mode).
- **Documentation skeleton** (README with roadmap, links to deeper docs).
- **All unit & benchmark tests pass** (see CI output).

Phase 0 therefore establishes a solid foundation for the upcoming phases: a performant adjacency representation, a complete suite of baseline and incremental shortest‑path algorithms, and a reproducible testing/benchmarking pipeline.

---

*This file is linked from the main README under the "In‑Depth Documentation" section.*
