cmake_minimum_required(VERSION 3.20)

project(DynamicRoute
    VERSION 1.0.0
    DESCRIPTION "Adaptive Shortest-Path & Route Recovery Engine for Dynamic Graphs"
    LANGUAGES CXX
)

# ---------------------------------------------------------------------------
# C++ Standard
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# Compiler Warnings
# ---------------------------------------------------------------------------
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(
        -Wall
        -Wextra
        -Wpedantic
        -Wshadow
        -Wconversion
        -Wsign-conversion
        -Wnull-dereference
    )
endif()

# ---------------------------------------------------------------------------
# Output Directories  (binaries land in <build>/bin/)
# ---------------------------------------------------------------------------
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# ---------------------------------------------------------------------------
# Include Paths
# ---------------------------------------------------------------------------
set(INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)

# ---------------------------------------------------------------------------
# Core Library  (all engine sources compiled once, linked by every target)
# ---------------------------------------------------------------------------
add_library(dynamic_route_core STATIC
    src/graph/graph.cpp
    src/algorithms/dijkstra.cpp
    src/algorithms/a_star.cpp
    src/algorithms/lpa_star.cpp
    src/algorithms/d_star_lite.cpp
    src/algorithms/ramalingam_reps.cpp
    src/simulation/event_simulator.cpp
    src/compression/degree2_compressor.cpp
)

target_include_directories(dynamic_route_core PUBLIC ${INCLUDE_DIR})

# ---------------------------------------------------------------------------
# Executable: dynamic_route  (interactive CLI / simulation demo)
# ---------------------------------------------------------------------------
add_executable(dynamic_route src/main.cpp)
target_include_directories(dynamic_route PRIVATE ${INCLUDE_DIR})
target_link_libraries(dynamic_route PRIVATE dynamic_route_core)

# ---------------------------------------------------------------------------
# Executable: run_tests  (invariant verification suite)
# ---------------------------------------------------------------------------
add_executable(run_tests tests/test_main.cpp)
target_include_directories(run_tests PRIVATE ${INCLUDE_DIR})
target_link_libraries(run_tests PRIVATE dynamic_route_core)

# ---------------------------------------------------------------------------
# Executable: run_benchmarks  (latency & node-expansion benchmarks)
# ---------------------------------------------------------------------------
add_executable(run_benchmarks benchmarks/benchmark_main.cpp)
target_include_directories(run_benchmarks PRIVATE ${INCLUDE_DIR})
target_link_libraries(run_benchmarks PRIVATE dynamic_route_core)

# ---------------------------------------------------------------------------
# Enable CTest  (optional: wire run_tests into ctest)
# ---------------------------------------------------------------------------
enable_testing()
add_test(NAME InvariantTests COMMAND run_tests)

# ---------------------------------------------------------------------------
# Install (optional, for packaging)
# ---------------------------------------------------------------------------
install(TARGETS dynamic_route run_tests run_benchmarks
    RUNTIME DESTINATION bin
)
