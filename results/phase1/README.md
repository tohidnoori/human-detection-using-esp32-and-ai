# Phase 1 Results — DBSCAN Benchmarking and Optimization

**Platform:** ESP32-S3 N16R8, dual-core 240 MHz, 16 MB flash, 8 MB PSRAM  
**Framework:** Arduino / PlatformIO  
**Scope:** 3D DBSCAN runtime, parameter sensitivity, spatial indexing, memory use, and dual-core strategies.

## Abstract

Phase 1 establishes a brute-force DBSCAN baseline and evaluates approaches for reducing its computational cost on the ESP32-S3. The experiments vary dataset size, epsilon, MinPts, and point density; compare brute-force DBSCAN with a uniform spatial grid; and test four dual-core strategies. The central result is that **spatial indexing delivered substantially greater speedups than the tested parallelization strategies**, while the reported optimized configurations preserved the baseline clustering labels.

## Main results

- The brute-force implementation showed approximately **O(N²)** runtime scaling and **O(N)** primary memory scaling.
- The baseline report recorded about **79.438 s at N = 11,520** and about **225 KB** for primary data structures in that benchmark.
- The uniform spatial grid searches the point's own cell and its 26 neighboring cells, then applies exact squared-distance checks.
- The maximum reported speedup was **21.48×** at **N = 720, ε = 0.10, MinPts = 5**.
- At **N = 1,440**, the reported optimized implementation reached **14.75×** speedup in the relevant benchmark.
- The reported grid implementation used about **69.88 KB** for the listed data structures.
- All **13 reported optimized benchmark configurations** matched the baseline clustering results.
- Among the tested dual-core strategies, parallel cluster expansion was the most effective, reaching **1.03× end-to-end speedup at N = 1,440** in the reported comparison. This is a modest gain and indicates that synchronization and serial work limit the benefit of using both cores.

> **Interpretation:** the speedups are specific to the reported data distributions, epsilon values, MinPts settings, and implementations. They should not be treated as guaranteed speedups for all point clouds.

## Experiment index

| Experiment | Question | Detailed report |
|---|---|---|
| 01 — Baseline scaling | How do runtime and memory change as point count increases? | [`README_01_Baseline_Scaling.md`](README_01_Baseline_Scaling.md) |
| 02 — Epsilon sensitivity | How does ε affect clusters, noise, runtime, and memory? | [`README_02_Epsilon_Sensitivity.md`](README_02_Epsilon_Sensitivity.md) |
| 03 — MinPts sensitivity | How does MinPts affect core/border/noise classification and performance? | [`README_03_MinPts_Sensitivity.md`](README_03_MinPts_Sensitivity.md) |
| 04 — Density sensitivity | How does point-cloud density affect clustering and runtime? | [`README_04_Density_Sensitivity.md`](README_04_Density_Sensitivity.md) |
| 05 — Baseline vs. spatial grid | Does spatial indexing accelerate DBSCAN while preserving results? | [`README_05_Baseline_vs_Optimized.md`](README_05_Baseline_vs_Optimized.md) |
| 06 — Dual-core strategies | Which parts of DBSCAN benefit from parallel execution? | [`README_06_Dual_Core_Experiments.md`](README_06_Dual_Core_Experiments.md) |
| 07 — Real-time sensor pipeline simulation | How does the processing pipeline behave under the reported simulated load? | [`README_07_Realtime_Sensor_Pipeline.md`](README_07_Realtime_Sensor_Pipeline.md) |

## What Phase 1 establishes

1. Brute-force DBSCAN is too computationally expensive as the number of points grows.
2. A spatial grid is the main measured optimization in the tested configurations.
3. Clustering correctness must be checked alongside timing; faster output is not useful if labels change unexpectedly.
4. Dual-core execution is not automatically faster. The parallel region, synchronization overhead, and serial portions of the algorithm matter.
5. Parameter sensitivity and point density influence DBSCAN behavior, so performance should be measured across more than one configuration.

## Limitations

- Benchmark results depend on the tested synthetic/deterministic point sets and configurations.
- Runtime microbenchmarks do not by themselves demonstrate accurate human detection.
- A processing target of 100 ms per frame must be checked against the complete application, not just DBSCAN.
- Memory figures should be interpreted according to the specific allocations measured in each report.

## Conclusion and handoff to Phase 2

Phase 1 identifies the uniform spatial grid as the strongest tested approach for reducing DBSCAN computation on the ESP32-S3. Dual-core parallelization produced smaller gains after this optimization. Phase 2 builds on this work by evaluating filtering, centroid extraction, and tracking strategies for a human-detection pipeline.
