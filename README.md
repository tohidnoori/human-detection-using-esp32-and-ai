# ESP32-S3 Human Detection Research

**Researching a real-time human-detection pipeline on the ESP32-S3 N16R8.** This repository studies efficient 3D point-cloud clustering, filtering, and tracking under embedded memory and timing constraints.

## Abstract

Human detection from multiple spatial sensors requires more than a fast clustering algorithm: the system must process measurements within a frame deadline while keeping localization stable and handling noise, dropouts, and nearby or crossing targets. This project develops and evaluates the processing pipeline in stages. **Phase 1** benchmarks DBSCAN and investigates spatial-grid optimization and dual-core execution. **Phase 2** evaluates Kalman filtering and centroid-level tracking in synthetic multi-sensor scenarios. Later phases can add physical-sensor integration, calibration, real-world evaluation, and further algorithmic improvements without changing this top-level structure.

**Current status:** the results summarized here are primarily from controlled benchmarks and synthetic simulations. They do not yet establish real-world detection accuracy or end-to-end timing with the physical sensor/UART pipeline.

## Research phases

| Phase | Scope | Status | Results |
|---|---|---|---|
| **Phase 1** | DBSCAN baseline, parameter sensitivity, spatial-grid optimization, and dual-core experiments | Benchmarked on ESP32-S3 | [Phase 1 results](results/phase1/README.md) |
| **Phase 2** | Multi-sensor pipeline timing, Kalman filtering, and centroid-level tracking comparisons | Synthetic evaluation; hardware pipeline validation remains | [Phase 2 results](results/phase2/README.md) |
| **Phase 3** | To be defined — suggested focus: physical sensor data acquisition, synchronization, and calibration | Planned | Not available yet |
| **Phase 4** | To be defined — suggested focus: real-world detection evaluation and robustness | Planned | Not available yet |
| **Phase 5+** | Additional experiments and system integration | Future work | Not available yet |

Each phase should have its own results README and link to detailed experiment reports, code, data, and figures. Add a row to this table as a phase becomes active; the abstract and existing phase reports can remain independent.

## Current platform and target

| Component | Specification |
|---|---|
| Microcontroller | ESP32-S3 N16R8 |
| CPU | Dual-core, 240 MHz |
| Flash | 16 MB |
| PSRAM | 8 MB |
| Framework / build | Arduino / PlatformIO |
| Target processing rate | 10 FPS (100 ms per frame) |

## Current research direction

The strongest current candidate from the reported synthetic comparisons is:

```text
Three sensor measurements
          ↓
Coordinate transformation and sensor fusion
          ↓
Spatial-grid DBSCAN
          ↓
Cluster validation and centroid extraction
          ↓
Kalman tracking of cluster centroids
          ↓
Human-level detections / tracks
```

This is a candidate architecture, not a validated real-world result. If DBSCAN merges nearby people into one cluster, centroid tracking may not be able to recover two separate targets from that single cluster.

## Key findings so far

- **Phase 1:** the uniform spatial grid produced the largest measured DBSCAN speedups in the tested configurations, while preserving labels against the baseline in the 13 reported comparisons.
- **Phase 1:** dual-core processing offered only modest end-to-end improvement after algorithmic optimization; cluster expansion was the most promising tested parallelization strategy.
- **Phase 2:** the synthetic Kalman microbenchmark reduced position RMSE, and centroid-level Kalman tracking was the strongest reported candidate across the tested synthetic scenarios.
- **Not yet established:** physical-sensor detection accuracy, full UART-to-output latency, performance on realistic multi-person scenes, and reliable identity continuity.

For exact configurations, tables, limitations, and figures, use the phase-specific reports linked above rather than treating these high-level findings as universal guarantees.

## Validation priorities

1. Capture and replay measurements from the actual three-sensor setup.
2. Validate coordinate transforms, calibration, timestamps, and sensor synchronization.
3. Include UART reception, parsing, fusion, clustering, and tracking in end-to-end latency measurements.
4. Evaluate precision, recall, false positives, missed detections, localization error, identity switches, track continuity, and frame-deadline misses separately.
5. Test close, crossing, merging, and dropout scenarios using recorded data.
6. Report average and worst-case latency, plus p95/p99 latency under the complete workload.

## Repository organization

```text
.
├── README.md
└── results/
    ├── images/
    ├── phase1/
    │   ├── README.md
    │   └── README_*.md
    ├── phase2/
    │   ├── README.md
    │   └── detailed experiment reports...
    └── phase3/                 # add when Phase 3 starts
```

The directory sketch is intended as a maintainable structure; retain existing report filenames and update links if the repository layout changes.
