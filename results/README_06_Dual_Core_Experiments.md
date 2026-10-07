# Experiment A–D — Dual-Core DBSCAN on ESP32-S3

## 1. Overview

This report evaluates four different approaches for using the dual-core architecture of the ESP32-S3 to accelerate DBSCAN clustering for 3D data.

The objective is not simply to make DBSCAN multithreaded, but to determine **which part of DBSCAN is worth parallelizing on an embedded dual-core MCU** and how synchronization overhead affects performance.

The experiments were performed on an **ESP32-S3 N16R8** configured at **240 MHz**, with **16 MB Flash** and **8 MB Octal PSRAM**.

The DBSCAN configuration used for the A–D experiments was:

| Parameter | Value |
|---|---:|
| Dimensions | 3 |
| Epsilon (ε) | 0.20 |
| MinPts | 5 |
| Cluster radius | 0.60 |
| Dataset sizes | 180, 360, 720, 1080, 1440 |
| CPU frequency | 240 MHz |
| Spatial grid buckets | 2048 |
| Maximum N | 1440 |

All A–D experiments use a deterministic synthetic 3D dataset, and within each comparison the single-core and dual-core implementations operate on the **same generated dataset**. The outputs are compared point-by-point using the final DBSCAN labels.

---

## 2. Hardware and Memory

The ESP32-S3 was verified to provide approximately:

- **240 MHz CPU frequency**
- **16 MB Flash**
- **8 MB PSRAM**
- approximately **360 KB internal free heap** during the benchmark runs
- more than **8 MB free PSRAM** before the experiment buffers were allocated

The implementations avoid an `N × N` distance matrix. Spatial-grid experiments use a hashed linked-list representation of occupied grid cells.

Memory reported by the A–D benchmark harnesses includes comparison and working buffers, so it should not be interpreted as the minimum theoretical memory requirement of DBSCAN itself.

---

# 3. Experiment A — Coarse-Grained Dual-Core Core Detection

## 3.1 Method

Experiment A divides the **core-point detection phase** between the two ESP32-S3 cores.

- Core 0 handles the first portion of the dataset.
- Core 1 handles the second portion.
- Each point is tested for the DBSCAN core-point condition.
- Cluster expansion remains serial.
- A persistent FreeRTOS worker and task notifications are used for synchronization.

The purpose was to test whether coarse-grained parallelism in core-point detection could reduce total DBSCAN execution time without introducing shared-label races.

## 3.2 Results

| N | Baseline | Dual-Core | Speedup | Improvement |
|---:|---:|---:|---:|---:|
| 180 | 6.694 ms | 6.776 ms | 0.99× | -1.22% |
| 360 | 30.113 ms | 30.146 ms | 1.00× | -0.11% |
| 720 | 153.441 ms | 152.543 ms | 1.01× | +0.59% |
| 1080 | 387.413 ms | 383.821 ms | 1.01× | +0.93% |
| 1440 | 693.067 ms | 685.656 ms | 1.01× | +1.07% |

For every test:

- `Different labels = 0`
- cluster count was identical
- core count was identical
- border count was identical
- noise count was identical

## 3.3 Conclusion

Coarse-grained parallelization of core-point detection produced **almost no measurable speedup**. The reason is that a substantial portion of DBSCAN execution remains serial, especially cluster expansion and neighborhood propagation.

Experiment A therefore demonstrates a practical form of **Amdahl's Law**: accelerating only one phase of DBSCAN does not significantly accelerate the complete algorithm when that phase is not the dominant bottleneck.

**Result:** Correct and stable, but not an effective optimization.

---

# 4. Experiment B — Fine-Grained Dual-Core Neighborhood Search

## 4.1 Method

Experiment B attempts to parallelize the expensive neighborhood search itself.

For each neighborhood query:

- Core 0 scans the first half of the dataset.
- Core 1 scans the second half.
- The partial neighbor counts are combined.
- Cluster expansion remains serial.

The experiment intentionally investigates a much finer level of parallelism than A.

## 4.2 Results

| N | Baseline | Dual-Core | Speedup | Improvement |
|---:|---:|---:|---:|---:|
| 180 | 6.663 ms | 11.025 ms | 0.60× | -65.47% |
| 360 | 29.444 ms | 39.729 ms | 0.74× | -34.93% |
| 720 | 140.462 ms | 176.340 ms | 0.80× | -25.54% |
| 1080 | 351.070 ms | 433.029 ms | 0.81× | -23.35% |
| 1440 | 645.262 ms | 787.090 ms | 0.82× | -21.98% |

For all dataset sizes:

- `Different labels = 0`
- all DBSCAN statistics were identical

## 4.3 Conclusion

Experiment B is the clearest demonstration that **parallelism does not automatically imply speedup** on a microcontroller.

Although more of the distance computation is theoretically parallelized, every neighborhood query requires cross-core coordination. Task notifications, synchronization, context switching, and waiting for the other core introduce enough overhead to outweigh the computational benefit.

The improvement trend from `0.60×` toward `0.82×` as N increases shows that synchronization becomes relatively less expensive for larger workloads, but it is still not enough to produce a net speedup at `N = 1440`.

**Result:** Correct, but significantly slower than the single-core baseline.

---

# 5. Experiment C — Spatial Grid + Dual-Core Core Detection

## 5.1 Method

Experiment C combines the spatial-grid optimization with the coarse-grained dual-core strategy.

The spatial grid reduces the neighborhood search from a brute-force scan over all points to a search of the 27 neighboring grid cells, followed by exact Euclidean distance checks.

The dual-core portion again divides **core-point detection** between Core 0 and Core 1.

This experiment answers an important question:

> Once DBSCAN has already been optimized with a spatial index, does adding dual-core core detection provide additional performance?

## 5.2 Results

| N | Single-Core Grid | Dual-Core Grid | Speedup | Improvement |
|---:|---:|---:|---:|---:|
| 180 | 0.976 ms | 0.979 ms | 1.00× | -0.31% |
| 360 | 2.771 ms | 2.803 ms | 0.99× | -1.15% |
| 720 | 9.755 ms | 9.752 ms | 1.00× | +0.03% |
| 1080 | 22.012 ms | 22.269 ms | 0.99× | -1.17% |
| 1440 | 37.085 ms | 36.703 ms | 1.01× | +1.03% |

For every dataset size:

- `Different labels = 0`
- DBSCAN cluster statistics were identical

## 5.3 Conclusion

Experiment C confirms the result of A in a more optimized environment: **parallel core detection is not the dominant opportunity once the spatial grid has already reduced neighborhood-search cost.**

The spatial grid itself provides the major performance benefit, while the additional dual-core core-detection stage changes the total runtime by only about one percent.

This is an important architectural finding: after algorithmic optimization, the remaining workload must be profiled before deciding where to add concurrency.

**Result:** Correct and efficient overall, but dual-core core detection adds essentially no useful speedup.

---

# 6. Experiment D — Parallel DBSCAN Cluster Expansion

## 6.1 Method

Experiment D changes the parallelization strategy completely.

Instead of parallelizing core-point detection, it parallelizes the **cluster expansion phase**, which performs repeated neighborhood propagation through the cluster frontier.

The implementation uses:

- spatial-grid neighborhood search
- a frontier-based DBSCAN expansion
- Core 0 processing the first half of the current frontier
- Core 1 processing the second half
- independent next-frontier buffers for the two cores
- a short cross-core critical section only when claiming a previously unclassified point
- serial frontier merging after both cores complete their current wave
- a threshold that keeps very small frontiers serial to avoid unnecessary synchronization overhead

This is the most substantial attempt at exploiting parallelism in the DBSCAN computation itself.

## 6.2 Results

| N | Baseline Expansion | Dual-Core Expansion | Expansion Speedup | Expansion Improvement | End-to-End Speedup |
|---:|---:|---:|---:|---:|---:|
| 180 | 0.068 ms | 0.092 ms | 0.74× | -35.29% | 0.98× |
| 360 | 0.622 ms | 0.961 ms | 0.65× | -54.50% | 0.92× |
| 720 | 5.041 ms | 6.199 ms | 0.81× | -22.97% | 0.92× |
| 1080 | 14.669 ms | 14.974 ms | 0.98× | -2.08% | 0.99× |
| **1440** | **27.831 ms** | **26.478 ms** | **1.05×** | **+4.86%** | **1.03×** |

At `N = 1440` the complete timing breakdown was:

```text
Grid build:       2.776 ms
Core detection:  16.421 ms
Baseline expand: 27.831 ms
Dual expand:     26.478 ms
```

Therefore:

```text
Baseline total = 47.028 ms
Dual total     = 45.675 ms
End-to-end     = 1.03×
```

Correctness remained exact for all tested sizes:

```text
Different labels: 0
IDENTICAL
```

Cluster count, core count, border count, and noise count were identical between the two implementations.

## 6.3 Conclusion

Experiment D is the **best of the four dual-core approaches**.

Unlike A–C, it produces a real speedup at the largest tested dataset size. At `N = 1440`, cluster expansion improves by `4.86%`, and the complete DBSCAN pipeline improves by `2.88%`.

The smaller datasets are still slower because the workload is not large enough to amortize task synchronization, frontier splitting, critical sections, and frontier merging.

The trend is therefore important:

```text
N=180   → 0.74×
N=360   → 0.65×
N=720   → 0.81×
N=1080  → 0.98×
N=1440  → 1.05×
```

As the cluster-expansion workload grows, the useful parallel computation becomes large enough to overcome the coordination overhead.

**Result:** Correct and the most effective dual-core strategy tested.

---

# 7. A–D Comparison

| Experiment | Parallelized component | Best observed result | Main finding |
|---|---|---:|---|
| A | Core-point detection | 1.01× | Too little of the total workload is parallelized |
| B | Fine-grained neighborhood search | 0.82× at N=1440 | Synchronization overhead dominates |
| C | Grid + parallel core detection | 1.01× | Grid optimization leaves core detection as a minor opportunity |
| D | Cluster expansion | **1.05× at N=1440** | Best target for dual-core parallelism |

The experiments show a clear progression:

```text
Coarse core detection
        ↓
     A ≈ 1.01×
        ↓
Fine-grained neighborhood search
        ↓
     B < 1×
        ↓
Spatial grid + core detection
        ↓
     C ≈ 1.00×
        ↓
Parallel cluster expansion
        ↓
     D = 1.05× at N=1440
```

---

# 8. Overall Scientific Findings

## 8.1 Algorithmic optimization is more important than naive parallelization

The earlier spatial-grid optimization produced a much larger performance improvement than the initial dual-core approaches. Reducing unnecessary distance calculations is more valuable than simply assigning the same work to two cores.

This is demonstrated by the difference between:

- spatial-grid optimization, which changed DBSCAN runtime by an order of magnitude at larger N in the earlier benchmark, and
- A/C, where dual-core core detection changed total runtime by roughly one percent.

## 8.2 Fine-grained synchronization is expensive on an MCU

Experiment B shows that repeatedly coordinating two FreeRTOS tasks inside neighborhood queries can make the algorithm slower than a single-core implementation.

This is especially important for embedded systems because task-management and synchronization costs are not negligible compared with floating-point distance calculations.

## 8.3 The location of parallelism matters

The experiments demonstrate that the best parallelization target is not necessarily the mathematically most expensive-looking function.

Core-point detection can involve many distance checks, but its contribution to the overall optimized workload is relatively small. Cluster expansion, on the other hand, generates repeated neighborhood propagation and becomes a more useful target as dataset size increases.

## 8.4 Amdahl's Law limits total speedup

Experiment D provides a concrete example.

At `N = 1440`:

```text
Grid build      = 2.776 ms
Core detection  = 16.421 ms
Expansion       = 27.831 ms baseline
```

Only the expansion phase was parallelized. Therefore even a faster expansion phase cannot eliminate the time spent in grid construction and core detection.

The measured result is:

```text
Expansion speedup = 1.05×
End-to-end speedup = 1.03×
```

This difference is exactly what should be expected from a partially parallel algorithm.

---

# 9. Final Conclusion

The A–D experiments show that **dual-core execution on the ESP32-S3 is feasible for DBSCAN, but meaningful acceleration requires careful workload selection and sufficiently large parallel work units**.

The main conclusions are:

1. **Parallelizing core-point detection alone is not sufficient.** Experiments A and C both produced approximately 1.00–1.01× speedup.

2. **Fine-grained per-query parallelism is counterproductive at these dataset sizes.** Experiment B remained slower than the single-core baseline, with the best observed value only 0.82×.

3. **Parallelizing cluster expansion is the most promising strategy tested.** Experiment D reached 1.05× expansion speedup and 1.03× end-to-end speedup at `N = 1440`.

4. **Correctness was preserved across all A–D experiments.** Every tested comparison reported `Different labels: 0`, with identical cluster/core/border/noise statistics.

5. **Spatial indexing remains the strongest optimization.** The dual-core results should be viewed as a second-stage optimization applied after reducing unnecessary neighborhood searches with a spatial grid.

6. **The ESP32-S3 is capable of useful DBSCAN parallelism, but the expected gain is modest for N ≤ 1440.** Synchronization, task scheduling, critical sections, and serial phases limit the achievable end-to-end acceleration.

The best architecture identified by these experiments is therefore:

```text
3D DBSCAN
    ↓
Spatial Grid
    ↓
Parallel Cluster Expansion
    ↓
ESP32-S3 Core 0 + Core 1
```

For the tested configuration, this architecture provides a **measured 2.88% end-to-end improvement at N = 1440**, while preserving exact DBSCAN output.

---

# 10. Recommended Final Architecture

For the current project and dataset scale, the recommended implementation is:

```text
Input: 3D points
        │
        ▼
   Spatial Grid
        │
        ▼
   Core Detection
        │
        ▼
Parallel Cluster Expansion
     ┌──┴──┐
     │     │
 Core 0 Core 1
     │     │
     └──┬──┘
        ▼
   Final Labels
```

The implementation should retain:

- exact squared-distance verification
- hashed spatial grid
- PSRAM for large buffers
- bounded per-core frontier buffers
- synchronization only around shared label claiming
- serial handling for small frontiers where parallel overhead would dominate

No `N × N` distance matrix is required.

---

# 11. Important Experimental Note

The A–D implementations compare single-core and dual-core methods on the **same deterministic dataset within each test**, which makes the paired speedup and correctness comparisons valid.

These A–D datasets should not automatically be treated as bit-for-bit identical to the dataset sequence used in the earlier Experiment 05 benchmark unless the exact same dataset-generator source and configuration are used. Therefore, comparisons between A–D and earlier independent benchmark tables should focus on the architectural conclusions unless the generator is explicitly matched.

---

# 12. Final Statement

**Experiment D is the best dual-core DBSCAN strategy tested on the ESP32-S3. However, the dominant performance gain in this project comes from algorithmic neighborhood reduction through the spatial grid, while dual-core execution provides a smaller second-stage optimization.**
