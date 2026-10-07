# ESP32-S3 DBSCAN Benchmark

Performance analysis and optimization of the DBSCAN clustering algorithm on an **ESP32-S3 N16R8** embedded platform.

The project evaluates DBSCAN from several perspectives:

* Dataset-size scaling
* Epsilon sensitivity
* MinPts sensitivity
* Dataset-density sensitivity
* Baseline vs. spatially optimized DBSCAN
* Memory consumption
* Execution-time behavior
* Correctness preservation

---

## Hardware

| Component          | Specification |
| ------------------ | ------------- |
| MCU                | ESP32-S3      |
| CPU                | Dual-core     |
| CPU Frequency      | 240 MHz       |
| Flash              | 16 MB         |
| PSRAM              | 8 MB          |
| Internal Free Heap | ~363 KB       |
| Free PSRAM         | ~8 MB         |
| Framework          | Arduino       |
| Build System       | PlatformIO    |

---

## Project Structure

```text
esp32-s3-final-project-phase-1/
│
├── README.md
│
├── 
│   ├── README_01_Baseline_Scaling.md
│   ├── README_02_Epsilon_Sensitivity.md
│   ├── README_03_MinPts_Sensitivity.md
│   ├── README_04_Density_Sensitivity.md
│   └── README_05_Baseline_vs_Optimized.md
│
├── src/
│   └── main.cpp
│
├── experiments/
│   ├── 01_baseline_scaling.cpp
│   ├── 02_epsilon_sensitivity.cpp
│   ├── 03_minpts_sensitivity.cpp
│   ├── 04_density_sensitivity.cpp
│   └── 05_baseline_vs_optimized.cpp
│
└── platformio.ini
```

---

# Experiments

## Experiment 01 — Baseline Scaling

The first experiment establishes the performance baseline of a brute-force DBSCAN implementation.

The number of points is increased from:

```text
90 → 180 → 360 → 720 → 1,440
→ 2,880 → 5,760 → 11,520 → 23,040
```

The experiment demonstrates:

* Approximately `O(N²)` execution-time scaling
* `O(N)` primary memory scaling
* Computational limitation before PSRAM exhaustion
* Baseline DBSCAN performance on the ESP32-S3

[Read Experiment 01 →](README_01_Baseline_Scaling.md)

---

## Experiment 02 — Epsilon Sensitivity

This experiment evaluates the effect of the DBSCAN epsilon parameter.

Configuration:

```text
N = 1,440
Dimensions = 3
MinPts = 5
Epsilon = 0.10 – 1.50
```

The experiment investigates:

* Cluster formation
* Noise points
* Execution time
* Memory consumption
* Effect of epsilon on brute-force DBSCAN

[Read Experiment 02 →](README_02_Epsilon_Sensitivity.md)

---

## Experiment 03 — MinPts Sensitivity

This experiment evaluates how changing `MinPts` affects DBSCAN.

Configuration:

```text
N = 1,440
Dimensions = 3
Epsilon = 0.20

MinPts:
2, 5, 10, 20, 50, 100, 200
```

The experiment measures:

* Core points
* Border points
* Noise points
* Number of clusters
* Execution time
* Memory usage

[Read Experiment 03 →](README_03_MinPts_Sensitivity.md)

---

## Experiment 04 — Density Sensitivity

This experiment evaluates DBSCAN under different dataset densities.

Configuration:

```text
N = 1,440
Dimensions = 3
Epsilon = 0.20
MinPts = 5
```

Three density levels are tested:

```text
Dense   → radius = 0.25
Medium  → radius = 0.60
Sparse  → radius = 1.20
```

The experiment investigates how density affects:

* Cluster formation
* Core/border/noise classification
* Execution time
* Memory usage

[Read Experiment 04 →](README_04_Density_Sensitivity.md)

---

## Experiment 05 — Baseline vs Optimized DBSCAN

The final experiment compares the original brute-force implementation with a **uniform spatial-grid implementation**.

The optimized version limits neighborhood searches to the point's own grid cell and its 26 neighboring cells.

Three benchmark categories are used:

### 5A — Scaling

```text
N = 180, 360, 720, 1080, 1440
```

### 5B — Epsilon

```text
ε = 0.10, 0.20, 0.30, 0.50
```

### 5C — MinPts

```text
MinPts = 2, 5, 10, 20
```

The optimization achieved:

```text
Maximum total speedup: 21.48×

Speedup at N=1440: 14.75×

Memory: 69.88 KB

Correctness:
All 13 benchmark configurations matched the baseline
```

[Read Experiment 05 →](README_05_Baseline_vs_Optimized.md)

---

# Overall Findings

The experiments demonstrate several important properties of DBSCAN on embedded hardware.

### Baseline complexity

The brute-force implementation exhibits approximately:

```text
Time   → O(N²)
Memory → O(N)
```

Doubling the number of points approximately quadruples the execution time while approximately doubling the primary memory requirement.

---

### Computational bottleneck

At:

```text
N = 11,520
```

the baseline implementation required approximately:

```text
79.438 seconds
```

while the primary DBSCAN data structures occupied only:

```text
225 KB
```

with approximately:

```text
7.96 MB
```

of PSRAM still available.

Therefore, the baseline implementation is primarily:

> **Computation-bound rather than memory-bound.**

---

### Effect of DBSCAN parameters

The experiments show that:

* `ε` strongly affects neighborhood structure and clustering behavior.
* `MinPts` strongly controls the density requirement.
* Dataset density significantly affects practical execution time.
* Memory is primarily determined by `N`, not by epsilon or density.

---

### Spatial-grid optimization

The spatial-grid implementation significantly reduces practical neighborhood-search cost.

The strongest measured result was:

```text
N = 720
ε = 0.10
MinPts = 5

Baseline:
111.411 ms

Optimized total:
5.186 ms

Speedup:
21.48×
```

For the largest tested optimized dataset:

```text
N = 1440

Baseline:
837.438 ms

Optimized total:
56.784 ms

Speedup:
14.75×
```

All 13 optimized benchmark configurations produced identical results to the baseline implementation.

---

# Final Conclusion

The experiments establish a complete performance baseline for DBSCAN on the ESP32-S3 and demonstrate that spatial indexing can provide substantial practical performance improvements without requiring an `N × N` distance matrix.

The baseline implementation is limited primarily by quadratic neighborhood searching.

The optimized spatial-grid implementation significantly reduces unnecessary distance calculations while preserving the same clustering results.

Therefore, for larger datasets on resource-constrained embedded hardware, spatial indexing provides a practical and effective optimization for DBSCAN.

---

## Detailed Reports

| Experiment | Report                                                           |
| ---------- | ---------------------------------------------------------------- |
| 01         | [Baseline Scaling](README_01_Baseline_Scaling.md)           |
| 02         | [Epsilon Sensitivity](README_02_Epsilon_Sensitivity.md)     |
| 03         | [MinPts Sensitivity](README_03_MinPts_Sensitivity.md)       |
| 04         | [Density Sensitivity](README_04_Density_Sensitivity.md)     |
| 05         | [Baseline vs Optimized](README_05_Baseline_vs_Optimized.md) |
