# ESP32-S3 DBSCAN Benchmark

Performance analysis and optimization of the DBSCAN clustering algorithm on an **ESP32-S3 N16R8** embedded platform.

The project evaluates DBSCAN from several perspectives:

- Dataset-size scaling
- Epsilon sensitivity
- MinPts sensitivity
- Dataset-density sensitivity
- Baseline vs. spatially optimized DBSCAN
- Dual-core parallelization strategies
- Memory consumption
- Execution-time behavior
- Correctness preservation

---

## Hardware

| Component | Specification |
|---|---|
| MCU | ESP32-S3 |
| CPU | Dual-core |
| CPU Frequency | 240 MHz |
| Flash | 16 MB |
| PSRAM | 8 MB |
| Internal Free Heap | ~363 KB |
| Free PSRAM | ~8 MB |
| Framework | Arduino |
| Build System | PlatformIO |

---

## Project Structure

```text
esp32-s3-final-project-phase-1/
│
├── README.md
│
├── README_01_Baseline_Scaling.md
├── README_02_Epsilon_Sensitivity.md
├── README_03_MinPts_Sensitivity.md
├── README_04_Density_Sensitivity.md
├── README_05_Baseline_vs_Optimized.md
├── README_06_Dual_Core_Experiments.md
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

- Approximately `O(N²)` execution-time scaling
- `O(N)` primary memory scaling
- Computational limitation before PSRAM exhaustion
- Baseline DBSCAN performance on the ESP32-S3

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

- Cluster formation
- Noise points
- Execution time
- Memory consumption
- Effect of epsilon on brute-force DBSCAN

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

- Core points
- Border points
- Noise points
- Number of clusters
- Execution time
- Memory usage

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

- Cluster formation
- Core/border/noise classification
- Execution time
- Memory usage

[Read Experiment 04 →](README_04_Density_Sensitivity.md)

---

## Experiment 05 — Baseline vs Optimized DBSCAN

The final single-core optimization experiment compares the original brute-force implementation with a **uniform spatial-grid implementation**.

The optimized version limits neighborhood searches to the point's own grid cell and its 26 neighboring cells, while still performing exact squared-distance checks.

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

## Experiment 06 — Dual-Core DBSCAN

After the spatial-grid optimization, the project investigates whether the ESP32-S3's two CPU cores can provide additional acceleration.

Four dual-core strategies are evaluated:

```text
Experiment A → Coarse-grained core-point detection
Experiment B → Fine-grained neighborhood search
Experiment C → Spatial Grid + dual-core core detection
Experiment D → Parallel cluster expansion
```

The experiments compare single-core and dual-core execution using deterministic 3D datasets with:

```text
N = 180, 360, 720, 1080, 1440
ε = 0.20
MinPts = 5
```

The dual-core study focuses on:

- Speedup and execution-time improvement
- Synchronization overhead
- Workload partitioning
- Correctness preservation
- The effect of Amdahl's Law
- The most effective DBSCAN phase to parallelize on an embedded dual-core MCU

The main finding is that **parallelizing cluster expansion is more effective than parallelizing core detection or performing very fine-grained per-query parallelism**. At `N = 1440`, Experiment D achieved `1.05×` speedup for cluster expansion and `1.03×` end-to-end speedup, while all tested A–D configurations preserved identical DBSCAN labels.

[Read Experiment 06 — Dual-Core Experiments →](README_06_Dual_Core_Experiments.md)

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

- `ε` strongly affects neighborhood structure and clustering behavior.
- `MinPts` strongly controls the density requirement.
- Dataset density significantly affects practical execution time.
- Memory is primarily determined by `N`, not by epsilon or density.

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

### Dual-core optimization

The A–D experiments show that adding a second CPU core does not automatically improve DBSCAN performance.

The main conclusions are:

- Coarse-grained core detection provides only about `1.00–1.01×` speedup.
- Fine-grained neighborhood parallelism is slower because synchronization overhead dominates.
- Spatial Grid + dual-core core detection also provides approximately `1.00×` speedup.
- Parallel cluster expansion is the most promising dual-core strategy, reaching `1.05×` expansion speedup and `1.03×` end-to-end speedup at `N = 1440`.

Therefore, **algorithmic optimization through spatial indexing remains much more important than naive parallelization**, while dual-core execution is best treated as a second-stage optimization after reducing unnecessary neighborhood searches.

[See the complete A–D analysis →](README_06_Dual_Core_Experiments.md)

---

## Real-Time Three-Sensor Pipeline

This experiment evaluates a realistic three-sensor 3D human-detection pipeline using simulated sensor observations, coordinate fusion, spatial-grid DBSCAN, and human-like cluster analysis.

At a target of 10 FPS, the corrected simulation sustains:

- 660 points/frame
- 6,600 points/s input rate
- 89.357 ms average pipeline time
- 96.496 ms maximum measured pipeline time
- stable detection of 2 simulated humans

The first tested failing configuration was 684 points/frame, where the maximum pipeline time reached 101.247 ms.

[Read Experiment 07B →](README_07_Realtime_Sensor_Pipeline.md)

# Final Conclusion

The experiments establish a complete performance baseline for DBSCAN on the ESP32-S3 and demonstrate that spatial indexing can provide substantial practical performance improvements without requiring an `N × N` distance matrix.

The baseline implementation is limited primarily by quadratic neighborhood searching.

The optimized spatial-grid implementation significantly reduces unnecessary distance calculations while preserving the same clustering results.

The dual-core experiments further show that **where parallelism is introduced matters more than simply using both CPU cores**. The best tested dual-core design parallelizes cluster expansion on top of the spatial-grid implementation, but the overall gain remains modest because grid construction, core detection, synchronization, and other serial phases still limit total speedup.

Therefore, for larger datasets on resource-constrained embedded hardware, the recommended architecture is:

```text
3D DBSCAN
    ↓
Spatial Grid
    ↓
Parallel Cluster Expansion
    ↓
ESP32-S3 Core 0 + Core 1
```

This design preserves exact clustering results while providing the best measured dual-core performance among the tested approaches.

---

## Detailed Reports

| Experiment | Report |
|---|---|
| 01 | [Baseline Scaling](README_01_Baseline_Scaling.md) |
| 02 | [Epsilon Sensitivity](README_02_Epsilon_Sensitivity.md) |
| 03 | [MinPts Sensitivity](README_03_MinPts_Sensitivity.md) |
| 04 | [Density Sensitivity](README_04_Density_Sensitivity.md) |
| 05 | [Baseline vs Optimized](README_05_Baseline_vs_Optimized.md) |
| 06 | [Dual-Core Experiments A–D](README_06_Dual_Core_Experiments.md) |
