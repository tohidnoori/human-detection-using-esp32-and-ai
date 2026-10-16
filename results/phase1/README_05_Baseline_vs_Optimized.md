# Experiment 05 — Baseline vs Optimized DBSCAN

## Objective

The objective of this experiment is to compare the performance of the baseline brute-force DBSCAN implementation with an optimized implementation based on **spatial grid indexing**.

The baseline implementation performs a complete scan of all `N` points for neighborhood queries.

The optimized implementation divides the 3-dimensional space into grid cells with a cell size equal to epsilon.

For each point, only its own cell and the 26 neighboring cells are examined.

This reduces the number of distance calculations required in typical datasets while preserving the DBSCAN clustering behavior.

---

# Optimization Method

## Baseline

The baseline implementation performs:

```text
For each point:
    Scan all N points
    Calculate squared distance
    Check distance <= epsilon²
```

This produces approximately:

```text
O(N²)
```

neighborhood-search behavior.

---

## Optimized Spatial Grid

Each point is assigned to a grid cell:

```text
cellX = floor(x / epsilon)
cellY = floor(y / epsilon)
cellZ = floor(z / epsilon)
```

A neighborhood query examines the point's own cell and neighboring cells.

In three dimensions:

```text
3 × 3 × 3 = 27 cells
```

The grid does not replace the actual distance check.

After candidate points are retrieved, the squared Euclidean distance is still checked against:

```text
epsilon²
```

Therefore, the optimization changes the way candidate neighbors are found, but does not change the DBSCAN distance criterion.

---

# Complexity Note

The baseline implementation has approximately:

```text
O(N²)
```

neighborhood-search complexity.

The spatial-grid implementation reduces the practical number of candidate points examined for typical low-dimensional datasets.

However, it should **not** be described as having guaranteed `O(N)` worst-case complexity.

In pathological cases, many points may occupy the same or neighboring cells, causing the number of candidate comparisons to approach the baseline behavior.

Therefore:

> The spatial grid is a practical spatial-indexing optimization that reduces unnecessary distance calculations while retaining the same worst-case quadratic behavior.

---

# Experimental Design

Experiment 05 contains three benchmark groups.

## 5A — Scaling With N

| N | Epsilon | MinPts |
|---:|---:|---:|
| 180 | 0.20 | 5 |
| 360 | 0.20 | 5 |
| 720 | 0.20 | 5 |
| 1080 | 0.20 | 5 |
| 1440 | 0.20 | 5 |

## 5B — Epsilon Sensitivity

| N | Epsilon | MinPts |
|---:|---:|---:|
| 720 | 0.10 | 5 |
| 720 | 0.20 | 5 |
| 720 | 0.30 | 5 |
| 720 | 0.50 | 5 |

## 5C — MinPts Sensitivity

| N | Epsilon | MinPts |
|---:|---:|---:|
| 720 | 0.20 | 2 |
| 720 | 0.20 | 5 |
| 720 | 0.20 | 10 |
| 720 | 0.20 | 20 |

---

# Memory Configuration

| Component | Memory |
|---|---:|
| Points | 16.88 KB |
| Labels | 11.25 KB |
| Queues | 11.25 KB |
| Grid heads | 8.00 KB |
| Grid links | 5.62 KB |
| Cell coordinates | 16.88 KB |
| **Total** | **69.88 KB** |

Initial memory:

```text
Free heap  = 363 KB
Free PSRAM = 8119 KB
```

The optimized implementation does not require an `N × N` distance matrix.

A float distance matrix would require:

```text
4N² bytes
```

which quickly becomes several megabytes.

The spatial grid therefore provides a substantially more memory-efficient alternative.

---

# Experiment 5A — Scaling With N

Fixed parameters:

```text
Epsilon = 0.20
MinPts  = 5
```

| N | Baseline | Optimized DBSCAN | Grid Build | Optimized Total | Total Speedup |
|---:|---:|---:|---:|---:|---:|
| 180 | 6.964 ms | 0.972 ms | 0.460 ms | 1.432 ms | **4.86×** |
| 360 | 28.451 ms | 2.682 ms | 0.625 ms | 3.307 ms | **8.60×** |
| 720 | 151.242 ms | 11.240 ms | 1.246 ms | 12.486 ms | **12.11×** |
| 1080 | 424.513 ms | 28.688 ms | 2.152 ms | 30.840 ms | **13.77×** |
| 1440 | 837.438 ms | 53.803 ms | 2.981 ms | 56.784 ms | **14.75×** |

The total speedup includes grid construction:

```text
Total speedup =
Baseline time / (Grid build + Optimized DBSCAN)
```

This is the primary speedup metric because grid construction is a real cost of the optimized implementation.

---

## N = 1440 Result

```text
Baseline:
837.438 ms

Optimized DBSCAN:
53.803 ms

Grid construction:
2.981 ms

Optimized total:
56.784 ms
```

Therefore:

```text
837.438 / 56.784 = 14.75×
```

The grid construction overhead is small compared with the computational savings.

All five scaling configurations reported:

```text
Results identical: YES
```

---

# Experiment 5B — Epsilon Sensitivity

Fixed:

```text
N = 720
MinPts = 5
```

| Epsilon | Baseline | Optimized DBSCAN | Grid Build | Optimized Total | Total Speedup |
|---:|---:|---:|---:|---:|---:|
| 0.10 | 111.411 ms | 3.772 ms | 1.414 ms | 5.186 ms | **21.48×** |
| 0.20 | 151.239 ms | 11.169 ms | 1.297 ms | 12.466 ms | **12.13×** |
| 0.30 | 225.033 ms | 32.538 ms | 1.297 ms | 33.835 ms | **6.65×** |
| 0.50 | 231.401 ms | 74.894 ms | 1.288 ms | 76.182 ms | **3.04×** |

---

## Analysis

The optimization is most effective when epsilon is small.

```text
ε = 0.10 → 21.48×
ε = 0.20 → 12.13×
ε = 0.30 →  6.65×
ε = 0.50 →  3.04×
```

As epsilon increases, neighborhoods become larger.

More candidate points fall into the relevant neighboring cells, so the optimized implementation must perform more actual distance calculations.

Therefore, the advantage of spatial indexing decreases as epsilon increases.

All four configurations reported:

```text
Results identical: YES
```

Clustering results:

| Epsilon | Clusters | Noise |
|---:|---:|---:|
| 0.10 | 1 | 713 |
| 0.20 | 27 | 259 |
| 0.30 | 3 | 5 |
| 0.50 | 3 | 0 |

The optimized implementation exactly reproduces these baseline results.

---

# Experiment 5C — MinPts Sensitivity

Fixed:

```text
N = 720
Epsilon = 0.20
```

| MinPts | Baseline | Optimized DBSCAN | Grid Build | Optimized Total | Total Speedup |
|---:|---:|---:|---:|---:|---:|
| 2 | 215.957 ms | 15.420 ms | 1.316 ms | 16.736 ms | **12.90×** |
| 5 | 151.228 ms | 11.174 ms | 1.307 ms | 12.481 ms | **12.12×** |
| 10 | 112.232 ms | 8.063 ms | 1.283 ms | 9.346 ms | **12.01×** |
| 20 | 111.060 ms | 7.958 ms | 1.283 ms | 9.241 ms | **12.02×** |

The relative performance improvement remains approximately:

```text
12× – 13×
```

for all tested MinPts values.

This indicates that epsilon and spatial density have a stronger influence on spatial-grid efficiency than MinPts in this benchmark.

Clustering results:

| MinPts | Clusters | Noise |
|---:|---:|---:|
| 2 | 15 | 43 |
| 5 | 27 | 259 |
| 10 | 3 | 687 |
| 20 | 0 | 720 |

Again:

```text
Results identical: YES
```

for every configuration.

---

# Overall Results

## 1. Significant Performance Improvement

The maximum measured end-to-end speedup was:

```text
21.48×
```

at:

```text
N = 720
Epsilon = 0.10
MinPts = 5
```

For the largest scaling test:

```text
N = 1440
```

the total speedup was:

```text
14.75×
```

---

## 2. Speedup Increases With Dataset Size

```text
N = 180   →  4.86×
N = 360   →  8.60×
N = 720   → 12.11×
N = 1080  → 13.77×
N = 1440  → 14.75×
```

Spatial indexing becomes increasingly beneficial as the brute-force neighborhood-search cost increases.

---

## 3. Epsilon Strongly Affects Optimization Efficiency

```text
ε = 0.10 → 21.48×
ε = 0.20 → 12.13×
ε = 0.30 →  6.65×
ε = 0.50 →  3.04×
```

Small epsilon values allow the spatial grid to eliminate many irrelevant candidate points.

---

## 4. MinPts Has a Smaller Effect

The total speedup remains close to:

```text
12× – 13×
```

for all tested MinPts values.

Therefore, in this implementation, epsilon and spatial density have a stronger influence on the optimization benefit than MinPts.

---

## 5. Exact Correctness Was Preserved

All 13 benchmark configurations reported:

```text
Results identical: YES
```

This provides strong experimental evidence that the optimized implementation preserves the baseline clustering results across the tested parameter space.

---

## 6. Low Memory Overhead

The complete optimized data structure requires:

```text
69.88 KB
```

and does not require an `N × N` distance matrix.

---

# Final Conclusion

Experiment 05 demonstrates that spatial grid indexing is an effective optimization for DBSCAN on the ESP32-S3.

The optimized implementation significantly reduces execution time while preserving the same clustering results as the brute-force baseline.

The strongest result was:

```text
N = 720
Epsilon = 0.10
MinPts = 5

Baseline:
111.411 ms

Optimized total:
5.186 ms

Speedup:
21.48×
```

For the largest tested dataset:

```text
N = 1440

Baseline:
837.438 ms

Optimized total:
56.784 ms

Speedup:
14.75×
```

The optimization is particularly effective for small epsilon values and larger datasets.

Importantly, the optimized implementation achieves these improvements without requiring an `N × N` distance matrix.

The spatial-grid approach therefore provides a practical performance optimization for DBSCAN on resource-constrained embedded hardware while maintaining the same clustering results observed in the baseline implementation.

---

# Experiment 05 Summary

```text
========================================
EXPERIMENT 05 SUMMARY
========================================

Optimization:
Uniform spatial grid indexing

Baseline:
Brute-force neighborhood search

Maximum total speedup:
21.48×

Speedup at N=1440:
14.75×

Largest tested dataset:
N = 1440

Optimized total time at N=1440:
56.784 ms

Baseline time at N=1440:
837.438 ms

Memory:
69.88 KB

Correctness:
All 13 tests matched the baseline

Worst-case complexity:
O(N²)

Practical benefit:
Substantial reduction in neighborhood distance checks

========================================
```