# Experiment 01 — Baseline DBSCAN Scaling

## Objective

The first experiment establishes the performance baseline of a brute-force DBSCAN implementation on the ESP32-S3 N16R8.

The primary objectives are to measure:

- Execution time as dataset size increases
- Memory consumption
- Cluster detection
- Noise points
- Scaling behavior

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

---

## DBSCAN Configuration

| Parameter | Value |
|---|---:|
| Dimensions | 3 |
| Epsilon (`ε`) | 0.50 |
| MinPts | 5 |
| Distance metric | Euclidean |
| Dataset storage | PSRAM |
| Labels storage | PSRAM |
| Expansion queue | PSRAM |

The benchmark uses a synthetic 3-dimensional dataset containing three spatial clusters.

Dataset sizes:

```text
90
180
360
720
1,440
2,880
5,760
11,520
23,040
```

---

# Results

| N | Points | Labels | Queue | Total PSRAM | Execution Time | Clusters | Noise |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 90 | 1.05 KB | 0.35 KB | 0.35 KB | 1.76 KB | 2.331 ms | 3 | 3 |
| 180 | 2.11 KB | 0.70 KB | 0.70 KB | 3.52 KB | 9.622 ms | 3 | 1 |
| 360 | 4.22 KB | 1.41 KB | 1.41 KB | 7.03 KB | 38.456 ms | 3 | 0 |
| 720 | 8.44 KB | 2.81 KB | 2.81 KB | 14.06 KB | 153.528 ms | 3 | 0 |
| 1,440 | 16.88 KB | 5.62 KB | 5.62 KB | 28.12 KB | 613.605 ms | 3 | 0 |
| 2,880 | 33.75 KB | 11.25 KB | 11.25 KB | 56.25 KB | 3,720.202 ms | 3 | 0 |
| 5,760 | 67.50 KB | 22.50 KB | 22.50 KB | 112.50 KB | 19,862.623 ms | 3 | 0 |
| 11,520 | 135.00 KB | 45.00 KB | 45.00 KB | 225.00 KB | 79,437.742 ms | 3 | 0 |
| 23,040 | 270.00 KB | 90.00 KB | 90.00 KB | 450.00 KB | In progress | — | — |

> The `N = 23,040` execution was not included because the benchmark output ended before a final result was reported.

---

# Execution-Time Scaling

The relationship between dataset size and execution time is approximately quadratic.

| N | Execution Time | Ratio |
|---:|---:|---:|
| 90 | 2.331 ms | — |
| 180 | 9.622 ms | 4.13× |
| 360 | 38.456 ms | 4.00× |
| 720 | 153.528 ms | 3.99× |
| 1,440 | 613.605 ms | 4.00× |
| 2,880 | 3,720.202 ms | 6.06× |
| 5,760 | 19,862.623 ms | 5.34× |
| 11,520 | 79,437.742 ms | 4.00× |

For the range from 90 to 1,440 points:

```text
N × 2  →  execution time × ~4
```

This is consistent with:

```text
T(N) ∝ N²
```

Therefore, the baseline implementation exhibits approximately:

```text
O(N²)
```

time complexity.

The reason is that neighborhood searches perform a linear scan through the dataset. As `N` increases, the number of distance comparisons grows approximately as `N²`.

---

# Practical Impact

The quadratic behavior becomes increasingly important at larger dataset sizes.

```text
N = 1,440   → 0.614 seconds
N = 2,880   → 3.720 seconds
N = 5,760   → 19.863 seconds
N = 11,520  → 79.438 seconds
```

At:

```text
N = 11,520
```

the primary dataset memory is only:

```text
225 KB
```

while approximately:

```text
7.96 MB
```

of PSRAM remains available.

Therefore:

> **The baseline implementation is computation-bound rather than memory-bound.**

---

# Memory Analysis

Each point contains three `float` values:

```text
3 × 4 bytes = 12 bytes
```

Therefore:

```text
Points = 12N bytes
```

The implementation additionally stores:

```text
Labels = 4N bytes
Queue  = 4N bytes
```

Total primary memory:

```text
12N + 4N + 4N
= 20N bytes
```

Therefore:

```text
M(N) = O(N)
```

For example:

```text
N = 11,520

11,520 × 20
= 230,400 bytes
≈ 225 KB
```

This matches the measured result.

---

# Memory vs Computational Scaling

The experiment demonstrates two different scaling behaviors.

### Memory

```text
M(N) = O(N)
```

Doubling `N` approximately doubles memory.

### Execution Time

```text
T(N) = O(N²)
```

Doubling `N` approximately quadruples execution time.

Therefore:

```text
Memory:

90 → 180 → 360 → ...
     ×2     ×2


Execution:

90 → 180 → 360 → ...
     ×4     ×4
```

This difference is the central result of the baseline experiment.

---

# DBSCAN Correctness

For the tested configuration:

```text
ε = 0.50
MinPts = 5
```

the algorithm consistently identified:

```text
3 clusters
```

for every completed dataset size.

The noise count changed:

```text
3 → 1 → 0
```

as the dataset size increased.

The stable detection of the three main clusters indicates that the implementation successfully identifies the intended large-scale cluster structure.

However, the changing noise count indicates that the synthetic dataset generator should be improved before using the benchmark as a formal correctness evaluation.

Future experiments should use controlled datasets with known ground truth to separately evaluate:

1. Clustering correctness
2. Execution time
3. Memory usage

---

# Conclusion

The baseline experiment demonstrates that the ESP32-S3 can process substantially larger DBSCAN datasets than the original 90-point configuration.

The largest completed benchmark was:

```text
N = 11,520

Execution time = 79.438 seconds
Memory usage    = 225 KB
Clusters        = 3
Noise           = 0
```

while approximately:

```text
7.96 MB
```

of PSRAM remained available.

Therefore, the primary limitation of the baseline implementation is computational complexity rather than available memory.

This experiment establishes the baseline required for the subsequent parameter-sensitivity and optimization experiments.