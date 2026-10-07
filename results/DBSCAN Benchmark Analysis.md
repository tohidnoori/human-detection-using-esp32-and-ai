# DBSCAN Benchmark Analysis

## 1. Experimental Configuration

The first benchmark evaluates the performance of a baseline DBSCAN implementation on the ESP32-S3 N16R8.

### Hardware

* **MCU:** ESP32-S3
* **CPU:** Dual-core
* **CPU Frequency:** 240 MHz
* **Flash:** 16 MB
* **PSRAM:** 8 MB
* **Internal Free Heap:** approximately 363 KB during the benchmark
* **Free PSRAM:** approximately 8 MB

### DBSCAN Configuration

| Parameter       |              Value |
| --------------- | -----------------: |
| Dimensions      |                  3 |
| Epsilon (`ε`)   |               0.50 |
| MinPts          |                  5 |
| Distance metric | Euclidean distance |
| Dataset storage |              PSRAM |
| Labels storage  |              PSRAM |
| Expansion queue |              PSRAM |

The dataset sizes were increased by a factor of two for each experiment:

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

The benchmark uses a synthetic 3-dimensional dataset containing three spatial clusters.

---

## 2. Benchmark Results

The following results were obtained on the ESP32-S3 running at 240 MHz.

|      N | Points Memory |   Labels |    Queue | Total PSRAM | Execution Time | Clusters | Noise |
| -----: | ------------: | -------: | -------: | ----------: | -------------: | -------: | ----: |
|     90 |       1.05 KB |  0.35 KB |  0.35 KB |     1.76 KB |       2.331 ms |        3 |     3 |
|    180 |       2.11 KB |  0.70 KB |  0.70 KB |     3.52 KB |       9.622 ms |        3 |     1 |
|    360 |       4.22 KB |  1.41 KB |  1.41 KB |     7.03 KB |      38.456 ms |        3 |     0 |
|    720 |       8.44 KB |  2.81 KB |  2.81 KB |    14.06 KB |     153.528 ms |        3 |     0 |
|  1,440 |      16.88 KB |  5.62 KB |  5.62 KB |    28.12 KB |     613.605 ms |        3 |     0 |
|  2,880 |      33.75 KB | 11.25 KB | 11.25 KB |    56.25 KB |   3,720.202 ms |        3 |     0 |
|  5,760 |      67.50 KB | 22.50 KB | 22.50 KB |   112.50 KB |  19,862.623 ms |        3 |     0 |
| 11,520 |     135.00 KB | 45.00 KB | 45.00 KB |   225.00 KB |  79,437.742 ms |        3 |     0 |
| 23,040 |     270.00 KB | 90.00 KB | 90.00 KB |   450.00 KB |    In progress |        — |     — |

> **Note:** The `N = 23,040` execution time was not included because the provided benchmark output ended before the result was reported.

---

## 3. Execution-Time Scaling

The most important observation is the relationship between dataset size and execution time.

When the dataset size is doubled, the execution time increases by approximately four times.

|      N |          Time | Time Ratio vs. Previous N |
| -----: | ------------: | ------------------------: |
|     90 |      2.331 ms |                         — |
|    180 |      9.622 ms |                     4.13× |
|    360 |     38.456 ms |                     4.00× |
|    720 |    153.528 ms |                     3.99× |
|  1,440 |    613.605 ms |                     4.00× |
|  2,880 |  3,720.202 ms |                     6.06× |
|  5,760 | 19,862.623 ms |                     5.34× |
| 11,520 | 79,437.742 ms |                     4.00× |

For the range from 90 to 1,440 points, the scaling is particularly clear:

```text
N × 2  →  execution time × ~4
```

This is consistent with:

$$
T(N) \propto N^2
$$

Therefore, the current implementation exhibits approximately:

$$
\boxed{O(N^2)}
$$

time complexity for the tested datasets.

This behavior is expected because the implementation performs a linear scan of the entire dataset when searching for neighboring points. Consequently, as `N` increases, the number of distance calculations grows approximately as:

$$
N^2
$$

---

## 4. Practical Impact of the Quadratic Complexity

The quadratic behavior becomes increasingly important at larger dataset sizes.

For example:

```text
N = 1,440    → 0.614 seconds
N = 2,880    → 3.720 seconds
N = 5,760    → 19.863 seconds
N = 11,520   → 79.438 seconds
```

The difference between a small and large dataset is therefore substantial.

Although the ESP32-S3 is capable of storing the data in PSRAM, the computation becomes the limiting factor long before the available memory is exhausted.

A particularly important observation is:

```text
N = 11,520

Dataset memory:
225 KB

Available PSRAM:
~7,964 KB
```

Only a small fraction of the available PSRAM is being used, while DBSCAN already requires approximately **79 seconds**.

Therefore:

> **The current implementation is computation-bound rather than memory-bound.**

---

## 5. Memory Analysis

The memory requirements scale linearly with the number of points.

Each point contains three `float` values:

```text
3 × 4 bytes = 12 bytes
```

Therefore:

$$
Memory_{points} = 12N
$$

The implementation additionally stores:

* one `int` label per point → `4N` bytes
* one `int` queue entry per point → `4N` bytes

Thus, the primary DBSCAN memory requirement is approximately:

$$
12N + 4N + 4N
$$

or:

$$
\boxed{20N\ bytes}
$$

This matches the measured results.

For example, for:

```text
N = 11,520
```

the expected memory is:

$$
11,520 \times 20 = 230,400\ bytes
$$

which is approximately:

```text
225 KB
```

The measured value was exactly:

```text
225 KB
```

This confirms that the memory model is behaving as expected.

---

## 6. Memory vs. Computational Limit

The benchmark demonstrates two very different scaling behaviors.

### Memory

Memory increases linearly:

$$
\boxed{M(N)=O(N)}
$$

Doubling `N` approximately doubles memory usage.

### Execution Time

Execution time increases approximately quadratically:

$$
\boxed{T(N)=O(N^2)}
$$

Doubling `N` approximately quadruples the execution time.

Therefore:

```text
Memory:
90 → 180 → 360 → ...
     ×2     ×2

Execution:
90 → 180 → 360 → ...
     ×4     ×4
```

This difference is the central result of the first benchmark.

---

## 7. DBSCAN Correctness

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

The noise count decreased from:

```text
3 → 1 → 0
```

as the dataset size increased.

The stable detection of three clusters indicates that the implementation is successfully identifying the intended large-scale cluster structure.

However, the changing noise count means that the synthetic dataset generator should be improved before using the benchmark as a formal algorithm-correctness evaluation.

For future experiments, a controlled dataset with known ground truth should be used, allowing the benchmark to separately evaluate:

1. clustering correctness
2. execution time
3. memory usage

---

# 8. First-Level Conclusion

The first ESP32-S3 DBSCAN experiment demonstrates that the baseline implementation is capable of processing datasets much larger than the original 90-point test.

The largest completed benchmark was:

```text
N = 11,520
```

with:

```text
Execution time = 79.438 seconds
Memory usage    = 225 KB
Clusters        = 3
Noise           = 0
```

The ESP32-S3 N16R8 still had approximately:

```text
7.96 MB free PSRAM
```

at this point.

Therefore, the limiting factor is not the available memory. The primary limitation is the quadratic computational complexity of the neighborhood-search implementation.

This establishes a clear baseline for optimization.

---

# 9. Next-Level Analysis

The next stage should not simply increase `N`. Instead, the benchmark should investigate **why DBSCAN becomes expensive and how the parameters affect its performance**.

## Experiment A — Epsilon (`ε`) Sensitivity

Keep:

```text
N = 1,440
MinPts = 5
Dimensions = 3
```

and vary:

```text
ε = 0.10
ε = 0.20
ε = 0.30
ε = 0.50
ε = 0.80
ε = 1.00
ε = 1.50
```

This experiment measures how neighborhood density affects DBSCAN.

A larger `ε` means that each point has more neighboring points, potentially causing significantly more cluster expansion work.

---

## Experiment B — MinPts Sensitivity

Keep:

```text
N = 1,440
ε = 0.50
Dimensions = 3
```

and test:

```text
MinPts = 2
MinPts = 5
MinPts = 10
MinPts = 20
MinPts = 50
MinPts = 100
```

This determines how the density threshold affects:

* number of clusters
* number of noise points
* execution time

---

## Experiment C — Dataset Density

Create datasets with different spatial densities while keeping `N` constant.

For example:

```text
Sparse
Medium
Dense
```

This is important because DBSCAN's computational behavior depends not only on `N`, but also on how many neighbors each point has.

---

## Experiment D — Algorithm Optimization

After establishing the baseline, optimize the neighborhood search.

The current implementation effectively performs:

```text
For each point:
    scan every other point
    calculate distance
```

which produces the observed:

$$
O(N^2)
$$

behavior.

Possible optimizations include:

* reducing repeated distance calculations
* minimizing PSRAM accesses
* storing frequently accessed data in internal RAM
* precomputing or caching neighborhood information
* using a spatial grid
* using spatial partitioning
* exploiting both ESP32-S3 CPU cores

The objective is to determine how much the execution time can be reduced while maintaining the same clustering results.

---

# 10. Baseline Result

The current implementation provides the following baseline:

```text
Hardware:
ESP32-S3 @ 240 MHz
16 MB Flash
8 MB PSRAM

DBSCAN:
3 dimensions
ε = 0.50
MinPts = 5

Largest completed dataset:
N = 11,520

Execution time:
79.438 seconds

Memory used:
225 KB

Clusters:
3

Noise:
0

Observed complexity:
approximately O(N²)
```

This baseline will be used to evaluate subsequent DBSCAN optimizations.
