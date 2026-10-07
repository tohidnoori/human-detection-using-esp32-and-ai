# Experiment 02 — Epsilon Sensitivity

## Objective

The purpose of this experiment is to investigate the effect of the DBSCAN epsilon parameter (`ε`) on:

- Clustering results
- Noise points
- Execution time
- Memory consumption

The dataset size, dimensionality, and `MinPts` are kept constant while only epsilon is changed.

---

## Configuration

| Parameter | Value |
|---|---:|
| Dataset size | 1,440 |
| Dimensions | 3 |
| MinPts | 5 |
| Epsilon | 0.10 – 1.50 |
| Hardware | ESP32-S3 N16R8 |
| Flash | 16 MB |
| PSRAM | 8 MB |
| Dataset | Fixed deterministic 3D dataset |

---

# Results

| Epsilon | Execution Time | Clusters | Noise |
|---:|---:|---:|---:|
| 0.10 | 423.763 ms | 3 | 3 |
| 0.20 | 429.540 ms | 3 | 0 |
| 0.30 | 432.358 ms | 3 | 0 |
| 0.50 | 438.154 ms | 3 | 0 |
| 0.80 | 439.261 ms | 3 | 0 |
| 1.00 | 439.261 ms | 3 | 0 |
| 1.50 | 439.262 ms | 3 | 0 |

---

# Clustering Behavior

The number of detected clusters remained constant at:

```text
3 clusters
```

for every tested epsilon value.

At:

```text
ε = 0.10
```

three points were classified as noise because the neighborhood radius was relatively small.

Starting from:

```text
ε = 0.20
```

all 1,440 points were successfully assigned to the three clusters.

Therefore, for this dataset and `MinPts = 5`, an epsilon of approximately `0.20` or greater was sufficient to connect the points into the intended dense regions.

---

# Execution-Time Analysis

Execution time changed only slightly:

```text
Minimum: 423.763 ms
Maximum: 439.262 ms
```

The total increase was approximately:

```text
3.7%
```

This small variation is expected because the baseline implementation scans the entire dataset for every neighborhood query.

Changing epsilon changes whether a distance qualifies as a neighbor, but it does not eliminate the need to perform the distance comparisons.

Therefore, the dominant computational behavior remains:

```text
O(N²)
```

Epsilon has a much stronger effect on clustering behavior than on execution time in this implementation.

---

# Memory Analysis

Memory remained constant.

| Component | Memory |
|---|---:|
| Points | 16.88 KB |
| Labels | 5.62 KB |
| Queue | 5.62 KB |
| **Total** | **28.12 KB** |

Free heap remained approximately:

```text
363 KB
```

and free PSRAM remained approximately:

```text
8161 KB
```

Therefore, epsilon does not significantly affect memory consumption.

---

# Conclusion

The experiment demonstrates that:

1. **Epsilon strongly affects clustering behavior.**
2. The number of clusters remained stable at three.
3. `ε = 0.10` produced three noise points.
4. `ε ≥ 0.20` produced zero noise points.
5. Execution time changed by only approximately 3.7%.
6. Memory usage remained effectively constant.

Therefore, in the current brute-force implementation, epsilon is primarily a **clustering-quality parameter** rather than a major performance or memory parameter.