# Experiment 03 — MinPts Sensitivity

## Objective

The objective of this experiment is to investigate the effect of the DBSCAN `MinPts` parameter on:

- Cluster structure
- Core points
- Border points
- Noise points
- Execution time
- Memory usage

The dataset and epsilon are kept constant while only `MinPts` is changed.

---

## Configuration

| Parameter | Value |
|---|---:|
| Dataset size | 1,440 |
| Dimensions | 3 |
| Epsilon | 0.20 |
| MinPts | 2, 5, 10, 20, 50, 100, 200 |
| Hardware | ESP32-S3 N16R8 |
| PSRAM | 8 MB |
| Dataset | Fixed deterministic 3D dataset |

---

# Results

| MinPts | Execution Time | Clusters | Core | Border | Noise |
|---:|---:|---:|---:|---:|---:|
| 2 | 428.304 ms | 8 | 1438 | 0 | 2 |
| 5 | 406.967 ms | 3 | 1302 | 116 | 22 |
| 10 | 295.574 ms | 10 | 560 | 644 | 236 |
| 20 | 210.354 ms | 2 | 6 | 53 | 1381 |
| 50 | 209.149 ms | 0 | 0 | 0 | 1440 |
| 100 | 209.150 ms | 0 | 0 | 0 | 1440 |
| 200 | 209.150 ms | 0 | 0 | 0 | 1440 |

For every experiment:

```text
Core + Border + Noise = 1440
```

---

# Clustering Behavior

### MinPts = 2

Almost every point qualifies as a core point.

Results:

```text
Core   = 1438
Border = 0
Noise  = 2
Clusters = 8
```

The very low density requirement allows small connected groups to form independent clusters.

---

### MinPts = 5

This configuration provides the closest result to the intended three-cluster structure.

```text
Clusters = 3

Core   = 1302
Border = 116
Noise  = 22
```

Cluster sizes:

```text
473
474
471
```

This is the most meaningful result among the tested MinPts values.

---

### MinPts = 10

Increasing MinPts makes the density requirement stricter.

```text
Core   = 560
Border = 644
Noise  = 236
Clusters = 10
```

The dataset becomes fragmented because many points no longer satisfy the core-point requirement.

---

### MinPts = 20

The density requirement becomes extremely strict.

```text
Core   = 6
Border = 53
Noise  = 1381
Clusters = 2
```

Only two small clusters survive.

---

### MinPts ≥ 50

No point satisfies the density requirement.

```text
Core   = 0
Border = 0
Noise  = 1440
Clusters = 0
```

Therefore, the entire dataset is classified as noise.

---

# Execution-Time Analysis

Execution time decreases as MinPts increases:

```text
428.304 ms
↓
406.967 ms
↓
295.574 ms
↓
210.354 ms
↓
209.149 ms
```

This does **not** mean that increasing MinPts changes the fundamental complexity.

The neighborhood search remains approximately:

```text
O(N²)
```

The reduction occurs because low MinPts values allow many points to become core points.

Core points trigger cluster expansion and additional neighborhood searches.

As MinPts increases, fewer points become core points, so cluster expansion decreases.

At MinPts values of 50 and above, every point becomes noise, greatly reducing the amount of expansion work.

---

# Memory Analysis

Memory remains constant.

| Component | Memory |
|---|---:|
| Points | 16.88 KB |
| Labels | 5.62 KB |
| Queue | 5.62 KB |
| **Total** | **28.12 KB** |

Free heap:

```text
~363 KB
```

Free PSRAM:

```text
~8161 KB
```

Therefore, MinPts does not change the allocated memory footprint.

---

# Conclusion

The experiment demonstrates that `MinPts` has a strong influence on DBSCAN's density requirement.

A low MinPts makes the algorithm permissive and allows small connected groups to become clusters.

Increasing MinPts progressively increases the density requirement, producing more border and noise points and eventually eliminating all clusters.

For this dataset:

```text
MinPts = 5
```

produced the most meaningful result, recovering three large clusters close to the expected 480 points per cluster.

The experiment also demonstrates that parameter changes can affect practical execution time without changing the underlying `O(N²)` complexity.

Overall, `MinPts` is primarily a **density-control parameter**, while its effect on execution time is indirect and depends on the amount of cluster expansion.