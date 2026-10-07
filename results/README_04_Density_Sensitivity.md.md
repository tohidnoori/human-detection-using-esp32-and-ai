# Experiment 04 — Density Sensitivity

## Objective

The objective of this experiment is to evaluate DBSCAN under different dataset densities while keeping the number of points, dimensionality, epsilon, and MinPts constant.

This experiment investigates how density affects:

- Cluster formation
- Core/border/noise classification
- Execution time
- Memory usage

---

## Configuration

| Parameter | Value |
|---|---:|
| Number of points | 1,440 |
| Dimensions | 3 |
| Epsilon | 0.20 |
| MinPts | 5 |
| Dense radius | 0.25 |
| Medium radius | 0.60 |
| Sparse radius | 1.20 |

The dataset contains three clusters with 480 points per cluster.

The same deterministic random pattern and cluster centers are used for all density levels. Only the cluster radius is changed.

---

# Results

| Density | Radius | Execution Time | Clusters | Core | Border | Noise |
|---|---:|---:|---:|---:|---:|---:|
| Dense | 0.25 | 916.474 ms | 3 | 1440 | 0 | 0 |
| Medium | 0.60 | 826.984 ms | 3 | 1181 | 222 | 37 |
| Sparse | 1.20 | 429.805 ms | 6 | 9 | 16 | 1415 |

---

# Dense Dataset

With:

```text
Radius = 0.25
```

all 1,440 points are classified as core points.

The algorithm identifies exactly three clusters:

```text
Cluster 1: 480
Cluster 2: 480
Cluster 3: 480
```

No border or noise points are produced.

This represents a highly dense dataset in which almost every point has sufficient neighbors within the epsilon radius.

---

# Medium-Density Dataset

With:

```text
Radius = 0.60
```

the dataset becomes less dense.

Results:

```text
Core   = 1181
Border = 222
Noise  = 37
Clusters = 3
```

Cluster sizes:

```text
468
469
466
```

The three original clusters remain detectable despite the reduction in local density.

---

# Sparse Dataset

With:

```text
Radius = 1.20
```

the local density becomes too low for the selected parameters.

Results:

```text
Core   = 9
Border = 16
Noise  = 1415
Clusters = 6
```

The six detected clusters have sizes:

```text
9
5
3
6
1
1
```

The original three-cluster structure is no longer recovered.

Approximately:

```text
98.3%
```

of the points are classified as noise.

---

# Execution-Time Analysis

Execution time decreases as the dataset becomes sparser:

```text
Dense:
916.474 ms

Medium:
826.984 ms

Sparse:
429.805 ms
```

The baseline implementation still has approximately:

```text
O(N²)
```

neighborhood-search complexity.

However, practical runtime also depends on cluster expansion.

Dense datasets contain many core points, causing substantial cluster expansion and additional neighborhood searches.

Sparse datasets contain mostly noise points. These points fail the MinPts requirement early, reducing the amount of cluster expansion.

Therefore, density affects practical execution time even though the asymptotic complexity remains unchanged.

---

# Memory Analysis

Memory remains constant across all density levels.

| Component | Size |
|---|---:|
| Points | 16.88 KB |
| Labels | 5.62 KB |
| Queue | 5.62 KB |
| **Total** | **28.12 KB** |

Free heap remained approximately:

```text
363 KB
```

Free PSRAM remained approximately:

```text
8161 KB
```

The memory footprint depends primarily on the number of points rather than dataset density.

---

# Conclusion

Experiment 04 demonstrates that DBSCAN is highly sensitive to dataset density.

The dense dataset produces three clean clusters with all points classified as core points.

At medium density, the three clusters remain detectable, but border and noise points begin to appear.

At low density, the majority of points become noise and the original cluster structure can no longer be recovered.

The experiment also demonstrates an important distinction between theoretical complexity and practical runtime.

The baseline implementation remains:

```text
O(N²)
```

but denser datasets trigger more cluster expansion and therefore require more computation.

From a memory perspective, density has little effect because the implementation allocates memory primarily according to `N`.

Overall, the results confirm that selecting appropriate DBSCAN parameters requires considering the density characteristics of the target dataset.