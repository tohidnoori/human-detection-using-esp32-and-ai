# ESP32-S3 Human Detection — Experiment 12
## DBSCAN and Kalman Tracking Performance Comparison

## 1. Objective

The goal of this experiment is to determine which processing pipeline is most suitable for real-time human detection and tracking on the ESP32-S3.

We compare three approaches using synthetic 3D point clouds representing two moving humans observed by three sensors.

The experiment evaluates:

- Position estimation accuracy.
- Tracking continuity during crossing and merging.
- Robustness to measurement noise and missing measurements.
- Processing latency against a 100 ms frame deadline.

**Important:** All points and human trajectories in this experiment are generated synthetically. The experiment compares algorithms under controlled conditions; it does not yet validate detection accuracy with real sensor measurements.

## 2. Algorithms Compared

### Pipeline A — Kalman → DBSCAN

Each point is filtered before DBSCAN clustering is performed.

### Pipeline B — DBSCAN → Kalman

DBSCAN first identifies point clusters. Their centroids are then passed to Kalman filters for human-level tracking.

### Baseline — Raw DBSCAN

DBSCAN extracts cluster centroids without the additional human-level Kalman tracking stage.

## 3. Experimental Configuration

| Parameter | Value |
|---|---:|
| Hardware | ESP32-S3 |
| CPU frequency | 240 MHz |
| Sensors in simulation | 3 |
| Simulated humans | 2 |
| Points per human per sensor | 40 |
| Noise points per sensor | 8 |
| Maximum points per complete frame | 264 |
| Frame interval | 100 ms |
| DBSCAN epsilon | 0.35 m |
| DBSCAN MinPts | 5 |
| Default measurement noise sigma | 0.05 m |
| Frames per scenario | 300 |
| Warm-up frames | 50 |
| Random seeds | 5 |

## 4. Results

### 4.1 Position estimation accuracy

The following values are the reported unconditional RMSE, including the experiment's missed-target penalty. Lower is better.

| Scenario | Pipeline A (m) | Pipeline B (m) | Baseline (m) |
|---|---:|---:|---:|
| Normal walking | 0.01208 | **0.00880** | 0.01103 |
| Crossing | 0.36352 | **0.34983** | 0.35990 |
| Merging | 0.56289 | **0.55221** | 0.55995 |
| Low noise, σ = 0.02 | 0.00928 | **0.00781** | 0.00821 |
| Medium noise, σ = 0.10 | 0.01712 | **0.01092** | 0.01771 |
| High noise, σ = 0.20 | 0.02262 | **0.01576** | 0.03335 |
| 10% dropout | 0.02144 | **0.00985** | 0.02105 |
| 30% dropout | 0.20794 | **0.01188** | 0.20796 |
| 50% dropout | 0.38416 | **0.01488** | 0.38420 |

**Result:** Pipeline B achieved the lowest reported RMSE in all nine scenarios.

The largest apparent advantage occurs under measurement dropout and higher noise. These results require further validation to ensure that prediction-only tracks and missed-target penalties are evaluated correctly.

### 4.2 Tracking coverage

Coverage is the percentage of scored frames with at least two outputs.

| Scenario | Pipeline A | Pipeline B | Baseline |
|---|---:|---:|---:|
| Normal walking | 100.0% | 99.7% | 100.0% |
| Crossing | 82.3% | 83.4% | 82.5% |
| Merging | 57.1% | 58.3% | 57.5% |
| 10% dropout | 99.9% | 99.7% | 99.9% |
| 30% dropout | 93.3% | 99.6% | 93.3% |
| 50% dropout | 76.9% | 99.5% | 76.9% |

Pipeline B maintains high output coverage during simulated dropout. This suggests that Kalman tracking can help bridge missing measurements, but coverage alone does not prove that every output represents a correctly tracked human.

### 4.3 Processing latency

The frame deadline is 100 ms.

| Scenario | Pipeline A mean | Pipeline B mean | Pipeline B p95 |
|---|---:|---:|---:|
| Normal walking | 45.75 ms | 14.74 ms | 14.95 ms |
| Crossing | 53.74 ms | 16.44 ms | 27.07 ms |
| Merging | 69.98 ms | 20.52 ms | 27.69 ms |
| High noise | 38.23 ms | 10.60 ms | 11.12 ms |
| 50% dropout | 19.26 ms | 5.24 ms | 10.87 ms |

Pipeline B is approximately 2.6–3.9 times faster than Pipeline A by mean latency across the tested scenarios.

Its reported maximum latency is below 28 ms in these scenarios, comfortably below the 100 ms deadline.

These measurements apply to the synthetic benchmark and should not be treated as guaranteed worst-case timing for the complete real-sensor system.

## 5. Main Findings

1. **Pipeline B is the strongest candidate.** It achieved the lowest reported RMSE in all nine scenarios.
2. **Pipeline B is substantially faster.** Tracking cluster centroids is less computationally expensive than filtering every point before clustering.
3. **Kalman tracking appears helpful under simulated dropout.** B maintains high output coverage when measurements are missing.
4. **Merging remains the main algorithmic challenge.** When DBSCAN combines two humans into one cluster, the tracker may not receive enough information to estimate both independently.
5. **Real-world performance remains unverified.** Synthetic measurements cannot fully represent real antenna geometry, sensor noise, synchronization, and packet loss.

## 6. What We Are Aiming For

The ultimate goal is a reliable real-time human-detection and tracking system running on the ESP32-S3 with three physical sensors.

The next stage should focus on:

- **Accuracy:** Estimate human positions correctly using real sensor measurements.
- **Identity continuity:** Keep each person's track consistent while people cross or temporarily overlap.
- **Dropout handling:** Predict through short measurement gaps without continuing stale tracks indefinitely.
- **Real-time operation:** Process each complete frame within the 100 ms deadline, including sensor input and synchronization.
- **Reliable evaluation:** Measure position error, detection recall, false positives, missed detections, and identity switches separately.

## 7. Conclusion

Based on the current synthetic experiment, **Pipeline B — DBSCAN → centroid extraction → Kalman tracking — should be the primary candidate for the next development stage.**

It provides the best reported RMSE across the tested scenarios and significantly lower processing latency than Pipeline A.

The next objective is not simply to run more synthetic tests. It is to validate the scoring logic, test difficult merging and dropout cases, and replay real measurements from the three sensors through all three pipelines.

Only after those tests can we determine whether Pipeline B is genuinely better for the intended real-world human-detection system.

## Results Visualization

### 1. Localization Accuracy

Compares the unconditional RMSE of the three pipelines across all nine synthetic scenarios. Lower values indicate better performance. The chart uses a logarithmic scale.

![Experiment 12 RMSE Comparison](docs/images/01_rmse_comparison.png)

### 2. Processing Latency

Compares mean processing latency for Pipeline A and Pipeline B. The dashed line represents the 100 ms frame deadline required for 10 Hz processing.

![Experiment 12 Processing Latency](docs/images/02_latency_comparison.png)

### 3. Detection Coverage

Shows the percentage of frames in which each pipeline produces at least two outputs. Higher coverage indicates more consistent output, although it does not necessarily guarantee correct detections.

![Experiment 12 Detection Coverage](docs/images/03_coverage_comparison.png)

