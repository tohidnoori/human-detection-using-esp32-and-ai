# Experiment 07B — Real-Time Three-Sensor Human Detection Pipeline

Real-time performance evaluation of a three-sensor 3D point-processing pipeline on an **ESP32-S3 N16R8**.

This experiment extends the DBSCAN benchmarks by simulating a realistic multi-sensor human-detection workload. Three sensors observe the same two moving humans from different positions and orientations. Their local measurements are transformed back into a common global coordinate system, fused, clustered using a spatial-grid DBSCAN implementation, and then evaluated with a lightweight human-like cluster classifier.

The main objective is to determine the **maximum point load that the ESP32-S3 can process in real time at 10 FPS while still maintaining stable human detection**.

---

## 1. Relationship to the Previous Experiments

This experiment represents the transition from isolated DBSCAN benchmarking to a more realistic embedded sensing pipeline.

```text
Experiments 01–04
        ↓
DBSCAN behavior and sensitivity
        ↓
Experiment 05
        ↓
Spatial-grid optimization
        ↓
Experiments A–D
        ↓
Dual-core optimization
        ↓
Experiment 07
        ↓
Three-sensor pipeline simulation
        ↓
Experiment 07B
        ↓
Real-time capacity limit
```

Experiment 07B should therefore be interpreted as a **system-level workload benchmark**, rather than only another DBSCAN microbenchmark.

See the Dual-Core results in:

[README_06_Dual_Core_Experiments.md](README_06_Dual_Core_Experiments.md)

---

## 2. Hardware and Runtime Configuration

| Parameter | Value |
|---|---:|
| MCU | ESP32-S3 N16R8 |
| CPU | Dual-core Xtensa |
| CPU frequency | 240 MHz |
| Flash | 16 MB |
| PSRAM | 8 MB Octal PSRAM |
| Framework | Arduino |
| Build system | PlatformIO |
| Number of simulated sensors | 3 |
| Number of simulated humans | 2 |
| Target frame rate | 10 FPS |
| Frame budget | 100 ms |
| DBSCAN epsilon | 0.35 |
| DBSCAN MinPts | 5 |
| Grid buckets | 4096 |
| Sensor noise | 0.025 |
| Background noise | 12 points/sensor/frame |
| Maximum buffer capacity | 4096 points/frame |
| Frames per load test | 12 |

---

## 3. Sensor Geometry

The simulator uses three sensors with different positions and orientations.

```text
Sensor 1: (-2.50, 0.00, 1.00), yaw = +15°
Sensor 2: ( 0.00, 0.00, 1.00), yaw =   0°
Sensor 3: ( 2.50, 0.00, 1.00), yaw = -15°
```

The important property of this model is that the sensors do **not** independently generate three different humans.

Instead:

```text
Global human point
        ↓
Global → sensor-local coordinate transform
        ↓
Sensor measurement + noise
        ↓
Sensor-local → global coordinate reconstruction
        ↓
Fusion
        ↓
DBSCAN
```

Therefore all three sensors observe the same underlying human targets in the same global coordinate system.

This corrected geometry prevents the earlier simulation error in which each sensor could form a separate cluster for the same human.

---

## 4. Human Movement Model

Two moving humans are simulated.

Each human is represented by a 3D point cloud containing simplified body regions:

- torso
- head
- left/right arms
- left/right legs

Small measurement noise is added to each observation.

The humans move continuously within a bounded simulated environment by updating their global position between frames.

The resulting workload is therefore dynamic rather than a static random point cloud.

---

## 5. Processing Pipeline

The complete simulated pipeline is:

```text
Three simulated sensors
          ↓
Sensor-local point generation
          ↓
Noise / measurement model
          ↓
Coordinate transformation
          ↓
Global point fusion
          ↓
Spatial-grid construction
          ↓
DBSCAN clustering
          ↓
Cluster geometry analysis
          ↓
Human-like cluster detection
```

The DBSCAN implementation uses a spatial grid rather than an `N × N` distance matrix.

For each point, only its own grid cell and the 26 neighboring cells are searched, followed by an exact squared-Euclidean-distance check.

---

# 6. Real-Time Criterion

The target operating frequency is:

```text
10 FPS
```

Therefore the available processing time per frame is:

```text
1000 ms / 10 FPS = 100 ms/frame
```

A load is considered **real-time** only when:

```text
maximum observed pipeline time ≤ 100 ms
```

The experiment also requires stable semantic detection:

```text
Expected humans = 2

Minimum human-like detections = 2
Maximum human-like detections = 2
```

Thus performance alone is not sufficient; the system must remain computationally real-time **and** preserve the expected two-human detection result.

---

# 7. Load Search Method

Experiment 07B searches for the maximum sustainable point load in two phases.

## Phase 1 — Coarse Search

The number of points per human per sensor is increased in larger steps.

The search reached:

```text
48
64
80
96
112
points/human/sensor
```

## Phase 2 — Fine Search

The boundary between the highest passing configuration and the first failing configuration is tested in smaller increments.

The final boundary was:

```text
104 points/human/sensor → PASS
108 points/human/sensor → FAIL
```

---

# 8. Coarse Search Results

| Points/human/sensor | Points/frame | Average pipeline | Maximum pipeline | Human detection | Real-time |
|---:|---:|---:|---:|---:|:---:|
| 48 | 324 | 27.858 ms | 29.451 ms | 2/2 | YES |
| 64 | 420 | 43.720 ms | 45.074 ms | 2/2 | YES |
| 80 | 516 | 65.853 ms | 68.175 ms | 2/2 | YES |
| 96 | 612 | 76.136 ms | 79.331 ms | 2/2 | YES |
| 112 | 708 | 104.160 ms | 109.295 ms | 2/2 | NO |

The coarse search established that the real-time boundary lies between **612 and 708 points/frame**.

---

# 9. Fine Search Results

| Points/human/sensor | Points/frame | Average pipeline | Maximum pipeline | Human detection | Real-time |
|---:|---:|---:|---:|---:|:---:|
| 100 | 636 | 83.365 ms | 85.181 ms | 2/2 | YES |
| **104** | **660** | **89.260 ms** | **91.456 ms** | **2/2** | **YES** |
| 108 | 684 | 99.360 ms | 102.145 ms | 2/2 | NO |

The first fine-search failure occurred at 108 points per human per sensor.

---

# 10. Final Verification

The highest passing candidate was tested for 12 consecutive frames.

Configuration:

```text
Points/human/sensor: 104
Points/frame:        660
Input rate @ 10 FPS: 6600 points/s
```

Observed frame times:

```text
Frame 0  → 96.496 ms
Frame 1  → 90.684 ms
Frame 2  → 92.018 ms
Frame 3  → 89.418 ms
Frame 4  → 90.493 ms
Frame 5  → 90.737 ms
Frame 6  → 88.994 ms
Frame 7  → 85.724 ms
Frame 8  → 87.061 ms
Frame 9  → 87.677 ms
Frame 10 → 86.688 ms
Frame 11 → 86.294 ms
```

The final verification produced:

```text
Average total pipeline:       89.357 ms
Maximum total pipeline:       96.496 ms
Estimated maximum FPS:        11.19
Estimated throughput:       7386.1 points/s
Frame overruns:                0 / 12
Human-like clusters:           2 / 2 stable
```

The real-time condition was therefore satisfied.

---

# 11. Pipeline Timing at the Final Passing Load

At 660 points/frame:

| Pipeline stage | Average time |
|---|---:|
| Ingest / fusion | 3.396 ms |
| Grid + DBSCAN | 84.708 ms |
| Human analysis | 1.250 ms |
| **Total** | **89.357 ms** |

The dominant component is clearly DBSCAN.

Approximately:

```text
Grid + DBSCAN
≈ 84.708 / 89.357
≈ 94.8%
```

of the average pipeline time is spent in the clustering stage.

This means that further system optimization should primarily target DBSCAN and its surrounding spatial-index operations rather than the final human-like classifier.

---

# 12. First Failing Configuration

The first failing configuration was:

```text
Points/human/sensor: 108
Points/frame:        684
Input rate @ 10 FPS: 6840 points/s
```

Measured results:

```text
Average pipeline:       98.610 ms
Maximum pipeline:      101.247 ms
Estimated maximum FPS: 10.14
Estimated throughput: 6936.4 points/s
Frame overruns:          5 / 12
```

Human detection remained correct:

```text
Average human-like clusters: 2
Minimum:                     2
Maximum:                     2
Detection:                   STABLE
```

Therefore the first failure is **computational**, not a failure of the human-detection logic.

The maximum frame time exceeds the 100-ms real-time budget by:

```text
101.247 - 100.000 = 1.247 ms
```

---

# 13. Final Measured Limit

The final measured passing configuration is:

```text
================================================
MAXIMUM PASSING REAL-TIME LOAD
================================================

Points/human/sensor:       104
Points/frame:              660
Sensors:                     3
Humans:                      2
Input rate @ 10 FPS:      6600 points/s

Average pipeline:        89.357 ms
Maximum pipeline:        96.496 ms
Frame budget:           100.000 ms

Estimated maximum FPS:    11.19
Estimated throughput:    7386.1 points/s

Frame overruns:            0 / 12
Human detection:         2 / 2 stable

STATUS:                    REAL-TIME
```

The next tested load was:

```text
684 points/frame
6840 points/s
```

and it failed the 100-ms worst-case frame budget.

Therefore, for this simulated workload:

> **The ESP32-S3 can sustain at least 660 fused 3D points per frame at 10 FPS with stable two-human detection, while 684 points/frame is the first tested configuration that violates the real-time budget.**

---

# 14. Interpretation

The results demonstrate that the ESP32-S3 is capable of running a non-trivial 3D sensing pipeline in real time at 10 FPS, but the practical limit is primarily determined by the DBSCAN workload.

At the final passing load:

```text
660 points/frame
```

is processed within approximately:

```text
89 ms average
96.5 ms worst measured frame
```

This leaves only a small worst-case margin of:

```text
100.000 - 96.496 = 3.504 ms
```

Therefore the 660-point configuration should be considered a **measured real-time operating point**, rather than a large safety-margin configuration.

A production implementation should normally target a lower load to tolerate actual UART timing variation, packet jitter, operating-system activity, sensor interruptions, and other runtime overhead.

---

# 15. Relationship to Spatial-Grid Optimization

The experiment confirms the importance of the spatial-grid optimization developed in Experiment 05.

The complete sensing pipeline contains:

```text
Sensor simulation
      ↓
Coordinate fusion
      ↓
Spatial grid
      ↓
DBSCAN
      ↓
Human analysis
```

At the final passing configuration, the overwhelming computational cost is still:

```text
Spatial grid + DBSCAN
```

Therefore the spatial-index strategy remains the primary algorithmic optimization for this embedded workload.

The Dual-Core experiments in `README_06_Dual_Core_Experiments.md` can be used as a second-stage optimization study on top of this architecture.

---

# 16. Current Experimental Limitation

This is a **UART-style simulation**, not a benchmark using the physical antennas.

The experiment currently simulates:

- sensor observations
- sensor-local coordinates
- measurement noise
- coordinate transforms
- fusion of three sensor streams
- DBSCAN processing
- human-like cluster classification

It does **not** yet measure the exact physical UART transfer cost, baud-rate limitation, hardware receive behavior, or the actual data format generated by the antennas.

When the three real sensors/antennas become available, the input layer should be replaced with the real UART streams while preserving the same downstream pipeline:

```text
UART 1 ─┐
UART 2 ─┼─→ Parse → Global Fusion → Grid → DBSCAN → Human Detection
UART 3 ─┘
```

The benchmark should then be repeated to measure the true end-to-end sensor-to-decision latency.

---

# 17. Recommended Production Margin

Because the maximum passing configuration has only approximately 3.5 ms of worst-case headroom, it should not automatically be treated as the recommended production operating point.

A practical deployment should preferably operate below the measured boundary, leaving room for:

- real UART reception overhead
- packet jitter
- occasional larger point clouds
- sensor synchronization
- FreeRTOS scheduling
- communication and logging
- future tracking/classification logic

Thus the experiment defines a **capacity boundary**, not necessarily the final deployment configuration.

---

# 18. Conclusion

Experiment 07B successfully extends the earlier DBSCAN benchmarks from an isolated algorithm to a realistic three-sensor embedded perception pipeline.

The corrected sensor geometry ensures that all three sensors observe the same two moving humans and that their measurements are fused into a common global coordinate system. The resulting DBSCAN output consistently identifies exactly two human-like clusters under both passing and failing computational loads.

The measured real-time boundary at a target of 10 FPS is:

```text
104 points/human/sensor
660 points/frame
6600 points/second
```

with:

```text
Average pipeline: 89.357 ms
Maximum pipeline: 96.496 ms
```

The first tested failing configuration is:

```text
108 points/human/sensor
684 points/frame
6840 points/second
Maximum pipeline: 101.247 ms
```

Therefore the experiment demonstrates that the ESP32-S3 can execute the simulated multi-sensor 3D human-detection pipeline in real time at 10 FPS for at least **660 fused points per frame**, while the next tested load exceeds the real-time budget.

The main computational limitation remains the **spatial-grid DBSCAN stage**, which accounts for approximately 95% of average pipeline time at the final passing load.

---

# 19. Main README Reference

Add the following entry to the main project `README.md`:

```markdown
## Experiment 07B — Real-Time Three-Sensor Pipeline

This experiment evaluates a realistic three-sensor 3D human-detection pipeline using simulated sensor observations, coordinate fusion, spatial-grid DBSCAN, and human-like cluster analysis.

At a target of 10 FPS, the corrected simulation sustains:

- 660 points/frame
- 6,600 points/s input rate
- 89.357 ms average pipeline time
- 96.496 ms maximum measured pipeline time
- stable detection of 2 simulated humans

The first tested failing configuration was 684 points/frame, where the maximum pipeline time reached 101.247 ms.

[Read Experiment 07B →](README_07_Realtime_Sensor_Pipeline.md)
```

---

## 20. Final Status

```text
Experiment 07B

Status: COMPLETE

Sensors simulated:       3
Humans simulated:         2
DBSCAN:                   Spatial Grid
Target frame rate:       10 FPS

Maximum passing load:
  660 points/frame
  6600 points/s

Human detection:
  Stable 2/2

Real-time status:
  YES at 660 points/frame
  NO at 684 points/frame

Physical UART sensors:
  Not yet connected

Next validation:
  Repeat with real UART sensor streams
```
