# ESP32-S3 DBSCAN — Phase 2 Results

**Hardware:** ESP32-S3 N16R8, dual-core 240 MHz, 8 MB PSRAM  
**Framework:** PlatformIO / Arduino  
**Frame-rate target:** 10 FPS (100 ms frame budget)

This report records Experiments 08 and 09 using the serial output supplied for the ESP32-S3. Experiment 10 (raw points versus Kalman-filtered points before DBSCAN) is planned and is **not yet measured**.

---

## Experiment 08 — Raw baseline, no Kalman

Three-sensor human simulation, spatial grid + DBSCAN, no Kalman filtering.

### Measured pipeline latency

| Points/frame | Average pipeline (ms) | Maximum pipeline (ms) | Frame overruns |
|---:|---:|---:|---:|
| 96 | 1.423 | 1.681 | 0/20 |
| 168 | 3.638 | 3.788 | 0/20 |
| 240 | 6.579 | 7.080 | 0/20 |
| 312 | 10.207 | 10.725 | 0/20 |
| 384 | 14.508 | 15.137 | 0/20 |
| 456 | 19.519 | 20.122 | 0/20 |
| 552 | 26.951 | 28.852 | 0/20 |
| 624 | 33.840 | 34.601 | 0/20 |

### Graph: average and maximum pipeline time

![Experiment 08 — Pipeline latency](images/experiment08_latency.png)

*Blue: average latency. Orange: maximum observed latency.
The target frame budget is 100 ms.*
*First bar series: raw RMSE. Second bar series: Kalman-filtered RMSE.*

*First line: average pipeline latency. Second line: maximum observed latency. The 100 ms frame deadline is the upper bound of the y-axis.*

### Interpretation

- All eight loads met the 10 FPS deadline in the 20-frame samples; no overruns were reported.
- At 624 points/frame (6,240 points/s at 10 FPS), average pipeline time was 33.840 ms and maximum observed time was 34.601 ms.
- The simulation suggests useful processing headroom, but it does not include the full cost of real sensor UART reception, packet parsing, calibration, and synchronization.
- The cluster output sometimes contained more than two human-like clusters. Treat detection quality as **not yet validated**; inspect sensor-coordinate transforms and clustering behavior before using cluster count as an accuracy metric.

---

## Experiment 09 — Kalman filter microbenchmark

A 3D constant-velocity Kalman filter was tested against the same synthetic trajectory at three measurement-noise levels. The filter was corrected after the first run because the covariance update used old rather than predicted covariance values.

### Measured accuracy

RMSE is per coordinate across the 3D trajectory, in metres. Lower is better.

| Noise σ (m) | Raw RMSE (m) | Filtered RMSE (m) | RMSE reduction |
|---:|---:|---:|---:|
| 0.025 | 0.02500 | 0.01730 | 30.79% |
| 0.050 | 0.04991 | 0.03031 | 39.28% |
| 0.100 | 0.09999 | 0.05279 | 47.21% |

### Graph: raw versus filtered RMSE

### Graph: average and maximum pipeline latency

### Graph: raw versus Kalman RMSE

![Experiment 09 — Position RMSE](images/experiment09_rmse.png)

*Blue: raw RMSE. Green: Kalman-filtered RMSE.
Lower is better.*

### Measured processing cost

| Metric | Result |
|---|---:|
| Average 3D update time | 3.449–3.473 μs |
| Maximum observed update | 11–12 μs |
| Estimated update throughput | about 288,000 updates/s |
| Free heap after test | about 372 KB |
| Free PSRAM | about 8.39 MB |

At a 10 FPS target, 660 independent point updates would nominally take around 2.3 ms at the measured average update time. This is an estimate, not a measured end-to-end pipeline result.

### Interpretation and limitations

- Filtered RMSE was lower for all three tested noise levels.
- The percentage reduction increased from 30.79% at σ = 0.025 m to 47.21% at σ = 0.100 m.
- These are synthetic trajectory results. They do not establish the same improvement for real sensor point clouds.
- Point-level filtering requires persistent point identity/data association across frames. An arbitrary point index cannot safely be assumed to represent the same physical point every frame.
- The next experiment should compare **raw points → DBSCAN** against **Kalman-filtered points → DBSCAN** using identical generated measurements and stable synthetic point identities.

---

## Experiment 10 — Planned comparison (not yet measured)

The next ESP32 run will compare two pipelines on the same generated frame data:

1. **Raw:** generate measurements → spatial grid + DBSCAN.
2. **Kalman:** generate the same measurements → per-identity Kalman filters → spatial grid + DBSCAN.

Record at least:

- average and maximum total processing time;
- DBSCAN cluster count and large-cluster count;
- raw versus filtered position RMSE against synthetic ground truth;
- frame overruns against the 100 ms deadline;
- free heap and PSRAM.

Do not fill this section with assumed results. Add the measured serial output after the ESP32 run.

---

## Current conclusion

Experiment 08 shows that the unfiltered simulated pipeline met the 10 FPS target up to 624 points/frame. Experiment 09 shows that the corrected Kalman filter reduced synthetic position RMSE by 30.79–47.21%, with an average update cost around 3.5 μs per 3D point. Experiment 10 is needed to establish whether filtering improves DBSCAN clustering and end-to-end latency under the same workload.
