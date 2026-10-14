# ESP32-S3 DBSCAN — Phase 2

## Experiment 10: Raw DBSCAN vs. Kalman Before DBSCAN

**Board:** ESP32-S3 N16R8  
**CPU:** 240 MHz  
**DBSCAN:** spatial grid, ε = 0.35 m, MinPts = 5  
**Target:** 10 FPS (100 ms/frame)  
**Measurement noise:** σ = 0.05 m  
**Kalman process-noise parameter:** Q = 0.8  
**Frames per load:** 20

## Purpose

Compare two pipelines using the same generated measurements:

1. Raw measurements → spatial grid → DBSCAN.
2. Raw measurements → Kalman filtering → spatial grid → DBSCAN.

Tracked synthetic points retain a stable identity between frames. The 24 clutter points per frame are not filtered. This makes the experiment a controlled comparison; it does **not** solve data association for real, unlabeled sensor point clouds.

## Pipeline latency

![Average pipeline latency](images/experiment10_latency.png)

| Points/frame | Raw avg (ms) | Kalman avg (ms) | Added time (ms) |
|---:|---:|---:|---:|
| 96 | 2.453 | 2.641 | 0.188 |
| 168 | 6.867 | 7.243 | 0.376 |
| 240 | 13.506 | 14.074 | 0.568 |
| 312 | 22.300 | 23.061 | 0.761 |
| 384 | 33.376 | 34.332 | 0.956 |
| 456 | 46.539 | 47.698 | 1.159 |
| 552 | 67.637 | 69.078 | 1.441 |
| 624 | 85.821 | 87.461 | 1.640 |

The measured end-to-end difference includes Kalman filtering. The raw and filtered grid/DBSCAN timings are very similar, so the primary added cost is the filtering stage.

## Position accuracy

![Position RMSE comparison](images/experiment10_rmse.png)

| Points/frame | Raw RMSE (m) | Kalman RMSE (m) | RMSE reduction |
|---:|---:|---:|---:|
| 96 | 0.04983 | 0.03429 | 31.19% |
| 168 | 0.04967 | 0.03484 | 29.85% |
| 240 | 0.05008 | 0.03521 | 29.69% |
| 312 | 0.04967 | 0.03476 | 30.02% |
| 384 | 0.04990 | 0.03497 | 29.93% |
| 456 | 0.05001 | 0.03514 | 29.74% |
| 552 | 0.04985 | 0.03478 | 30.23% |
| 624 | 0.04997 | 0.03495 | 30.06% |

**Result:** the Kalman stage reduced position RMSE by about 30% for every tested load. Raw RMSE remained near 5 cm, and filtered RMSE near 3.5 cm.

## Real-time behavior

- All eight loads completed with **0/20 frame overruns** for both pipelines.
- At 624 points/frame, average raw pipeline time was 85.821 ms and average Kalman pipeline time was 87.461 ms.
- The largest tested load therefore stayed below the 100 ms frame budget in this simulation, with about 12.5 ms average headroom for the Kalman pipeline.
- This is a simulated workload result, not a confirmed limit for real UART sensors. Packet handling, calibration, real point distributions, data association, and other application tasks can increase latency.

## Clustering behavior

Both pipelines reported two large clusters at the first and last frames of each load, and the measured large-cluster range was 2..2 throughout. This is encouraging for this synthetic scene, but does not prove clustering accuracy for real sensor data or close/crossing people.

## Memory

The reported free heap after the test was 259,672 bytes, and free PSRAM was 8,385,035 bytes. These are observed free-memory readings, not a complete measurement of total memory allocated by the algorithm.

## Conclusions

1. Kalman filtering improves the synthetic tracked-point position estimate: about **30% lower RMSE**.
2. It adds a modest latency cost: approximately **0.19–1.64 ms per frame** across the tested loads.
3. The clustering outputs stayed stable at two large clusters for this scenario.
4. Both pipelines met the 10 FPS target through 624 points/frame in this test.
5. For real sensors, filtering every raw point requires point association across frames. If stable identities are unavailable, filtering cluster centroids or tracks after DBSCAN is usually a more defensible next experiment.

## Reproducibility note

These results were transcribed from the user's ESP32-S3 Serial Monitor output. They describe the current synthetic test and parameters, not an independently verified real-sensor performance guarantee.
