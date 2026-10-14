#include <Arduino.h>
#include <math.h>
#include <string.h>

// ============================================================
// ESP32-S3 DBSCAN — PHASE 2
// Experiment 10: Raw DBSCAN vs Kalman-before-DBSCAN
//
// Both pipelines process the SAME generated measurements:
//   A) Raw points -> spatial grid -> DBSCAN
//   B) Same points -> per-identity Kalman -> spatial grid -> DBSCAN
//
// Synthetic point identities persist across frames. This is a
// controlled experiment, not yet a real-sensor data association
// implementation.
// ============================================================

static constexpr int SENSOR_COUNT = 3;
static constexpr int HUMAN_COUNT = 2;
static constexpr int MAX_POINTS_PER_HUMAN_SENSOR = 100;
static constexpr int NOISE_POINTS_PER_SENSOR = 8;

static constexpr int MAX_TRACKED =
    SENSOR_COUNT * HUMAN_COUNT * MAX_POINTS_PER_HUMAN_SENSOR;

static constexpr int MAX_POINTS =
    MAX_TRACKED + SENSOR_COUNT * NOISE_POINTS_PER_SENSOR;

static constexpr int GRID_BUCKETS = 1024;
static constexpr int MIN_PTS = 5;
static constexpr float EPSILON = 0.35f;
static constexpr float EPSILON_SQ = EPSILON * EPSILON;
static constexpr float DT = 0.1f;
static constexpr float MEASUREMENT_SIGMA = 0.05f;
static constexpr float PROCESS_Q = 0.8f;
static constexpr int FRAMES_PER_LOAD = 20;
static constexpr float FRAME_BUDGET_MS = 100.0f;
static constexpr int MIN_HUMAN_CLUSTER_POINTS = 15;

static const int LOADS[] = {12, 24, 36, 48, 60, 72, 88, 100};
static constexpr int LOAD_COUNT = sizeof(LOADS) / sizeof(LOADS[0]);

// ------------------------------------------------------------
// Point3D — must have a 3-arg constructor so brace-init
// assignment (e.g. `p = {x, y, z};`) compiles.
// ------------------------------------------------------------
struct Point3D {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Point3D() = default;
    Point3D(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

struct Vec3 {
    float x, y, z;
};

// Static arrays keep memory use predictable. MAX_POINTS is 624
// for the current loads, with headroom to 1200.
static Point3D rawPoints[MAX_POINTS];
static Point3D filteredPoints[MAX_POINTS];
static Point3D truthPoints[MAX_TRACKED];

static int labels[MAX_POINTS];
static int queueBuffer[MAX_POINTS];
static int gridHead[GRID_BUCKETS];
static int gridNext[MAX_POINTS];

static int cellX[MAX_POINTS];
static int cellY[MAX_POINTS];
static int cellZ[MAX_POINTS];

static uint32_t seenStamp[MAX_POINTS];
static uint32_t queryStamp = 0;

static constexpr int MAX_CLUSTERS = MAX_POINTS;
static constexpr float ASSOCIATION_GATE = 1.10f;
static constexpr int MAX_MISSED_FRAMES = 4;

struct ClusterCentroid {
    Point3D p;
    int size;
    int label;
};

static ClusterCentroid detections[MAX_CLUSTERS];
static Point3D rawCentroids[HUMAN_COUNT];
static Point3D kalmanCentroids[HUMAN_COUNT];
static bool rawCentroidValid[HUMAN_COUNT];
static bool trackValid[HUMAN_COUNT];
static int trackMisses[HUMAN_COUNT];
static int trackTruthId[HUMAN_COUNT];
static Point3D trackEstimates[HUMAN_COUNT];
static bool trackInitialized[HUMAN_COUNT];

// ============================================================
// Kalman filter: 1D constant-velocity model
// ============================================================

struct KalmanAxis {
    float pos = 0.0f;
    float vel = 0.0f;

    float p00 = 1.0f;
    float p01 = 0.0f;
    float p10 = 0.0f;
    float p11 = 1.0f;

    float q = PROCESS_Q;
    float r = MEASUREMENT_SIGMA * MEASUREMENT_SIGMA;
    bool initialized = false;

    void reset(float measurementVariance) {
        pos = 0.0f;
        vel = 0.0f;
        p00 = 1.0f;
        p01 = 0.0f;
        p10 = 0.0f;
        p11 = 1.0f;
        q = PROCESS_Q;
        r = measurementVariance;
        initialized = false;
    }

    float update(float measurement, float dt) {
        if (!initialized) {
            pos = measurement;
            vel = 0.0f;
            p00 = 1.0f;
            p01 = 0.0f;
            p10 = 0.0f;
            p11 = 1.0f;
            initialized = true;
            return pos;
        }

        // State prediction
        pos += vel * dt;

        const float oldP00 = p00;
        const float oldP01 = p01;
        const float oldP10 = p10;
        const float oldP11 = p11;

        const float dt2 = dt * dt;
        const float dt3 = dt2 * dt;
        const float dt4 = dt2 * dt2;

        // Covariance prediction
        const float predP00 =
            oldP00 + dt * (oldP01 + oldP10)
            + dt2 * oldP11 + q * dt4 * 0.25f;

        const float predP01 =
            oldP01 + dt * oldP11 + q * dt3 * 0.5f;

        const float predP10 =
            oldP10 + dt * oldP11 + q * dt3 * 0.5f;

        const float predP11 = oldP11 + q * dt2;

        // Measurement update
        const float innovation = measurement - pos;
        const float innovationVariance = predP00 + r;
        const float k0 = predP00 / innovationVariance;
        const float k1 = predP10 / innovationVariance;

        pos += k0 * innovation;
        vel += k1 * innovation;

        // Covariance update uses predicted covariance.
        p00 = (1.0f - k0) * predP00;
        p01 = (1.0f - k0) * predP01;
        p10 = predP10 - k1 * predP00;
        p11 = predP11 - k1 * predP01;

        return pos;
    }
};

struct Kalman3D {
    KalmanAxis fx, fy, fz;

    void reset(float measurementVariance) {
        fx.reset(measurementVariance);
        fy.reset(measurementVariance);
        fz.reset(measurementVariance);
    }

    Point3D update(const Point3D &m) {
        return Point3D(
            fx.update(m.x, DT),
            fy.update(m.y, DT),
            fz.update(m.z, DT)
        );
    }
};

static Kalman3D centroidFilters[HUMAN_COUNT];
static Kalman3D pointFilters[MAX_TRACKED];

// ============================================================
// Deterministic pseudo-random number generator
// ============================================================

static uint32_t rngState = 0xA53C91E7u;

uint32_t random32() {
    rngState ^= rngState << 13;
    rngState ^= rngState >> 17;
    rngState ^= rngState << 5;
    return rngState;
}

float uniform01() {
    return (random32() & 0x00FFFFFFu) / 16777216.0f;
}

float uniformRange(float lo, float hi) {
    return lo + (hi - lo) * uniform01();
}

// Approximate N(0,1) using the central limit theorem.
float gaussianNoise() {
    float sum = 0.0f;
    for (int i = 0; i < 12; ++i) {
        sum += uniform01();
    }
    return sum - 6.0f;
}

// Stable hash for fixed body offsets. Offsets remain the same
// for a given synthetic point identity across all frames.
uint32_t mix32(uint32_t x) {
    x ^= x >> 16;
    x *= 0x7feb352du;
    x ^= x >> 15;
    x *= 0x846ca68bu;
    x ^= x >> 16;
    return x;
}

float stableOffset(int identity, uint32_t salt) {
    const uint32_t h = mix32((uint32_t)identity * 2654435761u + salt);
    const float u = (h & 0x00FFFFFFu) / 16777216.0f;
    return (u * 2.0f - 1.0f) * 0.10f;
}

// ============================================================
// Synthetic crossing trajectories
// All points are generated in one calibrated world coordinate frame.
// Human 0 crosses horizontally; human 1 crosses vertically.
// They coincide near the middle, deliberately stressing DBSCAN and tracking.
// ============================================================

Vec3 humanPosition(int human, float t) {
    const float progress = (t < 0.0f) ? 0.0f : (t > 1.9f ? 1.9f : t);
    if (human == 0) {
        Vec3 v;
        v.x = -0.70f + 0.70f * progress;
        v.y = 0.0f;
        v.z = 0.90f;
        return v;
    }

    Vec3 v;
    v.x = 0.0f;
    v.y = -0.70f + 0.70f * progress;
    v.z = 0.95f;
    return v;
}

float distanceSquared(const Point3D &a, const Point3D &b) {
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    const float dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}

// Generate one frame once. Stable offsets simulate points distributed
// around each person, while independent Gaussian noise perturbs measurements.
int generateFrame(
    int pointsPerHumanSensor,
    int frameIndex,
    int &trackedCount,
    uint32_t &generationMicros
) {
    const uint32_t start = micros();
    const float t = frameIndex * DT;
    trackedCount = SENSOR_COUNT * HUMAN_COUNT * pointsPerHumanSensor;
    int n = 0;

    for (int s = 0; s < SENSOR_COUNT; ++s) {
        for (int h = 0; h < HUMAN_COUNT; ++h) {
            const Vec3 center = humanPosition(h, t);
            for (int k = 0; k < pointsPerHumanSensor; ++k) {
                const int identity =
                    (s * HUMAN_COUNT + h) * pointsPerHumanSensor + k;
                const Point3D truth(
                    center.x + stableOffset(identity, 0x1111u),
                    center.y + stableOffset(identity, 0x2222u),
                    center.z + stableOffset(identity, 0x3333u)
                );
                rawPoints[n++] = Point3D(
                    truth.x + MEASUREMENT_SIGMA * gaussianNoise(),
                    truth.y + MEASUREMENT_SIGMA * gaussianNoise(),
                    truth.z + MEASUREMENT_SIGMA * gaussianNoise()
                );
            }
        }
    }

    for (int s = 0; s < SENSOR_COUNT; ++s) {
        for (int j = 0; j < NOISE_POINTS_PER_SENSOR; ++j) {
            rawPoints[n++] = Point3D(
                uniformRange(-3.5f, 3.5f),
                uniformRange(-2.5f, 2.5f),
                uniformRange(0.1f, 2.0f)
            );
        }
    }

    generationMicros = micros() - start;
    return n;
}

// ============================================================
// Spatial hash grid
// ============================================================

int hashCell(int x, int y, int z) {
    const uint32_t h =
        ((uint32_t)x * 73856093u)
        ^ ((uint32_t)y * 19349663u)
        ^ ((uint32_t)z * 83492791u);

    return (int)(h % GRID_BUCKETS);
}

void buildGrid(const Point3D *points, int n) {
    for (int i = 0; i < GRID_BUCKETS; ++i) {
        gridHead[i] = -1;
    }

    for (int i = 0; i < n; ++i) {
        const int cx = (int)floorf(points[i].x / EPSILON);
        const int cy = (int)floorf(points[i].y / EPSILON);
        const int cz = (int)floorf(points[i].z / EPSILON);

        cellX[i] = cx;
        cellY[i] = cy;
        cellZ[i] = cz;

        const int bucket = hashCell(cx, cy, cz);
        gridNext[i] = gridHead[bucket];
        gridHead[bucket] = i;
    }
}

// Finds neighbors in the 27 adjacent cells. seenStamp prevents
// duplicate candidates when different cells hash to one bucket.
int regionQuery(
    const Point3D *points,
    int n,
    int index,
    int *output
) {
    ++queryStamp;
    if (queryStamp == 0) {
        memset(seenStamp, 0, sizeof(seenStamp));
        queryStamp = 1;
    }

    const int cx = cellX[index];
    const int cy = cellY[index];
    const int cz = cellZ[index];

    int count = 0;

    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                const int tx = cx + dx;
                const int ty = cy + dy;
                const int tz = cz + dz;

                const int bucket = hashCell(tx, ty, tz);

                for (int j = gridHead[bucket]; j != -1; j = gridNext[j]) {
                    if (j < 0 || j >= n) {
                        continue;
                    }

                    // Reject hash collisions from other actual cells.
                    if (cellX[j] != tx ||
                        cellY[j] != ty ||
                        cellZ[j] != tz) {
                        continue;
                    }

                    if (seenStamp[j] == queryStamp) {
                        continue;
                    }
                    seenStamp[j] = queryStamp;

                    if (distanceSquared(points[index], points[j]) <= EPSILON_SQ) {
                        if (output != nullptr) {
                            output[count] = j;
                        }
                        ++count;
                    }
                }
            }
        }
    }

    return count;
}

// ============================================================
// DBSCAN using the spatial grid
// Labels: -2 = unvisited, -1 = noise, >= 0 = cluster ID
// ============================================================

int runDBSCAN(const Point3D *points, int n) {
    for (int i = 0; i < n; ++i) {
        labels[i] = -2;
    }

    int clusterId = 0;

    for (int i = 0; i < n; ++i) {
        if (labels[i] != -2) {
            continue;
        }

        int neighborCount = regionQuery(points, n, i, queueBuffer);

        if (neighborCount < MIN_PTS) {
            labels[i] = -1;
            continue;
        }

        labels[i] = clusterId;

        int head = 0;
        int tail = 0;

        for (int k = 0; k < neighborCount; ++k) {
            const int j = queueBuffer[k];

            if (j == i) {
                continue;
            }

            if (labels[j] == -2 || labels[j] == -1) {
                labels[j] = clusterId;
                queueBuffer[tail++] = j;
            }
        }

        while (head < tail) {
            const int q = queueBuffer[head++];

            // Retrieve expansion neighbors into a temporary region.
            // queueBuffer cannot serve as both output and expansion
            // queue, so use a fixed-size scratch array.
            static int expansion[MAX_POINTS];
            const int expansionCount =
                regionQuery(points, n, q, expansion);

            if (expansionCount < MIN_PTS) {
                continue;
            }

            for (int k = 0; k < expansionCount; ++k) {
                const int j = expansion[k];

                if (labels[j] == -2 || labels[j] == -1) {
                    labels[j] = clusterId;

                    if (tail < MAX_POINTS) {
                        queueBuffer[tail++] = j;
                    }
                }
            }
        }

        ++clusterId;
    }

    return clusterId;
}

struct ClusterSummary {
    int clusters;
    int largeClusters;
    int noisePoints;
};

ClusterSummary summarizeLabels(int n, int clusterCount) {
    static int clusterSizes[MAX_POINTS];
    for (int c = 0; c < clusterCount; ++c) {
        clusterSizes[c] = 0;
    }

    int noise = 0;

    for (int i = 0; i < n; ++i) {
        if (labels[i] >= 0 && labels[i] < clusterCount) {
            ++clusterSizes[labels[i]];
        } else if (labels[i] == -1) {
            ++noise;
        }
    }

    int large = 0;
    for (int c = 0; c < clusterCount; ++c) {
        if (clusterSizes[c] >= MIN_HUMAN_CLUSTER_POINTS) {
            ++large;
        }
    }

    ClusterSummary s;
    s.clusters = clusterCount;
    s.largeClusters = large;
    s.noisePoints = noise;
    return s;
}


// ============================================================
// Cluster extraction and two-target centroid tracker
// ============================================================

int extractHumanCentroids(int n, int clusterCount) {
    static int sizes[MAX_POINTS];
    static double sumX[MAX_POINTS], sumY[MAX_POINTS], sumZ[MAX_POINTS];

    for (int c = 0; c < clusterCount; ++c) {
        sizes[c] = 0;
        sumX[c] = sumY[c] = sumZ[c] = 0.0;
    }

    for (int i = 0; i < n; ++i) {
        const int c = labels[i];
        if (c >= 0 && c < clusterCount) {
            ++sizes[c];
            sumX[c] += rawPoints[i].x;
            sumY[c] += rawPoints[i].y;
            sumZ[c] += rawPoints[i].z;
        }
    }

    int count = 0;
    for (int c = 0; c < clusterCount; ++c) {
        if (sizes[c] < MIN_HUMAN_CLUSTER_POINTS) continue;
        detections[count].p = Point3D(
            (float)(sumX[c] / sizes[c]),
            (float)(sumY[c] / sizes[c]),
            (float)(sumZ[c] / sizes[c])
        );
        detections[count].size = sizes[c];
        detections[count].label = c;
        ++count;
    }

    // Keep only the two largest clusters as human candidates.
    for (int i = 0; i < count; ++i) {
        for (int j = i + 1; j < count; ++j) {
            if (detections[j].size > detections[i].size) {
                const ClusterCentroid tmp = detections[i];
                detections[i] = detections[j];
                detections[j] = tmp;
            }
        }
    }
    if (count > HUMAN_COUNT) count = HUMAN_COUNT;
    return count;
}

Point3D truthHuman(int human, int frameIndex) {
    const Vec3 p = humanPosition(human, frameIndex * DT);
    return Point3D(p.x, p.y, p.z);
}

void resetTracker() {
    for (int i = 0; i < HUMAN_COUNT; ++i) {
        centroidFilters[i].reset(MEASUREMENT_SIGMA * MEASUREMENT_SIGMA);
        trackValid[i] = false;
        trackMisses[i] = 0;
        trackTruthId[i] = -1;
        trackInitialized[i] = false;
        trackEstimates[i] = Point3D(0, 0, 0);
        rawCentroidValid[i] = false;
        rawCentroids[i] = Point3D(0, 0, 0);
        kalmanCentroids[i] = Point3D(0, 0, 0);
    }
}

float dist(const Point3D &a, const Point3D &b) {
    return sqrtf(distanceSquared(a, b));
}

// The Kalman state advances even when no cluster is assigned.
void predictAxisOnly(KalmanAxis &f) {
    if (!f.initialized) return;
    const float oldP00 = f.p00, oldP01 = f.p01;
    const float oldP10 = f.p10, oldP11 = f.p11;
    const float dt2 = DT * DT, dt3 = dt2 * DT, dt4 = dt2 * dt2;
    f.pos += f.vel * DT;
    const float p00 = oldP00 + DT * (oldP01 + oldP10)
                    + dt2 * oldP11 + f.q * dt4 * 0.25f;
    const float p01 = oldP01 + DT * oldP11 + f.q * dt3 * 0.5f;
    const float p10 = oldP10 + DT * oldP11 + f.q * dt3 * 0.5f;
    const float p11 = oldP11 + f.q * dt2;
    f.p00=p00; f.p01=p01; f.p10=p10; f.p11=p11;
}

void predictTrackOnly(int track) {
    predictAxisOnly(centroidFilters[track].fx);
    predictAxisOnly(centroidFilters[track].fy);
    predictAxisOnly(centroidFilters[track].fz);
    trackEstimates[track] = Point3D(
        centroidFilters[track].fx.pos,
        centroidFilters[track].fy.pos,
        centroidFilters[track].fz.pos
    );
}

// For two tracks/detections, test all assignment permutations and choose
// the lowest total prediction distance. A gate prevents implausible matches.
void associateAndFilter(int detectionCount, int frameIndex,
                        int &missedTracks, int &identitySwitches) {
    int assignedDetection[HUMAN_COUNT] = {-1, -1};
    bool detectionUsed[HUMAN_COUNT] = {false, false};

    // First frame: deterministic left-to-right initialization. Track IDs
    // are arbitrary; this establishes the evaluation identity mapping.
    if (frameIndex == 0) {
        if (detectionCount == 2) {
            if (detections[0].p.x > detections[1].p.x) {
                const ClusterCentroid tmp = detections[0];
                detections[0] = detections[1];
                detections[1] = tmp;
            }
            assignedDetection[0] = 0;
            assignedDetection[1] = 1;
        } else if (detectionCount == 1) {
            assignedDetection[0] = 0;
        }
        for (int t = 0; t < HUMAN_COUNT; ++t) {
            if (assignedDetection[t] >= 0) {
                const Point3D m = detections[assignedDetection[t]].p;
                const Point3D f = centroidFilters[t].update(m);
                trackEstimates[t] = f;
                trackValid[t] = true;
                trackInitialized[t] = true;
                trackMisses[t] = 0;
                // Ground-truth mapping is only used to score synthetic output.
                const Point3D gt0 = truthHuman(0, frameIndex);
                const Point3D gt1 = truthHuman(1, frameIndex);
                trackTruthId[t] =
                    dist(m, gt0) <= dist(m, gt1) ? 0 : 1;
                detectionUsed[assignedDetection[t]] = true;
            } else {
                trackValid[t] = false;
                trackMisses[t] = 1;
            }
        }
        return;
    }

    // Predict before association.
    Point3D predicted[HUMAN_COUNT];
    for (int t = 0; t < HUMAN_COUNT; ++t) {
        predicted[t] = Point3D(
            centroidFilters[t].fx.pos + centroidFilters[t].fx.vel * DT,
            centroidFilters[t].fy.pos + centroidFilters[t].fy.vel * DT,
            centroidFilters[t].fz.pos + centroidFilters[t].fz.vel * DT
        );
    }

    if (detectionCount == 2) {
        const float c00 = dist(predicted[0], detections[0].p);
        const float c01 = dist(predicted[0], detections[1].p);
        const float c10 = dist(predicted[1], detections[0].p);
        const float c11 = dist(predicted[1], detections[1].p);
        const float direct = c00 + c11;
        const float crossed = c01 + c10;
        if (direct <= crossed) {
            if (c00 <= ASSOCIATION_GATE) assignedDetection[0] = 0;
            if (c11 <= ASSOCIATION_GATE) assignedDetection[1] = 1;
        } else {
            if (c01 <= ASSOCIATION_GATE) assignedDetection[0] = 1;
            if (c10 <= ASSOCIATION_GATE) assignedDetection[1] = 0;
        }
    } else if (detectionCount == 1) {
        const float c0 = dist(predicted[0], detections[0].p);
        const float c1 = dist(predicted[1], detections[0].p);
        const int best = c0 <= c1 ? 0 : 1;
        if (min(c0, c1) <= ASSOCIATION_GATE) {
            assignedDetection[best] = 0;
        }
    }

    for (int t = 0; t < HUMAN_COUNT; ++t) {
        const int di = assignedDetection[t];
        if (di >= 0 && !detectionUsed[di]) {
            const Point3D m = detections[di].p;
            const Point3D f = centroidFilters[t].update(m);
            trackEstimates[t] = f;
            trackValid[t] = true;
            trackMisses[t] = 0;
            detectionUsed[di] = true;

            // Count a switch only if a track's nearest ground-truth identity
            // changes away from its initialization identity. This is a
            // synthetic diagnostic, not information used for association.
            if (trackTruthId[t] >= 0) {
                const Point3D gt0 = truthHuman(0, frameIndex);
                const Point3D gt1 = truthHuman(1, frameIndex);
                const int nearest = dist(f, gt0) <= dist(f, gt1) ? 0 : 1;
                if (nearest != trackTruthId[t]) ++identitySwitches;
            }
        } else {
            predictTrackOnly(t);
            ++trackMisses[t];
            ++missedTracks;
            if (trackMisses[t] > MAX_MISSED_FRAMES) {
                trackValid[t] = false;
            }
        }
    }
}

// Match raw centroids to ground truth for per-frame centroid RMSE.
// The best permutation is used when two detections are present.
void accumulateRawCentroidError(int detectionCount, int frameIndex,
                                double &squaredError, uint32_t &samples,
                                int &missedDetections) {
    const Point3D gt0 = truthHuman(0, frameIndex);
    const Point3D gt1 = truthHuman(1, frameIndex);
    if (detectionCount == 2) {
        const float direct = distanceSquared(detections[0].p, gt0)
                           + distanceSquared(detections[1].p, gt1);
        const float crossed = distanceSquared(detections[0].p, gt1)
                            + distanceSquared(detections[1].p, gt0);
        if (direct <= crossed) {
            squaredError += distanceSquared(detections[0].p, gt0);
            squaredError += distanceSquared(detections[1].p, gt1);
        } else {
            squaredError += distanceSquared(detections[0].p, gt1);
            squaredError += distanceSquared(detections[1].p, gt0);
        }
        samples += 2;
    } else if (detectionCount == 1) {
        const float e0 = distanceSquared(detections[0].p, gt0);
        const float e1 = distanceSquared(detections[0].p, gt1);
        squaredError += (e0 <= e1) ? e0 : e1;
        ++samples;
        ++missedDetections;
    } else {
        missedDetections += HUMAN_COUNT;
    }
}

void accumulateTrackError(int frameIndex, double &squaredError,
                          uint32_t &samples, int &missedTrackSamples) {
    for (int t = 0; t < HUMAN_COUNT; ++t) {
        if (!trackInitialized[t] || trackTruthId[t] < 0) {
            ++missedTrackSamples;
            continue;
        }
        const Point3D truth = truthHuman(trackTruthId[t], frameIndex);
        squaredError += distanceSquared(trackEstimates[t], truth);
        samples += 1;
    }
}

void runLoad(int pointsPerHumanSensor) {
    resetTracker();
    double rawDbscanMs = 0.0, rawCentroidMs = 0.0, rawTotalMs = 0.0;
    double kalmanMs = 0.0, kalmanTotalMs = 0.0;
    double rawSquaredError = 0.0, trackSquaredError = 0.0;
    uint32_t rawSamples = 0, trackSamples = 0;
    int missedDetections = 0, missedTrackSamples = 0;
    int missedTracks = 0, identitySwitches = 0;
    int minClusters = 999, maxClusters = 0;
    int maxRawUs = 0, maxKalmanUs = 0, rawOverruns = 0, kalmanOverruns = 0;
    int lastClusters = 0, lastDetections = 0;
    int expectedPoints = 0, expectedTracked = 0;

    Serial.println();
    Serial.println("================================================");
    Serial.printf("LOAD: %d points/human/sensor\n", pointsPerHumanSensor);
    Serial.println("================================================");

    for (int frame = 0; frame < FRAMES_PER_LOAD; ++frame) {
        int trackedCount = 0;
        uint32_t generationUs = 0;
        const int n = generateFrame(pointsPerHumanSensor, frame,
                                    trackedCount, generationUs);
        expectedPoints = n;
        expectedTracked = trackedCount;

        // Run DBSCAN once. Both comparison branches consume the same
        // detections, so this isolates centroid smoothing/tracking overhead.
        const uint32_t pipelineStart = micros();
        buildGrid(rawPoints, n);
        const int clusterCount = runDBSCAN(rawPoints, n);
        const uint32_t dbscanUs = micros() - pipelineStart;
        rawDbscanMs += dbscanUs / 1000.0;

        const uint32_t centroidStart = micros();
        const int detectionCount = extractHumanCentroids(n, clusterCount);
        const uint32_t centroidUs = micros() - centroidStart;
        rawCentroidMs += centroidUs / 1000.0;

        // Raw baseline: current cluster centroids, no temporal filter.
        const uint32_t rawTotalUs = generationUs + dbscanUs + centroidUs;
        rawTotalMs += rawTotalUs / 1000.0;
        if ((int)rawTotalUs > maxRawUs) maxRawUs = rawTotalUs;
        if (rawTotalUs > FRAME_BUDGET_MS * 1000.0f) ++rawOverruns;

        if (frame > 0) {
            accumulateRawCentroidError(detectionCount, frame, rawSquaredError,
                                       rawSamples, missedDetections);
        }

        if (detectionCount < minClusters) minClusters = detectionCount;
        if (detectionCount > maxClusters) maxClusters = detectionCount;
        lastClusters = clusterCount;
        lastDetections = detectionCount;

        // Kalman branch: association + centroid Kalman updates.
        const uint32_t kalmanStart = micros();
        associateAndFilter(detectionCount, frame, missedTracks, identitySwitches);
        const uint32_t kUs = micros() - kalmanStart;
        kalmanMs += kUs / 1000.0;

        const uint32_t kalmanTotalUs = generationUs + dbscanUs + centroidUs + kUs;
        kalmanTotalMs += kalmanTotalUs / 1000.0;
        if ((int)kalmanTotalUs > maxKalmanUs) maxKalmanUs = kalmanTotalUs;
        if (kalmanTotalUs > FRAME_BUDGET_MS * 1000.0f) ++kalmanOverruns;

        // Ignore first frame for RMSE to avoid initialization bias.
        if (frame > 0) {
            accumulateTrackError(frame, trackSquaredError, trackSamples,
                                 missedTrackSamples);
        }

        if (frame == 0 || frame == FRAMES_PER_LOAD - 1 ||
            (frame >= 8 && frame <= 12)) {
            Serial.printf("Frame %02d: DBSCAN clusters=%d human detections=%d\n",
                          frame, clusterCount, detectionCount);
        }
    }

    const float frames = (float)FRAMES_PER_LOAD;
    const float rawRmse = rawSamples
        ? sqrtf((float)(rawSquaredError / rawSamples)) : 0.0f;
    const float trackRmse = trackSamples
        ? sqrtf((float)(trackSquaredError / trackSamples)) : 0.0f;

    Serial.println();
    Serial.println("RESULTS");
    Serial.printf("Points/frame:                  %d\n", expectedPoints);
    Serial.printf("Tracked synthetic points:      %d\n", expectedTracked);
    Serial.printf("Noise points/frame:             %d\n", expectedPoints - expectedTracked);
    Serial.printf("Average DBSCAN + centroid:      %.3f ms\n",
                  rawDbscanMs / frames + rawCentroidMs / frames);
    Serial.printf("Average raw total pipeline:     %.3f ms\n", rawTotalMs / frames);
    Serial.printf("Maximum raw total pipeline:     %.3f ms\n", maxRawUs / 1000.0f);
    Serial.printf("Raw detected human count range: %d..%d\n",
                  minClusters, maxClusters);
    Serial.printf("Last frame DBSCAN/detections:   %d / %d\n",
                  lastClusters, lastDetections);
    Serial.printf("Raw frame overruns:             %d/%d\n", rawOverruns, FRAMES_PER_LOAD);
    Serial.println();
    Serial.println("DBSCAN -> CENTROID -> KALMAN TRACKING");
    Serial.printf("Average association + Kalman:   %.3f ms\n", kalmanMs / frames);
    Serial.printf("Average tracked total pipeline: %.3f ms\n", kalmanTotalMs / frames);
    Serial.printf("Maximum tracked total pipeline: %.3f ms\n", maxKalmanUs / 1000.0f);
    Serial.printf("Missed track assignments:       %d\n", missedTracks);
    Serial.printf("Identity-switch diagnostic:     %d\n", identitySwitches);
    Serial.printf("Track RMSE (m):                 %.5f\n", trackRmse);
    Serial.printf("Missed track RMSE samples:      %d\n", missedTrackSamples);
    Serial.printf("Tracked frame overruns:         %d/%d\n", kalmanOverruns, FRAMES_PER_LOAD);
    Serial.println();
    Serial.println("RAW CENTROID BASELINE");
    Serial.printf("Raw centroid RMSE (m):          %.5f\n", rawRmse);
    Serial.printf("Frames with missing detections: %d\n", missedDetections);
    Serial.printf("Free heap:                      %u bytes\n", (unsigned int)ESP.getFreeHeap());
    Serial.printf("Free PSRAM:                     %u bytes\n", (unsigned int)ESP.getFreePsram());
    Serial.printf("Frame budget:                   %.1f ms\n", FRAME_BUDGET_MS);
    Serial.println("------------------------------------------------");
}

void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println();
    Serial.println("================================================");
    Serial.println("ESP32-S3 DBSCAN — PHASE 2");
    Serial.println("Experiment 11: DBSCAN -> Centroids -> Kalman Tracking");
    Serial.println("Synthetic crossing trajectories; 2 target tracks");
    Serial.println("================================================");
    Serial.printf("CPU: %u MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("Free heap: %u bytes\n", (unsigned int)ESP.getFreeHeap());
    Serial.printf("Free PSRAM: %u bytes\n", (unsigned int)ESP.getFreePsram());
    Serial.printf("Epsilon: %.2f\n", EPSILON);
    Serial.printf("MinPts: %d\n", MIN_PTS);
    Serial.printf("Measurement sigma: %.3f m\n", MEASUREMENT_SIGMA);
    Serial.printf("Process Q: %.3f\n", PROCESS_Q);
    Serial.printf("Target FPS: %.1f\n", 1.0f / DT);
    Serial.printf("Frame budget: %.1f ms\n", FRAME_BUDGET_MS);
    Serial.printf("Frames/load: %d\n", FRAMES_PER_LOAD);
    for (int i = 0; i < LOAD_COUNT; ++i) runLoad(LOADS[i]);
    Serial.println();
    Serial.println("Experiment 11 completed. Send the complete Serial Monitor output.");
}

void loop() {
    delay(1000);
}