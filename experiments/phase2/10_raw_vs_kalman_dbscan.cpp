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

struct Point3D {
    float x, y, z;
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
        return {
            fx.update(m.x, DT),
            fy.update(m.y, DT),
            fz.update(m.z, DT)
        };
    }
};

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
// Simulated human ground truth
// All coordinates are already in a shared calibrated world frame.
// Sensor-specific observations have independent measurement noise.
// ============================================================

Vec3 humanPosition(int human, float t) {
    if (human == 0) {
        return {
            -0.8f + 0.7f * sinf(t * 0.55f),
            1.2f + 0.25f * sinf(t * 0.8f),
            0.90f + 0.12f * sinf(t * 0.3f)
        };
    }

    return {
        2.0f - 0.6f * sinf(t * 0.45f),
        -0.8f + 0.30f * cosf(t * 0.7f),
        0.95f + 0.10f * cosf(t * 0.35f)
    };
}

float distanceSquared(const Point3D &a, const Point3D &b) {
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    const float dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}

// ============================================================
// Generate one frame exactly once.
// Tracked points appear first, with stable identity ordering.
// Random clutter is appended and is NOT Kalman filtered.
// ============================================================

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

    // Generate stable-identity points from all sensors/humans.
    // Each sensor is assumed calibrated into world coordinates.
    for (int s = 0; s < SENSOR_COUNT; ++s) {
        for (int h = 0; h < HUMAN_COUNT; ++h) {
            const Vec3 center = humanPosition(h, t);

            for (int k = 0; k < pointsPerHumanSensor; ++k) {
                const int identity =
                    (s * HUMAN_COUNT + h) * pointsPerHumanSensor + k;

                const Point3D truth = {
                    center.x + stableOffset(identity, 0x1111u),
                    center.y + stableOffset(identity, 0x2222u),
                    center.z + stableOffset(identity, 0x3333u)
                };

                truthPoints[identity] = truth;

                rawPoints[n] = {
                    truth.x + MEASUREMENT_SIGMA * gaussianNoise(),
                    truth.y + MEASUREMENT_SIGMA * gaussianNoise(),
                    truth.z + MEASUREMENT_SIGMA * gaussianNoise()
                };

                ++n;
            }
        }
    }

    // Add untracked clutter points. Keep these identical in both
    // pipelines; do not pass them through point-identity filters.
    for (int s = 0; s < SENSOR_COUNT; ++s) {
        for (int j = 0; j < NOISE_POINTS_PER_SENSOR; ++j) {
            rawPoints[n++] = {
                uniformRange(-3.5f, 3.5f),
                uniformRange(-2.5f, 2.5f),
                uniformRange(0.1f, 2.0f)
            };
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

    return {clusterCount, large, noise};
}

// ============================================================
// Reset filters for each load. Each identity has its own filter.
// ============================================================

void resetFilters(int trackedCount) {
    const float r = MEASUREMENT_SIGMA * MEASUREMENT_SIGMA;

    for (int i = 0; i < trackedCount; ++i) {
        pointFilters[i].reset(r);
    }
}

// Filter tracked points by persistent synthetic identity.
// Clutter points are copied unchanged.
uint32_t makeFilteredFrame(
    int n,
    int trackedCount
) {
    const uint32_t start = micros();

    for (int i = 0; i < trackedCount; ++i) {
        filteredPoints[i] = pointFilters[i].update(rawPoints[i]);
    }

    for (int i = trackedCount; i < n; ++i) {
        filteredPoints[i] = rawPoints[i];
    }

    return micros() - start;
}

void calculateRMSE(
    int trackedCount,
    double &rawSquaredError,
    double &filteredSquaredError,
    int frameIndex
) {
    // Only evaluate after the first frame to reduce initialization bias.
    if (frameIndex == 0) {
        return;
    }

    for (int i = 0; i < trackedCount; ++i) {
        const Point3D &truth = truthPoints[i];
        const Point3D &raw = rawPoints[i];
        const Point3D &filtered = filteredPoints[i];

        const float rdx = raw.x - truth.x;
        const float rdy = raw.y - truth.y;
        const float rdz = raw.z - truth.z;

        const float fdx = filtered.x - truth.x;
        const float fdy = filtered.y - truth.y;
        const float fdz = filtered.z - truth.z;

        rawSquaredError +=
            (double)rdx * rdx + (double)rdy * rdy + (double)rdz * rdz;

        filteredSquaredError +=
            (double)fdx * fdx + (double)fdy * fdy + (double)fdz * fdz;
    }
}

// ============================================================
// Run one load, comparing both pipelines on identical frames
// ============================================================

void runLoad(int pointsPerHumanSensor) {
    resetFilters(
        SENSOR_COUNT * HUMAN_COUNT * pointsPerHumanSensor
    );

    double rawGenMs = 0.0;
    double rawGridMs = 0.0;
    double rawDbscanMs = 0.0;
    double rawTotalMs = 0.0;

    double kalmanUs = 0.0;
    double filteredGridMs = 0.0;
    double filteredDbscanMs = 0.0;
    double filteredTotalMs = 0.0;

    double rawSquaredError = 0.0;
    double filteredSquaredError = 0.0;
    uint64_t rmsePointSamples = 0;

    int minRawLarge = 100000;
    int maxRawLarge = 0;
    int minFilteredLarge = 100000;
    int maxFilteredLarge = 0;

    int lastRawClusters = 0;
    int lastRawLarge = 0;
    int lastFilteredClusters = 0;
    int lastFilteredLarge = 0;

    uint32_t maxRawTotalUs = 0;
    uint32_t maxFilteredTotalUs = 0;
    int overrunsRaw = 0;
    int overrunsFiltered = 0;

    int expectedTracked = 0;
    int expectedPoints = 0;

    Serial.println();
    Serial.println("================================================");
    Serial.printf(
        "LOAD: %d points/human/sensor\n",
        pointsPerHumanSensor
    );
    Serial.println("================================================");

    for (int frame = 0; frame < FRAMES_PER_LOAD; ++frame) {
        int trackedCount = 0;
        uint32_t generationUs = 0;

        const int n = generateFrame(
            pointsPerHumanSensor,
            frame,
            trackedCount,
            generationUs
        );

        expectedTracked = trackedCount;
        expectedPoints = n;
        rawGenMs += generationUs / 1000.0;

        // Pipeline A: raw points -> grid -> DBSCAN.
        uint32_t startRaw = micros();

        uint32_t start = micros();
        buildGrid(rawPoints, n);
        const uint32_t rawGridUs = micros() - start;

        start = micros();
        const int rawClusters = runDBSCAN(rawPoints, n);
        const uint32_t rawDbscanUs = micros() - start;

        const uint32_t rawTotalUs = micros() - startRaw;

        const ClusterSummary rawSummary =
            summarizeLabels(n, rawClusters);

        rawGridMs += rawGridUs / 1000.0;
        rawDbscanMs += rawDbscanUs / 1000.0;
        rawTotalMs += (generationUs + rawTotalUs) / 1000.0;

        if (rawSummary.largeClusters < minRawLarge) {
            minRawLarge = rawSummary.largeClusters;
        }
        if (rawSummary.largeClusters > maxRawLarge) {
            maxRawLarge = rawSummary.largeClusters;
        }

        const uint32_t rawFullUs = generationUs + rawTotalUs;

        if (rawFullUs > maxRawTotalUs) {
            maxRawTotalUs = rawFullUs;
        }

        if (rawFullUs > FRAME_BUDGET_MS * 1000.0f) {
            ++overrunsRaw;
        }

        lastRawClusters = rawSummary.clusters;
        lastRawLarge = rawSummary.largeClusters;

        // Generate Kalman estimates from the exact same measurements.
        const uint32_t filterUs =
            makeFilteredFrame(n, trackedCount);

        kalmanUs += filterUs;

        // Pipeline B: filtered points -> grid -> DBSCAN.
        uint32_t startFiltered = micros();

        start = micros();
        buildGrid(filteredPoints, n);
        const uint32_t filteredGridUs = micros() - start;

        start = micros();
        const int filteredClusters =
            runDBSCAN(filteredPoints, n);
        const uint32_t filteredDbscanUs = micros() - start;

        const uint32_t filteredTotalUs =
            micros() - startFiltered;

        const ClusterSummary filteredSummary =
            summarizeLabels(n, filteredClusters);

        filteredGridMs += filteredGridUs / 1000.0;
        filteredDbscanMs += filteredDbscanUs / 1000.0;

        // Includes generation, filtering, and filtered DBSCAN pipeline.
        filteredTotalMs +=
            (generationUs + filterUs + filteredTotalUs) / 1000.0;

        if (filteredSummary.largeClusters < minFilteredLarge) {
            minFilteredLarge = filteredSummary.largeClusters;
        }
        if (filteredSummary.largeClusters > maxFilteredLarge) {
            maxFilteredLarge = filteredSummary.largeClusters;
        }

        if (filteredTotalUs + filterUs + generationUs > maxFilteredTotalUs) {
            maxFilteredTotalUs = filteredTotalUs + filterUs + generationUs;
        }

        if ((filteredTotalUs + filterUs + generationUs) >
            FRAME_BUDGET_MS * 1000.0f) {
            ++overrunsFiltered;
        }

        lastFilteredClusters = filteredSummary.clusters;
        lastFilteredLarge = filteredSummary.largeClusters;

        calculateRMSE(
            trackedCount,
            rawSquaredError,
            filteredSquaredError,
            frame
        );

        if (frame > 0) {
            rmsePointSamples += trackedCount;
        }

        if (frame == 0 || frame == FRAMES_PER_LOAD - 1) {
            Serial.printf(
                "Frame %d: points=%d | raw clusters=%d large=%d "
                "| Kalman clusters=%d large=%d\n",
                frame,
                n,
                rawSummary.clusters,
                rawSummary.largeClusters,
                filteredSummary.clusters,
                filteredSummary.largeClusters
            );
        }
    }

    const float frames = (float)FRAMES_PER_LOAD;

    const float avgRawTotalMs = rawTotalMs / frames;
    const float avgFilteredTotalMs = filteredTotalMs / frames;

    const double denom = 3.0 * (double)rmsePointSamples;

    const float rawRMSE =
        denom > 0.0 ? sqrtf((float)(rawSquaredError / denom)) : 0.0f;

    const float filteredRMSE =
        denom > 0.0 ? sqrtf((float)(filteredSquaredError / denom)) : 0.0f;

    const float rmseReduction =
        rawRMSE > 0.0f
        ? (1.0f - filteredRMSE / rawRMSE) * 100.0f
        : 0.0f;

    Serial.println();
    Serial.println("RESULTS");
    Serial.printf("Points/frame:                  %d\n", expectedPoints);
    Serial.printf("Tracked points/frame:          %d\n", expectedTracked);
    Serial.printf("Noise points/frame:             %d\n",
                  expectedPoints - expectedTracked);

    Serial.println();
    Serial.println("RAW -> GRID -> DBSCAN");
    Serial.printf("Avg generation:                 %.3f ms\n",
                  rawGenMs / frames);
    Serial.printf("Avg grid build:                 %.3f ms\n",
                  rawGridMs / frames);
    Serial.printf("Avg DBSCAN:                     %.3f ms\n",
                  rawDbscanMs / frames);
    Serial.printf("Avg total pipeline:             %.3f ms\n",
                  avgRawTotalMs);
    Serial.printf("Max total pipeline:             %.3f ms\n",
                  maxRawTotalUs / 1000.0f);
    Serial.printf("Large-cluster range:            %d..%d\n",
                  minRawLarge, maxRawLarge);
    Serial.printf("Last frame clusters/large:      %d / %d\n",
                  lastRawClusters, lastRawLarge);
    Serial.printf("Frame overruns:                 %d/%d\n",
                  overrunsRaw, FRAMES_PER_LOAD);

    Serial.println();
    Serial.println("KALMAN -> GRID -> DBSCAN");
    Serial.printf("Avg Kalman filtering:           %.3f ms\n",
                  (kalmanUs / frames) / 1000.0);
    Serial.printf("Avg grid build:                 %.3f ms\n",
                  filteredGridMs / frames);
    Serial.printf("Avg DBSCAN:                     %.3f ms\n",
                  filteredDbscanMs / frames);
    Serial.printf("Avg total pipeline:             %.3f ms\n",
                  avgFilteredTotalMs);
    Serial.printf("Max total pipeline:             %.3f ms\n",
                  maxFilteredTotalUs / 1000.0f);
    Serial.printf("Large-cluster range:            %d..%d\n",
                  minFilteredLarge, maxFilteredLarge);
    Serial.printf("Last frame clusters/large:      %d / %d\n",
                  lastFilteredClusters, lastFilteredLarge);
    Serial.printf("Frame overruns:                 %d/%d\n",
                  overrunsFiltered, FRAMES_PER_LOAD);

    Serial.println();
    Serial.println("POSITION ACCURACY (tracked points only)");
    Serial.printf("Raw position RMSE:              %.5f m\n", rawRMSE);
    Serial.printf("Kalman position RMSE:           %.5f m\n", filteredRMSE);
    Serial.printf("RMSE reduction:                 %.2f %%\n", rmseReduction);

    Serial.println();
    Serial.printf("Free heap:                      %u bytes\n",
                  (unsigned int)ESP.getFreeHeap());
    Serial.printf("Free PSRAM:                     %u bytes\n",
                  (unsigned int)ESP.getFreePsram());
    Serial.printf("Frame budget:                   %.1f ms\n",
                  FRAME_BUDGET_MS);
    Serial.println("------------------------------------------------");
}

void setup() {
    Serial.begin(115200);
    delay(1500);

    Serial.println();
    Serial.println("================================================");
    Serial.println("ESP32-S3 DBSCAN — PHASE 2");
    Serial.println("Experiment 10: Raw vs Kalman Before DBSCAN");
    Serial.println("Same measurements, persistent synthetic identities");
    Serial.println("================================================");

    Serial.printf("CPU: %u MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("Free heap: %u bytes\n",
                  (unsigned int)ESP.getFreeHeap());
    Serial.printf("Free PSRAM: %u bytes\n",
                  (unsigned int)ESP.getFreePsram());

    Serial.printf("Epsilon: %.2f\n", EPSILON);
    Serial.printf("MinPts: %d\n", MIN_PTS);
    Serial.printf("Measurement sigma: %.3f m\n", MEASUREMENT_SIGMA);
    Serial.printf("Process Q: %.3f\n", PROCESS_Q);
    Serial.printf("Target FPS: %.1f\n", 1.0f / DT);
    Serial.printf("Frame budget: %.1f ms\n", FRAME_BUDGET_MS);
    Serial.printf("Max points capacity: %d\n", MAX_POINTS);

    for (int i = 0; i < LOAD_COUNT; ++i) {
        runLoad(LOADS[i]);
    }

    Serial.println();
    Serial.println("Experiment 10 completed.");
    Serial.println("Send the complete Serial Monitor output.");
}

void loop() {
    delay(1000);
}
