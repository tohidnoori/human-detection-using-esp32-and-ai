#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <esp_timer.h>

// ============================================================================
// EXPERIMENT 12 — second audit revision
// A : Kalman->DBSCAN     B : DBSCAN->Kalman     BL : raw DBSCAN centroids
//
// Changes since previous revision (all identified defects addressed):
//   [x] KalmanAxis::correct() Joseph-form equations corrected.
//   [x] B evaluation uses current kf.pos(), not stale lastPos.
//   [x] 2x2 solveAssignment gate handling corrected; single-match fallback.
//   [x] Evaluator uses exhaustive assignment (2 GT x N outputs).
//   [x] Identity switches gated on GT separation > 1 m and clear margin.
//   [x] False-positive and miss counts retained and printed per scenario.
//   [x] Point-track association uses Mahalanobis gate with predicted P00.
//   [x] p95 latency computed and reported per pipeline.
//   [x] Active scenario sigma forwarded into filter R and association logic
//       (documented as "estimator knows true sigma"); flag to disable.
//   [x] Crossing trajectory comment corrected (paths intersect at ~7.5 s).
// ============================================================================

// ---------------------------------------------------------------- Config ---
static constexpr int    SENSOR_COUNT          = 3;
static constexpr int    HUMAN_COUNT           = 2;
static constexpr int    PTS_PER_HUMAN_SENSOR  = 40;
static constexpr int    NOISE_PTS_PER_SENSOR  = 8;

static constexpr int    MAX_TRACKED  = SENSOR_COUNT * HUMAN_COUNT * PTS_PER_HUMAN_SENSOR;
static constexpr int    MAX_POINTS   = MAX_TRACKED + SENSOR_COUNT * NOISE_PTS_PER_SENSOR;
static constexpr int    MAX_POINT_TRACKS = 2 * MAX_POINTS;
static constexpr int    MAX_HUMAN_TRACKS = 8;

static_assert(HUMAN_COUNT == 2,
    "Exhaustive evaluator assumes exactly 2 ground-truth targets.");

static constexpr float  DT             = 0.1f;
static constexpr float  MEAS_SIGMA     = 0.05f;
static constexpr float  MEAS_VAR       = MEAS_SIGMA * MEAS_SIGMA;

static constexpr float  Q_HUMAN_PSD    = 0.50f;    // m^2/s^3
static constexpr float  Q_POINT_PSD    = 1.0e-4f;  // m^2/s^3

static constexpr float  EPSILON        = 0.35f;
static constexpr float  EPSILON_SQ     = EPSILON * EPSILON;
static constexpr int    MIN_PTS        = 5;
static constexpr int    MIN_HUMAN_CLUSTER_PTS = 15;

static constexpr float  BODY_EXT_XY    = 0.12f;
static constexpr float  BODY_EXT_Z     = 0.20f;

static constexpr int    GRID_BUCKETS   = 2048;
static constexpr float  ASSOC_GATE     = 1.10f;
static constexpr float  ASSOC_GATE_SQ  = ASSOC_GATE * ASSOC_GATE;
static constexpr float  MISS_PENALTY_SQ = ASSOC_GATE_SQ;
static constexpr float  CONFIDENT_MATCH_SQ = 0.25f;

// Identity metrics require GT targets separated by more than this.
static constexpr float  ID_GT_SEP_SQ   = 1.0f;   // 1 m^2
// 3-DOF chi-square upper tail at p=0.999; gate for point-track association.
static constexpr float  GATE_CHI2_3DOF = 16.266f;

static constexpr int    MAX_HUMAN_MISS = 4;
static constexpr int    MAX_POINT_MISS = 2;
static constexpr int    CONFIRM_HITS   = 2;

static constexpr int    FRAMES_PER_RUN = 300;
static constexpr int    WARMUP_FRAMES  = 50;

static constexpr int    N_SEEDS        = 5;
static constexpr int    MAX_SEEDS      = 32;

// If true, the Kalman filters and association gates use the *scenario's*
// active measurement sigma, not the default MEAS_SIGMA.  This is the
// "well-specified estimator" test.  Set to false to test robustness to a
// mis-specified noise model.
static constexpr bool   KALMAN_KNOWS_TRUE_SIGMA = true;

// --------------------------------------------------------------- Types -----
struct Point3D {
    float x = 0.0f, y = 0.0f, z = 0.0f;
    Point3D() = default;
    Point3D(float X, float Y, float Z) : x(X), y(Y), z(Z) {}
};

struct GroundTruth { float x, y, z; };

enum ScenarioKind { SCEN_WALK, SCEN_CROSSING, SCEN_MERGE };

// Active measurement variance: default, overridable per scenario.
static float g_measVar   = MEAS_VAR;
static float g_measSigma = MEAS_SIGMA;

// ------------------------------------------------------------- Utilities ---
static inline float square2(float v) { return v * v; }
static inline float distSq(const Point3D &a, const Point3D &b) {
    return square2(a.x - b.x) + square2(a.y - b.y) + square2(a.z - b.z);
}

static uint32_t rngState = 0xA53C91E7u;
static inline uint32_t random32() {
    rngState ^= rngState << 13; rngState ^= rngState >> 17;
    rngState ^= rngState << 5;  return rngState;
}
static inline float uniform01() { return (random32() & 0x00FFFFFFu) / 16777216.0f; }
static inline float uniformRange(float lo, float hi) { return lo + (hi - lo) * uniform01(); }
static float gaussianNoise() {
    float s = 0.0f;
    for (int i = 0; i < 12; ++i) s += uniform01();
    return s - 6.0f;
}
static inline uint32_t mix32(uint32_t x) {
    x ^= x >> 16; x *= 0x7feb352du;
    x ^= x >> 15; x *= 0x846ca68bu;
    x ^= x >> 16; return x;
}
static float stableOffset(int identity, uint32_t salt, float extent) {
    const uint32_t h = mix32((uint32_t)identity * 2654435761u + salt);
    const float u = (h & 0x00FFFFFFu) / 16777216.0f;
    return (u * 2.0f - 1.0f) * extent;
}

// ============================================================================
// 1-D Kalman axis, CV model.  API:
//     predict(dt)  - one frame advance (state + covariance)
//     correct(z)   - measurement update ONLY (no state advance)
//     initialize(z)- first measurement
// Call predict() exactly once per frame per track, then correct() for
// matched tracks.  Never call predict inside correct.
//
// Continuous-time acceleration-noise PSD q (m^2/s^3) → discrete process
// covariance Q(dt) = q * [[dt^4/4, dt^3/2], [dt^3/2, dt^2]].
// ============================================================================
struct KalmanAxis {
    float pos = 0.0f, vel = 0.0f;
    float p00 = 1.0f, p01 = 0.0f, p10 = 0.0f, p11 = 1.0f;
    float q = 0.0f;
    float r = MEAS_VAR;
    bool  init = false;

    void reset(float rVar, float qPsd) {
        pos = vel = 0.0f;
        p00 = 1.0f; p01 = p10 = 0.0f; p11 = 1.0f;
        q = qPsd; r = rVar; init = false;
    }
    void initialize(float z) {
        pos = z; vel = 0.0f;
        p00 = r; p01 = 0.0f; p10 = 0.0f; p11 = 1.0f;
        init = true;
    }
    void predict(float dt) {
        if (!init) return;
        pos += vel * dt;
        const float d2 = dt * dt, d3 = d2 * dt, d4 = d2 * d2;
        const float n00 = p00 + dt * (p01 + p10) + d2 * p11 + q * d4 * 0.25f;
        const float n01 = p01 + dt * p11 + q * d3 * 0.5f;
        const float n10 = p10 + dt * p11 + q * d3 * 0.5f;
        const float n11 = p11 + q * d2;
        p00 = n00; p01 = n01; p10 = n10; p11 = n11;
    }
    // Correct Joseph-form covariance update for H = [1 0].
    //   P_new = (I-KH) P (I-KH)^T + K R K^T
    // Expands to:
    //   P00' = a0^2 * P00 + K0^2 * R
    //   P01' = a0 * (P01 - K1*P00) + K0*K1*R
    //   P10' = a0 * (P10 - K1*P00) + K1*K0*R
    //   P11' = P11 - K1*(P01+P10) + K1^2*(P00+R)
    void correct(float z) {
        if (!init) { initialize(z); return; }

        const float o00 = p00, o01 = p01, o10 = p10, o11 = p11;

        const float S  = o00 + r;
        const float k0 = o00 / S;
        const float k1 = o10 / S;

        const float innovation = z - pos;
        pos += k0 * innovation;
        vel += k1 * innovation;

        const float a0 = 1.0f - k0;
        const float np00 = a0 * a0 * o00 + k0 * k0 * r;
        const float np01 = a0 * (o01 - k1 * o00) + k0 * k1 * r;
        const float np10 = a0 * (o10 - k1 * o00) + k1 * k0 * r;
        const float np11 = o11 - k1 * (o01 + o10) + k1 * k1 * (o00 + r);

        p00 = (np00 < 1e-6f) ? 1e-6f : np00;
        p11 = (np11 < 1e-6f) ? 1e-6f : np11;
        const float offDiag = 0.5f * (np01 + np10);   // enforce symmetry
        p01 = offDiag;
        p10 = offDiag;
    }
};

struct Kalman3D {
    KalmanAxis ax, ay, az;
    void reset(float rVar, float qPsd) {
        ax.reset(rVar, qPsd); ay.reset(rVar, qPsd); az.reset(rVar, qPsd);
    }
    void initialize(const Point3D &m) {
        ax.initialize(m.x); ay.initialize(m.y); az.initialize(m.z);
    }
    void predict(float dt) { ax.predict(dt); ay.predict(dt); az.predict(dt); }
    void correct(const Point3D &m) { ax.correct(m.x); ay.correct(m.y); az.correct(m.z); }
    Point3D pos() const { return Point3D(ax.pos, ay.pos, az.pos); }
};

// ============================================================================
// Sensor model.  Documented simplification: each sensor produces a direct
// 3-D position observation with a fixed bias and additive white Gaussian
// noise.  The 3-D sensor coordinates are metadata only; they are NOT used
// to project measurements.  This is not a physical antenna simulator.
// ============================================================================
struct SensorSpec {
    Point3D pos;
    float   bias;
    float   sigma;
};

static SensorSpec sensors[SENSOR_COUNT];

static void initSensors(float sigma) {
    sensors[0] = { Point3D(-4.0f,  0.0f,  3.0f),  0.005f, sigma };
    sensors[1] = { Point3D( 0.0f,  4.0f,  3.5f), -0.008f, sigma };
    sensors[2] = { Point3D( 3.5f, -3.0f,  3.0f),  0.004f, sigma };
}

// ============================================================================
// Trajectories
// ============================================================================
struct TrajSample { Point3D p; };

// WALK: parallel paths, 3 m apart in y, no crossing.
static TrajSample trajWalk(int human, float t) {
    const float speed = 0.10f;
    TrajSample s{};
    s.p.x = -1.0f + speed * t;
    s.p.y = (human == 0) ? -1.5f : 1.5f;
    s.p.y += 0.03f * sinf(0.5f * t + human * 1.7f);
    s.p.z = (human == 0) ? 0.90f : 0.95f;
    return s;
}

// CROSSING: human 0 travels along +x, human 1 along +y.  Both use linear
// ramps from ±1.5 to ±1.5 over 0..15 s.  Their paths cross at the origin
// near t = 7.5 s (mid-ramp), then both continue to their endpoints by
// t = 15 s and remain at the endpoints.
static TrajSample trajCrossing(int human, float t) {
    const float T = (t < 0.0f) ? 0.0f : t;
    TrajSample s{};
    const float phase = (T < 15.0f) ? (T / 15.0f) : 1.0f;
    if (human == 0) {
        s.p.x = -1.5f + 3.0f * phase;
        s.p.y =  0.05f * sinf(0.4f * T);
        s.p.z = 0.90f;
    } else {
        s.p.y = -1.5f + 3.0f * phase;
        s.p.x = -0.05f * sinf(0.4f * T);
        s.p.z = 0.95f;
    }
    return s;
}

// MERGE: approach (±2, ±1.2) to (0, ±0.10) by t = 10 s, hold 0.20 m apart
// until t = 20 s (below EPSILON so DBSCAN merges them), then separate.
static TrajSample trajMerge(int human, float t) {
    const float T = t;
    const float ySign = (human == 0) ? -1.0f : 1.0f;
    TrajSample s{};
    if (T < 10.0f) {
        const float p = T / 10.0f;
        s.p.x = (human == 0) ? (-2.0f + 2.0f * p) : (2.0f - 2.0f * p);
        s.p.y = ySign * (1.2f - 1.1f * p);
    } else if (T < 20.0f) {
        s.p.x = 0.0f;
        s.p.y = ySign * 0.10f;
    } else {
        const float p = (T - 20.0f) / 10.0f;
        s.p.x = (human == 0) ? (-2.0f * p) : (2.0f * p);
        s.p.y = ySign * (0.10f + 1.1f * p);
    }
    s.p.z = (human == 0) ? 0.90f : 0.95f;
    return s;
}

static TrajSample humanTrajectory(ScenarioKind kind, int human, float t) {
    switch (kind) {
        case SCEN_WALK:     return trajWalk(human, t);
        case SCEN_CROSSING: return trajCrossing(human, t);
        case SCEN_MERGE:    return trajMerge(human, t);
    }
    return trajWalk(human, t);
}

// ============================================================================
// Point buffers — single source of truth for A, B, and BL
// ============================================================================
static Point3D rawPoints[MAX_POINTS];
static Point3D filteredPoints[MAX_POINTS];
static int rawCount = 0;
static int filteredCount = 0;

static int generateFrame(ScenarioKind kind, int seed, int frameIndex, float dropP) {
    rngState = 0xA53C91E7u
             ^ (uint32_t)(seed * 1597334677u)
             ^ (uint32_t)(frameIndex * 2654435761u);
    const float t = frameIndex * DT;
    int n = 0;

    for (int s = 0; s < SENSOR_COUNT; ++s) {
        for (int h = 0; h < HUMAN_COUNT; ++h) {
            // Documented as sensor-human group dropout (whole body group).
            if (dropP > 0.0f && uniform01() < dropP) continue;
            const TrajSample tr = humanTrajectory(kind, h, t);
            for (int k = 0; k < PTS_PER_HUMAN_SENSOR; ++k) {
                if (n >= MAX_POINTS) break;
                const int id = (s * HUMAN_COUNT + h) * PTS_PER_HUMAN_SENSOR + k;
                const Point3D body(
                    tr.p.x + stableOffset(id, 0x11u, BODY_EXT_XY),
                    tr.p.y + stableOffset(id, 0x22u, BODY_EXT_XY),
                    tr.p.z + stableOffset(id, 0x33u, BODY_EXT_Z));
                rawPoints[n++] = Point3D(
                    body.x + sensors[s].bias + sensors[s].sigma * gaussianNoise(),
                    body.y + sensors[s].bias + sensors[s].sigma * gaussianNoise(),
                    body.z + sensors[s].bias + sensors[s].sigma * gaussianNoise());
            }
        }
    }

    for (int s = 0; s < SENSOR_COUNT; ++s) {
        for (int j = 0; j < NOISE_PTS_PER_SENSOR; ++j) {
            if (n >= MAX_POINTS) break;
            rawPoints[n++] = Point3D(
                uniformRange(-3.5f, 3.5f),
                uniformRange(-3.5f, 3.5f),
                uniformRange(0.2f, 2.0f));
        }
    }
    return n;
}

// ============================================================================
// DBSCAN (grid-accelerated, iterative, separate buffers)
// ============================================================================
static int labels[MAX_POINTS];
static int gridHead[GRID_BUCKETS];
static int gridNext[MAX_POINTS];
static int cellX[MAX_POINTS], cellY[MAX_POINTS], cellZ[MAX_POINTS];
static uint32_t seen[MAX_POINTS];
static uint32_t stamp = 0;
static int initialNeighbors[MAX_POINTS];
static int expansionBuf[MAX_POINTS];
static int queueBuf[MAX_POINTS];

static inline int hashCell(int x, int y, int z) {
    const uint32_t h = ((uint32_t)x * 73856093u)
                     ^ ((uint32_t)y * 19349663u)
                     ^ ((uint32_t)z * 83492791u);
    return (int)(h % GRID_BUCKETS);
}

static void buildGrid(const Point3D *pts, int n) {
    for (int i = 0; i < GRID_BUCKETS; ++i) gridHead[i] = -1;
    for (int i = 0; i < n; ++i) {
        cellX[i] = (int)floorf(pts[i].x / EPSILON);
        cellY[i] = (int)floorf(pts[i].y / EPSILON);
        cellZ[i] = (int)floorf(pts[i].z / EPSILON);
        const int b = hashCell(cellX[i], cellY[i], cellZ[i]);
        gridNext[i] = gridHead[b];
        gridHead[b] = i;
    }
}

static int regionQuery(const Point3D *pts, int n, int idx, int *out) {
    if (++stamp == 0) { memset(seen, 0, sizeof(seen)); stamp = 1; }
    const int cx = cellX[idx], cy = cellY[idx], cz = cellZ[idx];
    int count = 0;
    for (int dx = -1; dx <= 1; ++dx)
    for (int dy = -1; dy <= 1; ++dy)
    for (int dz = -1; dz <= 1; ++dz) {
        const int tx = cx + dx, ty = cy + dy, tz = cz + dz;
        const int b = hashCell(tx, ty, tz);
        for (int j = gridHead[b]; j != -1; j = gridNext[j]) {
            if (j < 0 || j >= n) continue;
            if (cellX[j] != tx || cellY[j] != ty || cellZ[j] != tz) continue;
            if (seen[j] == stamp) continue;
            seen[j] = stamp;
            if (distSq(pts[idx], pts[j]) <= EPSILON_SQ) {
                if (count < MAX_POINTS) out[count++] = j;
            }
        }
    }
    return count;
}

static int runDBSCAN(const Point3D *pts, int n) {
    if (n <= 0) return 0;
    for (int i = 0; i < n; ++i) labels[i] = -2;

    int clusterId = 0;
    for (int i = 0; i < n; ++i) {
        if (labels[i] != -2) continue;

        const int nn = regionQuery(pts, n, i, initialNeighbors);
        if (nn < MIN_PTS) { labels[i] = -1; continue; }

        labels[i] = clusterId;

        int qHead = 0, qTail = 0;
        for (int k = 0; k < nn; ++k) {
            const int j = initialNeighbors[k];
            if (j == i) continue;
            if (labels[j] == -2 || labels[j] == -1) {
                labels[j] = clusterId;
                if (qTail < MAX_POINTS) queueBuf[qTail++] = j;
            }
        }

        while (qHead < qTail) {
            const int q = queueBuf[qHead++];
            const int ec = regionQuery(pts, n, q, expansionBuf);
            if (ec < MIN_PTS) continue;
            for (int k = 0; k < ec; ++k) {
                const int j = expansionBuf[k];
                if (labels[j] == -2 || labels[j] == -1) {
                    labels[j] = clusterId;
                    if (qTail < MAX_POINTS) queueBuf[qTail++] = j;
                }
            }
        }
        ++clusterId;
    }
    return clusterId;
}

// ============================================================================
// Centroid extraction
// ============================================================================
struct ClusterCentroid { Point3D p; int size; int label; };

static int extractCentroidsInto(const Point3D *pts, int n, int clusterCount,
                                ClusterCentroid *out) {
    static int sz[MAX_POINTS];
    static double sx[MAX_POINTS], sy[MAX_POINTS], szv[MAX_POINTS];
    for (int c = 0; c < clusterCount; ++c) { sz[c] = 0; sx[c] = sy[c] = szv[c] = 0.0; }
    for (int i = 0; i < n; ++i) {
        const int c = labels[i];
        if (c >= 0 && c < clusterCount) {
            ++sz[c];
            sx[c] += pts[i].x; sy[c] += pts[i].y; szv[c] += pts[i].z;
        }
    }
    int count = 0;
    for (int c = 0; c < clusterCount; ++c) {
        if (sz[c] < MIN_HUMAN_CLUSTER_PTS) continue;
        out[count].p = Point3D(
            (float)(sx[c] / sz[c]),
            (float)(sy[c] / sz[c]),
            (float)(szv[c] / sz[c]));
        out[count].size = sz[c];
        out[count].label = c;
        ++count;
    }
    for (int i = 0; i < count; ++i)
        for (int j = i + 1; j < count; ++j)
            if (out[j].size > out[i].size) {
                const ClusterCentroid t = out[i]; out[i] = out[j]; out[j] = t;
            }
    if (count > MAX_HUMAN_TRACKS) count = MAX_HUMAN_TRACKS;
    return count;
}

static ClusterCentroid centroids[MAX_POINTS];
static ClusterCentroid filteredCentroids[MAX_POINTS];

// ============================================================================
// Architecture A — point-level Kalman BEFORE DBSCAN
// ============================================================================
struct PointTrack {
    Kalman3D kf;
    int   miss = 0;
    int   hits = 0;
    bool  alive = false;
    bool  matchedThisFrame = false;
    Point3D lastFiltered;
};

static PointTrack pointTracks[MAX_POINT_TRACKS];
static int        pointTrackCount = 0;

static int     ptGridHead[GRID_BUCKETS];
static int     ptGridNext[MAX_POINT_TRACKS];
static int     ptCellX[MAX_POINT_TRACKS];
static int     ptCellY[MAX_POINT_TRACKS];
static int     ptCellZ[MAX_POINT_TRACKS];
static Point3D ptPredicted[MAX_POINT_TRACKS];
static bool    s_ptUsedPoint[MAX_POINTS];

static void resetPointTracks() { pointTrackCount = 0; }

static void compactPointTracks() {
    int w = 0;
    for (int i = 0; i < pointTrackCount; ++i) {
        if (pointTracks[i].alive) {
            if (w != i) pointTracks[w] = pointTracks[i];
            ++w;
        }
    }
    pointTrackCount = w;
}

static void buildPointTrackGrid() {
    for (int i = 0; i < GRID_BUCKETS; ++i) ptGridHead[i] = -1;
    for (int i = 0; i < pointTrackCount; ++i) {
        if (!pointTracks[i].alive) continue;
        ptPredicted[i] = pointTracks[i].kf.pos();
        ptCellX[i] = (int)floorf(ptPredicted[i].x / EPSILON);
        ptCellY[i] = (int)floorf(ptPredicted[i].y / EPSILON);
        ptCellZ[i] = (int)floorf(ptPredicted[i].z / EPSILON);
        const int b = hashCell(ptCellX[i], ptCellY[i], ptCellZ[i]);
        ptGridNext[i] = ptGridHead[b];
        ptGridHead[b] = i;
    }
}

// Mahalanobis squared distance for a point-track candidate.  Uses the
// per-axis predicted position variance plus the active measurement
// variance.  Units cancel; the result is dimensionless.
static inline float ptMahalanobisSq(const Point3D &m, int j) {
    const float sx = pointTracks[j].kf.ax.p00 + g_measVar;
    const float sy = pointTracks[j].kf.ay.p00 + g_measVar;
    const float sz = pointTracks[j].kf.az.p00 + g_measVar;
    const float dx = m.x - ptPredicted[j].x;
    const float dy = m.y - ptPredicted[j].y;
    const float dz = m.z - ptPredicted[j].z;
    const float inv_sx = (sx > 1e-9f) ? (1.0f / sx) : 0.0f;
    const float inv_sy = (sy > 1e-9f) ? (1.0f / sy) : 0.0f;
    const float inv_sz = (sz > 1e-9f) ? (1.0f / sz) : 0.0f;
    return dx * dx * inv_sx + dy * dy * inv_sy + dz * dz * inv_sz;
}

static int runArchitectureA_FilterPoints() {
    // 1) One prediction per track per frame.
    for (int i = 0; i < pointTrackCount; ++i)
        if (pointTracks[i].alive) pointTracks[i].kf.predict(DT);

    // 2) Grid over predicted positions.
    buildPointTrackGrid();

    // 3) Mahalanobis-gated nearest-neighbour association.
    for (int i = 0; i < pointTrackCount; ++i) pointTracks[i].matchedThisFrame = false;
    for (int i = 0; i < rawCount; ++i)        s_ptUsedPoint[i] = false;

    for (int i = 0; i < rawCount; ++i) {
        const Point3D &m = rawPoints[i];
        const int cx = (int)floorf(m.x / EPSILON);
        const int cy = (int)floorf(m.y / EPSILON);
        const int cz = (int)floorf(m.z / EPSILON);
        int best = -1;
        float bestD2 = GATE_CHI2_3DOF;
        // Search ±2 cells: covers chi-square gate even after several missed
        // predictions where P00 grows.
        for (int dx = -2; dx <= 2; ++dx)
        for (int dy = -2; dy <= 2; ++dy)
        for (int dz = -2; dz <= 2; ++dz) {
            const int b = hashCell(cx + dx, cy + dy, cz + dz);
            for (int j = ptGridHead[b]; j != -1; j = ptGridNext[j]) {
                if (pointTracks[j].matchedThisFrame) continue;
                if (!pointTracks[j].alive) continue;
                const float d2 = ptMahalanobisSq(m, j);
                if (d2 < bestD2) { bestD2 = d2; best = j; }
            }
        }
        if (best >= 0) {
            pointTracks[best].kf.correct(m);
            pointTracks[best].lastFiltered = pointTracks[best].kf.pos();
            pointTracks[best].miss = 0;
            if (pointTracks[best].hits < 255) ++pointTracks[best].hits;
            pointTracks[best].matchedThisFrame = true;
            s_ptUsedPoint[i] = true;
        }
    }

    // 4) Spawn tracks for unmatched points.
    for (int i = 0; i < rawCount; ++i) {
        if (s_ptUsedPoint[i]) continue;
        if (pointTrackCount >= MAX_POINT_TRACKS) break;
        PointTrack &nt = pointTracks[pointTrackCount++];
        nt.kf.reset(g_measVar, Q_POINT_PSD);
        nt.kf.initialize(rawPoints[i]);
        nt.lastFiltered = rawPoints[i];
        nt.miss = 0;
        nt.hits = 1;
        nt.alive = true;
        nt.matchedThisFrame = true;
    }

    // 5) Age out missed tracks.
    for (int i = 0; i < pointTrackCount; ++i) {
        if (!pointTracks[i].alive) continue;
        if (!pointTracks[i].matchedThisFrame) {
            if (++pointTracks[i].miss > MAX_POINT_MISS) pointTracks[i].alive = false;
        }
    }

    // 6) Compact.
    compactPointTracks();

    // 7) Emit only matched-this-frame tracks.
    int out = 0;
    for (int i = 0; i < pointTrackCount && out < MAX_POINTS; ++i) {
        if (pointTracks[i].alive && pointTracks[i].matchedThisFrame)
            filteredPoints[out++] = pointTracks[i].lastFiltered;
    }
    return out;
}

// ============================================================================
// Architecture B — DBSCAN -> centroid -> Kalman tracking
// ============================================================================
struct HumanTrack {
    Kalman3D kf;
    int   id = -1;
    int   miss = 0;
    int   hits = 0;
    bool  confirmed = false;
    bool  alive = false;
    bool  matchedThisFrame = false;
    int   evalLastGt = -1;   // diagnostics only
    Point3D lastPos;         // last measurement-updated position (diagnostic)
};

static HumanTrack humanTracks[MAX_HUMAN_TRACKS];
static int        humanTrackCount = 0;
static int        nextHumanId = 1;

static void resetHumanTracks() { humanTrackCount = 0; nextHumanId = 1; }

static void compactHumanTracks() {
    int w = 0;
    for (int i = 0; i < humanTrackCount; ++i) {
        if (humanTracks[i].alive) {
            if (w != i) humanTracks[w] = humanTracks[i];
            ++w;
        }
    }
    humanTrackCount = w;
}

static int countConfirmedTracks() {
    int n = 0;
    for (int i = 0; i < humanTrackCount; ++i)
        if (humanTracks[i].alive && humanTracks[i].confirmed) ++n;
    return n;
}

// Assignment.  For the 2x2 case, both complete assignments are evaluated;
// any that passes the gate is eligible; the cheaper of the eligible ones
// wins.  If neither complete assignment is valid, the best single gated
// match is applied.  Greedy fallback for larger cases.
static void solveAssignment(const Point3D *predicted, int trackCount,
                            const Point3D *dets, int detCount,
                            int *trackToDet) {
    for (int i = 0; i < trackCount; ++i) trackToDet[i] = -1;
    if (detCount == 0 || trackCount == 0) return;

    if (detCount == 1) {
        int best = -1; float bestD = ASSOC_GATE_SQ;
        for (int t = 0; t < trackCount; ++t) {
            const float d = distSq(predicted[t], dets[0]);
            if (d < bestD) { bestD = d; best = t; }
        }
        if (best >= 0) trackToDet[best] = 0;
        return;
    }

    if (detCount == 2 && trackCount == 2) {
        const float d00 = distSq(predicted[0], dets[0]);
        const float d01 = distSq(predicted[0], dets[1]);
        const float d10 = distSq(predicted[1], dets[0]);
        const float d11 = distSq(predicted[1], dets[1]);
        const bool aGate = (d00 <= ASSOC_GATE_SQ) && (d11 <= ASSOC_GATE_SQ);
        const bool bGate = (d01 <= ASSOC_GATE_SQ) && (d10 <= ASSOC_GATE_SQ);

        if (aGate || bGate) {
            // Pick the cheaper of the *valid* complete assignments.
            if (aGate && (!bGate || (d00 + d11) <= (d01 + d10))) {
                trackToDet[0] = 0; trackToDet[1] = 1;
            } else {
                trackToDet[0] = 1; trackToDet[1] = 0;
            }
            return;
        }

        // Neither complete assignment is valid: fall back to the best
        // single gated match, leaving the other track unmatched.
        float bestSingle = ASSOC_GATE_SQ;
        int bt = -1, bd = -1;
        if (d00 < bestSingle) { bestSingle = d00; bt = 0; bd = 0; }
        if (d01 < bestSingle) { bestSingle = d01; bt = 0; bd = 1; }
        if (d10 < bestSingle) { bestSingle = d10; bt = 1; bd = 0; }
        if (d11 < bestSingle) { bestSingle = d11; bt = 1; bd = 1; }
        if (bt >= 0) trackToDet[bt] = bd;
        return;
    }

    // Greedy fallback for larger cases (documented limitation).
    bool detUsed[MAX_HUMAN_TRACKS] = {};
    for (int t = 0; t < trackCount; ++t) {
        int bestD = -1; float bestDist = ASSOC_GATE_SQ;
        for (int d = 0; d < detCount; ++d) {
            if (detUsed[d]) continue;
            const float dd = distSq(predicted[t], dets[d]);
            if (dd < bestDist) { bestDist = dd; bestD = d; }
        }
        if (bestD >= 0) { trackToDet[t] = bestD; detUsed[bestD] = true; }
    }
}

static void runArchitectureB_Update(const ClusterCentroid *dets, int detCount) {
    // 1) One prediction per live track per frame.
    for (int i = 0; i < humanTrackCount; ++i)
        if (humanTracks[i].alive) humanTracks[i].kf.predict(DT);

    // 2) Association uses post-predict positions.
    Point3D predicted[MAX_HUMAN_TRACKS];
    for (int t = 0; t < humanTrackCount; ++t) predicted[t] = humanTracks[t].kf.pos();

    Point3D detPts[MAX_HUMAN_TRACKS];
    for (int d = 0; d < detCount; ++d) detPts[d] = dets[d].p;

    int trackToDet[MAX_HUMAN_TRACKS];
    solveAssignment(predicted, humanTrackCount, detPts, detCount, trackToDet);

    // 3) Measurement update for matched tracks only.
    bool detUsed[MAX_HUMAN_TRACKS] = {};
    for (int t = 0; t < humanTrackCount; ++t) {
        humanTracks[t].matchedThisFrame = false;
        const int d = trackToDet[t];
        if (d >= 0 && !detUsed[d]) {
            humanTracks[t].kf.correct(detPts[d]);
            humanTracks[t].lastPos = humanTracks[t].kf.pos();
            humanTracks[t].miss = 0;
            humanTracks[t].matchedThisFrame = true;
            if (humanTracks[t].hits < 255) ++humanTracks[t].hits;
            if (humanTracks[t].hits >= CONFIRM_HITS) humanTracks[t].confirmed = true;
            detUsed[d] = true;
        } else {
            if (++humanTracks[t].miss > MAX_HUMAN_MISS) humanTracks[t].alive = false;
        }
    }

    // 4) Spawn tracks for unmatched detections.
    for (int d = 0; d < detCount; ++d) {
        if (detUsed[d]) continue;
        if (humanTrackCount >= MAX_HUMAN_TRACKS) break;
        HumanTrack &nt = humanTracks[humanTrackCount++];
        nt.kf.reset(g_measVar, Q_HUMAN_PSD);
        nt.kf.initialize(detPts[d]);
        nt.id = nextHumanId++;
        nt.miss = 0;
        nt.hits = 1;
        nt.alive = true;
        nt.confirmed = false;
        nt.matchedThisFrame = true;
        nt.evalLastGt = -1;
        nt.lastPos = detPts[d];
    }

    // 5) Compact.
    compactHumanTracks();
}

// ============================================================================
// Evaluation.  All three pipelines use the same exhaustive 2-GT assignment
// contract; missing targets incur MISS_PENALTY_SQ.
// ============================================================================
struct Metrics {
    double sumErrSq = 0.0;
    double sumErrSqX = 0.0, sumErrSqY = 0.0, sumErrSqZ = 0.0;
    uint32_t samples = 0;
    uint32_t misses = 0;
    uint32_t falsePositives = 0;
    uint32_t idSwitches = 0;
    uint32_t idEligible = 0;

    uint32_t frames0 = 0, frames1 = 0, frames2 = 0, frames3plus = 0;

    uint64_t totalUs = 0;
    uint64_t maxUs = 0;
    int      overruns = 0;
    uint32_t latUs[FRAMES_PER_RUN];
    int      latCount = 0;
};

static Metrics s_mA, s_mB, s_mBase;

static void resetAllMetrics() { s_mA = Metrics{}; s_mB = Metrics{}; s_mBase = Metrics{}; }

// Exhaustive assignment for HUMAN_COUNT == 2.
// Enumerate (a, b) with a, b in {-1, 0..nOut-1}, a != b when both >= 0.
// A match with d^2 > ASSOC_GATE_SQ is rejected.  Maximise the number of
// matched GT targets; break ties by lower total squared error.
struct FrameAssignment { int a; int b; int matched; double totalSq; };

static FrameAssignment bestAssignment(const Point3D *outputs, int nOut,
                                      const GroundTruth *gt) {
    FrameAssignment best{};
    best.a = -1; best.b = -1; best.matched = 0; best.totalSq = 1e30;
    for (int a = -1; a < nOut; ++a) {
        for (int b = -1; b < nOut; ++b) {
            if (a >= 0 && a == b) continue;
            int matched = 0;
            double sq = 0.0;
            bool ok = true;
            if (a < 0) {
                sq += MISS_PENALTY_SQ;
            } else {
                const float d2 = distSq(outputs[a], Point3D(gt[0].x, gt[0].y, gt[0].z));
                if (d2 > ASSOC_GATE_SQ) { ok = false; }
                else { sq += d2; ++matched; }
            }
            if (!ok) continue;
            if (b < 0) {
                sq += MISS_PENALTY_SQ;
            } else {
                const float d2 = distSq(outputs[b], Point3D(gt[1].x, gt[1].y, gt[1].z));
                if (d2 > ASSOC_GATE_SQ) { ok = false; }
                else { sq += d2; ++matched; }
            }
            if (!ok) continue;
            if (matched > best.matched ||
                (matched == best.matched && sq < best.totalSq)) {
                best.matched = matched;
                best.totalSq = sq;
                best.a = a;
                best.b = b;
            }
        }
    }
    return best;
}

static void accumulateAssignment(const FrameAssignment &as,
                                 const Point3D *outputs,
                                 const GroundTruth *gt, Metrics &m) {
    int assignment[2] = { as.a, as.b };
    bool usedOut[MAX_POINTS] = {};
    for (int t = 0; t < HUMAN_COUNT; ++t) {
        if (assignment[t] >= 0) {
            usedOut[assignment[t]] = true;
            const Point3D g(gt[t].x, gt[t].y, gt[t].z);
            const float d2 = distSq(outputs[assignment[t]], g);
            m.sumErrSq += d2;
            m.sumErrSqX += square2(outputs[assignment[t]].x - g.x);
            m.sumErrSqY += square2(outputs[assignment[t]].y - g.y);
            m.sumErrSqZ += square2(outputs[assignment[t]].z - g.z);
        } else {
            m.sumErrSq += MISS_PENALTY_SQ;
            m.sumErrSqX += MISS_PENALTY_SQ / 3.0;
            m.sumErrSqY += MISS_PENALTY_SQ / 3.0;
            m.sumErrSqZ += MISS_PENALTY_SQ / 3.0;
            ++m.misses;
        }
        ++m.samples;
    }
    (void)usedOut;
}

static void scoreOutputs(const Point3D *outputs, int nOut,
                         const GroundTruth *gt, Metrics &m) {
    const FrameAssignment as = bestAssignment(outputs, nOut, gt);
    accumulateAssignment(as, outputs, gt, m);
    // False positives: outputs not assigned to any GT.
    bool usedOut[MAX_POINTS] = {};
    if (as.a >= 0) usedOut[as.a] = true;
    if (as.b >= 0) usedOut[as.b] = true;
    for (int i = 0; i < nOut; ++i) if (!usedOut[i]) ++m.falsePositives;
}

// Track scoring.  Uses current kf.pos() so missed tracks still contribute
// their predicted position — consistent with coverage counting them as
// live, confirmed outputs.  Identity switches counted only when GT targets
// are separated by more than 1 m and the assignment is unambiguous.
static void scoreTracks(HumanTrack *tracks, int nTracks,
                        const GroundTruth *gt, Metrics &m) {
    // Build current-estimate outputs.
    Point3D outputs[MAX_HUMAN_TRACKS];
    int     idxMap[MAX_HUMAN_TRACKS];
    int     nOut = 0;
    for (int i = 0; i < nTracks; ++i) {
        if (!tracks[i].alive || !tracks[i].confirmed) continue;
        outputs[nOut] = tracks[i].kf.pos();
        idxMap[nOut] = i;
        ++nOut;
    }
    const FrameAssignment as = bestAssignment(outputs, nOut, gt);
    accumulateAssignment(as, outputs, gt, m);

    // False positives among confirmed tracks not assigned.
    bool usedOut[MAX_HUMAN_TRACKS] = {};
    if (as.a >= 0) usedOut[as.a] = true;
    if (as.b >= 0) usedOut[as.b] = true;
    for (int i = 0; i < nOut; ++i) if (!usedOut[i]) ++m.falsePositives;

    // Identity metric: only when GT targets are well separated and the
    // current assignment is confident.
    const float gtSepSq = distSq(Point3D(gt[0].x, gt[0].y, gt[0].z),
                                 Point3D(gt[1].x, gt[1].y, gt[1].z));
    if (gtSepSq <= ID_GT_SEP_SQ) return;

    const int assignment[2] = { as.a, as.b };
    for (int t = 0; t < HUMAN_COUNT; ++t) {
        if (assignment[t] < 0) continue;
        HumanTrack &track = tracks[idxMap[assignment[t]]];
        const Point3D g(gt[t].x, gt[t].y, gt[t].z);
        const float dAssigned = distSq(outputs[assignment[t]], g);
        if (dAssigned > CONFIDENT_MATCH_SQ) continue;

        // Require a clear margin: assigned GT must be at least 2x closer
        // than the other GT (i.e. dAssigned < dOther / 2).
        const int otherT = (t == 0) ? 1 : 0;
        const Point3D gOther(gt[otherT].x, gt[otherT].y, gt[otherT].z);
        const float dOther = distSq(outputs[assignment[t]], gOther);
        if (dAssigned * 2.0f >= dOther) continue;

        ++m.idEligible;
        if (track.evalLastGt != -1 && track.evalLastGt != t) ++m.idSwitches;
        track.evalLastGt = t;
    }
}

static void bumpCoverage(Metrics &m, int nOut) {
    if (nOut <= 0)        ++m.frames0;
    else if (nOut == 1)   ++m.frames1;
    else if (nOut == 2)   ++m.frames2;
    else                  ++m.frames3plus;
}

// ============================================================================
// Frame processing — same rawPoints[] for all three pipelines.
// ============================================================================
static void processFrame(int &clusterRaw, int &detRaw,
                         int &clusterFiltered, int &detFiltered,
                         uint64_t &aUs, uint64_t &blUs, uint64_t &bUs) {
    // ---- A: filter points, then DBSCAN -> centroids ----
    const uint64_t a0 = esp_timer_get_time();
    filteredCount = runArchitectureA_FilterPoints();
    buildGrid(filteredPoints, filteredCount);
    clusterFiltered = runDBSCAN(filteredPoints, filteredCount);
    detFiltered = extractCentroidsInto(filteredPoints, filteredCount,
                                       clusterFiltered, filteredCentroids);
    const uint64_t a1 = esp_timer_get_time();
    aUs = a1 - a0;

    // ---- BL (also the front-end for B): raw DBSCAN -> centroids ----
    const uint64_t b0 = esp_timer_get_time();
    buildGrid(rawPoints, rawCount);
    clusterRaw = runDBSCAN(rawPoints, rawCount);
    detRaw = extractCentroidsInto(rawPoints, rawCount, clusterRaw, centroids);
    const uint64_t b1 = esp_timer_get_time();
    blUs = b1 - b0;

    // ---- B: track those centroids ----
    runArchitectureB_Update(centroids, detRaw);
    const uint64_t b2 = esp_timer_get_time();
    bUs = b2 - b1;
}

// ============================================================================
// Per-seed and per-scenario aggregation
// ============================================================================
struct SeedOutcome {
    float rmseA = 0, rmseB = 0, rmseBL = 0;
    float idSwitches = 0;
    float idEligible = 0;
    float covA2 = 0, covB2 = 0, covBL2 = 0;
    float fpPerFrameA = 0, fpPerFrameB = 0, fpPerFrameBL = 0;
    float missPerFrameA = 0, missPerFrameB = 0, missPerFrameBL = 0;
    float overrunPctA = 0, overrunPctBL = 0, overrunPctB = 0;
    float p95UsA = 0, p95UsBL = 0, p95UsB = 0;
    float meanUsA = 0, meanUsBL = 0, meanUsB = 0;
    float maxUsA = 0, maxUsBL = 0, maxUsB = 0;
};

static float computeP95Us(const uint32_t *lat, int n) {
    if (n <= 0) return 0.0f;
    static uint32_t tmp[FRAMES_PER_RUN];
    for (int i = 0; i < n; ++i) tmp[i] = lat[i];
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j)
            if (tmp[j] < tmp[i]) { const uint32_t t = tmp[i]; tmp[i] = tmp[j]; tmp[j] = t; }
    const int idx = (int)(0.95f * (n - 1));
    return (float)tmp[idx];
}

static void runOneSeed(ScenarioKind kind, int seed, float sigma, float dropP,
                       SeedOutcome &out) {
    initSensors(sigma);
    if (KALMAN_KNOWS_TRUE_SIGMA) {
        g_measSigma = sigma;
        g_measVar   = sigma * sigma;
    } else {
        g_measSigma = MEAS_SIGMA;
        g_measVar   = MEAS_VAR;
    }

    resetPointTracks();
    resetHumanTracks();
    resetAllMetrics();
    rawCount = filteredCount = 0;

    GroundTruth gt[HUMAN_COUNT];

    for (int f = 0; f < FRAMES_PER_RUN; ++f) {
        rawCount = generateFrame(kind, seed, f, dropP);
        for (int h = 0; h < HUMAN_COUNT; ++h) {
            const TrajSample tr = humanTrajectory(kind, h, f * DT);
            gt[h].x = tr.p.x; gt[h].y = tr.p.y; gt[h].z = tr.p.z;
        }

        int cRaw = 0, dRaw = 0, cFilt = 0, dFilt = 0;
        uint64_t aUs = 0, blUs = 0, bUs = 0;
        processFrame(cRaw, dRaw, cFilt, dFilt, aUs, blUs, bUs);

        const uint64_t bEndToEnd = blUs + bUs;

        // Latency
        s_mA.totalUs    += aUs;
        s_mBase.totalUs += blUs;
        s_mB.totalUs    += bEndToEnd;
        if (aUs > s_mA.maxUs)        s_mA.maxUs    = aUs;
        if (blUs > s_mBase.maxUs)    s_mBase.maxUs = blUs;
        if (bEndToEnd > s_mB.maxUs)  s_mB.maxUs    = bEndToEnd;
        if (aUs > 100000ULL)         ++s_mA.overruns;
        if (blUs > 100000ULL)        ++s_mBase.overruns;
        if (bEndToEnd > 100000ULL)   ++s_mB.overruns;
        if (s_mA.latCount < FRAMES_PER_RUN) {
            s_mA.latUs[s_mA.latCount]       = (uint32_t)aUs;
            s_mBase.latUs[s_mBase.latCount] = (uint32_t)blUs;
            s_mB.latUs[s_mB.latCount]       = (uint32_t)bEndToEnd;
            ++s_mA.latCount;
            ++s_mBase.latCount;
            ++s_mB.latCount;
        }

        bumpCoverage(s_mA,    dFilt);
        bumpCoverage(s_mBase, dRaw);
        bumpCoverage(s_mB,    countConfirmedTracks());

        if (f >= WARMUP_FRAMES) {
            Point3D aOut[MAX_POINTS], blOut[MAX_POINTS];
            for (int i = 0; i < dFilt; ++i) aOut[i]  = filteredCentroids[i].p;
            for (int i = 0; i < dRaw;  ++i) blOut[i] = centroids[i].p;
            scoreOutputs(aOut,  dFilt, gt, s_mA);
            scoreOutputs(blOut, dRaw,  gt, s_mBase);
            scoreTracks(humanTracks, humanTrackCount, gt, s_mB);
        }
    }

    const int scoredFrames = FRAMES_PER_RUN - WARMUP_FRAMES;
    const int latencyFrames = s_mA.latCount;

    auto rmse = [](const Metrics &m) {
        return m.samples ? sqrtf((float)(m.sumErrSq / m.samples)) : 0.0f;
    };
    auto cov2 = [](const Metrics &m) {
        const uint32_t total = m.frames0 + m.frames1 + m.frames2 + m.frames3plus;
        return total ? (float)(m.frames2 + m.frames3plus) / total : 0.0f;
    };
    auto overrunPct = [](const Metrics &m) {
        const uint32_t total = m.frames0 + m.frames1 + m.frames2 + m.frames3plus;
        return total ? (float)m.overruns / total : 0.0f;
    };

    out.rmseA  = rmse(s_mA);
    out.rmseB  = rmse(s_mB);
    out.rmseBL = rmse(s_mBase);

    out.idSwitches = (float)s_mB.idSwitches;
    out.idEligible = (float)s_mB.idEligible;

    out.covA2  = cov2(s_mA);
    out.covB2  = cov2(s_mB);
    out.covBL2 = cov2(s_mBase);

    out.fpPerFrameA  = scoredFrames ? (float)s_mA.falsePositives    / scoredFrames : 0.0f;
    out.fpPerFrameB  = scoredFrames ? (float)s_mB.falsePositives    / scoredFrames : 0.0f;
    out.fpPerFrameBL = scoredFrames ? (float)s_mBase.falsePositives / scoredFrames : 0.0f;

    out.missPerFrameA  = scoredFrames ? (float)s_mA.misses    / scoredFrames : 0.0f;
    out.missPerFrameB  = scoredFrames ? (float)s_mB.misses    / scoredFrames : 0.0f;
    out.missPerFrameBL = scoredFrames ? (float)s_mBase.misses / scoredFrames : 0.0f;

    out.overrunPctA  = overrunPct(s_mA);
    out.overrunPctBL = overrunPct(s_mBase);
    out.overrunPctB  = overrunPct(s_mB);

    out.p95UsA  = computeP95Us(s_mA.latUs,    latencyFrames);
    out.p95UsBL = computeP95Us(s_mBase.latUs, latencyFrames);
    out.p95UsB  = computeP95Us(s_mB.latUs,    latencyFrames);
    out.meanUsA  = latencyFrames ? (float)s_mA.totalUs    / latencyFrames : 0.0f;
    out.meanUsBL = latencyFrames ? (float)s_mBase.totalUs / latencyFrames : 0.0f;
    out.meanUsB  = latencyFrames ? (float)s_mB.totalUs    / latencyFrames : 0.0f;
    out.maxUsA  = (float)s_mA.maxUs;
    out.maxUsBL = (float)s_mBase.maxUs;
    out.maxUsB  = (float)s_mB.maxUs;
}

struct Stat { float mean = 0, std = 0; };
static Stat meanStd(const float *v, int n) {
    Stat s{};
    if (n <= 0) return s;
    double sum = 0, sumSq = 0;
    for (int i = 0; i < n; ++i) { sum += v[i]; sumSq += (double)v[i] * v[i]; }
    s.mean = (float)(sum / n);
    const double var = (sumSq / n) - (double)s.mean * s.mean;
    s.std = (float)sqrtf(var > 0 ? (float)var : 0.0f);
    return s;
}

static void printScenarioReport(const char *name, int frames,
                                float sigma, float dropP,
                                const SeedOutcome *runs, int n) {
    float v[MAX_SEEDS];
    auto pull = [&](float (SeedOutcome::*member)) {
        for (int i = 0; i < n; ++i) v[i] = runs[i].*member;
        return meanStd(v, n);
    };

    const Stat sA  = pull(&SeedOutcome::rmseA);
    const Stat sB  = pull(&SeedOutcome::rmseB);
    const Stat sBL = pull(&SeedOutcome::rmseBL);
    const Stat sID = pull(&SeedOutcome::idSwitches);
    const Stat sIDE= pull(&SeedOutcome::idEligible);
    const Stat sCA = pull(&SeedOutcome::covA2);
    const Stat sCB = pull(&SeedOutcome::covB2);
    const Stat sCBL= pull(&SeedOutcome::covBL2);
    const Stat sFpA  = pull(&SeedOutcome::fpPerFrameA);
    const Stat sFpB  = pull(&SeedOutcome::fpPerFrameB);
    const Stat sFpBL = pull(&SeedOutcome::fpPerFrameBL);
    const Stat sMsA  = pull(&SeedOutcome::missPerFrameA);
    const Stat sMsB  = pull(&SeedOutcome::missPerFrameB);
    const Stat sMsBL = pull(&SeedOutcome::missPerFrameBL);
    const Stat sOvA  = pull(&SeedOutcome::overrunPctA);
    const Stat sOvB  = pull(&SeedOutcome::overrunPctB);
    const Stat sOvBL = pull(&SeedOutcome::overrunPctBL);
    const Stat sP95A = pull(&SeedOutcome::p95UsA);
    const Stat sP95B = pull(&SeedOutcome::p95UsB);
    const Stat sP95BL= pull(&SeedOutcome::p95UsBL);
    const Stat sMnA  = pull(&SeedOutcome::meanUsA);
    const Stat sMnB  = pull(&SeedOutcome::meanUsB);
    const Stat sMnBL = pull(&SeedOutcome::meanUsBL);
    const Stat sMxA  = pull(&SeedOutcome::maxUsA);
    const Stat sMxB  = pull(&SeedOutcome::maxUsB);
    const Stat sMxBL = pull(&SeedOutcome::maxUsBL);

    const double improvementA = (sBL.mean > 1e-9)
        ? 100.0 * (sBL.mean - sA.mean) / sBL.mean : 0.0;

    Serial.println();
    Serial.printf("SCENARIO: %s  frames=%d  sigma=%.3f  dropout=%.2f  seeds=%d\n",
                  name, frames, sigma, dropP, n);
    Serial.println("  Position (unconditional RMSE with miss penalty, m, mean ± pop.std):");
    Serial.printf("    A  Kalman->DBSCAN   : %.5f ± %.5f\n", sA.mean, sA.std);
    Serial.printf("    B  DBSCAN->Kalman   : %.5f ± %.5f\n", sB.mean, sB.std);
    Serial.printf("    BL raw centroids     : %.5f ± %.5f\n", sBL.mean, sBL.std);
    Serial.printf("  A vs BL RMSE change    : %+.2f %%  (positive = A better)\n", improvementA);
    Serial.println("  Coverage (frames with >= 2 outputs, %):");
    Serial.printf("    A  : %.1f ± %.1f     B  : %.1f ± %.1f     BL : %.1f ± %.1f\n",
                  100.0 * sCA.mean,  100.0 * sCA.std,
                  100.0 * sCB.mean,  100.0 * sCB.std,
                  100.0 * sCBL.mean, 100.0 * sCBL.std);
    Serial.println("  Misses per scored frame (2 GT targets):");
    Serial.printf("    A  : %.3f ± %.3f     B  : %.3f ± %.3f     BL : %.3f ± %.3f\n",
                  sMsA.mean,  sMsA.std,
                  sMsB.mean,  sMsB.std,
                  sMsBL.mean, sMsBL.std);
    Serial.println("  False positives per scored frame:");
    Serial.printf("    A  : %.3f ± %.3f     B  : %.3f ± %.3f     BL : %.3f ± %.3f\n",
                  sFpA.mean,  sFpA.std,
                  sFpB.mean,  sFpB.std,
                  sFpBL.mean, sFpBL.std);
    Serial.printf("  Identity (B): switches %.2f ± %.2f   eligible %.1f ± %.1f\n",
                  sID.mean, sID.std, sIDE.mean, sIDE.std);
    Serial.println("  Latency per frame (mean / p95 / max, ms):");
    Serial.printf("    A  : %.2f / %.2f / %.2f\n",
                  sMnA.mean / 1000.0, sP95A.mean / 1000.0, sMxA.mean / 1000.0);
    Serial.printf("    BL : %.2f / %.2f / %.2f\n",
                  sMnBL.mean / 1000.0, sP95BL.mean / 1000.0, sMxBL.mean / 1000.0);
    Serial.printf("    B  : %.2f / %.2f / %.2f  (BL + tracking, end-to-end)\n",
                  sMnB.mean / 1000.0, sP95B.mean / 1000.0, sMxB.mean / 1000.0);
    Serial.printf("  Overrun %% (>100 ms): A %.2f  BL %.2f  B %.2f\n",
                  100.0 * sOvA.mean, 100.0 * sOvBL.mean, 100.0 * sOvB.mean);
    Serial.println("  ----------------------------------------------------------------");
}

// ============================================================================
// Scenario table
// ============================================================================
struct ScenarioDef {
    const char *name;
    ScenarioKind kind;
    float sigma;
    float dropP;
};

static ScenarioDef scenarios[] = {
    { "12A  WALK  sigma=0.05",       SCEN_WALK,     0.05f, 0.00f },
    { "12B  CROSSING  sigma=0.05",   SCEN_CROSSING, 0.05f, 0.00f },
    { "12C  MERGE  sigma=0.05",      SCEN_MERGE,    0.05f, 0.00f },
    { "12D1 WALK sigma=0.02",        SCEN_WALK,     0.02f, 0.00f },
    { "12D2 WALK sigma=0.10",        SCEN_WALK,     0.10f, 0.00f },
    { "12D3 WALK sigma=0.20",        SCEN_WALK,     0.20f, 0.00f },
    { "12E1 WALK drop=0.10",         SCEN_WALK,     0.05f, 0.10f },
    { "12E2 WALK drop=0.30",         SCEN_WALK,     0.05f, 0.30f },
    { "12E3 WALK drop=0.50",         SCEN_WALK,     0.05f, 0.50f },
};
static constexpr int SCENARIO_COUNT =
    sizeof(scenarios) / sizeof(scenarios[0]);

// ============================================================================
// setup / loop
// ============================================================================
void setup() {
    pinMode(2, OUTPUT);
    for (int i = 0; i < 3; ++i) {
        digitalWrite(2, HIGH); delay(120);
        digitalWrite(2, LOW);  delay(120);
    }

    delay(3000);
    Serial.begin(115200);
    const uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < 2000) delay(10);
    delay(500);

    Serial.println();
    Serial.println("=============================================");
    Serial.println("EXPERIMENT 12 — second audit revision");
    Serial.println("A : Kalman->DBSCAN   B : DBSCAN->Kalman   BL : raw");
    Serial.println("=============================================");
    Serial.printf("CPU: %u MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("Heap free: %u B   PSRAM free: %u B\n",
                  (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getFreePsram());
    Serial.printf("Config: sensors=%d humans=%d pts/human/sensor=%d\n",
                  SENSOR_COUNT, HUMAN_COUNT, PTS_PER_HUMAN_SENSOR);
    Serial.printf("        eps=%.2f minPts=%d  sigma_default=%.3f m  DT=%.2f s\n",
                  EPSILON, MIN_PTS, MEAS_SIGMA, DT);
    Serial.printf("        Q_human_psd=%.4f  Q_point_psd=%.5f (m^2/s^3)\n",
                  Q_HUMAN_PSD, Q_POINT_PSD);
    Serial.printf("        frames/scenario=%d  warmup=%d  seeds=%d\n",
                  FRAMES_PER_RUN, WARMUP_FRAMES, N_SEEDS);
    Serial.printf("        assoc gate=%.2f m  miss penalty=%.4f m^2\n",
                  ASSOC_GATE, (double)MISS_PENALTY_SQ);
    Serial.printf("        Kalman uses true sigma: %s\n",
                  KALMAN_KNOWS_TRUE_SIGMA ? "yes" : "no (mis-specified)");
    Serial.flush();

    static SeedOutcome outcomes[MAX_SEEDS];

    const int nSeeds = (N_SEEDS < MAX_SEEDS) ? N_SEEDS : MAX_SEEDS;

    for (int si = 0; si < SCENARIO_COUNT; ++si) {
        const ScenarioDef &sd = scenarios[si];
        for (int seed = 0; seed < nSeeds; ++seed) {
            runOneSeed(sd.kind, seed, sd.sigma, sd.dropP, outcomes[seed]);
            delay(0);
        }
        printScenarioReport(sd.name, FRAMES_PER_RUN, sd.sigma, sd.dropP,
                            outcomes, nSeeds);
    }

    Serial.println();
    Serial.println("Experiment 12 completed.");
    Serial.flush();
    for (int i = 0; i < 5; ++i) {
        digitalWrite(2, HIGH); delay(120);
        digitalWrite(2, LOW);  delay(120);
    }
}

void loop() { delay(1000); }