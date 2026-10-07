#include <Arduino.h>
#include <math.h>
#include "esp_timer.h"

// ============================================================
// Experiment 07B
// Real-Time Limit Search with Correct 3-Sensor Geometry
//
// This version fixes the previous simulator's geometry model.
//
// Each human is generated ONCE in a shared global 3D coordinate
// system. Every sensor observes that same human from its own
// pose, producing a local measurement. Sensor noise is added in
// local coordinates, and the measurement is transformed back to
// the global coordinate system before DBSCAN.
//
// Therefore, the three sensors should contribute points to the
// SAME human cluster instead of creating one cluster per sensor.
//
// Pipeline:
//
// Global human scene
//      |
//      +--> Sensor 1: world -> local -> noisy measurement
//      |
//      +--> Sensor 2: world -> local -> noisy measurement
//      |
//      +--> Sensor 3: world -> local -> noisy measurement
//                         |
//                    Sensor fusion
//                         |
//                  Spatial Grid
//                         |
//                       DBSCAN
//                         |
//                Human-like analysis
//
// No physical UART hardware is required yet. The sensor
// acquisition layer is intentionally isolated so it can later be
// replaced by HardwareSerial input from three real antennas.
// ============================================================


// ============================================================
// Configuration
// ============================================================

#define NUM_SENSORS 3
#define NUM_HUMANS 2

// Enough room for approximately 400 points/human/sensor:
// 2 humans * 3 sensors * 400 + 36 noise = 2436 points.
#define MAX_POINTS 4096
#define GRID_BUCKETS 4096

const float TARGET_FPS = 10.0f;
const float FRAME_BUDGET_MS = 1000.0f / TARGET_FPS;

const float EPSILON = 0.35f;
const float EPSILON2 = EPSILON * EPSILON;
const int MIN_PTS = 5;

const float SENSOR_NOISE = 0.025f;
const float PI_F = 3.14159265358979323846f;
const float DEG_TO_RAD_F = PI_F / 180.0f;
const int NOISE_POINTS_PER_SENSOR = 12;

// Search configuration.
const int SEARCH_START = 48;
const int SEARCH_COARSE_END = 320;
const int SEARCH_COARSE_STEP = 16;
const int SEARCH_EXTENDED_END = 400;
const int SEARCH_FINE_STEP = 4;

// Fewer frames during search; final candidate uses more frames.
const int SEARCH_FRAMES = 4;
const int FINAL_FRAMES = 12;

// Expected result of this controlled simulation.
const int EXPECTED_HUMANS = NUM_HUMANS;


// ============================================================
// Data structures
// ============================================================

struct Point3D
{
    float x;
    float y;
    float z;
};

struct SensorPose
{
    float x;
    float y;
    float z;
    float yaw;
};

struct HumanState
{
    float x;
    float y;
    float z;
    float vx;
    float vy;
};

struct ClusterInfo
{
    int count;

    float cx;
    float cy;
    float cz;

    float minX;
    float maxX;
    float minY;
    float maxY;
    float minZ;
    float maxZ;
};

struct RunResult
{
    int pointsPerHumanSensor;
    int pointsPerFrame;

    float averageIngestMs;
    float averageDbscanMs;
    float averageHumanMs;
    float averageTotalMs;

    float maximumTotalMs;

    float estimatedMaxFPS;
    float throughput;

    int frameOverruns;

    int averageHumanLike;
    int minHumanLike;
    int maxHumanLike;

    bool realtime;
    bool humanDetectionCorrect;
};


// ============================================================
// Global buffers
// ============================================================

Point3D *points = nullptr;
int *labels = nullptr;
int *queueBuffer = nullptr;

int *gridHead = nullptr;
int *gridNext = nullptr;

int *cellX = nullptr;
int *cellY = nullptr;
int *cellZ = nullptr;


// ============================================================
// Three sensor poses
//
// They observe the same room from different positions and yaw
// orientations. The exact geometry is known to the simulator.
// ============================================================

SensorPose sensors[NUM_SENSORS] =
{
    {
        -2.5f,
        0.0f,
        1.0f,
        15.0f * DEG_TO_RAD_F
    },
    {
        0.0f,
        0.0f,
        1.0f,
        0.0f
    },
    {
        2.5f,
        0.0f,
        1.0f,
        -15.0f * DEG_TO_RAD_F
    }
};


// ============================================================
// Human states in GLOBAL coordinates
// ============================================================

HumanState humans[NUM_HUMANS] =
{
    {
        0.0f,
        0.8f,
        0.0f,
        0.08f,
        0.025f
    },
    {
        2.0f,
        -1.2f,
        0.0f,
        -0.06f,
        0.035f
    }
};

uint32_t currentFrame = 0;


// ============================================================
// Small deterministic hash/random helpers
//
// Every sensor can reproduce the same underlying human point,
// while still receiving independent measurement noise.
// ============================================================

uint32_t hash32(uint32_t x)
{
    x ^= x >> 16;
    x *= 0x7feb352dU;
    x ^= x >> 15;
    x *= 0x846ca68bU;
    x ^= x >> 16;
    return x;
}


float randomUnit(uint32_t seed)
{
    uint32_t h = hash32(seed);
    return (float)(h % 1000000U) / 1000000.0f;
}


float randomRange(
    uint32_t seed,
    float minValue,
    float maxValue
)
{
    return minValue +
           randomUnit(seed) *
           (maxValue - minValue);
}


// ============================================================
// Squared Euclidean distance
// ============================================================

inline float squaredDistance(
    const Point3D &a,
    const Point3D &b
)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;

    return dx * dx +
           dy * dy +
           dz * dz;
}


// ============================================================
// Human movement
// ============================================================

void updateHumans()
{
    for (int i = 0; i < NUM_HUMANS; i++)
    {
        humans[i].x += humans[i].vx;
        humans[i].y += humans[i].vy;

        if (humans[i].x > 3.0f || humans[i].x < -1.0f)
        {
            humans[i].vx *= -1.0f;
        }

        if (humans[i].y > 2.0f || humans[i].y < -2.0f)
        {
            humans[i].vy *= -1.0f;
        }
    }
}


// ============================================================
// GLOBAL -> SENSOR LOCAL
// Inverse of sensorToWorld().
// ============================================================

Point3D worldToSensor(
    const Point3D &world,
    const SensorPose &sensor
)
{
    float dx = world.x - sensor.x;
    float dy = world.y - sensor.y;
    float dz = world.z - sensor.z;

    float c = cosf(sensor.yaw);
    float s = sinf(sensor.yaw);

    Point3D local;

    // R(-yaw) * translation-adjusted world point.
    local.x = dx * c + dy * s;
    local.y = -dx * s + dy * c;
    local.z = dz;

    return local;
}


// ============================================================
// SENSOR LOCAL -> GLOBAL
// ============================================================

Point3D sensorToWorld(
    const Point3D &local,
    const SensorPose &sensor
)
{
    float c = cosf(sensor.yaw);
    float s = sinf(sensor.yaw);

    Point3D world;

    world.x =
        sensor.x +
        local.x * c -
        local.y * s;

    world.y =
        sensor.y +
        local.x * s +
        local.y * c;

    world.z =
        sensor.z +
        local.z;

    return world;
}


// ============================================================
// Generate ONE canonical human point in GLOBAL coordinates.
// ============================================================

Point3D makeHumanWorldPoint(
    int humanIndex,
    int pointIndex,
    int pointsPerHumanSensor
)
{
    (void)pointsPerHumanSensor;

    const HumanState &human = humans[humanIndex];

    // Deterministic sample-specific random values.
    uint32_t baseSeed =
        0x13579BDFU +
        currentFrame * 0x9E3779B9U +
        (uint32_t)humanIndex * 0x10203040U +
        (uint32_t)pointIndex * 0x55667788U;

    int section = pointIndex % 100;

    float px = human.x;
    float py = human.y;
    float pz = 0.9f;

    if (section < 50)
    {
        // Torso.
        px += randomRange(baseSeed + 1, -0.22f, 0.22f);
        py += randomRange(baseSeed + 2, -0.14f, 0.14f);
        pz = randomRange(baseSeed + 3, 0.55f, 1.45f);
    }
    else if (section < 65)
    {
        // Head.
        px += randomRange(baseSeed + 4, -0.16f, 0.16f);
        py += randomRange(baseSeed + 5, -0.16f, 0.16f);
        pz = randomRange(baseSeed + 6, 1.45f, 1.75f);
    }
    else if (section < 82)
    {
        // Arms.
        float side =
            ((pointIndex & 1) == 0)
                ? -1.0f
                : 1.0f;

        px +=
            randomRange(baseSeed + 7, 0.25f, 0.50f) * side;

        py += randomRange(baseSeed + 8, -0.10f, 0.10f);
        pz = randomRange(baseSeed + 9, 0.75f, 1.40f);
    }
    else
    {
        // Legs.
        float side =
            ((pointIndex & 1) == 0)
                ? -1.0f
                : 1.0f;

        px +=
            randomRange(baseSeed + 10, 0.08f, 0.18f) * side;

        py += randomRange(baseSeed + 11, -0.10f, 0.10f);
        pz = randomRange(baseSeed + 12, 0.05f, 0.70f);
    }

    Point3D world =
    {
        px,
        py,
        pz
    };

    return world;
}


// ============================================================
// Simulate one sensor measurement of a GLOBAL point.
//
// This is the critical correction compared with the previous
// simulator:
//
//     same global target point
//          -> each sensor's local coordinates
//          -> sensor measurement noise
//          -> reconstructed global coordinates
//
// Therefore all three sensors contribute to the SAME human.
// ============================================================

Point3D simulateSensorMeasurement(
    const Point3D &worldPoint,
    int sensorIndex,
    int humanIndex,
    int pointIndex
)
{
    const SensorPose &sensor =
        sensors[sensorIndex];

    Point3D local =
        worldToSensor(
            worldPoint,
            sensor
        );

    uint32_t noiseSeed =
        0xABCDEF01U +
        currentFrame * 0x45D9F3BU +
        (uint32_t)sensorIndex * 0x01010101U +
        (uint32_t)humanIndex * 0x11111111U +
        (uint32_t)pointIndex * 0x22222222U;

    // Sensor-domain measurement noise.
    local.x +=
        randomRange(
            noiseSeed + 1,
            -SENSOR_NOISE,
            SENSOR_NOISE
        );

    local.y +=
        randomRange(
            noiseSeed + 2,
            -SENSOR_NOISE,
            SENSOR_NOISE
        );

    local.z +=
        randomRange(
            noiseSeed + 3,
            -SENSOR_NOISE,
            SENSOR_NOISE
        );

    return sensorToWorld(
        local,
        sensor
    );
}


// ============================================================
// Simulated environmental clutter point.
//
// Clutter is generated in the GLOBAL room, then measured by each
// sensor using the exact same transformation pipeline.
// ============================================================

Point3D makeNoiseWorldPoint(
    int sensorIndex,
    int noiseIndex
)
{
    uint32_t seed =
        0x2468ACE1U +
        currentFrame * 0x27D4EB2DU +
        (uint32_t)sensorIndex * 0x13572468U +
        (uint32_t)noiseIndex * 0x31415926U;

    Point3D world =
    {
        randomRange(seed + 1, -3.0f, 4.0f),
        randomRange(seed + 2, -2.5f, 2.5f),
        randomRange(seed + 3, 0.05f, 1.8f)
    };

    return world;
}


// ============================================================
// Simulated 3-sensor acquisition / fusion
// ============================================================

int generateFrame(
    int pointsPerHumanSensor
)
{
    int pointCount = 0;

    // --------------------------------------------------------
    // All three sensors observe the same humans.
    // --------------------------------------------------------

    for (int sensor = 0; sensor < NUM_SENSORS; sensor++)
    {
        for (int human = 0; human < NUM_HUMANS; human++)
        {
            for (int p = 0;
                 p < pointsPerHumanSensor;
                 p++)
            {
                if (pointCount >= MAX_POINTS)
                {
                    break;
                }

                Point3D worldPoint =
                    makeHumanWorldPoint(
                        human,
                        p,
                        pointsPerHumanSensor
                    );

                Point3D measured =
                    simulateSensorMeasurement(
                        worldPoint,
                        sensor,
                        human,
                        p
                    );

                points[pointCount++] = measured;
            }
        }

        // ----------------------------------------------------
        // Background clutter.
        // ----------------------------------------------------

        for (int n = 0;
             n < NOISE_POINTS_PER_SENSOR;
             n++)
        {
            if (pointCount >= MAX_POINTS)
            {
                break;
            }

            Point3D worldNoise =
                makeNoiseWorldPoint(
                    sensor,
                    n
                );

            Point3D measuredNoise =
                simulateSensorMeasurement(
                    worldNoise,
                    sensor,
                    sensor,
                    n
                );

            points[pointCount++] = measuredNoise;
        }
    }

    return pointCount;
}


// ============================================================
// Spatial grid
// ============================================================

inline uint32_t gridHash(
    int x,
    int y,
    int z
)
{
    uint32_t h =
        (uint32_t)(x * 73856093);

    h ^=
        (uint32_t)(y * 19349663);

    h ^=
        (uint32_t)(z * 83492791);

    return h % GRID_BUCKETS;
}


inline int getCellCoordinate(
    float value
)
{
    return (int)floorf(
        value / EPSILON
    );
}


void clearGrid()
{
    for (int i = 0;
         i < GRID_BUCKETS;
         i++)
    {
        gridHead[i] = -1;
    }
}


void buildGrid(int N)
{
    clearGrid();

    for (int i = 0;
         i < N;
         i++)
    {
        int x =
            getCellCoordinate(points[i].x);

        int y =
            getCellCoordinate(points[i].y);

        int z =
            getCellCoordinate(points[i].z);

        cellX[i] = x;
        cellY[i] = y;
        cellZ[i] = z;

        uint32_t hash =
            gridHash(x, y, z);

        gridNext[i] =
            gridHead[hash];

        gridHead[hash] = i;
    }
}


// ============================================================
// Grid neighbor count
// ============================================================

int gridCountNeighbors(
    int index
)
{
    int cx = cellX[index];
    int cy = cellY[index];
    int cz = cellZ[index];

    int count = 0;

    const Point3D &query =
        points[index];

    for (int dx = -1;
         dx <= 1;
         dx++)
    {
        for (int dy = -1;
             dy <= 1;
             dy++)
        {
            for (int dz = -1;
                 dz <= 1;
                 dz++)
            {
                int nx = cx + dx;
                int ny = cy + dy;
                int nz = cz + dz;

                uint32_t hash =
                    gridHash(nx, ny, nz);

                int current =
                    gridHead[hash];

                while (current != -1)
                {
                    if (
                        cellX[current] == nx &&
                        cellY[current] == ny &&
                        cellZ[current] == nz
                    )
                    {
                        if (
                            squaredDistance(
                                query,
                                points[current]
                            ) <= EPSILON2
                        )
                        {
                            count++;

                            if (count >= MIN_PTS)
                            {
                                return count;
                            }
                        }
                    }

                    current =
                        gridNext[current];
                }
            }
        }
    }

    return count;
}


// ============================================================
// Add neighbors to DBSCAN queue
// ============================================================

void addNeighborsToQueue(
    int index,
    int clusterId,
    int N,
    int &queueSize
)
{
    (void)clusterId;

    int cx = cellX[index];
    int cy = cellY[index];
    int cz = cellZ[index];

    const Point3D &query =
        points[index];

    for (int dx = -1;
         dx <= 1;
         dx++)
    {
        for (int dy = -1;
             dy <= 1;
             dy++)
        {
            for (int dz = -1;
                 dz <= 1;
                 dz++)
            {
                int nx = cx + dx;
                int ny = cy + dy;
                int nz = cz + dz;

                uint32_t hash =
                    gridHash(nx, ny, nz);

                int current =
                    gridHead[hash];

                while (current != -1)
                {
                    if (
                        cellX[current] == nx &&
                        cellY[current] == ny &&
                        cellZ[current] == nz
                    )
                    {
                        if (
                            squaredDistance(
                                query,
                                points[current]
                            ) <= EPSILON2
                        )
                        {
                            if (labels[current] == 0)
                            {
                                labels[current] = clusterId;

                                if (queueSize < N)
                                {
                                    queueBuffer[
                                        queueSize++
                                    ] = current;
                                }
                            }
                        }
                    }

                    current =
                        gridNext[current];
                }
            }
        }
    }
}


// ============================================================
// Spatial-grid DBSCAN
// ============================================================

int runDBSCAN(int N)
{
    for (int i = 0;
         i < N;
         i++)
    {
        labels[i] = 0;
    }

    int clusterId = 0;

    for (int i = 0;
         i < N;
         i++)
    {
        if (labels[i] != 0)
        {
            continue;
        }

        int neighborCount =
            gridCountNeighbors(i);

        if (neighborCount < MIN_PTS)
        {
            labels[i] = -2;
            continue;
        }

        clusterId++;

        int queueSize = 0;
        int queuePosition = 0;

        labels[i] = clusterId;

        addNeighborsToQueue(
            i,
            clusterId,
            N,
            queueSize
        );

        while (queuePosition < queueSize)
        {
            int current =
                queueBuffer[queuePosition++];

            if (
                gridCountNeighbors(current) >= MIN_PTS
            )
            {
                addNeighborsToQueue(
                    current,
                    clusterId,
                    N,
                    queueSize
                );
            }
        }
    }

    return clusterId;
}


// ============================================================
// Human-like cluster analysis
// ============================================================

int analyzeHumanClusters(
    int N,
    int clusterCount,
    bool printClusters
)
{
    if (clusterCount > 16)
    {
        clusterCount = 16;
    }

    ClusterInfo clusters[16];

    for (int c = 0;
         c < clusterCount;
         c++)
    {
        clusters[c].count = 0;

        clusters[c].cx = 0.0f;
        clusters[c].cy = 0.0f;
        clusters[c].cz = 0.0f;

        clusters[c].minX = 9999.0f;
        clusters[c].maxX = -9999.0f;
        clusters[c].minY = 9999.0f;
        clusters[c].maxY = -9999.0f;
        clusters[c].minZ = 9999.0f;
        clusters[c].maxZ = -9999.0f;
    }

    for (int i = 0;
         i < N;
         i++)
    {
        int label = labels[i];

        if (label <= 0)
        {
            continue;
        }

        int c = label - 1;

        if (c < 0 || c >= clusterCount)
        {
            continue;
        }

        ClusterInfo &cluster =
            clusters[c];

        cluster.count++;

        cluster.cx += points[i].x;
        cluster.cy += points[i].y;
        cluster.cz += points[i].z;

        if (points[i].x < cluster.minX)
            cluster.minX = points[i].x;

        if (points[i].x > cluster.maxX)
            cluster.maxX = points[i].x;

        if (points[i].y < cluster.minY)
            cluster.minY = points[i].y;

        if (points[i].y > cluster.maxY)
            cluster.maxY = points[i].y;

        if (points[i].z < cluster.minZ)
            cluster.minZ = points[i].z;

        if (points[i].z > cluster.maxZ)
            cluster.maxZ = points[i].z;
    }

    int humanLike = 0;

    for (int c = 0;
         c < clusterCount;
         c++)
    {
        ClusterInfo &cluster =
            clusters[c];

        if (cluster.count == 0)
        {
            continue;
        }

        cluster.cx /= cluster.count;
        cluster.cy /= cluster.count;
        cluster.cz /= cluster.count;

        float height =
            cluster.maxZ - cluster.minZ;

        float width =
            cluster.maxX - cluster.minX;

        float depth =
            cluster.maxY - cluster.minY;

        float footprint =
            sqrtf(
                width * width +
                depth * depth
            );

        // Controlled simulation heuristic.
        bool human =
            cluster.count >= 20 &&
            height >= 0.80f &&
            height <= 2.50f &&
            footprint <= 2.50f;

        if (human)
        {
            humanLike++;
        }

        if (printClusters)
        {
            Serial.printf(
                "  C%d: n=%d centroid=(%.2f, %.2f, %.2f) "
                "height=%.2f footprint=%.2f %s\n",
                c + 1,
                cluster.count,
                cluster.cx,
                cluster.cy,
                cluster.cz,
                height,
                footprint,
                human ? "HUMAN-LIKE" : "other"
            );
        }
    }

    return humanLike;
}


// ============================================================
// Load test
// ============================================================

RunResult runLoadTest(
    int pointsPerHumanSensor,
    int frames,
    bool printFrames
)
{
    RunResult result{};

    result.pointsPerHumanSensor =
        pointsPerHumanSensor;

    result.pointsPerFrame =
        NUM_HUMANS *
        NUM_SENSORS *
        pointsPerHumanSensor +
        NUM_SENSORS *
        NOISE_POINTS_PER_SENSOR;

    float sumIngest = 0.0f;
    float sumDbscan = 0.0f;
    float sumHuman = 0.0f;
    float sumTotal = 0.0f;

    float maxTotal = 0.0f;

    int totalHumanLike = 0;
    int minHumanLike = 999;
    int maxHumanLike = -999;

    for (int frame = 0;
         frame < frames;
         frame++)
    {
        currentFrame++;

        updateHumans();

        uint64_t frameStart =
            esp_timer_get_time();

        // ----------------------------------------------------
        // Sensor acquisition + fusion
        // ----------------------------------------------------

        uint64_t ingestStart =
            esp_timer_get_time();

        int pointCount =
            generateFrame(
                pointsPerHumanSensor
            );

        uint64_t ingestEnd =
            esp_timer_get_time();

        // ----------------------------------------------------
        // Spatial Grid + DBSCAN
        // ----------------------------------------------------

        uint64_t dbStart =
            esp_timer_get_time();

        buildGrid(pointCount);

        int clusterCount =
            runDBSCAN(pointCount);

        uint64_t dbEnd =
            esp_timer_get_time();

        // ----------------------------------------------------
        // Human analysis
        // ----------------------------------------------------

        uint64_t humanStart =
            esp_timer_get_time();

        bool printClusters =
            printFrames && frame == 0;

        int humanLike =
            analyzeHumanClusters(
                pointCount,
                clusterCount,
                printClusters
            );

        uint64_t humanEnd =
            esp_timer_get_time();

        uint64_t frameEnd =
            esp_timer_get_time();

        float ingestMs =
            (ingestEnd - ingestStart) /
            1000.0f;

        float dbscanMs =
            (dbEnd - dbStart) /
            1000.0f;

        float humanMs =
            (humanEnd - humanStart) /
            1000.0f;

        float totalMs =
            (frameEnd - frameStart) /
            1000.0f;

        sumIngest += ingestMs;
        sumDbscan += dbscanMs;
        sumHuman += humanMs;
        sumTotal += totalMs;

        if (totalMs > maxTotal)
        {
            maxTotal = totalMs;
        }

        totalHumanLike += humanLike;

        if (humanLike < minHumanLike)
            minHumanLike = humanLike;

        if (humanLike > maxHumanLike)
            maxHumanLike = humanLike;

        if (totalMs > FRAME_BUDGET_MS)
        {
            result.frameOverruns++;
        }

        if (printFrames)
        {
            Serial.printf(
                "Frame %d | points=%d | clusters=%d | "
                "human-like=%d | pipeline=%.3f ms\n",
                frame,
                pointCount,
                clusterCount,
                humanLike,
                totalMs
            );
        }
    }

    float frameCount =
        (float)frames;

    result.averageIngestMs =
        sumIngest / frameCount;

    result.averageDbscanMs =
        sumDbscan / frameCount;

    result.averageHumanMs =
        sumHuman / frameCount;

    result.averageTotalMs =
        sumTotal / frameCount;

    result.maximumTotalMs =
        maxTotal;

    result.estimatedMaxFPS =
        result.averageTotalMs > 0.0f
            ? 1000.0f / result.averageTotalMs
            : 0.0f;

    result.throughput =
        result.averageTotalMs > 0.0f
            ? result.pointsPerFrame *
              result.estimatedMaxFPS
            : 0.0f;

    result.averageHumanLike =
        (int)roundf(
            totalHumanLike / frameCount
        );

    result.minHumanLike =
        minHumanLike;

    result.maxHumanLike =
        maxHumanLike;

    result.realtime =
        result.maximumTotalMs <=
        FRAME_BUDGET_MS;

    result.humanDetectionCorrect =
        result.minHumanLike == EXPECTED_HUMANS &&
        result.maxHumanLike == EXPECTED_HUMANS;

    return result;
}


// ============================================================
// Print load result
// ============================================================

void printLoadResult(
    const RunResult &r
)
{
    Serial.println();
    Serial.println(
        "----------------------------------------------"
    );

    Serial.printf(
        "Points/human/sensor: %d\n",
        r.pointsPerHumanSensor
    );

    Serial.printf(
        "Points/frame: %d\n",
        r.pointsPerFrame
    );

    Serial.printf(
        "Input rate at %.1f Hz: %.1f points/s\n",
        TARGET_FPS,
        r.pointsPerFrame * TARGET_FPS
    );

    Serial.printf(
        "Average ingest/fusion: %.3f ms\n",
        r.averageIngestMs
    );

    Serial.printf(
        "Average grid+DBSCAN: %.3f ms\n",
        r.averageDbscanMs
    );

    Serial.printf(
        "Average human analysis: %.3f ms\n",
        r.averageHumanMs
    );

    Serial.printf(
        "Average total pipeline: %.3f ms\n",
        r.averageTotalMs
    );

    Serial.printf(
        "Maximum total pipeline: %.3f ms\n",
        r.maximumTotalMs
    );

    Serial.printf(
        "Estimated max FPS: %.2f\n",
        r.estimatedMaxFPS
    );

    Serial.printf(
        "Estimated throughput: %.1f points/s\n",
        r.throughput
    );

    Serial.printf(
        "Human-like clusters: avg=%d min=%d max=%d\n",
        r.averageHumanLike,
        r.minHumanLike,
        r.maxHumanLike
    );

    Serial.printf(
        "Target FPS: %.2f\n",
        TARGET_FPS
    );

    Serial.printf(
        "Frame budget: %.3f ms\n",
        FRAME_BUDGET_MS
    );

    Serial.printf(
        "Real-time at target: %s\n",
        r.realtime ? "YES" : "NO"
    );

    Serial.printf(
        "Human detection stable: %s\n",
        r.humanDetectionCorrect ? "YES" : "NO"
    );

    Serial.printf(
        "Frame overruns: %d / %d\n",
        r.frameOverruns,
        (r.pointsPerFrame > 0) ? FINAL_FRAMES : 0
    );

    Serial.println(
        "----------------------------------------------"
    );
}


// ============================================================
// Setup
// ============================================================

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println(
        "================================================"
    );
    Serial.println(
        "ESP32-S3 DBSCAN"
    );
    Serial.println(
        "Experiment 07B"
    );
    Serial.println(
        "Real-Time Limit Search"
    );
    Serial.println(
        "Correct 3-Sensor Geometry + Human Simulation"
    );
    Serial.println(
        "Spatial Grid + DBSCAN"
    );
    Serial.println(
        "================================================"
    );

    Serial.printf(
        "CPU frequency: %u MHz\n",
        getCpuFrequencyMhz()
    );

    Serial.printf(
        "Running on CPU: %d\n",
        xPortGetCoreID()
    );

    Serial.printf(
        "Free heap: %u bytes\n",
        ESP.getFreeHeap()
    );

    Serial.printf(
        "Free PSRAM: %u bytes\n",
        ESP.getFreePsram()
    );

    Serial.println();

    // ========================================================
    // Allocate buffers in PSRAM
    // ========================================================

    points =
        (Point3D *)ps_malloc(
            sizeof(Point3D) * MAX_POINTS
        );

    labels =
        (int *)ps_malloc(
            sizeof(int) * MAX_POINTS
        );

    queueBuffer =
        (int *)ps_malloc(
            sizeof(int) * MAX_POINTS
        );

    gridHead =
        (int *)ps_malloc(
            sizeof(int) * GRID_BUCKETS
        );

    gridNext =
        (int *)ps_malloc(
            sizeof(int) * MAX_POINTS
        );

    cellX =
        (int *)ps_malloc(
            sizeof(int) * MAX_POINTS
        );

    cellY =
        (int *)ps_malloc(
            sizeof(int) * MAX_POINTS
        );

    cellZ =
        (int *)ps_malloc(
            sizeof(int) * MAX_POINTS
        );

    if (
        !points ||
        !labels ||
        !queueBuffer ||
        !gridHead ||
        !gridNext ||
        !cellX ||
        !cellY ||
        !cellZ
    )
    {
        Serial.println(
            "ERROR: PSRAM allocation failed!"
        );

        while (true)
        {
            delay(1000);
        }
    }

    Serial.println(
        "PSRAM buffers allocated successfully."
    );

    Serial.println();
    Serial.println("Configuration:");

    Serial.printf(
        "Sensors: %d\n",
        NUM_SENSORS
    );

    Serial.printf(
        "Humans: %d\n",
        NUM_HUMANS
    );

    Serial.printf(
        "Target FPS: %.1f\n",
        TARGET_FPS
    );

    Serial.printf(
        "Frame budget: %.3f ms\n",
        FRAME_BUDGET_MS
    );

    Serial.printf(
        "Epsilon: %.2f\n",
        EPSILON
    );

    Serial.printf(
        "MinPts: %d\n",
        MIN_PTS
    );

    Serial.printf(
        "Grid buckets: %d\n",
        GRID_BUCKETS
    );

    Serial.printf(
        "Maximum points/frame: %d\n",
        MAX_POINTS
    );

    Serial.printf(
        "Sensor noise: %.3f\n",
        SENSOR_NOISE
    );

    Serial.printf(
        "Noise points/sensor: %d\n",
        NOISE_POINTS_PER_SENSOR
    );

    Serial.println();

    Serial.println(
        "Sensor geometry:"
    );

    for (int i = 0;
         i < NUM_SENSORS;
         i++)
    {
        Serial.printf(
            "  S%d: pos=(%.2f, %.2f, %.2f), yaw=%.1f deg\n",
            i + 1,
            sensors[i].x,
            sensors[i].y,
            sensors[i].z,
            sensors[i].yaw * 180.0f / PI_F
        );
    }


    // ========================================================
    // Phase 1: coarse search
    // ========================================================

    Serial.println();
    Serial.println(
        "================================================"
    );
    Serial.println(
        "PHASE 1 - COARSE LOAD SEARCH"
    );
    Serial.println(
        "================================================"
    );

    int lastPass = -1;
    int firstFail = -1;

    for (
        int p = SEARCH_START;
        p <= SEARCH_COARSE_END;
        p += SEARCH_COARSE_STEP
    )
    {
        Serial.printf(
            "Testing %d points/human/sensor...\n",
            p
        );

        RunResult result =
            runLoadTest(
                p,
                SEARCH_FRAMES,
                false
            );

        Serial.printf(
            "  points/frame: %d\n",
            result.pointsPerFrame
        );

        Serial.printf(
            "  avg pipeline: %.3f ms\n",
            result.averageTotalMs
        );

        Serial.printf(
            "  max pipeline: %.3f ms\n",
            result.maximumTotalMs
        );

        Serial.printf(
            "  human-like: %d..%d\n",
            result.minHumanLike,
            result.maxHumanLike
        );

        Serial.printf(
            "  realtime: %s\n",
            result.realtime ? "YES" : "NO"
        );

        Serial.printf(
            "  detection stable: %s\n",
            result.humanDetectionCorrect ? "YES" : "NO"
        );

        bool pass =
            result.realtime &&
            result.humanDetectionCorrect;

        if (pass)
        {
            lastPass = p;
        }
        else
        {
            firstFail = p;
            break;
        }
    }

    // ========================================================
    // Extend search if no failure was found.
    // ========================================================

    if (firstFail == -1)
    {
        for (
            int p = SEARCH_COARSE_END + SEARCH_COARSE_STEP;
            p <= SEARCH_EXTENDED_END;
            p += SEARCH_COARSE_STEP
        )
        {
            Serial.printf(
                "Testing extended %d points/human/sensor...\n",
                p
            );

            RunResult result =
                runLoadTest(
                    p,
                    SEARCH_FRAMES,
                    false
                );

            Serial.printf(
                "  points/frame: %d\n",
                result.pointsPerFrame
            );

            Serial.printf(
                "  avg pipeline: %.3f ms\n",
                result.averageTotalMs
            );

            Serial.printf(
                "  max pipeline: %.3f ms\n",
                result.maximumTotalMs
            );

            Serial.printf(
                "  human-like: %d..%d\n",
                result.minHumanLike,
                result.maxHumanLike
            );

            Serial.printf(
                "  realtime: %s\n",
                result.realtime ? "YES" : "NO"
            );

            Serial.printf(
                "  detection stable: %s\n",
                result.humanDetectionCorrect ? "YES" : "NO"
            );

            bool pass =
                result.realtime &&
                result.humanDetectionCorrect;

            if (pass)
            {
                lastPass = p;
            }
            else
            {
                firstFail = p;
                break;
            }
        }
    }


    // ========================================================
    // Phase 2: fine search
    // ========================================================

    Serial.println();
    Serial.println(
        "================================================"
    );
    Serial.println(
        "PHASE 2 - FINE LIMIT SEARCH"
    );
    Serial.println(
        "================================================"
    );

    if (lastPass == -1)
    {
        Serial.println(
            "WARNING: First tested load failed."
        );
    }
    else if (firstFail == -1)
    {
        Serial.println(
            "No failure found in configured search range."
        );
    }
    else
    {
        Serial.printf(
            "Fine searching between %d and %d...\n",
            lastPass,
            firstFail
        );

        for (
            int p = lastPass + SEARCH_FINE_STEP;
            p < firstFail;
            p += SEARCH_FINE_STEP
        )
        {
            Serial.printf(
                "Testing fine %d points/human/sensor...\n",
                p
            );

            RunResult result =
                runLoadTest(
                    p,
                    SEARCH_FRAMES,
                    false
                );

            Serial.printf(
                "  points/frame: %d | avg=%.3f ms | max=%.3f ms | "
                "human=%d..%d | realtime=%s | detection=%s\n",
                result.pointsPerFrame,
                result.averageTotalMs,
                result.maximumTotalMs,
                result.minHumanLike,
                result.maxHumanLike,
                result.realtime ? "YES" : "NO",
                result.humanDetectionCorrect ? "YES" : "NO"
            );

            bool pass =
                result.realtime &&
                result.humanDetectionCorrect;

            if (pass)
            {
                lastPass = p;
            }
            else
            {
                firstFail = p;
                break;
            }
        }
    }


    // ========================================================
    // Final candidate
    // ========================================================

    int finalCandidate =
        lastPass;

    if (finalCandidate < 0)
    {
        finalCandidate = SEARCH_START;
    }

    Serial.println();
    Serial.println(
        "================================================"
    );
    Serial.println(
        "FINAL VERIFICATION"
    );
    Serial.println(
        "================================================"
    );

    Serial.printf(
        "Final candidate: %d points/human/sensor\n",
        finalCandidate
    );

    RunResult finalResult =
        runLoadTest(
            finalCandidate,
            FINAL_FRAMES,
            true
        );

    printLoadResult(finalResult);


    // ========================================================
    // First failure verification
    // ========================================================

    if (firstFail != -1)
    {
        Serial.println();
        Serial.println(
            "First failing load verification:"
        );

        RunResult failResult =
            runLoadTest(
                firstFail,
                FINAL_FRAMES,
                false
            );

        printLoadResult(failResult);
    }


    // ========================================================
    // Final summary
    // ========================================================

    Serial.println();
    Serial.println(
        "================================================"
    );
    Serial.println(
        "EXPERIMENT 07B FINAL RESULT"
    );
    Serial.println(
        "================================================"
    );

    Serial.printf(
        "Maximum passing points/human/sensor: %d\n",
        finalCandidate
    );

    Serial.printf(
        "Maximum passing points/frame: %d\n",
        finalResult.pointsPerFrame
    );

    Serial.printf(
        "Maximum passing input rate at %.1f Hz: %.1f points/s\n",
        TARGET_FPS,
        finalResult.pointsPerFrame * TARGET_FPS
    );

    Serial.printf(
        "Average pipeline: %.3f ms\n",
        finalResult.averageTotalMs
    );

    Serial.printf(
        "Maximum pipeline: %.3f ms\n",
        finalResult.maximumTotalMs
    );

    Serial.printf(
        "Estimated maximum FPS: %.2f\n",
        finalResult.estimatedMaxFPS
    );

    Serial.printf(
        "Estimated throughput: %.1f points/s\n",
        finalResult.throughput
    );

    Serial.printf(
        "Human detection stable: %s\n",
        finalResult.humanDetectionCorrect
            ? "YES"
            : "NO"
    );

    Serial.printf(
        "Real-time: %s\n",
        finalResult.realtime
            ? "YES"
            : "NO"
    );

    Serial.println();

    Serial.println(
        "Experiment 07B completed."
    );

    Serial.println(
        "================================================"
    );
}


// ============================================================
// Arduino loop
// ============================================================

void loop()
{
    delay(1000);
}
