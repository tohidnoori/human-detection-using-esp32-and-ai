#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ============================================================
// Experiment C
// Spatial Grid + Dual-Core DBSCAN
//
// Single-Core:
//   Spatial Grid + DBSCAN
//
// Dual-Core:
//   Spatial Grid
//        +
//   Core detection split between Core 0 / Core 1
//        +
//   Serial DBSCAN expansion
//
// No N x N distance matrix.
// Exact squared-distance verification is still performed.
// ============================================================


// ============================================================
// Configuration
// ============================================================

#define MAX_N 1440

#define GRID_BUCKETS 2048

const float EPSILON = 0.20f;
const float EPSILON2 = EPSILON * EPSILON;

const int MIN_PTS = 5;

const float CLUSTER_RADIUS = 0.60f;


// ============================================================
// Data structures
// ============================================================

struct Point3D
{
    float x;
    float y;
    float z;
};


// ============================================================
// Main buffers
// ============================================================

Point3D *points = nullptr;

// Baseline Grid DBSCAN
int *baselineLabels = nullptr;
int *baselineQueue = nullptr;

// Dual-Core Grid DBSCAN
int *dualLabels = nullptr;
int *dualQueue = nullptr;

// Core flags
uint8_t *baselineCore = nullptr;
uint8_t *dualCore = nullptr;


// ============================================================
// Spatial Grid
// ============================================================

// Grid linked-list representation:
//
// gridHead[cell] = first point in cell
//
// gridNext[point] = next point in same cell
//
// cellX/Y/Z are stored so every point's grid coordinates
// are known.

int *gridHead = nullptr;
int *gridNext = nullptr;

int *cellX = nullptr;
int *cellY = nullptr;
int *cellZ = nullptr;


// ============================================================
// Dual-Core worker
// ============================================================

TaskHandle_t dualWorkerHandle = nullptr;
TaskHandle_t mainTaskHandle = nullptr;

volatile int workerStart = 0;
volatile int workerEnd = 0;

volatile bool workerFinished = false;


// ============================================================
// Deterministic random generator
// ============================================================

uint32_t rngState = 123456789;

uint32_t nextRandom()
{
    rngState ^= rngState << 13;
    rngState ^= rngState >> 17;
    rngState ^= rngState << 5;

    return rngState;
}


float randomFloat(
    float minValue,
    float maxValue
)
{
    float r =
        (nextRandom() % 1000000) /
        1000000.0f;

    return minValue +
           r * (maxValue - minValue);
}


// ============================================================
// Distance
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
// Dataset generation
// ============================================================

void generateDataset(int N)
{
    rngState = 123456789;

    const Point3D centers[3] =
    {
        {0.0f, 0.0f, 0.0f},
        {5.0f, 5.0f, 5.0f},
        {10.0f, 0.0f, 5.0f}
    };

    int perCluster = N / 3;

    for (int i = 0; i < N; i++)
    {
        int cluster =
            i / perCluster;

        if (cluster > 2)
            cluster = 2;

        points[i].x =
            centers[cluster].x +
            randomFloat(
                -CLUSTER_RADIUS,
                CLUSTER_RADIUS
            );

        points[i].y =
            centers[cluster].y +
            randomFloat(
                -CLUSTER_RADIUS,
                CLUSTER_RADIUS
            );

        points[i].z =
            centers[cluster].z +
            randomFloat(
                -CLUSTER_RADIUS,
                CLUSTER_RADIUS
            );
    }
}


// ============================================================
// Spatial Grid Hash
// ============================================================

inline uint32_t gridHash(
    int x,
    int y,
    int z
)
{
    // Large primes for spatial hashing.
    uint32_t h =
        (uint32_t)(x * 73856093);

    h ^=
        (uint32_t)(y * 19349663);

    h ^=
        (uint32_t)(z * 83492791);

    return h % GRID_BUCKETS;
}


// ============================================================
// Grid coordinate
// ============================================================

inline int getCellCoordinate(
    float value
)
{
    return (int)floorf(
        value / EPSILON
    );
}


// ============================================================
// Clear grid
// ============================================================

void clearGrid()
{
    for (int i = 0; i < GRID_BUCKETS; i++)
    {
        gridHead[i] = -1;
    }
}


// ============================================================
// Build spatial grid
// ============================================================

void buildGrid(int N)
{
    clearGrid();

    for (int i = 0; i < N; i++)
    {
        int x =
            getCellCoordinate(
                points[i].x
            );

        int y =
            getCellCoordinate(
                points[i].y
            );

        int z =
            getCellCoordinate(
                points[i].z
            );

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
//
// Because epsilon is the cell size, only 27 cells need to
// be searched:
//
//        3 x 3 x 3 = 27
//
// Exact distance is still checked, so the result remains
// equivalent to brute-force DBSCAN.
// ============================================================

int gridCountNeighbors(
    int index,
    int N
)
{
    (void)N;

    int cx = cellX[index];
    int cy = cellY[index];
    int cz = cellZ[index];

    int count = 0;

    const Point3D &query =
        points[index];

    for (int dx = -1; dx <= 1; dx++)
    {
        for (int dy = -1; dy <= 1; dy++)
        {
            for (int dz = -1; dz <= 1; dz++)
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
                    // Hash collisions are possible.
                    // Verify actual cell coordinates.
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
// Add neighbors using spatial grid
// ============================================================

void gridAddNeighborsToQueue(
    int index,
    int N,
    int *labels,
    int *queue,
    int &queueSize
)
{
    (void)N;

    int cx = cellX[index];
    int cy = cellY[index];
    int cz = cellZ[index];

    const Point3D &query =
        points[index];

    for (int dx = -1; dx <= 1; dx++)
    {
        for (int dy = -1; dy <= 1; dy++)
        {
            for (int dz = -1; dz <= 1; dz++)
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
                                labels[current] = -1;

                                if (
                                    queueSize < N
                                )
                                {
                                    queue[
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
// BASELINE
// Spatial Grid + Single Core
// ============================================================

void runBaselineGridDBSCAN(
    int N,
    int *labels,
    int *queue,
    uint8_t *coreFlags
)
{
    // Reset labels
    for (int i = 0; i < N; i++)
    {
        labels[i] = 0;
        coreFlags[i] = 0;
    }

    // --------------------------------------------------------
    // Core detection
    // --------------------------------------------------------

    for (int i = 0; i < N; i++)
    {
        int neighbors =
            gridCountNeighbors(
                i,
                N
            );

        if (neighbors >= MIN_PTS)
        {
            coreFlags[i] = 1;
        }
        else
        {
            coreFlags[i] = 0;
        }
    }

    // --------------------------------------------------------
    // DBSCAN expansion
    // --------------------------------------------------------

    int clusterId = 0;

    for (int i = 0; i < N; i++)
    {
        if (labels[i] != 0)
            continue;

        if (!coreFlags[i])
        {
            labels[i] = -2;
            continue;
        }

        clusterId++;

        int queueSize = 0;
        int queuePosition = 0;

        labels[i] =
            clusterId;

        gridAddNeighborsToQueue(
            i,
            N,
            labels,
            queue,
            queueSize
        );

        while (
            queuePosition <
            queueSize
        )
        {
            int current =
                queue[
                    queuePosition++
                ];

            if (labels[current] == -1)
            {
                labels[current] =
                    clusterId;
            }

            if (coreFlags[current])
            {
                gridAddNeighborsToQueue(
                    current,
                    N,
                    labels,
                    queue,
                    queueSize
                );
            }
        }
    }
}


// ============================================================
// DUAL-CORE WORKER
// ============================================================

void dualWorkerTask(
    void *parameter
)
{
    (void)parameter;

    while (true)
    {
        // Wait for Core 0.
        ulTaskNotifyTake(
            pdTRUE,
            portMAX_DELAY
        );

        int start =
            workerStart;

        int end =
            workerEnd;

        // ----------------------------------------------------
        // Core 1 performs core detection for its half.
        // ----------------------------------------------------

        for (int i = start; i < end; i++)
        {
            int neighbors =
                gridCountNeighbors(
                    i,
                    end
                );

            if (neighbors >= MIN_PTS)
            {
                dualCore[i] = 1;
            }
            else
            {
                dualCore[i] = 0;
            }
        }

        workerFinished = true;

        // Notify Core 0.
        xTaskNotifyGive(
            mainTaskHandle
        );
    }
}


// ============================================================
// DUAL-CORE CORE DETECTION
// ============================================================

void runDualCoreDetection(
    int N
)
{
    int midpoint =
        N / 2;

    // --------------------------------------------------------
    // Core 0 handles first half.
    // --------------------------------------------------------

    for (int i = 0; i < midpoint; i++)
    {
        int neighbors =
            gridCountNeighbors(
                i,
                N
            );

        if (neighbors >= MIN_PTS)
        {
            dualCore[i] = 1;
        }
        else
        {
            dualCore[i] = 0;
        }
    }

    // --------------------------------------------------------
    // Start Core 1.
    // --------------------------------------------------------

    workerStart =
        midpoint;

    workerEnd =
        N;

    workerFinished = false;

    xTaskNotifyGive(
        dualWorkerHandle
    );

    // --------------------------------------------------------
    // Wait until Core 1 completes.
    // --------------------------------------------------------

    ulTaskNotifyTake(
        pdTRUE,
        portMAX_DELAY
    );
}


// ============================================================
// DUAL-CORE DBSCAN
// ============================================================

void runDualGridDBSCAN(
    int N,
    int *labels,
    int *queue,
    uint8_t *coreFlags
)
{
    // Reset labels.
    for (int i = 0; i < N; i++)
    {
        labels[i] = 0;
        coreFlags[i] = 0;
    }

    // --------------------------------------------------------
    // Parallel core detection
    // --------------------------------------------------------

    runDualCoreDetection(
        N
    );

    // --------------------------------------------------------
    // Copy/keep result
    // --------------------------------------------------------

    // coreFlags points to the same classification
    // calculated by the dual-core stage.

    // --------------------------------------------------------
    // Serial DBSCAN expansion
    // --------------------------------------------------------

    int clusterId = 0;

    for (int i = 0; i < N; i++)
    {
        if (labels[i] != 0)
            continue;

        if (!coreFlags[i])
        {
            labels[i] = -2;
            continue;
        }

        clusterId++;

        int queueSize = 0;
        int queuePosition = 0;

        labels[i] =
            clusterId;

        gridAddNeighborsToQueue(
            i,
            N,
            labels,
            queue,
            queueSize
        );

        while (
            queuePosition <
            queueSize
        )
        {
            int current =
                queue[
                    queuePosition++
                ];

            if (labels[current] == -1)
            {
                labels[current] =
                    clusterId;
            }

            if (coreFlags[current])
            {
                gridAddNeighborsToQueue(
                    current,
                    N,
                    labels,
                    queue,
                    queueSize
                );
            }
        }
    }
}


// ============================================================
// Statistics
// ============================================================

struct DBSCANStats
{
    int clusters;
    int core;
    int border;
    int noise;
};


// ============================================================

DBSCANStats calculateStats(
    int N,
    int *labels,
    uint8_t *coreFlags
)
{
    DBSCANStats stats{};

    int maxCluster = 0;

    for (int i = 0; i < N; i++)
    {
        if (labels[i] == -2)
        {
            stats.noise++;
        }
        else if (
            labels[i] > maxCluster
        )
        {
            maxCluster =
                labels[i];
        }
    }

    stats.clusters =
        maxCluster;

    // --------------------------------------------------------
    // Core / Border
    // --------------------------------------------------------

    for (int i = 0; i < N; i++)
    {
        if (labels[i] <= 0)
            continue;

        if (coreFlags[i])
        {
            stats.core++;
        }
        else
        {
            stats.border++;
        }
    }

    return stats;
}


// ============================================================
// Compare labels
// ============================================================

int compareLabels(
    int N,
    int *a,
    int *b
)
{
    int different = 0;

    for (int i = 0; i < N; i++)
    {
        if (a[i] != b[i])
        {
            different++;
        }
    }

    return different;
}


// ============================================================
// Memory
// ============================================================

void printMemoryInfo()
{
    size_t pointsMemory =
        sizeof(Point3D) * MAX_N;

    size_t labelsMemory =
        sizeof(int) * MAX_N * 2;

    size_t queueMemory =
        sizeof(int) * MAX_N * 2;

    size_t coreMemory =
        sizeof(uint8_t) * MAX_N * 2;

    size_t gridHeadMemory =
        sizeof(int) * GRID_BUCKETS;

    size_t gridNextMemory =
        sizeof(int) * MAX_N;

    size_t cellMemory =
        sizeof(int) * MAX_N * 3;

    size_t total =
        pointsMemory +
        labelsMemory +
        queueMemory +
        coreMemory +
        gridHeadMemory +
        gridNextMemory +
        cellMemory;

    Serial.printf(
        "Comparison buffers: %.2f KB\n",
        total / 1024.0f
    );

    Serial.printf(
        "Free heap: %u bytes\n",
        ESP.getFreeHeap()
    );

    Serial.printf(
        "Free PSRAM: %u bytes\n",
        ESP.getFreePsram()
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
        "=============================================="
    );
    Serial.println(
        "ESP32-S3 DBSCAN"
    );
    Serial.println(
        "Experiment C"
    );
    Serial.println(
        "Spatial Grid + Dual-Core"
    );
    Serial.println(
        "=============================================="
    );

    Serial.printf(
        "CPU frequency: %u MHz\n",
        getCpuFrequencyMhz()
    );

    Serial.printf(
        "Free heap: %u bytes\n",
        ESP.getFreeHeap()
    );

    Serial.printf(
        "Free PSRAM: %u bytes\n",
        ESP.getFreePsram()
    );


    // ========================================================
    // Allocate buffers
    // ========================================================

    points =
        (Point3D *)ps_malloc(
            sizeof(Point3D) * MAX_N
        );

    baselineLabels =
        (int *)ps_malloc(
            sizeof(int) * MAX_N
        );

    dualLabels =
        (int *)ps_malloc(
            sizeof(int) * MAX_N
        );

    baselineQueue =
        (int *)ps_malloc(
            sizeof(int) * MAX_N
        );

    dualQueue =
        (int *)ps_malloc(
            sizeof(int) * MAX_N
        );

    baselineCore =
        (uint8_t *)ps_malloc(
            sizeof(uint8_t) * MAX_N
        );

    dualCore =
        (uint8_t *)ps_malloc(
            sizeof(uint8_t) * MAX_N
        );


    // Grid

    gridHead =
        (int *)ps_malloc(
            sizeof(int) * GRID_BUCKETS
        );

    gridNext =
        (int *)ps_malloc(
            sizeof(int) * MAX_N
        );

    cellX =
        (int *)ps_malloc(
            sizeof(int) * MAX_N
        );

    cellY =
        (int *)ps_malloc(
            sizeof(int) * MAX_N
        );

    cellZ =
        (int *)ps_malloc(
            sizeof(int) * MAX_N
        );


    if (
        !points ||
        !baselineLabels ||
        !dualLabels ||
        !baselineQueue ||
        !dualQueue ||
        !baselineCore ||
        !dualCore ||
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


    // ========================================================
    // Main task handle
    // ========================================================

    mainTaskHandle =
        xTaskGetCurrentTaskHandle();


    // ========================================================
    // Core 1 worker
    // ========================================================

    BaseType_t result =
        xTaskCreatePinnedToCore(
            dualWorkerTask,
            "GridDBSCANWorker",
            8192,
            nullptr,
            2,
            &dualWorkerHandle,
            1
        );

    if (result != pdPASS)
    {
        Serial.println(
            "ERROR: Core 1 worker creation failed!"
        );

        while (true)
        {
            delay(1000);
        }
    }

    Serial.println(
        "Core 1 grid worker created."
    );


    // ========================================================
    // Configuration
    // ========================================================

    Serial.println();

    Serial.println(
        "Configuration:"
    );

    Serial.println(
        "Dimensions: 3"
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
        "Cluster radius: %.2f\n",
        CLUSTER_RADIUS
    );

    Serial.println();

    printMemoryInfo();

    Serial.println();


    // ========================================================
    // Experiment sizes
    // ========================================================

    const int testSizes[] =
    {
        180,
        360,
        720,
        1080,
        1440
    };

    const int testCount =
        sizeof(testSizes) /
        sizeof(testSizes[0]);


    // ========================================================
    // Run experiments
    // ========================================================

    for (int t = 0; t < testCount; t++)
    {
        int N =
            testSizes[t];

        Serial.println();

        Serial.println(
            "----------------------------------------------"
        );

        Serial.printf(
            "N = %d\n",
            N
        );

        // ----------------------------------------------------
        // Generate ONE dataset
        // ----------------------------------------------------

        Serial.println(
            "Generating deterministic dataset..."
        );

        generateDataset(N);


        // ----------------------------------------------------
        // Build ONE grid
        // ----------------------------------------------------

        Serial.println(
            "Building spatial grid..."
        );

        uint64_t gridStart =
            esp_timer_get_time();

        buildGrid(N);

        uint64_t gridEnd =
            esp_timer_get_time();

        double gridBuildMs =
            (gridEnd - gridStart) /
            1000.0;


        Serial.printf(
            "Grid build: %.3f ms\n",
            gridBuildMs
        );


        // ====================================================
        // Baseline Spatial Grid
        // ====================================================

        Serial.println(
            "Running single-core grid DBSCAN..."
        );

        uint64_t baselineStart =
            esp_timer_get_time();

        runBaselineGridDBSCAN(
            N,
            baselineLabels,
            baselineQueue,
            baselineCore
        );

        uint64_t baselineEnd =
            esp_timer_get_time();

        double baselineMs =
            (baselineEnd - baselineStart) /
            1000.0;


        // ====================================================
        // Dual-Core Spatial Grid
        // ====================================================

        Serial.println(
            "Running dual-core grid DBSCAN..."
        );

        uint64_t dualStart =
            esp_timer_get_time();

        runDualGridDBSCAN(
            N,
            dualLabels,
            dualQueue,
            dualCore
        );

        uint64_t dualEnd =
            esp_timer_get_time();

        double dualMs =
            (dualEnd - dualStart) /
            1000.0;


        // ====================================================
        // Statistics
        // ====================================================

        DBSCANStats baselineStats =
            calculateStats(
                N,
                baselineLabels,
                baselineCore
            );

        DBSCANStats dualStats =
            calculateStats(
                N,
                dualLabels,
                dualCore
            );


        int different =
            compareLabels(
                N,
                baselineLabels,
                dualLabels
            );


        // ----------------------------------------------------
        // Speedup
        // ----------------------------------------------------

        double speedup =
            baselineMs /
            dualMs;

        double improvement =
            (
                (baselineMs - dualMs) /
                baselineMs
            ) * 100.0;


        // ----------------------------------------------------
        // End-to-end including grid build
        //
        // Grid is shared by both algorithms in this
        // comparison, so DBSCAN runtime is the primary
        // comparison.
        //
        // We also report dual total = grid build + dual DBSCAN.
        // ----------------------------------------------------

        double dualTotalMs =
            gridBuildMs +
            dualMs;

        double baselineTotalMs =
            gridBuildMs +
            baselineMs;

        double totalSpeedup =
            baselineTotalMs /
            dualTotalMs;


        // ====================================================
        // Output
        // ====================================================

        Serial.println();

        Serial.printf(
            "Grid build: %.3f ms\n",
            gridBuildMs
        );

        Serial.printf(
            "Baseline Grid: %.3f ms\n",
            baselineMs
        );

        Serial.printf(
            "Dual-Core Grid: %.3f ms\n",
            dualMs
        );

        Serial.printf(
            "Dual total: %.3f ms\n",
            dualTotalMs
        );

        Serial.printf(
            "DBSCAN speedup: %.2fx\n",
            speedup
        );

        Serial.printf(
            "DBSCAN improvement: %.2f%%\n",
            improvement
        );

        Serial.printf(
            "End-to-end speedup: %.2fx\n",
            totalSpeedup
        );

        Serial.println();

        Serial.printf(
            "Clusters: %d / %d\n",
            baselineStats.clusters,
            dualStats.clusters
        );

        Serial.printf(
            "Core: %d / %d\n",
            baselineStats.core,
            dualStats.core
        );

        Serial.printf(
            "Border: %d / %d\n",
            baselineStats.border,
            dualStats.border
        );

        Serial.printf(
            "Noise: %d / %d\n",
            baselineStats.noise,
            dualStats.noise
        );

        Serial.printf(
            "Different labels: %d\n",
            different
        );


        if (different == 0)
        {
            Serial.println(
                "IDENTICAL"
            );
        }
        else
        {
            Serial.println(
                "WARNING: RESULTS DIFFER"
            );
        }


        Serial.println(
            "----------------------------------------------"
        );
    }


    // ========================================================
    // Finished
    // ========================================================

    Serial.println();

    Serial.println(
        "Experiment C completed."
    );

    Serial.println(
        "=============================================="
    );
}


// ============================================================
// Loop
// ============================================================

void loop()
{
    delay(1000);
}