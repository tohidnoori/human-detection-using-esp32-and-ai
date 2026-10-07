#include <Arduino.h>
#include <esp_heap_caps.h>
#include <math.h>

// ============================================================
// EXPERIMENT 05
// Baseline DBSCAN vs Memory-Efficient Optimized DBSCAN
//
// Baseline:
//   Brute-force neighborhood search.
//   Every neighborhood query scans all N points.
//
// Optimized:
//   Uniform spatial grid.
//   Only neighboring grid cells are searched.
//
// Experiments:
//   A) Scaling with N
//   B) Epsilon sensitivity
//   C) MinPts sensitivity
//
// The same deterministic dataset is used for both algorithms.
//
// Important:
//   The optimized implementation must produce the same DBSCAN
//   classification as the baseline implementation.
// ============================================================


// ============================================================
// CONSTANTS
// ============================================================

const int MAX_N = 1440;

const int DIMENSIONS = 3;

const int UNCLASSIFIED = -1;
const int NOISE = -2;

// Maximum number of hash buckets.
// This is NOT an N x N matrix.
const int GRID_BUCKETS = 2048;


// ============================================================
// POINT
// ============================================================

struct Point3D
{
    float x;
    float y;
    float z;
};


// ============================================================
// GLOBAL DATA
// ============================================================

Point3D* points = nullptr;


// ------------------------------------------------------------
// Baseline
// ------------------------------------------------------------

int* baselineLabels = nullptr;
int* baselineQueue = nullptr;


// ------------------------------------------------------------
// Optimized
// ------------------------------------------------------------

int* optimizedLabels = nullptr;
int* optimizedQueue = nullptr;


// ============================================================
// SPATIAL GRID
// ============================================================
//
// Each point belongs to one grid cell.
//
// Cell size = epsilon.
//
// Therefore any point within epsilon distance must be located
// either in the same cell or one of the 26 neighboring cells.
//
// We use a hash table:
//
// bucket -> linked list of points
//
// This avoids allocating an enormous N x N matrix.
// ============================================================

int* gridHead = nullptr;
int* gridNext = nullptr;

int* cellX = nullptr;
int* cellY = nullptr;
int* cellZ = nullptr;


// ============================================================
// DATASET CONFIGURATION
// ============================================================

float currentEpsilon = 0.20f;
float currentEpsilonSquared = 0.20f * 0.20f;

int currentMinPts = 5;

float datasetRadius = 0.60f;


// ============================================================
// RANDOM GENERATOR
// ============================================================

uint32_t randomState = 123456789;


uint32_t nextRandom()
{
    randomState =
        randomState * 1664525UL +
        1013904223UL;

    return randomState;
}


float randomFloat(
    float minValue,
    float maxValue
)
{
    uint32_t value = nextRandom();

    float normalized =
        (float)value / 4294967295.0f;

    return minValue +
           normalized *
           (maxValue - minValue);
}


// ============================================================
// DISTANCE
// ============================================================

inline float squaredDistance(
    const Point3D& a,
    const Point3D& b
)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;

    return
        dx * dx +
        dy * dy +
        dz * dz;
}


// ============================================================
// DATASET GENERATION
// ============================================================
//
// Three clusters:
//
// C1 = (0, 0, 0)
// C2 = (5, 5, 5)
// C3 = (10, 0, 5)
//
// The random sequence is reset for every experiment.
//
// Therefore datasets are deterministic and reproducible.
// ============================================================

void generateDataset(int N)
{
    randomState = 123456789;

    int pointsPerCluster = N / 3;

    for (int i = 0; i < N; i++)
    {
        int cluster =
            i / pointsPerCluster;

        float centerX;
        float centerY;
        float centerZ;

        if (cluster == 0)
        {
            centerX = 0.0f;
            centerY = 0.0f;
            centerZ = 0.0f;
        }
        else if (cluster == 1)
        {
            centerX = 5.0f;
            centerY = 5.0f;
            centerZ = 5.0f;
        }
        else
        {
            centerX = 10.0f;
            centerY = 0.0f;
            centerZ = 5.0f;
        }

        points[i].x =
            centerX +
            randomFloat(
                -datasetRadius,
                datasetRadius
            );

        points[i].y =
            centerY +
            randomFloat(
                -datasetRadius,
                datasetRadius
            );

        points[i].z =
            centerZ +
            randomFloat(
                -datasetRadius,
                datasetRadius
            );
    }
}


// ============================================================
// BASELINE DBSCAN
// ============================================================


// ------------------------------------------------------------
// Count neighbors
// ------------------------------------------------------------

int baselineCountNeighbors(
    int pointIndex,
    int N
)
{
    int count = 0;

    for (int i = 0; i < N; i++)
    {
        if (i == pointIndex)
            continue;

        if (
            squaredDistance(
                points[pointIndex],
                points[i]
            ) <= currentEpsilonSquared
        )
        {
            count++;
        }
    }

    return count;
}


// ------------------------------------------------------------
// Add neighbors
// ------------------------------------------------------------

int baselineAddNeighbors(
    int pointIndex,
    int queueSize,
    int N
)
{
    for (int i = 0; i < N; i++)
    {
        if (i == pointIndex)
            continue;

        if (
            squaredDistance(
                points[pointIndex],
                points[i]
            ) <= currentEpsilonSquared
        )
        {
            if (
                baselineLabels[i] ==
                UNCLASSIFIED
            )
            {
                baselineLabels[i] = 0;

                if (queueSize < N)
                {
                    baselineQueue[
                        queueSize
                    ] = i;

                    queueSize++;
                }
            }
        }
    }

    return queueSize;
}


// ------------------------------------------------------------
// Expand cluster
// ------------------------------------------------------------

void baselineExpandCluster(
    int pointIndex,
    int clusterId,
    int N
)
{
    int queueSize = 0;

    queueSize =
        baselineAddNeighbors(
            pointIndex,
            queueSize,
            N
        );

    baselineLabels[pointIndex] =
        clusterId;

    int queueIndex = 0;

    while (queueIndex < queueSize)
    {
        int currentPoint =
            baselineQueue[
                queueIndex
            ];

        queueIndex++;

        if (
            baselineLabels[
                currentPoint
            ] == NOISE
        )
        {
            baselineLabels[
                currentPoint
            ] = clusterId;
        }

        if (
            baselineLabels[
                currentPoint
            ] != 0
        )
        {
            continue;
        }

        baselineLabels[
            currentPoint
        ] = clusterId;

        int neighbors =
            baselineCountNeighbors(
                currentPoint,
                N
            );

        if (neighbors >= currentMinPts)
        {
            queueSize =
                baselineAddNeighbors(
                    currentPoint,
                    queueSize,
                    N
                );
        }
    }
}


// ------------------------------------------------------------
// Baseline DBSCAN
// ------------------------------------------------------------

int runBaseline(
    int N
)
{
    for (int i = 0; i < N; i++)
    {
        baselineLabels[i] =
            UNCLASSIFIED;
    }

    int clusterId = 0;

    for (int i = 0; i < N; i++)
    {
        if (
            baselineLabels[i] !=
            UNCLASSIFIED
        )
        {
            continue;
        }

        int neighbors =
            baselineCountNeighbors(
                i,
                N
            );

        if (neighbors < currentMinPts)
        {
            baselineLabels[i] =
                NOISE;
        }
        else
        {
            clusterId++;

            baselineExpandCluster(
                i,
                clusterId,
                N
            );
        }
    }

    return clusterId;
}


// ============================================================
// SPATIAL GRID
// ============================================================


// ------------------------------------------------------------
// Convert coordinate to cell coordinate
// ------------------------------------------------------------

inline int getCellCoordinate(
    float value
)
{
    return (int)floor(
        value / currentEpsilon
    );
}


// ------------------------------------------------------------
// Hash cell coordinate
// ------------------------------------------------------------

inline int hashCell(
    int x,
    int y,
    int z
)
{
    uint32_t h =
        (uint32_t)(
            x * 73856093
        );

    h ^=
        (uint32_t)(
            y * 19349663
        );

    h ^=
        (uint32_t)(
            z * 83492791
        );

    return h % GRID_BUCKETS;
}


// ------------------------------------------------------------
// Build spatial grid
// ------------------------------------------------------------

void buildGrid(int N)
{
    // Clear bucket heads
    for (int i = 0;
         i < GRID_BUCKETS;
         i++)
    {
        gridHead[i] = -1;
    }

    // Insert every point
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

        int bucket =
            hashCell(
                x,
                y,
                z
            );

        gridNext[i] =
            gridHead[bucket];

        gridHead[bucket] = i;
    }
}


// ============================================================
// OPTIMIZED NEIGHBOR SEARCH
// ============================================================


// ------------------------------------------------------------
// Count neighbors using grid
// ------------------------------------------------------------

int optimizedCountNeighbors(
    int pointIndex,
    int N
)
{
    int count = 0;

    int baseX = cellX[pointIndex];
    int baseY = cellY[pointIndex];
    int baseZ = cellZ[pointIndex];

    for (int dx = -1; dx <= 1; dx++)
    {
        for (int dy = -1; dy <= 1; dy++)
        {
            for (int dz = -1; dz <= 1; dz++)
            {
                int targetX =
                    baseX + dx;

                int targetY =
                    baseY + dy;

                int targetZ =
                    baseZ + dz;

                int bucket =
                    hashCell(
                        targetX,
                        targetY,
                        targetZ
                    );

                int candidate =
                    gridHead[bucket];

                while (candidate != -1)
                {
                    if (
                        candidate != pointIndex &&
                        cellX[candidate] == targetX &&
                        cellY[candidate] == targetY &&
                        cellZ[candidate] == targetZ
                    )
                    {
                        if (
                            squaredDistance(
                                points[pointIndex],
                                points[candidate]
                            ) <=
                            currentEpsilonSquared
                        )
                        {
                            count++;
                        }
                    }

                    candidate =
                        gridNext[candidate];
                }
            }
        }
    }

    return count;
}


// ------------------------------------------------------------
// Add neighbors using grid
// ------------------------------------------------------------

int optimizedAddNeighbors(
    int pointIndex,
    int queueSize,
    int N
)
{
    int baseX = cellX[pointIndex];
    int baseY = cellY[pointIndex];
    int baseZ = cellZ[pointIndex];

    for (int dx = -1; dx <= 1; dx++)
    {
        for (int dy = -1; dy <= 1; dy++)
        {
            for (int dz = -1; dz <= 1; dz++)
            {
                int targetX =
                    baseX + dx;

                int targetY =
                    baseY + dy;

                int targetZ =
                    baseZ + dz;

                int bucket =
                    hashCell(
                        targetX,
                        targetY,
                        targetZ
                    );

                int candidate =
                    gridHead[bucket];

                while (candidate != -1)
                {
                    if (
                        candidate != pointIndex &&
                        cellX[candidate] == targetX &&
                        cellY[candidate] == targetY &&
                        cellZ[candidate] == targetZ
                    )
                    {
                        if (
                            squaredDistance(
                                points[pointIndex],
                                points[candidate]
                            ) <=
                            currentEpsilonSquared
                        )
                        {
                            if (
                                optimizedLabels[
                                    candidate
                                ] == UNCLASSIFIED
                            )
                            {
                                optimizedLabels[
                                    candidate
                                ] = 0;

                                if (
                                    queueSize < N
                                )
                                {
                                    optimizedQueue[
                                        queueSize
                                    ] = candidate;

                                    queueSize++;
                                }
                            }
                        }
                    }

                    candidate =
                        gridNext[candidate];
                }
            }
        }
    }

    return queueSize;
}


// ------------------------------------------------------------
// Optimized cluster expansion
// ------------------------------------------------------------

void optimizedExpandCluster(
    int pointIndex,
    int clusterId,
    int N
)
{
    int queueSize = 0;

    queueSize =
        optimizedAddNeighbors(
            pointIndex,
            queueSize,
            N
        );

    optimizedLabels[
        pointIndex
    ] = clusterId;

    int queueIndex = 0;

    while (queueIndex < queueSize)
    {
        int currentPoint =
            optimizedQueue[
                queueIndex
            ];

        queueIndex++;

        if (
            optimizedLabels[
                currentPoint
            ] == NOISE
        )
        {
            optimizedLabels[
                currentPoint
            ] = clusterId;
        }

        if (
            optimizedLabels[
                currentPoint
            ] != 0
        )
        {
            continue;
        }

        optimizedLabels[
            currentPoint
        ] = clusterId;

        int neighbors =
            optimizedCountNeighbors(
                currentPoint,
                N
            );

        if (neighbors >= currentMinPts)
        {
            queueSize =
                optimizedAddNeighbors(
                    currentPoint,
                    queueSize,
                    N
                );
        }
    }
}


// ------------------------------------------------------------
// Optimized DBSCAN
// ------------------------------------------------------------

int runOptimized(
    int N
)
{
    for (int i = 0; i < N; i++)
    {
        optimizedLabels[i] =
            UNCLASSIFIED;
    }

    int clusterId = 0;

    for (int i = 0; i < N; i++)
    {
        if (
            optimizedLabels[i] !=
            UNCLASSIFIED
        )
        {
            continue;
        }

        int neighbors =
            optimizedCountNeighbors(
                i,
                N
            );

        if (neighbors < currentMinPts)
        {
            optimizedLabels[i] =
                NOISE;
        }
        else
        {
            clusterId++;

            optimizedExpandCluster(
                i,
                clusterId,
                N
            );
        }
    }

    return clusterId;
}


// ============================================================
// RESULT ANALYSIS
// ============================================================

int countNoise(
    int* labels,
    int N
)
{
    int count = 0;

    for (int i = 0; i < N; i++)
    {
        if (labels[i] == NOISE)
            count++;
    }

    return count;
}


bool resultsMatch(
    int N
)
{
    for (int i = 0; i < N; i++)
    {
        if (
            baselineLabels[i] !=
            optimizedLabels[i]
        )
        {
            return false;
        }
    }

    return true;
}


// ============================================================
// MEMORY
// ============================================================

void printMemory()
{
    size_t pointsMemory =
        MAX_N * sizeof(Point3D);

    size_t labelsMemory =
        MAX_N * sizeof(int);

    size_t queueMemory =
        MAX_N * sizeof(int);

    size_t gridHeadMemory =
        GRID_BUCKETS * sizeof(int);

    size_t gridPointMemory =
        MAX_N * sizeof(int);

    size_t cellMemory =
        MAX_N *
        3 *
        sizeof(int);

    Serial.println(
        "Memory allocation:"
    );

    Serial.printf(
        "Points: %.2f KB\n",
        pointsMemory / 1024.0
    );

    Serial.printf(
        "Labels: %.2f KB\n",
        (labelsMemory * 2) / 1024.0
    );

    Serial.printf(
        "Queues: %.2f KB\n",
        (queueMemory * 2) / 1024.0
    );

    Serial.printf(
        "Grid heads: %.2f KB\n",
        gridHeadMemory / 1024.0
    );

    Serial.printf(
        "Grid links: %.2f KB\n",
        gridPointMemory / 1024.0
    );

    Serial.printf(
        "Cell coordinates: %.2f KB\n",
        cellMemory / 1024.0
    );

    size_t total =
        pointsMemory +
        labelsMemory * 2 +
        queueMemory * 2 +
        gridHeadMemory +
        gridPointMemory +
        cellMemory;

    Serial.printf(
        "Total allocated: %.2f KB\n",
        total / 1024.0
    );
}


// ============================================================
// SINGLE BENCHMARK
// ============================================================

void runBenchmark(
    int N,
    float epsilon,
    int minPts
)
{
    currentEpsilon =
        epsilon;

    currentEpsilonSquared =
        epsilon * epsilon;

    currentMinPts =
        minPts;


    // --------------------------------------------------------
    // Generate identical dataset
    // --------------------------------------------------------

    generateDataset(N);


    // --------------------------------------------------------
    // BASELINE
    // --------------------------------------------------------

    size_t heapBeforeBaseline =
        ESP.getFreeHeap();

    size_t psramBeforeBaseline =
        ESP.getFreePsram();


    unsigned long baselineStart =
        micros();

    int baselineClusters =
        runBaseline(N);

    unsigned long baselineEnd =
        micros();


    double baselineTime =
        (baselineEnd -
         baselineStart) / 1000.0;


    int baselineNoise =
        countNoise(
            baselineLabels,
            N
        );


    // --------------------------------------------------------
    // Build spatial grid
    //
    // NOT included in optimized DBSCAN time.
    //
    // We measure it separately.
    // --------------------------------------------------------

    unsigned long gridStart =
        micros();

    buildGrid(N);

    unsigned long gridEnd =
        micros();


    double gridBuildTime =
        (gridEnd -
         gridStart) / 1000.0;


    // --------------------------------------------------------
    // OPTIMIZED
    // --------------------------------------------------------

    size_t heapBeforeOptimized =
        ESP.getFreeHeap();

    size_t psramBeforeOptimized =
        ESP.getFreePsram();


    unsigned long optimizedStart =
        micros();

    int optimizedClusters =
        runOptimized(N);

    unsigned long optimizedEnd =
        micros();


    double optimizedTime =
        (optimizedEnd -
         optimizedStart) / 1000.0;


    int optimizedNoise =
        countNoise(
            optimizedLabels,
            N
        );


    // --------------------------------------------------------
    // Comparison
    // --------------------------------------------------------

    bool identical =
        resultsMatch(N);


    double speedup =
        baselineTime /
        optimizedTime;


    double totalOptimizedTime =
        gridBuildTime +
        optimizedTime;


    double totalSpeedup =
        baselineTime /
        totalOptimizedTime;


    // --------------------------------------------------------
    // Output
    // --------------------------------------------------------

    Serial.println();
    Serial.println(
        "----------------------------------------"
    );

    Serial.printf(
        "N=%d | Epsilon=%.2f | MinPts=%d\n",
        N,
        epsilon,
        minPts
    );

    Serial.println(
        "----------------------------------------"
    );

    Serial.printf(
        "Baseline time: %.3f ms\n",
        baselineTime
    );

    Serial.printf(
        "Optimized time: %.3f ms\n",
        optimizedTime
    );

    Serial.printf(
        "Grid build time: %.3f ms\n",
        gridBuildTime
    );

    Serial.printf(
        "Optimized total: %.3f ms\n",
        totalOptimizedTime
    );

    Serial.printf(
        "DBSCAN speedup: %.2fx\n",
        speedup
    );

    Serial.printf(
        "Total speedup: %.2fx\n",
        totalSpeedup
    );

    Serial.println();

    Serial.printf(
        "Baseline clusters: %d\n",
        baselineClusters
    );

    Serial.printf(
        "Optimized clusters: %d\n",
        optimizedClusters
    );

    Serial.printf(
        "Baseline noise: %d\n",
        baselineNoise
    );

    Serial.printf(
        "Optimized noise: %d\n",
        optimizedNoise
    );

    Serial.printf(
        "Results identical: %s\n",
        identical ? "YES" : "NO"
    );

    Serial.println();

    Serial.printf(
        "Heap before baseline: %u KB\n",
        (unsigned int)(
            heapBeforeBaseline / 1024
        )
    );

    Serial.printf(
        "Heap before optimized: %u KB\n",
        (unsigned int)(
            heapBeforeOptimized / 1024
        )
    );

    Serial.printf(
        "PSRAM before baseline: %u KB\n",
        (unsigned int)(
            psramBeforeBaseline / 1024
        )
    );

    Serial.printf(
        "PSRAM before optimized: %u KB\n",
        (unsigned int)(
            psramBeforeOptimized / 1024
        )
    );

    Serial.printf(
        "PSRAM after: %u KB\n",
        (unsigned int)(
            ESP.getFreePsram() / 1024
        )
    );
}


// ============================================================
// EXPERIMENT A
// SCALING WITH N
// ============================================================

void experimentScaling()
{
    Serial.println();
    Serial.println(
        "========================================"
    );

    Serial.println(
        "EXPERIMENT 5A"
    );

    Serial.println(
        "SCALING WITH N"
    );

    Serial.println(
        "========================================"
    );

    Serial.println(
        "Fixed: Epsilon=0.20, MinPts=5"
    );


    runBenchmark(
        180,
        0.20f,
        5
    );

    runBenchmark(
        360,
        0.20f,
        5
    );

    runBenchmark(
        720,
        0.20f,
        5
    );

    runBenchmark(
        1080,
        0.20f,
        5
    );

    runBenchmark(
        1440,
        0.20f,
        5
    );
}


// ============================================================
// EXPERIMENT B
// EPSILON SENSITIVITY
// ============================================================

void experimentEpsilon()
{
    Serial.println();
    Serial.println(
        "========================================"
    );

    Serial.println(
        "EXPERIMENT 5B"
    );

    Serial.println(
        "EPSILON SENSITIVITY"
    );

    Serial.println(
        "========================================"
    );

    Serial.println(
        "Fixed: N=720, MinPts=5"
    );


    runBenchmark(
        720,
        0.10f,
        5
    );

    runBenchmark(
        720,
        0.20f,
        5
    );

    runBenchmark(
        720,
        0.30f,
        5
    );

    runBenchmark(
        720,
        0.50f,
        5
    );
}


// ============================================================
// EXPERIMENT C
// MINPTS SENSITIVITY
// ============================================================

void experimentMinPts()
{
    Serial.println();
    Serial.println(
        "========================================"
    );

    Serial.println(
        "EXPERIMENT 5C"
    );

    Serial.println(
        "MINPTS SENSITIVITY"
    );

    Serial.println(
        "========================================"
    );

    Serial.println(
        "Fixed: N=720, Epsilon=0.20"
    );


    runBenchmark(
        720,
        0.20f,
        2
    );

    runBenchmark(
        720,
        0.20f,
        5
    );

    runBenchmark(
        720,
        0.20f,
        10
    );

    runBenchmark(
        720,
        0.20f,
        20
    );
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(2000);

    Serial.println();
    Serial.println(
        "========================================"
    );

    Serial.println(
        "ESP32-S3 DBSCAN"
    );

    Serial.println(
        "Experiment 05"
    );

    Serial.println(
        "Baseline vs Optimized"
    );

    Serial.println(
        "========================================"
    );

    Serial.println();

    // --------------------------------------------------------
    // Allocate maximum-size buffers
    // --------------------------------------------------------

    points =
        (Point3D*)ps_malloc(
            MAX_N *
            sizeof(Point3D)
        );

    baselineLabels =
        (int*)ps_malloc(
            MAX_N *
            sizeof(int)
        );

    baselineQueue =
        (int*)ps_malloc(
            MAX_N *
            sizeof(int)
        );

    optimizedLabels =
        (int*)ps_malloc(
            MAX_N *
            sizeof(int)
        );

    optimizedQueue =
        (int*)ps_malloc(
            MAX_N *
            sizeof(int)
        );


    // Grid

    gridHead =
        (int*)ps_malloc(
            GRID_BUCKETS *
            sizeof(int)
        );

    gridNext =
        (int*)ps_malloc(
            MAX_N *
            sizeof(int)
        );

    cellX =
        (int*)ps_malloc(
            MAX_N *
            sizeof(int)
        );

    cellY =
        (int*)ps_malloc(
            MAX_N *
            sizeof(int)
        );

    cellZ =
        (int*)ps_malloc(
            MAX_N *
            sizeof(int)
        );


    // --------------------------------------------------------
    // Allocation check
    // --------------------------------------------------------

    if (
        points == nullptr ||
        baselineLabels == nullptr ||
        baselineQueue == nullptr ||
        optimizedLabels == nullptr ||
        optimizedQueue == nullptr ||
        gridHead == nullptr ||
        gridNext == nullptr ||
        cellX == nullptr ||
        cellY == nullptr ||
        cellZ == nullptr
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


    // --------------------------------------------------------
    // Memory information
    // --------------------------------------------------------

    printMemory();

    Serial.println();

    Serial.printf(
        "Initial free heap: %u KB\n",
        (unsigned int)(
            ESP.getFreeHeap() / 1024
        )
    );

    Serial.printf(
        "Initial free PSRAM: %u KB\n",
        (unsigned int)(
            ESP.getFreePsram() / 1024
        )
    );


    // --------------------------------------------------------
    // Run experiments
    // --------------------------------------------------------

    experimentScaling();

    experimentEpsilon();

    experimentMinPts();


    // --------------------------------------------------------
    // Finished
    // --------------------------------------------------------

    Serial.println();
    Serial.println(
        "========================================"
    );

    Serial.println(
        "EXPERIMENT 05 COMPLETED"
    );

    Serial.println(
        "========================================"
    );
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    delay(1000);
}