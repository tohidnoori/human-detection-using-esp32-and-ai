#include <Arduino.h>
#include <esp_heap_caps.h>

// ============================================================
// ESP32-S3 N16R8
// DBSCAN BENCHMARK - SAFE VERSION
// ============================================================

#define DIMENSIONS 3

// DBSCAN parameters
#define EPSILON 0.50f
#define MIN_PTS 5

// Dataset sizes
const size_t TEST_SIZES[] = {
    90,
    180,
    360,
    720,
    1440,
    2880,
    5760,
    11520,
    23040
};

const int NUM_TESTS =
    sizeof(TEST_SIZES) / sizeof(TEST_SIZES[0]);

// ============================================================
// DBSCAN labels
// ============================================================

#define UNCLASSIFIED -1
#define NOISE        -2

// ============================================================
// Point
// ============================================================

struct Point3D
{
    float x;
    float y;
    float z;
};

// ============================================================
// Global data
// ============================================================

Point3D *points = nullptr;
int *labels = nullptr;
int *queueBuffer = nullptr;

size_t currentN = 0;

// ============================================================
// Distance squared
// ============================================================

inline float distanceSquared(
    const Point3D &a,
    const Point3D &b)
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
// Count neighbors
//
// No allocation.
// No free.
// Much safer for embedded systems.
//
// ============================================================

size_t countNeighbors(size_t pointIndex)
{
    const float epsilonSquared =
        EPSILON * EPSILON;

    size_t count = 0;

    for (size_t i = 0; i < currentN; i++)
    {
        if (distanceSquared(
                points[pointIndex],
                points[i]) <= epsilonSquared)
        {
            count++;
        }
    }

    return count;
}

// ============================================================
// Put all neighbors into queue
//
// Only adds currently unclassified points.
// This prevents duplicate queue entries.
//
// ============================================================

size_t addNeighborsToQueue(
    size_t pointIndex,
    int clusterId,
    size_t queueSize)
{
    const float epsilonSquared =
        EPSILON * EPSILON;

    for (size_t i = 0; i < currentN; i++)
    {
        if (distanceSquared(
                points[pointIndex],
                points[i]) <= epsilonSquared)
        {
            // Already assigned?
            if (labels[i] != UNCLASSIFIED)
            {
                continue;
            }

            // Safety check
            if (queueSize >= currentN)
            {
                Serial.println(
                    "ERROR: DBSCAN queue overflow"
                );

                return queueSize;
            }

            labels[i] = clusterId;

            queueBuffer[queueSize] = i;

            queueSize++;
        }
    }

    return queueSize;
}

// ============================================================
// Expand cluster
// ============================================================

bool expandCluster(
    size_t pointIndex,
    int clusterId)
{
    // --------------------------------------------------------
    // Check whether initial point is a core point
    // --------------------------------------------------------

    size_t neighborCount =
        countNeighbors(pointIndex);

    if (neighborCount < MIN_PTS)
    {
        labels[pointIndex] = NOISE;

        return true;
    }

    // --------------------------------------------------------
    // Start queue
    // --------------------------------------------------------

    size_t queueSize = 0;
    size_t queueIndex = 0;

    labels[pointIndex] = clusterId;

    // Add initial neighbors
    queueSize =
        addNeighborsToQueue(
            pointIndex,
            clusterId,
            queueSize
        );

    // --------------------------------------------------------
    // Process queue
    // --------------------------------------------------------

    while (queueIndex < queueSize)
    {
        size_t currentPoint =
            queueBuffer[queueIndex];

        queueIndex++;

        size_t currentNeighborCount =
            countNeighbors(currentPoint);

        // ----------------------------------------------------
        // If this point is a core point,
        // expand the cluster.
        // ----------------------------------------------------

        if (currentNeighborCount >= MIN_PTS)
        {
            queueSize =
                addNeighborsToQueue(
                    currentPoint,
                    clusterId,
                    queueSize
                );
        }
    }

    return true;
}

// ============================================================
// DBSCAN
// ============================================================

bool dbscan()
{
    int clusterId = 0;

    for (size_t i = 0; i < currentN; i++)
    {
        // Already processed
        if (labels[i] != UNCLASSIFIED)
        {
            continue;
        }

        // Expand cluster
        bool success =
            expandCluster(
                i,
                clusterId
            );

        if (!success)
        {
            return false;
        }

        // Determine whether this point
        // actually created a cluster.
        //
        // If it became NOISE, don't increment.
        if (labels[i] == clusterId)
        {
            clusterId++;
        }
    }

    return true;
}

// ============================================================
// Count clusters
// ============================================================

int countClusters()
{
    int maxCluster = -1;

    for (size_t i = 0; i < currentN; i++)
    {
        if (labels[i] >= 0 &&
            labels[i] > maxCluster)
        {
            maxCluster = labels[i];
        }
    }

    return maxCluster + 1;
}

// ============================================================
// Count noise
// ============================================================

size_t countNoise()
{
    size_t noise = 0;

    for (size_t i = 0; i < currentN; i++)
    {
        if (labels[i] == NOISE)
        {
            noise++;
        }
    }

    return noise;
}

// ============================================================
// Generate deterministic 3D dataset
// ============================================================

void generateDataset(size_t N)
{
    for (size_t i = 0; i < N; i++)
    {
        float noiseX =
            ((float)((i * 17) % 100) / 100.0f) - 0.5f;

        float noiseY =
            ((float)((i * 31) % 100) / 100.0f) - 0.5f;

        float noiseZ =
            ((float)((i * 47) % 100) / 100.0f) - 0.5f;

        int cluster =
            i % 3;

        if (cluster == 0)
        {
            points[i].x = noiseX;
            points[i].y = noiseY;
            points[i].z = noiseZ;
        }
        else if (cluster == 1)
        {
            points[i].x = 5.0f + noiseX;
            points[i].y = 5.0f + noiseY;
            points[i].z = 5.0f + noiseZ;
        }
        else
        {
            points[i].x = 10.0f + noiseX;
            points[i].y = noiseY;
            points[i].z = 5.0f + noiseZ;
        }

        labels[i] = UNCLASSIFIED;
    }
}

// ============================================================
// Allocate benchmark memory
// ============================================================

bool allocateMemory(size_t N)
{
    size_t pointMemory =
        N * sizeof(Point3D);

    size_t labelMemory =
        N * sizeof(int);

    size_t queueMemory =
        N * sizeof(int);

    points =
        (Point3D *)heap_caps_malloc(
            pointMemory,
            MALLOC_CAP_SPIRAM
        );

    labels =
        (int *)heap_caps_malloc(
            labelMemory,
            MALLOC_CAP_SPIRAM
        );

    queueBuffer =
        (int *)heap_caps_malloc(
            queueMemory,
            MALLOC_CAP_SPIRAM
        );

    if (points == nullptr ||
        labels == nullptr ||
        queueBuffer == nullptr)
    {
        Serial.println(
            "ERROR: PSRAM allocation failed."
        );

        return false;
    }

    return true;
}

// ============================================================
// Free benchmark memory
// ============================================================

void freeMemory()
{
    if (points != nullptr)
    {
        heap_caps_free(points);
        points = nullptr;
    }

    if (labels != nullptr)
    {
        heap_caps_free(labels);
        labels = nullptr;
    }

    if (queueBuffer != nullptr)
    {
        heap_caps_free(queueBuffer);
        queueBuffer = nullptr;
    }

    currentN = 0;
}

// ============================================================
// Run benchmark
// ============================================================

bool runBenchmark(size_t N)
{
    currentN = N;

    Serial.println();
    Serial.println(
        "--------------------------------------------------------"
    );

    Serial.printf(
        "Testing N = %u\n",
        (unsigned)N
    );

    // --------------------------------------------------------
    // Allocate
    // --------------------------------------------------------

    if (!allocateMemory(N))
    {
        freeMemory();

        return false;
    }

    size_t pointMemory =
        N * sizeof(Point3D);

    size_t labelMemory =
        N * sizeof(int);

    size_t queueMemory =
        N * sizeof(int);

    size_t totalMemory =
        pointMemory +
        labelMemory +
        queueMemory;

    Serial.printf(
        "Points memory: %.2f KB\n",
        pointMemory / 1024.0f
    );

    Serial.printf(
        "Labels memory: %.2f KB\n",
        labelMemory / 1024.0f
    );

    Serial.printf(
        "Queue memory:  %.2f KB\n",
        queueMemory / 1024.0f
    );

    Serial.printf(
        "Total PSRAM:   %.2f KB\n",
        totalMemory / 1024.0f
    );

    // --------------------------------------------------------
    // Generate dataset
    // --------------------------------------------------------

    generateDataset(N);

    Serial.println(
        "Dataset generated."
    );

    // --------------------------------------------------------
    // Memory before
    // --------------------------------------------------------

    size_t freeHeapBefore =
        ESP.getFreeHeap();

    size_t freePsramBefore =
        ESP.getFreePsram();

    Serial.printf(
        "Free heap before:  %u KB\n",
        (unsigned)(freeHeapBefore / 1024)
    );

    Serial.printf(
        "Free PSRAM before: %u KB\n",
        (unsigned)(freePsramBefore / 1024)
    );

    // --------------------------------------------------------
    // Run DBSCAN
    // --------------------------------------------------------

    unsigned long startTime =
        micros();

    bool success =
        dbscan();

    unsigned long endTime =
        micros();

    if (!success)
    {
        Serial.println(
            "DBSCAN FAILED."
        );

        freeMemory();

        return false;
    }

    unsigned long elapsedUs =
        endTime - startTime;

    float elapsedMs =
        elapsedUs / 1000.0f;

    // --------------------------------------------------------
    // Results
    // --------------------------------------------------------

    int clusters =
        countClusters();

    size_t noise =
        countNoise();

    size_t freeHeapAfter =
        ESP.getFreeHeap();

    size_t freePsramAfter =
        ESP.getFreePsram();

    Serial.println();
    Serial.println(
        "                    RESULTS"
    );

    Serial.printf(
        "N:              %u\n",
        (unsigned)N
    );

    Serial.printf(
        "Dimensions:     %d\n",
        DIMENSIONS
    );

    Serial.printf(
        "Epsilon:        %.2f\n",
        EPSILON
    );

    Serial.printf(
        "MinPts:         %d\n",
        MIN_PTS
    );

    Serial.printf(
        "Clusters:       %d\n",
        clusters
    );

    Serial.printf(
        "Noise points:   %u\n",
        (unsigned)noise
    );

    Serial.printf(
        "Execution:      %.3f ms\n",
        elapsedMs
    );

    Serial.printf(
        "Free heap:      %u KB\n",
        (unsigned)(freeHeapAfter / 1024)
    );

    Serial.printf(
        "Free PSRAM:     %u KB\n",
        (unsigned)(freePsramAfter / 1024)
    );

    Serial.println(
        "--------------------------------------------------------"
    );

    // --------------------------------------------------------
    // Free
    // --------------------------------------------------------

    freeMemory();

    return true;
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
        "========================================================"
    );
    Serial.println(
        "       ESP32-S3 N16R8 - DBSCAN BENCHMARK"
    );
    Serial.println(
        "========================================================"
    );

    Serial.println();

    // --------------------------------------------------------
    // Hardware
    // --------------------------------------------------------

    Serial.println(
        "[ HARDWARE ]"
    );

    Serial.printf(
        "CPU: ESP32-S3\n"
    );

    Serial.printf(
        "CPU frequency: %lu MHz\n",
        ESP.getCpuFreqMHz()
    );

    Serial.printf(
        "Flash: %u MB\n",
        ESP.getFlashChipSize() /
        (1024 * 1024)
    );

    Serial.printf(
        "PSRAM: %u MB\n",
        ESP.getPsramSize() /
        (1024 * 1024)
    );

    Serial.printf(
        "Free internal heap: %u KB\n",
        ESP.getFreeHeap() / 1024
    );

    Serial.printf(
        "Free PSRAM: %u KB\n",
        ESP.getFreePsram() / 1024
    );

    Serial.println();

    // --------------------------------------------------------
    // Configuration
    // --------------------------------------------------------

    Serial.println(
        "[ DBSCAN CONFIGURATION ]"
    );

    Serial.printf(
        "Dimensions: %d\n",
        DIMENSIONS
    );

    Serial.printf(
        "Epsilon: %.2f\n",
        EPSILON
    );

    Serial.printf(
        "MinPts: %d\n",
        MIN_PTS
    );

    Serial.println();

    // --------------------------------------------------------
    // Benchmark
    // --------------------------------------------------------

    Serial.println(
        "[ STARTING BENCHMARK ]"
    );

    for (int i = 0;
         i < NUM_TESTS;
         i++)
    {
        bool success =
            runBenchmark(
                TEST_SIZES[i]
            );

        if (!success)
        {
            Serial.println();
            Serial.println(
                "Benchmark stopped."
            );

            break;
        }

        delay(500);
    }

    // --------------------------------------------------------
    // Complete
    // --------------------------------------------------------

    Serial.println();
    Serial.println(
        "========================================================"
    );
    Serial.println(
        "              BENCHMARK COMPLETE"
    );
    Serial.println(
        "========================================================"
    );
}

// ============================================================
// Loop
// ============================================================

void loop()
{
}