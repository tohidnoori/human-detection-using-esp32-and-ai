#include <Arduino.h>
#include <math.h>

// ============================================================
// Experiment 02 - DBSCAN Epsilon Sensitivity
// ESP32-S3 N16R8
//
// Fixed:
//   N       = 1440
//   DIM     = 3
//   MinPts  = 5
//
// Variable:
//   Epsilon = 0.10, 0.20, 0.30, 0.50,
//             0.80, 1.00, 1.50
//
// Purpose:
//   Analyze how epsilon affects DBSCAN clustering,
//   noise points, number of clusters and execution time.
// ============================================================

#define DIMENSIONS 3
#define DATASET_SIZE 1440
#define MIN_PTS 5

// Labels
#define UNCLASSIFIED 0
#define NOISE        -1

// ------------------------------------------------------------
// Point structure
// ------------------------------------------------------------

struct Point3D
{
    float x;
    float y;
    float z;
};

// ------------------------------------------------------------
// Global buffers
// Allocated in PSRAM
// ------------------------------------------------------------

Point3D *points = nullptr;
int *labels = nullptr;
int *queueBuffer = nullptr;

// ------------------------------------------------------------
// Deterministic pseudo-random generator
// ------------------------------------------------------------

uint32_t rngState = 123456789;

uint32_t fastRandom()
{
    rngState = rngState * 1664525UL + 1013904223UL;
    return rngState;
}

float randomFloat(float minValue, float maxValue)
{
    uint32_t value = fastRandom();

    float normalized =
        (float)(value & 0x00FFFFFF) / 16777215.0f;

    return minValue +
           normalized * (maxValue - minValue);
}

// ------------------------------------------------------------
// Generate deterministic 3D dataset
//
// Three compact clusters:
//
// Cluster 0 -> around (0, 0, 0)
// Cluster 1 -> around (5, 5, 5)
// Cluster 2 -> around (10, 0, 5)
//
// Each cluster contains 480 points.
//
// The dataset is regenerated before each epsilon test so that
// every epsilon sees exactly the same dataset.
// ------------------------------------------------------------

void generateDataset()
{
    rngState = 123456789;

    const int pointsPerCluster = DATASET_SIZE / 3;

    for (int i = 0; i < DATASET_SIZE; i++)
    {
        int cluster = i / pointsPerCluster;

        float cx;
        float cy;
        float cz;

        if (cluster == 0)
        {
            cx = 0.0f;
            cy = 0.0f;
            cz = 0.0f;
        }
        else if (cluster == 1)
        {
            cx = 5.0f;
            cy = 5.0f;
            cz = 5.0f;
        }
        else
        {
            cx = 10.0f;
            cy = 0.0f;
            cz = 5.0f;
        }

        // Compact deterministic distribution
        points[i].x = cx + randomFloat(-0.25f, 0.25f);
        points[i].y = cy + randomFloat(-0.25f, 0.25f);
        points[i].z = cz + randomFloat(-0.25f, 0.25f);
    }
}

// ------------------------------------------------------------
// Squared Euclidean distance
//
// Avoids sqrt(), which makes the benchmark faster and cleaner.
// Compare against epsilon^2 instead.
// ------------------------------------------------------------

inline float squaredDistance(
    const Point3D &a,
    const Point3D &b)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;

    return dx * dx +
           dy * dy +
           dz * dz;
}

// ------------------------------------------------------------
// Count neighbors
// ------------------------------------------------------------

int countNeighbors(
    int pointIndex,
    float epsilonSquared)
{
    int count = 0;

    for (int i = 0; i < DATASET_SIZE; i++)
    {
        if (squaredDistance(
                points[pointIndex],
                points[i]) <= epsilonSquared)
        {
            count++;
        }
    }

    return count;
}

// ------------------------------------------------------------
// Add neighbors to queue
//
// Only unclassified points are inserted.
// A point already marked NOISE can be converted into the
// current cluster during expansion.
// ------------------------------------------------------------

int addNeighborsToQueue(
    int pointIndex,
    int clusterId,
    float epsilonSquared,
    int queueSize)
{
    for (int i = 0; i < DATASET_SIZE; i++)
    {
        if (squaredDistance(
                points[pointIndex],
                points[i]) <= epsilonSquared)
        {
            if (labels[i] == UNCLASSIFIED)
            {
                labels[i] = clusterId;

                if (queueSize < DATASET_SIZE)
                {
                    queueBuffer[queueSize] = i;
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

void expandCluster(
    int startPoint,
    int clusterId,
    float epsilon,
    int &queueSize)
{
    float epsilonSquared = epsilon * epsilon;

    queueSize = 0;

    labels[startPoint] = clusterId;

    queueBuffer[queueSize++] = startPoint;

    int queueIndex = 0;

    while (queueIndex < queueSize)
    {
        int currentPoint = queueBuffer[queueIndex++];

        int neighborCount =
            countNeighbors(
                currentPoint,
                epsilonSquared);

        if (neighborCount >= MIN_PTS)
        {
            queueSize =
                addNeighborsToQueue(
                    currentPoint,
                    clusterId,
                    epsilonSquared,
                    queueSize);
        }
    }
}

// ------------------------------------------------------------
// DBSCAN
// ------------------------------------------------------------

int dbscan(float epsilon)
{
    // Reset labels
    for (int i = 0; i < DATASET_SIZE; i++)
    {
        labels[i] = UNCLASSIFIED;
    }

    int clusterId = 0;

    int queueSize = 0;

    for (int i = 0; i < DATASET_SIZE; i++)
    {
        if (labels[i] != UNCLASSIFIED)
        {
            continue;
        }

        float epsilonSquared =
            epsilon * epsilon;

        int neighborCount =
            countNeighbors(
                i,
                epsilonSquared);

        if (neighborCount < MIN_PTS)
        {
            labels[i] = NOISE;
            continue;
        }

        clusterId++;

        expandCluster(
            i,
            clusterId,
            epsilon,
            queueSize);
    }

    return clusterId;
}

// ------------------------------------------------------------
// Count noise
// ------------------------------------------------------------

int countNoise()
{
    int noise = 0;

    for (int i = 0; i < DATASET_SIZE; i++)
    {
        if (labels[i] == NOISE)
        {
            noise++;
        }
    }

    return noise;
}

// ------------------------------------------------------------
// Memory information
// ------------------------------------------------------------

void printMemoryInfo()
{
    size_t pointsMemory =
        sizeof(Point3D) * DATASET_SIZE;

    size_t labelsMemory =
        sizeof(int) * DATASET_SIZE;

    size_t queueMemory =
        sizeof(int) * DATASET_SIZE;

    size_t totalMemory =
        pointsMemory +
        labelsMemory +
        queueMemory;

    Serial.println();
    Serial.println("Memory:");
    Serial.printf(
        "  Points: %.2f KB\n",
        pointsMemory / 1024.0f);

    Serial.printf(
        "  Labels: %.2f KB\n",
        labelsMemory / 1024.0f);

    Serial.printf(
        "  Queue: %.2f KB\n",
        queueMemory / 1024.0f);

    Serial.printf(
        "  Total PSRAM: %.2f KB\n",
        totalMemory / 1024.0f);
}

// ------------------------------------------------------------
// Run one epsilon experiment
// ------------------------------------------------------------

void runExperiment(float epsilon)
{
    Serial.println();
    Serial.println("========================================");
    Serial.printf(
        "EPSILON = %.2f\n",
        epsilon);
    Serial.println("========================================");

    // Always use exactly the same dataset
    generateDataset();

    // Reset labels
    for (int i = 0; i < DATASET_SIZE; i++)
    {
        labels[i] = UNCLASSIFIED;
    }

    size_t freeHeapBefore =
        ESP.getFreeHeap();

    size_t freePsramBefore =
        ESP.getFreePsram();

    uint64_t startTime =
        micros();

    int clusterCount =
        dbscan(epsilon);

    uint64_t endTime =
        micros();

    int noiseCount =
        countNoise();

    uint64_t executionTime =
        endTime - startTime;

    size_t freeHeapAfter =
        ESP.getFreeHeap();

    size_t freePsramAfter =
        ESP.getFreePsram();

    Serial.printf(
        "Execution time: %.3f ms\n",
        executionTime / 1000.0);

    Serial.printf(
        "Clusters: %d\n",
        clusterCount);

    Serial.printf(
        "Noise points: %d\n",
        noiseCount);

    Serial.printf(
        "Free heap before: %u KB\n",
        (unsigned int)(freeHeapBefore / 1024));

    Serial.printf(
        "Free heap after: %u KB\n",
        (unsigned int)(freeHeapAfter / 1024));

    Serial.printf(
        "Free PSRAM before: %u KB\n",
        (unsigned int)(freePsramBefore / 1024));

    Serial.printf(
        "Free PSRAM after: %u KB\n",
        (unsigned int)(freePsramAfter / 1024));
}

// ------------------------------------------------------------
// Setup
// ------------------------------------------------------------

void setup()
{
    Serial.begin(115200);

    delay(2000);

    Serial.println();
    Serial.println("========================================");
    Serial.println("DBSCAN - EXPERIMENT 02");
    Serial.println("EPSILON SENSITIVITY");
    Serial.println("ESP32-S3 N16R8");
    Serial.println("========================================");

    Serial.printf(
        "Dataset size: %d\n",
        DATASET_SIZE);

    Serial.printf(
        "Dimensions: %d\n",
        DIMENSIONS);

    Serial.printf(
        "MinPts: %d\n",
        MIN_PTS);

    Serial.printf(
        "Free heap: %u KB\n",
        (unsigned int)(ESP.getFreeHeap() / 1024));

    Serial.printf(
        "Free PSRAM: %u KB\n",
        (unsigned int)(ESP.getFreePsram() / 1024));

    // --------------------------------------------------------
    // Allocate buffers in PSRAM
    // --------------------------------------------------------

    points = (Point3D *)ps_malloc(
        sizeof(Point3D) * DATASET_SIZE);

    labels = (int *)ps_malloc(
        sizeof(int) * DATASET_SIZE);

    queueBuffer = (int *)ps_malloc(
        sizeof(int) * DATASET_SIZE);

    if (points == nullptr ||
        labels == nullptr ||
        queueBuffer == nullptr)
    {
        Serial.println();
        Serial.println("ERROR: PSRAM allocation failed!");

        while (true)
        {
            delay(1000);
        }
    }

    Serial.println();
    Serial.println("PSRAM allocation successful.");

    printMemoryInfo();

    // --------------------------------------------------------
    // Epsilon experiments
    // --------------------------------------------------------

    const float epsilonValues[] =
    {
        0.10f,
        0.20f,
        0.30f,
        0.50f,
        0.80f,
        1.00f,
        1.50f
    };

    const int epsilonCount =
        sizeof(epsilonValues) /
        sizeof(epsilonValues[0]);

    Serial.println();
    Serial.println("========================================");
    Serial.println("STARTING EPSILON EXPERIMENTS");
    Serial.println("========================================");

    for (int i = 0; i < epsilonCount; i++)
    {
        runExperiment(
            epsilonValues[i]);

        delay(500);
    }

    // --------------------------------------------------------
    // Final summary
    // --------------------------------------------------------

    Serial.println();
    Serial.println("========================================");
    Serial.println("EXPERIMENT 02 COMPLETE");
    Serial.println("========================================");
}

// ------------------------------------------------------------
// Loop
// ------------------------------------------------------------

void loop()
{
    delay(1000);
}