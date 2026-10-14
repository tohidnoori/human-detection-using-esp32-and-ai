#include <Arduino.h>
#include <math.h>

// ============================================================
// DBSCAN - EXPERIMENT 03
// MinPts Sensitivity
//
// ESP32-S3 N16R8
//
// Fixed:
//   N       = 1440
//   DIM     = 3
//   Epsilon = 0.20
//
// Variable:
//   MinPts = 2, 5, 10, 20, 50, 100, 200
//
// Purpose:
//   Study how MinPts affects:
//   - core points
//   - border points
//   - noise points
//   - number of clusters
//   - cluster sizes
//   - execution time
//   - memory usage
// ============================================================

#define DIMENSIONS 3
#define DATASET_SIZE 1440

#define EPSILON 0.20f

#define UNCLASSIFIED 0
#define NOISE        -1

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
// PSRAM buffers
// ============================================================

Point3D *points = nullptr;
int *labels = nullptr;
int *queueBuffer = nullptr;

// ============================================================
// Deterministic random generator
// ============================================================

uint32_t rngState = 123456789;

uint32_t fastRandom()
{
    rngState =
        rngState * 1664525UL +
        1013904223UL;

    return rngState;
}

float randomFloat(
    float minValue,
    float maxValue)
{
    uint32_t value =
        fastRandom();

    float normalized =
        (float)(value & 0x00FFFFFF) /
        16777215.0f;

    return minValue +
           normalized *
           (maxValue - minValue);
}

// ============================================================
// Dataset generation
//
// Three clusters.
// Each cluster contains 480 points.
//
// Compared with the previous experiment, the cluster radius
// is increased so that epsilon=0.20 produces a meaningful
// density transition.
//
// Cluster 1: around (0, 0, 0)
// Cluster 2: around (5, 5, 5)
// Cluster 3: around (10, 0, 5)
// ============================================================

void generateDataset()
{
    rngState = 123456789;

    const int pointsPerCluster =
        DATASET_SIZE / 3;

    const float radius = 0.60f;

    for (int i = 0;
         i < DATASET_SIZE;
         i++)
    {
        int cluster =
            i / pointsPerCluster;

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

        points[i].x =
            cx + randomFloat(
                -radius,
                radius);

        points[i].y =
            cy + randomFloat(
                -radius,
                radius);

        points[i].z =
            cz + randomFloat(
                -radius,
                radius);
    }
}

// ============================================================
// Squared Euclidean distance
// ============================================================

inline float squaredDistance(
    const Point3D &a,
    const Point3D &b)
{
    float dx =
        a.x - b.x;

    float dy =
        a.y - b.y;

    float dz =
        a.z - b.z;

    return dx * dx +
           dy * dy +
           dz * dz;
}

// ============================================================
// Count neighbors
// ============================================================

int countNeighbors(
    int pointIndex,
    float epsilonSquared)
{
    int count = 0;

    for (int i = 0;
         i < DATASET_SIZE;
         i++)
    {
        if (squaredDistance(
                points[pointIndex],
                points[i])
            <= epsilonSquared)
        {
            count++;
        }
    }

    return count;
}

// ============================================================
// Add neighbors to queue
// ============================================================

int addNeighborsToQueue(
    int pointIndex,
    int clusterId,
    float epsilonSquared,
    int queueSize)
{
    for (int i = 0;
         i < DATASET_SIZE;
         i++)
    {
        if (squaredDistance(
                points[pointIndex],
                points[i])
            <= epsilonSquared)
        {
            if (labels[i] ==
                UNCLASSIFIED)
            {
                labels[i] =
                    clusterId;

                if (queueSize <
                    DATASET_SIZE)
                {
                    queueBuffer[
                        queueSize] = i;

                    queueSize++;
                }
            }
        }
    }

    return queueSize;
}

// ============================================================
// Expand cluster
// ============================================================

void expandCluster(
    int startPoint,
    int clusterId,
    int minPts,
    float epsilon,
    int &queueSize)
{
    float epsilonSquared =
        epsilon * epsilon;

    queueSize = 0;

    labels[startPoint] =
        clusterId;

    queueBuffer[queueSize++] =
        startPoint;

    int queueIndex = 0;

    while (queueIndex <
           queueSize)
    {
        int currentPoint =
            queueBuffer[queueIndex++];

        int neighborCount =
            countNeighbors(
                currentPoint,
                epsilonSquared);

        if (neighborCount >=
            minPts)
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

// ============================================================
// DBSCAN
// ============================================================

int dbscan(
    float epsilon,
    int minPts)
{
    for (int i = 0;
         i < DATASET_SIZE;
         i++)
    {
        labels[i] =
            UNCLASSIFIED;
    }

    int clusterId = 0;

    int queueSize = 0;

    float epsilonSquared =
        epsilon * epsilon;

    for (int i = 0;
         i < DATASET_SIZE;
         i++)
    {
        if (labels[i] !=
            UNCLASSIFIED)
        {
            continue;
        }

        int neighborCount =
            countNeighbors(
                i,
                epsilonSquared);

        if (neighborCount <
            minPts)
        {
            labels[i] =
                NOISE;

            continue;
        }

        clusterId++;

        expandCluster(
            i,
            clusterId,
            minPts,
            epsilon,
            queueSize);
    }

    return clusterId;
}

// ============================================================
// Analyze point types
//
// Important:
//
// DBSCAN labels alone cannot directly distinguish border
// points from core points.
//
// We therefore perform an additional neighborhood analysis
// after clustering.
//
// Core:
//   neighborhood >= MinPts
//
// Border:
//   non-core point belonging to a cluster
//
// Noise:
//   label == NOISE
// ============================================================

void analyzePointTypes(
    int minPts,
    int &corePoints,
    int &borderPoints,
    int &noisePoints)
{
    corePoints = 0;
    borderPoints = 0;
    noisePoints = 0;

    float epsilonSquared =
        EPSILON * EPSILON;

    for (int i = 0;
         i < DATASET_SIZE;
         i++)
    {
        if (labels[i] ==
            NOISE)
        {
            noisePoints++;
            continue;
        }

        int neighborCount =
            countNeighbors(
                i,
                epsilonSquared);

        if (neighborCount >=
            minPts)
        {
            corePoints++;
        }
        else
        {
            borderPoints++;
        }
    }
}

// ============================================================
// Print cluster sizes
// ============================================================

void printClusterSizes(
    int clusterCount)
{
    Serial.println();
    Serial.println(
        "Cluster sizes:");

    for (int cluster = 1;
         cluster <= clusterCount;
         cluster++)
    {
        int count = 0;

        for (int i = 0;
             i < DATASET_SIZE;
             i++)
        {
            if (labels[i] ==
                cluster)
            {
                count++;
            }
        }

        Serial.printf(
            "  Cluster %d: %d points\n",
            cluster,
            count);
    }
}

// ============================================================
// Print memory information
// ============================================================

void printMemoryInfo()
{
    size_t pointsMemory =
        sizeof(Point3D) *
        DATASET_SIZE;

    size_t labelsMemory =
        sizeof(int) *
        DATASET_SIZE;

    size_t queueMemory =
        sizeof(int) *
        DATASET_SIZE;

    size_t totalMemory =
        pointsMemory +
        labelsMemory +
        queueMemory;

    Serial.println();
    Serial.println(
        "Memory:");

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

// ============================================================
// Run one experiment
// ============================================================

void runExperiment(
    int minPts)
{
    Serial.println();
    Serial.println(
        "========================================");

    Serial.printf(
        "MINPTS = %d\n",
        minPts);

    Serial.println(
        "========================================");

    // --------------------------------------------------------
    // Generate identical dataset
    // --------------------------------------------------------

    generateDataset();

    // --------------------------------------------------------
    // Reset labels
    // --------------------------------------------------------

    for (int i = 0;
         i < DATASET_SIZE;
         i++)
    {
        labels[i] =
            UNCLASSIFIED;
    }

    size_t freeHeapBefore =
        ESP.getFreeHeap();

    size_t freePsramBefore =
        ESP.getFreePsram();

    // --------------------------------------------------------
    // DBSCAN
    // --------------------------------------------------------

    uint64_t startTime =
        micros();

    int clusterCount =
        dbscan(
            EPSILON,
            minPts);

    uint64_t endTime =
        micros();

    uint64_t executionTime =
        endTime -
        startTime;

    // --------------------------------------------------------
    // Analyze result
    // --------------------------------------------------------

    int corePoints;
    int borderPoints;
    int noisePoints;

    analyzePointTypes(
        minPts,
        corePoints,
        borderPoints,
        noisePoints);

    size_t freeHeapAfter =
        ESP.getFreeHeap();

    size_t freePsramAfter =
        ESP.getFreePsram();

    // --------------------------------------------------------
    // Print results
    // --------------------------------------------------------

    Serial.printf(
        "Epsilon: %.2f\n",
        EPSILON);

    Serial.printf(
        "MinPts: %d\n",
        minPts);

    Serial.printf(
        "Execution time: %.3f ms\n",
        executionTime / 1000.0);

    Serial.printf(
        "Clusters: %d\n",
        clusterCount);

    Serial.printf(
        "Core points: %d\n",
        corePoints);

    Serial.printf(
        "Border points: %d\n",
        borderPoints);

    Serial.printf(
        "Noise points: %d\n",
        noisePoints);

    Serial.printf(
        "Classification check: %d + %d + %d = %d\n",
        corePoints,
        borderPoints,
        noisePoints,
        corePoints +
        borderPoints +
        noisePoints);

    printClusterSizes(
        clusterCount);

    Serial.printf(
        "Free heap before: %u KB\n",
        (unsigned int)(
            freeHeapBefore / 1024));

    Serial.printf(
        "Free heap after: %u KB\n",
        (unsigned int)(
            freeHeapAfter / 1024));

    Serial.printf(
        "Free PSRAM before: %u KB\n",
        (unsigned int)(
            freePsramBefore / 1024));

    Serial.printf(
        "Free PSRAM after: %u KB\n",
        (unsigned int)(
            freePsramAfter / 1024));
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
        "========================================");

    Serial.println(
        "DBSCAN - EXPERIMENT 03");

    Serial.println(
        "MINPTS SENSITIVITY");

    Serial.println(
        "ESP32-S3 N16R8");

    Serial.println(
        "========================================");

    Serial.printf(
        "Dataset size: %d\n",
        DATASET_SIZE);

    Serial.printf(
        "Dimensions: %d\n",
        DIMENSIONS);

    Serial.printf(
        "Epsilon: %.2f\n",
        EPSILON);

    Serial.println(
        "MinPts values: "
        "2, 5, 10, 20, 50, 100, 200");

    Serial.printf(
        "Free heap: %u KB\n",
        (unsigned int)(
            ESP.getFreeHeap() / 1024));

    Serial.printf(
        "Free PSRAM: %u KB\n",
        (unsigned int)(
            ESP.getFreePsram() / 1024));

    // --------------------------------------------------------
    // Allocate PSRAM
    // --------------------------------------------------------

    points =
        (Point3D *)ps_malloc(
            sizeof(Point3D) *
            DATASET_SIZE);

    labels =
        (int *)ps_malloc(
            sizeof(int) *
            DATASET_SIZE);

    queueBuffer =
        (int *)ps_malloc(
            sizeof(int) *
            DATASET_SIZE);

    if (points == nullptr ||
        labels == nullptr ||
        queueBuffer == nullptr)
    {
        Serial.println();
        Serial.println(
            "ERROR: PSRAM allocation failed!");

        while (true)
        {
            delay(1000);
        }
    }

    Serial.println();
    Serial.println(
        "PSRAM allocation successful.");

    printMemoryInfo();

    // --------------------------------------------------------
    // MinPts values
    // --------------------------------------------------------

    const int minPtsValues[] =
    {
        2,
        5,
        10,
        20,
        50,
        100,
        200
    };

    const int minPtsCount =
        sizeof(minPtsValues) /
        sizeof(minPtsValues[0]);

    // --------------------------------------------------------
    // Run
    // --------------------------------------------------------

    Serial.println();
    Serial.println(
        "========================================");

    Serial.println(
        "STARTING MINPTS EXPERIMENTS");

    Serial.println(
        "========================================");

    for (int i = 0;
         i < minPtsCount;
         i++)
    {
        runExperiment(
            minPtsValues[i]);

        delay(500);
    }

    // --------------------------------------------------------
    // Complete
    // --------------------------------------------------------

    Serial.println();
    Serial.println(
        "========================================");

    Serial.println(
        "EXPERIMENT 03 COMPLETE");

    Serial.println(
        "========================================");
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    delay(1000);
}