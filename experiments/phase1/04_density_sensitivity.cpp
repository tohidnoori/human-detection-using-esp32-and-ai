#include <Arduino.h>
#include <esp_heap_caps.h>

// ============================================================
// Experiment 04 — Density Sensitivity Analysis
// ============================================================
//
// Objective:
//   Study how DBSCAN behaves when the density of the dataset
//   changes while N, epsilon and MinPts remain constant.
//
// Fixed:
//   N         = 1440
//   Dimensions= 3
//   Epsilon   = 0.20
//   MinPts    = 5
//
// Variable:
//   Dense     radius = 0.25
//   Medium    radius = 0.60
//   Sparse    radius = 1.20
//
// Memory:
//   Points = N * 3 * sizeof(float)
//   Labels = N * sizeof(int)
//   Queue  = N * sizeof(int)
//
// ============================================================


// ------------------------------------------------------------
// Configuration
// ------------------------------------------------------------

const int N = 1440;
const int DIMENSIONS = 3;

const float EPSILON = 0.20f;
const int MINPTS = 5;

const float DENSE_RADIUS  = 0.25f;
const float MEDIUM_RADIUS = 0.60f;
const float SPARSE_RADIUS = 1.20f;


// ------------------------------------------------------------
// DBSCAN labels
// ------------------------------------------------------------

const int UNCLASSIFIED = -1;
const int NOISE = -2;


// ------------------------------------------------------------
// 3D point
// ------------------------------------------------------------

struct Point3D
{
    float x;
    float y;
    float z;
};


// ------------------------------------------------------------
// PSRAM buffers
// ------------------------------------------------------------

Point3D* points = nullptr;
int* labels = nullptr;
int* queueBuffer = nullptr;


// ------------------------------------------------------------
// Deterministic random generator
// ------------------------------------------------------------

uint32_t randomState = 123456789;


uint32_t nextRandom()
{
    randomState =
        randomState * 1664525UL +
        1013904223UL;

    return randomState;
}


float randomFloat(float minValue, float maxValue)
{
    uint32_t value = nextRandom();

    float normalized =
        (float)value / 4294967295.0f;

    return minValue +
           normalized * (maxValue - minValue);
}


// ------------------------------------------------------------
// Generate deterministic dataset
// ------------------------------------------------------------
//
// Three clusters:
//
// Cluster 1 -> (0, 0, 0)
// Cluster 2 -> (5, 5, 5)
// Cluster 3 -> (10, 0, 5)
//
// The same random pattern is used for each density.
// Only the radius changes.
//
// This makes the experiment more controlled because the
// geometry is comparable across density levels.
// ------------------------------------------------------------

void generateDataset(float radius)
{
    randomState = 123456789;

    const int pointsPerCluster = N / 3;

    for (int i = 0; i < N; i++)
    {
        int cluster = i / pointsPerCluster;

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
            randomFloat(-radius, radius);

        points[i].y =
            centerY +
            randomFloat(-radius, radius);

        points[i].z =
            centerZ +
            randomFloat(-radius, radius);

        labels[i] = UNCLASSIFIED;
    }
}


// ------------------------------------------------------------
// Squared Euclidean distance
// ------------------------------------------------------------
//
// Avoid sqrt() because DBSCAN only needs to know whether:
//
// distance <= epsilon
//
// Therefore:
//
// distance² <= epsilon²
// ------------------------------------------------------------

float squaredDistance(
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


// ------------------------------------------------------------
// Count neighbors
// ------------------------------------------------------------

int countNeighbors(int pointIndex)
{
    int count = 0;

    float epsilonSquared =
        EPSILON * EPSILON;

    for (int i = 0; i < N; i++)
    {
        if (i == pointIndex)
            continue;

        if (
            squaredDistance(
                points[pointIndex],
                points[i]
            ) <= epsilonSquared
        )
        {
            count++;
        }
    }

    return count;
}


// ------------------------------------------------------------
// Add neighbors to queue
// ------------------------------------------------------------

int addNeighborsToQueue(
    int pointIndex,
    int queueSize
)
{
    float epsilonSquared =
        EPSILON * EPSILON;

    for (int i = 0; i < N; i++)
    {
        if (i == pointIndex)
            continue;

        if (
            squaredDistance(
                points[pointIndex],
                points[i]
            ) <= epsilonSquared
        )
        {
            if (labels[i] == UNCLASSIFIED)
            {
                labels[i] = 0;

                if (queueSize < N)
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
    int pointIndex,
    int clusterId
)
{
    int queueSize = 0;

    queueSize =
        addNeighborsToQueue(
            pointIndex,
            queueSize
        );

    labels[pointIndex] = clusterId;

    int queueIndex = 0;

    while (queueIndex < queueSize)
    {
        int currentPoint =
            queueBuffer[queueIndex];

        queueIndex++;

        if (labels[currentPoint] == NOISE)
        {
            labels[currentPoint] =
                clusterId;
        }

        if (labels[currentPoint] != 0)
        {
            continue;
        }

        labels[currentPoint] =
            clusterId;

        int neighbors =
            countNeighbors(currentPoint);

        if (neighbors >= MINPTS)
        {
            queueSize =
                addNeighborsToQueue(
                    currentPoint,
                    queueSize
                );
        }
    }
}


// ------------------------------------------------------------
// DBSCAN
// ------------------------------------------------------------

int dbscan()
{
    int clusterId = 0;

    for (int i = 0; i < N; i++)
    {
        if (labels[i] != UNCLASSIFIED)
            continue;

        int neighbors =
            countNeighbors(i);

        if (neighbors < MINPTS)
        {
            labels[i] = NOISE;
        }
        else
        {
            clusterId++;

            expandCluster(
                i,
                clusterId
            );
        }
    }

    return clusterId;
}


// ------------------------------------------------------------
// Analyze result
// ------------------------------------------------------------

void analyzeResult(
    int clusterCount,
    int& corePoints,
    int& borderPoints,
    int& noisePoints
)
{
    corePoints = 0;
    borderPoints = 0;
    noisePoints = 0;

    float epsilonSquared =
        EPSILON * EPSILON;

    for (int i = 0; i < N; i++)
    {
        if (labels[i] == NOISE)
        {
            noisePoints++;
            continue;
        }

        int neighbors = 0;

        for (int j = 0; j < N; j++)
        {
            if (i == j)
                continue;

            if (
                squaredDistance(
                    points[i],
                    points[j]
                ) <= epsilonSquared
            )
            {
                neighbors++;
            }
        }

        if (neighbors >= MINPTS)
        {
            corePoints++;
        }
        else
        {
            borderPoints++;
        }
    }
}


// ------------------------------------------------------------
// Print cluster sizes
// ------------------------------------------------------------

void printClusterSizes(int clusterCount)
{
    for (int cluster = 1;
         cluster <= clusterCount;
         cluster++)
    {
        int size = 0;

        for (int i = 0; i < N; i++)
        {
            if (labels[i] == cluster)
            {
                size++;
            }
        }

        Serial.printf(
            "  Cluster %d: %d\n",
            cluster,
            size
        );
    }
}


// ------------------------------------------------------------
// Run one density experiment
// ------------------------------------------------------------

void runExperiment(
    const char* densityName,
    float radius
)
{
    Serial.println();
    Serial.println(
        "========================================"
    );

    Serial.printf(
        "DENSITY = %s\n",
        densityName
    );

    Serial.println(
        "========================================"
    );

    Serial.printf(
        "Radius: %.2f\n",
        radius
    );

    Serial.printf(
        "Epsilon: %.2f\n",
        EPSILON
    );

    Serial.printf(
        "MinPts: %d\n",
        MINPTS
    );

    // --------------------------------------------------------
    // Generate the dataset
    // --------------------------------------------------------

    generateDataset(radius);

    // --------------------------------------------------------
    // Memory before DBSCAN
    // --------------------------------------------------------

    size_t freeHeapBefore =
        ESP.getFreeHeap();

    size_t freePsramBefore =
        ESP.getFreePsram();

    // --------------------------------------------------------
    // Run DBSCAN
    // --------------------------------------------------------

    unsigned long startTime =
        micros();

    int clusterCount =
        dbscan();

    unsigned long endTime =
        micros();

    double executionTime =
        (endTime - startTime) / 1000.0;

    // --------------------------------------------------------
    // Analyze classification
    //
    // NOTE:
    // This analysis is NOT included in execution time.
    // --------------------------------------------------------

    int corePoints;
    int borderPoints;
    int noisePoints;

    analyzeResult(
        clusterCount,
        corePoints,
        borderPoints,
        noisePoints
    );

    // --------------------------------------------------------
    // Output
    // --------------------------------------------------------

    Serial.printf(
        "Execution time: %.3f ms\n",
        executionTime
    );

    Serial.printf(
        "Clusters: %d\n",
        clusterCount
    );

    Serial.printf(
        "Core points: %d\n",
        corePoints
    );

    Serial.printf(
        "Border points: %d\n",
        borderPoints
    );

    Serial.printf(
        "Noise points: %d\n",
        noisePoints
    );

    Serial.printf(
        "Classification check: %d + %d + %d = %d\n",
        corePoints,
        borderPoints,
        noisePoints,
        corePoints +
        borderPoints +
        noisePoints
    );

    Serial.println(
        "Cluster sizes:"
    );

    printClusterSizes(
        clusterCount
    );

    // --------------------------------------------------------
    // Memory
    // --------------------------------------------------------

    Serial.printf(
        "Free heap before/after: %u KB / %u KB\n",
        (unsigned int)(
            freeHeapBefore / 1024
        ),
        (unsigned int)(
            ESP.getFreeHeap() / 1024
        )
    );

    Serial.printf(
        "Free PSRAM before/after: %u KB / %u KB\n",
        (unsigned int)(
            freePsramBefore / 1024
        ),
        (unsigned int)(
            ESP.getFreePsram() / 1024
        )
    );

    Serial.println();
}


// ------------------------------------------------------------
// Setup
// ------------------------------------------------------------

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
        "Experiment 04 — Density Sensitivity"
    );
    Serial.println(
        "========================================"
    );

    Serial.printf(
        "N: %d\n",
        N
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
        MINPTS
    );

    Serial.println();

    // --------------------------------------------------------
    // Allocate PSRAM
    // --------------------------------------------------------

    points = (Point3D*)ps_malloc(
        N * sizeof(Point3D)
    );

    labels = (int*)ps_malloc(
        N * sizeof(int)
    );

    queueBuffer = (int*)ps_malloc(
        N * sizeof(int)
    );

    if (
        points == nullptr ||
        labels == nullptr ||
        queueBuffer == nullptr
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
    // Memory calculation
    // --------------------------------------------------------

    size_t pointsMemory =
        N * sizeof(Point3D);

    size_t labelsMemory =
        N * sizeof(int);

    size_t queueMemory =
        N * sizeof(int);

    size_t totalMemory =
        pointsMemory +
        labelsMemory +
        queueMemory;

    Serial.println(
        "Memory allocation:"
    );

    Serial.printf(
        "Points: %.2f KB\n",
        pointsMemory / 1024.0
    );

    Serial.printf(
        "Labels: %.2f KB\n",
        labelsMemory / 1024.0
    );

    Serial.printf(
        "Queue: %.2f KB\n",
        queueMemory / 1024.0
    );

    Serial.printf(
        "Total PSRAM: %.2f KB\n",
        totalMemory / 1024.0
    );

    Serial.println();

    // --------------------------------------------------------
    // Run density experiments
    // --------------------------------------------------------

    runExperiment(
        "DENSE",
        DENSE_RADIUS
    );

    runExperiment(
        "MEDIUM",
        MEDIUM_RADIUS
    );

    runExperiment(
        "SPARSE",
        SPARSE_RADIUS
    );

    // --------------------------------------------------------
    // Final
    // --------------------------------------------------------

    Serial.println();
    Serial.println(
        "========================================"
    );
    Serial.println(
        "Experiment 04 completed."
    );
    Serial.println(
        "========================================"
    );
}


// ------------------------------------------------------------
// Loop
// ------------------------------------------------------------

void loop()
{
    delay(1000);
}