#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ============================================================
// ESP32-S3 Dual-Core DBSCAN
// Experiment A
//
// Parallel Core-Point Detection
//
// This experiment compares:
//
//   1. Baseline single-core DBSCAN
//   2. Dual-core DBSCAN
//
// BOTH algorithms run on the EXACT SAME DATASET.
//
// Core 0:
//   - processes first half of core-point detection
//   - performs DBSCAN cluster expansion
//
// Core 1:
//   - processes second half of core-point detection
//
// Cluster expansion remains on Core 0.
// ============================================================


// ============================================================
// Configuration
// ============================================================

#define MAX_N 1440

#define EPSILON 0.20f
#define MIN_PTS 5

#define NUM_TESTS 5

const int TEST_SIZES[NUM_TESTS] =
{
    180,
    360,
    720,
    1080,
    1440
};


// ============================================================
// Point structure
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

int *baselineLabels = nullptr;
int *dualLabels = nullptr;

int *queueBuffer = nullptr;

bool *baselineCoreFlags = nullptr;
bool *dualCoreFlags = nullptr;


// ============================================================
// Dual-core synchronization
// ============================================================

TaskHandle_t core1TaskHandle = nullptr;
TaskHandle_t mainTaskHandle = nullptr;

volatile int workerStartIndex = 0;
volatile int workerEndIndex = 0;
volatile int workerN = 0;


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

    return
        dx * dx +
        dy * dy +
        dz * dz;
}


// ============================================================
// Neighbor counting
//
// This is deliberately the same operation for both
// baseline and dual-core implementations.
// ============================================================

int countNeighbors(
    int index,
    int n
)
{
    const float epsilonSquared =
        EPSILON * EPSILON;

    int count = 0;

    for (int j = 0; j < n; j++)
    {
        if (
            squaredDistance(
                points[index],
                points[j]
            ) <= epsilonSquared
        )
        {
            count++;

            // We only need to know whether the point
            // reaches MinPts.
            if (count >= MIN_PTS)
            {
                return count;
            }
        }
    }

    return count;
}


// ============================================================
// Deterministic dataset
//
// 3 clusters:
//
//   C1 = (0, 0, 0)
//   C2 = (5, 5, 5)
//   C3 = (10, 0, 5)
//
// Radius = 0.60
//
// IMPORTANT:
// The same generated points are used by both algorithms.
// ============================================================

void generateDataset(int n)
{
    const float centers[3][3] =
    {
        {0.0f, 0.0f, 0.0f},
        {5.0f, 5.0f, 5.0f},
        {10.0f, 0.0f, 5.0f}
    };

    const float radius = 0.60f;

    // Fixed seed.
    // Therefore every execution generates the same dataset.
    uint32_t seed = 123456789UL;

    for (int i = 0; i < n; i++)
    {
        int cluster = i % 3;

        // LCG pseudo-random generator.
        seed =
            seed * 1664525UL +
            1013904223UL;

        float rx =
            ((float)((seed >> 8) & 0xFFFFFF) /
             16777215.0f) * 2.0f * radius -
            radius;

        seed =
            seed * 1664525UL +
            1013904223UL;

        float ry =
            ((float)((seed >> 8) & 0xFFFFFF) /
             16777215.0f) * 2.0f * radius -
            radius;

        seed =
            seed * 1664525UL +
            1013904223UL;

        float rz =
            ((float)((seed >> 8) & 0xFFFFFF) /
             16777215.0f) * 2.0f * radius -
            radius;

        points[i].x =
            centers[cluster][0] + rx;

        points[i].y =
            centers[cluster][1] + ry;

        points[i].z =
            centers[cluster][2] + rz;
    }
}


// ============================================================
// ============================================================
// BASELINE DBSCAN
// ============================================================
// ============================================================


// ------------------------------------------------------------
// Baseline core-point detection
// ------------------------------------------------------------

void detectCorePointsBaseline(
    int n
)
{
    for (int i = 0; i < n; i++)
    {
        int neighbors =
            countNeighbors(i, n);

        baselineCoreFlags[i] =
            (neighbors >= MIN_PTS);
    }
}


// ------------------------------------------------------------
// Baseline cluster expansion
// ------------------------------------------------------------

void expandClusterBaseline(
    int pointIndex,
    int clusterId,
    int n
)
{
    const float epsilonSquared =
        EPSILON * EPSILON;

    int queueSize = 0;

    baselineLabels[pointIndex] =
        clusterId;

    // --------------------------------------------------------
    // Initial neighborhood
    // --------------------------------------------------------

    for (int j = 0; j < n; j++)
    {
        if (
            squaredDistance(
                points[pointIndex],
                points[j]
            ) <= epsilonSquared
        )
        {
            if (baselineLabels[j] == -1)
            {
                baselineLabels[j] =
                    clusterId;

                if (baselineCoreFlags[j])
                {
                    if (queueSize < n)
                    {
                        queueBuffer[queueSize++] =
                            j;
                    }
                }
            }
        }
    }

    // --------------------------------------------------------
    // Expand cluster
    // --------------------------------------------------------

    int queueIndex = 0;

    while (queueIndex < queueSize)
    {
        int current =
            queueBuffer[queueIndex++];

        for (int j = 0; j < n; j++)
        {
            if (
                squaredDistance(
                    points[current],
                    points[j]
                ) <= epsilonSquared
            )
            {
                if (baselineLabels[j] == -1)
                {
                    baselineLabels[j] =
                        clusterId;

                    if (baselineCoreFlags[j])
                    {
                        if (queueSize < n)
                        {
                            queueBuffer[queueSize++] =
                                j;
                        }
                    }
                }
            }
        }
    }
}


// ------------------------------------------------------------
// Baseline DBSCAN
// ------------------------------------------------------------

int runBaselineDBSCAN(
    int n
)
{
    // -1 = unclassified
    // -2 = noise
    // >0 = cluster

    for (int i = 0; i < n; i++)
    {
        baselineLabels[i] = -1;
    }

    // --------------------------------------------------------
    // Core-point detection
    // --------------------------------------------------------

    detectCorePointsBaseline(n);

    // --------------------------------------------------------
    // Cluster expansion
    // --------------------------------------------------------

    int clusterCount = 0;

    for (int i = 0; i < n; i++)
    {
        if (baselineLabels[i] != -1)
        {
            continue;
        }

        if (!baselineCoreFlags[i])
        {
            baselineLabels[i] = -2;

            continue;
        }

        clusterCount++;

        expandClusterBaseline(
            i,
            clusterCount,
            n
        );
    }

    return clusterCount;
}


// ============================================================
// ============================================================
// DUAL-CORE DBSCAN
// ============================================================
// ============================================================


// ------------------------------------------------------------
// Core 1 worker
// ------------------------------------------------------------

void core1Worker(
    void *parameter
)
{
    Serial.printf(
        "Core 1 worker started on CPU %d\n",
        xPortGetCoreID()
    );

    while (true)
    {
        // ----------------------------------------------------
        // Wait for work from Core 0.
        // ----------------------------------------------------

        ulTaskNotifyTake(
            pdTRUE,
            portMAX_DELAY
        );

        // ----------------------------------------------------
        // Copy work parameters locally.
        // ----------------------------------------------------

        int start =
            workerStartIndex;

        int end =
            workerEndIndex;

        int n =
            workerN;

        // ----------------------------------------------------
        // Process second half.
        // ----------------------------------------------------

        for (int i = start; i < end; i++)
        {
            int neighbors =
                countNeighbors(i, n);

            dualCoreFlags[i] =
                (neighbors >= MIN_PTS);
        }

        // ----------------------------------------------------
        // Notify Core 0.
        // ----------------------------------------------------

        xTaskNotifyGive(
            mainTaskHandle
        );
    }
}


// ------------------------------------------------------------
// Parallel core-point detection
// ------------------------------------------------------------

void detectCorePointsDual(
    int n
)
{
    int half =
        n / 2;

    Serial.printf(
        "Core 0 will process: 0 -> %d\n",
        half
    );

    Serial.printf(
        "Core 1 will process: %d -> %d\n",
        half,
        n
    );

    // --------------------------------------------------------
    // Give Core 1 its range.
    // --------------------------------------------------------

    workerStartIndex = half;
    workerEndIndex = n;
    workerN = n;

    // --------------------------------------------------------
    // Start Core 1.
    // --------------------------------------------------------

    xTaskNotifyGive(
        core1TaskHandle
    );

    // --------------------------------------------------------
    // Core 0 processes first half.
    // --------------------------------------------------------

    for (int i = 0; i < half; i++)
    {
        int neighbors =
            countNeighbors(i, n);

        dualCoreFlags[i] =
            (neighbors >= MIN_PTS);
    }

    // --------------------------------------------------------
    // Wait for Core 1.
    // --------------------------------------------------------

    ulTaskNotifyTake(
        pdTRUE,
        portMAX_DELAY
    );
}


// ------------------------------------------------------------
// Dual-core cluster expansion
//
// This intentionally uses the SAME expansion algorithm
// as the baseline.
// ------------------------------------------------------------

void expandClusterDual(
    int pointIndex,
    int clusterId,
    int n
)
{
    const float epsilonSquared =
        EPSILON * EPSILON;

    int queueSize = 0;

    dualLabels[pointIndex] =
        clusterId;

    // --------------------------------------------------------
    // Initial neighborhood
    // --------------------------------------------------------

    for (int j = 0; j < n; j++)
    {
        if (
            squaredDistance(
                points[pointIndex],
                points[j]
            ) <= epsilonSquared
        )
        {
            if (dualLabels[j] == -1)
            {
                dualLabels[j] =
                    clusterId;

                if (dualCoreFlags[j])
                {
                    if (queueSize < n)
                    {
                        queueBuffer[queueSize++] =
                            j;
                    }
                }
            }
        }
    }

    // --------------------------------------------------------
    // Expand cluster
    // --------------------------------------------------------

    int queueIndex = 0;

    while (queueIndex < queueSize)
    {
        int current =
            queueBuffer[queueIndex++];

        for (int j = 0; j < n; j++)
        {
            if (
                squaredDistance(
                    points[current],
                    points[j]
                ) <= epsilonSquared
            )
            {
                if (dualLabels[j] == -1)
                {
                    dualLabels[j] =
                        clusterId;

                    if (dualCoreFlags[j])
                    {
                        if (queueSize < n)
                        {
                            queueBuffer[queueSize++] =
                                j;
                        }
                    }
                }
            }
        }
    }
}


// ------------------------------------------------------------
// Dual-core DBSCAN
// ------------------------------------------------------------

int runDualCoreDBSCAN(
    int n
)
{
    // -1 = unclassified
    // -2 = noise
    // >0 = cluster

    for (int i = 0; i < n; i++)
    {
        dualLabels[i] = -1;
    }

    // --------------------------------------------------------
    // Parallel phase
    // --------------------------------------------------------

    detectCorePointsDual(n);

    // --------------------------------------------------------
    // Serial cluster expansion
    // --------------------------------------------------------

    int clusterCount = 0;

    for (int i = 0; i < n; i++)
    {
        if (dualLabels[i] != -1)
        {
            continue;
        }

        if (!dualCoreFlags[i])
        {
            dualLabels[i] = -2;

            continue;
        }

        clusterCount++;

        expandClusterDual(
            i,
            clusterCount,
            n
        );
    }

    return clusterCount;
}


// ============================================================
// Result comparison
// ============================================================

bool compareLabels(
    int n,
    int &differentCount
)
{
    differentCount = 0;

    for (int i = 0; i < n; i++)
    {
        if (
            baselineLabels[i] !=
            dualLabels[i]
        )
        {
            differentCount++;
        }
    }

    return differentCount == 0;
}


// ============================================================
// Count result statistics
// ============================================================

void getStatistics(
    int *labels,
    bool *coreFlags,
    int n,
    int &clusters,
    int &core,
    int &border,
    int &noise
)
{
    clusters = 0;
    core = 0;
    border = 0;
    noise = 0;

    for (int i = 0; i < n; i++)
    {
        if (labels[i] == -2)
        {
            noise++;
        }
        else if (labels[i] > 0)
        {
            if (coreFlags[i])
            {
                core++;
            }
            else
            {
                border++;
            }

            if (labels[i] > clusters)
            {
                clusters =
                    labels[i];
            }
        }
    }
}


// ============================================================
// Memory report
// ============================================================

void printMemoryReport(
    int n
)
{
    size_t pointsBytes =
        sizeof(Point3D) * n;

    size_t labelsBytes =
        sizeof(int) * n * 2;

    size_t queueBytes =
        sizeof(int) * n;

    size_t flagsBytes =
        sizeof(bool) * n * 2;

    size_t totalBytes =
        pointsBytes +
        labelsBytes +
        queueBytes +
        flagsBytes;

    Serial.println();
    Serial.println("Memory:");

    Serial.printf(
        "  Points:          %.2f KB\n",
        pointsBytes / 1024.0f
    );

    Serial.printf(
        "  Labels (2x):     %.2f KB\n",
        labelsBytes / 1024.0f
    );

    Serial.printf(
        "  Queue:           %.2f KB\n",
        queueBytes / 1024.0f
    );

    Serial.printf(
        "  Core flags (2x): %.2f KB\n",
        flagsBytes / 1024.0f
    );

    Serial.printf(
        "  Total:           %.2f KB\n",
        totalBytes / 1024.0f
    );

    Serial.printf(
        "  Free heap:       %u bytes\n",
        ESP.getFreeHeap()
    );

    Serial.printf(
        "  Free PSRAM:      %u bytes\n",
        ESP.getFreePsram()
    );
}


// ============================================================
// Setup
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);

    mainTaskHandle =
        xTaskGetCurrentTaskHandle();

    Serial.println();
    Serial.println(
        "========================================"
    );

    Serial.println(
        "ESP32-S3 Dual-Core DBSCAN"
    );

    Serial.println(
        "Experiment A"
    );

    Serial.println(
        "Baseline vs Dual-Core"
    );

    Serial.println(
        "========================================"
    );

    Serial.printf(
        "Running setup on Core %d\n",
        xPortGetCoreID()
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

    // --------------------------------------------------------
    // Allocate PSRAM buffers.
    // --------------------------------------------------------

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

    queueBuffer =
        (int *)ps_malloc(
            sizeof(int) * MAX_N
        );

    baselineCoreFlags =
        (bool *)ps_malloc(
            sizeof(bool) * MAX_N
        );

    dualCoreFlags =
        (bool *)ps_malloc(
            sizeof(bool) * MAX_N
        );

    if (
        points == nullptr ||
        baselineLabels == nullptr ||
        dualLabels == nullptr ||
        queueBuffer == nullptr ||
        baselineCoreFlags == nullptr ||
        dualCoreFlags == nullptr
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

    Serial.println();
    Serial.println(
        "PSRAM buffers allocated successfully."
    );

    // --------------------------------------------------------
    // Create persistent Core 1 worker.
    // --------------------------------------------------------

    BaseType_t result =
        xTaskCreatePinnedToCore(
            core1Worker,
            "DBSCAN_Core1",
            8192,
            nullptr,
            1,
            &core1TaskHandle,
            1
        );

    if (result != pdPASS)
    {
        Serial.println(
            "ERROR: Could not create Core 1 worker!"
        );

        while (true)
        {
            delay(1000);
        }
    }

    Serial.println(
        "Core 1 DBSCAN worker created."
    );

    delay(500);
}


// ============================================================
// Benchmark
// ============================================================

void loop()
{
    static bool completed = false;

    if (completed)
    {
        delay(1000);
        return;
    }

    completed = true;

    // ========================================================
    // Run all benchmark sizes
    // ========================================================

    for (int test = 0;
         test < NUM_TESTS;
         test++)
    {
        int n =
            TEST_SIZES[test];

        Serial.println();
        Serial.println(
            "========================================"
        );

        Serial.printf(
            "Experiment A | N = %d\n",
            n
        );

        Serial.println(
            "========================================"
        );

        // ----------------------------------------------------
        // Generate ONE dataset.
        //
        // Both algorithms will use this exact data.
        // ----------------------------------------------------

        generateDataset(n);

        memset(
            baselineCoreFlags,
            0,
            sizeof(bool) * n
        );

        memset(
            dualCoreFlags,
            0,
            sizeof(bool) * n
        );

        // ----------------------------------------------------
        // Memory
        // ----------------------------------------------------

        printMemoryReport(n);

        // ====================================================
        // BASELINE
        // ====================================================

        Serial.println();
        Serial.println(
            "Running baseline DBSCAN..."
        );

        uint32_t baselineStart =
            micros();

        int baselineClusters =
            runBaselineDBSCAN(n);

        uint32_t baselineEnd =
            micros();

        float baselineTime =
            (baselineEnd - baselineStart) /
            1000.0f;

        // ----------------------------------------------------
        // Baseline statistics
        // ----------------------------------------------------

        int baselineClusterCount;
        int baselineCore;
        int baselineBorder;
        int baselineNoise;

        getStatistics(
            baselineLabels,
            baselineCoreFlags,
            n,
            baselineClusterCount,
            baselineCore,
            baselineBorder,
            baselineNoise
        );

        // ====================================================
        // DUAL CORE
        // ====================================================

        Serial.println();
        Serial.println(
            "Running dual-core DBSCAN..."
        );

        uint32_t dualStart =
            micros();

        int dualClusters =
            runDualCoreDBSCAN(n);

        uint32_t dualEnd =
            micros();

        float dualTime =
            (dualEnd - dualStart) /
            1000.0f;

        // ----------------------------------------------------
        // Dual statistics
        // ----------------------------------------------------

        int dualClusterCount;
        int dualCore;
        int dualBorder;
        int dualNoise;

        getStatistics(
            dualLabels,
            dualCoreFlags,
            n,
            dualClusterCount,
            dualCore,
            dualBorder,
            dualNoise
        );

        // ====================================================
        // Compare
        // ====================================================

        int differentLabels = 0;

        bool identical =
            compareLabels(
                n,
                differentLabels
            );

        float speedup =
            baselineTime /
            dualTime;

        float improvement =
            (
                (baselineTime - dualTime) /
                baselineTime
            ) * 100.0f;

        // ====================================================
        // OUTPUT
        // ====================================================

        Serial.println();
        Serial.println(
            "----------------------------------------"
        );

        Serial.println(
            "BASELINE"
        );

        Serial.printf(
            "  Time:           %.3f ms\n",
            baselineTime
        );

        Serial.printf(
            "  Clusters:       %d\n",
            baselineClusters
        );

        Serial.printf(
            "  Core points:    %d\n",
            baselineCore
        );

        Serial.printf(
            "  Border points:  %d\n",
            baselineBorder
        );

        Serial.printf(
            "  Noise:          %d\n",
            baselineNoise
        );

        Serial.println();
        Serial.println(
            "DUAL-CORE"
        );

        Serial.printf(
            "  Time:           %.3f ms\n",
            dualTime
        );

        Serial.printf(
            "  Clusters:       %d\n",
            dualClusters
        );

        Serial.printf(
            "  Core points:    %d\n",
            dualCore
        );

        Serial.printf(
            "  Border points:  %d\n",
            dualBorder
        );

        Serial.printf(
            "  Noise:          %d\n",
            dualNoise
        );

        Serial.println();
        Serial.println(
            "COMPARISON"
        );

        Serial.printf(
            "  Speedup:        %.2fx\n",
            speedup
        );

        Serial.printf(
            "  Improvement:    %.2f%%\n",
            improvement
        );

        Serial.printf(
            "  Baseline labels vs Dual labels: "
        );

        if (identical)
        {
            Serial.println(
                "IDENTICAL"
            );
        }
        else
        {
            Serial.println(
                "DIFFERENT"
            );
        }

        Serial.printf(
            "  Different labels: %d\n",
            differentLabels
        );

        Serial.printf(
            "  Cluster count: %d / %d\n",
            baselineClusters,
            dualClusters
        );

        Serial.printf(
            "  Noise: %d / %d\n",
            baselineNoise,
            dualNoise
        );

        Serial.println(
            "----------------------------------------"
        );

        delay(1000);
    }

    // ========================================================
    // Finished
    // ========================================================

    Serial.println();
    Serial.println(
        "========================================"
    );

    Serial.println(
        "Experiment A completed."
    );

    Serial.println(
        "========================================"
    );

    while (true)
    {
        delay(1000);
    }
}