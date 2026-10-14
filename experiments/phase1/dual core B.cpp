#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ============================================================
// Experiment B
// Fine-Grained Dual-Core DBSCAN
//
// Baseline:
//   Single-core DBSCAN
//
// Dual-Core:
//   For every neighborhood query, the dataset is split:
//   Core 0 -> [0, N/2)
//   Core 1 -> [N/2, N)
//
// The two partial neighbor counts are then combined.
//
// Cluster expansion itself remains serial on Core 0.
// This keeps the DBSCAN result deterministic and avoids
// shared-label race conditions.
//
// Correctness:
//   Baseline and Dual-Core run on exactly the same dataset.
//   All labels are compared.
//
// ============================================================

// ------------------------------------------------------------
// Configuration
// ------------------------------------------------------------

#define MAX_N 1440

const float EPSILON = 0.20f;
const int MIN_PTS = 5;

// Dataset generation
const float CLUSTER_RADIUS = 0.60f;

// ------------------------------------------------------------
// Data structures
// ------------------------------------------------------------

struct Point3D
{
    float x;
    float y;
    float z;
};

// ------------------------------------------------------------
// Global buffers
// ------------------------------------------------------------

Point3D *points = nullptr;

int *baselineLabels = nullptr;
int *dualLabels = nullptr;

uint8_t *baselineCore = nullptr;
uint8_t *dualCore = nullptr;

int *baselineQueue = nullptr;
int *dualQueue = nullptr;

// ------------------------------------------------------------
// Baseline globals
// ------------------------------------------------------------

int baselineN = 0;

// ------------------------------------------------------------
// Dual-core worker globals
// ------------------------------------------------------------

// Current neighborhood query
volatile int dualQueryIndex = -1;
volatile int dualQueryN = 0;

// Partial counts
volatile int dualCountCore0 = 0;
volatile int dualCountCore1 = 0;

// Worker synchronization
TaskHandle_t dualWorkerHandle = nullptr;

volatile bool dualWorkerReady = false;

// ------------------------------------------------------------
// Utility
// ------------------------------------------------------------

float squaredDistance(const Point3D &a, const Point3D &b)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;

    return dx * dx + dy * dy + dz * dz;
}

// ------------------------------------------------------------
// Deterministic random generator
// ------------------------------------------------------------

uint32_t rngState = 123456789;

uint32_t nextRandom()
{
    rngState ^= rngState << 13;
    rngState ^= rngState >> 17;
    rngState ^= rngState << 5;

    return rngState;
}

float randomFloat(float minValue, float maxValue)
{
    float r = (nextRandom() % 1000000) / 1000000.0f;

    return minValue + r * (maxValue - minValue);
}

// ------------------------------------------------------------
// Dataset generation
// ------------------------------------------------------------

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
        int cluster = i / perCluster;

        if (cluster > 2)
            cluster = 2;

        points[i].x =
            centers[cluster].x +
            randomFloat(-CLUSTER_RADIUS, CLUSTER_RADIUS);

        points[i].y =
            centers[cluster].y +
            randomFloat(-CLUSTER_RADIUS, CLUSTER_RADIUS);

        points[i].z =
            centers[cluster].z +
            randomFloat(-CLUSTER_RADIUS, CLUSTER_RADIUS);
    }
}

// ============================================================
// BASELINE DBSCAN
// ============================================================

int baselineCountNeighbors(
    int index,
    int N
)
{
    int count = 0;

    const float eps2 = EPSILON * EPSILON;

    for (int j = 0; j < N; j++)
    {
        if (squaredDistance(points[index], points[j]) <= eps2)
        {
            count++;

            if (count >= MIN_PTS)
                break;
        }
    }

    return count;
}

// ------------------------------------------------------------

void baselineAddNeighborsToQueue(
    int index,
    int N,
    int *labels,
    int *queue,
    int &queueSize
)
{
    const float eps2 = EPSILON * EPSILON;

    for (int j = 0; j < N; j++)
    {
        if (squaredDistance(points[index], points[j]) <= eps2)
        {
            if (labels[j] == 0)
            {
                labels[j] = -1;

                if (queueSize < N)
                {
                    queue[queueSize++] = j;
                }
            }
        }
    }
}

// ------------------------------------------------------------

void baselineExpandCluster(
    int startIndex,
    int clusterId,
    int N,
    int *labels,
    int *queue
)
{
    int queueSize = 0;
    int queuePosition = 0;

    labels[startIndex] = clusterId;

    baselineAddNeighborsToQueue(
        startIndex,
        N,
        labels,
        queue,
        queueSize
    );

    while (queuePosition < queueSize)
    {
        int current = queue[queuePosition++];

        if (labels[current] == -1)
        {
            labels[current] = clusterId;
        }

        int neighborCount =
            baselineCountNeighbors(current, N);

        if (neighborCount >= MIN_PTS)
        {
            baselineAddNeighborsToQueue(
                current,
                N,
                labels,
                queue,
                queueSize
            );
        }
    }
}

// ------------------------------------------------------------

void runBaselineDBSCAN(
    int N,
    int *labels,
    int *queue
)
{
    for (int i = 0; i < N; i++)
    {
        labels[i] = 0;
    }

    int clusterId = 0;

    for (int i = 0; i < N; i++)
    {
        if (labels[i] != 0)
            continue;

        int neighborCount =
            baselineCountNeighbors(i, N);

        if (neighborCount < MIN_PTS)
        {
            labels[i] = -2; // noise
            continue;
        }

        clusterId++;

        baselineExpandCluster(
            i,
            clusterId,
            N,
            labels,
            queue
        );
    }
}

// ============================================================
// DUAL-CORE NEIGHBOR COUNTING
// ============================================================

void dualWorkerTask(void *parameter)
{
    while (true)
    {
        // Wait until Core 0 gives us work.
        ulTaskNotifyTake(
            pdTRUE,
            portMAX_DELAY
        );

        int queryIndex = dualQueryIndex;
        int N = dualQueryN;

        if (queryIndex < 0 || N <= 0)
            continue;

        int start = N / 2;
        int count = 0;

        const float eps2 = EPSILON * EPSILON;

        for (int j = start; j < N; j++)
        {
            if (squaredDistance(
                    points[queryIndex],
                    points[j]
                ) <= eps2)
            {
                count++;

                if (count >= MIN_PTS)
                    break;
            }
        }

        dualCountCore1 = count;

        // Notify Core 0 that Core 1 finished.
        xTaskNotifyGive(
            xTaskGetCurrentTaskHandle()
        );

        // The above notification would notify ourselves,
        // so instead we use a separate completion flag.
        // Core 0 waits on dualWorkerReady below.
        dualWorkerReady = true;
    }
}

// ============================================================
// IMPORTANT:
// A second notification handle is required for clean
// synchronization.
// ============================================================

TaskHandle_t dualMainHandle = nullptr;

// ------------------------------------------------------------

void dualWorkerTaskFixed(void *parameter)
{
    while (true)
    {
        ulTaskNotifyTake(
            pdTRUE,
            portMAX_DELAY
        );

        int queryIndex = dualQueryIndex;
        int N = dualQueryN;

        if (queryIndex < 0 || N <= 0)
        {
            dualWorkerReady = true;
            xTaskNotifyGive(dualMainHandle);
            continue;
        }

        int start = N / 2;
        int count = 0;

        const float eps2 = EPSILON * EPSILON;

        for (int j = start; j < N; j++)
        {
            if (squaredDistance(
                    points[queryIndex],
                    points[j]
                ) <= eps2)
            {
                count++;

                if (count >= MIN_PTS)
                    break;
            }
        }

        dualCountCore1 = count;

        dualWorkerReady = true;

        // Notify Core 0.
        xTaskNotifyGive(dualMainHandle);
    }
}

// ------------------------------------------------------------

int dualCountNeighbors(
    int index,
    int N
)
{
    // Core 0 handles first half.
    int end = N / 2;

    int count0 = 0;

    const float eps2 = EPSILON * EPSILON;

    for (int j = 0; j < end; j++)
    {
        if (squaredDistance(
                points[index],
                points[j]
            ) <= eps2)
        {
            count0++;

            if (count0 >= MIN_PTS)
                break;
        }
    }

    // Prepare Core 1 query.
    dualCountCore1 = 0;
    dualWorkerReady = false;

    dualQueryIndex = index;
    dualQueryN = N;

    // Start Core 1.
    xTaskNotifyGive(dualWorkerHandle);

    // Wait for Core 1.
    ulTaskNotifyTake(
        pdTRUE,
        portMAX_DELAY
    );

    int total = count0 + dualCountCore1;

    return total;
}

// ============================================================
// DUAL-CORE DBSCAN
// ============================================================

void dualAddNeighborsToQueue(
    int index,
    int N,
    int *labels,
    int *queue,
    int &queueSize
)
{
    const float eps2 = EPSILON * EPSILON;

    for (int j = 0; j < N; j++)
    {
        if (squaredDistance(
                points[index],
                points[j]
            ) <= eps2)
        {
            if (labels[j] == 0)
            {
                labels[j] = -1;

                if (queueSize < N)
                {
                    queue[queueSize++] = j;
                }
            }
        }
    }
}

// ------------------------------------------------------------

void dualExpandCluster(
    int startIndex,
    int clusterId,
    int N,
    int *labels,
    int *queue
)
{
    int queueSize = 0;
    int queuePosition = 0;

    labels[startIndex] = clusterId;

    dualAddNeighborsToQueue(
        startIndex,
        N,
        labels,
        queue,
        queueSize
    );

    while (queuePosition < queueSize)
    {
        int current =
            queue[queuePosition++];

        if (labels[current] == -1)
        {
            labels[current] = clusterId;
        }

        int neighborCount =
            dualCountNeighbors(
                current,
                N
            );

        if (neighborCount >= MIN_PTS)
        {
            dualAddNeighborsToQueue(
                current,
                N,
                labels,
                queue,
                queueSize
            );
        }
    }
}

// ------------------------------------------------------------

void runDualDBSCAN(
    int N,
    int *labels,
    int *queue
)
{
    for (int i = 0; i < N; i++)
    {
        labels[i] = 0;
    }

    int clusterId = 0;

    for (int i = 0; i < N; i++)
    {
        if (labels[i] != 0)
            continue;

        int neighborCount =
            dualCountNeighbors(
                i,
                N
            );

        if (neighborCount < MIN_PTS)
        {
            labels[i] = -2;
            continue;
        }

        clusterId++;

        dualExpandCluster(
            i,
            clusterId,
            N,
            labels,
            queue
        );
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

// ------------------------------------------------------------

DBSCANStats calculateStats(
    int N,
    int *labels
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
        else if (labels[i] > maxCluster)
        {
            maxCluster = labels[i];
        }
    }

    stats.clusters = maxCluster;

    // Determine core/border classification.
    for (int i = 0; i < N; i++)
    {
        if (labels[i] <= 0)
            continue;

        int neighbors = 0;

        const float eps2 =
            EPSILON * EPSILON;

        for (int j = 0; j < N; j++)
        {
            if (squaredDistance(
                    points[i],
                    points[j]
                ) <= eps2)
            {
                neighbors++;

                if (neighbors >= MIN_PTS)
                    break;
            }
        }

        if (neighbors >= MIN_PTS)
            stats.core++;
        else
            stats.border++;
    }

    return stats;
}

// ============================================================
// Exact label comparison
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

void printMemoryInfo(int N)
{
    size_t pointMemory =
        sizeof(Point3D) * N;

    size_t labelMemory =
        sizeof(int) * N;

    size_t queueMemory =
        sizeof(int) * N;

    size_t coreMemory =
        sizeof(uint8_t) * N;

    size_t total =
        pointMemory +
        2 * labelMemory +
        2 * queueMemory +
        2 * coreMemory;

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
        "ESP32-S3 Dual-Core DBSCAN"
    );
    Serial.println(
        "Experiment B"
    );
    Serial.println(
        "Fine-Grained Dual-Core Neighborhood Search"
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

    // --------------------------------------------------------
    // Allocate PSRAM buffers
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

    baselineCore =
        (uint8_t *)ps_malloc(
            sizeof(uint8_t) * MAX_N
        );

    dualCore =
        (uint8_t *)ps_malloc(
            sizeof(uint8_t) * MAX_N
        );

    baselineQueue =
        (int *)ps_malloc(
            sizeof(int) * MAX_N
        );

    dualQueue =
        (int *)ps_malloc(
            sizeof(int) * MAX_N
        );

    if (
        !points ||
        !baselineLabels ||
        !dualLabels ||
        !baselineCore ||
        !dualCore ||
        !baselineQueue ||
        !dualQueue
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

    // --------------------------------------------------------
    // Save Core 0 task handle
    // --------------------------------------------------------

    dualMainHandle =
        xTaskGetCurrentTaskHandle();

    // --------------------------------------------------------
    // Create Core 1 worker
    // --------------------------------------------------------

    BaseType_t result =
        xTaskCreatePinnedToCore(
            dualWorkerTaskFixed,
            "DualDBSCANWorker",
            8192,
            nullptr,
            2,
            &dualWorkerHandle,
            1
        );

    if (result != pdPASS)
    {
        Serial.println(
            "ERROR: Failed to create Core 1 worker!"
        );

        while (true)
        {
            delay(1000);
        }
    }

    Serial.println(
        "Core 1 worker created."
    );

    delay(500);

    Serial.printf(
        "Worker running on CPU %d\n",
        xPortGetCoreID()
    );

    Serial.println();

    Serial.println(
        "Configuration:"
    );

    Serial.printf(
        "Dimensions: 3\n"
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
        "Cluster radius: %.2f\n",
        CLUSTER_RADIUS
    );

    Serial.println();

    printMemoryInfo(MAX_N);

    Serial.println();

    // ========================================================
    // Experiment B
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

    for (int t = 0; t < testCount; t++)
    {
        int N = testSizes[t];

        Serial.println();
        Serial.println(
            "----------------------------------------------"
        );

        Serial.printf(
            "N = %d\n",
            N
        );

        Serial.println(
            "Generating deterministic dataset..."
        );

        generateDataset(N);

        // ----------------------------------------------------
        // Baseline
        // ----------------------------------------------------

        Serial.println(
            "Running baseline..."
        );

        uint64_t startBaseline =
            esp_timer_get_time();

        runBaselineDBSCAN(
            N,
            baselineLabels,
            baselineQueue
        );

        uint64_t endBaseline =
            esp_timer_get_time();

        double baselineMs =
            (endBaseline - startBaseline)
            / 1000.0;

        // ----------------------------------------------------
        // Dual-core
        // ----------------------------------------------------

        Serial.println(
            "Running dual-core..."
        );

        uint64_t startDual =
            esp_timer_get_time();

        runDualDBSCAN(
            N,
            dualLabels,
            dualQueue
        );

        uint64_t endDual =
            esp_timer_get_time();

        double dualMs =
            (endDual - startDual)
            / 1000.0;

        // ----------------------------------------------------
        // Statistics
        // ----------------------------------------------------

        DBSCANStats baselineStats =
            calculateStats(
                N,
                baselineLabels
            );

        DBSCANStats dualStats =
            calculateStats(
                N,
                dualLabels
            );

        int different =
            compareLabels(
                N,
                baselineLabels,
                dualLabels
            );

        double speedup =
            baselineMs / dualMs;

        double improvement =
            ((baselineMs - dualMs) /
             baselineMs) * 100.0;

        // ----------------------------------------------------
        // Output
        // ----------------------------------------------------

        Serial.println();

        Serial.printf(
            "Baseline: %.3f ms\n",
            baselineMs
        );

        Serial.printf(
            "Dual-Core: %.3f ms\n",
            dualMs
        );

        Serial.printf(
            "Speedup: %.2fx\n",
            speedup
        );

        Serial.printf(
            "Improvement: %.2f%%\n",
            improvement
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

    Serial.println();
    Serial.println(
        "Experiment B completed."
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
