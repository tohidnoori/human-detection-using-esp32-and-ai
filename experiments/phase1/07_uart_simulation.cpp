#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ============================================================
// Experiment 07
// Three-Sensor UART-Style Real-Time DBSCAN Pipeline
//
// Current mode:
//   SOFTWARE SIMULATION
//
// The simulator produces three independent sensor streams using
// the SAME packet format that the real UART parser consumes.
// Each sensor observes two moving human-like point clouds.
// Local sensor coordinates are transformed back into one global
// coordinate system before DBSCAN.
//
// DBSCAN:
//   3D spatial grid, cell size = epsilon
//   exact squared-distance verification
//   no N x N matrix
//
// Later:
//   Set USE_REAL_UART = 1 and feed the same CSV packet format
//   through UART1, UART2 and UART0 (the third UART).
//
// Packet format per sensor:
//   frame_id,x,y,z\n
// End of sensor frame:
//   frame_id,END\n
// Example:
//   15,0.421,-0.183,1.122
//   15,0.398,-0.201,1.081
//   15,END
// ============================================================


// ============================================================
// Build mode
// ============================================================

#define USE_REAL_UART 0

// UART pin configuration for future real sensors.
// Change these to the actual wiring.
#define UART1_RX_PIN 4
#define UART1_TX_PIN 5

#define UART2_RX_PIN 6
#define UART2_TX_PIN 7

// UART0 is the third hardware UART. If your monitor uses UART0,
// keep USE_REAL_UART = 0 until the monitor is moved to USB CDC.
#define UART3_RX_PIN 15
#define UART3_TX_PIN 16

#define UART_BAUD 921600


// ============================================================
// Sensor / point configuration
// ============================================================

#define SENSOR_COUNT 3
#define HUMAN_COUNT 2

#define MAX_POINTS_PER_FRAME 2048
#define GRID_BUCKETS 4096
#define MAX_CLUSTER_STATS 32

// Parser line size.
#define SENSOR_LINE_SIZE 96

// Software UART buffer used by the simulator.
#define SIM_UART_BUFFER_BYTES 32768

// Number of frames for each load level.
#define SIM_FRAMES_PER_LOAD 30

// Target sensor frame rate used only to evaluate real-time
// feasibility. The benchmark itself does NOT sleep.
#define SIM_TARGET_HZ 10.0f

// Print a movement/cluster summary every N frames.
#define PRINT_EVERY_N_FRAMES 10

// Human-like cluster classification thresholds.
#define MIN_HUMAN_CLUSTER_POINTS 12
#define MIN_HUMAN_HEIGHT 0.65f
#define MAX_HUMAN_HEIGHT 2.50f
#define MAX_HUMAN_FOOTPRINT 2.50f

// Point-load sweep.
// Total points/frame = SENSOR_COUNT *
//                       (HUMAN_COUNT * pointsPerHuman + noisePerSensor)
const int SIM_POINTS_PER_HUMAN_LEVELS[] =
{
    12,
    24,
    48,
    96,
    160,
    240
};

const int SIM_NOISE_POINTS_PER_SENSOR = 12;


// ============================================================
// DBSCAN configuration
// ============================================================

const float EPSILON = 0.35f;
const float EPSILON2 = EPSILON * EPSILON;
const int MIN_PTS = 5;


// ============================================================
// Data structures
// ============================================================

struct Point3D
{
    float x;
    float y;
    float z;
};
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
struct SensorPose
{
    float x;
    float y;
    float z;
    float yaw;
};

struct HumanPose
{
    float x;
    float y;
    float z;
};

struct ClusterInfo
{
    int count;
    float sumX;
    float sumY;
    float sumZ;
    float minX;
    float minY;
    float minZ;
    float maxX;
    float maxY;
    float maxZ;
};

struct PipelineMetrics
{
    uint32_t frames;
    uint32_t framesReady;
    uint32_t droppedPoints;
    uint32_t realtimeOverruns;

    uint64_t totalIngestUs;
    uint64_t totalDbscanUs;
    uint64_t totalAnalysisUs;
    uint64_t totalPipelineUs;

    uint32_t totalInputPoints;
    uint32_t minPoints;
    uint32_t maxPoints;
    uint64_t maxPipelineUs;
};


// ============================================================
// Global point / DBSCAN buffers
// ============================================================

Point3D *points = nullptr;

int *labels = nullptr;
uint8_t *coreFlags = nullptr;
int *queueBuffer = nullptr;

int *gridHead = nullptr;
int *gridNext = nullptr;
int *cellX = nullptr;
int *cellY = nullptr;
int *cellZ = nullptr;

ClusterInfo clusterStats[MAX_CLUSTER_STATS];


// ============================================================
// Simulated UART
// ============================================================

class SimulatedUart
{
public:
    uint8_t *buffer = nullptr;
    size_t head = 0;
    size_t tail = 0;
    uint32_t overflowBytes = 0;

    bool begin()
    {
        buffer = (uint8_t *)ps_malloc(
            SIM_UART_BUFFER_BYTES
        );

        if (!buffer)
            return false;

        reset();
        return true;
    }

    void reset()
    {
        head = 0;
        tail = 0;
        overflowBytes = 0;
    }

    size_t size() const
    {
        return head - tail;
    }

    bool writeByte(uint8_t b)
    {
        if (size() >= SIM_UART_BUFFER_BYTES)
        {
            overflowBytes++;
            return false;
        }

        // Linear per-frame buffer. It is reset once the frame is parsed.
        buffer[head++] = b;
        return true;
    }

    size_t writeBytes(const char *data, size_t len)
    {
        size_t accepted = 0;

        for (size_t i = 0; i < len; i++)
        {
            if (writeByte((uint8_t)data[i]))
                accepted++;
            else
                break;
        }

        return accepted;
    }

    bool available() const
    {
        return tail < head;
    }

    uint8_t readByte()
    {
        return buffer[tail++];
    }
};


SimulatedUart simUart[SENSOR_COUNT];


// ============================================================
// Real UARTs
// ============================================================

#if USE_REAL_UART
HardwareSerial SensorUART1(1);
HardwareSerial SensorUART2(2);
HardwareSerial SensorUART3(0);
#endif


// ============================================================
// Sensor parser state
// ============================================================

struct ParserState
{
    char line[SENSOR_LINE_SIZE];
    size_t length;
};

ParserState parserState[SENSOR_COUNT];


// ============================================================
// Frame accumulator
// ============================================================

const uint32_t INVALID_FRAME_ID = 0xFFFFFFFFUL;

uint32_t activeFrameId = INVALID_FRAME_ID;
uint8_t sensorDoneMask = 0;
uint32_t droppedPoints = 0;
bool frameReady = false;

int pointCount = 0;


// ============================================================
// Sensor poses
// ============================================================

SensorPose sensorPose[SENSOR_COUNT] =
{
    // x, y, z, yaw
    {-4.0f,  0.0f, 1.60f,  0.0f},
    { 4.0f,  0.0f, 1.60f,  PI},
    { 0.0f,  4.0f, 1.60f, -PI / 2.0f}
};


// ============================================================
// Simulator random generator
// ============================================================

uint32_t simRngState = 0xA341316Cu;

uint32_t simRandom()
{
    simRngState ^= simRngState << 13;
    simRngState ^= simRngState >> 17;
    simRngState ^= simRngState << 5;

    return simRngState;
}


float simRandom01()
{
    return (simRandom() & 0x00FFFFFF) /
           16777215.0f;
}


float simRandomRange(
    float minValue,
    float maxValue
)
{
    return minValue +
           simRandom01() *
           (maxValue - minValue);
}


// ============================================================
// Human movement model
// ============================================================

HumanPose getHumanPose(
    int humanId,
    uint32_t frameId
)
{
    float t =
        frameId / SIM_TARGET_HZ;

    HumanPose pose{};

    if (humanId == 0)
    {
        // Person A: walks left/right while gently moving in y.
        float phase =
            0.35f * t;

        pose.x =
            -2.0f +
            4.0f *
            (0.5f + 0.5f * sinf(phase));

        pose.y =
            0.9f +
            0.45f * sinf(0.9f * t);

        pose.z = 0.95f;
    }
    else
    {
        // Person B: moves on a different path.
        float phase =
            0.55f * t;

        pose.x =
            0.9f +
            1.15f * cosf(phase);

        pose.y =
            -1.25f +
            0.75f * sinf(0.75f * t);

        pose.z = 1.00f;
    }

    return pose;
}


// ============================================================
// Transform: global -> sensor local
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

    Point3D local{};

    local.x =
        c * dx + s * dy;

    local.y =
        -s * dx + c * dy;

    local.z = dz;

    return local;
}


// ============================================================
// Transform: sensor local -> global
// ============================================================

Point3D sensorToWorld(
    const Point3D &local,
    const SensorPose &sensor
)
{
    float c = cosf(sensor.yaw);
    float s = sinf(sensor.yaw);

    Point3D world{};

    world.x =
        sensor.x +
        c * local.x -
        s * local.y;

    world.y =
        sensor.y +
        s * local.x +
        c * local.y;

    world.z =
        sensor.z +
        local.z;

    return world;
}


// ============================================================
// Generate a human-like point around a pose
// ============================================================

Point3D generateHumanPoint(
    const HumanPose &pose
)
{
    Point3D p =
    {
        pose.x,
        pose.y,
        pose.z
    };

    float r = simRandom01();

    float rx = simRandomRange(-1.0f, 1.0f);
    float ry = simRandomRange(-1.0f, 1.0f);
    float rz = simRandomRange(-1.0f, 1.0f);

    // Torso
    if (r < 0.50f)
    {
        p.x += rx * 0.27f;
        p.y += ry * 0.23f;
        p.z += rz * 0.38f;
    }
    // Head / shoulders
    else if (r < 0.68f)
    {
        p.x += rx * 0.15f;
        p.y += ry * 0.14f;
        p.z += 0.55f +
               fabsf(rz) * 0.18f;
    }
    // Arms
    else if (r < 0.84f)
    {
        float side =
            simRandom01() < 0.5f
                ? -1.0f
                : 1.0f;

        p.x +=
            side *
            (0.28f +
             fabsf(rx) * 0.34f);

        p.y += ry * 0.16f;
        p.z += rz * 0.32f;
    }
    // Legs
    else
    {
        float side =
            simRandom01() < 0.5f
                ? -1.0f
                : 1.0f;

        p.x +=
            side *
            (0.10f +
             fabsf(rx) * 0.08f);

        p.y += ry * 0.14f;

        p.z -=
            0.30f +
            fabsf(rz) * 0.55f;
    }

    return p;
}


// ============================================================
// Add serialized packet to simulated UART
// ============================================================

void simWritePoint(
    SimulatedUart &uart,
    uint32_t frameId,
    const Point3D &local
)
{
    char line[96];

    int length = snprintf(
        line,
        sizeof(line),
        "%lu,%.4f,%.4f,%.4f\n",
        (unsigned long)frameId,
        local.x,
        local.y,
        local.z
    );

    if (length > 0)
    {
        uart.writeBytes(
            line,
            (size_t)length
        );
    }
}


void simWriteEnd(
    SimulatedUart &uart,
    uint32_t frameId
)
{
    char line[48];

    int length = snprintf(
        line,
        sizeof(line),
        "%lu,END\n",
        (unsigned long)frameId
    );

    if (length > 0)
    {
        uart.writeBytes(
            line,
            (size_t)length
        );
    }
}


// ============================================================
// Fill three simulated sensor streams
// ============================================================

int simulateFrame(
    uint32_t frameId,
    int pointsPerHuman
)
{
    for (int s = 0; s < SENSOR_COUNT; s++)
    {
        simUart[s].reset();
        parserState[s].length = 0;
    }

    simRngState =
        0xA341316Cu ^
        (frameId * 2654435761UL);

    int generatedPoints = 0;

    for (int sensor = 0;
         sensor < SENSOR_COUNT;
         sensor++)
    {
        // Every sensor observes both humans.
        for (int human = 0;
             human < HUMAN_COUNT;
             human++)
        {
            HumanPose pose =
                getHumanPose(
                    human,
                    frameId
                );

            for (int p = 0;
                 p < pointsPerHuman;
                 p++)
            {
                Point3D world =
                    generateHumanPoint(pose);

                Point3D local =
                    worldToSensor(
                        world,
                        sensorPose[sensor]
                    );

                // Sensor measurement noise.
                local.x +=
                    simRandomRange(
                        -0.035f,
                        0.035f
                    );

                local.y +=
                    simRandomRange(
                        -0.035f,
                        0.035f
                    );

                local.z +=
                    simRandomRange(
                        -0.045f,
                        0.045f
                    );

                simWritePoint(
                    simUart[sensor],
                    frameId,
                    local
                );

                generatedPoints++;
            }
        }

        // Sensor clutter / environment noise.
        for (int n = 0;
             n < SIM_NOISE_POINTS_PER_SENSOR;
             n++)
        {
            Point3D world{};

            world.x =
                simRandomRange(
                    -3.8f,
                    3.8f
                );

            world.y =
                simRandomRange(
                    -3.8f,
                    3.8f
                );

            world.z =
                simRandomRange(
                    0.10f,
                    2.30f
                );

            Point3D local =
                worldToSensor(
                    world,
                    sensorPose[sensor]
                );

            simWritePoint(
                simUart[sensor],
                frameId,
                local
            );

            generatedPoints++;
        }

        simWriteEnd(
            simUart[sensor],
            frameId
        );
    }

    return generatedPoints;
}


// ============================================================
// Reset parser/frame accumulator
// ============================================================

void resetFrameAccumulator()
{
    activeFrameId = INVALID_FRAME_ID;
    sensorDoneMask = 0;
    frameReady = false;
    pointCount = 0;

    for (int s = 0;
         s < SENSOR_COUNT;
         s++)
    {
        parserState[s].length = 0;
    }
}


// ============================================================
// Append transformed global point
// ============================================================

void appendGlobalPoint(
    int sensorId,
    const Point3D &local
)
{
    if (pointCount >= MAX_POINTS_PER_FRAME)
    {
        droppedPoints++;
        return;
    }

    points[pointCount++] =
        sensorToWorld(
            local,
            sensorPose[sensorId]
        );
}


// ============================================================
// Parse one sensor packet line
// ============================================================

bool parseSensorLine(
    int sensorId,
    const char *line
)
{
    char *cursor =
        (char *)line;

    char *endPtr = nullptr;

    unsigned long frame =
        strtoul(
            cursor,
            &endPtr,
            10
        );

    if (endPtr == cursor ||
        *endPtr != ',')
    {
        return false;
    }

    cursor = endPtr + 1;

    if (strncmp(
            cursor,
            "END",
            3
        ) == 0)
    {
        uint32_t frameId =
            (uint32_t)frame;

        if (activeFrameId == INVALID_FRAME_ID)
        {
            activeFrameId = frameId;
        }

        if (frameId != activeFrameId)
        {
            return false;
        }

        sensorDoneMask |=
            (uint8_t)(1U << sensorId);

        if (sensorDoneMask == 0x07)
        {
            frameReady = true;
        }

        return true;
    }

    float x =
        strtof(
            cursor,
            &endPtr
        );

    if (endPtr == cursor ||
        *endPtr != ',')
    {
        return false;
    }

    cursor = endPtr + 1;

    float y =
        strtof(
            cursor,
            &endPtr
        );

    if (endPtr == cursor ||
        *endPtr != ',')
    {
        return false;
    }

    cursor = endPtr + 1;

    float z =
        strtof(
            cursor,
            &endPtr
        );

    if (endPtr == cursor)
    {
        return false;
    }

    uint32_t frameId =
        (uint32_t)frame;

    if (activeFrameId == INVALID_FRAME_ID)
    {
        activeFrameId = frameId;
    }

    if (frameId != activeFrameId)
    {
        // For this benchmark one complete frame is processed at
        // a time. A production implementation can keep a small
        // two-frame reorder buffer if sensor timing is asynchronous.
        return false;
    }

    Point3D local =
    {
        x,
        y,
        z
    };

    appendGlobalPoint(
        sensorId,
        local
    );

    return true;
}


// ============================================================
// Consume bytes from a simulated UART
// ============================================================

uint32_t consumeSimUart(
    int sensorId
)
{
    uint32_t parsedLines = 0;

    SimulatedUart &uart =
        simUart[sensorId];

    ParserState &parser =
        parserState[sensorId];

    while (uart.available())
    {
        char c =
            (char)uart.readByte();

        if (c == '\r')
            continue;

        if (c == '\n')
        {
            parser.line[
                parser.length
            ] = '\0';

            if (parser.length > 0)
            {
                if (parseSensorLine(
                        sensorId,
                        parser.line
                    ))
                {
                    parsedLines++;
                }
            }

            parser.length = 0;
            continue;
        }

        if (
            parser.length + 1 <
            SENSOR_LINE_SIZE
        )
        {
            parser.line[
                parser.length++
            ] = c;
        }
        else
        {
            // Drop malformed/oversized line.
            parser.length = 0;
        }
    }

    return parsedLines;
}


// ============================================================
// Real UART support
// ============================================================

#if USE_REAL_UART

HardwareSerial *getRealUart(int sensorId)
{
    if (sensorId == 0)
        return &SensorUART1;

    if (sensorId == 1)
        return &SensorUART2;

    return &SensorUART3;
}


uint32_t consumeRealUart(
    int sensorId
)
{
    HardwareSerial *uart =
        getRealUart(sensorId);

    ParserState &parser =
        parserState[sensorId];

    uint32_t parsedLines = 0;

    while (uart->available())
    {
        char c =
            (char)uart->read();

        if (c == '\r')
            continue;

        if (c == '\n')
        {
            parser.line[
                parser.length
            ] = '\0';

            if (parser.length > 0)
            {
                if (parseSensorLine(
                        sensorId,
                        parser.line
                    ))
                {
                    parsedLines++;
                }
            }

            parser.length = 0;
            continue;
        }

        if (
            parser.length + 1 <
            SENSOR_LINE_SIZE
        )
        {
            parser.line[
                parser.length++
            ] = c;
        }
        else
        {
            parser.length = 0;
        }
    }

    return parsedLines;
}

#endif


// ============================================================
// Ingest one complete simulated frame
// ============================================================

uint32_t ingestSimulationFrame()
{
    resetFrameAccumulator();

    uint32_t parsedLines = 0;

    for (int sensor = 0;
         sensor < SENSOR_COUNT;
         sensor++)
    {
        parsedLines +=
            consumeSimUart(sensor);
    }

    return parsedLines;
}


// ============================================================
// Ingest real UART streams
// ============================================================

#if USE_REAL_UART
uint32_t ingestRealUarts()
{
    uint32_t parsedLines = 0;

    for (int sensor = 0;
         sensor < SENSOR_COUNT;
         sensor++)
    {
        parsedLines +=
            consumeRealUart(sensor);
    }

    return parsedLines;
}
#endif


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


inline int cellCoordinate(
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
            cellCoordinate(
                points[i].x
            );

        int y =
            cellCoordinate(
                points[i].y
            );

        int z =
            cellCoordinate(
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
// Exact grid neighbor count
// ============================================================

int gridCountNeighbors(int index)
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
                    gridHash(
                        nx,
                        ny,
                        nz
                    );

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
// Grid neighbor expansion
//
// DBSCAN labels:
//   0  = unvisited
//  -1  = noise
//  >0  = cluster id
//
// A noise point discovered later as a border point is correctly
// reassigned to the cluster but is not queued unless it was
// previously unvisited.
// ============================================================

void addNeighborsToQueue(
    int index,
    int clusterId,
    int N,
    int &queueSize
)
{
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
                    gridHash(
                        nx,
                        ny,
                        nz
                    );

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
                                labels[current] =
                                    clusterId;

                                if (queueSize < N)
                                {
                                    queueBuffer[
                                        queueSize++
                                    ] = current;
                                }
                            }
                            else if (labels[current] == -1)
                            {
                                // Previously marked noise but now
                                // discovered to be a border point.
                                labels[current] =
                                    clusterId;
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
// Grid DBSCAN
// ============================================================

int runGridDBSCAN(int N)
{
    for (int i = 0;
         i < N;
         i++)
    {
        labels[i] = 0;
        coreFlags[i] = 0;
    }

    buildGrid(N);

    // --------------------------------------------------------
    // Core-point detection
    // --------------------------------------------------------

    for (int i = 0;
         i < N;
         i++)
    {
        if (
            gridCountNeighbors(i) >=
            MIN_PTS
        )
        {
            coreFlags[i] = 1;
        }
    }

    // --------------------------------------------------------
    // Expansion
    // --------------------------------------------------------

    int clusterId = 0;

    for (int i = 0;
         i < N;
         i++)
    {
        if (labels[i] != 0)
            continue;

        if (!coreFlags[i])
        {
            labels[i] = -1;
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
                queueBuffer[
                    queuePosition++
                ];

            if (coreFlags[current])
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
// Cluster analysis / human detection
// ============================================================

int analyzeClusters(
    int N,
    int clusterCount,
    int &humanCount
)
{
    if (clusterCount > MAX_CLUSTER_STATS)
    {
        clusterCount =
            MAX_CLUSTER_STATS;
    }

    for (int c = 0;
         c < MAX_CLUSTER_STATS;
         c++)
    {
        clusterStats[c].count = 0;
        clusterStats[c].sumX = 0.0f;
        clusterStats[c].sumY = 0.0f;
        clusterStats[c].sumZ = 0.0f;
        clusterStats[c].minX = 9999.0f;
        clusterStats[c].minY = 9999.0f;
        clusterStats[c].minZ = 9999.0f;
        clusterStats[c].maxX = -9999.0f;
        clusterStats[c].maxY = -9999.0f;
        clusterStats[c].maxZ = -9999.0f;
    }

    for (int i = 0;
         i < N;
         i++)
    {
        int label = labels[i];

        if (label <= 0)
            continue;

        if (label > MAX_CLUSTER_STATS)
            continue;

        ClusterInfo &c =
            clusterStats[label - 1];

        const Point3D &p =
            points[i];

        c.count++;
        c.sumX += p.x;
        c.sumY += p.y;
        c.sumZ += p.z;

        if (p.x < c.minX) c.minX = p.x;
        if (p.y < c.minY) c.minY = p.y;
        if (p.z < c.minZ) c.minZ = p.z;

        if (p.x > c.maxX) c.maxX = p.x;
        if (p.y > c.maxY) c.maxY = p.y;
        if (p.z > c.maxZ) c.maxZ = p.z;
    }

    humanCount = 0;

    for (int c = 0;
         c < clusterCount;
         c++)
    {
        ClusterInfo &info =
            clusterStats[c];

        if (info.count == 0)
            continue;

        float height =
            info.maxZ - info.minZ;

        float footprintX =
            info.maxX - info.minX;

        float footprintY =
            info.maxY - info.minY;

        float footprint =
            max(
                footprintX,
                footprintY
            );

        bool humanLike =
            info.count >=
                MIN_HUMAN_CLUSTER_POINTS &&
            height >=
                MIN_HUMAN_HEIGHT &&
            height <=
                MAX_HUMAN_HEIGHT &&
            footprint <=
                MAX_HUMAN_FOOTPRINT;

        if (humanLike)
            humanCount++;
    }

    return clusterCount;
}


// ============================================================
// Print selected frame result
// ============================================================

void printFrameResult(
    uint32_t frameId,
    int N,
    int clusters,
    int humanCount,
    double pipelineMs
)
{
    Serial.printf(
        "Frame %lu | points=%d | clusters=%d | human-like=%d | pipeline=%.3f ms\n",
        (unsigned long)frameId,
        N,
        clusters,
        humanCount,
        pipelineMs
    );

    for (int c = 0;
         c < clusters && c < MAX_CLUSTER_STATS;
         c++)
    {
        ClusterInfo &info =
            clusterStats[c];

        if (info.count == 0)
            continue;

        float cx =
            info.sumX / info.count;

        float cy =
            info.sumY / info.count;

        float cz =
            info.sumZ / info.count;

        float height =
            info.maxZ - info.minZ;

        float footprint =
            max(
                info.maxX - info.minX,
                info.maxY - info.minY
            );

        bool humanLike =
            info.count >=
                MIN_HUMAN_CLUSTER_POINTS &&
            height >=
                MIN_HUMAN_HEIGHT &&
            height <=
                MAX_HUMAN_HEIGHT &&
            footprint <=
                MAX_HUMAN_FOOTPRINT;

        Serial.printf(
            "  C%d: n=%d centroid=(%.2f, %.2f, %.2f) height=%.2f footprint=%.2f %s\n",
            c + 1,
            info.count,
            cx,
            cy,
            cz,
            height,
            footprint,
            humanLike ? "HUMAN-LIKE" : "other"
        );
    }
}


// ============================================================
// Metrics helpers
// ============================================================

void resetMetrics(PipelineMetrics &m)
{
    memset(
        &m,
        0,
        sizeof(m)
    );

    m.minPoints =
        0xFFFFFFFFUL;
}


void printMetrics(
    const PipelineMetrics &m,
    int pointsPerHuman
)
{
    if (m.frames == 0)
        return;

    double avgIngestMs =
        m.totalIngestUs /
        1000.0 /
        m.frames;

    double avgDbscanMs =
        m.totalDbscanUs /
        1000.0 /
        m.frames;

    double avgAnalysisMs =
        m.totalAnalysisUs /
        1000.0 /
        m.frames;

    double avgPipelineMs =
        m.totalPipelineUs /
        1000.0 /
        m.frames;

    double maxPipelineMs =
        m.maxPipelineUs /
        1000.0;

    double fps =
        avgPipelineMs > 0.0
            ? 1000.0 / avgPipelineMs
            : 0.0;

    double avgPoints =
        (double)m.totalInputPoints /
        m.frames;

    double pointsPerSecond =
        avgPoints * fps;

    double targetFrameMs =
        1000.0 / SIM_TARGET_HZ;

    bool realTime =
        maxPipelineMs <= targetFrameMs;

    Serial.println();
    Serial.println(
        "Load result"
    );

    Serial.printf(
        "Points/human/sensor: %d\n",
        pointsPerHuman
    );

    Serial.printf(
        "Average points/frame: %.1f\n",
        avgPoints
    );

    Serial.printf(
        "Min points/frame: %u\n",
        (unsigned)m.minPoints
    );

    Serial.printf(
        "Max points/frame: %u\n",
        (unsigned)m.maxPoints
    );

    Serial.printf(
        "Average ingest/parse: %.3f ms\n",
        avgIngestMs
    );

    Serial.printf(
        "Average grid+DBSCAN: %.3f ms\n",
        avgDbscanMs
    );

    Serial.printf(
        "Average human analysis: %.3f ms\n",
        avgAnalysisMs
    );

    Serial.printf(
        "Average total pipeline: %.3f ms\n",
        avgPipelineMs
    );

    Serial.printf(
        "Maximum total pipeline: %.3f ms\n",
        maxPipelineMs
    );

    Serial.printf(
        "Estimated max FPS: %.2f\n",
        fps
    );

    Serial.printf(
        "Estimated throughput: %.1f points/s\n",
        pointsPerSecond
    );

    Serial.printf(
        "Target FPS: %.2f\n",
        SIM_TARGET_HZ
    );

    Serial.printf(
        "Target frame budget: %.3f ms\n",
        targetFrameMs
    );

    Serial.printf(
        "Real-time at target: %s\n",
        realTime ? "YES" : "NO"
    );

    Serial.printf(
        "Frame overruns: %u / %u\n",
        (unsigned)m.realtimeOverruns,
        (unsigned)m.frames
    );

    Serial.printf(
        "Dropped points: %u\n",
        (unsigned)m.droppedPoints
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
// Memory info
// ============================================================

void printMemoryInfo()
{
    size_t pointsMemory =
        sizeof(Point3D) * MAX_POINTS_PER_FRAME;

    size_t labelsMemory =
        sizeof(int) * MAX_POINTS_PER_FRAME;

    size_t coreMemory =
        sizeof(uint8_t) * MAX_POINTS_PER_FRAME;

    size_t queueMemory =
        sizeof(int) * MAX_POINTS_PER_FRAME;

    size_t gridHeadMemory =
        sizeof(int) * GRID_BUCKETS;

    size_t gridNextMemory =
        sizeof(int) * MAX_POINTS_PER_FRAME;

    size_t cellsMemory =
        sizeof(int) * MAX_POINTS_PER_FRAME * 3;

    size_t uartMemory =
        SIM_UART_BUFFER_BYTES * SENSOR_COUNT;

    size_t total =
        pointsMemory +
        labelsMemory +
        coreMemory +
        queueMemory +
        gridHeadMemory +
        gridNextMemory +
        cellsMemory +
        uartMemory;

    Serial.printf(
        "Main buffers + simulated UART buffers: %.2f KB\n",
        total / 1024.0f
    );
}


// ============================================================
// Allocate buffers
// ============================================================

bool allocateBuffers()
{
    points =
        (Point3D *)ps_malloc(
            sizeof(Point3D) *
            MAX_POINTS_PER_FRAME
        );

    labels =
        (int *)ps_malloc(
            sizeof(int) *
            MAX_POINTS_PER_FRAME
        );

    coreFlags =
        (uint8_t *)ps_malloc(
            sizeof(uint8_t) *
            MAX_POINTS_PER_FRAME
        );

    queueBuffer =
        (int *)ps_malloc(
            sizeof(int) *
            MAX_POINTS_PER_FRAME
        );

    gridHead =
        (int *)ps_malloc(
            sizeof(int) *
            GRID_BUCKETS
        );

    gridNext =
        (int *)ps_malloc(
            sizeof(int) *
            MAX_POINTS_PER_FRAME
        );

    cellX =
        (int *)ps_malloc(
            sizeof(int) *
            MAX_POINTS_PER_FRAME
        );

    cellY =
        (int *)ps_malloc(
            sizeof(int) *
            MAX_POINTS_PER_FRAME
        );

    cellZ =
        (int *)ps_malloc(
            sizeof(int) *
            MAX_POINTS_PER_FRAME
        );

    if (
        !points ||
        !labels ||
        !coreFlags ||
        !queueBuffer ||
        !gridHead ||
        !gridNext ||
        !cellX ||
        !cellY ||
        !cellZ
    )
    {
        return false;
    }

#if !USE_REAL_UART
    for (int s = 0;
         s < SENSOR_COUNT;
         s++)
    {
        if (!simUart[s].begin())
        {
            return false;
        }
    }
#endif

    return true;
}


// ============================================================
// Real UART setup
// ============================================================

#if USE_REAL_UART
void setupRealUarts()
{
    SensorUART1.begin(
        UART_BAUD,
        SERIAL_8N1,
        UART1_RX_PIN,
        UART1_TX_PIN
    );

    SensorUART2.begin(
        UART_BAUD,
        SERIAL_8N1,
        UART2_RX_PIN,
        UART2_TX_PIN
    );

    SensorUART3.begin(
        UART_BAUD,
        SERIAL_8N1,
        UART3_RX_PIN,
        UART3_TX_PIN
    );

    Serial.println(
        "Real UART mode enabled."
    );
}
#endif


// ============================================================
// Run one simulation frame
// ============================================================

bool runSimulationFrame(
    uint32_t frameId,
    int pointsPerHuman,
    PipelineMetrics &metrics
)
{
    // --------------------------------------------------------
    // Simulator generation is intentionally OUTSIDE the timing.
    // The real sensors generate the bytes, not the ESP32.
    // --------------------------------------------------------

    simulateFrame(
        frameId,
        pointsPerHuman
    );

    // --------------------------------------------------------
    // Ingest / parse / fusion
    // --------------------------------------------------------

    resetFrameAccumulator();

    uint64_t ingestStart =
        esp_timer_get_time();

    uint32_t lines =
        ingestSimulationFrame();

    uint64_t ingestEnd =
        esp_timer_get_time();

    uint64_t ingestUs =
        ingestEnd - ingestStart;

    if (!frameReady)
    {
        return false;
    }

    (void)lines;

    // --------------------------------------------------------
    // Grid + DBSCAN
    // --------------------------------------------------------

    uint64_t dbscanStart =
        esp_timer_get_time();

    int clusterCount =
        runGridDBSCAN(pointCount);

    uint64_t dbscanEnd =
        esp_timer_get_time();

    uint64_t dbscanUs =
        dbscanEnd - dbscanStart;

    // --------------------------------------------------------
    // Cluster/human analysis
    // --------------------------------------------------------

    uint64_t analysisStart =
        esp_timer_get_time();

    int humanCount = 0;

    analyzeClusters(
        pointCount,
        clusterCount,
        humanCount
    );

    uint64_t analysisEnd =
        esp_timer_get_time();

    uint64_t analysisUs =
        analysisEnd - analysisStart;

    uint64_t pipelineUs =
        ingestUs +
        dbscanUs +
        analysisUs;

    // --------------------------------------------------------
    // Metrics
    // --------------------------------------------------------

    metrics.frames++;
    metrics.framesReady++;

    metrics.totalInputPoints +=
        pointCount;

    metrics.totalIngestUs +=
        ingestUs;

    metrics.totalDbscanUs +=
        dbscanUs;

    metrics.totalAnalysisUs +=
        analysisUs;

    metrics.totalPipelineUs +=
        pipelineUs;

    if (pipelineUs > metrics.maxPipelineUs)
    {
        metrics.maxPipelineUs =
            pipelineUs;
    }

    if ((uint32_t)pointCount < metrics.minPoints)
    {
        metrics.minPoints =
            pointCount;
    }

    if ((uint32_t)pointCount > metrics.maxPoints)
    {
        metrics.maxPoints =
            pointCount;
    }

    double targetFrameUs =
        1000000.0 / SIM_TARGET_HZ;

    if ((double)pipelineUs > targetFrameUs)
    {
        metrics.realtimeOverruns++;
    }

    metrics.droppedPoints +=
        droppedPoints;

    droppedPoints = 0;

    // --------------------------------------------------------
    // Human movement / cluster printout
    // --------------------------------------------------------

    if (
        frameId == 0 ||
        frameId % PRINT_EVERY_N_FRAMES == 0
    )
    {
        printFrameResult(
            frameId,
            pointCount,
            clusterCount,
            humanCount,
            pipelineUs / 1000.0
        );
    }

    return true;
}


// ============================================================
// Experiment task
// ============================================================

void runExperiment07()
{
    Serial.println();
    Serial.println(
        "=============================================="
    );
    Serial.println(
        "ESP32-S3 DBSCAN"
    );
    Serial.println(
        "Experiment 07"
    );
    Serial.println(
        "Three-Sensor UART-Style Real-Time Pipeline"
    );
    Serial.println(
        "Spatial Grid + Human Movement Simulation"
    );
    Serial.println(
        "=============================================="
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

    Serial.println(
        "Simulation model: 2 moving humans + sensor noise"
    );

    Serial.printf(
        "Sensors: %d\n",
        SENSOR_COUNT
    );

    Serial.printf(
        "Target frame rate: %.1f Hz\n",
        SIM_TARGET_HZ
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
        MAX_POINTS_PER_FRAME
    );

    Serial.println();

    printMemoryInfo();

    Serial.println();

    // ========================================================
    // Load sweep
    // ========================================================

    const int loadCount =
        sizeof(SIM_POINTS_PER_HUMAN_LEVELS) /
        sizeof(SIM_POINTS_PER_HUMAN_LEVELS[0]);

    for (int load = 0;
         load < loadCount;
         load++)
    {
        int pointsPerHuman =
            SIM_POINTS_PER_HUMAN_LEVELS[load];

        Serial.println();
        Serial.println(
            "=============================================="
        );

        Serial.printf(
            "LOAD %d / %d\n",
            load + 1,
            loadCount
        );

        int expectedPoints =
            SENSOR_COUNT *
            (
                HUMAN_COUNT *
                pointsPerHuman +
                SIM_NOISE_POINTS_PER_SENSOR
            );

        Serial.printf(
            "Points per human per sensor: %d\n",
            pointsPerHuman
        );

        Serial.printf(
            "Expected points/frame: %d\n",
            expectedPoints
        );

        Serial.printf(
            "Expected input rate at %.1f Hz: %.1f points/s\n",
            SIM_TARGET_HZ,
            expectedPoints * SIM_TARGET_HZ
        );

        Serial.println();

        PipelineMetrics metrics{};

        resetMetrics(metrics);

        for (uint32_t frame = 0;
             frame < SIM_FRAMES_PER_LOAD;
             frame++)
        {
            runSimulationFrame(
                frame,
                pointsPerHuman,
                metrics
            );
        }

        printMetrics(
            metrics,
            pointsPerHuman
        );

        Serial.println(
            "=============================================="
        );

        // Allow the serial monitor to flush.
        delay(100);
    }

    Serial.println();
    Serial.println(
        "Experiment 07 completed."
    );
    Serial.println(
        "This simulation measured UART-style parsing,\n"
        "sensor fusion, spatial-grid DBSCAN, and\n"
        "human-like cluster analysis."
    );
    Serial.println(
        "=============================================="
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
        "Booting Experiment 07..."
    );

    if (!allocateBuffers())
    {
        Serial.println(
            "ERROR: PSRAM allocation failed."
        );

        while (true)
        {
            delay(1000);
        }
    }

    Serial.println(
        "PSRAM buffers allocated successfully."
    );

#if USE_REAL_UART
    setupRealUarts();
#else
    Serial.println(
        "Mode: SOFTWARE SENSOR SIMULATION"
    );
#endif

    resetFrameAccumulator();

    Serial.println();

    runExperiment07();
}


// ============================================================
// Loop
// ============================================================

void loop()
{
#if USE_REAL_UART
    // ========================================================
    // Future real-UART processing mode
    // ========================================================
    // The same parser accepts the same packet format from the
    // three physical UART ports. A complete frame is processed
    // after all three sensors send their END packet.

    if (!frameReady)
    {
        ingestRealUarts();
    }

    if (frameReady)
    {
        uint64_t start =
            esp_timer_get_time();

        int clusterCount =
            runGridDBSCAN(pointCount);

        int humanCount = 0;

        analyzeClusters(
            pointCount,
            clusterCount,
            humanCount
        );

        uint64_t end =
            esp_timer_get_time();

        Serial.printf(
            "REAL FRAME %lu | points=%d | clusters=%d | humans=%d | time=%.3f ms\n",
            (unsigned long)activeFrameId,
            pointCount,
            clusterCount,
            humanCount,
            (end - start) / 1000.0
        );

        resetFrameAccumulator();
    }
#else
    // Simulation benchmark runs from setup().
    delay(1000);
#endif
}
