
#include <Arduino.h>
#include <math.h>

// ============================================================
// ESP32-S3 DBSCAN — PHASE 2
// Experiment 09: Kalman Filter Microbenchmark
// 3D constant-velocity Kalman filter
// ============================================================

static constexpr int SAMPLES = 3000;
static constexpr int WARMUP = 100;
static constexpr float DT = 0.1f;       // 10 Hz
static constexpr float PROCESS_Q = 0.8f;
static constexpr float NOISE_LEVELS[] = {0.025f, 0.05f, 0.10f};
static constexpr int NOISE_COUNT = 3;

struct Vec3 {
    float x, y, z;
};

// One constant-velocity Kalman filter for one axis.
// State: position, velocity.
// Covariance: p00, p01, p10, p11.
struct KalmanAxis {
    float pos = 0.0f;
    float vel = 0.0f;

    float p00 = 1.0f;
    float p01 = 0.0f;
    float p10 = 0.0f;
    float p11 = 1.0f;

    float q = PROCESS_Q;
    float r = 0.01f;
    bool initialized = false;

    void reset(float measurementVariance) {
        pos = 0.0f;
        vel = 0.0f;
        p00 = 1.0f;
        p01 = 0.0f;
        p10 = 0.0f;
        p11 = 1.0f;
        r = measurementVariance;
        initialized = false;
    }

    float update(float measurement, float dt) {
        if (!initialized) {
            pos = measurement;
            vel = 0.0f;
            p00 = 1.0f;
            p01 = 0.0f;
            p10 = 0.0f;
            p11 = 1.0f;
            initialized = true;
            return pos;
        }

        // Prediction
        pos += vel * dt;

        const float oldP00 = p00;
        const float oldP01 = p01;
        const float oldP10 = p10;
        const float oldP11 = p11;

        const float dt2 = dt * dt;
        const float dt3 = dt2 * dt;
        const float dt4 = dt2 * dt2;

        p00 = oldP00 + dt * (oldP01 + oldP10)
              + dt2 * oldP11 + q * dt4 * 0.25f;
        p01 = oldP01 + dt * oldP11 + q * dt3 * 0.5f;
        p10 = oldP10 + dt * oldP11 + q * dt3 * 0.5f;
        p11 = oldP11 + q * dt2;

        // Measurement update
        const float innovation = measurement - pos;
        const float innovationVariance = p00 + r;

        const float k0 = p00 / innovationVariance;
        const float k1 = p10 / innovationVariance;

        pos += k0 * innovation;
        vel += k1 * innovation;

        // Covariance update
        p00 = (1.0f - k0) * oldP00;
        p01 = (1.0f - k0) * oldP01;
        p10 = oldP10 - k1 * oldP00;
        p11 = oldP11 - k1 * oldP01;

        return pos;
    }
};

struct Kalman3D {
    KalmanAxis fx, fy, fz;

    void reset(float measurementVariance) {
        fx.reset(measurementVariance);
        fy.reset(measurementVariance);
        fz.reset(measurementVariance);
    }

    Vec3 update(const Vec3 &measurement, float dt) {
        return {
            fx.update(measurement.x, dt),
            fy.update(measurement.y, dt),
            fz.update(measurement.z, dt)
        };
    }
};

// Deterministic lightweight pseudo-random generator
static uint32_t rngState = 0x13579BDF;

uint32_t random32() {
    rngState ^= rngState << 13;
    rngState ^= rngState >> 17;
    rngState ^= rngState << 5;
    return rngState;
}

float uniform01() {
    return (random32() & 0x00FFFFFF) / 16777216.0f;
}

// Approximate zero-mean, unit-variance Gaussian using
// the central limit theorem. Noise generation is outside
// the measured Kalman-update timing.
float gaussianNoise() {
    float sum = 0.0f;
    for (int i = 0; i < 12; ++i) {
        sum += uniform01();
    }
    return sum - 6.0f;
}

Vec3 truePosition(float t) {
    return {
        0.7f * sinf(t * 0.55f),
        1.2f + 0.25f * sinf(t * 0.8f),
        0.9f + 0.30f * sinf(t * 0.3f)
    };
}

Vec3 addNoise(const Vec3 &p, float sigma) {
    return {
        p.x + sigma * gaussianNoise(),
        p.y + sigma * gaussianNoise(),
        p.z + sigma * gaussianNoise()
    };
}

void runNoiseTest(float sigma, int testIndex) {
    Kalman3D filter;
    filter.reset(sigma * sigma);

    double rawSquaredError = 0.0;
    double filteredSquaredError = 0.0;
    double rawAxisSquaredError = 0.0;
    double filteredAxisSquaredError = 0.0;

    uint64_t totalUpdateMicros = 0;
    float lastRawError = 0.0f;
    float lastFilteredError = 0.0f;

    // Reset the random stream for repeatable runs.
    rngState = 0x13579BDFu + (uint32_t)testIndex * 7919u;

    for (int i = 0; i < SAMPLES; ++i) {
        const float t = i * DT;
        const Vec3 truth = truePosition(t);
        const Vec3 measurement = addNoise(truth, sigma);

        const uint32_t start = micros();
        const Vec3 estimate = filter.update(measurement, DT);
        const uint32_t elapsed = micros() - start;

        totalUpdateMicros += elapsed;

        const float rx = measurement.x - truth.x;
        const float ry = measurement.y - truth.y;
        const float rz = measurement.z - truth.z;

        const float fx = estimate.x - truth.x;
        const float fy = estimate.y - truth.y;
        const float fz = estimate.z - truth.z;

        if (i >= WARMUP) {
            rawSquaredError += rx * rx + ry * ry + rz * rz;
            filteredSquaredError += fx * fx + fy * fy + fz * fz;

            rawAxisSquaredError += (rx * rx + ry * ry + rz * rz) / 3.0;
            filteredAxisSquaredError +=
                (fx * fx + fy * fy + fz * fz) / 3.0;

            lastRawError = sqrtf(rx * rx + ry * ry + rz * rz);
            lastFilteredError = sqrtf(fx * fx + fy * fy + fz * fz);
        }
    }

    const int evaluated = SAMPLES - WARMUP;

    const float rawRMSE =
        sqrtf((float)(rawSquaredError / (3.0 * evaluated)));
    const float filteredRMSE =
        sqrtf((float)(filteredSquaredError / (3.0 * evaluated)));

    const float reduction =
        rawRMSE > 0.0f
            ? (1.0f - filteredRMSE / rawRMSE) * 100.0f
            : 0.0f;

    const float avgUpdateUs =
        (float)totalUpdateMicros / SAMPLES;

    Serial.println();
    Serial.println("----------------------------------------------");
    Serial.printf("NOISE TEST %d: sigma = %.3f m\n", testIndex + 1, sigma);
    Serial.println("----------------------------------------------");
    Serial.printf("Samples:                 %d\n", SAMPLES);
    Serial.printf("Warmup excluded:         %d\n", WARMUP);
    Serial.printf("Raw position RMSE:       %.5f m\n", rawRMSE);
    Serial.printf("Filtered position RMSE:  %.5f m\n", filteredRMSE);
    Serial.printf("RMSE reduction:          %.2f %%\n", reduction);
    Serial.printf("Avg 3D update time:      %.3f us\n", avgUpdateUs);
    Serial.printf("Estimated filter rate:   %.0f updates/s\n",
                  avgUpdateUs > 0.0f ? 1000000.0f / avgUpdateUs : 0.0f);
    Serial.printf("Last raw error:          %.5f m\n", lastRawError);
    Serial.printf("Last filtered error:     %.5f m\n", lastFilteredError);
    Serial.printf("Free heap:               %u bytes\n",
                  (unsigned int)ESP.getFreeHeap());
    Serial.printf("Free PSRAM:              %u bytes\n",
                  (unsigned int)ESP.getFreePsram());
    Serial.println("----------------------------------------------");
}

void setup() {
    Serial.begin(115200);
    delay(1500);

    Serial.println();
    Serial.println("================================================");
    Serial.println("ESP32-S3 DBSCAN - PHASE 2");
    Serial.println("Experiment 09: Kalman Microbenchmark");
    Serial.println("3D Constant-Velocity Kalman Filter");
    Serial.println("================================================");

    Serial.printf("CPU: %u MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("Free heap: %u bytes\n",
                  (unsigned int)ESP.getFreeHeap());
    Serial.printf("Free PSRAM: %u bytes\n",
                  (unsigned int)ESP.getFreePsram());
    Serial.printf("Samples/test: %d\n", SAMPLES);
    Serial.printf("Sample interval: %.3f s\n", DT);
    Serial.printf("Process noise Q: %.3f\n", PROCESS_Q);

    for (int i = 0; i < NOISE_COUNT; ++i) {
        runNoiseTest(NOISE_LEVELS[i], i);
    }

    Serial.println();
    Serial.println("Experiment 09 completed.");
    Serial.println("Send the complete Serial Monitor output.");
}

void loop() {
    delay(1000);
}
