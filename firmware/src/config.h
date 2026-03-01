#pragma once

// ─── CAN Bus Pin Configuration ─────────────────────────────────────────────
#ifndef CAN_TX_GPIO
#define CAN_TX_GPIO  27
#endif
#ifndef CAN_RX_GPIO
#define CAN_RX_GPIO  26
#endif

// ─── Motor Configuration ────────────────────────────────────────────────────
#ifndef MAX_MOTORS
#define MAX_MOTORS  6
#endif
#ifndef DEFAULT_NUM_MOTORS
#define DEFAULT_NUM_MOTORS  2
#endif

// ─── Encoder Configuration ──────────────────────────────────────────────────
// MKS SERVO57D: 16384 counts per revolution (14-bit encoder)
#ifndef ENCODER_CPR
#define ENCODER_CPR  16384
#endif

// ─── Timing Configuration ───────────────────────────────────────────────────
#ifndef TELEMETRY_HZ
#define TELEMETRY_HZ  100
#endif
#define TELEMETRY_PERIOD_MS  (1000 / TELEMETRY_HZ)

// ─── micro-ROS Serial Configuration ────────────────────────────────────────
#ifndef UROS_BAUD
#define UROS_BAUD  921600
#endif

// Agent ping timeout (ms) — keep short for fast reconnection
#define AGENT_PING_TIMEOUT   100
#define AGENT_PING_ATTEMPTS  1

// ─── QoS ────────────────────────────────────────────────────────────────────
#ifndef USE_BEST_EFFORT_QOS
#define USE_BEST_EFFORT_QOS  1
#endif

// ─── Unit Conversion Macros ─────────────────────────────────────────────────
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define ENCODER_TO_RAD(enc)   ((double)(enc) * 2.0 * M_PI / (double)ENCODER_CPR)
#define RAD_TO_ENCODER(rad)   ((int32_t)((rad) * (double)ENCODER_CPR / (2.0 * M_PI)))
#define RPM_TO_RAD_S(rpm)     ((double)(rpm) * 2.0 * M_PI / 60.0)
#define RAD_S_TO_RPM(rad_s)   ((uint16_t)(fabs(rad_s) * 60.0 / (2.0 * M_PI)))

// ─── CAN Timeouts ───────────────────────────────────────────────────────────
#define CAN_RESPONSE_TIMEOUT_MS  2
#define CAN_SEND_TIMEOUT_MS      10

// ─── Motion Defaults ────────────────────────────────────────────────────────
#define DEFAULT_ACCEL      10
#define DEFAULT_SPEED      500
#define MAX_SPEED_RPM      3000

// ─── LED ────────────────────────────────────────────────────────────────────
#define LED_PIN  2
