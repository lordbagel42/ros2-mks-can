#pragma once

#include <Arduino.h>
#include <TwaiCan.h>
#include <MKSServoCAN.h>
#include "config.h"

/// Per-motor state updated from CAN responses.
struct MotorState {
    uint32_t can_id          = 0;
    int64_t  encoder_value   = 0;       // cumulative encoder count
    int16_t  speed_rpm       = 0;       // real-time RPM from driver
    int32_t  angle_error     = 0;       // angle error in encoder ticks
    uint8_t  status          = 0;       // motor status byte (0xF1)
    bool     enabled         = false;
    bool     protection      = false;
    uint32_t last_update_us  = 0;       // micros() of last CAN response
    bool     data_valid      = false;   // true after first response

    // Derived SI values (computed on update)
    double   position_rad    = 0.0;
    double   velocity_rad_s  = 0.0;
    double   effort          = 0.0;     // angle error in radians
};

/// Manages CAN communication with one or more MKS SERVO57D motors.
class MotorManager {
public:
    MotorManager();

    /// Initialise the TWAI bus via MKSServoCAN.
    bool begin(gpio_num_t tx_pin, gpio_num_t rx_pin);

    /// Set the CAN IDs of the motors to manage.
    void setMotorIds(const uint32_t* ids, uint8_t count);

    uint8_t getMotorCount() const { return _motor_count; }

    const MotorState& getState(uint8_t index) const { return _states[index]; }

    // ── Telemetry ────────────────────────────────────────────────────────
    /// Send telemetry request frames for all motors.
    void requestTelemetry();

    /// Receive and parse any pending CAN response frames.
    void processResponses();

    // ── Motion commands ──────────────────────────────────────────────────
    void cmdPosition(uint8_t index, double position_rad,
                     uint16_t speed = DEFAULT_SPEED,
                     uint8_t  accel = DEFAULT_ACCEL);
    void cmdVelocity(uint8_t index, double velocity_rad_s,
                     uint8_t accel = DEFAULT_ACCEL);
    void cmdVelocityStop(uint8_t index);
    void cmdEnable(uint8_t index, bool enable);
    void cmdEnableAll(bool enable);
    void cmdEmergencyStop();
    void cmdHome(uint8_t index);
    void cmdHomeAll();
    void cmdSetZero(uint8_t index);

private:
    TwaiCan*   _bus          = nullptr;
    MotorState _states[MAX_MOTORS];
    uint8_t    _motor_count  = 0;

    void   parseResponse(const CanFrame& frame);
    int8_t findMotorIndex(uint32_t can_id) const;
};
