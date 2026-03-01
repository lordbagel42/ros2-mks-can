#include "motor_manager.h"
#include <cmath>

MotorManager::MotorManager() {}

bool MotorManager::begin(gpio_num_t tx_pin, gpio_num_t rx_pin) {
    _bus = new TwaiCan(tx_pin, rx_pin);
    return MKSServoCAN::begin(_bus);
}

void MotorManager::setMotorIds(const uint32_t* ids, uint8_t count) {
    _motor_count = min(count, (uint8_t)MAX_MOTORS);
    for (uint8_t i = 0; i < _motor_count; i++) {
        _states[i].can_id     = ids[i];
        _states[i].data_valid = false;
    }
}

// ─── Telemetry ──────────────────────────────────────────────────────────────

void MotorManager::requestTelemetry() {
    for (uint8_t i = 0; i < _motor_count; i++) {
        uint32_t id = _states[i].can_id;
        MKSServoCAN::readEncoderAdd(id);   // 0x31 – position
        MKSServoCAN::readSpeed(id);        // 0x32 – velocity
        MKSServoCAN::readAngleError(id);   // 0x39 – effort / error
    }
}

void MotorManager::processResponses() {
    CanFrame frame;
    while (_bus->receive(frame, CAN_RESPONSE_TIMEOUT_MS)) {
        parseResponse(frame);
    }
}

void MotorManager::parseResponse(const CanFrame& frame) {
    int8_t idx = findMotorIndex(frame.id);
    if (idx < 0) return;

    MotorState& s   = _states[idx];
    uint8_t     code = frame.data[0];
    s.last_update_us = micros();
    s.data_valid     = true;

    switch (code) {
        // ── Encoder cumulative (0x31) ────────────────────────────────────
        case 0x31:
            if (frame.dlc >= 8) {
                int64_t v = 0;
                for (int i = 1; i <= 6; ++i)
                    v = (v << 8) | frame.data[i];
                if (v & (int64_t(1) << 47))          // sign-extend 48-bit
                    v |= ~((int64_t(1) << 48) - 1);
                s.encoder_value = v;
                s.position_rad  = ENCODER_TO_RAD(v);
            }
            break;

        // ── Speed RPM (0x32) ─────────────────────────────────────────────
        case 0x32:
            if (frame.dlc >= 4) {
                s.speed_rpm     = (int16_t)(frame.data[1] << 8 | frame.data[2]);
                s.velocity_rad_s = RPM_TO_RAD_S(s.speed_rpm);
            }
            break;

        // ── Angle error (0x39) ───────────────────────────────────────────
        case 0x39:
            if (frame.dlc >= 6) {
                s.angle_error = (int32_t)(
                    (uint32_t)frame.data[1] << 24 |
                    (uint32_t)frame.data[2] << 16 |
                    (uint32_t)frame.data[3] <<  8 |
                              frame.data[4]);
                s.effort = ENCODER_TO_RAD(s.angle_error);
            }
            break;

        // ── Motor status (0xF1) ──────────────────────────────────────────
        case 0xF1:
            if (frame.dlc >= 2) s.status = frame.data[1];
            break;

        // ── Enable response (0xF3) ──────────────────────────────────────
        case 0xF3:
            if (frame.dlc >= 2) s.enabled = (frame.data[1] == 1);
            break;

        // ── Protection state (0x3E) ─────────────────────────────────────
        case 0x3E:
            if (frame.dlc >= 2) s.protection = (frame.data[1] != 0);
            break;

        default:
            break;
    }
}

int8_t MotorManager::findMotorIndex(uint32_t can_id) const {
    for (uint8_t i = 0; i < _motor_count; i++) {
        if (_states[i].can_id == can_id) return (int8_t)i;
    }
    return -1;
}

// ─── Motion commands ────────────────────────────────────────────────────────

void MotorManager::cmdPosition(uint8_t index, double position_rad,
                               uint16_t speed, uint8_t accel) {
    if (index >= _motor_count) return;
    int32_t axis = RAD_TO_ENCODER(position_rad);
    MKSServoCAN::posAbsolute(_states[index].can_id, axis,
                             min(speed, (uint16_t)MAX_SPEED_RPM), accel);
}

void MotorManager::cmdVelocity(uint8_t index, double velocity_rad_s,
                               uint8_t accel) {
    if (index >= _motor_count) return;
    bool     ccw = velocity_rad_s < 0;
    uint16_t rpm = RAD_S_TO_RPM(velocity_rad_s);
    rpm = min(rpm, (uint16_t)MAX_SPEED_RPM);
    MKSServoCAN::speedMode(_states[index].can_id, rpm, accel, ccw);
}

void MotorManager::cmdVelocityStop(uint8_t index) {
    if (index >= _motor_count) return;
    MKSServoCAN::speedModeStop(_states[index].can_id);
}

void MotorManager::cmdEnable(uint8_t index, bool enable) {
    if (index >= _motor_count) return;
    MKSServoCAN::enableMotor(_states[index].can_id, enable);
}

void MotorManager::cmdEnableAll(bool enable) {
    for (uint8_t i = 0; i < _motor_count; i++)
        MKSServoCAN::enableMotor(_states[i].can_id, enable);
}

void MotorManager::cmdEmergencyStop() {
    for (uint8_t i = 0; i < _motor_count; i++)
        MKSServoCAN::emergencyStop(_states[i].can_id);
}

void MotorManager::cmdHome(uint8_t index) {
    if (index >= _motor_count) return;
    MKSServoCAN::goHome(_states[index].can_id);
}

void MotorManager::cmdHomeAll() {
    for (uint8_t i = 0; i < _motor_count; i++)
        MKSServoCAN::goHome(_states[i].can_id);
}

void MotorManager::cmdSetZero(uint8_t index) {
    if (index >= _motor_count) return;
    MKSServoCAN::setZeroPoint(_states[index].can_id);
}
