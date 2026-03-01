#include <Arduino.h>
#include "config.h"
#include "motor_manager.h"
#include "uros_interface.h"

// ─── Globals ────────────────────────────────────────────────────────────────

static MotorManager   motors;
static UROSInterface* uros = nullptr;

static const uint32_t default_ids[DEFAULT_NUM_MOTORS] = {1, 2};

// ─── Setup ──────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(UROS_BAUD);
    pinMode(LED_PIN, OUTPUT);

    // Initialise CAN bus
    if (!motors.begin(static_cast<gpio_num_t>(CAN_TX_GPIO),
                      static_cast<gpio_num_t>(CAN_RX_GPIO))) {
        // CAN init failed — rapid blink
        while (true) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(100);
        }
    }

    motors.setMotorIds(default_ids, DEFAULT_NUM_MOTORS);

    // Initialise micro-ROS
    uros = new UROSInterface(motors);
    uros->begin();
}

// ─── Loop ───────────────────────────────────────────────────────────────────

void loop() {
    uros->spin();

    // Status LED
    static uint32_t last_blink = 0;
    uint32_t now = millis();

    switch (uros->getState()) {
        case WAITING_AGENT:
            // Slow blink — waiting for agent
            if (now - last_blink > 1000) {
                digitalWrite(LED_PIN, !digitalRead(LED_PIN));
                last_blink = now;
            }
            break;

        case AGENT_CONNECTED:
            digitalWrite(LED_PIN, HIGH);   // solid on
            break;

        case AGENT_DISCONNECTED:
            // Fast blink — lost agent
            if (now - last_blink > 200) {
                digitalWrite(LED_PIN, !digitalRead(LED_PIN));
                last_blink = now;
            }
            break;

        default:
            break;
    }
}
