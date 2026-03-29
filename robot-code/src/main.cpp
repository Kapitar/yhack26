#include <Arduino.h>
/**
 * @file main.cpp
 * @brief 8-Direction Motor Control
 *
 * [0] --|||--[1]       Motor index layout
 *  |          |        ^ counter-clockwise positive
 *  |          |
 *  |          |
 * [3] -------[2]       (back wheels not present on this unit)
 */

// ─── HARDWARE CONFIG ──────────────────────────────────────────────────────────
const uint8_t pwm_min = 50;
const uint8_t motorpwmPin[4]       = {10, 9, 6, 11};
const uint8_t motordirectionPin[4] = {12, 8, 7, 13};

// ─── WATCHDOG ─────────────────────────────────────────────────────────────────
// If no command is received within this many ms, stop motors (safety).
const uint16_t WATCHDOG_MS = 500;
unsigned long  lastCmdTime  = 0;

// ─── FORWARD DECLARATIONS ────────────────────────────────────────────────────
void Motor_Init(void);
void Velocity_Controller(uint16_t angle, uint8_t velocity, int8_t rot, bool drift);
void Motors_Set(int8_t M0, int8_t M1, int8_t M2, int8_t M3);
void stopMotors(void);
void handleSerial(void);

// ─── SETUP ───────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(9600);
    Serial.setTimeout(100);
    Motor_Init();
    lastCmdTime = millis();
}

// ─── MAIN LOOP ───────────────────────────────────────────────────────────────
void loop() {
    handleSerial();

    // Watchdog: stop if host hasn't sent a command recently
    if (millis() - lastCmdTime > WATCHDOG_MS) {
        stopMotors();
    }
}

// ─── SERIAL COMMAND HANDLER ──────────────────────────────────────────────────
//
//  Protocol (ASCII, newline-terminated):
//    "P\n"                → ping, replies "OK\n"
//    "S\n"                → emergency stop
//    "angle|velocity|rot\n"  → move command
//        angle    : 0–359  (0=forward, 90=left, 180=back, 270=right)
//        velocity : 0–100
//        rot      : -100–100 (CCW positive)
//
void handleSerial(void) {
    if (!Serial.available()) return;

    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    lastCmdTime = millis();

    // Ping
    if (line == "P") {
        Serial.println("OK");
        return;
    }

    // Stop
    if (line == "S") {
        stopMotors();
        return;
    }

    // Move: "angle|velocity|rot"
    int sep1 = line.indexOf('|');
    int sep2 = line.lastIndexOf('|');
    if (sep1 < 0 || sep2 <= sep1) return;   // malformed

    int angle    = line.substring(0, sep1).toInt();
    int velocity = line.substring(sep1 + 1, sep2).toInt();
    int rot      = line.substring(sep2 + 1).toInt();

    angle    = ((angle % 360) + 360) % 360;
    velocity = constrain(velocity, 0, 100);
    rot      = constrain(rot, -100, 100);

    if (velocity == 0 && rot == 0) {
        stopMotors();
    } else {
        Velocity_Controller(angle, velocity, rot, false);
    }
}

// ─── STOP ────────────────────────────────────────────────────────────────────
void stopMotors(void) {
    Motors_Set(0, 0, 0, 0);
}

// ─── MOTOR INIT ──────────────────────────────────────────────────────────────
void Motor_Init(void) {
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(motordirectionPin[i], OUTPUT);
        pinMode(motorpwmPin[i], OUTPUT);
    }
    stopMotors();
}

// ─── VELOCITY CONTROLLER ─────────────────────────────────────────────────────
void Velocity_Controller(uint16_t angle, uint8_t velocity, int8_t rot, bool drift) {
    int8_t velocity_0, velocity_1, velocity_2, velocity_3;
    float speed = (rot == 0) ? 1.0f : 0.5f;
    angle += 90;
    float rad = angle * PI / 180.0f;
    velocity /= sqrt(2);
    if (drift) {
        velocity_0 = (velocity * sin(rad) - velocity * cos(rad)) * speed;
        velocity_1 = (velocity * sin(rad) + velocity * cos(rad)) * speed;
        velocity_2 = (velocity * sin(rad) - velocity * cos(rad)) * speed - rot * speed * 2;
        velocity_3 = (velocity * sin(rad) + velocity * cos(rad)) * speed + rot * speed * 2;
    } else {
        velocity_0 = (velocity * sin(rad) - velocity * cos(rad)) * speed + rot * speed;
        velocity_1 = (velocity * sin(rad) + velocity * cos(rad)) * speed - rot * speed;
        velocity_2 = (velocity * sin(rad) - velocity * cos(rad)) * speed - rot * speed;
        velocity_3 = (velocity * sin(rad) + velocity * cos(rad)) * speed + rot * speed;
    }
    Motors_Set(velocity_0, velocity_1, velocity_2, velocity_3);
}

// ─── MOTORS SET ──────────────────────────────────────────────────────────────
void Motors_Set(int8_t M0, int8_t M1, int8_t M2, int8_t M3) {
    int16_t pwm_set[4];
    int8_t  motors[4]    = {M0, M1, M2, M3};
    bool    direction[4] = {1, 0, 0, 1};
    for (uint8_t i = 0; i < 4; ++i) {
        if (motors[i] < 0) direction[i] = !direction[i];
        pwm_set[i] = (motors[i] == 0) ? 0 : map(abs(motors[i]), 0, 100, pwm_min, 255);
        digitalWrite(motordirectionPin[i], direction[i]);
        analogWrite(motorpwmPin[i], pwm_set[i]);
    }
}
