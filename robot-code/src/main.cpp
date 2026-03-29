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

// ─── SPEED (0–100) ────────────────────────────────────────────────────────────
const uint8_t DRIVE_SPEED = 100;

// ─── PAUSE BETWEEN MOVES (ms) ────────────────────────────────────────────────
const uint16_t MOVE_GAP = 200;

// ─── FORWARD DECLARATIONS ────────────────────────────────────────────────────
void Motor_Init(void);
void Velocity_Controller(uint16_t angle, uint8_t velocity, int8_t rot, bool drift);
void Motors_Set(int8_t M0, int8_t M1, int8_t M2, int8_t M3);
void stopMotors(void);

void goForward(uint16_t ms);
void goBackward(uint16_t ms);
void goLeft(uint16_t ms);
void goRight(uint16_t ms);
void goForwardLeft(uint16_t ms);
void goForwardRight(uint16_t ms);
void goBackwardLeft(uint16_t ms);
void goBackwardRight(uint16_t ms);

// ─────────────────────────────────────────────────────────────────────────────
//  ★  MISSION — EDIT THIS SECTION TO SEQUENCE YOUR MOVES  ★
//
//  Each function takes a duration in milliseconds.
//  Example: goForward(2000) = drive forward for 2 seconds.
//
//  Available functions:
//    goForward(ms)        goBackward(ms)
//    goLeft(ms)           goRight(ms)
//    goForwardLeft(ms)    goForwardRight(ms)
//    goBackwardLeft(ms)   goBackwardRight(ms)
// ─────────────────────────────────────────────────────────────────────────────
void mission() {
    goForward(5000);          // forward  — tune ms for desired distance
    goRight(5000);            // right    — tune ms for desired distance
    goBackwardLeft(5000);     // bottom-left — tune ms for desired distance
}
// ─────────────────────────────────────────────────────────────────────────────

// ─── SETUP ───────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(9600);
    Serial.setTimeout(500);
    Motor_Init();
    mission();
    stopMotors();
}

void loop() {}

// ─── 8 DIRECTION FUNCTIONS ───────────────────────────────────────────────────
void goForward(uint16_t ms) {
    Velocity_Controller(0, DRIVE_SPEED, 0, 0);
    delay(ms);
    stopMotors();
    delay(MOVE_GAP);
}

void goBackward(uint16_t ms) {
    Velocity_Controller(180, DRIVE_SPEED, 0, 0);
    delay(ms);
    stopMotors();
    delay(MOVE_GAP);
}

void goLeft(uint16_t ms) {
    Velocity_Controller(90, DRIVE_SPEED, 0, 0);
    delay(ms);
    stopMotors();
    delay(MOVE_GAP);
}

void goRight(uint16_t ms) {
    Velocity_Controller(270, DRIVE_SPEED, 0, 0);
    delay(ms);
    stopMotors();
    delay(MOVE_GAP);
}

void goForwardLeft(uint16_t ms) {
    Velocity_Controller(45, DRIVE_SPEED, 0, 0);
    delay(ms);
    stopMotors();
    delay(MOVE_GAP);
}

void goForwardRight(uint16_t ms) {
    Velocity_Controller(315, DRIVE_SPEED, 0, 0);
    delay(ms);
    stopMotors();
    delay(MOVE_GAP);
}

void goBackwardLeft(uint16_t ms) {
    Velocity_Controller(135, DRIVE_SPEED, 0, 0);
    delay(ms);
    stopMotors();
    delay(MOVE_GAP);
}

void goBackwardRight(uint16_t ms) {
    Velocity_Controller(225, DRIVE_SPEED, 0, 0);
    delay(ms);
    stopMotors();
    delay(MOVE_GAP);
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
