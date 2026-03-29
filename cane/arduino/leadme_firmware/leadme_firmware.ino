/*
 * LeadMe — Arduino Firmware (PID on Arduino)
 *
 * Receives heading error from laptop over USB Serial.
 * Runs PID on-board to compute lateral correction force.
 * Drives 4-wheel omni-directional base via velocity_controller().
 *
 * ── Receive (laptop → Arduino via USB Serial) ────────────────────────────────
 *   "E:<heading_error>\n"   e.g. "E:13.5\n" or "E:-7.2\n"
 *     heading_error : -180.0 to +180.0 degrees
 *                     positive → user pointing left of target → tug right
 *                     negative → user pointing right of target → tug left
 *   "S\n"  → stop immediately, reset PID
 *   "P\n"  → ping, responds "OK\n"
 *
 * ── Wiring ───────────────────────────────────────────────────────────────────
 *   Laptop USB → Arduino USB (Serial, 115200 baud)
 *   Motor driver PWM/DIR pins as defined below
 */

#include <Arduino.h>

// ── Motor pins ────────────────────────────────────────────────────────────────
const uint8_t PWM_PIN[4] = {10, 9,  6,  11};
const uint8_t DIR_PIN[4] = {12, 8,  7,  13};
const uint8_t PWM_MIN    = 50;
const bool    FWD_DIR[4] = {1,  0,  0,  1};

// ── Navigation constants ──────────────────────────────────────────────────────
const float DEAD_ZONE_DEG  = 5.0f;   // errors smaller than this → go straight
const uint8_t BASE_SPEED   = 55;     // base forward speed (0–100)

// ── PID gains ─────────────────────────────────────────────────────────────────
const float Kp = 1.2f;
const float Ki = 0.04f;
const float Kd = 0.25f;

const float INTEGRAL_LIMIT = 40.0f;
const float OUTPUT_LIMIT   = 100.0f;

// ── PID state ─────────────────────────────────────────────────────────────────
float    pid_integral = 0.0f;
uint32_t pid_prev_time = 0;

// ── Runtime state ─────────────────────────────────────────────────────────────
float current_error  = 0.0f;
float current_gyro_z = 0.0f;   // yaw rate in °/s from laptop (RealSense)
bool  stopped        = true;

// ── Watchdog ──────────────────────────────────────────────────────────────────
const uint32_t WATCHDOG_MS = 500UL;
uint32_t       last_cmd_ms = 0;

// ── Serial receive buffer ─────────────────────────────────────────────────────
#define BUF_SIZE 32
char    rx_buf[BUF_SIZE];
uint8_t rx_idx = 0;


// ── Motor control ─────────────────────────────────────────────────────────────

void motors_set(int8_t m0, int8_t m1, int8_t m2, int8_t m3) {
    int8_t speeds[4] = {m0, m1, m2, m3};
    for (uint8_t i = 0; i < 4; i++) {
        bool dir = FWD_DIR[i];
        if (speeds[i] < 0) dir = !dir;
        uint8_t pwm = (speeds[i] == 0)
                      ? 0
                      : (uint8_t)map(abs(speeds[i]), 0, 100, PWM_MIN, 255);
        digitalWrite(DIR_PIN[i], dir);
        analogWrite(PWM_PIN[i], pwm);
    }
}

// angle: 0=forward, 90=strafe-left, 180=back, 270=strafe-right
// velocity: 0–100, rot: -100 to +100
void velocity_controller(uint16_t angle, uint8_t velocity, int8_t rot) {
    angle += 90;
    float rad   = angle * PI / 180.0f;
    float speed = (rot == 0) ? 1.0f : 0.5f;
    float vel   = velocity / sqrtf(2.0f);

    int8_t vals[4];
    vals[0] = (int8_t)constrain((vel * sinf(rad) - vel * cosf(rad)) * speed + rot * speed, -100, 100);
    vals[1] = (int8_t)constrain((vel * sinf(rad) + vel * cosf(rad)) * speed - rot * speed, -100, 100);
    vals[2] = (int8_t)constrain((vel * sinf(rad) - vel * cosf(rad)) * speed - rot * speed, -100, 100);
    vals[3] = (int8_t)constrain((vel * sinf(rad) + vel * cosf(rad)) * speed + rot * speed, -100, 100);
    motors_set(vals[0], vals[1], vals[2], vals[3]);
}

void stop_motors() {
    motors_set(0, 0, 0, 0);
}


// ── PID ───────────────────────────────────────────────────────────────────────

void pid_reset() {
    pid_integral  = 0.0f;
    pid_prev_time = millis();
}

// Returns correction in [-100, 100]
// positive → tug right, negative → tug left
// gyro_z_dps: yaw rate in °/s — used directly for D-term (no derivative kick)
float pid_compute(float error, float gyro_z_dps) {
    uint32_t now = millis();
    float dt = (now - pid_prev_time) / 1000.0f;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.05f;
    pid_prev_time = now;

    // Proportional
    float p = Kp * error;

    // Integral with anti-windup
    pid_integral += error * dt;
    pid_integral = constrain(pid_integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    float i = Ki * pid_integral;

    // Derivative: use gyro directly (negative because CCW gyro opposes error growth)
    float d = -Kd * gyro_z_dps;

    float output = p + i + d;
    return constrain(output, -OUTPUT_LIMIT, OUTPUT_LIMIT);
}


// ── Map PID output to motor angle + speed ─────────────────────────────────────
//
// Mirrors the blend logic in the original navigation.py:
//   correction > 0 → tug right  → angle blends toward 270°
//   correction < 0 → tug left   → angle blends toward 90°
//   forward bias always present (BASE_SPEED)

void apply_correction(float correction) {
    float abs_corr = fabsf(correction);

    if (abs_corr < DEAD_ZONE_DEG) {
        // Dead zone — just go straight forward
        velocity_controller(0, BASE_SPEED, 0);
        return;
    }

    int blend_angle;
    if (correction >= 0.0f) {
        // Tug right: 0° (forward) blending toward 270° (strafe-right)
        blend_angle = (360 - (int)min(90.0f, abs_corr * 0.6f)) % 360;
    } else {
        // Tug left: 0° blending toward 90° (strafe-left)
        blend_angle = (int)min(90.0f, abs_corr * 0.6f);
    }

    uint8_t tug_speed = (uint8_t)min(100, (int)(BASE_SPEED + abs_corr * 0.25f));

    velocity_controller((uint16_t)blend_angle, tug_speed, 0);
}


// ── Command handler ───────────────────────────────────────────────────────────

void handle_command(const char *cmd) {
    if (cmd[0] == 'P') {
        Serial.println("OK");
        return;
    }

    if (cmd[0] == 'S') {
        stopped = true;
        pid_reset();
        stop_motors();
        return;
    }

    // "E:<heading_error>:<gyro_z>"
    if (cmd[0] == 'E' && cmd[1] == ':') {
        float error  = 0.0f;
        float gyro_z = 0.0f;
        // find the second colon separating error from gyro_z
        const char *sep = strchr(cmd + 2, ':');
        if (sep) {
            error  = atof(cmd + 2);
            gyro_z = atof(sep + 1);
        } else {
            error = atof(cmd + 2);
        }
        if (error >= -180.0f && error <= 180.0f) {
            current_error  = error;
            current_gyro_z = gyro_z;
            stopped        = false;
            last_cmd_ms    = millis();
        }
        return;
    }

    // Unknown command — safe stop
    stopped = true;
    stop_motors();
}


// ── Setup ─────────────────────────────────────────────────────────────────────

void setup() {
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(DIR_PIN[i], OUTPUT);
        pinMode(PWM_PIN[i], OUTPUT);
    }
    stop_motors();
    Serial.begin(115200);
    pid_reset();
    last_cmd_ms = millis();
}


// ── Main loop ─────────────────────────────────────────────────────────────────

void loop() {
    uint32_t now = millis();

    // ── Receive commands from laptop ─────────────────────────────────────────
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (rx_idx > 0) {
                rx_buf[rx_idx] = '\0';
                handle_command(rx_buf);
                rx_idx = 0;
            }
        } else if (rx_idx < BUF_SIZE - 1) {
            rx_buf[rx_idx++] = c;
        }
    }

    // ── Watchdog — stop if laptop goes silent ─────────────────────────────────
    if (!stopped && (now - last_cmd_ms > WATCHDOG_MS)) {
        stopped = true;
        pid_reset();
        stop_motors();
        return;
    }

    if (stopped) return;

    // ── PID → motors ─────────────────────────────────────────────────────────
    float correction = pid_compute(current_error, current_gyro_z);
    apply_correction(correction);
}
