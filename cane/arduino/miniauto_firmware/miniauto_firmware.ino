/*
 * LeadMe — miniAuto Firmware
 *
 * Receives movement commands from Raspberry Pi via USB Serial.
 * IMU is now handled by the RPi (RealSense D457) — no MPU6050 needed here.
 *
 * ── Receive (RPi → Arduino via USB Serial) ──────────────────────────────────
 *   "<angle>|<velocity>|<rot>\n"
 *     angle    : 0–359 degrees  (0=forward, 90=strafe-left, 180=back, 270=strafe-right)
 *     velocity : 0–100
 *     rot      : -100 to 100   (positive=CCW, negative=CW)
 *   "S\n"  → stop immediately
 *   "P\n"  → ping, responds "OK\n"
 *
 * ── Wiring ───────────────────────────────────────────────────────────────────
 *   Raspberry Pi USB → Arduino USB (Serial, 9600 baud)
 *   Motor driver IN/PWM pins as defined below
 */

#include <Arduino.h>

// ── Motor pins ────────────────────────────────────────────────────────────────
const uint8_t PWM_PIN[4]  = {10, 9,  6,  11};
const uint8_t DIR_PIN[4]  = {12, 8,  7,  13};
const uint8_t PWM_MIN     = 50;
const bool    FWD_DIR[4]  = {1,  0,  0,  1};

void motors_set(int8_t m0, int8_t m1, int8_t m2, int8_t m3) {
    int8_t speeds[4] = {m0, m1, m2, m3};
    for (uint8_t i = 0; i < 4; i++) {
        bool    dir = FWD_DIR[i];
        if (speeds[i] < 0) dir = !dir;
        uint8_t pwm = (speeds[i] == 0)
                      ? 0
                      : (uint8_t)map(abs(speeds[i]), 0, 100, PWM_MIN, 255);
        digitalWrite(DIR_PIN[i], dir);
        analogWrite(PWM_PIN[i],  pwm);
    }
}

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

void stop_motors() { motors_set(0, 0, 0, 0); }

// ── Serial command parser ─────────────────────────────────────────────────────
#define BUF_SIZE 32
char    rx_buf[BUF_SIZE];
uint8_t rx_idx = 0;

void handle_command(const char *cmd) {
    if (cmd[0] == 'P') { Serial.println("OK"); return; }
    if (cmd[0] == 'S') { stop_motors(); return; }

    int angle, velocity, rot;
    if (sscanf(cmd, "%d|%d|%d", &angle, &velocity, &rot) == 3) {
        angle    = ((angle % 360) + 360) % 360;
        velocity = constrain(velocity, 0, 100);
        rot      = constrain(rot, -100, 100);
        velocity_controller((uint16_t)angle, (uint8_t)velocity, (int8_t)rot);
    } else {
        stop_motors();   // malformed command → safe stop
    }
}

// ── Watchdog ──────────────────────────────────────────────────────────────────
#define WATCHDOG_MS 500UL
unsigned long last_cmd_ms = 0;

// ── Setup & loop ──────────────────────────────────────────────────────────────
void setup() {
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(DIR_PIN[i], OUTPUT);
        pinMode(PWM_PIN[i], OUTPUT);
    }
    stop_motors();
    Serial.begin(9600);
    last_cmd_ms = millis();
}

void loop() {
    unsigned long now = millis();

    // ── Receive movement commands from RPi ───────────────────────────────────
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (rx_idx > 0) {
                rx_buf[rx_idx] = '\0';
                handle_command(rx_buf);
                last_cmd_ms = now;
                rx_idx = 0;
            }
        } else if (rx_idx < BUF_SIZE - 1) {
            rx_buf[rx_idx++] = c;
        }
    }

    // ── Watchdog: stop if RPi goes silent ────────────────────────────────────
    if (now - last_cmd_ms > WATCHDOG_MS) {
        stop_motors();
    }
}
