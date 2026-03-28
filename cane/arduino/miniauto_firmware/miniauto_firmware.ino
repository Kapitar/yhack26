/*
 * LeadMe — miniAuto Firmware
 *
 * Bidirectional BLE communication via HM-10 module (transparent UART bridge).
 * Receives movement commands from laptop, streams IMU data back.
 *
 * ── Receive (laptop → Arduino via BLE write) ────────────────────────────────
 *   "<angle>|<velocity>|<rot>\n"
 *     angle    : 0–359 degrees  (0=forward, 90=strafe-left, 180=back, 270=strafe-right)
 *     velocity : 0–100
 *     rot      : -100 to 100   (positive=CCW, negative=CW)
 *   "S\n"  → stop immediately
 *   "P\n"  → ping, responds "OK\n"
 *
 * ── Send (Arduino → laptop via BLE notify) ──────────────────────────────────
 *   "I:<ax>,<ay>,<az>,<gx>,<gy>,<gz>\n"  at 20 Hz
 *     ax/ay/az : raw accel int16  (±2g range  → divide by 16384.0 for g)
 *     gx/gy/gz : raw gyro  int16  (±250°/s    → divide by 131.0 for °/s)
 *
 * ── Wiring ───────────────────────────────────────────────────────────────────
 *   MPU6050  SDA → Arduino A4
 *   MPU6050  SCL → Arduino A5
 *   MPU6050  VCC → 3.3V or 5V
 *   MPU6050  GND → GND
 *   MPU6050  AD0 → GND (I2C address 0x68)
 *
 *   HM-10 BLE module is wired to Arduino hardware Serial (pins 0/1) by
 *   the miniAuto board — no extra wiring needed.
 */

#include <Arduino.h>
#include <Wire.h>

// ── MPU6050 ──────────────────────────────────────────────────────────────────
#define MPU_ADDR      0x68
#define REG_PWR_MGMT  0x6B
#define REG_ACCEL_OUT 0x3B   // first of 14 consecutive bytes: AX AY AZ TEMP GX GY GZ

struct ImuData {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
};

void mpu_init() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_PWR_MGMT);
    Wire.write(0x00);           // wake up (clear sleep bit)
    Wire.endTransmission(true);

    // Gyro range: ±250°/s (default, register 0x1B = 0x00)
    // Accel range: ±2g  (default, register 0x1C = 0x00)
    // No changes needed for defaults.
}

bool mpu_read(ImuData &d) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_OUT);
    if (Wire.endTransmission(false) != 0) return false;

    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);
    if (Wire.available() < 14) return false;

    d.ax = (int16_t)(Wire.read() << 8 | Wire.read());
    d.ay = (int16_t)(Wire.read() << 8 | Wire.read());
    d.az = (int16_t)(Wire.read() << 8 | Wire.read());
    Wire.read(); Wire.read();   // skip temperature
    d.gx = (int16_t)(Wire.read() << 8 | Wire.read());
    d.gy = (int16_t)(Wire.read() << 8 | Wire.read());
    d.gz = (int16_t)(Wire.read() << 8 | Wire.read());
    return true;
}

// ── Motor pins ────────────────────────────────────────────────────────────────
const uint8_t PWM_PIN[4]   = {10, 9, 6, 11};
const uint8_t DIR_PIN[4]   = {12, 8, 7, 13};
const uint8_t PWM_MIN      = 50;
const bool    FWD_DIR[4]   = {1, 0, 0, 1};

void motors_set(int8_t m0, int8_t m1, int8_t m2, int8_t m3) {
    int8_t speeds[4] = {m0, m1, m2, m3};
    for (uint8_t i = 0; i < 4; i++) {
        bool dir = FWD_DIR[i];
        if (speeds[i] < 0) dir = !dir;
        uint8_t pwm = (speeds[i] == 0) ? 0 : (uint8_t)map(abs(speeds[i]), 0, 100, PWM_MIN, 255);
        digitalWrite(DIR_PIN[i], dir);
        analogWrite(PWM_PIN[i], pwm);
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
        stop_motors();  // malformed command → safe stop
    }
}

// ── Watchdog ──────────────────────────────────────────────────────────────────
#define WATCHDOG_MS 500UL
unsigned long last_cmd_ms = 0;

// ── IMU transmit timing ───────────────────────────────────────────────────────
#define IMU_INTERVAL_MS 50UL   // 20 Hz
unsigned long last_imu_ms = 0;

// ── Setup & loop ──────────────────────────────────────────────────────────────
void setup() {
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(DIR_PIN[i], OUTPUT);
        pinMode(PWM_PIN[i], OUTPUT);
    }
    stop_motors();

    Wire.begin();
    mpu_init();

    Serial.begin(9600);
    last_cmd_ms = millis();
    last_imu_ms = millis();
}

void loop() {
    unsigned long now = millis();

    // ── Receive movement commands ────────────────────────────────────────────
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

    // ── Watchdog: stop if laptop goes silent ─────────────────────────────────
    if (now - last_cmd_ms > WATCHDOG_MS) {
        stop_motors();
    }

    // ── Stream IMU data to laptop at 20 Hz ───────────────────────────────────
    if (now - last_imu_ms >= IMU_INTERVAL_MS) {
        last_imu_ms = now;
        ImuData d;
        if (mpu_read(d)) {
            // Format: "I:<ax>,<ay>,<az>,<gx>,<gy>,<gz>\n"
            // Raw int16 values — laptop applies scale factors:
            //   accel: value / 16384.0 → g
            //   gyro:  value / 131.0   → °/s
            Serial.print("I:");
            Serial.print(d.ax); Serial.print(',');
            Serial.print(d.ay); Serial.print(',');
            Serial.print(d.az); Serial.print(',');
            Serial.print(d.gx); Serial.print(',');
            Serial.print(d.gy); Serial.print(',');
            Serial.println(d.gz);
        }
    }
}
