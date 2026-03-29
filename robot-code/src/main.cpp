#include <Arduino.h>

/**
 * @file main.cpp
 * @brief 电机驱动实验
 *
 * [0] --|||--[1]       数字对应motorpwmPin[4]和motordirectionPin[4]索引号
 *  |          |        ^逆时针
 *  |          |        |
 *  |          |        |
 *  |          |        |
 * [3] -------[2]
 */

const uint8_t pwm_min = 50;
const uint8_t motorpwmPin[4] = {10, 9, 6, 11};        // 控制轮子速度引脚
const uint8_t motordirectionPin[4] = {12, 8, 7, 13};  // 控制轮子方向引脚

void Motor_Init(void);
void Velocity_Controller(uint16_t angle, uint8_t velocity, int8_t rot, bool drift);
void Motors_Set(int8_t Motor_0, int8_t Motor_1, int8_t Motor_2, int8_t Motor_3);

unsigned long lastCommandTime = 0;
const unsigned long WATCHDOG_TIMEOUT = 500; // ms

void setup() {
    Serial.begin(9600);
    Serial.setTimeout(50); // Fast timeout for responsiveness

    Motor_Init();
    Velocity_Controller(0, 0, 0, 0); // Explicitly ensure we start stopped
}

void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        // Handle Ping Request
        if (input == "P") {
            Serial.println("OK");
            lastCommandTime = millis();
        } 
        // Handle explicit Stop Request
        else if (input == "S") {
            Velocity_Controller(0, 0, 0, 0);
            lastCommandTime = millis();
        } 
        // Handle Movement Command
        else {
            int firstSep = input.indexOf('|');
            int secondSep = input.lastIndexOf('|');
            
            if (firstSep != -1 && secondSep != -1 && firstSep != secondSep) {
                int angle = input.substring(0, firstSep).toInt();
                int velocity = input.substring(firstSep + 1, secondSep).toInt();
                int rot = input.substring(secondSep + 1).toInt();
                
                Velocity_Controller(angle, velocity, rot, false);
                lastCommandTime = millis();
            }
        }
    }
    
    // Safety Watchdog: If no valid commands for >500ms, stop motors
    if (millis() - lastCommandTime > WATCHDOG_TIMEOUT) {
        Velocity_Controller(0, 0, 0, 0);
    }
}

/* 电机初始化函数 */
void Motor_Init(void) {
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(motordirectionPin[i], OUTPUT);
        pinMode(motorpwmPin[i], OUTPUT);
    }
    Velocity_Controller(0, 0, 0, 0);
}

/**
 * @brief 速度控制函数
 * @param angle    用于控制小车的运动方向，小车以车头为0度方向，逆时针为正方向。取值为0~359
 * @param velocity 用于控制小车速度，取值为0~100
 * @param rot      用于控制小车的自转速度，取值为-100~100
 * @param drift    用于决定小车是否开启漂移功能
 */
void Velocity_Controller(uint16_t angle, uint8_t velocity, int8_t rot, bool drift) {
    int8_t velocity_0, velocity_1, velocity_2, velocity_3;
    float speed = 1.0f;

    angle += 90;
    float rad = angle * PI / 180.0f;

    if (rot == 0) {
        speed = 1.0f;
    } else {
        speed = 0.5f;
    }

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

/**
 * @brief PWM与轮子转向设置函数
 * @param Motor_x 作为PWM与电机转向的控制数值
 */
void Motors_Set(int8_t Motor_0, int8_t Motor_1, int8_t Motor_2, int8_t Motor_3) {
    int16_t pwm_set[4];
    int8_t motors[4] = {Motor_0, Motor_1, Motor_2, Motor_3};
    bool direction[4] = {1, 0, 0, 1};  // 前进 左1 右0

    for (uint8_t i = 0; i < 4; ++i) {
        if (motors[i] < 0) {
            direction[i] = !direction[i];
        }

        if (motors[i] == 0) {
            pwm_set[i] = 0;
        } else {
            pwm_set[i] = map(abs(motors[i]), 0, 100, pwm_min, 255);
        }

        digitalWrite(motordirectionPin[i], direction[i]);
        analogWrite(motorpwmPin[i], pwm_set[i]);
    }
}
