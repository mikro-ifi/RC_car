#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>

// ----------------------------------------------------
// DRV8833 MOTOR DRIVER PINS
// ----------------------------------------------------

const int PIN_STBY = 22;

const int PIN_AIN1 = 26;
const int PIN_AIN2 = 27;

// ----------------------------------------------------
// SERVO
// ----------------------------------------------------

const int PIN_SERVO = 18;

Servo steeringServo;

// ----------------------------------------------------
// MOTOR PWM SETTINGS
// ----------------------------------------------------

const int CHANNEL_AIN1 = 4;
const int CHANNEL_AIN2 = 5;

const int PWM_FREQUENCY = 5000;
const int PWM_RESOLUTION = 12;

const int PWM_MAX = 4095;

// ----------------------------------------------------
// RECEIVED DATA
// ----------------------------------------------------

struct ControlData
{
    int16_t throttle;   // -100 to +100
    int16_t steering;   // -100 to +100
};

ControlData incomingData;

// ----------------------------------------------------
// FAILSAFE
// ----------------------------------------------------

unsigned long lastReceiveTime = 0;

const unsigned long FAILSAFE_TIMEOUT = 500;

// ----------------------------------------------------
// STOP MOTOR
// ----------------------------------------------------

void stopMotor()
{
    ledcWrite(CHANNEL_AIN1, 0);
    ledcWrite(CHANNEL_AIN2, 0);
}

// ----------------------------------------------------
// DRIVE MOTOR
// ----------------------------------------------------

void driveMotor(int16_t throttle)
{
    // Extra protection around zero
    if (abs(throttle) <= 2)
    {
        stopMotor();
        return;
    }

    // ------------------------------------------------
    // FORWARD
    // ------------------------------------------------

    if (throttle > 0)
    {
        int pwmValue = map(
            throttle,
            0,
            100,
            0,
            PWM_MAX
        );

        pwmValue = constrain(
            pwmValue,
            0,
            PWM_MAX
        );

        ledcWrite(
            CHANNEL_AIN1,
            pwmValue
        );

        ledcWrite(
            CHANNEL_AIN2,
            0
        );
    }

    // ------------------------------------------------
    // REVERSE
    // ------------------------------------------------

    else
    {
        int pwmValue = map(
            abs(throttle),
            0,
            100,
            0,
            PWM_MAX
        );

        pwmValue = constrain(
            pwmValue,
            0,
            PWM_MAX
        );

        ledcWrite(
            CHANNEL_AIN1,
            0
        );

        ledcWrite(
            CHANNEL_AIN2,
            pwmValue
        );
    }
}

// ----------------------------------------------------
// ESP-NOW RECEIVE CALLBACK
// ----------------------------------------------------

void OnDataRecv(
    const uint8_t *mac,
    const uint8_t *incomingDataPtr,
    int len
)
{
    // Ignore packets with wrong size
    if (len != sizeof(ControlData))
    {
        return;
    }

    memcpy(
        &incomingData,
        incomingDataPtr,
        sizeof(incomingData)
    );

    lastReceiveTime = millis();

    // ------------------------------------------------
    // MOTOR
    // ------------------------------------------------

    driveMotor(
        incomingData.throttle
    );

    // ------------------------------------------------
    // STEERING
    // ------------------------------------------------

    int servoAngle = map(
        incomingData.steering,
        -100,
        100,
        0,
        180
    );

    servoAngle = constrain(
        servoAngle,
        0,
        180
    );

    steeringServo.write(
        servoAngle
    );

    // ------------------------------------------------
    // SERIAL DEBUG
    // ------------------------------------------------

    Serial.print("RX -> Throttle: ");
    Serial.print(incomingData.throttle);
    Serial.print("%");

    Serial.print("    Steering: ");
    Serial.print(incomingData.steering);
    Serial.print("%");

    Serial.print("    Servo: ");
    Serial.println(servoAngle);
}

// ----------------------------------------------------
// SETUP
// ----------------------------------------------------

void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("=== VEHICLE STARTING ===");

    // ------------------------------------------------
    // WIFI
    // ------------------------------------------------

    WiFi.mode(WIFI_STA);

    Serial.print("Vehicle MAC: ");
    Serial.println(WiFi.macAddress());

    // ------------------------------------------------
    // DRV8833 STANDBY
    // ------------------------------------------------

    pinMode(
        PIN_STBY,
        OUTPUT
    );

    // Enable motor driver
    digitalWrite(
        PIN_STBY,
        HIGH
    );

    // ------------------------------------------------
    // MOTOR PWM
    // ------------------------------------------------

    ledcSetup(
        CHANNEL_AIN1,
        PWM_FREQUENCY,
        PWM_RESOLUTION
    );

    ledcAttachPin(
        PIN_AIN1,
        CHANNEL_AIN1
    );

    ledcSetup(
        CHANNEL_AIN2,
        PWM_FREQUENCY,
        PWM_RESOLUTION
    );

    ledcAttachPin(
        PIN_AIN2,
        CHANNEL_AIN2
    );

    stopMotor();

    // ------------------------------------------------
    // SERVO
    // ------------------------------------------------

    steeringServo.attach(
        PIN_SERVO
    );

    steeringServo.write(90);

    // ------------------------------------------------
    // ESP-NOW
    // ------------------------------------------------

    if (esp_now_init() != ESP_OK)
    {
        Serial.println(
            "ERROR: ESP-NOW initialization failed"
        );

        return;
    }

    esp_now_register_recv_cb(
        OnDataRecv
    );

    Serial.println("Vehicle ready.");
}

// ----------------------------------------------------
// MAIN LOOP
// ----------------------------------------------------

void loop()
{
    // If controller packets disappear,
    // stop everything after 500 ms.

    if (
        millis() - lastReceiveTime >
        FAILSAFE_TIMEOUT
    )
    {
        stopMotor();

        steeringServo.write(90);
    }
}