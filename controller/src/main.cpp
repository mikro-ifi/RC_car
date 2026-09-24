#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// ----------------------------------------------------
// VEHICLE ESP32 MAC ADDRESS
// ----------------------------------------------------

uint8_t vehicleAddress[] = {
    0x70, 0x4B, 0xCA, 0x04, 0x5D, 0x58
};

// ----------------------------------------------------
// JOYSTICK PINS
// ----------------------------------------------------

// Throttle joystick VRx -> GPIO34
const int PIN_THROTTLE = 34;

// Steering joystick VRy -> GPIO35
const int PIN_STEERING = 35;

// ----------------------------------------------------
// DATA PACKET
// ----------------------------------------------------

struct ControlData
{
    int16_t throttle;   // -100 to +100
    int16_t steering;   // -100 to +100
};

ControlData myData;

esp_now_peer_info_t peerInfo;

// ----------------------------------------------------
// JOYSTICK SETTINGS
// ----------------------------------------------------

const int ADC_MIN = 0;
const int ADC_MAX = 4095;
const int ADC_CENTER = 2048;

// Raw ADC deadzone around center
const int DEADZONE = 250;

// ----------------------------------------------------
// CONVERT JOYSTICK TO -100 ... +100
// ----------------------------------------------------

int16_t joystickToPercent(int rawValue)
{
    // Neutral
    if (
        rawValue >= ADC_CENTER - DEADZONE &&
        rawValue <= ADC_CENTER + DEADZONE
    )
    {
        return 0;
    }

    // Positive direction
    if (rawValue > ADC_CENTER + DEADZONE)
    {
        int value = map(
            rawValue,
            ADC_CENTER + DEADZONE,
            ADC_MAX,
            0,
            100
        );

        return constrain(value, 0, 100);
    }

    // Negative direction
    int value = map(
        rawValue,
        ADC_CENTER - DEADZONE,
        ADC_MIN,
        0,
        -100
    );

    return constrain(value, -100, 0);
}

// ----------------------------------------------------
// SETUP
// ----------------------------------------------------

void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("=== CONTROLLER STARTING ===");

    WiFi.mode(WIFI_STA);

    Serial.print("Controller MAC: ");
    Serial.println(WiFi.macAddress());

    // Start ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("ERROR: ESP-NOW initialization failed");
        return;
    }

    // Clear peer configuration
    memset(&peerInfo, 0, sizeof(peerInfo));

    // Vehicle address
    memcpy(peerInfo.peer_addr, vehicleAddress, 6);

    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add vehicle as peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("ERROR: Failed to add vehicle");
        return;
    }

    Serial.println("Controller ready.");
}

// ----------------------------------------------------
// MAIN LOOP
// ----------------------------------------------------

void loop()
{
    // Read joystick ADC values
    int rawThrottle = analogRead(PIN_THROTTLE);
    int rawSteering = analogRead(PIN_STEERING);

    // Convert to -100 ... +100
    myData.throttle = joystickToPercent(rawThrottle);
    myData.steering = joystickToPercent(rawSteering);

    // ------------------------------------------------
    // SERIAL DEBUG
    // ------------------------------------------------

    Serial.print("Throttle raw: ");
    Serial.print(rawThrottle);

    Serial.print(" -> ");
    Serial.print(myData.throttle);
    Serial.print("%");

    Serial.print("    |    Steering raw: ");
    Serial.print(rawSteering);

    Serial.print(" -> ");
    Serial.print(myData.steering);
    Serial.println("%");

    // ------------------------------------------------
    // SEND TO VEHICLE
    // ------------------------------------------------

    esp_err_t result = esp_now_send(
        vehicleAddress,
        (uint8_t *)&myData,
        sizeof(myData)
    );

    if (result != ESP_OK)
    {
        Serial.println("ESP-NOW send error");
    }

    // Approximately 50 updates per second
    delay(20);
}

