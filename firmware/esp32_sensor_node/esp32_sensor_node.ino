/**
 * PROJECT: AR4 Autonomous Robotic Arm - Perception Node
 * DESCRIPTION: High-frequency distance sensing using VL53L0X (ToF) 
 * via I2C. Communicates spatial depth to the main Vision System.
 * HARDWARE: ESP32 DevKit V1 + VL53L0X Laser Sensor
 * AUTHOR: Cristian Montoya
 */

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h> // Library for Time-of-Flight sensing

// --- I2C CONFIGURATION ---
const int SDA_PIN = 21;
const int SCL_PIN = 22;

VL53L0X distanceSensor;

void setup() {
    // High-speed baud rate for low-latency telemetry
    Serial.begin(115200);
    
    // Initialize I2C with ESP32 standard pins
    Wire.begin(SDA_PIN, SCL_PIN);

    Serial.println("[*] Initializing ToF Sensor...");

    if (!distanceSensor.init()) {
        Serial.println("[-] ERROR: Sensor not detected. Check I2C wiring.");
        while (1); // Halt execution on hardware failure
    }

    // Long range / High accuracy mode configuration
    distanceSensor.setTimeout(500);
    distanceSensor.startContinuous();
    
    Serial.println("[+] Sensor READY. Streaming data...");
}

void loop() {
    // Read raw data in millimeters
    int distanceMm = distanceSensor.readRangeContinuousMillimeters();
    
    // Safety check for sensor timeout
    if (distanceSensor.timeoutOccurred()) {
        Serial.println("[-] ERROR: Sensor Timeout");
    } else {
        // Convert to centimeters for the Python vision script logic
        float distanceCm = distanceMm / 10.0;
        
        // FORMAT: "Distancia: XX.X cm" 
        // (Must match the parsing logic in vision_brazo_yolo.py)
        Serial.print("Distancia: ");
        Serial.print(distanceCm);
        Serial.println(" cm");
    }
    
    // 10Hz sampling rate to match the main control loop
    delay(100); 
}
