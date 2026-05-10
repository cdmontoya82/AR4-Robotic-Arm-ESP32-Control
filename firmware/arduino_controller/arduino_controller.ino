/**
 * PROJECT: AR4 Autonomous Robotic Arm - Servo Controller
 * DESCRIPTION: Handles serial communication with Python vision script 
 * and drives 5 MG996R/SG90 servos sequentially.
 * HARDWARE: Arduino Uno / Mega
 * AUTHOR: Cristian Montoya
 */

#include <Arduino.h>
#include <Servo.h>

// --- PIN DEFINITIONS (PWM) ---
const int PIN_BASE     = 3;
const int PIN_SHOULDER = 5;
const int PIN_ELBOW    = 6;
const int PIN_WRIST    = 9;
const int PIN_GRIPPER  = 10;

// --- SERVO OBJECTS ---
Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo wristServo;
Servo gripperServo;

void setup() {
    // Sychronized with Python script baud rate
    Serial.begin(115200); 
    
    // Attaching servos to defined PWM pins
    baseServo.attach(PIN_BASE);
    shoulderServo.attach(PIN_SHOULDER);
    elbowServo.attach(PIN_ELBOW);
    wristServo.attach(PIN_WRIST);
    gripperServo.attach(PIN_GRIPPER);

    // --- INITIAL SAFETY POSITION ---
    // Moving sequentially to prevent current spikes (Inrush current)
    baseServo.write(90);
    delay(200);
    shoulderServo.write(100);
    delay(200);
    elbowServo.write(85);
    delay(200);
    wristServo.write(90);
    delay(200);
    gripperServo.write(120);

    Serial.println("SYSTEM_READY");
}

void loop() {
    // Listen for incoming serial commands from Python
    if (Serial.available() > 0) {
        // Read incoming string until newline
        String incomingData = Serial.readStringUntil('\n');
        
        // Parsing CSV format: base,shoulder,elbow,wrist,gripper
        int comma1 = incomingData.indexOf(',');
        int comma2 = incomingData.indexOf(',', comma1 + 1);
        int comma3 = incomingData.indexOf(',', comma2 + 1);
        int comma4 = incomingData.indexOf(',', comma3 + 1);

        // Validation: Ensure all 5 values are present
        if (comma1 > 0 && comma2 > 0 && comma3 > 0 && comma4 > 0) {
            
            // Extract and convert substrings to integers
            int baseAngle     = incomingData.substring(0, comma1).toInt();
            int shoulderAngle = incomingData.substring(comma1 + 1, comma2).toInt();
            int elbowAngle    = incomingData.substring(comma2 + 1, comma3).toInt();
            int wristAngle    = incomingData.substring(comma3 + 1, comma4).toInt();
            int gripperAngle  = incomingData.substring(comma4 + 1).toInt();

            // --- SEQUENTIAL ACTUATION ---
            // Small delays mitigate power supply voltage drops
            baseServo.write(baseAngle);
            delay(15); 
            shoulderServo.write(shoulderAngle);
            delay(15);
            elbowServo.write(elbowAngle);
            delay(15);
            wristServo.write(wristAngle);
            delay(15);
            gripperServo.write(gripperAngle);
            
            // Acknowledge receipt to Python script
            Serial.println("OK: Moved"); 
        } else {
            Serial.println("ERR: Malformed Packet");
        }
    }
}
