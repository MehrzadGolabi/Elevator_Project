// Slave (Arduino Nano)
#include <Arduino.h>
#include <Wire.h>

#define SLAVE_ADDRESS 0x08
#define OUTPUT_PIN 2

void setup() {
    pinMode(OUTPUT_PIN, OUTPUT); // Set pin 2 as output
    Wire.begin(SLAVE_ADDRESS);   // Join I2C bus with address 0x08
    Wire.onReceive(receiveEvent); // Register receive event handler
    Serial.begin(9600);          // Start Serial communication for debugging
    Serial.println("Slave ready.");
}

void loop() {
    // Nothing to do here, waiting for commands in receiveEvent
}

void receiveEvent(int bytes) {
    while (Wire.available()) { // Check for incoming bytes
        char command = Wire.read();
        if (command == 'a') {
            digitalWrite(OUTPUT_PIN, HIGH); // Set pin 2 HIGH
            Serial.println("Command 'a' received. Pin 2 set HIGH.");
        }
    }
}
