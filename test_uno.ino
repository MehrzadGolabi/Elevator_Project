// Master (Arduino Uno)
#include <Arduino.h>
#include <Wire.h>

#define SLAVE_ADDRESS 0x08

void setup() {
    Serial.begin(9600); // Start Serial communication
    Wire.begin();       // Join I2C bus as master
    Serial.println("Master ready. Type 'a' to send to slave.");
}

void loop() {
    if (Serial.available() > 0) {
        char command = Serial.read();

        if (command == 'a') {
            Wire.beginTransmission(SLAVE_ADDRESS); // Start communication with slave
            Wire.write(command);                  // Send the command 'a'
            Wire.endTransmission();              // End transmission
            Serial.println("Command 'a' sent to slave.");
        } else {
            Serial.println("Invalid command. Only 'a' is supported.");
        }
    }
}
