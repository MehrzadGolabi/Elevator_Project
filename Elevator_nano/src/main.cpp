/************************************************************
 * Arduino Nano as I2C SLAVE for Elevator Cabin
 * 
 * Features:
 *  - Reads cabin’s 4 push buttons via resistor ladder (ADC).
 *  - Controls a servo door (open/close).
 *  - Controls a buzzer (on/off).
 *  - Sends 8 bytes of status to the Pico (master) when requested:
 *      Byte 0: Door status ( 'O' for open, 'C' for closed )
 *      Byte 1: Buzzer status ( '1' = on, '0' = off )
 *      Byte 2..5: 4 floor entries, e.g. '1','2','3','4' or 'N' if none
 *      Byte 6..7: (reserved) we fill with 'X' for now
 *  - Receives textual commands from Pico, e.g. "openDoor", 
 *    "closeDoor", "buzzerOn", "buzzerOff".
 ************************************************************/
#include<Arduino.h>
#include <Wire.h>
#include <Servo.h>

/*************** I2C Slave Address ***************/
#define SLAVE_ADDRESS 0x08

/*************** Pin Definitions ***************/
#define SERVO_PIN     9   // The servo controlling the door
#define BUZZER_PIN    8   // Buzzer or speaker
#define CABIN_ADC     A0  // The ADC pin for the 4 push buttons

/*************** Global Variables ***************/
// Door/buzzer states
bool doorIsOpen    = false;
bool buzzerIsOn    = false;

// Cabin floor array from the 4 push buttons
// We'll store up to 4 floors pressed: each is 1..4 or -1 if none
short cabinFloors[4] = { -1, -1, -1, -1 };

// We’ll read from the ADC in loop() and update cabinFloors accordingly
// The number of floors you can store is up to you. 
// This is a simplistic approach. For a queue, you'd store them differently.

// We will send 8 bytes back to the Pico
// Format: 
//  dataOut[0] = doorIsOpen ? 'O' : 'C';
//  dataOut[1] = buzzerIsOn ? '1' : '0';
//  dataOut[2..5] = cabinFloors[i] in ASCII or 'N' if none
//  dataOut[6..7] = 'X' (reserved)
char dataOut[8];

// The servo
Servo doorServo;

// For “open” and “closed” angles
// Adjust these angles for your physical door mechanism
const int DOOR_OPEN_ANGLE  = 90;
const int DOOR_CLOSED_ANGLE= 0;

/************************************************
 * Setup
 ************************************************/
void setup() {
  Serial.begin(9600);

  // Init servo
  doorServo.attach(SERVO_PIN);
  closeDoorInstant(); // start closed

  // Buzzer pin
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // I2C as slave
  Wire.begin(SLAVE_ADDRESS);
  // Register callbacks:
  Wire.onReceive(receiveEvent);  // When master writes to us
  Wire.onRequest(requestEvent);  // When master requests data

  // ADC pin as input
  pinMode(CABIN_ADC, INPUT);

  Serial.println("Nano: I2C Slave started at address 0x08");
}

/************************************************
 * Loop
 ************************************************/
void loop() {
  // Continuously read the cabin’s 4 push buttons from the ADC
  // For example, threshold logic for a 10-bit ADC (0..1023) 
  // on the Nano. Adjust as needed for your resistor-ladder.
  // If you use a 5 V Nano, the ADC range is 0..1023. 
  // If you have different resistor combos, calibrate them here.

  int adcVal = analogRead(CABIN_ADC);
  short pressedFloor = -1;

  // Example threshold approach (adjust to your resistor values):
  if      (adcVal < 100)               pressedFloor = 1;
  else if (adcVal >= 100 && adcVal < 300)  pressedFloor = 2;
  else if (adcVal >= 300 && adcVal < 500)  pressedFloor = 3;
  else if (adcVal >= 500 && adcVal < 700)  pressedFloor = 4;
  else                                    pressedFloor = -1;

  // If a valid floor is found, insert it in cabinFloors if not repeated
  if (pressedFloor != -1) {
    addCabinFloor(pressedFloor);
  }

  // [Optional] If you want to do some other logic or reading,
  // you can do it here.

  delay(50);
}

/************************************************
 * Add a floor to cabinFloors if not already present
 ************************************************/
void addCabinFloor(short floor) {
  // Check if it’s already in array
  for (int i=0; i<4; i++) {
    if (cabinFloors[i] == floor) {
      // Already present, do nothing
      return;
    }
  }
  // Insert in first empty slot
  for (int i=0; i<4; i++) {
    if (cabinFloors[i] == -1) {
      cabinFloors[i] = floor;
      Serial.print("Cabin floor pressed: ");
      Serial.println(floor);
      return;
    }
  }
  // If it's full, you can decide to discard or shift, etc.
  Serial.println("CabinFloors array full - ignoring new press.");
}

/************************************************
 * I2C: onRequest - Master wants 8 bytes
 ************************************************/
void requestEvent() {
  // dataOut[0] => 'O' or 'C' for door status
  dataOut[0] = (doorIsOpen) ? 'O' : 'C';

  // dataOut[1] => '1' or '0' for buzzer
  dataOut[1] = (buzzerIsOn) ? '1' : '0';

  // dataOut[2..5] => cabinFloors
  // If cabinFloors[i] is -1 => 'N'
  for (int i = 0; i < 4; i++) {
    if (cabinFloors[i] >= 1 && cabinFloors[i] <= 4) {
      dataOut[2 + i] = (char)('0' + cabinFloors[i]);
    } else {
      dataOut[2 + i] = 'N';
    }
  }

  // dataOut[6..7] => fill with 'X' or something else
  dataOut[6] = 'X';
  dataOut[7] = 'X';

  // Now write all 8 bytes to the master
  Wire.write((uint8_t*)dataOut, 8);
}

/************************************************
 * I2C: onReceive - Master writes commands
 * e.g., "openDoor", "closeDoor", "buzzerOn", "buzzerOff"
 ************************************************/
void receiveEvent(int howMany) {
  // Read all incoming bytes into a buffer
  char commandBuf[16];
  int idx = 0;
  while(Wire.available() > 0 && idx < (sizeof(commandBuf)-1)) {
    commandBuf[idx++] = Wire.read();
  }
  commandBuf[idx] = '\0'; // null-terminate

  Serial.print("Received command from master: ");
  Serial.println(commandBuf);

  // Compare with known commands
  if (strcmp(commandBuf, "openDoor") == 0) {
    openDoor();
    buzzerOn();
  }
  else if (strcmp(commandBuf, "closeDoor") == 0) {
    closeDoor();
    buzzerOn();
  }
  else if (strcmp(commandBuf, "buzzerOn") == 0) {
    buzzerOn();
  }
  else if (strcmp(commandBuf, "buzzerOff") == 0) {
    buzzerOff();
  }
  // If you have more commands, add them here.
}

/************************************************
 * Door Controls
 ************************************************/
void openDoor() {
  if (!doorIsOpen) {
    Serial.println("Opening door (servo)...");
    doorServo.write(DOOR_OPEN_ANGLE);
    delay(500); // Give servo a bit of time 
    doorIsOpen = true;
  } else {
    Serial.println("Door already open.");
  }
}

void closeDoor() {
  if (doorIsOpen) {
    Serial.println("Closing door (servo)...");
    doorServo.write(DOOR_CLOSED_ANGLE);
    delay(500);
    doorIsOpen = false;
  } else {
    Serial.println("Door already closed.");
  }
}

/**
 * If you want the servo to start closed in setup() 
 * without going via openDoor/closeDoor logic
 */
void closeDoorInstant() {
  doorServo.write(DOOR_CLOSED_ANGLE);
  doorIsOpen = false;
}

/************************************************
 * Buzzer Controls
 ************************************************/
void buzzerOn() {
  Serial.println("Buzzer ON");
  digitalWrite(BUZZER_PIN, HIGH);
  buzzerIsOn = true;

  // optionally beep for 1 second or so
  // then turn off automatically, if you prefer
  // delay(1000);
  // buzzerOff();
}

void buzzerOff() {
  Serial.println("Buzzer OFF");
  digitalWrite(BUZZER_PIN, LOW);
  buzzerIsOn = false;
}
