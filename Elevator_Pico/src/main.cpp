#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <LiquidCrystal_I2C.h>

#if defined(ARDUINO_ARCH_RP2040)
#include "pico/multicore.h"
#endif

/************** Pin Definitions  **************/
const int Floor1_reed = 1;   // GP0
const int Floor2_reed = 2;   // GP1
const int Floor3_reed = 4;   // GP2
const int Floor4_reed = 5;   // GP3

//GP5=SDA & GP7=SCL

const int Floor1_LED  = 9;   // GP6
const int Floor2_LED  = 10;   // GP7
const int Floor3_LED  = 11;   // GP8
const int Floor4_LED  = 12;   // GP9

#define motStp 10            // GP10
#define motdir 11            // GP11
#define trig   12            // GP12
#define echo   13            // GP13
#define CALIBRATION_SENSOR 14// GP14

// For the resistor-ladder from control room
#define controlroom_buttons A0 // e.g., GP26
#define SLAVE_READY_PIN     A1 // e.g., GP27

#define lcd_address   0x3F
#define SLAVE_ADDRESS 0x08

/************** Elevator Status Flags & Constants **************/
#define MOVING_UP     1
#define MOVING_DOWN  -1
#define STOPPED       0
#define ARRIVED       1
#define NOT          -1

// We'll wait 10 seconds on each floor
#define FLOOR_WAIT_MS 10000

/************** Floors **************/
#define floors 4

// Placeholder distances for each floor (from ultrasonic).
// Replace these with real measured distances in centimeters.
#define DIST_FLOOR_1  100  // placeholder
#define DIST_FLOOR_2  200  // placeholder
#define DIST_FLOOR_3  300  // placeholder
#define DIST_FLOOR_4  400  // placeholder

// Array to store distances for convenience:
long floorDistances[4] = {
  DIST_FLOOR_1,
  DIST_FLOOR_2,
  DIST_FLOOR_3,
  DIST_FLOOR_4
};

/************** Stepper Params **************/
#define max_speed  550
#define speed_val  400
#define max_accl   1000
#define SLOW_SPEED 300

AccelStepper stepper(1, motStp, motdir); // Using driver mode = 1
LiquidCrystal_I2C lcd(lcd_address, 16, 2);

/************** Queues & State Variables **************/
// We maintain one main queue for floors requested from both
// control room and elevator cabin. 
short floorQueue[10] = {NOT, NOT, NOT, NOT, NOT, NOT, NOT, NOT, NOT, NOT};
int   queueSize      = 10; // capacity

// These are read from the Arduino Nano (the elevator's cabin board)
bool  doorIsOpen     = false; 
bool  buzzerIsOn     = false; 

// This array will reflect the 4 floors pressed inside the cabin (Nano)
short cabinFloors[4] = {NOT, NOT, NOT, NOT};

// Current floor (1..4)
short registeredFloor = 1;

// Movement flags
short elevatorState   = STOPPED;
bool  canWork         = false;
bool  arrivedFlag     = false;

// For the 2-core logic
// We'll keep all motor stepping on Core 1
#if defined(ARDUINO_ARCH_RP2040)
void core1Task();
#endif

/************** Forward Declarations **************/
// Setup & loop
void setup();
void loop();

// Functions for reading input, controlling elevator
void readControlRoomButtons();
void addFloorToQueue(short floor);
void checkAndMoveElevator();
void moveElevator(short currentFloor, short destinationFloor);
bool validateFloor(short floor);
void removeArrivedFloorFromQueue();
void shiftQueue();

// I2C comm with Nano
void slave_data(); 
void readFromNano(); 
void sendCommandToNano(const char* cmd); // e.g., "openDoor", "closeDoor", "buzzerOn", etc.

// Utility
int  whichfloor(int pin);
void print_init();
void printFloor(short floor);
void ultrasonicRead();
long getUltrasonicDistance();
void calibration();

/*************************************************************
   SETUP - Runs on Core 0
*************************************************************/
void setup() {
  Serial.begin(9600);
  Wire.begin();  

  // Pin modes
  pinMode(Floor1_reed, INPUT);
  pinMode(Floor2_reed, INPUT);
  pinMode(Floor3_reed, INPUT);
  pinMode(Floor4_reed, INPUT);

  pinMode(Floor1_LED, OUTPUT);
  pinMode(Floor2_LED, OUTPUT);
  pinMode(Floor3_LED, OUTPUT);
  pinMode(Floor4_LED, OUTPUT);

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  pinMode(SLAVE_READY_PIN, INPUT);
  pinMode(CALIBRATION_SENSOR, INPUT);

  pinMode(LED_BUILTIN,OUTPUT);

  // LCD
  lcd.begin(16, 2);
  lcd.backlight();
  print_init();

  // Stepper
  stepper.setMaxSpeed(max_speed);
  stepper.setSpeed(speed_val);
  stepper.setAcceleration(max_accl);
  stepper.setCurrentPosition(0);

  // Calibration
  calibration();

  // After calibration, we know we are on floor 1
  // Validate that floor 1 reed is also triggered, and check ultrasonic
  Serial.println("Validating floor 1 after calibration...");
  if(!validateFloor(1)) {
    Serial.println("WARNING: Floor 1 validation mismatch!");
    lcd.clear();
    lcd.print("Floor1 mismatch!");
  }

  // Indicate we are now ready
  canWork = true; 

#if defined(ARDUINO_ARCH_RP2040)
  // Launch Core 1 to keep the stepper stepping
  multicore_launch_core1(core1Task);
  Serial.println("Core 1 launched to continuously run stepper.");
#endif
}

/*************************************************************
   LOOP - Runs on Core 0
   - Reads inputs
   - Manages queue
   - Moves elevator if needed
*************************************************************/
void loop() {
  // 1) Read data from cabin (Nano) over I2C
  slave_data();

  // 2) Read control room resistor-ladder
  readControlRoomButtons();

  // 3) Elevator logic
  checkAndMoveElevator();

  delay(50); // Small pause
}

/*************************************************************
   CORE 1 TASK - Continuously calls stepper.run()
*************************************************************/
#if defined(ARDUINO_ARCH_RP2040)
void core1Task() {
  for (;;) {
    stepper.run();
    // Optionally small sleep to avoid 100% CPU usage
    // sleep_us(50);
  }
}
#endif

/*************************************************************
   Elevator Logic
*************************************************************/

/**
 * Reads control room ADC once per loop, and if a button is pressed,
 * add that floor to the main floorQueue (avoid duplicates).
 */
void readControlRoomButtons() {
  short val = analogRead(controlroom_buttons);
  // For the Pico's 12-bit ADC, range is 0..4095.
  // Place thresholds accordingly (example):
  Serial.print("ADC from control room: ");
  Serial.println(val);

  short pressedFloor = NOT;
  if      (val <= 1000)                 pressedFloor = 4;
  else if (val > 1000 && val <= 2000)   pressedFloor = 3;
  else if (val > 2000 && val <= 3000)   pressedFloor = 2;
  else if (val > 3000 && val <= 4095)   pressedFloor = 1;

  if (pressedFloor != NOT) {
    addFloorToQueue(pressedFloor);
  }
}

/**
 * Add a requested floor to the queue unless it's already in there.
 */
void addFloorToQueue(short floor) {
  // Make sure floor is 1..4
  if (floor < 1 || floor > 4) return;
  
  // Check duplicates
  for(int i=0; i<queueSize; i++){
    if(floorQueue[i] == floor) {
      // Already in queue, do nothing
      return;
    }
  }

  // Insert at first empty slot
  for(int i=0; i<queueSize; i++){
    if(floorQueue[i] == NOT){
      floorQueue[i] = floor;
      Serial.print("Added floor ");
      Serial.print(floor);
      Serial.println(" to queue");
      lcd.clear();
      lcd.print("Added Floor ");
      lcd.print(floor);
      return;
    }
  }
  // If queue is full, ignore or handle error
  Serial.println("Queue is full! Cannot add.");
}

/**
 * Main elevator “state machine.”
 * If queue is not empty and canWork==true, move to the next floor.
 * Once arrived, wait 10 seconds, then remove floor from queue.
 */
void checkAndMoveElevator() {
  // Check the first floor in the queue
  short nextFloor = floorQueue[0];
  if(nextFloor == NOT || !canWork) {
    // No floors to go or not allowed to move
    return;
  }

  // If we are not currently “arrived,” let's move
  if(!arrivedFlag) {
    // Start move
    Serial.print("Moving from floor ");
    Serial.print(registeredFloor);
    Serial.print(" to floor ");
    Serial.println(nextFloor);

    // Make sure door is closed before moving
    if(doorIsOpen) {
      Serial.println("Requesting door closure via I2C...");
      sendCommandToNano("closeDoor");
      sendCommandToNano("buzzerOn"); // beep on door movement
      // you might want to wait for some ack that door is closed
      doorIsOpen = false; 
    }

    // Move
    moveElevator(registeredFloor, nextFloor);

    // We do not mark arrivedFlag = true yet. The function
    // itself or the validation will handle that.
  } 
  else {
    // We must have just arrived. We do a 10-second wait 
    // and then move on.
    Serial.println("Arrived at floor. Waiting 10s before next...");
    delay(FLOOR_WAIT_MS);

    // Remove this floor from the queue
    removeArrivedFloorFromQueue();

    // Reset arrivedFlag
    arrivedFlag = false;
  }
}

/**
 * The single function that moves from 'currentFloor' to 'destinationFloor'
 * using AccelStepper with acceleration. 
 * We rely on reed & ultrasonic to confirm arrival—no overshoot.
 */
void moveElevator(short currentFloor, short destinationFloor) {
  // Determine direction
  if(destinationFloor > currentFloor) {
    elevatorState = MOVING_UP;
  } else if (destinationFloor < currentFloor) {
    elevatorState = MOVING_DOWN;
  } else {
    // same floor
    elevatorState = STOPPED;
    arrivedFlag = true;
    return;
  }

  // Example approach: set a target position in steps 
  // (just placeholder logic). 
  // Real steps depends on how many steps each floor requires.
  // For demonstration, assume 500 steps per floor difference:
  int stepsPerFloor = 500;
  int distanceFloors = (destinationFloor - currentFloor); 
  int targetSteps = stepper.currentPosition() + distanceFloors * stepsPerFloor;

  stepper.setMaxSpeed(max_speed);
  stepper.setAcceleration(max_accl);
  stepper.moveTo(targetSteps);

  Serial.print("moveTo target steps = ");
  Serial.println(targetSteps);
  lcd.clear();
  lcd.print("Move: ");
  lcd.print(currentFloor);
  lcd.print(" -> ");
  lcd.print(destinationFloor);

  // Now we let Core 1 actually do the stepping via stepper.run().
  // We need to wait until we truly “arrive.” 
  // We'll poll in this function. 
  // But be mindful that we do not block other tasks too much.

  // A simple approach: 
  // Poll until we get close to the target OR we validate the floor
  bool arrived = false;
  while(!arrived) {
    // Let the stepper move a bit (Core 1 does the stepping, but 
    // we can call run() here too for safety, or yield)
    stepper.run();  
    delay(5); 

    // Check if we are at the target in steps
    if(!stepper.isRunning()) {
      // Possibly we are at the step-target
      // Validate physically that we are on the correct floor
      if(validateFloor(destinationFloor)) {
        arrived = true;
        elevatorState = STOPPED;
        arrivedFlag = true;
        registeredFloor = destinationFloor;
        Serial.println("Arrived (step-target) + validated by sensor!");
      }
    }
    else {
      // Optionally, do a mid-ride check with ultrasonic to see 
      // if we have accidentally reached the floor earlier 
      // or any emergency check. 
    }
  }

  // Once arrived, let’s open the door automatically
  Serial.println("Arrived -> Opening door via I2C...");
  sendCommandToNano("openDoor");
  sendCommandToNano("buzzerOn");
  doorIsOpen = true;
}

/**
 * Validate that 'floor' is truly reached using 
 * - Reed switch
 * - Ultrasonic distance 
 */
bool validateFloor(short floor) {
  // 1) Check which reed is associated with that floor
  bool reedOk = false;
  int reedPin = -1;
  switch(floor) {
    case 1: reedPin = Floor1_reed; break;
    case 2: reedPin = Floor2_reed; break;
    case 3: reedPin = Floor3_reed; break;
    case 4: reedPin = Floor4_reed; break;
    default: return false;
  }

  if(digitalRead(reedPin) == LOW) {
    reedOk = true;
  }

  // 2) Ultrasonic check
  long currentDist = getUltrasonicDistance();
  long expectedDist = floorDistances[floor - 1];

  long diff = abs(currentDist - expectedDist);
  bool distanceOk = (diff < 10); // 10 cm tolerance, for example

  Serial.print("Floor Validation F");
  Serial.print(floor);
  Serial.print(" => ReedOk? ");
  Serial.print(reedOk);
  Serial.print(", DistOk? ");
  Serial.println(distanceOk);

  // Optionally, print to LCD as well
  lcd.clear();
  lcd.print("Val Floor ");
  lcd.print(floor);
  lcd.setCursor(0,1);
  lcd.print("R:");
  lcd.print(reedOk);
  lcd.print(" D:");
  lcd.print(distanceOk);

  return (reedOk && distanceOk);
}

/**
 * Removes the arrived floor from the queue (the first entry),
 * and shifts everything forward.
 */
void removeArrivedFloorFromQueue() {
  // floorQueue[0] is presumably the arrived floor
  floorQueue[0] = NOT;
  shiftQueue();
  Serial.println("Queue after removing arrived floor:");
  for(int i=0; i<queueSize; i++){
    Serial.print(floorQueue[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void shiftQueue() {
  for(int i=0; i<queueSize-1; i++){
    floorQueue[i] = floorQueue[i+1];
  }
  floorQueue[queueSize-1] = NOT;
}

/*************************************************************
   I2C Master to Nano
*************************************************************/

void slave_data() {
  // If Nano is ready, read data
  if(digitalRead(SLAVE_READY_PIN) == HIGH) {
    readFromNano();
  }
}

/**
 * Read 8 bytes from the Arduino Nano.
 * You mentioned:
 *    - Door status (open/closed)
 *    - 4 floors from the cabin's ADC array
 *    - Buzzer status
 */
void readFromNano() {
  const uint8_t LENGTH = 8;
  char dataBuf[LENGTH];

  Wire.requestFrom(SLAVE_ADDRESS, LENGTH);
  if(Wire.available() >= LENGTH) {
    for(int i=0; i<LENGTH; i++){
      dataBuf[i] = Wire.read();
    }
    // Example layout:
    // dataBuf[0] => door status: 'O' or 'C'
    // dataBuf[1] => buzzer: '1' or '0'
    // dataBuf[2..5] => floors pressed inside cabin (1..4 or -1 if none)
    // dataBuf[6..7] => reserved or something else

    // door
    if(dataBuf[0] == 'O') {
      doorIsOpen = true;
    } else {
      doorIsOpen = false;
    }

    // buzzer
    buzzerIsOn = (dataBuf[1] == '1');

    // cabin floors (4 bytes)
    for(int i=0; i<4; i++){
      char c = dataBuf[2 + i];
      if(c >= '1' && c <= '4') {
        cabinFloors[i] = c - '0'; 
      } else {
        cabinFloors[i] = NOT; 
      }
    }

    Serial.println("Data from Nano:");
    Serial.print("Door: ");  Serial.println(doorIsOpen ? "Open" : "Closed");
    Serial.print("Buzzer: ");Serial.println(buzzerIsOn ? "On" : "Off");
    Serial.print("CabinFloors: ");
    for(int i=0; i<4; i++){
      Serial.print(cabinFloors[i]);
      Serial.print(" ");
    }
    Serial.println();

    // Now merge cabinFloors into main floorQueue
    for(int i=0; i<4; i++) {
      if(cabinFloors[i] != NOT) {
        addFloorToQueue(cabinFloors[i]);
      }
    }
  }
}

/**
 * Send a command string to the Nano.
 * For example, "openDoor", "closeDoor", "buzzerOn", "buzzerOff", etc.
 * You’ll define how the Nano interprets these.
 */
void sendCommandToNano(const char* cmd) {
  Wire.beginTransmission(SLAVE_ADDRESS);
  Wire.write((const uint8_t*)cmd, strlen(cmd));
  Wire.endTransmission();

  Serial.print("Sent command to Nano: ");
  Serial.println(cmd);
}

/*************************************************************
   Utility & Setup
*************************************************************/

void print_init() {
  lcd.setCursor(0,0);
  lcd.print("   Elevator V2  ");
  lcd.setCursor(0,1);
  lcd.print("Pico->Nano I2C");
  delay(3000);
  lcd.clear();
}

/**
 * Ultrasonic reading
 */
long getUltrasonicDistance() {
  digitalWrite(trig, LOW);  
  delayMicroseconds(2); 
  digitalWrite(trig, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trig, LOW);

  long dur = pulseIn(echo, HIGH);
  long dist = (dur / 2) / 29.1; // in cm
  return dist;
}

/**
 * For debugging from older code
 */
void ultrasonicRead() {
  long dist = getUltrasonicDistance();
  Serial.print("Ultrasonic dist: ");
  Serial.print(dist);
  Serial.println(" cm");
}

/**
 * Calibration: Move down until calibration switch triggers.
 * Then set floor=1. 
 */
void calibration() {
  Serial.println("Starting calibration...");
  flash_LED();
  lcd.clear();
  lcd.print("CALIBRATION...");

  // Turn on all floor LEDs
  digitalWrite(Floor1_LED, HIGH);
  digitalWrite(Floor2_LED, HIGH);
  digitalWrite(Floor3_LED, HIGH);
  digitalWrite(Floor4_LED, HIGH);

  while(digitalRead(CALIBRATION_SENSOR) == LOW) {
    // Move downward in small increments
    stepper.move(-20);
    delay(50);
    stepper.run(); 
    // Meanwhile, core1 is also calling run() 
    // but a bit more “help” here doesn’t hurt
  }

  // Turn off all floor LEDs
  digitalWrite(Floor1_LED, LOW);
  digitalWrite(Floor2_LED, LOW);
  digitalWrite(Floor3_LED, LOW);
  digitalWrite(Floor4_LED, LOW);

  // We are physically at floor 1 now
  registeredFloor = 1;
  // Reset stepper position to 0
  stepper.setCurrentPosition(0);

  lcd.clear();
  lcd.print("Calibrated @F1");
  Serial.println("Calibration done. Elevator at floor 1.");
  delay(2000);
}

/**
 * If needed to interpret which floor from pin 
 * (used in older code)
 */
int whichfloor(int pin) {
  if(pin == Floor1_reed) return 1;
  if(pin == Floor2_reed) return 2;
  if(pin == Floor3_reed) return 3;
  if(pin == Floor4_reed) return 4;
  return 0;
}

/**
 * Prints floor info to LCD
 */
void printFloor(short floor) {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Floor: ");
  lcd.print(floor);
}

void flash_LED() {
  // Turn LED on
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500); // Wait 0.5 seconds
  
  // Turn LED off
  digitalWrite(LED_BUILTIN, LOW);
  delay(500); // Wait 0.5 seconds
}
