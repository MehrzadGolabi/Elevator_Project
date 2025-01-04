#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <LiquidCrystal_I2C.h>

// If you want PID:
#include <PID_v1.h>

// For dual-core on RP2040
#if defined(ARDUINO_ARCH_RP2040)
#include "pico/multicore.h"
#endif

/******************************************************
   Pin Definitions
 ******************************************************/
// Reeds
const int Floor1_reed = 6;   // GP6
const int Floor2_reed = 7;   // GP7
const int Floor3_reed = 8;   // GP8
const int Floor4_reed = 9;   // GP9

// Floor LEDs
const int Floor1_LED  = 10;  // GP10
const int Floor2_LED  = 11;  // GP11
const int Floor3_LED  = 12;  // GP12
const int Floor4_LED  = 13;  // GP13

// Stepper
#define motStp 14            // GP14
#define motdir 15            // GP15

// Ultrasonic
#define trig 16              // GP16
#define echo 17              // GP17

// Calibration switch
#define CALIBRATION_SENSOR 18 // GP18

// Analog input for 4-button ladder
#define controlroom_buttons A0 // GP26

// Slave ready pin (Nano)
#define SLAVE_READY_PIN A1     // GP27

// Onboard LED
#define LED_PICO 25            // or LED_BUILTIN

// I2C address
#define lcd_address   0x3F
#define SLAVE_ADDRESS 0x08

/******************************************************
   Elevator Status & Constants
 ******************************************************/
#define MOVING_UP     1
#define MOVING_DOWN  -1
#define STOPPED       0
#define ARRIVED       1
#define NOT          -1

#define FLOOR_WAIT_MS 10000

#define floors 4

// We keep the default I2C pins free (GP4, GP5), so we do:
#define USE_DEFAULT_I2C_PINS false
// But note: With the Mbed RP2040 core, you can't easily reassign Wire pins.
// If you use Earle Philhower's RP2040 core, you can do Wire.setSDA(...) etc.

/******************************************************
   Floor Distances (Ultrasonic placeholders)
 ******************************************************/
#define DIST_FLOOR_1  100
#define DIST_FLOOR_2  200
#define DIST_FLOOR_3  300
#define DIST_FLOOR_4  400

long floorDistances[4] = {
  DIST_FLOOR_1,
  DIST_FLOOR_2,
  DIST_FLOOR_3,
  DIST_FLOOR_4
};

/******************************************************
   Stepper & LCD
 ******************************************************/
#define max_speed  550
#define speed_val  400
#define max_accl   1000

AccelStepper stepper(1, motStp, motdir); 
LiquidCrystal_I2C lcd(lcd_address, 16, 2);

/******************************************************
   Queues & Variables
 ******************************************************/
short floorQueue[10] = {NOT, NOT, NOT, NOT, NOT, NOT, NOT, NOT, NOT, NOT};
int   queueSize      = 10;

bool  doorIsOpen     = false; 
bool  buzzerIsOn     = false; 
short cabinFloors[4] = {NOT, NOT, NOT, NOT};

short registeredFloor = 1;
short elevatorState   = STOPPED;
bool  canWork         = false;
bool  arrivedFlag     = false;

// We'll keep all motor stepping on Core 1
#if defined(ARDUINO_ARCH_RP2040)
void core1Task();
#endif

/******************************************************
   PID Example (Optional)
 ******************************************************/
// Suppose we want to do a PID on the ultrasonic reading
// to keep the elevator at a setpoint distance. This is
// just a demonstration. If you don't need PID, ignore it.
double setpoint = 0;
double inputUltrasonic = 0;
double outputPID = 0;

// Kp, Ki, Kd - placeholders
double Kp=2, Ki=0.5, Kd=1;

// Construct the PID object
PID myPID(&inputUltrasonic, &outputPID, &setpoint, Kp, Ki, Kd, DIRECT);

/******************************************************
   Forward Declarations
 ******************************************************/
void setup();
void loop();
void handleSerialInput();
void scenarioDemo();

// Elevator logic
void readControlRoomButtons();
void addFloorToQueue(short floor);
void checkAndMoveElevator();
void moveElevator(short currentFloor, short destinationFloor);
bool validateFloor(short floor);
void removeArrivedFloorFromQueue();
void shiftQueue();

// I2C master to Nano
void slave_data(); 
void readFromNano(); 
void sendCommandToNano(const char* cmd);

// Utility
int  whichfloor(int pin);
void print_init();
void printFloor(short floor);
long getUltrasonicDistance();
void calibration();

// LED patterns
void ledAlwaysOn();
void blinkInternalLED(int times, int intervalMs);

// The code you had for older references:
void ultrasonicRead(); 

/******************************************************
   SETUP (Core 0)
 ******************************************************/
void setup() {
  Serial.begin(9600);

  // If using Earle Philhower core and want to change pins:
  // if(!USE_DEFAULT_I2C_PINS){
  //   Wire.setSDA(??);
  //   Wire.setSCL(??);
  // }
  Wire.begin(); // Will use default GP4, GP5 for Mbed or Earle

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
  pinMode(CALIBRATION_SENSOR, INPUT);

  pinMode(LED_PICO, OUTPUT);  
  ledAlwaysOn(); // remain solid on if idle

  pinMode(SLAVE_READY_PIN, INPUT);

  // LCD
  lcd.begin(16, 2);
  lcd.backlight();
  print_init();

  // Stepper
  stepper.setMaxSpeed(max_speed);
  stepper.setSpeed(speed_val);
  stepper.setAcceleration(max_accl);
  stepper.setCurrentPosition(0);

  // PID example initialization
  myPID.SetOutputLimits(-200, 200); // example range
  myPID.SetMode(AUTOMATIC);

  calibration();

  // Indicate we are now ready
  canWork = true; 

#if defined(ARDUINO_ARCH_RP2040)
  // Launch Core 1 to keep the stepper stepping
  multicore_launch_core1(core1Task);
  Serial.println("Core 1 launched.");
#endif
}

/******************************************************
   LOOP (Core 0)
 ******************************************************/
void loop() {
  handleSerialInput();      // If 'a' is received, do scenario
  slave_data();             // I2C from Nano
  readControlRoomButtons(); // Control room ladder
  checkAndMoveElevator();   // Normal elevator logic

  // Example usage of PID (optional):
  // inputUltrasonic = getUltrasonicDistance();
  // setpoint = 200; // example: we want to hold 200 cm
  // myPID.Compute();
  // Now 'outputPID' is the correction. 
  // You could do stepper.setSpeed(speed_val + outputPID) 
  // or something to fine-tune the approach.

  delay(50);
}

/******************************************************
   Handle Serial Input
   - If we receive 'a', run scenarioDemo()
 ******************************************************/
void handleSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'a') {
      scenarioDemo();
      // After scenario finishes, code will continue normal flow
    }
  }
}

/******************************************************
   scenarioDemo()
   - Ignores sensors & validations
   - Moves motor up 500, then down 500
   - Flashes all floor LEDs
   - Prints sensor values
 ******************************************************/
void scenarioDemo() {
  Serial.println("=== Demo scenario triggered (command 'a') ===");

  // 1) Blink internal LED pattern to indicate start
  blinkInternalLED(3, 200);  // e.g. 3 quick blinks

  // 2) Move up 500 steps, ignoring sensors
  Serial.println("Moving stepper +500 steps (ignore validation)...");
  stepper.move(500);
  // Let’s block until done
  while(stepper.distanceToGo() != 0){
    stepper.run();
  }

  // 3) Move down 500 steps
  Serial.println("Moving stepper -500 steps...");
  stepper.move(-500);
  while(stepper.distanceToGo() != 0){
    stepper.run();
  }

  // 4) Flash all floor LEDs
  Serial.println("Flashing all floor LEDs...");
  for(int i=0; i<5; i++){ // 5 flashes
    digitalWrite(Floor1_LED, HIGH);
    digitalWrite(Floor2_LED, HIGH);
    digitalWrite(Floor3_LED, HIGH);
    digitalWrite(Floor4_LED, HIGH);
    delay(200);

    digitalWrite(Floor1_LED, LOW);
    digitalWrite(Floor2_LED, LOW);
    digitalWrite(Floor3_LED, LOW);
    digitalWrite(Floor4_LED, LOW);
    delay(200);
  }

  // 5) Print sensor values
  Serial.println("Sensor Readings:");
  Serial.print("Floor1_reed: "); Serial.println(digitalRead(Floor1_reed));
  Serial.print("Floor2_reed: "); Serial.println(digitalRead(Floor2_reed));
  Serial.print("Floor3_reed: "); Serial.println(digitalRead(Floor3_reed));
  Serial.print("Floor4_reed: "); Serial.println(digitalRead(Floor4_reed));

  long dist = getUltrasonicDistance();
  Serial.print("Ultrasonic: "); Serial.println(dist);

  int calib = digitalRead(CALIBRATION_SENSOR);
  Serial.print("Calibration Switch: "); Serial.println(calib);

  // Another blink pattern indicating scenario done
  blinkInternalLED(2, 300);
  // Then LED remains on
  ledAlwaysOn();

  Serial.println("=== Demo scenario finished ===");
}

/******************************************************
   CORE 1 TASK - stepper.run() loop
 ******************************************************/
#if defined(ARDUINO_ARCH_RP2040)
void core1Task() {
  for (;;) {
    stepper.run();
    // sleep_us(50); // optional
  }
}
#endif

/******************************************************
   Elevator Logic (unchanged except for pin references)
 ******************************************************/

void readControlRoomButtons() {
  short val = analogRead(controlroom_buttons);
  // thresholds
  short pressedFloor = NOT;
  if      (val <= 1000)                 pressedFloor = 4;
  else if (val > 1000 && val <= 2000)   pressedFloor = 3;
  else if (val > 2000 && val <= 3000)   pressedFloor = 2;
  else if (val > 3000 && val <= 4095)   pressedFloor = 1;

  if (pressedFloor != NOT) {
    addFloorToQueue(pressedFloor);
  }
}

void addFloorToQueue(short floor) {
  if (floor < 1 || floor > 4) return;
  for(int i=0; i<queueSize; i++){
    if(floorQueue[i] == floor) return;
  }
  for(int i=0; i<queueSize; i++){
    if(floorQueue[i] == NOT){
      floorQueue[i] = floor;
      Serial.print("Added floor ");
      Serial.println(floor);
      lcd.clear();
      lcd.print("Added Floor ");
      lcd.print(floor);
      return;
    }
  }
  Serial.println("Queue is full! Cannot add.");
}

void checkAndMoveElevator() {
  short nextFloor = floorQueue[0];
  if(nextFloor == NOT || !canWork) return;

  if(!arrivedFlag) {
    Serial.print("Moving from floor ");
    Serial.print(registeredFloor);
    Serial.print(" to floor ");
    Serial.println(nextFloor);
    // if door open => close it
    if(doorIsOpen){
      sendCommandToNano("closeDoor");
      sendCommandToNano("buzzerOn");
      doorIsOpen = false; 
    }
    moveElevator(registeredFloor, nextFloor);
  } else {
    Serial.println("Arrived at floor. Waiting 10s...");
    delay(FLOOR_WAIT_MS);
    removeArrivedFloorFromQueue();
    arrivedFlag = false;
  }
}

void moveElevator(short currentFloor, short destinationFloor) {
  if(destinationFloor > currentFloor) {
    elevatorState = MOVING_UP;
  } else if (destinationFloor < currentFloor) {
    elevatorState = MOVING_DOWN;
  } else {
    elevatorState = STOPPED;
    arrivedFlag = true;
    return;
  }

  // 500 steps/floor example
  int stepsPerFloor = 500;
  int distanceFloors = (destinationFloor - currentFloor); 
  int targetSteps = stepper.currentPosition() + distanceFloors * stepsPerFloor;

  stepper.setMaxSpeed(max_speed);
  stepper.setAcceleration(max_accl);
  stepper.moveTo(targetSteps);

  Serial.print("moveTo = ");
  Serial.println(targetSteps);
  lcd.clear();
  lcd.print("Move: ");
  lcd.print(currentFloor);
  lcd.print("->");
  lcd.print(destinationFloor);

  bool arrived = false;
  while(!arrived) {
    stepper.run();  
    delay(5);
    if(!stepper.isRunning()) {
      if(validateFloor(destinationFloor)) {
        arrived = true;
        elevatorState = STOPPED;
        arrivedFlag = true;
        registeredFloor = destinationFloor;
        Serial.println("Arrived + validated!");
      }
    }
  }
  // open door
  sendCommandToNano("openDoor");
  sendCommandToNano("buzzerOn");
  doorIsOpen = true;
}

bool validateFloor(short floor) {
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

  long currentDist = getUltrasonicDistance();
  long expectedDist = floorDistances[floor -1];
  long diff = abs(currentDist - expectedDist);
  bool distanceOk = (diff < 10);

  Serial.print("Val Floor ");
  Serial.print(floor);
  Serial.print(" => R=");
  Serial.print(reedOk);
  Serial.print(" D=");
  Serial.println(distanceOk);

  return (reedOk && distanceOk);
}

void removeArrivedFloorFromQueue() {
  floorQueue[0] = NOT;
  shiftQueue();
  Serial.print("Queue after arrival: ");
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

/******************************************************
   I2C Master to Nano
 ******************************************************/
void slave_data() {
  if(digitalRead(SLAVE_READY_PIN) == HIGH) {
    readFromNano();
  }
}

void readFromNano() {
  const uint8_t LENGTH = 8;
  char dataBuf[LENGTH];

  Wire.requestFrom(SLAVE_ADDRESS, LENGTH);
  if(Wire.available() >= LENGTH) {
    for(int i=0; i<LENGTH; i++){
      dataBuf[i] = Wire.read();
    }

    doorIsOpen   = (dataBuf[0] == 'O');
    buzzerIsOn   = (dataBuf[1] == '1');

    for(int i=0; i<4; i++){
      char c = dataBuf[2 + i];
      if(c >= '1' && c <= '4') {
        cabinFloors[i] = c - '0'; 
        addFloorToQueue(cabinFloors[i]);
      } else {
        cabinFloors[i] = NOT;
      }
    }

    Serial.println("Data from Nano:");
    Serial.print("Door: "); Serial.println(doorIsOpen ? "Open" : "Closed");
    Serial.print("Buzzer: "); Serial.println(buzzerIsOn ? "On" : "Off");
  }
}

void sendCommandToNano(const char* cmd) {
  Wire.beginTransmission(SLAVE_ADDRESS);
  Wire.write((const uint8_t*)cmd, strlen(cmd));
  Wire.endTransmission();

  Serial.print("Sent to Nano: ");
  Serial.println(cmd);
}

/******************************************************
   Utility & Setup
 ******************************************************/
void print_init() {
  lcd.setCursor(0,0);
  lcd.print("   Elevator V2  ");
  lcd.setCursor(0,1);
  lcd.print(" Pico->Nano I2C ");
  delay(2000);
  lcd.clear();
}

long getUltrasonicDistance() {
  digitalWrite(trig, LOW);  
  delayMicroseconds(2); 
  digitalWrite(trig, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trig, LOW);

  long dur = pulseIn(echo, HIGH);
  long dist = (dur / 2) / 29.1;
  return dist;
}

void ultrasonicRead() {
  long dist = getUltrasonicDistance();
  Serial.print("Ultrasonic: ");
  Serial.print(dist);
  Serial.println(" cm");
}

void calibration() {
  Serial.println("Starting calibration...");
  blinkInternalLED(2, 300);
  lcd.clear();
  lcd.print("CALIBRATION...");

  digitalWrite(Floor1_LED, HIGH);
  digitalWrite(Floor2_LED, HIGH);
  digitalWrite(Floor3_LED, HIGH);
  digitalWrite(Floor4_LED, HIGH);

  while(digitalRead(CALIBRATION_SENSOR) == LOW) {
    stepper.move(-20);
    delay(50);
    stepper.run();
  }

  digitalWrite(Floor1_LED, LOW);
  digitalWrite(Floor2_LED, LOW);
  digitalWrite(Floor3_LED, LOW);
  digitalWrite(Floor4_LED, LOW);

  registeredFloor = 1;
  stepper.setCurrentPosition(0);

  lcd.clear();
  lcd.print("Calibrated @F1");
  Serial.println("Calibration done, floor=1");
  delay(2000);
  ledAlwaysOn();
}

/******************************************************
   Internal LED Patterns
 ******************************************************/
void ledAlwaysOn(){
  digitalWrite(LED_PICO, HIGH);
}

void blinkInternalLED(int times, int intervalMs){
  for(int i=0; i<times; i++){
    digitalWrite(LED_PICO, LOW);
    delay(intervalMs);
    digitalWrite(LED_PICO, HIGH);
    delay(intervalMs);
  }
  // End with LED on to indicate "idle" or "normal"
  digitalWrite(LED_PICO, HIGH);
}

/******************************************************
   If needed to interpret floor from pin (older usage)
 ******************************************************/
int whichfloor(int pin) {
  if(pin == Floor1_reed) return 1;
  if(pin == Floor2_reed) return 2;
  if(pin == Floor3_reed) return 3;
  if(pin == Floor4_reed) return 4;
  return 0;
}
