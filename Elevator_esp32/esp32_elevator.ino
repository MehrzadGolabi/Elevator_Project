/********************************************************
 * Elevator Project - ESP32 DevKit V1 (30-pin) Version
 * 
 * Features:
 *  - Two tasks (multithreading):
 *     1) Main code on Core 0
 *     2) Stepper runner on Core 1
 *  - AccelStepper for motor control
 *  - Optional PID control
 *  - Built-in Wi-Fi Access Point + simple webserver 
 *    for debug/status page
 ********************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h> // optional, if you want PID

/********************************************************
 * Pin Definitions for ESP32
 * Adjust these to match your wiring. 
 * We avoid GPIO1, GPIO3 (default Serial), 
 * and GPIO21, GPIO22 (default I2C) to keep them free.
 ********************************************************/

// Reed switches
const int Floor1_reed = 4;  
const int Floor2_reed = 5;  
const int Floor3_reed = 16; 
const int Floor4_reed = 17; 

// Floor LEDs
const int Floor1_LED  = 18;
const int Floor2_LED  = 19;
const int Floor3_LED  = 23;
const int Floor4_LED  = 25; 
// (Note: GPIO25 is valid on many ESP32 boards, but if yours conflicts, pick something else)

// Stepper pins (AccelStepper in DRIVER mode => 2 pins: STEP, DIR)
#define motStp 26
#define motdir 27

// Ultrasonic sensor
#define trig 12
#define echo 13

// Calibration switch
#define CALIBRATION_SENSOR 14

// ADC input for 4-button ladder
#define controlroom_buttons 34

// Slave ready pin (if using an external I2C device, e.g. Nano)
#define SLAVE_READY_PIN 35

// Built-In LED (often on GPIO2 for many ESP32 dev boards)
#define LED_ESP 2

// I2C address (if using an LCD or other I2C device)
#define lcd_address   0x3F
#define SLAVE_ADDRESS 0x08

/********************************************************
 * Elevator status & constants
 ********************************************************/
#define MOVING_UP     1
#define MOVING_DOWN  -1
#define STOPPED       0
#define ARRIVED       1
#define NOT          -1

// Time to wait on each floor (ms)
#define FLOOR_WAIT_MS 10000
#define floors 4

/********************************************************
 * Distance placeholders per floor (Ultrasonic)
 ********************************************************/
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

/********************************************************
 * Stepper & LCD
 ********************************************************/
#define max_speed  550
#define speed_val  400
#define max_accl   1000

AccelStepper stepper(AccelStepper::DRIVER, motStp, motdir);
LiquidCrystal_I2C lcd(lcd_address, 16, 2);

/********************************************************
 * Queues & Variables
 ********************************************************/
short floorQueue[10] = {NOT, NOT, NOT, NOT, NOT, NOT, NOT, NOT, NOT, NOT};
int   queueSize      = 10;

bool  doorIsOpen     = false; 
bool  buzzerIsOn     = false; 
short cabinFloors[4] = {NOT, NOT, NOT, NOT};

short registeredFloor = 1;
short elevatorState   = STOPPED;
bool  canWork         = false;
bool  arrivedFlag     = false;

/********************************************************
 * PID Example (Optional)
 ********************************************************/
double setpoint = 0;
double inputUltrasonic = 0;
double outputPID = 0;

// Kp, Ki, Kd placeholders
double Kp=2, Ki=0.5, Kd=1;
PID myPID(&inputUltrasonic, &outputPID, &setpoint, Kp, Ki, Kd, DIRECT);

/********************************************************
 * Wi-Fi & Webserver
 ********************************************************/
const char* AP_SSID     = "ElevatorAP";  // AP SSID
const char* AP_PASSWORD = "12345678";    // AP password
WiFiServer server(80);                  // HTTP server on port 80

// For multi-tasking:
TaskHandle_t stepperTaskHandle = NULL;  // handle for stepper task

/********************************************************
 * Forward Declarations
 ********************************************************/
void setup();
void loop();
void setupWiFiAP();
void handleWebServer();

// Elevator logic
void readControlRoomButtons();
void addFloorToQueue(short floor);
void checkAndMoveElevator();
void moveElevator(short currentFloor, short destinationFloor);
bool validateFloor(short floor);
void removeArrivedFloorFromQueue();
void shiftQueue();

// I2C to Nano (if needed)
void slave_data(); 
void readFromNano(); 
void sendCommandToNano(const char* cmd);

// Utility
int  whichfloor(int pin);
void print_init();
void printFloor(short floor);
long getUltrasonicDistance();
void calibration();
void ledAlwaysOn();
void blinkInternalLED(int times, int intervalMs);
void scenarioDemo();
void handleSerialInput();

// RTOS tasks
void coreTaskStepper(void* pvParameters);

/********************************************************
 * SETUP (runs on Core 0 by default)
 ********************************************************/
void setup() {
  Serial.begin(115200);
  delay(100);

  // Setup built-in LED
  pinMode(LED_ESP, OUTPUT);
  ledAlwaysOn();

  // I2C if needed (for LCD or external board)
  Wire.begin(); // On ESP32, defaults are SDA=21, SCL=22. 
               // If your board's I2C is pinned differently, do: Wire.begin(21,22);

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
  pinMode(SLAVE_READY_PIN, INPUT);

  // LCD init (assuming a library that works on ESP32)
  lcd.begin(16, 2);
  lcd.backlight();
  print_init();

  // Stepper init
  stepper.setMaxSpeed(max_speed);
  stepper.setSpeed(speed_val);
  stepper.setAcceleration(max_accl);
  stepper.setCurrentPosition(0);

  // PID init
  myPID.SetOutputLimits(-200, 200);
  myPID.SetMode(AUTOMATIC);

  // Calibration
  calibration();
  canWork = true;

  // Create a FreeRTOS task for continuous stepper.run()
  xTaskCreatePinnedToCore(
    coreTaskStepper,   // Function to run
    "StepperTask",     // Name of task
    4096,              // Stack size
    NULL,              // Parameter
    1,                 // Priority (1 = low, 2+ = higher)
    &stepperTaskHandle,// Task handle
    1                  // Pin to core 1
  );
  Serial.println("Stepper task created on core 1.");

  // Setup WiFi Access Point and start webserver
  setupWiFiAP();
  server.begin();
  Serial.println("Webserver started.");
}

/********************************************************
 * LOOP (runs on Core 0)
 ********************************************************/
void loop() {
  // 1) Check for incoming 'a' command on Serial
  handleSerialInput();

  // 2) If using an external Nano for door/buzzer, read it
  slave_data();

  // 3) Read control room buttons
  readControlRoomButtons();

  // 4) Elevator logic
  checkAndMoveElevator();

  // 5) (Optional) Update PID if desired
  // inputUltrasonic = getUltrasonicDistance();
  // setpoint = 200; 
  // myPID.Compute();
  // stepper.setSpeed(speed_val + outputPID);  

  // 6) Handle webserver requests
  handleWebServer();

  delay(50);
}

/********************************************************
 * Task for Stepper (runs on Core 1)
 ********************************************************/
void coreTaskStepper(void* pvParameters) {
  for(;;) {
    stepper.run();
    // small yield
    vTaskDelay(1); 
  }
}

/********************************************************
 * Wi-Fi AP & Webserver
 ********************************************************/
void setupWiFiAP() {
  Serial.println("Setting up WiFi AP...");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP); // Typically 192.168.4.1
}

void handleWebServer() {
  // Accept new client
  WiFiClient client = server.available();
  if (!client) return; // no client, do nothing

  // Wait for request
  while(!client.available()) {
    delay(1);
    if(!client.connected()) return;
  }

  // Read request line
  String reqLine = client.readStringUntil('\r');
  client.readStringUntil('\n'); // discard rest of line

  // Build debug page
  String html = "<!DOCTYPE html><html>";
  html += "<head><meta charset='UTF-8'>";
  // Auto-refresh every 5s
  html += "<meta http-equiv='refresh' content='5'>";
  html += "<title>Elevator Status</title></head>";
  html += "<body style='font-family:Arial;'>";

  html += "<h2>Elevator Debug Page</h2>";
  html += "<p><b>Door Status:</b> " + String(doorIsOpen ? "Open" : "Closed") + "</p>";
  html += "<p><b>Buzzer Status:</b> " + String(buzzerIsOn ? "On" : "Off") + "</p>";
  html += "<p><b>Current Floor:</b> " + String(registeredFloor) + "</p>";
  html += "<p><b>Elevator State:</b> ";
  if(elevatorState == MOVING_UP)   html += "Moving Up";
  else if(elevatorState == MOVING_DOWN) html += "Moving Down";
  else if(elevatorState == STOPPED)     html += "Stopped";
  else                                  html += "Unknown";
  html += "</p>";

  // Floor queue
  html += "<p><b>Floor Queue:</b> [ ";
  for(int i=0; i<queueSize; i++){
    if(floorQueue[i] == NOT) html += " - ";
    else html += String(floorQueue[i]) + " ";
  }
  html += "]</p>";

  // Reed pins
  html += "<p><b>Reed Switches (raw):</b><br/>";
  html += "F1: " + String(digitalRead(Floor1_reed)) + " | ";
  html += "F2: " + String(digitalRead(Floor2_reed)) + " | ";
  html += "F3: " + String(digitalRead(Floor3_reed)) + " | ";
  html += "F4: " + String(digitalRead(Floor4_reed)) + "</p>";

  // Ultrasonic reading
  long dist = getUltrasonicDistance();
  html += "<p><b>Ultrasonic:</b> " + String(dist) + " cm</p>";

  // Calibration switch
  int calib = digitalRead(CALIBRATION_SENSOR);
  html += "<p><b>Calibration Switch:</b> " + String(calib) + "</p>";

  html += "<hr/><p>Auto-refreshes every 5 seconds.</p>";
  html += "</body></html>";

  // Send response
  client.println("HTTP/1.1 200 OK");
  client.println("Content-type:text/html");
  client.println("Connection: close");
  client.println(); 
  client.print(html);

  // Wait a bit, then flush & close
  delay(10);
  client.stop();
}

/********************************************************
 * Handling 'a' Command from Serial
 ********************************************************/
void handleSerialInput() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'a') {
      scenarioDemo();
    }
  }
}

/********************************************************
 * scenarioDemo()
 * - Moves motor up 500, down 500 ignoring sensors
 * - Flashes all floor LEDs
 * - Prints sensor data
 ********************************************************/
void scenarioDemo() {
  Serial.println("=== Demo scenario triggered (command 'a') ===");
  blinkInternalLED(3, 200);

  // Move stepper +500
  Serial.println("Moving stepper +500 (no validation)...");
  stepper.move(500);
  while(stepper.distanceToGo() != 0) {
    stepper.run();
    delay(1);
  }

  // Move stepper -500
  Serial.println("Moving stepper -500...");
  stepper.move(-500);
  while(stepper.distanceToGo() != 0) {
    stepper.run();
    delay(1);
  }

  // Flash floor LEDs 5 times
  Serial.println("Flashing all floor LEDs...");
  for(int i=0; i<5; i++){
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

  // Print sensor values
  Serial.println("Sensor Readings:");
  Serial.printf("Floor1_reed=%d\n", digitalRead(Floor1_reed));
  Serial.printf("Floor2_reed=%d\n", digitalRead(Floor2_reed));
  Serial.printf("Floor3_reed=%d\n", digitalRead(Floor3_reed));
  Serial.printf("Floor4_reed=%d\n", digitalRead(Floor4_reed));

  long dist = getUltrasonicDistance();
  Serial.printf("Ultrasonic=%ld\n", dist);

  int calib = digitalRead(CALIBRATION_SENSOR);
  Serial.printf("Calibration Switch=%d\n", calib);

  blinkInternalLED(2, 300);
  ledAlwaysOn();
  Serial.println("=== Demo scenario finished ===");
}

/********************************************************
 * Elevator Logic
 ********************************************************/
void readControlRoomButtons() {
  int val = analogRead(controlroom_buttons);
  // For ESP32, ADC range is typically 0..4095
  short pressedFloor = NOT;
  if      (val >= 0   && val < 1000)  pressedFloor = 4;
  else if (val >=1000 && val < 2000)  pressedFloor = 3;
  else if (val >=2000 && val < 3000)  pressedFloor = 2;
  else if (val >=3000 && val <=4095) pressedFloor = 1;

  if (pressedFloor != NOT) addFloorToQueue(pressedFloor);
}

void addFloorToQueue(short floor) {
  if(floor <1 || floor>4) return;
  // avoid duplicates
  for(int i=0; i<queueSize; i++){
    if(floorQueue[i] == floor) return;
  }
  // find first NOT slot
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
  Serial.println("Queue is full! Cannot add floor.");
}

void checkAndMoveElevator() {
  short nextFloor = floorQueue[0];
  if(nextFloor == NOT || !canWork) return;

  if(!arrivedFlag){
    Serial.printf("Moving from floor %d to %d\n", registeredFloor, nextFloor);
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
  if(destinationFloor>currentFloor) elevatorState=MOVING_UP;
  else if(destinationFloor<currentFloor) elevatorState=MOVING_DOWN;
  else {
    elevatorState=STOPPED;
    arrivedFlag=true;
    return;
  }
  int stepsPerFloor=500;
  int distanceFloors = (destinationFloor - currentFloor);
  int targetSteps = stepper.currentPosition() + (distanceFloors*stepsPerFloor);

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

  bool arrived=false;
  while(!arrived){
    stepper.run();
    delay(5);
    if(!stepper.isRunning()){
      if(validateFloor(destinationFloor)){
        arrived=true;
        elevatorState=STOPPED;
        arrivedFlag=true;
        registeredFloor=destinationFloor;
        Serial.println("Arrived + validated by sensor!");
      }
    }
  }
  // open door
  sendCommandToNano("openDoor");
  sendCommandToNano("buzzerOn");
  doorIsOpen=true;
}

bool validateFloor(short floor) {
  // pick correct reed
  int reedPin=-1;
  switch(floor){
    case 1: reedPin=Floor1_reed; break;
    case 2: reedPin=Floor2_reed; break;
    case 3: reedPin=Floor3_reed; break;
    case 4: reedPin=Floor4_reed; break;
  }
  bool reedOk = (digitalRead(reedPin)==LOW);
  long currentDist=getUltrasonicDistance();
  long expectedDist=floorDistances[floor-1];
  bool distanceOk=(abs(currentDist-expectedDist)<10);

  Serial.printf("Val Floor %d => R=%d D=%d\n", floor, reedOk, distanceOk);
  return(reedOk && distanceOk);
}

void removeArrivedFloorFromQueue() {
  floorQueue[0]=NOT;
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

/********************************************************
 * I2C Master to Nano (if used)
 ********************************************************/
void slave_data() {
  if(digitalRead(SLAVE_READY_PIN) == HIGH){
    readFromNano();
  }
}

void readFromNano() {
  const uint8_t LENGTH=8;
  char dataBuf[LENGTH];
  Wire.requestFrom(SLAVE_ADDRESS, LENGTH);
  if(Wire.available()>=LENGTH){
    for(int i=0; i<LENGTH;i++){
      dataBuf[i]=Wire.read();
    }
    doorIsOpen  =(dataBuf[0]=='O');
    buzzerIsOn  =(dataBuf[1]=='1');
    for(int i=0; i<4; i++){
      char c=dataBuf[2+i];
      if(c>='1' && c<='4'){
        cabinFloors[i]=c-'0';
        addFloorToQueue(cabinFloors[i]);
      } else {
        cabinFloors[i]=NOT;
      }
    }
    Serial.println("Data from Nano:");
    Serial.print("Door: ");Serial.println(doorIsOpen?"Open":"Closed");
    Serial.print("Buzzer: ");Serial.println(buzzerIsOn?"On":"Off");
  }
}

void sendCommandToNano(const char* cmd) {
  Wire.beginTransmission(SLAVE_ADDRESS);
  Wire.write((const uint8_t*)cmd, strlen(cmd));
  Wire.endTransmission();
  Serial.printf("Sent to Nano: %s\n", cmd);
}

/********************************************************
 * Utility & Setup
 ********************************************************/
void print_init() {
  lcd.setCursor(0,0);
  lcd.print("Elevator (ESP32)");
  lcd.setCursor(0,1);
  lcd.print("Init...");
  delay(2000);
  lcd.clear();
}

long getUltrasonicDistance() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long dur = pulseIn(echo, HIGH, 20000UL); // 20ms timeout
  long dist=(dur/2)/29.1; 
  return dist;
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

  while(digitalRead(CALIBRATION_SENSOR)==LOW){
    stepper.move(-20);
    stepper.run();
    delay(50);
  }
  digitalWrite(Floor1_LED, LOW);
  digitalWrite(Floor2_LED, LOW);
  digitalWrite(Floor3_LED, LOW);
  digitalWrite(Floor4_LED, LOW);

  registeredFloor=1;
  stepper.setCurrentPosition(0);

  lcd.clear();
  lcd.print("Calibrated @F1");
  Serial.println("Calibration done, floor=1");
  delay(2000);
  ledAlwaysOn();
}

/********************************************************
 * LED Patterns
 ********************************************************/
void ledAlwaysOn(){
  digitalWrite(LED_ESP, HIGH);
}

void blinkInternalLED(int times, int intervalMs){
  for(int i=0; i<times; i++){
    digitalWrite(LED_ESP, LOW);
    delay(intervalMs);
    digitalWrite(LED_ESP, HIGH);
    delay(intervalMs);
  }
  // End with LED on
  digitalWrite(LED_ESP, HIGH);
}

/********************************************************
 * If needed to interpret floor from pin
 ********************************************************/
int whichfloor(int pin) {
  if(pin==Floor1_reed) return 1;
  if(pin==Floor2_reed) return 2;
  if(pin==Floor3_reed) return 3;
  if(pin==Floor4_reed) return 4;
  return 0;
}
