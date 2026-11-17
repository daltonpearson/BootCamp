#include <Arduino.h>
#include <ESP32Servo.h>  // by Kevin Harrington
#include <esp_now.h>
#include <WiFi.h>
uint32_t thisReceiverIndex = 3;
// Structure to receive message
typedef struct struct_message {
    uint32_t receiverIndex;
    uint16_t buttons;
    uint8_t dpad;
    int32_t axisX, axisY;
    int32_t axisRX, axisRY;
    uint32_t brake, throttle;
    uint16_t miscButtons;
    bool thumbR, thumbL, r1, l1, r2, l2;
    uint8_t controllerSlot;   // Which base-station controller slot sent this packet
    uint32_t selectionTime;   // millis() when that controller last changed vehicle
} struct_message;
bool dataUpdated;
bool initialConnectionMade = false; // Flag to track if initial connection has been established
bool connectionActive = false; // Tracks if connection is currently active
unsigned long lastPacketTime = 0; // Timestamp of last received packet
// Failsafe window. The base heartbeats every 20ms, but ESP-NOW broadcast over a
// radio shared with two Bluetooth links drops bursts, so allow several missed
// heartbeats before cutting power. This is only a backstop: a real pad
// disconnect is stopped immediately by the base's "neutral" packets.
const unsigned long CONNECTION_TIMEOUT = 500; // ms without a packet = link lost
struct_message receivedData;

// Which base controller owns this vehicle (-1 = none) and the time of its last
// deliberate selection. Keeps two controllers from fighting over one vehicle.
int activeControllerSlot = -1;
uint32_t activeSelectionTime = 0;
// ControllerPtr myControllers[BP32_MAX_GAMEPADS];

#define steeringServoPin 23
#define auxServoPin 22

#define auxAttach0 25  // Used for controlling auxiliary attachment movement
#define auxAttach1 26  // Used for controlling auxiliary attachment movement
#define auxAttach2 18  // Used for controlling auxiliary attachment movement
#define auxAttach3 17  // Used for controlling auxiliary attachment movement

#define leftMotor0 33   // Used for controlling the left motor movement
#define leftMotor1 32   // Used for controlling the left motor movement
#define rightMotor0 21  // Used for controlling the right motor movement
#define rightMotor1 19  // Used for controlling the right motor movement


#define FORWARD 1
#define BACKWARD -1
#define STOP 0

// Forward function declarations
void flashConnectionIndicator();       // start the (non-blocking) indicator
void serviceConnectionIndicator();     // advance it; call every loop()

Servo steeringServo;
Servo auxServo;
int lightSwitchTime = 0;
int adjustedSteeringValue = 86;
int steeringTrim = 0;
int auxServoValue = 90;
bool lightsOn = false;

// Callback function for received data
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    struct_message tempReceivedData;
    memcpy(&tempReceivedData, incomingData, sizeof(receivedData));

    // Ownership filter: the current owner keeps control; another controller can
    // take over only by making a newer, deliberate vehicle selection.
    bool owned = (tempReceivedData.controllerSlot == activeControllerSlot);
    bool newer = (tempReceivedData.selectionTime > activeSelectionTime);
    if (tempReceivedData.receiverIndex == thisReceiverIndex &&
        (activeControllerSlot == -1 || owned || newer)) {
      activeControllerSlot = tempReceivedData.controllerSlot;
      if (newer) activeSelectionTime = tempReceivedData.selectionTime;
      memcpy(&receivedData, &tempReceivedData, sizeof(receivedData));
      dataUpdated = true;
      
      // Update connection timestamp
      lastPacketTime = millis();
      
      // Check if connection needs to be re-established
      if (!connectionActive) {
        connectionActive = true;
        flashConnectionIndicator();
      }
    }
    // Serial.print("Buttons: ");
    // Serial.println(receivedData.buttons);

    // Serial.print("Axis X: ");
    // Serial.println(receivedData.axisX);

    // Serial.print("Axis Y: ");
    // Serial.println(receivedData.axisY);

    // Serial.println("----------------------");
}

void moveMotor(int motorPin0, int motorPin1, int velocity) {
  // Two per-motor protections for the H-bridge (static slot table):
  //  * dead-time - on a direction flip, coast (both inputs LOW) for one control
  //    tick, so the bridge never sees an instantaneous reversal;
  //  * soft-start ramp - the applied duty steps toward the commanded duty, so
  //    the motor eases into speed instead of slamming on.
  const int RAMP_STEP = 16;   // duty units per tick (~50Hz) => ~320ms to full
  static int    dtPin0[4]  = {-1, -1, -1, -1};
  static int    dtPin1[4]  = {-1, -1, -1, -1};
  static int8_t dtDir[4]   = {0, 0, 0, 0};
  static bool   dtCoast[4] = {false, false, false, false};
  static int    dtLevel[4] = {0, 0, 0, 0};   // currently applied duty magnitude

  int8_t dir    = (velocity > 15) ? 1 : ((velocity < -15) ? -1 : 0);
  int    target = (dir == 0) ? 0 : ((velocity < 0) ? -velocity : velocity);

  int slot = -1, freeSlot = -1;
  for (int i = 0; i < 4; i++) {
    if (dtPin0[i] == motorPin0 && dtPin1[i] == motorPin1) { slot = i; break; }
    if (freeSlot < 0 && dtPin0[i] == -1) freeSlot = i;
  }
  if (slot < 0 && freeSlot >= 0) {
    slot = freeSlot;
    dtPin0[slot] = motorPin0; dtPin1[slot] = motorPin1;
    dtDir[slot] = 0; dtCoast[slot] = false; dtLevel[slot] = 0;
  }

  if (slot >= 0 && dir != 0 && dtDir[slot] != 0 && dir != dtDir[slot]) {
    if (!dtCoast[slot]) {          // reversal: hold the bridge off for this tick
      dtCoast[slot] = true;
      dtLevel[slot] = 0;           // restart the ramp in the new direction
      analogWrite(motorPin0, 0);
      analogWrite(motorPin1, 0);
      return;
    }
    dtCoast[slot] = false;         // dead-time served - allow the new direction
  }
  if (slot >= 0) dtDir[slot] = dir;

  int level = (slot >= 0) ? dtLevel[slot] : target;
  if (level < target)      level = min(target, level + RAMP_STEP);
  else if (level > target) level = max(target, level - RAMP_STEP);
  if (slot >= 0) dtLevel[slot] = level;

  if (dir > 0) {
    analogWrite(motorPin0, level);
    analogWrite(motorPin1, LOW);
  } else if (dir < 0) {
    analogWrite(motorPin0, LOW);
    analogWrite(motorPin1, level);
  } else {
    analogWrite(motorPin0, 0);
    analogWrite(motorPin1, 0);
  }
}

// Failsafe: cut the drive, centre the steering and release the dump bed, so a
// lost link can neither run the truck on nor hold an accessory at its limit.
void safeStop() {
  moveMotor(leftMotor0, leftMotor1, 0);
  moveMotor(rightMotor0, rightMotor1, 0);
  adjustedSteeringValue = 90 - steeringTrim;
  steeringServo.write(adjustedSteeringValue);
  digitalWrite(auxAttach2, LOW);
  digitalWrite(auxAttach3, LOW);
}

void moveServo(int movement, Servo &servo, int &servoValue) {
  switch (movement) {
    case 1:
      if (servoValue >= 10 && servoValue < 170) {
        servoValue = servoValue + 5;
        servo.write(servoValue);
        delay(10);
      }
      break;
    case -1:
      if (servoValue <= 170 && servoValue > 10) {
        servoValue = servoValue - 5;
        servo.write(servoValue);
        delay(10);
      }
      break;
  }
}

// Connection indicator - deliberately NON-BLOCKING. It is (re)started from the
// ESP-NOW receive callback, so any delay() here would stall the WiFi task and
// cause further packet loss. flashConnectionIndicator() just arms it;
// serviceConnectionIndicator() advances the flash from loop().
bool indicatorRunning = false;
uint8_t indicatorStep = 0;
uint32_t indicatorStepTime = 0;
int indicatorServoOffset = 0;   // steering twitch added while the indicator runs

void flashConnectionIndicator() {
  indicatorRunning = true;
  indicatorStep = 0;
  indicatorStepTime = millis();
}

void serviceConnectionIndicator() {
  if (!indicatorRunning) return;
  if (millis() - indicatorStepTime < 200) return;
  indicatorStepTime = millis();
  bool on = (indicatorStep % 2) == 0;
  digitalWrite(auxAttach0, on ? HIGH : LOW);
  digitalWrite(auxAttach1, LOW);
  // Brief steering twitch (small +/-8 deg swing) alongside the light flashes.
  // Applied as an offset in processSteering() so it survives the 50Hz updates.
  indicatorServoOffset = on ? 8 : -8;
  if (++indicatorStep >= 6) {          // 3 on/off flashes
    indicatorRunning = false;
    digitalWrite(auxAttach0, LOW);
    digitalWrite(auxAttach1, LOW);
    indicatorServoOffset = 0;
  }
}

// --- Speed limiting -------------------------------------------------------
// Restricted by default; press Triangle to toggle between restricted and full
// speed. Edge-triggered (one press = one toggle) with a debounce, and the first
// packet after a connect only seeds prevButtons so a sticky button bit at
// connect can't silently switch us to full speed.
const uint16_t buttonMaskTriangle = 8;         // PS4 Triangle
const uint32_t SPEED_MODE_DEBOUNCE = 250;      // ms
const int      RESTRICTED_SPEED_PERCENT = 50;  // top speed while restricted
bool     fullSpeedMode = false;                // false = restricted (default)
bool     speedModeInit = false;
uint16_t prevButtons   = 0;
uint32_t speedModeTime = 0;

void processSpeedMode(uint16_t buttons) {
  if (!speedModeInit) {          // seed on first packet; do not act on it
    prevButtons = buttons;
    speedModeInit = true;
    return;
  }
  uint16_t pressed = buttons & ~prevButtons;
  prevButtons = buttons;
  if (!(pressed & buttonMaskTriangle)) return;
  if (millis() - speedModeTime < SPEED_MODE_DEBOUNCE) return;
  speedModeTime = millis();
  fullSpeedMode = !fullSpeedMode;
}

void processThrottle(int axisYValue) {
  int adjustedThrottleValue = axisYValue / 2;
  if (!fullSpeedMode) {
    adjustedThrottleValue = adjustedThrottleValue * RESTRICTED_SPEED_PERCENT / 100;
  }
  moveMotor(leftMotor0, leftMotor1, adjustedThrottleValue);
  moveMotor(rightMotor0, rightMotor1, adjustedThrottleValue);
}
void processTrimRight(int trimValue)
{
  if(trimValue == 1 && trimValue < 20)
  {
  steeringTrim = steeringTrim + 2;
  delay(50);
  }
}

void processTrimLeft(int trimValue)
{
  if(trimValue == 1 && trimValue > -20)
  {
  steeringTrim = steeringTrim - 2;
  delay(50);
  }
}


void processSteering(int axisRXValue) {
  adjustedSteeringValue = (90 - (axisRXValue / 11))-steeringTrim;
  // + the connection-indicator twitch offset (0 when no indicator is running)
  steeringServo.write(adjustedSteeringValue + indicatorServoOffset);
}

void processDumpBed(int dpadValue) {

  if (dpadValue == 1) {
    digitalWrite(auxAttach2, HIGH);
    digitalWrite(auxAttach3, LOW);
  } else if (dpadValue == 2) {
    digitalWrite(auxAttach2, LOW);
    digitalWrite(auxAttach3, HIGH);
  } else {
    digitalWrite(auxAttach2, LOW);
    digitalWrite(auxAttach3, LOW);
  }
}
void processAux(bool buttonValue) {
  if (buttonValue && (millis() - lightSwitchTime) > 200) {
    if (lightsOn) {
      digitalWrite(auxAttach0, LOW);
      digitalWrite(auxAttach1, LOW);
      lightsOn = false;
    } else {
      digitalWrite(auxAttach0, HIGH);
      digitalWrite(auxAttach1, LOW);
      lightsOn = true;
    }

    lightSwitchTime = millis();
  }
}

void processGamepad() {
  //Throttle
  processThrottle(receivedData.axisY);
  //Steering
  processSteering(receivedData.axisRX);
  //DumpBed
  processDumpBed(receivedData.dpad);
  //Aux
  processAux(receivedData.thumbR);
  //Speed mode (Triangle toggles restricted <-> full speed)
  processSpeedMode(receivedData.buttons);

  processTrimRight(receivedData.r1);
  processTrimLeft(receivedData.l1);

}
void processControllers() {
  processGamepad();
}

// Arduino setup function. Runs in CPU 1
void setup() {
  pinMode(auxAttach2, OUTPUT);
  pinMode(auxAttach3, OUTPUT);
  pinMode(auxAttach0, OUTPUT);
  pinMode(auxAttach1, OUTPUT);
  digitalWrite(auxAttach2, LOW);
  digitalWrite(auxAttach3, LOW);
  digitalWrite(auxAttach0, LOW);
  digitalWrite(auxAttach1, LOW);
  pinMode(leftMotor0, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(rightMotor0, OUTPUT);
  pinMode(rightMotor1, OUTPUT);

  // Initialize connection variables
  connectionActive = false;
  lastPacketTime = 0;

  steeringServo.attach(steeringServoPin);
  steeringServo.write(adjustedSteeringValue);

  Serial.begin(115200);
  WiFi.setSleep(false);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
  }
  esp_now_register_recv_cb(OnDataRecv);
}


void loop() {
  serviceConnectionIndicator();

  if (dataUpdated) {
    // dumpGamepadState();
    processControllers();
    dataUpdated = false;
  }
  else { 
    // Check if connection has timed out
    if (connectionActive && (millis() - lastPacketTime > CONNECTION_TIMEOUT)) {
      // Link lost: fail safe at once (motors, steering, dump bed).
      connectionActive = false;
      safeStop();
    }
    vTaskDelay(1); 
  }
}
