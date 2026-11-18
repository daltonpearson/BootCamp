#include <Arduino.h>
#include <ESP32Servo.h>  // by Kevin Harrington
#include <esp_now.h>
#include <WiFi.h>

uint32_t thisReceiverIndex = 2;

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

// Forward declarations
void processThrottle(int axisYValue);
void processMast(int axisRYValue);
void processTrimRight(int trimValue);
void processTrimLeft(int trimValue);
void processSteering(int axisRXValue);
void processMastTilt(int dpadValue);
void processAux(bool buttonValue);
void moveMotor(int motorPin0, int motorPin1, int velocity);
void processControllers();
void flashConnectionIndicator();       // start the (non-blocking) indicator
void serviceConnectionIndicator();     // advance it; call every loop()

#define steeringServoPin 23
#define mastTiltServoPin 22

#define mastMotor0 25  // Used for controlling auxiliary attachment movement
#define mastMotor1 26  // Used for controlling auxiliary attachment movement
#define auxAttach0 18  // Used for controlling auxiliary attachment movement
#define auxAttach1 17  // Used for controlling auxiliary attachment movement

#define leftMotor0 21   // Used for controlling the left motor movement
#define leftMotor1 19   // Used for controlling the left motor movement
#define rightMotor0 33  // Used for controlling the right motor movement
#define rightMotor1 32  // Used for controlling the right motor movement

#define FORWARD 1
#define BACKWARD -1
#define STOP 0

Servo steeringServo;
Servo mastTiltServo;

int servoDelay = 0;
int lightSwitchTime = 0;
float adjustedSteeringValue = 86;
float steeringAdjustment = 1;
int steeringTrim = 0;
int mastTiltValue = 90;

bool lightsOn = false;
bool moveMastTiltServoDown = false;
bool moveMastTiltServoUp = false;
bool hardLeft;
bool hardRight;

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
}

// Connection indicator - deliberately NON-BLOCKING (started from the ESP-NOW
// receive callback; a delay() there would stall the WiFi task). Nudges the
// steering servo left/right twice, then hands the servo back to real steering.
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
  if (millis() - indicatorStepTime < 150) return;
  indicatorStepTime = millis();
  if (indicatorStep >= 4) {            // two left/right swings done
    indicatorRunning = false;
    indicatorServoOffset = 0;
    return;
  }
  // +/-30 deg swing either side of the real steering angle. Applied as an
  // offset in processSteering() so the 50Hz steering updates don't stomp it.
  indicatorServoOffset = (indicatorStep % 2) == 0 ? 30 : -30;
  indicatorStep++;
}

void processGamepad() {
  //Throttle
  processThrottle(receivedData.axisY);
  //Steering
  processSteering(receivedData.axisRX);
  //Rasing and lowering of mast
  processMast(receivedData.axisRY);
  //MastTilt
  processMastTilt(receivedData.dpad);
  //Aux
  processAux(receivedData.thumbR);

  processTrimRight(receivedData.r1);
  processTrimLeft(receivedData.l1);

  if (receivedData.l2) {
    hardLeft = true;
  } else {
    hardLeft = false;
  }
  if (receivedData.r2) {
    hardRight = true;
  } else {
    hardRight = false;
  }
}

void processThrottle(int axisYValue) {
  float adjustedThrottleValue = axisYValue / 2;
  if (adjustedThrottleValue > 15 || adjustedThrottleValue < -15) {
    if (hardRight) {
      moveMotor(rightMotor0, rightMotor1, -1 * (adjustedThrottleValue * steeringAdjustment));
    } else if (hardLeft) {
      moveMotor(leftMotor0, leftMotor1, -1 * (adjustedThrottleValue * steeringAdjustment));
    } else if (adjustedSteeringValue > 100) {
      moveMotor(leftMotor0, leftMotor1, adjustedThrottleValue * steeringAdjustment);
      moveMotor(rightMotor0, rightMotor1, adjustedThrottleValue);
    } else if (adjustedSteeringValue < 80) {
      moveMotor(leftMotor0, leftMotor1, adjustedThrottleValue);
      moveMotor(rightMotor0, rightMotor1, adjustedThrottleValue * steeringAdjustment);
    } else {
      moveMotor(leftMotor0, leftMotor1, adjustedThrottleValue);
      moveMotor(rightMotor0, rightMotor1, adjustedThrottleValue);
    }
  } else {
    moveMotor(leftMotor0, leftMotor1, 0);
    moveMotor(rightMotor0, rightMotor1, 0);
  }
}

void processMast(int axisRYValue) {
  int adjustedMastValue = axisRYValue / 2;
  if (adjustedMastValue > 100 || adjustedMastValue < -100) {
    moveMotor(mastMotor0, mastMotor1, adjustedMastValue);
  } else {
    moveMotor(mastMotor0, mastMotor1, 0);
  }
}

void processTrimRight(int trimValue) {
  if (trimValue == 1 && trimValue < 20) {
    steeringTrim = steeringTrim + 2;
    delay(50);
  }
}

void processTrimLeft(int trimValue) {
  if (trimValue == 1 && trimValue > -20) {
    steeringTrim = steeringTrim - 2;
    delay(50);
  }
}

void processSteering(int axisRXValue) {
  adjustedSteeringValue = (90 - (axisRXValue / 9));
  // + connection-indicator twitch offset (0 when no indicator is running)
  steeringServo.write(adjustedSteeringValue - steeringTrim + indicatorServoOffset);

  if (adjustedSteeringValue > 100) {
    steeringAdjustment = ((200 - adjustedSteeringValue) / 100);
  } else if (adjustedSteeringValue < 80) {
    steeringAdjustment = ((200 - (90 + (90 - adjustedSteeringValue))) / 100);
  }
}

void processMastTilt(int dpadValue) {
  if (dpadValue == 1) {
    if (servoDelay == 4) {
      if (mastTiltValue >= 10 && mastTiltValue < 170) {
        //if using a ps3 controller that was flashed as an xbox360 controller change the value "1 " below to a "3" or "4" to make up for the slower movement.
        mastTiltValue = mastTiltValue + 1;
        mastTiltServo.write(mastTiltValue);
      }
      servoDelay = 0;
    }
    servoDelay++;
  } else if (dpadValue == 2) {
    if (servoDelay == 4) {
      if (mastTiltValue <= 170 && mastTiltValue > 10) {
        //if using a ps3 controller that was flashed as an xbox360 controller change the value "1" below to a "3" or "4" to make up for the slower movement.
        mastTiltValue = mastTiltValue - 1;
        mastTiltServo.write(mastTiltValue);
      }
      servoDelay = 0;
    }
    servoDelay++;
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

  int8_t dir    = (velocity > 1) ? 1 : ((velocity < -1) ? -1 : 0);
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
    // Fully stopped: clear the ramp/dead-time state so the next move starts clean.
    if (slot >= 0) {
      dtLevel[slot] = 0;
      dtCoast[slot] = false;
    }
  }
}

void processControllers() {
  processGamepad();
}

// Failsafe: cut the drive and mast motors, centre the steering and drop the aux
// output so a lost link can't leave the forklift driving or sticking at a limit.
void safeStop() {
  moveMotor(leftMotor0, leftMotor1, 0);
  moveMotor(rightMotor0, rightMotor1, 0);
  moveMotor(mastMotor0, mastMotor1, 0);
  adjustedSteeringValue = 90 - steeringTrim;
  steeringServo.write(adjustedSteeringValue);
  digitalWrite(auxAttach0, LOW);
  digitalWrite(auxAttach1, LOW);
}

// Arduino setup function. Runs in CPU 1
void setup() {
  pinMode(mastMotor0, OUTPUT);
  pinMode(mastMotor1, OUTPUT);
  pinMode(auxAttach0, OUTPUT);
  pinMode(auxAttach1, OUTPUT);
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
  mastTiltServo.attach(mastTiltServoPin);
  mastTiltServo.write(mastTiltValue);

  Serial.begin(115200);

  WiFi.setSleep(false);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
  }
  esp_now_register_recv_cb(OnDataRecv);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
  serviceConnectionIndicator();

  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  if (dataUpdated) {
    processControllers();
    dataUpdated = false;
  }
  else { 
    // Check if connection has timed out
    if (connectionActive && (millis() - lastPacketTime > CONNECTION_TIMEOUT)) {
      // Link lost: fail safe at once (drive, mast, steering, aux).
      connectionActive = false;
      safeStop();
    }
    vTaskDelay(1); 
  }
}
