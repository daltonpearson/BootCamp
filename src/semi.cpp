#include <Arduino.h>
#include <ESP32Servo.h>  // by Kevin Harrington
#include <esp_now.h>
#include <WiFi.h>


uint32_t thisReceiverIndex = 4;
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
uint16_t buttonMaskY = 8;      // Triangle on PS4
uint16_t buttonMaskA = 1;      // Cross on PS4
uint16_t buttonMaskB = 2;      // Circle on PS4
uint16_t buttonMaskX = 4;      // Square on PS4
// ControllerPtr myControllers[BP32_MAX_GAMEPADS];

/*What the different Serial commands for the trailer esp32 daughter board do
1-Trailer Legs Up
2-Trailer Legs Down
3-Ramp Up
4-Ramp Down
5-auxMotor1 Forward
6-auxMotor1 Reverse
7-auxMotor1 STOP
8-auxMotor2 Forward
9-auxMotor2 Reverse
10-auxMotor2 STOP
11- LT1 LOW
12- LT1 HIGH
13- LT2 LOW
14- LT2 HIGH
15- LT3 LOW
16- LT3 HIGH
*/
#define LT1 15
#define LT2 27
#define LT3 14

#define RX0 3
#define TX0 1

#define frontSteeringServoPin 23
#define hitchServoPin 22

Servo frontSteeringServo;
Servo hitchServo;

#define frontMotor0 33  // \ Used for controlling front drive motor movement
#define frontMotor1 32  // /
#define rearMotor0 2    // \ Used for controlling rear drive motor movement
#define rearMotor1 4    // /
#define rearMotor2 12   // \ Used for controlling second rear drive motor movement.
#define rearMotor3 13   // /

#define auxAttach0 18  // \ "Aux1" on PCB. Used for controlling auxillary motor or lights.  Keep in mind this will always breifly turn on when the model is powered on.
#define auxAttach1 5   // /
#define auxAttach2 17  // \ "AUX2" on PCB. Used for controlling auxillary motors or lights.
#define auxAttach3 16  // /
#define auxAttach4 25  // \ "Aux3" on PCB. Used for controlling auxillary motors or lights.
#define auxAttach5 26  // /

// Forward declarations
void flashConnectionIndicator();       // start the (non-blocking) indicator
void serviceConnectionIndicator();     // advance it; call every loop()

int lightSwitchButtonTime = 0;
int lightSwitchTime = 0;
int hitchButtonTime = 0;
const int hitchDebounceDelay = 500; // Debounce delay in milliseconds
int adjustedSteeringValue = 90;
int rawSteeringValue = 90; // Raw steering value from controller before trim
int hitchServoValueEngaged = 155;
int hitchServoValueDisengaged = 100;
int steeringTrim = 0;
int lightMode = 0;
bool lightsOn = false;
bool auxLightsOn = false;
bool blinkLT = false;
bool hazardLT = false;
bool hazardsOn = false;
bool smokeGenOn = false;
bool trailerAuxMtr1Forward = false;
bool trailerAuxMtr1Reverse = false;
bool trailerAuxMtr2Forward = false;
bool trailerAuxMtr2Reverse = false;
bool hitchUp = true;
bool trailerRampUp = true;
bool trailerLegsUp = true;
// Speed limit: RESTRICTED (50%) by default; press Triangle to toggle full speed.
bool reducedSpeedMode = true;
bool speedModeInit = false;
uint16_t prevSpeedButtons = 0;
unsigned long speedModeButtonTime = 0;
unsigned long rampButtonTime = 0;
unsigned long legsButtonTime = 0;
const int speedModeDebounceDelay = 500; // Debounce delay for speed mode toggle
const int rampDebounceDelay = 500; // Debounce delay for ramp toggle
const int legsDebounceDelay = 500; // Debounce delay for legs toggle

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
// receive callback; a delay() there would stall the WiFi task).
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
  uint8_t level = on ? HIGH : LOW;
  digitalWrite(LT1, level);
  digitalWrite(LT2, level);
  digitalWrite(LT3, level);
  // Brief steering twitch alongside the light flashes. Applied as an offset in
  // processSteering() so the 50Hz steering updates don't stomp it.
  indicatorServoOffset = on ? 8 : -8;
  if (++indicatorStep >= 6) {          // 3 on/off flashes
    indicatorRunning = false;
    digitalWrite(LT1, LOW);
    digitalWrite(LT2, LOW);
    digitalWrite(LT3, LOW);
    indicatorServoOffset = 0;
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
    // Fully stopped: clear the ramp/dead-time state so the next move starts clean.
    if (slot >= 0) {
      dtLevel[slot] = 0;
      dtCoast[slot] = false;
    }
  }
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


void processTrailerLegs(int value) {
  // Use Cross button (buttonMaskA) for toggling the trailer legs position
  if ((value & buttonMaskA) && (millis() - legsButtonTime > legsDebounceDelay)) {
    // Toggle the legs position
    if (trailerLegsUp) {
      Serial.println(2); // Legs Down
      Serial.println("Trailer Legs: Down");
      trailerLegsUp = false;
    } else {
      Serial.println(1); // Legs Up
      Serial.println("Trailer Legs: Up");
      trailerLegsUp = true;
    }
    delay(10);
    
    // Update the last button press time
    legsButtonTime = millis();
  }
}
void processTrailerRamp(int value) {
  // Use Circle button (buttonMaskB) for toggling the ramp position
  if ((value & buttonMaskB) && (millis() - rampButtonTime > rampDebounceDelay)) {
    // Toggle the ramp position
    if (trailerRampUp) {
      Serial.println(4); // Ramp Down
      Serial.println("Ramp: Down");
      trailerRampUp = false;
    } else {
      Serial.println(3); // Ramp Up
      Serial.println("Ramp: Up");
      trailerRampUp = true;
    }
    delay(10);
    
    // Update the last button press time
    rampButtonTime = millis();
  }
}

void processSpeedMode(uint16_t buttons) {
  // Triangle toggles between restricted (the default) and full speed. This is
  // EDGE-triggered (the old version re-fired every debounce period while the
  // button was held) and the first packet after a connect only seeds
  // prevSpeedButtons, so a sticky button bit at connect can't silently change
  // the mode.
  if (!speedModeInit) {
    prevSpeedButtons = buttons;
    speedModeInit = true;
    return;
  }
  uint16_t pressed = buttons & ~prevSpeedButtons;
  prevSpeedButtons = buttons;
  if (!(pressed & buttonMaskY)) return;
  if (millis() - speedModeButtonTime < speedModeDebounceDelay) return;
  speedModeButtonTime = millis();
  reducedSpeedMode = !reducedSpeedMode;
  Serial.print("Speed Mode: ");
  Serial.println(reducedSpeedMode ? "Restricted (50%)" : "Full (100%)");
}

void processThrottle(int axisYValue) {
  int adjustedThrottleValue = axisYValue / 2;
  
  // Apply 50% speed reduction if reduced speed mode is enabled
  if (reducedSpeedMode) {
    adjustedThrottleValue = adjustedThrottleValue / 2; // Further reduce to 50%
  }
  
  int smokeThrottle = adjustedThrottleValue / 3;
  
  moveMotor(rearMotor0, rearMotor1, adjustedThrottleValue);
  moveMotor(rearMotor2, rearMotor3, adjustedThrottleValue);
  moveMotor(frontMotor0, frontMotor1, adjustedThrottleValue);
  moveMotor(auxAttach2, auxAttach3, smokeThrottle);
}

void processTrimAndHitch(int dpadValue) {
  if (dpadValue == 4 && steeringTrim < 20) {
    steeringTrim = steeringTrim + 1;
    delay(50);
  } else if (dpadValue == 8 && steeringTrim > -20) {
    steeringTrim = steeringTrim - 1;
    delay(50);
  }
  
  // Hitch toggle with debounce - only process if sufficient time has passed since last button press
  if (dpadValue == 2 && (millis() - hitchButtonTime > hitchDebounceDelay)) {
    // Toggle the hitch state
    if (hitchUp) {
      hitchServo.write(hitchServoValueDisengaged);
      hitchUp = false;
    } else {
      hitchServo.write(hitchServoValueEngaged);
      hitchUp = true;
    }
    delay(10);
    
    // Update the last button press time
    hitchButtonTime = millis();
  }
}
void processSteering(int axisRXValue) {
  rawSteeringValue = 90 - (axisRXValue / 9); // Store raw steering value without trim
  adjustedSteeringValue = rawSteeringValue - steeringTrim; // Apply trim for actual steering
  // + connection-indicator twitch offset (0 when no indicator is running)
  frontSteeringServo.write(180 - adjustedSteeringValue + indicatorServoOffset);
}

void processLights(bool buttonValue) {
  if (buttonValue && (millis() - lightSwitchButtonTime) > 300) {
    lightMode++;
    if (lightMode == 1) {
      digitalWrite(LT1, HIGH);
      digitalWrite(LT2, HIGH);
      Serial.println(12);
      delay(10);
      Serial.println(14);
    } else if (lightMode == 2) {
      digitalWrite(LT1, LOW);
      digitalWrite(LT2, LOW);
      delay(100);
      digitalWrite(LT1, HIGH);
      digitalWrite(LT2, HIGH);
      blinkLT = true;
    } else if (lightMode == 3) {
      blinkLT = false;
      hazardLT = true;
    } else if (lightMode == 4) {
      hazardLT = false;
      digitalWrite(LT1, LOW);
      digitalWrite(LT2, LOW);
      Serial.println(11);
      delay(10);
      Serial.println(13);
      lightMode = 0;
      if (!auxLightsOn) {
        digitalWrite(LT3, HIGH);
        Serial.println(16);
        auxLightsOn = true;
      } else {
        digitalWrite(LT3, LOW);
        Serial.println(15);
        auxLightsOn = false;
      }
    }
    lightSwitchButtonTime = millis();
  }
}

void processSmokeGen(bool buttonValue) {
  if (buttonValue) {
    if (!smokeGenOn) {
      digitalWrite(auxAttach0, LOW);
      digitalWrite(auxAttach1, HIGH);
      smokeGenOn = true;
    } else {
      digitalWrite(auxAttach0, LOW);
      digitalWrite(auxAttach1, LOW);
      smokeGenOn = false;
    }
  }
}

void processTrailerAuxMtr1Forward(bool value) {
  if (value) {
      Serial.println(5);
      delay(10);
      trailerAuxMtr1Forward = true;
  } else if (trailerAuxMtr1Forward) {
    Serial.println(7);
    delay(10);
    trailerAuxMtr1Forward = false;
  }
}
void processTrailerAuxMtr1Reverse(bool value) {
  if (value) {
      Serial.println(6);
      delay(10);
      trailerAuxMtr1Reverse = true;
  } else if (trailerAuxMtr1Reverse) {
    Serial.println(7);
    delay(10);
    trailerAuxMtr1Reverse = false;
  }
}
void processTrailerAuxMtr2Forward(bool value) {
  if (value) {
      Serial.println(8);
      delay(10);
      trailerAuxMtr2Forward = true;
  } else if (trailerAuxMtr2Forward) {
    Serial.println(10);
    delay(10);
    trailerAuxMtr2Forward = false;
  }
}
void processTrailerAuxMtr2Reverse(bool value) {
  if (value) {
      Serial.println(9);
      delay(10);
      trailerAuxMtr2Reverse = true;
  } else if (trailerAuxMtr2Reverse) {
    Serial.println(10);
    delay(10);
    trailerAuxMtr2Reverse = false;
  }
}

void processGamepad() {
  //Throttle
  processThrottle(receivedData.axisY);
  //Steering
  processSteering(receivedData.axisRX);
  //Steering trim and hitch
  processTrimAndHitch(receivedData.dpad);
  //Lights
  processLights(receivedData.thumbR);
  processSmokeGen(receivedData.thumbL);
  
  // Process speed mode toggle (Triangle button)
  processSpeedMode(receivedData.buttons);

  processTrailerLegs(receivedData.buttons);
  processTrailerRamp(receivedData.buttons);

  processTrailerAuxMtr1Forward(receivedData.r1);
  processTrailerAuxMtr1Reverse(receivedData.r2);
  processTrailerAuxMtr2Forward(receivedData.l1);
  processTrailerAuxMtr2Reverse(receivedData.l2);

  if (blinkLT && (millis() - lightSwitchTime) > 300) {
    if (!lightsOn) {
      if (rawSteeringValue <= 85) {
        digitalWrite(LT1, HIGH);
        Serial.println(12);
      } else if (rawSteeringValue >= 95) {
        digitalWrite(LT2, HIGH);
        Serial.println(14);
      }
      lightsOn = true;
    } else {
      if (rawSteeringValue <= 85) {
        digitalWrite(LT2, HIGH);
        digitalWrite(LT1, LOW);
        Serial.println(11);
        delay(10);
        Serial.println(14);
      } else if (rawSteeringValue >= 95) {
        digitalWrite(LT1, HIGH);
        digitalWrite(LT2, LOW);
        Serial.println(13);
        delay(10);
        Serial.println(12);
      }
      lightsOn = false;
    }
    lightSwitchTime = millis();
  }
  if (blinkLT && rawSteeringValue > 85 && rawSteeringValue < 95) {
    digitalWrite(LT1, HIGH);
    digitalWrite(LT2, HIGH);
    Serial.println(12);
    delay(10);
    Serial.println(14);
  }
  if (hazardLT && (millis() - lightSwitchTime) > 300) {
    if (!hazardsOn) {
      digitalWrite(LT1, HIGH);
      digitalWrite(LT2, HIGH);
      Serial.println(12);
      delay(10);
      Serial.println(14);
      hazardsOn = true;
    } else {
      digitalWrite(LT1, LOW);
      digitalWrite(LT2, LOW);
      Serial.println(11);
      delay(10);
      Serial.println(13);
      hazardsOn = false;
    }
    lightSwitchTime = millis();
  }
}

void processControllers() {
  processGamepad();
}

// Failsafe: stop the drive and smoke motors, centre the steering, switch the
// lights off and tell the trailer daughter board to stop its aux motors.
void safeStop() {
  moveMotor(frontMotor0, frontMotor1, 0);
  moveMotor(rearMotor0, rearMotor1, 0);
  moveMotor(rearMotor2, rearMotor3, 0);
  moveMotor(auxAttach2, auxAttach3, 0);   // smoke generator throttle
  digitalWrite(auxAttach0, LOW);          // smoke generator on/off
  digitalWrite(auxAttach1, LOW);
  adjustedSteeringValue = 90 - steeringTrim;
  frontSteeringServo.write(180 - adjustedSteeringValue);
  digitalWrite(LT1, LOW);
  digitalWrite(LT2, LOW);
  digitalWrite(LT3, LOW);
  Serial.println(7);                      // trailer aux motor 1 STOP
  Serial.println(10);                     // trailer aux motor 2 STOP
}

void setup() {
  Serial.begin(115200);

  // Initialize connection variables
  connectionActive = false;
  lastPacketTime = 0;

  pinMode(rearMotor0, OUTPUT);
  pinMode(rearMotor1, OUTPUT);
  pinMode(rearMotor2, OUTPUT);
  pinMode(rearMotor3, OUTPUT);
  pinMode(frontMotor0, OUTPUT);
  pinMode(frontMotor1, OUTPUT);
  pinMode(auxAttach0, OUTPUT);
  pinMode(auxAttach1, OUTPUT);
  pinMode(auxAttach2, OUTPUT);
  pinMode(auxAttach3, OUTPUT);
  pinMode(auxAttach4, OUTPUT);
  pinMode(auxAttach5, OUTPUT);
  pinMode(LT1, OUTPUT);
  pinMode(LT2, OUTPUT);
  pinMode(LT3, OUTPUT);

  digitalWrite(rearMotor0, LOW);
  digitalWrite(rearMotor1, LOW);
  digitalWrite(rearMotor2, LOW);
  digitalWrite(rearMotor3, LOW);
  digitalWrite(frontMotor0, LOW);
  digitalWrite(frontMotor1, LOW);
  digitalWrite(auxAttach0, LOW);
  digitalWrite(auxAttach1, LOW);
  digitalWrite(auxAttach2, LOW);
  digitalWrite(auxAttach3, LOW);
  digitalWrite(auxAttach4, LOW);
  digitalWrite(auxAttach5, LOW);
  digitalWrite(LT1, LOW);
  digitalWrite(LT2, LOW);
  digitalWrite(LT3, LOW);

  frontSteeringServo.attach(frontSteeringServoPin);
  frontSteeringServo.write(adjustedSteeringValue);
  hitchServo.attach(hitchServoPin);
  hitchServo.write(hitchServoValueDisengaged); // Set to disengaged position on boot
  hitchUp = false; // Initialize hitchUp to match the actual servo position
  trailerRampUp = true; // Initialize ramp to up position by default
  trailerLegsUp = true; // Initialize legs to up position by default

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

  if (dataUpdated) {
    processControllers();
    dataUpdated = false;
  }
  else { vTaskDelay(1); }

  // Check for connection timeout
  if (connectionActive && (millis() - lastPacketTime > CONNECTION_TIMEOUT)) {
    // Link lost: fail safe at once (drive, smoke, steering, lights, trailer).
    connectionActive = false;
    safeStop();
  }
}