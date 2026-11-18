#include <ESP32Servo.h>  // by Kevin Harrington
#include "Adafruit_MCP23X17.h"
#include <esp_now.h>
#include <WiFi.h>
uint32_t thisReceiverIndex = 1;
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
// defines
#define clawServoPin 5
#define auxServoPin 18
#define cabLights 32
#define auxLights 33

#define pivot0 15
#define pivot1 14
// #define mainBoom0 9
// #define mainBoom1 8
// #define dipper0 0
// #define dipper1 1
#define mainBoom0 1
#define mainBoom1 0
#define dipper0 8
#define dipper1 9
#define tiltAttach0 2
#define tiltAttach1 3
#define thumb0 10
#define thumb1 11
#define auxAttach0 12
#define auxAttach1 13

#define leftMotor0 7
#define leftMotor1 6
#define rightMotor0 4
#define rightMotor1 5

// Forward declarations
void flashConnectionIndicator();       // start the (non-blocking) indicator
void serviceConnectionIndicator();     // advance it; call every loop()

Adafruit_MCP23X17 mcp;
Servo clawServo;
Servo auxServo;

int dly = 250;
int clawServoValue = 90;
int auxServoValue = 90;
int servoDelay = 0;
int lightSwitchTime = 0;

bool cabLightsOn = false;
bool auxLightsOn = false;
bool moveClawServoUp = false;
bool moveClawServoDown = false;
bool moveAuxServoUp = false;
bool moveAuxServoDown = false;

void dumpGamepadState() {
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d "
        "misc: 0x%02x, misc Forward: %d, misc Backward: %d, misc Reset: %d, R1: %d, L1: %d, R2: %d, L2: %d, idx=%d\n",
        receivedData.receiverIndex,        // Receiver Index
        receivedData.dpad,         // D-pad
        receivedData.buttons,      // bitmask of pressed buttons
        receivedData.axisX,        // (-511 - 512) left X Axis
        receivedData.axisY,        // (-511 - 512) left Y axis
        receivedData.axisRX,       // (-511 - 512) right X axis
        receivedData.axisRY,       // (-511 - 512) right Y axis
        receivedData.brake,        // (0 - 1023): brake button
        receivedData.throttle,     // (0 - 1023): throttle (AKA gas) button
        receivedData.miscButtons,  // bitmask of pressed "misc" buttons
        receivedData.miscButtons & 4,
        receivedData.miscButtons & 2,
        receivedData.miscButtons & 8,
        receivedData.r1,
        receivedData.l1,
        receivedData.r2,
        receivedData.l2,
        receivedData.receiverIndex
    );
}
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


// Direction dead-time for MCP-driven H-bridges: when the commanded direction
// flips, coast (both outputs LOW) for one control tick before driving the new
// direction, so the bridge never sees an instantaneous reversal. dir: +1/-1/0.
void mcpMotor(int pin0, int pin1, int8_t dir) {
  static const int N = 8;
  static int    p0[N]      = {-1, -1, -1, -1, -1, -1, -1, -1};
  static int    p1[N]      = {-1, -1, -1, -1, -1, -1, -1, -1};
  static int8_t lastDir[N] = {0, 0, 0, 0, 0, 0, 0, 0};
  static bool   coast[N]   = {false, false, false, false, false, false, false, false};

  int slot = -1, freeSlot = -1;
  for (int i = 0; i < N; i++) {
    if (p0[i] == pin0 && p1[i] == pin1) { slot = i; break; }
    if (freeSlot < 0 && p0[i] == -1) freeSlot = i;
  }
  if (slot < 0 && freeSlot >= 0) {
    slot = freeSlot;
    p0[slot] = pin0; p1[slot] = pin1; lastDir[slot] = 0; coast[slot] = false;
  }

  if (slot >= 0 && dir != 0 && lastDir[slot] != 0 && dir != lastDir[slot]) {
    if (!coast[slot]) {          // reversal: hold the bridge off for this tick
      coast[slot] = true;
      mcp.digitalWrite(pin0, LOW);
      mcp.digitalWrite(pin1, LOW);
      return;
    }
    coast[slot] = false;         // dead-time served - allow the new direction
  }
  if (slot >= 0) lastDir[slot] = dir;

  mcp.digitalWrite(pin0, dir > 0 ? HIGH : LOW);
  mcp.digitalWrite(pin1, dir < 0 ? HIGH : LOW);
}

void processBoom(int axisYValue) {
  int adjustedValue = axisYValue / 2;
  mcpMotor(mainBoom0, mainBoom1, adjustedValue > 100 ? 1 : (adjustedValue < -100 ? -1 : 0));
}
void processPivot(int axisYValue) {
  int adjustedValue = axisYValue / 2;
  // Create a deadzone in the center - only activate when stick is moved to edges
  int deadzone = 75; // Deadzone threshold

  mcpMotor(pivot0, pivot1,
           adjustedValue > (100 + deadzone) ? 1 : (adjustedValue < (-100 - deadzone) ? -1 : 0));
}
void processDipper(int axisYValue) {
  int adjustedValue = axisYValue / 2;
  mcpMotor(dipper0, dipper1, adjustedValue > 100 ? 1 : (adjustedValue < -100 ? -1 : 0));
}
void processBucket(int axisYValue) {
  int adjustedValue = axisYValue / 2;
  mcpMotor(tiltAttach0, tiltAttach1, adjustedValue > 100 ? 1 : (adjustedValue < -100 ? -1 : 0));
}
void processAux(int dpadValue) {
  mcpMotor(thumb0, thumb1, dpadValue == 1 ? 1 : (dpadValue == 2 ? -1 : 0));
  mcpMotor(auxAttach0, auxAttach1, dpadValue == 4 ? 1 : (dpadValue == 8 ? -1 : 0));
}
void processGamepad() {
  //Boom
  processBoom(receivedData.axisY);
  //Pivot
  processPivot(receivedData.axisX);
  //Dipper
  processDipper(receivedData.axisRY);
  //TiltAttach
  processBucket(receivedData.axisRX);
  //Aux
  processAux(receivedData.dpad);
  //Lights
  if ((millis() - lightSwitchTime) > 200) {
    if (receivedData.thumbR == 1) {
      if (!cabLightsOn) {
        digitalWrite(cabLights, HIGH);
        cabLightsOn = true;
      } else {
        digitalWrite(cabLights, LOW);
        cabLightsOn = false;
      }
      delay(80);
    }
    if (receivedData.thumbL == 1) {
      if (!auxLightsOn) {
        digitalWrite(auxLights, HIGH);
        auxLightsOn = true;
      } else {
        digitalWrite(auxLights, LOW);
        auxLightsOn = false;
      }
      delay(80);
    }
    lightSwitchTime = millis();
  }
  // Drive motors, now via the dead-time helper. Also fixes the old copy/paste
  // bug in the left-motor stop condition (it tested l1/r2 instead of l2).
  mcpMotor(rightMotor0, rightMotor1, receivedData.r1 == 1 ? 1 : (receivedData.r2 == 1 ? -1 : 0));
  mcpMotor(leftMotor0, leftMotor1, receivedData.l1 == 1 ? 1 : (receivedData.l2 == 1 ? -1 : 0));

  if (receivedData.buttons & 1) {
    moveClawServoDown = true;
  } else if (receivedData.buttons & 8) {
    moveClawServoUp = true;
  } else {
    moveClawServoDown = false;
    moveClawServoUp = false;
  }
  if (receivedData.buttons & 4) {
    moveAuxServoDown = true;
  } else if (receivedData.buttons & 2) {
    moveAuxServoUp = true;
  } else {
    moveAuxServoDown = false;
    moveAuxServoUp = false;
  }  if (moveClawServoUp) {
    if (servoDelay == 2) {
      if (clawServoValue >= 10 && clawServoValue <= 164) {
        //if using a ps3 controller that was flashed an xbox360 controller change the value "2" below to a 3-4 to make up for the slower movement.
        clawServoValue = clawServoValue + 6;
        clawServo.write(clawServoValue);
      }
      servoDelay = 0;
    }
    servoDelay++;
  }
  if (moveClawServoDown) {
    if (servoDelay == 2) {
      if (clawServoValue <= 170 && clawServoValue >= 16) {
         //if using a ps3 controller that was flashed an xbox360 controller change the value "2" below to a 3-4 to make up for the slower movement.
        clawServoValue = clawServoValue - 6;
        clawServo.write(clawServoValue);
      }
      servoDelay = 0;
    }
    servoDelay++;
  }
  if (moveAuxServoUp) {
    if (servoDelay == 2) {
      if (auxServoValue >= 10 && auxServoValue < 170) {
         //if using a ps3 controller that was flashed an xbox360 controller change the value "2" below to a 3-4 to make up for the slower movement.
        auxServoValue = auxServoValue + 4;
        auxServo.write(auxServoValue);
      }
      servoDelay = 0;
    }
    servoDelay++;
  }
  if (moveAuxServoDown) {
    if (servoDelay == 2) {
      if (auxServoValue <= 170 && auxServoValue > 10) {
         //if using a ps3 controller that was flashed an xbox360 controller change the value "2" below to a 3-4 to make up for the slower movement.
        auxServoValue = auxServoValue - 2;
        auxServo.write(auxServoValue);
      }
      servoDelay = 0;
    }
    servoDelay++;
  }
}
void processControllers() {
  processGamepad();
}

// Failsafe: release every actuator so a lost link can't keep driving the boom,
// pivot, dipper, tilt, thumb or aux motors into their limits. The claw and aux
// servos hold position (they are position-commanded, not continuously driven).
void safeStop() {
  mcp.digitalWrite(mainBoom0, LOW);
  mcp.digitalWrite(mainBoom1, LOW);
  mcp.digitalWrite(pivot0, LOW);
  mcp.digitalWrite(pivot1, LOW);
  mcp.digitalWrite(dipper0, LOW);
  mcp.digitalWrite(dipper1, LOW);
  mcp.digitalWrite(tiltAttach0, LOW);
  mcp.digitalWrite(tiltAttach1, LOW);
  mcp.digitalWrite(thumb0, LOW);
  mcp.digitalWrite(thumb1, LOW);
  mcp.digitalWrite(auxAttach0, LOW);
  mcp.digitalWrite(auxAttach1, LOW);
  mcp.digitalWrite(leftMotor0, LOW);
  mcp.digitalWrite(leftMotor1, LOW);
  mcp.digitalWrite(rightMotor0, LOW);
  mcp.digitalWrite(rightMotor1, LOW);
}

void setup() {

  Serial.begin(115200);

  // Initialize connection variables
  connectionActive = false;
  lastPacketTime = 0;

  mcp.begin_I2C();
  //   put your setup code here, to run once:


  for (int i = 0; i <= 15; i++) {
    mcp.pinMode(i, OUTPUT);
  }

  pinMode(clawServoPin, OUTPUT);
  pinMode(auxServoPin, OUTPUT);

  pinMode(cabLights, OUTPUT);
  pinMode(auxLights, OUTPUT);

  clawServo.attach(clawServoPin);
  auxServo.attach(auxServoPin);
  clawServo.write(clawServoValue);
  auxServo.write(auxServoValue);
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

    if (receivedData.receiverIndex == thisReceiverIndex) {
      processControllers();
      dataUpdated = false;
    }
  }
  else { 
    // Check if connection has timed out
    if (connectionActive && (millis() - lastPacketTime > CONNECTION_TIMEOUT)) {
      // Link lost: release every actuator at once.
      connectionActive = false;
      safeStop();
    }
    vTaskDelay(1); 
  }
}

// Connection indicator - deliberately NON-BLOCKING (started from the ESP-NOW
// receive callback; a delay() there would stall the WiFi task).
bool indicatorRunning = false;
uint8_t indicatorStep = 0;
uint32_t indicatorStepTime = 0;

void flashConnectionIndicator() {
  indicatorRunning = true;
  indicatorStep = 0;
  indicatorStepTime = millis();
}

void serviceConnectionIndicator() {
  if (!indicatorRunning) return;
  if (millis() - indicatorStepTime < 200) return;
  indicatorStepTime = millis();
  digitalWrite(cabLights, (indicatorStep % 2) == 0 ? HIGH : LOW);
  if (++indicatorStep >= 6) {          // 3 on/off flashes
    indicatorRunning = false;
    digitalWrite(cabLights, LOW);
  }
}