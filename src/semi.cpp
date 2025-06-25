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
} struct_message;
bool dataUpdated;
bool connectionActive = false; // Tracks if connection is currently active
unsigned long lastPacketTime = 0; // Timestamp of last received packet
const unsigned long CONNECTION_TIMEOUT = 3000; // 3 seconds timeout for connection
struct_message receivedData;
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
void flashConnectionIndicator();

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
bool reducedSpeedMode = false;
unsigned long speedModeButtonTime = 0;
const int speedModeDebounceDelay = 500; // Debounce delay for speed mode toggle

// Callback function for received data
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    struct_message tempReceivedData;
    memcpy(&tempReceivedData, incomingData, sizeof(receivedData));
    if (tempReceivedData.receiverIndex == thisReceiverIndex){
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

// Function to flash lights as a connection indicator
void flashConnectionIndicator() {
  // Flash the lights 3 times when connected
  for (int i = 0; i < 3; i++) {
    digitalWrite(LT1, HIGH);
    digitalWrite(LT2, HIGH);
    digitalWrite(LT3, HIGH);
    delay(200);
    digitalWrite(LT1, LOW);
    digitalWrite(LT2, LOW);
    digitalWrite(LT3, LOW);
    delay(200);
  }
}

void moveMotor(int motorPin0, int motorPin1, int velocity) {
  if (velocity > 15) {
    analogWrite(motorPin0, velocity);
    analogWrite(motorPin1, LOW);
  } else if (velocity < -15) {
    analogWrite(motorPin0, LOW);
    analogWrite(motorPin1, (-1 * velocity));
  } else {
    analogWrite(motorPin0, 0);
    analogWrite(motorPin1, 0);
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


void processTrailerLegsUp(int value) {
  if (value & buttonMaskY) {
    Serial.println(1);
    delay(10);
  }
}
void processTrailerLegsDown(int value) {
  if (value & buttonMaskA) {
    Serial.println(2);
    delay(10);
  }
}
void processTrailerRampUp(int value) {
  if (value & buttonMaskB) {
    Serial.println(3);
    delay(10);
  }
}
void processTrailerRampDown(int value) {
  if (value & buttonMaskX) {
    Serial.println(4);
    delay(10);
  }
}

void processSpeedMode(int value) {
  // Triangle button toggles reduced speed mode with debouncing
  if ((value & buttonMaskY) && (millis() - speedModeButtonTime > speedModeDebounceDelay)) {
    reducedSpeedMode = !reducedSpeedMode; // Toggle the speed mode
    Serial.print("Speed Mode: ");
    Serial.println(reducedSpeedMode ? "Reduced (50%)" : "Normal (100%)");
    speedModeButtonTime = millis();
  }
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
  frontSteeringServo.write(180 - adjustedSteeringValue);

  Serial.print("Steering Value:");
  Serial.println(adjustedSteeringValue);
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

  processTrailerLegsUp(receivedData.buttons);
  processTrailerLegsDown(receivedData.buttons);
  processTrailerRampUp(receivedData.buttons);
  processTrailerRampDown(receivedData.buttons);

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
  if (dataUpdated) {
    processControllers();
    dataUpdated = false;
  }
  else { vTaskDelay(1); }

  // Check for connection timeout
  if (connectionActive && (millis() - lastPacketTime > CONNECTION_TIMEOUT)) {
    connectionActive = false;
    // Handle connection timeout (e.g., stop motors, reset values, etc.)
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
  }
}