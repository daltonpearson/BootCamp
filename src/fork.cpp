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
} struct_message;

bool dataUpdated;
bool connectionActive = false; // Tracks if connection is currently active
unsigned long lastPacketTime = 0; // Timestamp of last received packet
const unsigned long CONNECTION_TIMEOUT = 3000; // 3 seconds timeout for connection
struct_message receivedData;

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
void flashConnectionIndicator();

#define steeringServoPin 23
#define mastTiltServoPin 22
#define cabLights 32
#define auxLights 33

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

// Function to move steering servo left and right as a connection indicator
void flashConnectionIndicator() {
  // Store original position
  float originalPosition = adjustedSteeringValue;
  
  // Flash the steering servo left and right twice
  for (int i = 0; i < 2; i++) {
    // Turn left
    steeringServo.write(120);
    delay(150);
    // Turn right
    steeringServo.write(60);
    delay(150);
  }
  
  // Return to original position
  steeringServo.write(originalPosition);
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
  steeringServo.write(adjustedSteeringValue - steeringTrim);

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
  if (velocity > 1) {
    analogWrite(motorPin0, velocity);
    analogWrite(motorPin1, LOW);
  } else if (velocity < -1) {
    analogWrite(motorPin0, LOW);
    analogWrite(motorPin1, (-1 * velocity));
  } else {
    analogWrite(motorPin0, 0);
    analogWrite(motorPin1, 0);
  }
}

void processControllers() {
  processGamepad();
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
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  if (dataUpdated) {
    processControllers();
    dataUpdated = false;
  }
  else { 
    // Check if connection has timed out
    if (connectionActive && (millis() - lastPacketTime > CONNECTION_TIMEOUT)) {
      // Connection lost, reset the flag so steering will flash on reconnection
      connectionActive = false;
      // Stop motors for safety when connection is lost
      moveMotor(leftMotor0, leftMotor1, 0);
      moveMotor(rightMotor0, rightMotor1, 0);
      moveMotor(mastMotor0, mastMotor1, 0);
    }
    vTaskDelay(1); 
  }

  // Check for connection timeout
  if (connectionActive && (millis() - lastPacketTime > CONNECTION_TIMEOUT)) {
    // Connection has timed out
    connectionActive = false;
    // Stop all motors
    moveMotor(leftMotor0, leftMotor1, 0);
    moveMotor(rightMotor0, rightMotor1, 0);
    moveMotor(mastMotor0, mastMotor1, 0);
  }
}
