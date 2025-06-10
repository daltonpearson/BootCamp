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
} struct_message;
bool dataUpdated;
struct_message receivedData;
// ControllerPtr myControllers[BP32_MAX_GAMEPADS];

#define steeringServoPin 23
#define auxServoPin 22
#define cabLights 32
#define auxLights 33

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
    if (tempReceivedData.receiverIndex == thisReceiverIndex){
      memcpy(&receivedData, &tempReceivedData, sizeof(receivedData));
      dataUpdated = true;
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



void processThrottle(int axisYValue) {
  int adjustedThrottleValue = axisYValue / 2;
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
  steeringServo.write(adjustedSteeringValue);
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
  pinMode(leftMotor0, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(rightMotor0, OUTPUT);
  pinMode(rightMotor1, OUTPUT);


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
  if (dataUpdated) {
    // dumpGamepadState();
    processControllers();
    dataUpdated = false;
  }
  else { vTaskDelay(1); }
}
