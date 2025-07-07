/* ProfessorBoots
   John Cheroske 1/6/2024
   MiniSkidi 3.0

   Thank you to the following people for contributing to this sketch
   -TomVerr99 "Excellent Job organizing what was a very messy intial sketch"
   -CrabRC "I dont even know where to start, but thank you for making numerous improvemnts/suggestions
   across both mechanical designs and software."
   -Fortinbra "Always willing to provide the discord group with a good meme or two, as well as lend a helping hand
   in multiple ways."

  Some tidbits to check

  -Install the esp32 boards manager into the arduino IDE"
  Programming Electronics Academy has a good tutorial: https://youtu.be/py91SMg_TeY?si=m1OWPBPlK-QHJ2Xx"
  -Select "ESP32 Dev Module" under tools>Board>ESP32 Arduino before uploading sketch.
  -The following include statements with comments "by -----" are libraries that can be installed
  directly inside the arduino IDE under Sketch>Include Library>Manage Libraries
*/
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h> // by Kevin Harrington
//#include <ESPAsyncWebSrv.h> // by dvarrel
//#include <PS4Controller.h>
#include <iostream>
#include <sstream>
#include <vector>

uint32_t thisReceiverIndex = 5;

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




// defines
#define bucketServoPin  23
#define auxServoPin 22
#define lightPin1 18
#define lightPin2 5
#define UP 1
#define DOWN 2
#define LEFT 3
#define RIGHT 4
#define ARMUP 5
#define ARMDOWN 6
#define STOP 0

#define RIGHT_MOTOR 1
#define LEFT_MOTOR 0
#define ARM_MOTOR 2

#define FORWARD 1
#define BACKWARD -1

// Forward declarations
void flashConnectionIndicator();
void bucketTilt(int bucketServoValue);

// global constants

//extern const char* htmlHomePage PROGMEM;
//const char* ssid     = "ProfBoots MiniSkidi OG";

// global variables

Servo bucketServo;
Servo auxServo;

bool horizontalScreen;//When screen orientation is locked vertically this rotates the D-Pad controls so that forward would now be left.
bool removeArmMomentum = false;
bool light = false;
int aux_pos = 105;
int bucket_pos = 110;

struct MOTOR_PINS
{
  int pinIN1;
  int pinIN2;
};

std::vector<MOTOR_PINS> motorPins =
{
  {26, 25},  //RIGHT_MOTOR Pins (IN1, IN2)
  {33, 32},  //LEFT_MOTOR  Pins
  {21, 19}, //ARM_MOTOR pins
};

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
  int originalPosition = bucket_pos;

  // tilt the bucket servo up and down twice
  for (int i = 0; i < 2; i++) {
    // Turn left
    bucketTilt(120);
    delay(150);
    // Turn right
    bucketTilt(60);
    delay(150);
  }

  // Return to original position
  bucketTilt(originalPosition);
}

void rotateMotor(int motorNumber, int motorDirection)
{
  if (motorDirection == FORWARD)
  {
    digitalWrite(motorPins[motorNumber].pinIN1, HIGH);
    digitalWrite(motorPins[motorNumber].pinIN2, LOW);
  }
  else if (motorDirection == BACKWARD)
  {
    digitalWrite(motorPins[motorNumber].pinIN1, LOW);
    digitalWrite(motorPins[motorNumber].pinIN2, HIGH);
  }
  else
  {
    if (removeArmMomentum)
    {
      digitalWrite(motorPins[ARM_MOTOR].pinIN1, HIGH);
      digitalWrite(motorPins[ARM_MOTOR].pinIN2, LOW);
      delay(10);
      digitalWrite(motorPins[motorNumber].pinIN1, LOW);
      digitalWrite(motorPins[motorNumber].pinIN2, LOW);
      delay(5);
      digitalWrite(motorPins[ARM_MOTOR].pinIN1, HIGH);
      digitalWrite(motorPins[ARM_MOTOR].pinIN2, LOW);
      delay(10);
      removeArmMomentum = false;
    }
    digitalWrite(motorPins[motorNumber].pinIN1, LOW);
    digitalWrite(motorPins[motorNumber].pinIN2, LOW);
  }
}

void moveCar(int inputValue)
{
    switch (inputValue)
    {

      case UP:
        rotateMotor(RIGHT_MOTOR, FORWARD);
        rotateMotor(LEFT_MOTOR, FORWARD);
        Serial.printf("Moving forward\n");
        break;

      case DOWN:
        rotateMotor(RIGHT_MOTOR, BACKWARD);
        rotateMotor(LEFT_MOTOR, BACKWARD);

        Serial.printf("Moving backward\n");
        break;

      case LEFT:
        //ctl->setRumble(0x01 /* force */, 0x01 /* duration */);
        rotateMotor(RIGHT_MOTOR, BACKWARD);
        rotateMotor(LEFT_MOTOR, FORWARD);

        Serial.printf("Moving left\n");
        break;

      case RIGHT:
        //ctl->setRumble(0x01 /* force */, 0x01 /* duration */);
        rotateMotor(RIGHT_MOTOR, FORWARD);
        rotateMotor(LEFT_MOTOR, BACKWARD);
        Serial.printf("Moving right\n");
        break;

      case STOP:
        rotateMotor(ARM_MOTOR, STOP);
        rotateMotor(RIGHT_MOTOR, STOP);
        rotateMotor(LEFT_MOTOR, STOP);
        break;

      case ARMUP:
        rotateMotor(ARM_MOTOR, FORWARD);

        Serial.printf("Moving arm up\n");
        break;

      case ARMDOWN:
        rotateMotor(ARM_MOTOR, BACKWARD);
        removeArmMomentum = true;
        Serial.printf("Moving arm down\n");
        break;

      default:
        rotateMotor(ARM_MOTOR, STOP);
        rotateMotor(RIGHT_MOTOR, STOP);
        rotateMotor(LEFT_MOTOR, STOP);
        break;
    }
}

void bucketTilt(int bucketServoValue)
{
  bucketServo.write(bucketServoValue);
}
void auxControl(int auxServoValue)
{
  auxServo.write(auxServoValue);
}
void lightControl()
{
  Serial.println("Toggling lights!");
  if (!light)
  {
    digitalWrite(lightPin1, HIGH);
    digitalWrite(lightPin2, LOW);
    light = true;
  }
  else
  {
    digitalWrite(lightPin1, LOW);
    digitalWrite(lightPin2, LOW);
    light = false;
  }
}

//void handleRoot(AsyncWebServerRequest *request)
//{
//  request->send_P(200, "text/html", htmlHomePage);
//}

//void handleNotFound(AsyncWebServerRequest *request)
//{
//  request->send(404, "text/plain", "File Not Found");
//}



void setUpPinModes()
{

  for (int i = 0; i < motorPins.size(); i++)
  {
    pinMode(motorPins[i].pinIN1, OUTPUT);
    pinMode(motorPins[i].pinIN2, OUTPUT);
  }
  //moveCar(STOP);
  bucketServo.attach(bucketServoPin);
  auxServo.attach(auxServoPin);
  auxControl(aux_pos);
  bucketTilt(bucket_pos);

  pinMode(lightPin1, OUTPUT);
  pinMode(lightPin2, OUTPUT);
}


void setup(void)
{
    Serial.begin(115200);

    // Initialize connection variables
    connectionActive = false;
    lastPacketTime = 0;

    setUpPinModes();

    WiFi.setSleep(false);
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(OnDataRecv);

}

void increment_bucket(int increment)
{
  int button_delay = 1;
  bucket_pos += increment;
  if(bucket_pos < 150 && bucket_pos > 0)
  {
  bucketTilt(bucket_pos);
  //delay(button_delay);
  }
  else
  {
    //ctl->setRumble(0x01 /* force */, 0x01 /* duration */);
    bucket_pos -= increment;
  }
}

void increment_claw(int increment)
{
  aux_pos += increment;
  if(aux_pos < 110 && aux_pos > 40)
  {
    auxControl(aux_pos);
    //delay(button_delay);
  }
  else
  {
    //ctl->setRumble(0x01 /* force */, 0x01 /* duration */);
    aux_pos -= increment;
  }
}


void processGamepad() {
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...

    int minimal_control_input_bucket = 100;
    int right_x = receivedData.axisRX;       // (-511 - 512) right X axis
    int right_y = receivedData.axisRY;       // (-511 - 512) right Y axis
    if (abs(right_y)>=minimal_control_input_bucket)
    {
        int n_steps = right_y/200;
        Serial.println("Incrementing bucket ");
        Serial.println(n_steps);


        increment_bucket(-1*n_steps);
    }

    // THis codeblock moves the bucket via joysticks
    if (abs(right_x)>=minimal_control_input_bucket)
    {
      int n_steps = right_x/200; // Scale to appropriate speed
      Serial.println("Incrementing claw ");
      Serial.println(n_steps);
      increment_claw(-1*n_steps);
    }

    // This code block moves the claw via R2 and L2
    int throttle = receivedData.throttle;
    int brake = receivedData.brake;
    if (abs(throttle)>=minimal_control_input_bucket)
    {
      int n_steps = throttle/400;
      increment_claw(-1*n_steps);
    }
    if (abs(brake)>=minimal_control_input_bucket)
    {
      int n_steps = brake/700;
      increment_claw(n_steps);
    }

    if(receivedData.buttons & buttonMaskB)
    {
      increment_bucket(-1);
      return;
    }
    if(receivedData.buttons & buttonMaskA)
    {
      increment_bucket(1);
      return;
    }
    if(receivedData.buttons & buttonMaskX)
    {
      increment_claw(1);
      return;
    }
    if(receivedData.buttons & buttonMaskY)
    {
      increment_claw(-1);
      return;
    }

    if(receivedData.thumbR)
    {
      lightControl();
      delay(200);
      return;
    }

    int move_command = STOP;
    int dpad_command = receivedData.dpad;


    int left_x = receivedData.axisX;        // (-511 - 512) left X Axis
    int left_y = receivedData.axisY;        // (-511 - 512) left Y axis
    Serial.println(left_x);
    Serial.println(left_y);
    if (receivedData.dpad != 0x00)
    {
      if (receivedData.dpad == 0x04) move_command = RIGHT;
      else if (receivedData.dpad == 0x02) move_command = DOWN;
      else if (receivedData.dpad == 0x01) move_command = UP;
      else if (receivedData.dpad == 0x08) move_command = LEFT;

    }
    int minimal_control_input = 200;
    if (abs(left_x)>=minimal_control_input)
    {
      Serial.println(left_x);
    }
    if (abs(left_y)>=minimal_control_input && left_y >=0) move_command = DOWN;
    if (abs(left_y)>=minimal_control_input && left_y <=0) move_command = UP;
    if (abs(left_x)>=minimal_control_input && left_x <=0) move_command = LEFT;
    if (abs(left_x)>=minimal_control_input && left_x >=0) move_command = RIGHT;

    int l1_l2_command = receivedData.buttons;
    if(l1_l2_command == 0x0010) move_command = ARMUP;
    if(l1_l2_command == 0x0020) move_command = ARMDOWN;

    moveCar(move_command);
    // Another way to query controller data is by getting the buttons() function.
    // See how the different "dump*" functions dump the Controller info.
    //dumpGamepad(ctl);
}


void processControllers() {
    processGamepad();
}

void loop()
{

    if (dataUpdated) {
        processControllers();
        dataUpdated = false;
    }
    else { vTaskDelay(1); }

    // Check for connection timeout
    if (connectionActive && (millis() - lastPacketTime > CONNECTION_TIMEOUT)) {
        connectionActive = false;
        for (int i = 0; i < motorPins.size(); i++)
        {
            digitalWrite(motorPins[i].pinIN1, LOW);
            digitalWrite(motorPins[i].pinIN2, LOW);
        }
    }
    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    //     vTaskDelay(1);
    //delay(150);


}