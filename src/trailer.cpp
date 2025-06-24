
#include <Arduino.h>
#include <ESP32Servo.h>

#define RX0 3
#define TX0 1
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
#define trailerLegServoPin 23
#define trailerRampServoPin 22

Servo trailerLegServo;
Servo trailerRampServo;

#define auxMotor1 25
#define auxMotor2 26
#define auxMotor3 32
#define auxMotor4 33

#define LT1 15
#define LT2 27
#define LT3 14

int trailerLegValue = 10;
int trailerRampValue = 10;
bool lightsOn = false;
bool auxLightsOn = false;

String receivedDataStr = "";

void setup() {
  Serial.begin(115200);

  trailerRampServo.attach(trailerRampServoPin);
  trailerRampServo.write(trailerRampValue);
  delay(500);
  trailerLegServo.attach(trailerLegServoPin);
  trailerLegServo.write(140);
  delay(50);
  trailerLegServo.write(trailerLegValue);

  pinMode(auxMotor1, OUTPUT);
  pinMode(auxMotor2, OUTPUT);
  pinMode(auxMotor3, OUTPUT);
  pinMode(auxMotor4, OUTPUT);
  pinMode(LT1, OUTPUT);
  pinMode(LT2, OUTPUT);
  pinMode(LT3, OUTPUT);

  digitalWrite(auxMotor1, LOW);
  digitalWrite(auxMotor2, LOW);
  digitalWrite(auxMotor3, LOW);
  digitalWrite(auxMotor4, LOW);
  digitalWrite(LT1, LOW);
  digitalWrite(LT2, LOW);
  digitalWrite(LT3, LOW);
   while (Serial.available() > 0) {
        Serial.read();  // Discard the unread data
    }
}

void loop() {
  if (Serial.available() > 0) {
    /*//This grabs whatever we "serial.println" on from the semi and stores it inside recievedDataStr.
    Its crucial that if you add a function for the truck to send to the trailer you use "println" and not just "print" as it reads the value up until a new line which is specfied by the "ln" in "println"
    For example in the first if statement we check to see if mtr = 1. "1" is the value we sent from the truck. If thsi statement is true it will proceed with adding 2 to the trailerlegvalue which raises the traileg servo*/
    receivedDataStr = Serial.readStringUntil('\n');
    int mtr = receivedDataStr.toInt();  //Converts what we grabed from serial to an integer which we can easily use later in if statements
    Serial.print("Received: ");
    Serial.println(mtr);
    if (mtr == 1) {
      if (trailerLegValue <= 175) {
        trailerLegValue = trailerLegValue + 4;
        trailerLegServo.write(trailerLegValue);
      }
    } else if (mtr == 2) {
      if (trailerLegValue >= 5) {
        trailerLegValue = trailerLegValue - 4;
        trailerLegServo.write(trailerLegValue);
      }
    }
    if (mtr == 3) {
      if (trailerRampValue <= 150) {
        trailerRampValue = trailerRampValue + 4;
        trailerRampServo.write(trailerRampValue);
      }
    } else if (mtr == 4) {
      if (trailerRampValue >= 5) {
        trailerRampValue = trailerRampValue - 4;
        trailerRampServo.write(trailerRampValue);
      }
    }
    if (mtr == 5) {
      digitalWrite(auxMotor1, LOW);
      digitalWrite(auxMotor2, HIGH);
    } else if (mtr == 6) {
      digitalWrite(auxMotor1, HIGH);
      digitalWrite(auxMotor2, LOW);
    } else if (mtr == 7) {
      digitalWrite(auxMotor1, LOW);
      digitalWrite(auxMotor2, LOW);
    }
    if (mtr == 8) {
      digitalWrite(auxMotor3, LOW);
      digitalWrite(auxMotor4, HIGH);
    } else if (mtr == 9) {
      digitalWrite(auxMotor3, HIGH);
      digitalWrite(auxMotor4, LOW);
    } else if (mtr == 10) {
      digitalWrite(auxMotor3, LOW);
      digitalWrite(auxMotor4, LOW);
    }
    if (mtr == 11) {
      digitalWrite(LT1, LOW);
    }
    else if (mtr == 12) {
      digitalWrite(LT1, HIGH);
    }
    if (mtr == 13) {
      digitalWrite(LT2, LOW);
    }
    else if (mtr == 14) {
      digitalWrite(LT2, HIGH);
    }
    if (mtr == 15) {
      digitalWrite(LT3, LOW);
    }
    else if (mtr == 16) {
      digitalWrite(LT3, HIGH);
    }
  }
}