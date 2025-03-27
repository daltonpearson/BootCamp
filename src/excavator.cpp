#include <ESP32Servo.h>  // by Kevin Harrington
// #include <Bluepad32.h>
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
} struct_message;
bool dataUpdated;
struct_message receivedData;
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

// ControllerPtr myControllers[BP32_MAX_GAMEPADS];
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

// void onConnectedController(ControllerPtr ctl) {
//   bool foundEmptySlot = false;
//   for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
//     if (myControllers[i] == nullptr) {
//       Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
//       // Additionally, you can get certain gamepad properties like:
//       // Model, VID, PID, BTAddr, flags, etc.
//       ControllerProperties properties = ctl->getProperties();
//       Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
//                     properties.product_id);
//       myControllers[i] = ctl;
//       foundEmptySlot = true;
//       break;
//     }
//   }
//   if (!foundEmptySlot) {
//     Serial.println("CALLBACK: Controller connected, but could not find empty slot");
//   }
// }
// void onDisconnectedController(ControllerPtr ctl) {
//   bool foundController = false;

//   for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
//     if (myControllers[i] == ctl) {
//       Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
//       myControllers[i] = nullptr;
//       foundController = true;
//       break;
//     }
//   }

//   if (!foundController) {
//     Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
//   }
// }
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


void processBoom(int axisYValue) {
  int adjustedValue = axisYValue / 2;
  if (adjustedValue > 100) {
    mcp.digitalWrite(mainBoom0, HIGH);
    mcp.digitalWrite(mainBoom1, LOW);
  } else if (adjustedValue < -100) {
    mcp.digitalWrite(mainBoom0, LOW);
    mcp.digitalWrite(mainBoom1, HIGH);
  } else {
    mcp.digitalWrite(mainBoom0, LOW);
    mcp.digitalWrite(mainBoom1, LOW);
  }
}
void processPivot(int axisYValue) {
  int adjustedValue = axisYValue / 2;
  if (adjustedValue > 100) {
    mcp.digitalWrite(pivot0, HIGH);
    mcp.digitalWrite(pivot1, LOW);
  } else if (adjustedValue < -100) {
    mcp.digitalWrite(pivot0, LOW);
    mcp.digitalWrite(pivot1, HIGH);
  } else {
    mcp.digitalWrite(pivot0, LOW);
    mcp.digitalWrite(pivot1, LOW);
  }
}
void processDipper(int axisYValue) {
  int adjustedValue = axisYValue / 2;
  if (adjustedValue > 100) {
    mcp.digitalWrite(dipper0, HIGH);
    mcp.digitalWrite(dipper1, LOW);
  } else if (adjustedValue < -100) {
    mcp.digitalWrite(dipper0, LOW);
    mcp.digitalWrite(dipper1, HIGH);
  } else {
    mcp.digitalWrite(dipper0, LOW);
    mcp.digitalWrite(dipper1, LOW);
  }
}
void processBucket(int axisYValue) {
  int adjustedValue = axisYValue / 2;
  if (adjustedValue > 100) {
    mcp.digitalWrite(tiltAttach0, HIGH);
    mcp.digitalWrite(tiltAttach1, LOW);
  } else if (adjustedValue < -100) {
    mcp.digitalWrite(tiltAttach0, LOW);
    mcp.digitalWrite(tiltAttach1, HIGH);
  } else {
    mcp.digitalWrite(tiltAttach0, LOW);
    mcp.digitalWrite(tiltAttach1, LOW);
  }
}
void processAux(int dpadValue) {
  if (dpadValue == 1) {
    mcp.digitalWrite(thumb0, HIGH);
    mcp.digitalWrite(thumb1, LOW);
  } else if (dpadValue == 2) {
    mcp.digitalWrite(thumb0, LOW);
    mcp.digitalWrite(thumb1, HIGH);
  } else {
    mcp.digitalWrite(thumb0, LOW);
    mcp.digitalWrite(thumb1, LOW);
  }
  if (dpadValue == 4) {
    mcp.digitalWrite(auxAttach0, HIGH);
    mcp.digitalWrite(auxAttach1, LOW);
  } else if (dpadValue == 8) {
    mcp.digitalWrite(auxAttach0, LOW);
    mcp.digitalWrite(auxAttach1, HIGH);
  } else {
    mcp.digitalWrite(auxAttach0, LOW);
    mcp.digitalWrite(auxAttach1, LOW);
  }
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
  if (receivedData.r1 == 1) {
    mcp.digitalWrite(rightMotor0, HIGH);
    mcp.digitalWrite(rightMotor1, LOW);
  } else if (receivedData.r2 == 1) {
    mcp.digitalWrite(rightMotor0, LOW);
    mcp.digitalWrite(rightMotor1, HIGH);
  } else if (receivedData.r1 == 0 || receivedData.r2 == 0) {
    mcp.digitalWrite(rightMotor0, LOW);
    mcp.digitalWrite(rightMotor1, LOW);
  }
  if (receivedData.l1 == 1) {
    mcp.digitalWrite(leftMotor0, HIGH);
    mcp.digitalWrite(leftMotor1, LOW);
  } else if (receivedData.l2 == 1) {
    mcp.digitalWrite(leftMotor0, LOW);
    mcp.digitalWrite(leftMotor1, HIGH);
  } else if (receivedData.l1 == 0 || receivedData.r2 == 0) {
    mcp.digitalWrite(leftMotor0, LOW);
    mcp.digitalWrite(leftMotor1, LOW);
  }

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
  }
  if (moveClawServoUp) {
    if (servoDelay == 2) {
      if (clawServoValue >= 10 && clawServoValue < 170) {
        //if using a ps3 controller that was flashed an xbox360 controller change the value "2" below to a 3-4 to make up for the slower movement.
        clawServoValue = clawServoValue + 2;
        clawServo.write(clawServoValue);
      }
      servoDelay = 0;
    }
    servoDelay++;
  }
  if (moveClawServoDown) {
    if (servoDelay == 2) {
      if (clawServoValue <= 170 && clawServoValue > 10) {
         //if using a ps3 controller that was flashed an xbox360 controller change the value "2" below to a 3-4 to make up for the slower movement.
        clawServoValue = clawServoValue - 2;
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
        auxServoValue = auxServoValue + 2;
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
void setup() {

  Serial.begin(115200);

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
  //   put your setup code here, to run once:
  // Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  // const uint8_t *addr = BP32.localBdAddress();
  // Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  // BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  // BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  // BP32.enableVirtualDevice(false);
  // You could add additional error handling here,
  // such as logging the error or attempting to recover.
  // For example, you might attempt to reset the MCP23X17
  // and retry initialization before giving up completely.
  // Then, you could gracefully exit the program or continue
  // running with limited functionality.
  WiFi.setSleep(false);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
  }
  esp_now_register_recv_cb(OnDataRecv);
}



void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  // bool dataUpdated = BP32.update();
  if (dataUpdated) {

    if (receivedData.receiverIndex == thisReceiverIndex) {
      dumpGamepadState();
      processControllers();
      dataUpdated = false;
    }
  }
  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise, the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

  //     vTaskDelay(1);
  else { vTaskDelay(1); }
}