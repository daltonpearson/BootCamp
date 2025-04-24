#include <Bluepad32.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
// Structure to hold controller state
typedef struct ControllerState {
    uint32_t receiverIndex;
    uint16_t buttons;
    uint8_t dpad;
    int32_t axisX, axisY;
    int32_t axisRX, axisRY;
    uint32_t brake, throttle;
    uint16_t miscButtons;
    bool thumbR, thumbL, r1, l1, r2, l2;
};
int miscButtonTime = 0;
uint16_t miscForwardMask = 4;
uint16_t miscBackwardMask = 2;
uint16_t miscResetMask = 8;

typedef struct CalibrationData {
    int32_t axisX, axisY, axisRX, axisRY;
    bool isCalibrated;
};
CalibrationData controllerCalibrations[BP32_MAX_GAMEPADS];
uint32_t receiverIndexes[BP32_MAX_GAMEPADS];
ControllerState gamepadStates[BP32_MAX_GAMEPADS];
// Define the MAC address of the receiver
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
bool dataSent = true;

// Variable to store controller state
// ControllerState gamepadState;

// ESP-NOW peer info
// esp_now_peer_info_t peerInfo;

// void dumpGamepad(ControllerPtr ctl) {
//     Serial.printf(
//         "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
//         "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
//         ctl->index(),        // Controller Index
//         ctl->dpad(),         // D-pad
//         ctl->buttons(),      // bitmask of pressed buttons
//         ctl->axisX(),        // (-511 - 512) left X Axis
//         ctl->axisY(),        // (-511 - 512) left Y axis
//         ctl->axisRX(),       // (-511 - 512) right X axis
//         ctl->axisRY(),       // (-511 - 512) right Y axis
//         ctl->brake(),        // (0 - 1023): brake button
//         ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
//         ctl->miscButtons(),  // bitmask of pressed "misc" buttons
//         ctl->gyroX(),        // Gyro X
//         ctl->gyroY(),        // Gyro Y
//         ctl->gyroZ(),        // Gyro Z
//         ctl->accelX(),       // Accelerometer X
//         ctl->accelY(),       // Accelerometer Y
//         ctl->accelZ(),        // Accelerometer Z
//         ctl->r1()
//     );
// }
void dumpGamepadState(ControllerState *gamepadState) {

    // typedef struct ControllerState {
    //   uint32_t receiverIndex;
    //   uint16_t buttons;
    //   uint8_t dpad;
    //   int32_t axisX, axisY;
    //   int32_t axisRX, axisRY;
    //   uint32_t brake, throttle;
    //   uint16_t miscButtons;
    //   bool thumbR, thumbL, r1, l1, r2, l2;
    // };
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d "
        "misc: 0x%02x, misc Forward: %d, misc Backward: %d, misc Reset: %d, R1: %d, L1: %d, R2: %d, L2: %d\n",
        gamepadState->receiverIndex,        // Receiver Index
        gamepadState->dpad,         // D-pad
        gamepadState->buttons,      // bitmask of pressed buttons
        gamepadState->axisX,        // (-511 - 512) left X Axis
        gamepadState->axisY,        // (-511 - 512) left Y axis
        gamepadState->axisRX,       // (-511 - 512) right X axis
        gamepadState->axisRY,       // (-511 - 512) right Y axis
        gamepadState->brake,        // (0 - 1023): brake button
        gamepadState->throttle,     // (0 - 1023): throttle (AKA gas) button
        gamepadState->miscButtons,  // bitmask of pressed "misc" buttons
        gamepadState->miscButtons & miscForwardMask,
        gamepadState->miscButtons & miscBackwardMask,
        gamepadState->miscButtons & miscResetMask,
        gamepadState->r1,
        gamepadState->l1,
        gamepadState->r2,
        gamepadState->l2

    );
}
void sendGamepad(ControllerState *gamepadState) {
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)gamepadState, sizeof(*gamepadState));
  Serial.println(sizeof(*gamepadState));
  Serial.println((char *)gamepadState);
  dumpGamepadState(gamepadState);
}
// Controller event callback
void processGamepad(GamepadPtr gp, unsigned controllerIndex) {
    CalibrationData *calibrationData = &controllerCalibrations[controllerIndex];
    uint32_t *receiverIndex = &receiverIndexes[controllerIndex];
    // if (!calibrationData->isCalibrated) {
    //     calibrationData->axisX = gp->axisX();
    //     calibrationData->axisY = gp->axisY();
    //     calibrationData->axisRX = gp->axisRX();
    //     calibrationData->axisRY = gp->axisRY();
    //     calibrationData->isCalibrated = true;
    // }
    // Calibration seems to cause more problems than it solves so disabling for now, if you have a controller with drift you can try commenting the following code and uncommenting the previous code.
    if (!calibrationData->isCalibrated) {
        calibrationData->axisX = 0;
        calibrationData->axisY = 0;
        calibrationData->axisRX = 0;
        calibrationData->axisRY = 0;
        calibrationData->isCalibrated = true;
    }
    if (gp) {
        ControllerState *gamepadState = &gamepadStates[controllerIndex];
        gamepadState->buttons = gp->buttons();
        gamepadState->dpad = gp->dpad();
        gamepadState->axisX = gp->axisX() - calibrationData->axisX;
        gamepadState->axisY = gp->axisY() - calibrationData->axisY;
        gamepadState->axisRX = gp->axisRX() - calibrationData->axisRX;
        gamepadState->axisRY = gp->axisRY() - calibrationData->axisRY;
        gamepadState->brake = gp->brake();
        gamepadState->throttle = gp->throttle();
        gamepadState->thumbR = gp->thumbR();
        gamepadState->thumbL = gp->thumbL();
        gamepadState->r1 = gp->r1();
        gamepadState->l1 = gp->l1();
        gamepadState->r2 = gp->r2();
        gamepadState->l2 = gp->l2();


        gamepadState->miscButtons = gp->miscButtons();
        if (gamepadState->miscButtons && (millis() - miscButtonTime) > 200) {
          if (gamepadState->miscButtons & miscForwardMask) {
            gamepadState->receiverIndex++;
          } else if (gamepadState->miscButtons & miscBackwardMask) {
            gamepadState->receiverIndex--;
          } else if (gamepadState->miscButtons & miscResetMask) {
            gamepadState->receiverIndex = 0;
          }

          miscButtonTime = millis();
          // Serial.println(gamepadState.receiver_index);
        }
        sendGamepad(gamepadState);

    }
}

void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}
void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // Serial.print("Send Status: ");
    // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}
void setup() {
    Serial.begin(115200);

    // Initialize Bluepad32
    // BP32.setup(&onControllerData);

    BP32.setup(&onConnectedController, &onDisconnectedController);
    // Set device as Wi-Fi station
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_send_cb(OnDataSent);

    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

}

void processControllers() {
  unsigned i = 0;
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        // dumpGamepad(myController);
        processGamepad(myController, i);
      } else {
        Serial.println("Unsupported controller");
      }
    }

    i++;
  }
  dataSent = true;
}
// void processGamepad(ControllerPtr ctl) {
//   //Throttle
//   processThrottle(ctl->axisY());
//   //Steering
//   processSteering(ctl->axisRX());
//   //DumpBed
//   processDumpBed(ctl->dpad());
//   //Aux
//   processAux(ctl->thumbR());

//   processTrimRight(ctl->r1());
//   processTrimLeft(ctl->l1());

// }
void loop() {
    // Fetch controller updates

    // bool dataUpdated = BP32.update();
    // if (dataUpdated) {
    //   processControllers();
    //   esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&controllerData, sizeof(controllerData));

    //   if (result == ESP_OK) {
    //       Serial.println("Sent successfully");
    //   } else {
    //       Serial.println(result);
    //   }
    // }
    // // The main loop must have some kind of "yield to lower priority task" event.
    // // Otherwise, the watchdog will get triggered.
    // // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // // Detailed info here:
    // // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    // //     vTaskDelay(1);
    //     // Send controller state

    // else { vTaskDelay(1); }
    // delay(100);
    // // Send controller state
    // // esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&controllerData, sizeof(controllerData));

    // // if (result == ESP_OK) {
    // //     Serial.println("Sent successfully");
    // // } else {
    // //     Serial.println("Send failed");
    // // }

    // // delay(100);  // Send updates every 100ms

    //     // Fetch controller updates
    bool dataUpdated = BP32.update();
    if (dataUpdated & dataSent) {
      dataSent = false;
      processControllers();

          // Broadcast gamepad state via ESP-NOW
      // esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&gamepadState, sizeof(gamepadState));
      // dumpGamepadState();
      // printState();
      // Serial.print("Sending GamepadState | Buttons: ");
      // Serial.print(gamepadState.buttons);
      // Serial.print(" | DPad: ");
      // Serial.print(gamepadState.dpad);
      // Serial.print(" | L.X: ");
      // Serial.print(gamepadState.axisX);
      // Serial.print(" | L.Y: ");
      // Serial.println(gamepadState.axisY);
      // Serial.print(" | MiscButtons: ");
      // Serial.print(gamepadState.miscButtons);
      // Serial.print(" | ReceiverIndex: ");
      // Serial.print(gamepadState.receiverIndex);

      // if (result == ESP_OK) {
      //     // Serial.println("Broadcast sent successfully ✅");
      // } else {
      //     Serial.println("Broadcast send failed ❌");
      // }
    } else { vTaskDelay(1); }


    // delay(100);  // Send updates every 100ms
}

// 0x02 left 10, 0x04 right 100, 0x08 middle 1000, 0x01 xbox button 1
