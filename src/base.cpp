#include <Bluepad32.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ============================================
// CONTROLLER CONFIGURATION
// ============================================
// INSTRUCTIONS:
// 1. Uncomment ONE of the following lines to select your controller type
// 2. Comment out the other line 
// 3. Upload the code to your ESP32
//
// For Xbox/Xbox-style controllers (original configuration):
#define CONTROLLER_XBOX     
//
// For PS4/DualShock 4 controllers:
// #define CONTROLLER_PS4   
//
// The misc buttons are used to switch between receivers:
// - Forward button: increment receiver index (next vehicle)
// - Backward button: decrement receiver index (previous vehicle)  
// - Reset button: reset receiver index to 0
// ============================================

// Button mapping structures
typedef struct ButtonMapping {
    uint16_t buttonA;
    uint16_t buttonB; 
    uint16_t buttonX;
    uint16_t buttonY;
    uint16_t miscForwardMask;
    uint16_t miscBackwardMask;
    uint16_t miscResetMask;
} ButtonMapping;

// Xbox controller button mappings (original)
#ifdef CONTROLLER_XBOX
const ButtonMapping controllerMapping = {
    .buttonA = 1,           // A button
    .buttonB = 2,           // B button  
    .buttonX = 4,           // X button
    .buttonY = 8,           // Y button
    .miscForwardMask = 4,   // Forward button mask
    .miscBackwardMask = 2,  // Backward button mask
    .miscResetMask = 8      // Reset button mask
};
const char* CONTROLLER_TYPE = "Xbox";
#endif

// PS4 controller button mappings
#ifdef CONTROLLER_PS4
const ButtonMapping controllerMapping = {
    .buttonA = 2,           // Cross (X) button - typically mapped to A
    .buttonB = 1,           // Circle (O) button - typically mapped to B  
    .buttonX = 4,           // Square button - typically mapped to X
    .buttonY = 8,           // Triangle button - typically mapped to Y
    .miscForwardMask = 4, // Share button
    .miscBackwardMask = 2,// Options button
    .miscResetMask = 1   // PS button
};
const char* CONTROLLER_TYPE = "PS4";
#endif

// Fallback if no controller is selected
#if !defined(CONTROLLER_XBOX) && !defined(CONTROLLER_PS4)
#error "Please select a controller type by uncommenting either CONTROLLER_XBOX or CONTROLLER_PS4"
#endif

// ============================================

ControllerPtr myControllers[BP32_MAX_GAMEPADS];
// Structure to hold controller state
struct ControllerState {
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

struct CalibrationData {
    int32_t axisX, axisY, axisRX, axisRY;
    bool isCalibrated;
};
CalibrationData controllerCalibrations[BP32_MAX_GAMEPADS];
uint32_t receiverIndexes[BP32_MAX_GAMEPADS];
ControllerState gamepadStates[BP32_MAX_GAMEPADS];
// Define the MAC address of the receiver
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
bool dataSent = true;

void dumpGamepadState(ControllerState *gamepadState) {
    Serial.printf("ID:%d | BTN:0x%04x | DPAD:0x%02x | L:%d,%d | R:%d,%d | BT:%d TH:%d | MISC:0x%02x | FWD:%d BWD:%d RST:%d | R1:%d L1:%d R2:%d L2:%d | TL:%d TR:%d\n",
        gamepadState->receiverIndex,
        gamepadState->buttons,
        gamepadState->dpad,
        gamepadState->axisX,
        gamepadState->axisY,
        gamepadState->axisRX,
        gamepadState->axisRY,
        gamepadState->brake,
        gamepadState->throttle,
        gamepadState->miscButtons,
        gamepadState->miscButtons & controllerMapping.miscForwardMask,
        gamepadState->miscButtons & controllerMapping.miscBackwardMask,
        gamepadState->miscButtons & controllerMapping.miscResetMask,
        gamepadState->r1,
        gamepadState->l1,
        gamepadState->r2,
        gamepadState->l2,
        gamepadState->thumbL,
        gamepadState->thumbR
    );
}
void sendGamepad(ControllerState *gamepadState) {
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)gamepadState, sizeof(*gamepadState));
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
          if (gamepadState->miscButtons & controllerMapping.miscForwardMask) {
            gamepadState->receiverIndex++;
          } else if (gamepadState->miscButtons & controllerMapping.miscBackwardMask) {
            gamepadState->receiverIndex--;
          } else if (gamepadState->miscButtons & controllerMapping.miscResetMask) {
            gamepadState->receiverIndex = 0;
          }

          miscButtonTime = millis();
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
    
    // Print controller configuration
    Serial.println("=======================================");
    Serial.printf("Base Station Initialized\n");
    Serial.printf("Controller Type: %s\n", CONTROLLER_TYPE);
    Serial.printf("Forward Mask: 0x%04x\n", controllerMapping.miscForwardMask);
    Serial.printf("Backward Mask: 0x%04x\n", controllerMapping.miscBackwardMask);
    Serial.printf("Reset Mask: 0x%04x\n", controllerMapping.miscResetMask);
    Serial.println("=======================================");

    // Initialize Bluepad32
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

void loop() {
  // Fetch controller updates
  bool dataUpdated = BP32.update();
  if (dataUpdated & dataSent) {
    dataSent = false;
    processControllers();
  } else { vTaskDelay(1); }
    // delay(100);  // Send updates every 100ms
}

