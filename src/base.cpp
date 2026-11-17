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
// #define CONTROLLER_XBOX     
//
// For PS4/DualShock 4 controllers:
#define CONTROLLER_PS4   
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
// Bluepad32 misc bits: SYSTEM(PS)=0x01, SELECT(Share)=0x02, START(Options)=0x04
#ifdef CONTROLLER_PS4
const ButtonMapping controllerMapping = {
    .buttonA = 2,           // Cross (X) button - typically mapped to A
    .buttonB = 1,           // Circle (O) button - typically mapped to B  
    .buttonX = 4,           // Square button - typically mapped to X
    .buttonY = 8,           // Triangle button - typically mapped to Y
    .miscForwardMask = 2,   // Share (Select) - next vehicle
    .miscBackwardMask = 4,  // Options (Start) - previous vehicle
    .miscResetMask = 1      // PS (System) - reset to index 0
};
const char* CONTROLLER_TYPE = "PS4";
#endif

// Fallback if no controller is selected
#if !defined(CONTROLLER_XBOX) && !defined(CONTROLLER_PS4)
#error "Please select a controller type by uncommenting either CONTROLLER_XBOX or CONTROLLER_PS4"
#endif

// ============================================

// Highest valid vehicle index (0 = no vehicle). Matches the README receiver indices.
#define MAX_RECEIVER_INDEX 6

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
    uint8_t controllerSlot;   // Which Bluetooth controller slot produced this state
    uint32_t selectionTime;   // millis() when receiverIndex last changed (used for arbitration)
};
// Debounce timer per controller slot so two controllers don't block each other.
int miscButtonTime[BP32_MAX_GAMEPADS] = {0};
// Previous misc-button state per slot. Used to act on button presses (edges)
// instead of a held level, so one press = one step and a sticky bit is harmless.
uint16_t prevMiscButtons[BP32_MAX_GAMEPADS] = {0};
// Which controller slot owns each vehicle (index 0..MAX_RECEIVER_INDEX); -1 = free.
// Index 0 = "no vehicle" and is always free.
int vehicleOwner[MAX_RECEIVER_INDEX + 1] = {-1, -1, -1, -1, -1, -1, -1};

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

// ---- Failsafe / link tuning ----
#define HEARTBEAT_INTERVAL_MS 20   // link keep-alive cadence (50 Hz)
#define CONTROLLER_STALE_MS   500  // pad silent this long => send "stop"
#define DISCONNECT_NEUTRAL_MS 300  // keep re-sending "stop" after a disconnect
// millis() of the last fresh report from each slot, and the deadline until which
// we force-send neutral packets for a slot (covers just-disconnected pads).
uint32_t lastControllerData[BP32_MAX_GAMEPADS] = {0};
uint32_t neutralUntil[BP32_MAX_GAMEPADS] = {0};

void dumpGamepadState(ControllerState *gamepadState) {
    Serial.printf("ID:%d | BTN:0x%04x | DPAD:0x%02x | L:%d,%d | R:%d,%d | BT:%d TH:%d | MISC:0x%02x | FWD:%d BWD:%d RST:%d | R1:%d L1:%d R2:%d L2:%d | TL:%d TR:%d | SLOT:%d\n",
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
        gamepadState->thumbR,
        gamepadState->controllerSlot
    );
}
void sendGamepad(ControllerState *gamepadState) {
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)gamepadState, sizeof(*gamepadState));
  // Uncomment the next line to print every outgoing packet (very noisy).
  // dumpGamepadState(gamepadState);
  (void)result;
}

// Send a zeroed copy of a slot's state so its vehicle stops immediately instead
// of continuing on the last-received stick values. receiverIndex/controllerSlot
// are preserved so receivers still accept (and act on) the packet.
void sendNeutral(unsigned i) {
  ControllerState neutral = gamepadStates[i];
  neutral.buttons = 0;
  neutral.dpad = 0;
  neutral.axisX = neutral.axisY = neutral.axisRX = neutral.axisRY = 0;
  neutral.brake = neutral.throttle = 0;
  neutral.miscButtons = 0;
  neutral.thumbR = neutral.thumbL = neutral.r1 = neutral.l1 = neutral.r2 = neutral.l2 = false;
  sendGamepad(&neutral);
}
// Find the next selectable vehicle index starting from `start`, stepping by
// `dir`, wrapping through 0..MAX_RECEIVER_INDEX. Index 0 (no vehicle) is always
// allowed; otherwise a vehicle is only selectable if it is free or already ours.
// Returns the index, or -1 if every vehicle is owned by someone else.
int findFreeIndex(int start, int dir, int self) {
  const int count = MAX_RECEIVER_INDEX + 1;
  for (int step = 1; step <= count; step++) {
    int idx = ((start + dir * step) % count + count) % count;
    if (idx == 0) return 0;
    if (vehicleOwner[idx] == -1 || vehicleOwner[idx] == self) return idx;
  }
  return -1;
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
        gamepadState->controllerSlot = (uint8_t)controllerIndex;
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
        // Act on newly-pressed misc buttons (rising edge) rather than the held
        // level: one press = one step, and a sticky bit can't spam us.
        uint16_t pressed = gamepadState->miscButtons & ~prevMiscButtons[controllerIndex];
        if (gamepadState->miscButtons != prevMiscButtons[controllerIndex]) {
          Serial.printf("SLOT:%d misc=0x%02x\n", controllerIndex, gamepadState->miscButtons);
        }
        prevMiscButtons[controllerIndex] = gamepadState->miscButtons;

        if (pressed && (millis() - miscButtonTime[controllerIndex]) > 200) {
          int self = (int)controllerIndex;
          int newIndex = (int)gamepadState->receiverIndex;

          if (pressed & controllerMapping.miscResetMask) {
            newIndex = 0; // PS: deselect
          } else if (pressed & controllerMapping.miscForwardMask) {
            newIndex = findFreeIndex((int)gamepadState->receiverIndex, +1, self);
          } else if (pressed & controllerMapping.miscBackwardMask) {
            newIndex = findFreeIndex((int)gamepadState->receiverIndex, -1, self);
          }

          if (newIndex < 0) {
            // Every vehicle is taken by another controller.
            Serial.printf("SLOT:%d no free vehicle to select\n", self);
          } else if ((uint32_t)newIndex != gamepadState->receiverIndex) {
            // Release the vehicle we currently own, then claim the new one.
            int oldIndex = (int)gamepadState->receiverIndex;
            if (oldIndex > 0 && vehicleOwner[oldIndex] == self) vehicleOwner[oldIndex] = -1;
            if (newIndex > 0) vehicleOwner[newIndex] = self;

            gamepadState->receiverIndex = (uint32_t)newIndex;
            // Timestamp this deliberate selection (receivers also use it as a
            // safety net if two slots ever end up on the same index).
            gamepadState->selectionTime = millis();
            Serial.printf("SLOT:%d selected vehicle index %d\n", self, newIndex);
          }

          miscButtonTime[controllerIndex] = millis();
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
      // Fail safe: keep sending "stop" to this slot's vehicle for a moment so it
      // halts now rather than waiting for its own connection timeout.
      neutralUntil[i] = millis() + DISCONNECT_NEUTRAL_MS;
      // Release any vehicle this controller owned so others can select it.
      for (int v = 1; v <= MAX_RECEIVER_INDEX; v++) {
        if (vehicleOwner[v] == i) vehicleOwner[v] = -1;
      }
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
        lastControllerData[i] = millis();   // fresh input => link is alive
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
  if (dataUpdated) {
    processControllers();   // refresh state + send immediately on new input
  }

  // Fixed-cadence heartbeat so receivers never hit their connection timeout,
  // while still failing safe if the link to a pad actually goes quiet.
  static uint32_t lastHeartbeat = 0;
  if (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeat = millis();
    for (unsigned i = 0; i < BP32_MAX_GAMEPADS; i++) {
      if ((int32_t)(neutralUntil[i] - millis()) > 0) {
        // A pad just disconnected: keep shouting "stop" for a moment so the
        // vehicle halts at once instead of waiting for its own timeout.
        sendNeutral(i);
        continue;
      }
      if (!myControllers[i] || !myControllers[i]->isConnected() || !myControllers[i]->isGamepad()) {
        continue;
      }
      if (millis() - lastControllerData[i] < CONTROLLER_STALE_MS) {
        sendGamepad(&gamepadStates[i]);   // normal: re-send the last known state
      } else {
        sendNeutral(i);                   // pad went quiet mid-drive: fail safe
      }
    }
  }

  vTaskDelay(1);
}

