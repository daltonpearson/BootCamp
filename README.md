# BootCamp - RC Vehicle Control System
<!-- ALL-CONTRIBUTORS-BADGE:START - Do not remove or modify this section -->
[![All Contributors](https://img.shields.io/badge/all_contributors-2-orange.svg?style=flat-square)](#contributors-)
<!-- ALL-CONTRIBUTORS-BADGE:END -->

**BootCamp** is a wireless remote control system for multiple RC vehicles using ESP32 microcontrollers with ESP-NOW communication protocol. The system supports controlling different vehicle types (excavator, dump truck, semi-trailer, and forklift) from a base station, with one or more Bluetooth controllers talking to it at once.

## System Overview

BootCamp implements a **one-to-many** wireless control system where:
- **Base Station**: Receives input from up to four Bluetooth controllers and broadcasts commands via ESP-NOW
- **Vehicle Receivers**: Each vehicle has an ESP32 that receives commands and controls motors/servos
- **Controller Support**: Xbox and PS4/DualShock controllers supported
- **Multi-Vehicle**: Switch between vehicles using controller buttons
- **Multi-Operator**: Several controllers can be connected at once, each driving its own vehicle
- **Fail-Safe**: Vehicles stop themselves if the controller or base link goes quiet (see [Reliability & Failsafes](#reliability--failsafes))

## Project Structure

```
BootCamp/
├── src/
│   ├── base.cpp        # Base station (controller receiver & ESP-NOW transmitter)
│   ├── excavator.cpp   # Excavator vehicle controller
│   ├── dump.cpp        # Dump truck vehicle controller
│   ├── semi.cpp        # Semi-trailer vehicle controller
│   ├── fork.cpp        # Forklift vehicle controller
│   └── trailer.cpp     # Trailer board (serial-linked from the semi-trailer)
├── platformio.ini      # Build configurations for each vehicle
├── lib/                # Project libraries
├── include/            # Header files
└── test/               # Test files
```

## Vehicle Types & Features

BootCamp is designed for RC vehicles created by **ProfessorBoots**. For detailed vehicle designs, build instructions, and 3D printable files:
- **Website**: [professorboots.com](https://professorboots.com/)
- **GitHub**: [ProfessorBoots GitHub](https://github.com/ProfBoots)

### Supported Vehicles:
- 🚜 **Excavator** (Receiver Index: 1)
- 🏗️ **Forklift** (Receiver Index: 2)
- 🚛 **Dump Truck** (Receiver Index: 3)
- 🚚 **Semi-Trailer** (Receiver Index: 4)

### Coming Soon:
- 🏗️ **Crane** (Receiver Index: TBD) - *Not yet implemented*
- 🚜 **SkidSteer** (Receiver Index: TBD) - *Not yet implemented*

Each vehicle features realistic movement controls, working lights, and specialized functions appropriate to the vehicle type. BootCamp's wireless control system allows seamless switching between vehicles using controller buttons.

## Hardware Requirements

### Base Station
- ESP32 Development Board
- Bluetooth controller (Xbox or PS4/DualShock)

### Each Vehicle
- ESP32 Development Board
- Vehicle-specific components as designed by ProfessorBoots
- Motor drivers, servo motors, sensors (per vehicle design)
- Power supply system

**For detailed hardware specifications, wiring diagrams, and build instructions:**
- **Website**: [professorboots.com](https://professorboots.com/)
- **GitHub**: [ProfessorBoots GitHub](https://github.com/ProfBoots)

## Software Setup

### Prerequisites
- [PlatformIO](https://platformio.org/) installed
- ESP32 development environment configured

### Installation

1. **Clone the repository:**
   ```bash
   git clone <repository-url>
   cd BootCamp
   ```

2. **Configure controller type** (base station):
   Edit `src/base.cpp` and select your controller:
   ```cpp
   // For Xbox controllers:
   #define CONTROLLER_XBOX
   // #define CONTROLLER_PS4

   // For PS4 controllers:
   // #define CONTROLLER_XBOX
   #define CONTROLLER_PS4
   ```

3. **Important: Uninstall espressif32 package before building base:**
   ```bash
   pio pkg uninstall -p espressif32
   ```

4. **Build and upload base station:**
   ```bash
   pio run -e base --target upload
   ```

4. **Build and upload vehicle firmware:**
   ```bash
   # For excavator:
   pio run -e excavator --target upload

   # For dump truck:
   pio run -e dump --target upload

   # For semi:
   pio run -e semi --target upload

   # For semi-trailer:
   pio run -e trailer --target upload

   # For forklift:
   pio run -e fork --target upload
   ```

## Usage

### Controller Operations

#### **Basic Vehicle Control**
- **Left Stick**: Primary movement (forward/backward, steering)
- **Right Stick**: Secondary functions (varies by vehicle)
- **D-Pad**: Auxiliary functions
- **Shoulder Buttons**: Additional controls
- **Face Buttons**: Vehicle-specific functions

#### **Vehicle Switching**
Use the misc buttons on your controller to switch between vehicles:
- **Forward Button**: Next vehicle (increment receiver index)
- **Backward Button**: Previous vehicle (decrement receiver index)
- **Reset Button**: Return to receiver index 0

Every controller can select **any** vehicle — a pad is not tied to a particular machine, so the same
controller can drive the excavator one minute and the semi-trailer the next. The only rule is that a
vehicle has exactly one driver at a time: if the vehicle you would land on is currently held by
another controller, the selection **skips over it** to the next free vehicle. Pressing **Reset**
(PS) gives your vehicle up, which frees it for the other operator.

**Button Mappings:**
- **Xbox**: Guide button area controls
- **PS4**: Share/Options/PS button controls

#### **Multiple Controllers (Two Operators)**

The base station supports **up to 4 Bluetooth controllers at once** (Bluepad32 limit). In practice
that means two operators can drive two different vehicles at the same time, but the rules are
deliberately loose:

- **Any controller can drive any vehicle.** A controller is not bound to a particular machine — the
  same pad can take the excavator, then the dump truck, then the forklift.
- **Two operators, two vehicles, simultaneously.** Each controller has its own slot and its own
  independent receiver index, so input from one never leaks into the other's vehicle.
- **One driver per vehicle.** A vehicle can only be held by one controller at a time. If the vehicle
  you would land on is already taken, your switch press **skips past it** to the next free one, so
  you can't accidentally steal someone else's machine. If every vehicle is taken, the base prints
  `SLOT:n no free vehicle to select` and leaves you where you are.
- **Handing a vehicle over** is just a release: the current driver presses **PS** (or switches to a
  different vehicle), the vehicle is freed, and the other controller can now select it.
- **Disconnects free the vehicle** automatically, so a pad running out of battery doesn't lock a
  vehicle out of the session.
- Watch the serial monitor: each selection prints `SLOT:n selected vehicle index m`, so you can tell
  which pad took which machine.

> **Note:** Both controllers connect over Bluetooth and the commands go out over ESP-NOW. The
> ESP32 shares a single radio between BT and WiFi/ESP-NOW, which the base firmware already handles.

#### **Pairing (Important)**

Pair each controller **one at a time, with the other controller powered OFF**. Bluepad32 can hold
several already-paired controllers at once, but pairing a *second* new controller while another is
connected is unreliable (you'll see `Failed to inquiry name` / `Unknown GAP connection type` in the
serial monitor). Once both are paired, they reconnect normally - just press **PS** on each.

- **To pair:** hold **Share + PS** until the light bar double-blinks, then release.
- **To deselect:** press **PS**.
- **Do not** call `BP32.forgetBluetoothKeys()` in the base firmware. It erases the stored keys and
  forces re-pairing, which is exactly when the second controller fails to attach.

#### **Speed Modes (Dump Truck & Semi-Trailer)**

The dump truck and the semi-trailer both power up in a **restricted 50% speed mode**, which is
gentler on the drivetrain and much easier to drive indoors.

- **Triangle** toggles between **Restricted (50%)** and **Full (100%)**.
- The toggle is **edge-triggered** (one press = one change, 250 ms debounce), and the first packet
  after a controller connects only records the button state — a button that is already held, or a
  stuck bit, can't silently flip the truck into full speed.
- Restricted mode is restored every time the firmware starts, so the trucks never come up "hot".
- The semi-trailer prints the new mode to the serial monitor (`Speed Mode: Restricted (50%)` /
  `Speed Mode: Full (100%)`).
- The excavator and forklift have no speed limit mode.

#### **Connection Indicator**

When a controller is paired to a vehicle, the vehicle acknowledges the link with a short,
non-blocking indicator:

| Vehicle | Indicator |
|---|---|
| Excavator | Cab lights flash three times |
| Dump Truck | Lights flash three times, steering twitches ±8° |
| Semi-Trailer | Marker lights (LT1–LT3) flash three times, steering twitches ±8° |
| Forklift | Steering swings ±30° twice |

The indicator is armed from the ESP-NOW receive callback but advanced from `loop()` via
`serviceConnectionIndicator()`. That distinction matters: the old version used `delay(200)` per
flash inside the callback, which stalled the WiFi task and dropped packets — exactly when the link
was already marginal.

## Communication Protocol

### ESP-NOW Message Structure
```cpp
typedef struct {
    uint32_t receiverIndex;    // Vehicle identifier (0-6)
    uint16_t buttons;          // Button state bitmask
    uint8_t dpad;             // D-pad state
    int32_t axisX, axisY;     // Left stick values
    int32_t axisRX, axisRY;   // Right stick values
    uint32_t brake, throttle; // Trigger values
    uint16_t miscButtons;     // Misc button bitmask
    bool thumbR, thumbL;      // Thumb button states
    bool r1, l1, r2, l2;     // Shoulder button states
    uint8_t controllerSlot;   // Which base controller slot sent this (0-3)
    uint32_t selectionTime;   // millis() when this controller last changed vehicle
} struct_message;
```

### Receiver Indices
- **0**: No vehicle selected
- **1**: Excavator
- **2**: Forklift
- **3**: Dump Truck
- **4**: Semi-Trailer
- **5**: Crane *(Coming Soon)*
- **6**: SkidSteer *(Coming Soon)*

## Reliability & Failsafes

The radio is shared between Bluetooth (the pads) and WiFi/ESP-NOW (the vehicles), so the firmware
assumes packets will be lost now and then. Three mechanisms keep a dropout from becoming a runaway
vehicle.

**Heartbeat (base → vehicles).** The base transmits at a fixed 20 ms (50 Hz) cadence for every
connected controller, whether or not the sticks moved. Vehicles therefore never time out just
because the operator is holding a steady line, and a new input is still sent immediately when it
arrives.

**Fast link-loss detection.** Each vehicle treats **500 ms without a packet** as a lost link (it used
to be 3 s). On timeout it calls `safeStop()` once:

| Vehicle | `safeStop()` behaviour |
|---|---|
| Excavator | Releases every MCP23017 H-bridge (boom, pivot, dipper, bucket tilt, thumb, aux, drive); servos hold position |
| Dump Truck | Drive cut, steering recentred, dump-bed outputs released |
| Semi-Trailer | Drive and smoke generator cut, steering recentred, lights off, trailer aux motors told to stop (serial `7` / `10`) |
| Forklift | Drive and mast cut, steering recentred, aux output released |

**Neutral packets (base side).** If a controller stops reporting for more than 500 ms while still
paired, the base sends zeroed commands for that slot, so the vehicle stops immediately instead of
holding the last stick value. After a controller actually disconnects the base repeats those zeroed
packets for a further 300 ms, then releases the vehicle for another operator.

> **Limitation:** the trailer board has no serial-silence watchdog of its own — it stops because the
> semi-trailer sends it explicit stop commands. If the semi-trailer loses power outright its serial
> link dies with it and the trailer aux motors keep their last state, so cut the trailer's power too.

**Motor protection.** The PWM-driven H-bridges (dump truck, semi-trailer, forklift) get two
protections in `moveMotor()`: a **dead-time**, where both inputs are held LOW for one control tick
on a direction reversal so the bridge never sees an instantaneous reversal, and a **soft-start
ramp** of 16 duty steps per tick (~320 ms from standstill to full) so the truck eases away instead
of slamming. The excavator's MCP23017 outputs are on/off rather than PWM, so it gets dead-time only.
Stick values inside each vehicle's deadband are a hard stop. This rework also fixed a copy/paste bug
in the excavator, where the left drive motor's stop condition tested the right trigger (`r2`) instead
of `l2`.

## Troubleshooting

### Common Issues

**Base station build fails:**
- Make sure you ran `pio pkg uninstall -p espressif32` before building
- This is required due to a dependency conflict with the ESP32 Bluetooth library

**Base station build fails with `xtensa-esp32-elf-g++: error: CreateProcess: No such file or directory`:**
- That is a Windows command-line-length limit, not a code error. The Bluepad32 framework adds enough
  ESP-IDF include paths that the compile command for the bundled WiFi library exceeds ~32k characters.
- Workarounds: build from a shorter path, or check your own code compiles with
  `pio run -e base -t compiledb` and compile the generated `base.cpp` command through a response file
  (`xtensa-esp32-elf-g++ @args.rsp`).

**Controller not connecting:**
- Ensure correct controller type is selected in `base.cpp`
- Check Bluetooth pairing
- Verify ESP32 is in pairing mode

**Vehicle not responding:**
- Check receiver index matches vehicle
- Verify ESP-NOW initialization
- Check power supply to vehicle ESP32

**Vehicle stops on its own a moment after you let go, or when the base is switched off:**
- That is the 500 ms link failsafe doing its job (see [Reliability & Failsafes](#reliability--failsafes)), not a fault
- If it happens while you are still driving, check for 2.4 GHz interference or a failing pad battery

**Inconsistent control:**
- Check for controller drift (calibration may be needed)
- Verify signal strength between base and vehicle
- Check for interference

### Debug Information

Monitor serial output for debugging:
```bash
pio device monitor -e base
```

The base station outputs:
- Controller connection status
- Button mappings being used
- Receiver index changes
- ESP-NOW transmission status

## Development

### Adding New Vehicles

1. **Create new source file** in `src/` directory
2. **Add environment** to `platformio.ini`:
   ```ini
   [env:newvehicle]
   platform = espressif32@6.10.0
   board = esp32doit-devkit-v1
   framework = arduino
   build_src_filter = +<newvehicle.cpp>
   lib_deps =
     madhephaestus/ESP32Servo @ 3.0.6
   ```
3. **Implement ESP-NOW receiver** following existing patterns
4. **Set unique receiver index**
5. **Implement vehicle-specific control logic**

### Code Conversion Notes

The original project used Arduino `.ino` files with direct Bluetooth controller connections. This has been converted to:
- **PlatformIO C++ structure** for better organization
- **ESP-NOW communication** for multi-vehicle support
- **Centralized base station** for controller management

## Hardware Connections

### Pin Assignments

**For detailed wiring diagrams and pin assignments:**
- **Website**: [professorboots.com](https://professorboots.com/)
- **GitHub**: [ProfessorBoots GitHub](https://github.com/ProfBoots)

**Common ESP32 Pins:**
- Motor control: Various GPIO pins (vehicle-specific)
- Servo control: PWM-capable pins
- I2C (if used): GPIO 21 (SDA), GPIO 22 (SCL)
- Status LED: GPIO 2 (built-in)

**Pin assignments for each vehicle type are documented in the respective source files and ProfessorBoots' vehicle designs.**

## Safety Considerations

- **Always test in safe environment** before full operation
- **Ensure emergency stop capability**
- **Check battery levels** before operation
- **Verify control response** after any changes
- **Maintain clear line of sight** for ESP-NOW communication
- **Treat the 500 ms failsafe as a backstop, not an emergency stop** — keep a physical power cut
  within reach at all times
- **Test the failsafe deliberately**: wheels off the ground, power the controller off mid-throttle,
  and confirm the vehicle stops within about half a second
- **Don't use vehicle ownership as a safety interlock** — it keeps two operators from fighting over
  one machine, but it will not stop a vehicle on its own

## Credits

**Vehicle Designs**: All RC vehicles are designed by **ProfessorBoots**
- **Website**: [professorboots.com](https://professorboots.com/) - Plans, instructions, and resources
- **GitHub**: [ProfessorBoots GitHub](https://github.com/ProfBoots) - 3D printable files and code
- Original vehicle designs and mechanical engineering by ProfessorBoots
- BootCamp provides the wireless control system software for his vehicle designs

## License

[Add your license information here]

## Contributing

[Add contribution guidelines here]

## Support

For issues and questions:
- **BootCamp Software/Control System**: Check the troubleshooting section, review serial monitor output
- **Vehicle Hardware/Mechanical**: Visit [professorboots.com](https://professorboots.com/) or [ProfessorBoots GitHub](https://github.com/ProfBoots)
- Verify hardware connections and power supply

---

*BootCamp demonstrates advanced ESP32 capabilities including wireless communication, multi-device coordination, and real-time control systems for ProfessorBoots' RC vehicle designs.*

## Contributors ✨

Thanks go to these wonderful people ([emoji key](https://allcontributors.org/docs/en/emoji-key)):
<!-- ALL-CONTRIBUTORS-LIST:START - Do not remove or modify this section -->
<!-- prettier-ignore-start -->
<!-- markdownlint-disable -->
<table>
  <tbody>
    <tr>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/daltonpearson"><img src="https://avatars.githubusercontent.com/u/32880838?v=4?s=100" width="100px;" alt="Dalton Pearson"/><br /><sub><b>Dalton Pearson</b></sub></a><br /><a href="https://github.com/daltonpearson/BootCamp/commits?author=daltonpearson" title="Code">💻</a> <a href="#ideas-daltonpearson" title="Ideas, Planning, & Feedback">🤔</a> <a href="#projectManagement-daltonpearson" title="Project Management">📆</a> <a href="#maintenance-daltonpearson" title="Maintenance">🚧</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/PaulHam211"><img src="https://avatars.githubusercontent.com/u/7596825?v=4?s=100" width="100px;" alt="PaulHam211"/><br /><sub><b>PaulHam211</b></sub></a><br /><a href="https://github.com/daltonpearson/BootCamp/commits?author=PaulHam211" title="Code">💻</a> <a href="https://github.com/daltonpearson/BootCamp/commits?author=PaulHam211" title="Documentation">📖</a> <a href="#ideas-PaulHam211" title="Ideas, Planning, & Feedback">🤔</a></td>
    </tr>
  </tbody>
  <tfoot>
    <tr>
      <td align="center" size="13px" colspan="7">
        <img src="https://raw.githubusercontent.com/all-contributors/all-contributors-cli/1b8533af435da9854653492b1327a23a4dbd0a10/assets/logo-small.svg">
          <a href="https://all-contributors.js.org/docs/en/bot/usage">Add your contributions</a>
        </img>
      </td>
    </tr>
  </tfoot>
</table>

<!-- markdownlint-restore -->
<!-- prettier-ignore-end -->

<!-- ALL-CONTRIBUTORS-LIST:END -->


This project follows the [all-contributors](https://allcontributors.org) specification.
Contributions of any kind are welcome!
