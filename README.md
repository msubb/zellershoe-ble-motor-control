# ZellerShoe BLE Motor Control

Deployable XIAO nRF52840 firmware and Web Bluetooth control page for the ZellerShoe haptic motor.

## Deployable Firmware

Upload:

- `zellershoe_deployable/zellershoe_deployable.ino`

Board:

- `Seeeduino:mbed:xiaonRF52840`

Before uploading the right shoe, change:

```cpp
const char *DEVICE_NAME = "ZellerShoe-L";
```

to:

```cpp
const char *DEVICE_NAME = "ZellerShoe-R";
```

The deployable sketch keeps the v3 pressure-sensor logic, drives one PWM haptic motor on `D7`, and saves the selected power level across battery pulls using the mbed KVStore available in the Seeed XIAO nRF52840 mbed core.

## BLE Protocol

Service:

- `19b10000-e8f2-537e-4f6c-d104768a1214`

Command characteristic:

- `19b10001-e8f2-537e-4f6c-d104768a1214`
- Web page writes two bytes: `[command, power]`
- `1` = Sync saved power
- `2` = Start BLE Test Override and save power
- `3` = Live Adjustment while testing and save power
- `4` = Stop BLE Test Override

Device state characteristic:

- `19b10002-e8f2-537e-4f6c-d104768a1214`
- Web page reads three bytes: `[savedPower, overrideActive, activePower]`

## Web Control

Open `index.html` from GitHub Pages in Bluefy on iPhone, connect to each shoe, and wait for the page to read the device's saved value. The page does not save its startup value to the shoe.

- **Sync** saves the current slider value without running the motor.
- **Start** saves the current slider value and runs BLE Test Override.
- Slider changes during BLE Test Override are live and saved.
- **Stop** exits BLE Test Override; normal pressure activation resumes immediately.

## Reference Sketch

The older `ble_motor_power_test/ble_motor_power_test.ino` sketch remains as a minimal single-motor BLE PWM test.
