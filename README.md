# ZellerShoe BLE Motor Control

Deployable XIAO nRF52840 firmware and Web Bluetooth control page for the ZellerShoe haptic motor.

## Deployable Firmware

Upload:

- `zellershoe_deployable_v2_2026_06_07/zellershoe_deployable_v2_2026_06_07.ino`

Rollback firmware:

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

The v2 deployable sketch keeps the v3 pressure-sensor logic, drives one PWM haptic motor on `D7`, and saves the selected power level across battery pulls using the mbed KVStore available in the Seeed XIAO nRF52840 mbed core.

It also adds saved advanced calibration settings:

- Factory activation defaults stay in firmware and are always available.
- Custom calibration is opt-in per shoe.
- Coupled activation is the factory behavior: heel, ball, and combined pressure must all be loaded.
- Decoupled activation is optional: heel or ball pressure can trigger activation.
- Sensor readout is off by default and only broadcasts while requested by the web page.

## BLE Protocol

Service:

- `19b10000-e8f2-537e-4f6c-d104768a1214`

Command characteristic:

- `19b10001-e8f2-537e-4f6c-d104768a1214`
- Web page writes up to 20 bytes.
- `[1, power]` = Sync saved power
- `[2, power]` = Start BLE Test Override and save power
- `[3, power]` = Live Adjustment while testing and save power
- `[4]` = Stop BLE Test Override
- `[5, enabled]` = Save Custom Calibration enabled state
- `[6, enabled, activationMode, heelOn, heelOff, ballOn, ballOff, combinedOn, combinedOff]` = Sync calibration, with threshold values encoded as little-endian `uint16`
- `[7]` = Reset calibration to firmware defaults and disable Custom Calibration
- `[8]` = Start low-rate Sensor Readout notifications
- `[9]` = Stop Sensor Readout notifications

Device state characteristic:

- `19b10002-e8f2-537e-4f6c-d104768a1214`
- Web page reads three bytes: `[savedPower, overrideActive, activePower]`

Calibration characteristic:

- `19b10003-e8f2-537e-4f6c-d104768a1214`
- Web page reads 14 bytes: `[customEnabled, activationMode, six little-endian uint16 threshold values]`

Sensor readout characteristic:

- `19b10004-e8f2-537e-4f6c-d104768a1214`
- Web page subscribes only after the user starts readout.
- Notifications are sent at 2 Hz.
- Payload is 10 bytes: heel ohms, ball ohms, combined ohms, flags, activation mode, motor power, activation source.

## Web Control

Open `index.html` from GitHub Pages in Bluefy on iPhone, connect to each shoe, and wait for the page to read the device's saved value. The page does not save its startup value to the shoe.

- **Sync** saves the current slider value without running the motor.
- **Start** on a shoe saves the current slider value and runs BLE Test Override for that shoe.
- With **Sync Shoes** enabled, either shoe's **Start** or **Stop** controls all connected shoes.
- Slider changes during BLE Test Override are live and saved.
- **Stop** exits BLE Test Override; normal pressure activation resumes immediately.
- **Advanced** is collapsed by default per shoe.
- **Use Custom Calibration** enables saved non-factory sensor thresholds.
- **Sync Calibration** saves pending advanced threshold edits.
- **Reset Calibration** immediately restores the firmware defaults.
- **Copy Calibration** copies one connected shoe's saved calibration to the other connected shoe after confirmation.
- **Sensor Readout** is collapsed and off by default; when started, the page shows live BLE sensor values and the last message age.

## Reference Sketch

The older `ble_motor_power_test/ble_motor_power_test.ino` sketch remains as a minimal single-motor BLE PWM test.
