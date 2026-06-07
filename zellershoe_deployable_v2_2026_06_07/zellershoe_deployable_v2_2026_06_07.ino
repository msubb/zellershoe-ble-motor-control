#include <ArduinoBLE.h>
#include <kvstore_global_api.h>

// XIAO BLE / XIAO nRF52840 deployable ZellerShoe firmware v2.
// Change to "ZellerShoe-R" before uploading to the right shoe.
const char *DEVICE_NAME = "ZellerShoe-L";

const int HEEL_PIN = A1;
const int BALL_PIN = A0;
const int MOTOR_PIN = D7;

const float VCC = 3.3;
const float R_PULLDOWN = 10000.0;

const int ADC_BITS = 10;
const int ADC_MAX_VALUE = 1023;
const int PWM_BITS = 8;
const int NUM_SAMPLES = 5;

const uint16_t FACTORY_HEEL_ON_OHMS = 330;
const uint16_t FACTORY_HEEL_OFF_OHMS = 380;
const uint16_t FACTORY_BALL_ON_OHMS = 500;
const uint16_t FACTORY_BALL_OFF_OHMS = 560;
const uint16_t FACTORY_COMBINED_ON_OHMS = 820;
const uint16_t FACTORY_COMBINED_OFF_OHMS = 860;

const uint16_t CALIBRATION_MIN_OHMS = 0;
const uint16_t CALIBRATION_MAX_OHMS = 2000;
const uint8_t FACTORY_POWER = 64; // About 25%.

const char *POWER_KEY = "/kv/zellershoe_power";
const char *CALIBRATION_KEY = "/kv/zellershoe_cal_v2";
const uint32_t POWER_MAGIC = 0x5A534850; // "ZSHP"
const uint32_t CALIBRATION_MAGIC = 0x5A534332; // "ZSC2"

const char *SERVICE_UUID = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char *COMMAND_UUID = "19b10001-e8f2-537e-4f6c-d104768a1214";
const char *STATE_UUID = "19b10002-e8f2-537e-4f6c-d104768a1214";
const char *CALIBRATION_UUID = "19b10003-e8f2-537e-4f6c-d104768a1214";
const char *SENSOR_READOUT_UUID = "19b10004-e8f2-537e-4f6c-d104768a1214";

const unsigned long SENSOR_READOUT_INTERVAL_MS = 500;

BLEService shoeService(SERVICE_UUID);
BLECharacteristic commandCharacteristic(COMMAND_UUID, BLEWrite, 20);
BLECharacteristic stateCharacteristic(STATE_UUID, BLERead | BLENotify, 3);
BLECharacteristic calibrationCharacteristic(CALIBRATION_UUID, BLERead | BLENotify, 14);
BLECharacteristic sensorReadoutCharacteristic(SENSOR_READOUT_UUID, BLERead | BLENotify, 10);

enum CommandType {
  COMMAND_SYNC_POWER = 1,
  COMMAND_START_OVERRIDE = 2,
  COMMAND_LIVE_POWER = 3,
  COMMAND_STOP_OVERRIDE = 4,
  COMMAND_SET_CUSTOM_CALIBRATION = 5,
  COMMAND_SYNC_CALIBRATION = 6,
  COMMAND_RESET_CALIBRATION = 7,
  COMMAND_START_SENSOR_READOUT = 8,
  COMMAND_STOP_SENSOR_READOUT = 9
};

enum ActivationMode {
  ACTIVATION_COUPLED = 0,
  ACTIVATION_DECOUPLED = 1
};

enum ActivationSource {
  SOURCE_NONE = 0,
  SOURCE_PRESSURE = 1,
  SOURCE_BLE_TEST = 2
};

struct SensorData {
  int rawValue;
  float voltage;
  float resistance;
};

struct SensorCalibration {
  uint16_t heelOnOhms;
  uint16_t heelOffOhms;
  uint16_t ballOnOhms;
  uint16_t ballOffOhms;
  uint16_t combinedOnOhms;
  uint16_t combinedOffOhms;
  uint8_t activationMode;
  uint8_t customEnabled;
};

struct SavedPowerRecord {
  uint32_t magic;
  uint8_t version;
  uint8_t power;
  uint8_t checksum;
  uint8_t reserved;
};

struct SavedCalibrationRecord {
  uint32_t magic;
  uint8_t version;
  SensorCalibration calibration;
  uint8_t checksum;
};

const SensorCalibration FACTORY_CALIBRATION = {
  FACTORY_HEEL_ON_OHMS,
  FACTORY_HEEL_OFF_OHMS,
  FACTORY_BALL_ON_OHMS,
  FACTORY_BALL_OFF_OHMS,
  FACTORY_COMBINED_ON_OHMS,
  FACTORY_COMBINED_OFF_OHMS,
  ACTIVATION_COUPLED,
  0
};

bool heelLoaded = false;
bool ballLoaded = false;
bool footGrounded = false;
bool overrideActive = false;
bool wasBleConnected = false;
bool savedPowerStored = false;
bool calibrationStored = false;
bool sensorReadoutActive = false;

uint8_t savedPower = FACTORY_POWER;
uint8_t activePower = 0;
uint8_t motorPower = 0;
uint8_t activationSource = SOURCE_NONE;
unsigned long lastSensorReadoutMs = 0;

SensorData lastHeel = {0, 0.0, 9999999.0};
SensorData lastBall = {0, 0.0, 9999999.0};
float lastCombinedResistance = 9999999.0;
SensorCalibration savedCalibration = FACTORY_CALIBRATION;

uint8_t checksumPower(uint8_t power) {
  return power ^ 0xA5;
}

uint8_t checksumCalibration(const SensorCalibration &calibration) {
  const uint8_t *bytes = (const uint8_t *)&calibration;
  uint8_t checksum = 0x5A;

  for (size_t i = 0; i < sizeof(SensorCalibration); i++) {
    checksum ^= bytes[i];
  }

  return checksum;
}

uint16_t readUint16LE(const uint8_t *data, int index) {
  return (uint16_t)data[index] | ((uint16_t)data[index + 1] << 8);
}

void writeUint16LE(uint8_t *data, int index, uint16_t value) {
  data[index] = value & 0xFF;
  data[index + 1] = (value >> 8) & 0xFF;
}

uint16_t clampCalibrationOhms(uint16_t value) {
  return constrain(value, CALIBRATION_MIN_OHMS, CALIBRATION_MAX_OHMS);
}

uint16_t clampReadoutOhms(float value) {
  if (value < 0.0) {
    return 0;
  }

  if (value > 65535.0) {
    return 65535;
  }

  return (uint16_t)(value + 0.5);
}

SensorCalibration normalizedCalibration(SensorCalibration calibration) {
  calibration.heelOnOhms = clampCalibrationOhms(calibration.heelOnOhms);
  calibration.heelOffOhms = max(clampCalibrationOhms(calibration.heelOffOhms), calibration.heelOnOhms);
  calibration.ballOnOhms = clampCalibrationOhms(calibration.ballOnOhms);
  calibration.ballOffOhms = max(clampCalibrationOhms(calibration.ballOffOhms), calibration.ballOnOhms);
  calibration.combinedOnOhms = clampCalibrationOhms(calibration.combinedOnOhms);
  calibration.combinedOffOhms = max(clampCalibrationOhms(calibration.combinedOffOhms), calibration.combinedOnOhms);
  calibration.activationMode = calibration.activationMode == ACTIVATION_DECOUPLED ? ACTIVATION_DECOUPLED : ACTIVATION_COUPLED;
  calibration.customEnabled = calibration.customEnabled ? 1 : 0;

  return calibration;
}

SensorCalibration effectiveCalibration() {
  if (savedCalibration.customEnabled) {
    return normalizedCalibration(savedCalibration);
  }

  return FACTORY_CALIBRATION;
}

bool isValidPowerRecord(const SavedPowerRecord &record) {
  return record.magic == POWER_MAGIC &&
         record.version == 1 &&
         record.checksum == checksumPower(record.power);
}

bool isValidCalibrationRecord(const SavedCalibrationRecord &record) {
  return record.magic == CALIBRATION_MAGIC &&
         record.version == 1 &&
         record.checksum == checksumCalibration(record.calibration);
}

uint8_t loadSavedPower() {
  SavedPowerRecord record;
  size_t actualSize = 0;
  int result = kv_get(POWER_KEY, &record, sizeof(record), &actualSize);

  if (result == 0 && actualSize == sizeof(record) && isValidPowerRecord(record)) {
    savedPowerStored = true;
    return record.power;
  }

  savedPowerStored = false;
  return FACTORY_POWER;
}

SensorCalibration loadSensorCalibration() {
  SavedCalibrationRecord record;
  size_t actualSize = 0;
  int result = kv_get(CALIBRATION_KEY, &record, sizeof(record), &actualSize);

  if (result == 0 && actualSize == sizeof(record) && isValidCalibrationRecord(record)) {
    calibrationStored = true;
    return normalizedCalibration(record.calibration);
  }

  calibrationStored = false;
  return FACTORY_CALIBRATION;
}

bool savePower(uint8_t power) {
  SavedPowerRecord record = {
    POWER_MAGIC,
    1,
    power,
    checksumPower(power),
    0
  };

  return kv_set(POWER_KEY, &record, sizeof(record), 0) == 0;
}

bool saveCalibration(const SensorCalibration &calibration) {
  SavedCalibrationRecord record = {
    CALIBRATION_MAGIC,
    1,
    normalizedCalibration(calibration),
    checksumCalibration(normalizedCalibration(calibration))
  };

  return kv_set(CALIBRATION_KEY, &record, sizeof(record), 0) == 0;
}

void publishState() {
  uint8_t state[3] = {
    savedPower,
    overrideActive ? 1 : 0,
    overrideActive ? activePower : 0
  };

  stateCharacteristic.writeValue(state, sizeof(state));
}

void publishCalibration() {
  SensorCalibration calibration = normalizedCalibration(savedCalibration);
  uint8_t data[14] = {
    calibration.customEnabled,
    calibration.activationMode,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
  };

  writeUint16LE(data, 2, calibration.heelOnOhms);
  writeUint16LE(data, 4, calibration.heelOffOhms);
  writeUint16LE(data, 6, calibration.ballOnOhms);
  writeUint16LE(data, 8, calibration.ballOffOhms);
  writeUint16LE(data, 10, calibration.combinedOnOhms);
  writeUint16LE(data, 12, calibration.combinedOffOhms);

  calibrationCharacteristic.writeValue(data, sizeof(data));
}

void publishSensorReadout() {
  uint8_t flags = 0;
  flags |= heelLoaded ? 0x01 : 0x00;
  flags |= ballLoaded ? 0x02 : 0x00;
  flags |= footGrounded ? 0x04 : 0x00;
  flags |= overrideActive ? 0x08 : 0x00;
  flags |= effectiveCalibration().customEnabled ? 0x10 : 0x00;

  uint8_t data[10] = {0};
  writeUint16LE(data, 0, clampReadoutOhms(lastHeel.resistance));
  writeUint16LE(data, 2, clampReadoutOhms(lastBall.resistance));
  writeUint16LE(data, 4, clampReadoutOhms(lastCombinedResistance));
  data[6] = flags;
  data[7] = effectiveCalibration().activationMode;
  data[8] = motorPower;
  data[9] = activationSource;

  sensorReadoutCharacteristic.writeValue(data, sizeof(data));
}

void setSavedPower(uint8_t power) {
  if (savedPowerStored && savedPower == power) {
    return;
  }

  savedPower = power;
  savedPowerStored = savePower(savedPower);
}

void setSensorCalibration(SensorCalibration calibration) {
  SensorCalibration normalized = normalizedCalibration(calibration);
  savedCalibration = normalized;
  calibrationStored = saveCalibration(savedCalibration);
  publishCalibration();
}

void setCustomCalibrationEnabled(bool enabled) {
  SensorCalibration calibration = savedCalibration;
  calibration.customEnabled = enabled ? 1 : 0;
  setSensorCalibration(calibration);
}

void resetCalibration() {
  setSensorCalibration(FACTORY_CALIBRATION);
}

void setMotorPower(uint8_t power) {
  motorPower = power;
  analogWrite(MOTOR_PIN, motorPower);
}

int smoothRead(int pin) {
  long sum = 0;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum += analogRead(pin);
    delay(1);
  }

  return (int)(sum / NUM_SAMPLES);
}

float calculateResistance(int rawValue) {
  if (rawValue <= 0) {
    return 9999999.0;
  }

  float voltage = (rawValue / (float)ADC_MAX_VALUE) * VCC;

  if (voltage <= 0.01) {
    return 9999999.0;
  }

  if (voltage >= (VCC - 0.01)) {
    return 0.0;
  }

  return R_PULLDOWN * (VCC - voltage) / voltage;
}

SensorData readFSR(int pin) {
  SensorData data;

  data.rawValue = smoothRead(pin);
  data.voltage = (data.rawValue / (float)ADC_MAX_VALUE) * VCC;
  data.resistance = calculateResistance(data.rawValue);

  return data;
}

bool updateLoadedState(
  float resistance,
  bool previousState,
  uint16_t onThreshold,
  uint16_t offThreshold
) {
  if (previousState) {
    return resistance <= offThreshold;
  }

  return resistance <= onThreshold;
}

bool updateFootGroundedState(
  bool heelState,
  bool ballState,
  float combinedResistance,
  bool previousState,
  const SensorCalibration &calibration
) {
  if (calibration.activationMode == ACTIVATION_DECOUPLED) {
    return heelState || ballState;
  }

  if (previousState) {
    return heelState && ballState && (combinedResistance <= calibration.combinedOffOhms);
  }

  return heelState && ballState && (combinedResistance <= calibration.combinedOnOhms);
}

void updateSensors() {
  SensorCalibration calibration = effectiveCalibration();
  lastHeel = readFSR(HEEL_PIN);
  lastBall = readFSR(BALL_PIN);

  heelLoaded = updateLoadedState(
    lastHeel.resistance,
    heelLoaded,
    calibration.heelOnOhms,
    calibration.heelOffOhms
  );

  ballLoaded = updateLoadedState(
    lastBall.resistance,
    ballLoaded,
    calibration.ballOnOhms,
    calibration.ballOffOhms
  );

  lastCombinedResistance = lastHeel.resistance + lastBall.resistance;
  footGrounded = updateFootGroundedState(
    heelLoaded,
    ballLoaded,
    lastCombinedResistance,
    footGrounded,
    calibration
  );
}

void applyOutput() {
  if (overrideActive) {
    setMotorPower(activePower);
    activationSource = SOURCE_BLE_TEST;
  } else if (footGrounded) {
    setMotorPower(savedPower);
    activationSource = SOURCE_PRESSURE;
  } else {
    setMotorPower(0);
    activationSource = SOURCE_NONE;
  }
}

void stopOverride() {
  overrideActive = false;
  activePower = 0;
  publishState();
}

SensorCalibration parseCalibrationCommand(const uint8_t *data) {
  SensorCalibration calibration;
  calibration.customEnabled = data[1] ? 1 : 0;
  calibration.activationMode = data[2] == ACTIVATION_DECOUPLED ? ACTIVATION_DECOUPLED : ACTIVATION_COUPLED;
  calibration.heelOnOhms = readUint16LE(data, 3);
  calibration.heelOffOhms = readUint16LE(data, 5);
  calibration.ballOnOhms = readUint16LE(data, 7);
  calibration.ballOffOhms = readUint16LE(data, 9);
  calibration.combinedOnOhms = readUint16LE(data, 11);
  calibration.combinedOffOhms = readUint16LE(data, 13);

  return normalizedCalibration(calibration);
}

void handleCommand(const uint8_t *data, int length) {
  if (length < 1) {
    return;
  }

  uint8_t command = data[0];
  uint8_t power = length >= 2 ? data[1] : 0;

  switch (command) {
    case COMMAND_SYNC_POWER:
      setSavedPower(power);
      publishState();
      break;

    case COMMAND_START_OVERRIDE:
      setSavedPower(power);
      overrideActive = true;
      activePower = power;
      publishState();
      break;

    case COMMAND_LIVE_POWER:
      if (overrideActive) {
        setSavedPower(power);
        activePower = power;
        publishState();
      }
      break;

    case COMMAND_STOP_OVERRIDE:
      stopOverride();
      break;

    case COMMAND_SET_CUSTOM_CALIBRATION:
      if (length >= 2) {
        setCustomCalibrationEnabled(data[1] != 0);
      }
      break;

    case COMMAND_SYNC_CALIBRATION:
      if (length >= 15) {
        setSensorCalibration(parseCalibrationCommand(data));
      }
      break;

    case COMMAND_RESET_CALIBRATION:
      resetCalibration();
      break;

    case COMMAND_START_SENSOR_READOUT:
      sensorReadoutActive = true;
      lastSensorReadoutMs = 0;
      break;

    case COMMAND_STOP_SENSOR_READOUT:
      sensorReadoutActive = false;
      break;

    default:
      break;
  }
}

void pollBleCommands() {
  bool bleConnected = BLE.connected();

  if (wasBleConnected && !bleConnected) {
    stopOverride();
    sensorReadoutActive = false;
  }

  wasBleConnected = bleConnected;

  if (!commandCharacteristic.written()) {
    return;
  }

  uint8_t data[20] = {0};
  int length = commandCharacteristic.valueLength();
  commandCharacteristic.readValue(data, min(length, 20));
  handleCommand(data, length);
}

void maybePublishSensorReadout() {
  if (!sensorReadoutActive || !BLE.connected()) {
    return;
  }

  unsigned long now = millis();
  if (lastSensorReadoutMs == 0 || now - lastSensorReadoutMs >= SENSOR_READOUT_INTERVAL_MS) {
    publishSensorReadout();
    lastSensorReadoutMs = now;
  }
}

void setupBle() {
  if (!BLE.begin()) {
    while (true) {
      setMotorPower(0);
      delay(1000);
    }
  }

  BLE.setLocalName(DEVICE_NAME);
  BLE.setDeviceName(DEVICE_NAME);
  BLE.setAdvertisedService(shoeService);

  shoeService.addCharacteristic(commandCharacteristic);
  shoeService.addCharacteristic(stateCharacteristic);
  shoeService.addCharacteristic(calibrationCharacteristic);
  shoeService.addCharacteristic(sensorReadoutCharacteristic);
  BLE.addService(shoeService);

  publishState();
  publishCalibration();
  publishSensorReadout();
  BLE.advertise();
}

void setup() {
  analogReadResolution(ADC_BITS);
  analogWriteResolution(PWM_BITS);

  pinMode(MOTOR_PIN, OUTPUT);
  setMotorPower(0);

  Serial.begin(115200);
  delay(300);

  savedPower = loadSavedPower();
  savedCalibration = loadSensorCalibration();
  setupBle();

  Serial.print(DEVICE_NAME);
  Serial.println(" deployable firmware v2 ready");
  Serial.println("A1 = Heel, A0 = Ball of Foot, haptic motor on D7");
}

void loop() {
  BLE.poll();
  pollBleCommands();
  updateSensors();
  applyOutput();
  maybePublishSensorReadout();

  delay(50);
}
