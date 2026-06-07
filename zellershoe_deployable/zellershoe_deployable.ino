#include <ArduinoBLE.h>
#include <kvstore_global_api.h>

// XIAO BLE / XIAO nRF52840 deployable ZellerShoe firmware.
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

const float HEEL_ON_OHMS = 330.0;
const float HEEL_OFF_OHMS = 380.0;

const float BALL_ON_OHMS = 500.0;
const float BALL_OFF_OHMS = 560.0;

const float COMBINED_ON_OHMS = 820.0;
const float COMBINED_OFF_OHMS = 860.0;

const uint8_t FACTORY_POWER = 64; // About 25%.
const char *POWER_KEY = "/kv/zellershoe_power";
const uint32_t POWER_MAGIC = 0x5A534850; // "ZSHP"

const char *SERVICE_UUID = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char *COMMAND_UUID = "19b10001-e8f2-537e-4f6c-d104768a1214";
const char *STATE_UUID = "19b10002-e8f2-537e-4f6c-d104768a1214";

BLEService shoeService(SERVICE_UUID);
BLECharacteristic commandCharacteristic(COMMAND_UUID, BLEWrite, 2);
BLECharacteristic stateCharacteristic(STATE_UUID, BLERead | BLENotify, 3);

enum CommandType {
  COMMAND_SYNC = 1,
  COMMAND_START = 2,
  COMMAND_LIVE = 3,
  COMMAND_STOP = 4
};

struct SensorData {
  int rawValue;
  float voltage;
  float resistance;
};

struct SavedPowerRecord {
  uint32_t magic;
  uint8_t version;
  uint8_t power;
  uint8_t checksum;
  uint8_t reserved;
};

bool heelLoaded = false;
bool ballLoaded = false;
bool footGrounded = false;
bool overrideActive = false;
bool wasBleConnected = false;
bool savedPowerStored = false;

uint8_t savedPower = FACTORY_POWER;
uint8_t activePower = 0;
uint8_t motorPower = 0;

uint8_t checksumPower(uint8_t power) {
  return power ^ 0xA5;
}

bool isValidRecord(const SavedPowerRecord &record) {
  return record.magic == POWER_MAGIC &&
         record.version == 1 &&
         record.checksum == checksumPower(record.power);
}

uint8_t loadSavedPower() {
  SavedPowerRecord record;
  size_t actualSize = 0;
  int result = kv_get(POWER_KEY, &record, sizeof(record), &actualSize);

  if (result == 0 && actualSize == sizeof(record) && isValidRecord(record)) {
    savedPowerStored = true;
    return record.power;
  }

  savedPowerStored = false;
  return FACTORY_POWER;
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

void publishState() {
  uint8_t state[3] = {
    savedPower,
    overrideActive ? 1 : 0,
    overrideActive ? activePower : 0
  };

  stateCharacteristic.writeValue(state, sizeof(state));
}

void setSavedPower(uint8_t power) {
  if (savedPowerStored && savedPower == power) {
    return;
  }

  savedPower = power;
  savedPowerStored = savePower(savedPower);
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
  float onThreshold,
  float offThreshold
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
  bool previousState
) {
  if (previousState) {
    return heelState && ballState && (combinedResistance <= COMBINED_OFF_OHMS);
  }

  return heelState && ballState && (combinedResistance <= COMBINED_ON_OHMS);
}

void updateSensors() {
  SensorData heel = readFSR(HEEL_PIN);
  SensorData ball = readFSR(BALL_PIN);

  heelLoaded = updateLoadedState(
    heel.resistance,
    heelLoaded,
    HEEL_ON_OHMS,
    HEEL_OFF_OHMS
  );

  ballLoaded = updateLoadedState(
    ball.resistance,
    ballLoaded,
    BALL_ON_OHMS,
    BALL_OFF_OHMS
  );

  float combinedResistance = heel.resistance + ball.resistance;
  footGrounded = updateFootGroundedState(
    heelLoaded,
    ballLoaded,
    combinedResistance,
    footGrounded
  );
}

void applyOutput() {
  if (overrideActive) {
    setMotorPower(activePower);
  } else if (footGrounded) {
    setMotorPower(savedPower);
  } else {
    setMotorPower(0);
  }
}

void stopOverride() {
  overrideActive = false;
  activePower = 0;
  publishState();
}

void handleCommand(uint8_t command, uint8_t power) {
  switch (command) {
    case COMMAND_SYNC:
      setSavedPower(power);
      publishState();
      break;

    case COMMAND_START:
      setSavedPower(power);
      overrideActive = true;
      activePower = power;
      publishState();
      break;

    case COMMAND_LIVE:
      if (overrideActive) {
        setSavedPower(power);
        activePower = power;
        publishState();
      }
      break;

    case COMMAND_STOP:
      stopOverride();
      break;

    default:
      break;
  }
}

void pollBleCommands() {
  bool bleConnected = BLE.connected();

  if (wasBleConnected && !bleConnected) {
    stopOverride();
  }

  wasBleConnected = bleConnected;

  if (!commandCharacteristic.written()) {
    return;
  }

  uint8_t data[2] = {0, 0};
  int length = commandCharacteristic.valueLength();
  commandCharacteristic.readValue(data, min(length, 2));

  if (length < 1) {
    return;
  }

  handleCommand(data[0], data[1]);
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
  BLE.addService(shoeService);

  publishState();
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
  setupBle();

  Serial.print(DEVICE_NAME);
  Serial.println(" deployable firmware ready");
  Serial.println("A1 = Heel, A0 = Ball of Foot, haptic motor on D7");
}

void loop() {
  BLE.poll();
  pollBleCommands();
  updateSensors();
  applyOutput();

  delay(50);
}
