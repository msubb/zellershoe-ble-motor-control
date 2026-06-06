#include <ArduinoBLE.h>

const int MOTOR_PIN = D7;

// Custom BLE UUIDs. The web page uses these exact values.
BLEService motorService("19b10000-e8f2-537e-4f6c-d104768a1214");
BLECharacteristic powerCharacteristic(
  "19b10001-e8f2-537e-4f6c-d104768a1214",
  BLERead | BLEWrite,
  20
);

int motorPower = 0;

void setMotorPower(int value) {
  motorPower = constrain(value, 0, 255);
  analogWrite(MOTOR_PIN, motorPower);
}

int parsePowerValue(const unsigned char *data, int length) {
  if (length == 1) {
    return data[0];
  }

  char buffer[21];
  int copyLength = min(length, 20);
  memcpy(buffer, data, copyLength);
  buffer[copyLength] = '\0';

  return atoi(buffer);
}

void setup() {
  analogWriteResolution(8);

  pinMode(MOTOR_PIN, OUTPUT);
  setMotorPower(0);

  Serial.begin(115200);
  delay(300);

  if (!BLE.begin()) {
    while (true) {
      setMotorPower(0);
      delay(1000);
    }
  }

  BLE.setLocalName("ZellerShoe");
  BLE.setDeviceName("ZellerShoe");
  BLE.setAdvertisedService(motorService);

  motorService.addCharacteristic(powerCharacteristic);
  BLE.addService(motorService);

  unsigned char initialValue[1] = {0};
  powerCharacteristic.writeValue(initialValue, 1);

  BLE.advertise();

  Serial.println("ZellerShoe BLE motor power test");
  Serial.println("Write 0-255 to motorPower characteristic");
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected: ");
    Serial.println(central.address());

    while (central.connected()) {
      BLE.poll();

      if (powerCharacteristic.written()) {
        unsigned char data[20];
        int length = powerCharacteristic.valueLength();
        powerCharacteristic.readValue(data, length);

        setMotorPower(parsePowerValue(data, length));

        Serial.print("Motor power: ");
        Serial.println(motorPower);
      }
    }

    setMotorPower(0);
    Serial.println("Disconnected, motor off");
  }
}
