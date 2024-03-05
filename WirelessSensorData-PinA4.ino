#include <SoftwareSerial.h>

// Serial Data input pin
#define BLUETOOTH_RX 10
// Serial Data output pin
#define BLUETOOTH_TX 11

// USB Serial Port
#define OUTPUTMONITOR 0
#define OUTPUTPLOTTER 0

// Bluetooth Serial Port
#define OUTPUTBLUETOOTHMONITOR 1

SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

double sensorData;

int delayTime = 500;

void setup() {
  // put your setup code here, to run once:
  pinMode(A4, LOW);

  Serial.begin(115200);
  BluetoothSerial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:

  sensorData = analogRead(A4);
  BluetoothSerial.println(sensorData, DEC);
  Serial.println(sensorData);

  delay(delayTime);
}
