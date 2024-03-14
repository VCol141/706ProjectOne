int sensorPin = A2;   //define the pin that gyro is connected
int sensorValue = 0;  // read out value of sensor

float gyroZeroVoltage = 0;      // the value of voltage when gyro is zero
float gyroRate = 0;      // read out value of sensor in voltage
float currentAngle = 0;  // current angle calculated by angular velocity integral on

byte serialRead = 0;  // for serial print control

double oldTime = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // this section is initialize the sensor, find the value of voltage when gyro is zero
  int i;
  float sum = 0;

  pinMode(sensorPin, INPUT);

  Serial.println("please keep the sensor still for calibration");
  Serial.println("get the gyro zero voltage");

  for (i = 0; i < 100; i++)  // read 100 values of voltage when gyro is at still, to calculate the zero-drift.
  {
    sensorValue = analogRead(sensorPin);
    sum += sensorValue;
    delay(5);
  }
  gyroZeroVoltage = sum / 100;  // average the sum as the zero drifting
}

void loop() {

  // put your main code here, to run repeatedly:
  if (Serial.available())  // Check for input from terminal
  {
    serialRead = Serial.read();  // Read input
    if (serialRead == 49)        // Check for flag to execute, 49 is ascii for 1
    {
      Serial.end();  // end the serial communication to display the sensor data on monitor
    }
  }
  // convert the 0-1023 signal to 0-5v
  gyroRate = (analogRead(sensorPin) * 5.00) / 1023;

  // find the voltage offset the value of voltage when gyro is zero (still)
  gyroRate -= (gyroZeroVoltage / 1023 * 5.00);

  // read out voltage divided the gyro sensitivity to calculate the angular velocity
  float angularVelocity = gyroRate / 0.007;  // from Data Sheet, gyroSensitivity is 0.007 V/dps

  


  // if the angular velocity is less than the threshold, ignore it
  if (angularVelocity >= 1.50 || angularVelocity <= -1.50) {
    // we are running a loop in T (of T/1000 second).
    float angleChange = angularVelocity / (1000 / (millis() - oldTime));
    currentAngle += angleChange;
  }

  oldTime = millis();
  // keep the angle between 0-360
  if (currentAngle < 0) {
    currentAngle += 360;
  } else if (currentAngle > 359) {
    currentAngle -= 360;
  }
  Serial.print(angularVelocity);
  Serial.print(" ");
  Serial.println(currentAngle);
  // control the time per loop
  //delay(10);
}
