#include <Servo.h>
#include <SoftwareSerial.h>
#include <Arduino.h>

// Serial Data input pin
#define BLUETOOTH_RX 10
// Serial Data output pin
#define BLUETOOTH_TX 11

// Bluetooth Serial Port
#define OUTPUTBLUETOOTHMONITOR 1
SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

// Default motor control pins
const byte left_front = 47;
const byte left_rear = 46;
const byte right_rear = 50;
const byte right_front = 51;

// Default ultrasonic ranging sensor pins, these pins are defined my the Shield
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

Servo left_font_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor; // create servo object to control Vex Motor Controller 29
Servo right_font_motor; // create servo object to control Vex Motor Controller 29
Servo turret_motor;

int speed_val = 130;

// GYRO
int gyroPin = A2;
int gyroVal = 0;

float gyroZeroVoltage = 0;
float gyroAngle = 0;
float gyroRate = 0;

byte serialRead = 0;

double gyroTime = 0;

HardwareSerial *SerialCom;

enum STATE
{
    STARTUP,
    RUNNING,
    FINISHED
};

// Control Loop
double kp = 0;
double ki = 0;
double kd = 0;

double ki_integral = 0;

// Run time
double startTime = 0;
double runTime = 10000;

void setup(void)
{
    BluetoothSerial.begin(115200);

    pinMode(gyroPin, INPUT);

    // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
    SerialCom = &Serial;
    SerialCom->begin(115200);
    // Starting Timer Conditions
    cli();

    TCCR1A = 0;           // Init Timer1A
    TCCR1B = 0;           // Init Timer1B
    TCCR1B |= B00000011;  // Prescaler = 64
    TCNT1 = (16000000) / ((64 * 100) + 1);        // Timer Preloading
    TIMSK1 |= B00000001;  // Enable Timer Overflow Interrupt

    delay(1000); // settling time but no really needed
}

void loop(void)
{

    static STATE machine_state = STARTUP;

    switch (machine_state)
    {
    case STARTUP:
        machine_state = initialising();
        gyroAngle = 180;
        startTime = millis();
        gyroTime = millis();
        BluetoothSerial.println(" ------ SetUp Complete -----  ");
        sei();
        break;
    case RUNNING:
        machine_state = execution();
        break;
    case FINISHED:
        machine_state = stopping();
        break;
    };
}

STATE initialising()
{
    int i;
    float sum = 0;

    // Motor Initialising
    left_font_motor.attach(left_front);   // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
    left_rear_motor.attach(left_rear);    // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
    right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
    right_font_motor.attach(right_front); // attaches the servo on pin right_front to turn Vex Motor Controller 29 On

    // Gyro Setup
    for (i = 0; i < 100; i++)
    { // read 100 values of voltage when gyro is at still, to calculate the zero-drift.
        gyroVal = analogRead(gyroPin);
        sum += gyroVal;
        delay(5);
    }

    gyroZeroVoltage = sum / 100; // average the sum as the zero drifting

    return RUNNING;
}

STATE execution()
{
    STATE return_state = RUNNING;

    double e, correction_val;

    e = 180 - gyroAngle;

    correction_val = constrain(kp * e + ki * ki_integral, -120, 120);

    ki_integral += e;


    left_font_motor.writeMicroseconds(1500 + speed_val - correction_val);
    left_rear_motor.writeMicroseconds(1500 + speed_val - correction_val);
    right_rear_motor.writeMicroseconds(1500 - speed_val - 30);
    right_font_motor.writeMicroseconds(1500 - speed_val - 30);

    /*BluetoothSerial.print(gyroAngle);
    BluetoothSerial.print(" ");
    BluetoothSerial.println("");*/

    return (millis() - startTime > runTime) ? FINISHED : RUNNING;
}

STATE stopping()
{
    left_font_motor.detach();  // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
    left_rear_motor.detach();  // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
    right_rear_motor.detach(); // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
    right_font_motor.detach(); // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

    pinMode(left_front, INPUT);
    pinMode(left_rear, INPUT);
    pinMode(right_rear, INPUT);
    pinMode(right_front, INPUT);

    delay(10);

    return FINISHED;
}

ISR(TIMER1_OVF_vect)
{
  Gyro();
}

void Gyro()
{
  // put your main code here, to run repeatedly:
  if (Serial.available()) // Check for input from terminal
  {
      serialRead = Serial.read(); // Read input
      if (serialRead == 49)       // Check for flag to execute, 49 is ascii for 1
      {
          Serial.end(); // end the serial communication to display the sensor data on monitor
      }
  }
    
  // convert the 0-1023 signal to 0-5v
  gyroRate = (analogRead(gyroPin) * 5.00) / 1023;

  // find the voltage offset the value of voltage when gyro is zero (still)
  gyroRate -= (gyroZeroVoltage / 1023 * 5.00);

  // read out voltage divided the gyro sensitivity to calculate the angular velocity
  float angularVelocity = gyroRate / 0.007;  // from Data Sheet, gyroSensitivity is 0.007 V/dps

  // if the angular velocity is less than the threshold, ignore it
  if (angularVelocity >= 1.50 || angularVelocity <= -1.50) {
    // we are running a loop in T (of T/1000 second).
     float angleChange = angularVelocity / (1000 / 10);
      gyroAngle += angleChange;
  }

  BluetoothSerial.println(millis() - gyroTime);

  gyroTime = millis();

    // keep the angle between 0-360
    if (gyroAngle < 0)
    {
        gyroAngle += 360;
    }
    else if (gyroAngle > 359)
    {
        gyroAngle -= 360;
    }

}
