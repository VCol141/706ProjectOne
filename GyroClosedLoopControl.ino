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
int gyroPin = A15;
int gyroVal = 0;

float gyroZeroVoltage = 0;
float gyroRate = 0;
float gyroAngle = 0;

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
double kp = 5;
double ki = 0;
double kd = 0;

double ki_integral = 0;

// Gyro Comp
float InitialVoltage;

//Kalman variables
double prev_val;
double last_var = 999;
double sensor_noise = 5;
double process_noise = 5;



void setup(void)
{
    BluetoothSerial.begin(115200);

    pinMode(gyroPin, INPUT);
    pinMode(A14, INPUT);

    // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
    SerialCom = &Serial;
    SerialCom->begin(115200);

    delay(1000); // settling time but no really needed
}

void loop(void)
{

    static STATE machine_state = STARTUP;

    switch (machine_state)
    {
    case STARTUP:
        machine_state = initialising();
        Gyro();
        gyroAngle = 180;
        break;
    case RUNNING:
        machine_state = execution();
        delay(50);
        break;
    case FINISHED:
        machine_state = stopping();
        break;
    };
}

STATE initialising()
{
    int i;
    float sum1 = 0;
    float sum2 = 0;


    // Motor Initialising
    left_font_motor.attach(left_front);   // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
    left_rear_motor.attach(left_rear);    // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
    right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
    right_font_motor.attach(right_front); // attaches the servo on pin right_front to turn Vex Motor Controller 29 On

    // Gyro Setup
    for (i = 0; i < 100; i++)
    { // read 100 values of voltage when gyro is at still, to calculate the zero-drift.
        gyroVal = analogRead(gyroPin);
        sum1 += gyroVal;
        delay(5);
    }

    for (i = 0; i < 100; i++)
    {
        InitialVoltage = analogRead(A14);
        sum2 += InitialVoltage;
        delay(5);
    }

    InitialVoltage = sum2 / 100;

    gyroZeroVoltage = sum1 / 100; // average the sum as the zero drifting

    return RUNNING;
}

STATE execution()
{
    STATE return_state = RUNNING;

    double e, correction_val;
    // delay(10);

    Gyro();

    e = 180 - gyroAngle;

    correction_val = constrain(kp * e + ki * ki_integral, -120, 120);

    ki_integral += e;

    left_font_motor.writeMicroseconds(1500 + speed_val - correction_val);
    left_rear_motor.writeMicroseconds(1500 + speed_val - correction_val);
    right_rear_motor.writeMicroseconds(1500 - speed_val);
    right_font_motor.writeMicroseconds(1500 - speed_val);

    return return_state;
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

    return FINISHED;
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

    double current_val = analogRead(gyroPin);


    // convert the 0-1023 signal to 0-5v
    gyroRate = (Kalman(current_val, prev_val) * 5.00) / 1023;

    prev_val = current_val;

    // find the voltage offset the value of voltage when gyro is zero (still)
    gyroRate -= (gyroZeroVoltage * 5.00) / 1023;

    // read out voltage divided the gyro sensitivity to calculate the angular velocity
    float angularVelocity = gyroRate / 0.007; // from Data Sheet, gyroSensitivity is 0.007 V/dps

    // if the angular velocity is less than the threshold, ignore it
    if (angularVelocity >= 1.50 || angularVelocity <= -1.50)
    {
        // we are running a loop in T (of T/1000 second).
        float angleChange = angularVelocity / (1000 / (millis() - gyroTime));
        gyroAngle += angleChange;
    }

    BluetoothSerial.print("Current Gyro Angle: ");
    BluetoothSerial.println(gyroAngle);
    BluetoothSerial.print("Gyro Rate: ");
    BluetoothSerial.println(gyroRate);

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


double Kalman(double rawdata, double prev_est){   // Kalman Filter
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;

  a_priori_est = prev_est;  
  a_priori_var = last_var1 + process_noise1; 

  kalman_gain = a_priori_var/(a_priori_var+sensor_noise1);
  a_post_est = a_priori_est + kalman_gain*(rawdata-a_priori_est);
  a_post_var = (1- kalman_gain)*a_priori_var;
  last_var1 = a_post_var;
  return a_post_est;
}


double GyroComp()
{
    double currentVoltage = analogRead(A4); 
    double voltagePrecent = currentVoltage / InitialVoltage;

    return gyroZeroVoltage * voltagePrecent;
}