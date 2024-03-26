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
float gyroAngleChange = 0;

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
double kp_1 = 25;
double ki_1 = 10;

double ki_integral_1 = 0;

#define CONTROL_CONSTRAINT_1 100

// Gyro Comp
float InitialVoltage;

//Kalman variables
double prev_val_gyro;
double last_var_gyro = 999;
double sensor_noise_gyro = 10;
double process_noise_gyro = 1;

double sonar_cm;
const unsigned int MAX_DIST = 23200;

double sonar_dist = 0;

// Control Loop
double kp_2 = 15;
double ki_2 = 0.5;

double ki_integral_2 = 0;

//Kalman variables sonar
double prev_val_sonar;
double last_var_sonar = 999;
double sensor_noise_sonar = 10;
double process_noise_sonar = 1;

#define CONTROL_CONSTRAINT_2 150


void setup(void)
{
    BluetoothSerial.begin(115200);

    pinMode(gyroPin, INPUT);
    pinMode(A14, INPUT);

    // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
    SerialCom = &Serial;
    SerialCom->begin(115200);

    pinMode(TRIG_PIN, OUTPUT);
    digitalWrite(TRIG_PIN, LOW);
    pinMode(ECHO_PIN, INPUT); // Sets the echoPin as an Input

    delay(1000); // settling time but no really needed
}

void loop(void)
{

    static STATE machine_state = STARTUP;

    switch (machine_state)
    {
    case STARTUP:
        machine_state = initialising();
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

    gyroZeroVoltage = sum1 / 100; // average the sum as the zero drifting

    for (i = 0; i < 100; i++)
    {
        Sonar();
        sonar_dist = sonar_cm;
        sum2 += sonar_dist;
        delay(5);
    }

    sonar_dist = sum2 / 100;

    return RUNNING;
}

STATE execution()
{
    STATE return_state = RUNNING;

    ClosedLoopStaph(speed_val);

    return return_state;
}

void ClosedLoopStaph(int speed_val)
{
    double e_1, e_2, correction_val_1, correction_val_2;

    Gyro();
    delay(10);
    Sonar();

    (abs(gyroAngleChange) > 3) ? e_1 = 0 : e_1 = gyroAngleChange;

    e_2 = sonar_dist - sonar_cm;

    correction_val_1 = constrain(kp_1 * e_1 + ki_1 * ki_integral_1, -CONTROL_CONSTRAINT_1, CONTROL_CONSTRAINT_1);

    correction_val_2 = constrain(kp_2 * e_2 + ki_2 * ki_integral_2, -CONTROL_CONSTRAINT_2, CONTROL_CONSTRAINT_2);

    ki_integral_1 += e_1;
    ki_integral_2 += e_2;


    left_font_motor.writeMicroseconds(1500 + speed_val - correction_val_1 - correction_val_2);
    left_rear_motor.writeMicroseconds(1500 - speed_val - correction_val_1 - correction_val_1);
    right_rear_motor.writeMicroseconds(1500 - speed_val - correction_val_1 + correction_val_2);
    right_font_motor.writeMicroseconds(1500 + speed_val - correction_val_1 + correction_val_2);

    BluetoothSerial.print("e:                     ");
    BluetoothSerial.println(e_2);
    BluetoothSerial.print("correction:            ");
    BluetoothSerial.println(correction_val_2);
    BluetoothSerial.print("ki:                    ");
    BluetoothSerial.println(ki_integral_2);
    BluetoothSerial.print("current reading:       ");
    BluetoothSerial.println(sonar_cm);
    BluetoothSerial.print("Aimed reading:         ");
    BluetoothSerial.println(sonar_dist);
}

void ClosedLoopStraight(int speed_val)
{
    double e, correction_val;

    Gyro();

    (abs(gyroAngleChange) > 3) ? e = 0 : e = gyroAngleChange;

    correction_val = constrain(kp_1 * e + ki_1 * ki_integral_1, -CONTROL_CONSTRAINT_1, CONTROL_CONSTRAINT_1);

    ki_integral_1 += e;

    left_font_motor.writeMicroseconds(1500 + speed_val - correction_val);
    left_rear_motor.writeMicroseconds(1500 + speed_val - correction_val);
    right_rear_motor.writeMicroseconds(1500 - speed_val - correction_val);
    right_font_motor.writeMicroseconds(1500 - speed_val - correction_val);

    BluetoothSerial.print("e:            ");
    BluetoothSerial.println(e);
    BluetoothSerial.print("correction:   ");
    BluetoothSerial.println(correction_val);
    BluetoothSerial.print("ki:           ");
    BluetoothSerial.println(ki_integral_1);
    BluetoothSerial.println(" ");
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
    gyroRate = (KalmanGyro(current_val) * 5.00) / 1023;

    // find the voltage offset the value of voltage when gyro is zero (still)
    gyroRate -= (gyroZeroVoltage * 5.00) / 1023;

    // read out voltage divided the gyro sensitivity to calculate the angular velocity
    float angularVelocity = gyroRate / 0.007; // from Data Sheet, gyroSensitivity is 0.007 V/dps

    // if the angular velocity is less than the threshold, ignore it
    if (angularVelocity >= 1.50 || angularVelocity <= -1.50)
    {
        // we are running a loop in T (of T/1000 second).
        gyroAngleChange = angularVelocity / (1000 / (millis() - gyroTime));
    }

    gyroTime = millis();
}

void Sonar()
{
    unsigned long t1, t2, pulse_width;
    float cm;
     
    // Hold the trigger pin high for at least 10 us
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Wait for pulse on echo pin
    t1 = micros();
    while ( digitalRead(ECHO_PIN) == 0 ) {
        t2 = micros();
        pulse_width = t2 - t1;
        if ( pulse_width > (MAX_DIST + 1000)) {
            SerialCom->println("HC-SR04: NOT found");
            return;
        }
    }

    // Measure how long the echo pin was held high (pulse width)
    // Note: the micros() counter will overflow after ~70 min

    t1 = micros();
    while ( digitalRead(ECHO_PIN) == 1)
    {
        t2 = micros();
        pulse_width = t2 - t1;
        if ( pulse_width > (MAX_DIST + 1000) ) {
            SerialCom->println("HC-SR04: Out of range");
            return;
        }
    }

    t2 = micros();
    pulse_width = t2 - t1;

    // Calculate distance in centimeters and inches. The constants
    // are found in the datasheet, and calculated from the assumed speed
    //of sound in air at sea level (~340 m/s).
    cm = pulse_width / 58.0;

    ( 1 ? sonar_cm = cm : sonar_cm = KalmanSonar(cm));
}

double KalmanGyro(double rawdata){   // Kalman Filter
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;

  a_priori_var = last_var_gyro + process_noise_gyro; 

  kalman_gain = a_priori_var/(a_priori_var+sensor_noise_gyro);
  a_post_est = prev_val_gyro + kalman_gain*(rawdata-prev_val_gyro);
  a_post_var = (1- kalman_gain)*a_priori_var;
  last_var_gyro = a_post_var;
  prev_val_gyro = rawdata;
  return a_post_est;
}

double KalmanSonar(double rawdata){   // Kalman Filter
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;

  a_priori_var = last_var_sonar + process_noise_sonar; 

  kalman_gain = a_priori_var/(a_priori_var+sensor_noise_sonar);
  a_post_est = prev_val_sonar + kalman_gain*(rawdata-prev_val_sonar);
  a_post_var = (1 * kalman_gain)*a_priori_var;
  last_var_sonar = a_post_var;
  prev_val_sonar = rawdata;
  return a_post_est;
}