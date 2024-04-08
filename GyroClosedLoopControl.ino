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
float gyroAngle = 0;

byte serialRead = 0;

float gyroTime = 0;

HardwareSerial *SerialCom;

enum STATE
{
    STARTUP,
    RUNNING,
    FINISHED
};

// Control Loop
float kp_gyro = 25;
float ki_gyro = 10;

float ki_integral_gyro = 0;

#define CONTROL_CONSTRAINT_GYRO 100

// Gyro Comp
float InitialVoltage;

//Kalman variables
float prev_val_gyro;
float last_var_gyro = 999;
float sensor_noise_gyro = 10;
float process_noise_gyro = 1;

float sonar_cm;
const unsigned int MAX_DIST = 23200;

float sonar_dist = 0;


int sonar_MA_n = 20;
float sonar_values[20];
float sonar_average;
float sonar_range = 20;

// Control Loop
float kp_sonar = 15;
float ki_sonar = 0.5;
float ki_integral_sonar = 0;

//Kalman variables sonar
float prev_val_sonar;
float last_var_sonar = 999;
float sensor_noise_sonar = 10;
float process_noise_sonar = 1;

#define CONTROL_CONSTRAINT_SONAR 150

// Control Loop Angle Turn
float kp_angle = 0;
float ki_angle = 0;
float ki_integral_angle = 0;

// Timer values
#define TIMER_FREQUENCY 250
#define TIMER_COMPENSATION_VAL 25
int timerCount = 0;

void setup(void)
{

    cli();  // Disable interrupts

    BluetoothSerial.begin(115200);  // Initiate Bluetooth serial to output data

    pinMode(gyroPin, INPUT);    // Initiate pins
    pinMode(A14, INPUT);
    pinMode(TRIG_PIN, OUTPUT);
    digitalWrite(TRIG_PIN, LOW);
    pinMode(ECHO_PIN, INPUT); // Sets the echoPin as an Input

    // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
    SerialCom = &Serial;
    SerialCom->begin(115200);

    // Timer Set up
    TCCR2A = 0;// set entire TCCR2A register to 0
    TCCR2B = 0;// same for TCCR2B
    TCNT2  = 0;//initialize counter value to 0
    
    // Set timer compare value
    OCR2A = (16*10^6) / (TIMER_FREQUENCY * 256) - 1; // = (16*10^6) / (freq * prescaler) - 1 (must be < 256 for 8 bit timer)
    // turn on CTC mode
    TCCR2A |= (1 << WGM21);
    // Set CS21 bit for 256 prescaler
    TCCR2B |= (1 << CS22) | (1 << CS21);   
    // enable timer compare interrupt
    TIMSK2 |= (1 << OCIE2A);

    sei();
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

    for (int i = 0; i < sonar_MA_n; i++)
    {
        Sonar();
        sonar_values[i] = sonar_cm;
    }

    average_array();

    sonar_dist = sonar_average;

    sonar_average = 0;

    return RUNNING;
}

STATE execution()
{
    STATE return_state = RUNNING;

    delay(10);

    return return_state;
}

void ClosedLoopTurn(float speed, float angle_val)
{
    float e, correction_val;

    e = angle_val - gyroAngle;

    correction_val = constrain(kp_angle * e + ki_angle * ki_integral_angle, -speed, speed);

    ki_integral_angle += e;

    left_font_motor.writeMicroseconds(1500 + correction_val);
    left_rear_motor.writeMicroseconds(1500 + correction_val);
    right_rear_motor.writeMicroseconds(1500 + correction_val);
    right_font_motor.writeMicroseconds(1500 + correction_val);
}

void ClosedLoopStaph(int speed_val)
{
    float e_gyro, e_sonar, correction_val_gyro, correction_val_sonar;
    
    for (int i = 0; i < sonar_MA_n; i++)
    {
        Sonar();
        sonar_values[i] = sonar_cm;
    }

    average_array();

    e_gyro = gyroAngleChange;

    e_sonar = sonar_dist - sonar_average;

    correction_val_gyro = constrain(kp_gyro * e_gyro + ki_gyro * ki_integral_gyro, -CONTROL_CONSTRAINT_GYRO, CONTROL_CONSTRAINT_GYRO);

    correction_val_sonar = constrain(kp_sonar * e_sonar + ki_sonar * ki_integral_sonar, -CONTROL_CONSTRAINT_SONAR, CONTROL_CONSTRAINT_SONAR);

    ki_integral_gyro += e_gyro;
    ki_integral_sonar += e_sonar;

    left_font_motor.writeMicroseconds(1500 + speed_val - correction_val_gyro - correction_val_sonar);
    left_rear_motor.writeMicroseconds(1500 - speed_val - correction_val_gyro - correction_val_sonar);
    right_rear_motor.writeMicroseconds(1500 - speed_val - correction_val_gyro + correction_val_sonar);
    right_font_motor.writeMicroseconds(1500 + speed_val - correction_val_gyro + correction_val_sonar);

    BluetoothSerial.print("e:                     ");
    BluetoothSerial.println(e_sonar);
    BluetoothSerial.print("correction:            ");
    BluetoothSerial.println(correction_val_sonar);
    BluetoothSerial.print("ki:                    ");
    BluetoothSerial.println(ki_integral_sonar);
    BluetoothSerial.print("current reading:       ");
    BluetoothSerial.println(sonar_cm);
    BluetoothSerial.print("Aimed reading:         ");
    BluetoothSerial.println(sonar_dist);
}

void ClosedLoopStraight(int speed_val)
{
    float e, correction_val;

    e = gyroAngleChange;

    correction_val = constrain(kp_gyro * e + ki_gyro * ki_integral_gyro, -CONTROL_CONSTRAINT_GYRO, CONTROL_CONSTRAINT_GYRO);

    ki_integral_gyro += e;

    left_font_motor.writeMicroseconds(1500 + speed_val - correction_val);
    left_rear_motor.writeMicroseconds(1500 + speed_val - correction_val);
    right_rear_motor.writeMicroseconds(1500 - speed_val - correction_val);
    right_font_motor.writeMicroseconds(1500 - speed_val - correction_val);

    BluetoothSerial.print("e:            ");
    BluetoothSerial.println(e);
    BluetoothSerial.print("correction:   ");
    BluetoothSerial.println(correction_val);
    BluetoothSerial.print("ki:           ");
    BluetoothSerial.println(ki_integral_gyro);
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

ISR(TIMER2_COMPA_vect)
{
    timerCount++;

    if (timerCount == TIMER_COMPENSATION_VAL) { Gyro(); }
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
    gyroRate = (KalmanGyro(analogRead(gyroPin)) * 5.00) / 1023;

    // find the voltage offset the value of voltage when gyro is zero (still)
    gyroRate -= (gyroZeroVoltage * 5.00) / 1023;

    // read out voltage divided the gyro sensitivity to calculate the angular velocity
    float angularVelocity = gyroRate / 0.007; // from Data Sheet, gyroSensitivity is 0.007 V/dps

    // if the angular velocity is less than the threshold, ignore it
    if (angularVelocity >= 1.50 || angularVelocity <= -1.50)
    {
        // we are running a loop in T (of T/1000 second).
        gyroAngleChange = (angularVelocity * TIMER_COMPENSATION_VAL) / (1000 * TIMER_FREQUENCY);
        gyroAngle += gyroAngleChange;
    }

    BluetoothSerial.print("Anglew Change:      ");
    BluetoothSerial.println(gyroAngleChange);
    BluetoothSerial.print("Delta T Actual:     ");
    BluetoothSerial.println(millis() - gyroTime);
    BluetoothSerial.print("Delta T theretical: ");
    BluetoothSerial.println(millis() - gyroTime);

    gyroTime = millis();

    timerCount = 0;
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

double average_array()
{
    double sum = 0;
  
    for (int i = 0; i <= sonar_MA_n; i++)
    {
        // remove obviously rubbish readings, and keep current set of readings within expected range for better accuracy
        sum += (sonar_average == 0 ? sonar_values[i] : constrain(sonar_values[i], sonar_average - sonar_range, sonar_average + sonar_range));
    }

    return (sum == 0) ? 0 : sum / sonar_MA_n;
}