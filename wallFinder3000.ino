/***Test for open loop pathing***/
#include <Servo.h>  //Need for Servo pulse output
#include <SoftwareSerial.h>

// #define NO_READ_GYRO  //Uncomment of GYRO is not attached.
// #define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.

// Serial Data Pins
#define BLUETOOTH_RX 10
#define BLUETOOTH_TX 11

// USB Serial Port
#define OUTPUTMONITOR 0
#define OUTPUTPLOTTER 0

// Bluetooth Serial Port
#define OUTPUTBLUETOOTHMONITOR 1

// Board measurements
#define BOARD_WIDTH 925
#define BOARD_LENGTH 1990
#define WALL_LIMIT_DISTANCE 150

//Serial Set-up
HardwareSerial *SerialCom;
SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);


/*********STATE MACHINES************/
//Main state machine
enum STATE {
  INITIALISING,
  HOMING,
  RUNNING,
  STOPPED
};

//Homing variables
enum homing_state {
    ROTATE,
    APPROACHING_WALL,
    FIND_WALL,
    FACE_WALL,
    FIND_ORIENTATION,
    ALIGN_ROBOT,
    GO_HOME,
};

homing_state home_state = ROTATE;

//Turning directions 
enum turning_dir {
    CCW,
    CW
};
turning_dir turn_dir;

enum strafing_dir {
    LEFT,
    RIGHT
};

strafing_dir strafe_dir;

//Pathing variables
enum pathing_state {
    FORWARD,
    BACKWARD,
    STRAFE_RIGHT,
    STRAFE_LEFT
};

pathing_state path_state;
pathing_state last_path_state;


/*******************COMPONENT SET-UP**********************/
/***WHEEL MOTORS***/
const byte left_front = 47;
const byte left_rear = 46;
const byte right_rear = 50;
const byte right_front = 51;

Servo left_font_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29

int speed_val = 200;
int speed_change;


/***IRS***/
//IR Equation Variables - MAY NEED TO UPDATE
double MR1coeff = 12.000;
double MR1power = -0.833;

double MR2coeff = 12.887;
double MR2power = -0.889;

double LR1coeff = 50.4;
double LR1power = -1.016;

double LR3coeff = 911.866;
double LR3power = -1.508;

//IR Sensor working variables
double MR1mm, MR2mm, LR1mm, LR3mm;
double MR1mm_reading, MR2mm_reading, LR1mm_reading, LR3mm_reading;
int array_index = 0;
int iterations = 20;
double MR1arr[20], MR2arr[20], LR1arr[20], LR3arr[20];

//Kalman variables
double MR1var, MR2var, LR1var, LR3var;
double sensor_noise = 1;
double process_noise = 10;


/***ULTRASONIC***/
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;
const unsigned int MAX_DIST = 23200;

double ultraArray[20];
double sonar_average = 0;
double sonar_threshold = 1;
int wall_settled = 0;
double wall = 0;

//Sonar values
double sonar_cm;
float straight_time = 0;
float ki_integral_sonar = 0;
float sonar_dist = 0;
float ki_integral_angle = 0;

//Sonar Kalman
float process_noise_sonar = 10;
float sensor_noise_sonar = 1;
float sonar_variance = 0;


/***GYRO***/
//Gyro turn variables
float gyro_aim;
int aimup = 0;
// GYRO
int gyroPin = A15;
int gyroVal = 0;

float gyroZeroVoltage = 0;
float gyroRate = 0;
float gyroAngleChange = 0;
float gyroAngle = 0;
float error = 0;
float ki_integral_gyro = 0;
float gyroTime = 0;

//Kalman variables
float prev_val_gyro;
float last_var_gyro = 999;
float sensor_noise_gyro = 10;
float process_noise_gyro = 1;

/***SERVO***/
Servo turret_motor;

/*******************MAIN SET-UP**********************/
void setup(void)
{
  BluetoothSerial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  //Initialise the ultrasonic motor and set to 0 position
  turret_motor.attach(8);
  turret_motor.write(90);

  //Initialise sensor pins
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  
  //Initilaise Gyro Pins
  pinMode(gyroPin, INPUT);
  pinMode(A14, INPUT);
  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  SerialCom = &Serial;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code_25/01/2018");
  delay(1000);
  SerialCom->println("Setup....");

  pinMode(ECHO_PIN, INPUT); // Sets the echoPin as an Input
  delay(1000); //settling time but noT really needed

  path_state = FORWARD;
  last_path_state = FORWARD;
}

/*******************SUPER LOOP**********************/
void loop(void) //main loop
{
  static STATE machine_state = INITIALISING;
  //Finite-state machine Code
  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      break;
    case HOMING:
      machine_state = homing();
      break;
    case RUNNING: //Lipo Battery Volage OK
      machine_state =  running();
      break;
    case STOPPED: //Stop of Lipo Battery voltage is too low, to protect Battery
      machine_state = stopped();
      break;
  };
  delay(10);
}

/*******************INITIALISING**********************/
STATE initialising() {
  BluetoothSerial.println("INITIALISING....");
  
  enable_motors();

  float sum1 = 0;
  // Gyro Setup
  prev_val_gyro = analogRead(gyroPin);
  for (int i = 0; i < 100; i++)
  { // read 100 values of voltage when gyro is at still, to calculate the zero-drift.
      gyroVal = KalmanGyro(analogRead(gyroPin));
      sum1 += gyroVal;
      delay(5);
  }
  gyroZeroVoltage = sum1 / 100; // average the sum as the zero drifting
  
  intialise_sensors();
  return HOMING;
}

/*******************HOMING**********************/
STATE homing(){
  //Check if battery voltage is ok before proceeding
  #ifndef NO_BATTERY_V_OK
      if (!is_battery_voltage_OK()) return STOPPED;
  #endif

  //Run Gyro and initialise variables
  Gyro();
  double sonar_value;

  switch (home_state){
    case ROTATE: //Take initial average and  start turning in arbituary direction
      BluetoothSerial.println("Starting Rotation");
      wall = measure_sonar();
      ccw();
      home_state = APPROACHING_WALL;
      BluetoothSerial.println("Looking for max");
      break;
    case APPROACHING_WALL: //Detect when the robot is approaching a wall
      sonar_value = measure_sonar();
      if (sonar_value <= wall - sonar_threshold){ //If maximum surpassed/is already decreasing, start looking for wall
        BluetoothSerial.println("looking for min");
        home_state = FIND_WALL;
      }
      else if(wall < sonar_value){ //New maxmum detected, save this value
        wall = sonar_value;
      }
      break;
    case FIND_WALL: //Look for local min point
      sonar_value = measure_sonar();
      if (wall > sonar_value){ //New minimum found, log angle and min distance
        gyro_aim = gyroAngle;
        wall = sonar_value;
      }
      else if(sonar_value >= wall+sonar_threshold){ //Minimum surpassed, turn towards given location
        stop();
        delay(1000); //allow motors to power off before completely switching the direction
        gyroAngle = 0;
        BluetoothSerial.println("trying to face wall");
        BluetoothSerial.print("AIMING FOR ANGLE: ");
        BluetoothSerial.println(gyro_aim);
        home_state = FACE_WALL;
      }
      break;
    case FACE_WALL:
      BluetoothSerial.print("CURRENT ANGLE: ");
      BluetoothSerial.println(gyroAngle);
      float sonar_error;
      sonar_error = ClosedLoopWallTurn(200, wall, -gyro_aim); //Aim about 8 degrees back from the actual measured angle
      if (abs(sonar_error) <= 1){
        wall_settled++;
        if (wall_settled == 10){
          stop();
          delay(2000);
          BluetoothSerial.println("TURNING STOPPED");
          gyroAngle = 0;
          home_state = FIND_ORIENTATION;
        }
      }
      else{
        wall_settled = 0;
      }
      break;
    case FIND_ORIENTATION:
      delay(1000);
      //Find where the other walls are to figure out where robot is
      break;
    case ALIGN_ROBOT:
      //Turn robot to correct orientaion if incorrect
      break;
    case GO_HOME:
      //Robot uses gathered information to move to the corner of the field
      //So strafe to relevant wall and move to back wall
      //Once complete move to run
      break;
  }
  return HOMING;
}

/*******************RUNNING**********************/
STATE running() {

  #ifndef NO_READ_GYRO
      Gyro();
  #endif

  #ifndef NO_HC-SR04
    HC_SR04_range();
    open_loop_path(sonar_cm);
  #endif

  #ifndef NO_BATTERY_V_OK
      if (!is_battery_voltage_OK()) return STOPPED;
  #endif

  return RUNNING;
}

/*******************STOPPED**********************/
//Stop of Lipo Battery voltage is too low, to protect Battery
STATE stopped() {
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  int Lipo_level_cal;
  disable_motors();
  slow_flash_LED_builtin();

  if (millis() - previous_millis > 500) { //print massage every 500ms
    previous_millis = millis();
    SerialCom->println("STOPPED---------");


#ifndef NO_BATTERY_V_OK
    //500ms timed if statement to check lipo and output speed settings
    if (is_battery_voltage_OK()) {
      SerialCom->print("Lipo OK waiting of voltage Counter 10 < ");
      SerialCom->println(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10) { //Making sure lipo voltage is stable
        counter_lipo_voltage_ok = 0;
        enable_motors();
        SerialCom->println("Lipo OK returning to RUN STATE");
        return RUNNING;
      }
    } else
    {
      counter_lipo_voltage_ok = 0;
    }
#endif
  }
  return STOPPED;
}


/*******************READ SONAR FUNCTION**********************/
#ifndef NO_HC-SR04
void HC_SR04_range()
{
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
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
      BluetoothSerial.print("HC-SR04: NOT found");
      return;
    }
  }
  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while (digitalRead(ECHO_PIN) == 1) //Currently echo pin is not staying high for the true amount (likely due to the robot moving between the pulses being sent)
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
  cm = pulse_width / 58.0;  //May need to be divided by a different value when the robot is rotating??

  // Print out results
  if ( pulse_width > MAX_DIST ) {
    SerialCom->println("HC-SR04: Out of range");
  } else {
    BluetoothSerial.print("Last sonar reading:");
    BluetoothSerial.println(cm);
    BluetoothSerial.println("");
  }
  sonar_cm = cm;
  
}
#endif


double measure_sonar(){
  HC_SR04_range();
  return (KalmanSonar(sonar_cm));
  // ultraArray[array_index] = sonar_cm;
  // sonar_average = average_array(ultraArray, sonar_average);
  // array_index++;
  // if (array_index == 20){
  //   array_index = 0;
  // }
  
  // BluetoothSerial.print("Sonar Average:");
  // BluetoothSerial.println(sonar_average);
  // return sonar_average;
}


double KalmanSonar(double rawdata){   // Kalman Filter
  if (rawdata >= sonar_average + 20 || rawdata <= sonar_average - 20 || rawdata < 20){ //If the value is absolutely outrageous, ignore it and use the last recorded value
    return sonar_average;
  }
  else{
    double a_post_est, a_priori_var, a_post_var, kalman_gain;

    a_priori_var = sonar_variance + process_noise_sonar; 

    kalman_gain = a_priori_var/(a_priori_var+sensor_noise_sonar);
    a_post_est = sonar_average + kalman_gain*(rawdata-sonar_average);
    sonar_variance = (1 * kalman_gain)*a_priori_var;
    sonar_average = rawdata;
    return a_post_est;
  }   
}


/*******************SONAR/IR INITIALISATION**********************/
void intialise_sensors(){
double MR1sum, MR2sum, LR1sum, LR3sum;

  for (int i = 0; i<=iterations; i++){
    read_IR_sensors();
    MR1arr[i] = MR1mm_reading;
    MR2arr[i] = MR2mm_reading;
    LR1arr[i] = LR1mm_reading;
    LR3arr[i] = LR3mm_reading;
    HC_SR04_range();
    ultraArray[i] = sonar_cm;
    delay(50);
  }
  MR1mm = average_array(MR1arr, 0);
  MR2mm = average_array(MR2arr, 0);
  LR1mm = average_array(LR1arr, 0);
  LR3mm = average_array(LR3arr, 0);
  sonar_average = average_array(ultraArray, 0);

  //If using Kalman to filter, Set IR variance to 0
  // MR1var = 0;
  // MR2var = 0;
  // LR1var = 0;
  // LR3var = 0;
}


double average_array(double* input_array, double last_average){
  double sum = 0;
  int count = 0;
  
  for (int i = 0; i<= iterations;i++){
    // remove obviously rubbish readings, and keep current set of readings within expected range for better accuracy
    if ((input_array[i] > last_average-50 && input_array[i] <last_average+50) || (last_average == 0 && (input_array[i] > 0 && input_array[i] <1000))){ 
      sum += input_array[i];
      count++;
    }
  }
  //If no valid values were read, set the average to 0
  if (sum == 0){
    return 0;
  }
  else{
    return sum/count;
  }
}


/*******************IR FUNCTIONS**********************/
double read_IR(double coefficient, double power, double sensor_reading){
  double sensor_mm;
  // BluetoothSerial.print("SENSOR READING:");
  // BluetoothSerial.println(sensor_reading);
  sensor_mm = coefficient *1000*(pow(sensor_reading, power));
  return sensor_mm;
}

void read_IR_sensors(){
  // BluetoothSerial.println("READ SENSOR START");
  MR1mm_reading = read_IR(MR1coeff, MR1power, analogRead(A4));
  MR2mm_reading = read_IR(MR2coeff, MR2power, analogRead(A6));
  LR1mm_reading = read_IR(LR1coeff, LR1power, analogRead(A5));
  LR3mm_reading = read_IR(LR3coeff, LR3power, analogRead(A7));
}





/*******************GYRO FUNCTIONS**********************/
void Gyro()
{
    // convert the 0-1023 signal to 0-5v
    gyroRate = (KalmanGyro(analogRead(gyroPin)) * 5.00) / 1023;

    // find the voltage offset the value of voltage when gyro is zero (still)
    gyroRate -= (gyroZeroVoltage * 5.00) / 1023;

    // read out voltage divided the gyro sensitivity to calculate the angular velocity
    float angularVelocity = gyroRate / 0.007; // from Data Sheet, gyroSensitivity is 0.007 V/dps

    // if the angular velocity is less than the threshold, ignore it
    if (angularVelocity >= 1.50 || angularVelocity <= -1.5)
    {
        gyroAngleChange = millis()-gyroTime;
        gyroAngleChange = 1000 / gyroAngleChange;
        gyroAngleChange = angularVelocity / gyroAngleChange;
        gyroAngle += gyroAngleChange;
    }

    gyroTime = millis();
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



/*******************OPEN LOOP PATHING (HISTORIC)**********************/
void open_loop_path(double sonar_cm)
{
  switch (path_state){
    case FORWARD:
      if(sonar_cm < 15){
          stop();
          delay(500);
          path_state = STRAFE_RIGHT;
      }else{
          forward();
      }
      last_path_state = FORWARD;
      break;

    case BACKWARD:
      if(sonar_cm > 106.9){
          stop();
          delay(500);
          path_state = (strafe_dir == LEFT) ? STRAFE_RIGHT: STRAFE_LEFT;
      }else{
          reverse();
      }
      last_path_state = BACKWARD;
      break;

    case STRAFE_RIGHT:
      strafe_right();
      delay(2000);          //adjust for desired strafing distance
      stop();
      delay(500);
      path_state = (last_path_state == FORWARD) ? BACKWARD : FORWARD;
      break;

    case STRAFE_LEFT:
      strafe_left();
      delay(2000);          //adjust for desired strafing distance
      stop();
      path_state = (last_path_state == FORWARD ? BACKWARD : FORWARD);
      break;
  }
}


/*******************CLOSED LOOP FUNCTIONS**********************/
float ClosedLoopWallTurn(float speed, float sonar_aim, float gyro_aim)
{
  float e, correction_val, current_dist;
  float kp_angle = 10;
  float ki_angle = 0.1;

  e = gyro_aim - gyroAngle;

  // float kp_angle = 30;
  // float ki_angle = 0.1;
  // current_dist = measure_sonar();
  // e = current_dist - sonar_aim - 0.8;

  correction_val = constrain(kp_angle * e + ki_angle * ki_integral_angle, -speed, speed);

  ki_integral_angle += e;

  left_font_motor.writeMicroseconds(1500 + correction_val);
  left_rear_motor.writeMicroseconds(1500 + correction_val);
  right_rear_motor.writeMicroseconds(1500 + correction_val);
  right_font_motor.writeMicroseconds(1500 + correction_val);

  BluetoothSerial.print("Current Error: ");
  BluetoothSerial.println(e);
  // BluetoothSerial.print("correction:   ");
  // BluetoothSerial.println(correction_val);
  // BluetoothSerial.print("ki:           ");
  // BluetoothSerial.println(ki_integral_angle);
  BluetoothSerial.println(" ");
  return e;
}


void ClosedLoopStraight(int speed_val)
{
    float e, correction_val;
    float kp_gyro = 40;
    float ki_gyro = 10;

    e = gyroAngleChange;

    correction_val = constrain(kp_gyro * e + ki_gyro * ki_integral_gyro, -100, 100);

    ki_integral_gyro += e;

    left_font_motor.writeMicroseconds(1500 + speed_val - correction_val);
    left_rear_motor.writeMicroseconds(1500 + speed_val - correction_val);
    right_rear_motor.writeMicroseconds(1500 - speed_val - correction_val);
    right_font_motor.writeMicroseconds(1500 - speed_val - correction_val);
}


void ClosedLoopStaph(int speed_val)
{
    float e_gyro, e_sonar, correction_val_gyro, correction_val_sonar;
    
    float e, correction_val;
    float kp_gyro = 40;
    float ki_gyro = 10;
    float kp_sonar = 15;
    float ki_sonar = 0.5;
    e_gyro = gyroAngleChange;

    e_sonar = sonar_dist - sonar_average;

    correction_val_gyro = constrain(kp_gyro * e_gyro + ki_gyro * ki_integral_gyro, -100, 100);

    correction_val_sonar = constrain(kp_sonar * e_sonar + ki_sonar * ki_integral_sonar, -150, 150);

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


/*******************PROVIDED FUNCTIONS**********************/
void fast_flash_double_LED_builtin()
{
  static byte indexer = 0;
  static unsigned long fast_flash_millis;
  if (millis() > fast_flash_millis) {
    indexer++;
    if (indexer > 4) {
      fast_flash_millis = millis() + 700;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    } else {
      fast_flash_millis = millis() + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

void slow_flash_LED_builtin()
{
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000) {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void speed_change_smooth()
{
  speed_val += speed_change;
  if (speed_val > 1000)
    speed_val = 1000;
  speed_change = 0;
}

#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK()
{
  static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  //the voltage of a LiPo cell depends on its chemistry and varies from about 3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
  //to about 4.20-4.25V (fully charged) = 860(4.2V Max)
  //Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    previous_millis = millis();
    SerialCom->print("Lipo level:");
    SerialCom->print(Lipo_level_cal);
    SerialCom->print("%");
    // SerialCom->print(" : Raw Lipo:");
    // SerialCom->println(raw_lipo);
    SerialCom->println("");
    Low_voltage_counter = 0;
    return true;
  } else {
    if (Lipo_level_cal < 0)
      SerialCom->println("Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (Lipo_level_cal > 160)
      SerialCom->println("!Lipo is Overchanged!!!");
    else {
      SerialCom->println("Lipo voltage too LOW, any lower and the lipo with be damaged");
      SerialCom->print("Please Re-charge Lipo:");
      SerialCom->print(Lipo_level_cal);
      SerialCom->println("%");
    }

    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }

}
#endif


/*******************PROVIDED MOTOR FUNCTIONS**********************/
void disable_motors()
{
  left_font_motor.detach();  // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();  // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach();  // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_font_motor.detach();  // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors()
{
  left_font_motor.attach(left_front);  // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);  // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}
void stop() //Stop
{
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}

void forward()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void reverse ()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void ccw ()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void cw ()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void strafe_left ()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void strafe_right ()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}
