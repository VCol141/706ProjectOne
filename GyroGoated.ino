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
#define BOARD_WIDTH 150
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
  ALIGNING,
  RUNNING,
  STOPPED
};

//Homing variables
//Homing variables
enum homing_state {
    ROTATE,
    APPROACHING_WALL,
    FIND_WALL,
    FACE_WALL
};

homing_state home_state = ROTATE;

enum aligning_state{
    FIND_ORIENTATION,
    TURN,
    FIND_CLOSEST_WALL,
    GO_HOME_STRAIGHT,
    GO_HOME_STRAFE,
    ALIGN_ROBOT,
    DUMMY
};

aligning_state align_state = FIND_ORIENTATION;
bool found_closest_wall = false;
int right_side_distance, left_side_distance, across_distance;
double IR_angle_error;
bool found_angle_offset = false;

// enum start_corner {                  //for optimising homing
//     UP,
//     DOWN
// };

// start_corner starting_corner;

//Pathing variables
enum pathing_state {
    FORWARD,
    BACKWARD,
    STRAFE_RIGHT,
    STRAFE_LEFT
};

enum RUN
{
    STRAIGHT,
    STRAFE,
    STOP
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

int speed_val = 300;
int speed_change;


/***IRS***/
//IR Equation Variables - MAY NEED TO UPDATE
double MR1coeff = 19.000;
double MR1power = -0.94;

double MR2coeff = 20.000;
double MR2power = -0.97;

double LR1coeff = 325.000;
double LR1power = -1.33;

double LR3coeff = 540.000;
double LR3power = -1.42;

//IR Sensor working variables
double MR1mm, MR2mm, LR1mm, LR3mm;
double MR1mm_reading, MR2mm_reading, LR1mm_reading, LR3mm_reading;
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
double sonar_threshold = 2;
int wall_settled = 0;
double wall = 0;

//Sonar values
double sonar_cm = 0;
double straight_time = 0;
double ki_integral_sonar = 0;
double sonar_dist = 0;
double ki_integral_angle = 0;
double sonar_baseline;

//Sonar Kalman
double process_noise_sonar = 10;
double sensor_noise_sonar = 1;
double sonar_variance = 0;


/***GYRO***/
//Gyro turn variables
double gyro_aim;
int aimup = 0;
// GYRO
int gyroPin = A2;
int gyroVal = 0;

double gyro_error = 0;
double gyroZeroVoltage = 0;
double gyroRate = 0;
double gyroAngleChange = 0;
double gyroAngle = 0;
double error = 0;
double ki_integral_gyro = 0;
double gyroTime = 0;

int array_index = 0;
double gyro_array[20];
double gyro_average = 0;


//Kalman variables
double prev_val_gyro = 0;
double last_var_gyro = 999;
double sensor_noise_gyro = 10;
double process_noise_gyro = 1;

/***SERVO***/
Servo turret_motor;

// VLADS STUFF
double ki_straight_gyro = 0;

double ki_strafe_gyro = 0;
double ki_strafe_ir = 0;

double ki_turn_gyro = 0;

double ki_distance_sonar = 0;

#define CONTROL_CONSTRAINT_GYRO 100

// Run

bool forward_backward = 0;

bool last_lap = 0;

double distance_aim = 20;

static int MIN_DISTANCE = 15;
static int MAX_DISTANCE = 160;
static int STRAFE_DISTANCE = 20;

static int DISTANCE_OFFSET = 2.5;
static int MIN_SIDE_DIST = 20;

// Sonar
int sonar_MA_n = 30;
double sonar_values[30];

/*******************MAIN SET-UP**********************/
void setup(void)
{
  BluetoothSerial.begin(115200);
  BluetoothSerial.println("");
  BluetoothSerial.println("");
  BluetoothSerial.println("");
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
    case ALIGNING:
      machine_state = align();
      break;
    case RUNNING: //Lipo Battery Volage OK
      machine_state =  running();
      break;
    case STOPPED: //Stop of Lipo Battery voltage is too low, to protect Battery
      machine_state = stopped();
      break;
  };
  delay(10);

  Gyro();
  Sonar();
}

/*******************INITIALISING**********************/
STATE initialising() {
  BluetoothSerial.println("INITIALISING....");
  
  enable_motors();

  double sum1 = 0;
  for (int i = 0; i < 200; i++)
  { // read 100 values of voltage when gyro is at still, to calculate the zero-drift.
    gyroVal = analogRead(gyroPin);
    // gyroVal = constrain(analogRead(gyroPin), 500, 530);
    delay(10);
    sum1 += gyroVal;
    BluetoothSerial.println(gyroVal);
  }
  gyroZeroVoltage = sum1 / 200; // average the sum as the zero drifting


  // for(int i = 0; i < 20; i++){
  //   gyroVal = analogRead(gyroPin);
  //   gyroRate = (gyroVal * 5.00) / 1023;
  //   // find the voltage offset the value of voltage when gyro is zero (still)
  //   gyroRate -= (gyroZeroVoltage * 5.00) / 1023;
  //   gyro_array[i] = (gyroRate / 0.007);
  // }
  // gyro_average = average_array(gyro_array, 0, 20);

  BluetoothSerial.print("GYRO 0 VOLTAGE:");
  BluetoothSerial.println(gyroZeroVoltage);

  intialise_sensors();
  
  //COMMENT OUT LATER
  // sonar_baseline = SonarCheck(0);

  return RUNNING;
}

/*******************HOMING**********************/
STATE homing(){
  //Check if battery voltage is ok before proceeding
  #ifndef NO_BATTERY_V_OK
      if (!is_battery_voltage_OK()) return STOPPED;
  #endif


  switch (home_state){
    case ROTATE: //Take initial average and  start turning in arbituary direction
      BluetoothSerial.println("Starting Rotation");
      ccw();
      home_state = APPROACHING_WALL;
      BluetoothSerial.println("Looking for max");
      break;
    case APPROACHING_WALL: //Detect when the robot is approaching a wall
      if (sonar_cm <= wall - sonar_threshold){ //If maximum surpassed/is already decreasing, start looking for wall
        BluetoothSerial.println("looking for min");
        gyro_aim = gyroAngle;
        home_state = FIND_WALL;
      }
      else if(wall < sonar_cm){ //New maxmum detected, save this value
        wall = sonar_cm;
      }
      break;
    case FIND_WALL: //Look for local min point
      if (wall > sonar_cm){ //New minimum found, log angle and min distance
        gyro_aim = gyroAngle;
        wall = sonar_cm;
      }
      else if(sonar_cm >= wall+sonar_threshold){ //Minimum surpassed, turn towards given location
        stop();
        delay(1000); //allow motors to power off before completely switching the direction
        gyro_aim = gyroAngle - gyro_aim;
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
      double sonar_error;
      sonar_error = ClosedLoopTurn(130, gyro_aim); //Aim about 8 degrees back from the actual measured angle
      if (abs(sonar_error) <= 5){
        wall_settled++;
        if (wall_settled == 10){
          wall_settled = 0;
          stop();
          delay(300);
          BluetoothSerial.println("TURNING STOPPED");
          gyroAngle = 0;

          return ALIGNING;
        }
      }
      else{
        wall_settled = 0;
      }
      break;
  }
  return HOMING;
}

/*******************ALIGNING*********************/
STATE align()
{
  // BluetoothSerial.print("alligning: ");                           //debug
  // BluetoothSerial.println(align_state);

  STATE return_state = ALIGNING; 

  double e_distance, u_distance, kp_distance, ki_distance;

  switch (align_state){
    case FIND_ORIENTATION:
      if(!found_closest_wall){
        //turn the ultrasonic servo both sides and get measurements
        // turret_motor.write(0);
        // delay(500);
        right_side_distance = SonarCheck(0);
        BluetoothSerial.print("RIGHT DIST: ");
        BluetoothSerial.println(right_side_distance);

        // turret_motor.write(180);
        // delay(500);
        left_side_distance = SonarCheck(180);
        BluetoothSerial.print("LEFT DIST: ");
        BluetoothSerial.println(left_side_distance);

        //find which corner to go to
        //starting_corner = (right_side_distance < left_side_distance) ? UP : DOWN;       //FOR OPTIMISING
        turret_motor.write(90);
        found_closest_wall = true;
        across_distance = right_side_distance + left_side_distance;
        BluetoothSerial.println(across_distance);

      }else{
      //found closest all, will now turn or not turn

        //check if the robot is in the right orientation
        if(across_distance > 150){
          BluetoothSerial.println("TURNING 90");
          double turn_error;
          turn_error = ClosedLoopTurn(150, 90); //Aim 90 degrees
          if (abs(turn_error) <= 5){
            wall_settled++;
            if (wall_settled == 10){
              stop();
              delay(2000);
              BluetoothSerial.println("TURNING STOPPED");
              //turn the turret motor back to its closest direction
              align_state = GO_HOME_STRAIGHT;        //it is in the right orientation
              BluetoothSerial.println("GOING HOME");
              gyroAngle = 0;      //make sonar straight again
            }
          } 
        }
        else{
          align_state = GO_HOME_STRAIGHT;        //it is in the right orientation
          BluetoothSerial.println("GOING HOME");
        }
        
      }
      break;

    case GO_HOME_STRAIGHT:
      BluetoothSerial.print("Waiting to go home");
      kp_distance = 20;
      ki_distance = 0;

      e_distance = sonar_cm - MAX_DISTANCE - 10;
      u_distance = constrain(kp_distance * e_distance + ki_distance * ki_distance_sonar, -speed_val, speed_val);

      ClosedLoopStraight(u_distance);

      if (sonar_cm >= MAX_DISTANCE + DISTANCE_OFFSET) 
      {
        ClosedLoopStraight(u_distance);
        delay(1000);
        stop();
        align_state = GO_HOME_STRAFE;
        SonarCheck(0);

      }
      break;

    case GO_HOME_STRAFE:
      kp_distance = 20;
      ki_distance = 0;

      e_distance = sonar_cm - 105;
      u_distance = constrain(kp_distance * e_distance + ki_distance * ki_distance_sonar, -speed_val, speed_val);

      ClosedLoopStrafe(u_distance);

      if (sonar_cm >= 102) 
      {
        stop();
        delay(1000);
        align_state = GO_HOME_STRAFE;
        SonarCheck(90);
        return RUNNING;
      }
      break;

    case ALIGN_ROBOT:
      align_against_wall();
      break;

    case DUMMY:
      stop();
      break;
  }

  return return_state;
}

/*******************RUNNING**********************/
STATE running() {
    STATE return_state = RUNNING;
    static RUN run_state = STRAIGHT;

    double e_distance, u_distance;
    double kp_distance = 20;
    double ki_distance = 0;
    double average_ir = 0;
    // e_distance = sonar_cm - distance_aim;

    read_IR_sensors();
    filter_IR_reading();

    switch(run_state)
    { 
        case STRAIGHT:
                distance_aim = 140;
                if (!forward_backward) {
                  average_ir = average_IR(MR1mm, MR2mm);
                  // BluetoothSerial.print("Front IR distance: ");
                  // BluetoothSerial.println(average_ir);
                  e_distance = average_ir - distance_aim;
                }
                else {
                  average_ir = average_IR(LR1mm, LR3mm);
                  // BluetoothSerial.print("Rear IR distance: ");
                  // BluetoothSerial.println(average_ir);
                  e_distance = -(average_ir- distance_aim);
                }
                // BluetoothSerial.print("DISTANCE ERROR: ");
                // BluetoothSerial.println(e_distance);
                // BluetoothSerial.println("");
                u_distance = constrain(kp_distance * e_distance + ki_distance * ki_distance_sonar, -speed_val, speed_val);

                ClosedLoopStraight(u_distance);

            // if ((!forward_backward && (average_ir <= distance_aim)) || (forward_backward && (average_ir <= distance_aim))) 
            // {
            //     BluetoothSerial.println("STARTED STRAFING");
            //     stop();
            //     delay(500);
            //     sonar_baseline = SonarCheck(0);

            //     ki_distance_sonar = 0;

            //     (sonar_baseline < MIN_SIDE_DIST) ? run_state = STOP : run_state = STRAFE;

            //     distance_aim = sonar_baseline - STRAFE_DISTANCE;

            //  }
        break;

        case STRAFE:
            e_distance = sonar_cm - distance_aim;
            u_distance = constrain(kp_distance * e_distance + ki_distance * ki_distance_sonar, -speed_val, speed_val);
            // BluetoothSerial.println("STRAFING ");
            // BluetoothSerial.print("sonar_baseline ");
            // BluetoothSerial.println(distance_aim);
            // BluetoothSerial.print("Actual reading ");
            // BluetoothSerial.println(sonar_cm);
            // BluetoothSerial.print("Current Error: ");
            // BluetoothSerial.println(e_distance);
            // BluetoothSerial.println("");

            ClosedLoopStrafe(u_distance);
            

            if (sonar_cm <= (distance_aim + DISTANCE_OFFSET))
            {
                stop();
                delay(500);
                // sonar_baseline = SonarCheck(0);

                run_state = STRAIGHT;

                // SonarCheck(90);
                // distance_aim = (forward_backward) ? MIN_DISTANCE : MAX_DISTANCE;
                ki_distance_sonar = 0;

                forward_backward = !forward_backward;
            }
        break;

        case STOP:
            stop();
            return_state = STOPPED;
        break;
    };

    ki_distance_sonar += e_distance;

    return return_state;
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
  double cm;

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


double KalmanSonar(double rawdata){   // Kalman Filter
  if (rawdata > 40 + sonar_cm || rawdata < sonar_cm - 40){ //If the value is absolutely outrageous, ignore it and use the last recorded value || 
    return sonar_cm;
  }
  else{
    rawdata = constrain(rawdata, sonar_cm - 10, sonar_cm + 10);
    double a_post_est, a_priori_var, a_post_var, kalman_gain;

    a_priori_var = sonar_variance + process_noise_sonar; 

    kalman_gain = a_priori_var/(a_priori_var+sensor_noise_sonar);
    a_post_est = sonar_cm + kalman_gain*(rawdata-sonar_cm);
    sonar_variance = (1 * kalman_gain)*a_priori_var;
    sonar_cm = rawdata;
    //BluetoothSerial.println("Kalman");
    return a_post_est;
  }   
}

double SonarCheck(double angle_in)
{
    turret_motor.write(angle_in);

    delay(300);

    sonar_cm = 0;
    for (int i = 0; i < sonar_MA_n; i++)
    {
        sonar_cm = 0;
        while (sonar_cm < 15|| sonar_cm > 200){
          Sonar();
          ultraArray[i] = sonar_cm;
          delay(20);
        }
    }
    sonar_cm = average_array(ultraArray, 0, 20);
    return (sonar_cm);
}


/*******************SONAR/IR INITIALISATION**********************/
void intialise_sensors(){
double MR1sum, MR2sum, LR1sum, LR3sum;
int iterations = 20;

  for (int i = 0; i<=iterations; i++){
    read_IR_sensors();
    MR1arr[i] = MR1mm_reading;
    MR2arr[i] = MR2mm_reading;
    LR1arr[i] = LR1mm_reading;
    LR3arr[i] = LR3mm_reading;
    Sonar();
    ultraArray[i] = sonar_cm;
    delay(5);
  }
  MR1mm = average_array(MR1arr, 0, 20);
  MR2mm = average_array(MR2arr, 0, 20);
  LR1mm = average_array(LR1arr, 0, 20);
  LR3mm = average_array(LR3arr, 0, 20);
  sonar_cm = average_array(ultraArray, 0, 20);

  //If using Kalman to filter, Set IR variance to 0
  // MR1var = 0;
  // MR2var = 0;
  // LR1var = 0;
  // LR3var = 0;
}

double average_array(double* input_array, double last_average, int iterations){
  double sum = 0;
  int count = 0;
  
  for (int i = 0; i< iterations;i++){
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
  sensor_mm = coefficient *1000*(pow(sensor_reading, power));
  return sensor_mm;
}

void read_IR_sensors(){
  // BluetoothSerial.println("READ SENSOR START");
  MR1mm_reading = read_IR(MR1coeff, MR1power, analogRead(A5));
  MR2mm_reading = read_IR(MR2coeff, MR2power, analogRead(A6));
  LR1mm_reading = read_IR(LR1coeff, LR1power, analogRead(A4));
  LR3mm_reading = read_IR(LR3coeff, LR3power, analogRead(A7));
}

void filter_IR_reading(){
  //Average these to final value 
  double mrbuffer = 200;
  double lrbuffer = 500;
  MR1mm = constrain(MR1mm_reading, MR1mm-mrbuffer, MR1mm+mrbuffer);
  MR2mm = constrain(MR2mm_reading, MR2mm-mrbuffer, MR2mm+mrbuffer);
  LR1mm = constrain(LR1mm_reading, LR1mm-lrbuffer, LR1mm+lrbuffer);
  LR3mm = constrain(LR3mm_reading, LR3mm-lrbuffer, LR3mm+lrbuffer);
}

double average_IR(double IR1, double IR2) {
  // Average distance of IRs
  // BluetoothSerial.print("LR1 Raw ");
  // BluetoothSerial.println(LR1mm_reading);
  // BluetoothSerial.print("LR1 Filtered ");
  // BluetoothSerial.println(LR1mm);

  // BluetoothSerial.print("LR3 raw: ");
  // BluetoothSerial.println(LR3mm_reading);
  // BluetoothSerial.print("LR3 Filtered");
  // BluetoothSerial.println(LR3mm);
  // BluetoothSerial.println("");

  return (IR1 + IR2) / 2;
}



/*******************GYRO FUNCTIONS**********************/
void Gyro()
{
    // convert the 0-1023 signal to 0-5v
    // double gyro_reading = KalmanGyro(analogRead(gyroPin));
    double gyro_reading = (analogRead(gyroPin));

    gyroRate = (gyro_reading * 5.00) / 1023;

    // find the voltage offset the value of voltage when gyro is zero (still)
    gyroRate -= (gyroZeroVoltage * 5.00) / 1023;


    // read out voltage divided the gyro sensitivity to calculate the angular velocity
    double angularVelocity = (gyroRate / 0.007)-1; // from Data Sheet, gyroSensitivity is 0.007 V/dps

    gyro_average = KalmanGyro(angularVelocity);
    // average_gyro(angularVelocity);

    BluetoothSerial.print("Measured angular velocity");
    BluetoothSerial.println(angularVelocity);
    // BluetoothSerial.print("Average ANGULAR VELOCITY");
    // BluetoothSerial.println(gyro_average);
    
    // BluetoothSerial.print("GYRO RATE");
    // BluetoothSerial.println(gyroRate);
    // BluetoothSerial.print("Angular Velocity");
    // BluetoothSerial.println(angularVelocity);

    // if the angular velocity is less than the threshold, ignore it
    if (gyro_average >= 2 || gyro_average <= -2)
    {
        gyroAngleChange = millis()-gyroTime;
        gyroAngleChange = 1000 / gyroAngleChange;
        gyroAngleChange = gyro_average/ gyroAngleChange;
        // BluetoothSerial.print("Angle change");
        // BluetoothSerial.println(gyroAngleChange);
        gyroAngle += gyroAngleChange;
    }
    // BluetoothSerial.println("");
    gyroTime = millis();
}

double average_gyro(double rawdata){
  gyro_array[array_index] = KalmanGyro(rawdata);
  BluetoothSerial.print("KALMANED VALUE");
  BluetoothSerial.println(gyro_array[array_index]);
  array_index++;
  if (array_index == 20){
    array_index = 0;
  }
  gyro_average = average_array(gyro_array, gyro_average, 20);
  return gyro_average;
}

double KalmanGyro(double rawdata){   // Kalman Filter
  // if (rawdata > 1 + prev_val_gyro || rawdata < prev_val_gyro - 1){ //If the value is absolutely outrageous, ignore it and use the last recorded value || 
  //   return prev_val_gyro;
  // }
  // else{

  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;

  rawdata = constrain(rawdata, prev_val_gyro-1, prev_val_gyro +1);
  a_priori_var = last_var_gyro + process_noise_gyro; 

  kalman_gain = a_priori_var/(a_priori_var+sensor_noise_gyro);
  a_post_est = prev_val_gyro + kalman_gain*(rawdata-prev_val_gyro);
  a_post_var = (1- kalman_gain)*a_priori_var;
  last_var_gyro = a_post_var;
  prev_val_gyro = rawdata;
  return a_post_est;
  // }
}


/*******************CLOSED LOOP FUNCTIONS**********************/
double ClosedLoopTurn(double speed, double gyro_aim)
{
  double e, correction_val;
  double kp_angle = 6;
  double ki_angle = 0;

  e = gyro_aim - gyroAngle;

  correction_val = constrain(kp_angle * e + ki_angle * ki_integral_angle, -speed, speed);

  ki_integral_angle += e;

  left_font_motor.writeMicroseconds(1500 + correction_val);
  left_rear_motor.writeMicroseconds(1500 + correction_val);
  right_rear_motor.writeMicroseconds(1500 + correction_val);
  right_font_motor.writeMicroseconds(1500 + correction_val);

  // BluetoothSerial.print("Current Error: ");
  // BluetoothSerial.println(e);
  // BluetoothSerial.print("correction:   ");
  // BluetoothSerial.println(correction_val);
  // BluetoothSerial.print("ki:           ");
  // BluetoothSerial.println(ki_integral_angle);
  // BluetoothSerial.println(" ");
  return e;
}


void ClosedLoopStraight(int speed_val)
{
    double e, correction_val;

    double kp_gyro = 30;
    double ki_gyro = 0.1;

    //gyro_error = 0;
    e = gyroAngle;

    // (abs(gyroAngle) < 1) ? e = gyroAngle : e = 0;
    // e = ;

    BluetoothSerial.print("Gyro Angle: ");
    BluetoothSerial.println(e);
    BluetoothSerial.println("");

    correction_val = constrain(kp_gyro * e + ki_gyro * ki_straight_gyro, -CONTROL_CONSTRAINT_GYRO, CONTROL_CONSTRAINT_GYRO);

    ki_straight_gyro += e;

    left_font_motor.writeMicroseconds(1500 + speed_val - correction_val);
    left_rear_motor.writeMicroseconds(1500 + speed_val - correction_val);
    right_rear_motor.writeMicroseconds(1500 - speed_val - correction_val);
    right_font_motor.writeMicroseconds(1500 - speed_val - correction_val);
}

void ClosedLoopStrafe(int speed_val)
{
    double e_gyro, e_ir, correction_val_gyro = 0, correction_val_ir = 0;

    double kp_gyro = 25;
    double ki_gyro = 5;

    //double kp_ir = 0;
    //double ki_ir = 0;


    (abs(gyroAngleChange) < 3) ? e_gyro = gyroAngleChange : e_gyro = 0;

    correction_val_gyro = constrain(kp_gyro * e_gyro + ki_gyro * ki_strafe_gyro, -CONTROL_CONSTRAINT_GYRO, CONTROL_CONSTRAINT_GYRO);
    
    ki_strafe_gyro += e_gyro;

    //e_ir = analogRead((forward_backward) ? MR_B : MR_F) - ir_average;

    //correction_val_ir = constrain(kp_ir * e_ir + ki_ir * ki_strafe_ir, -CONTROL_CONSTRAINT_IR, CONTROL_CONSTRAINT_IR);

    //ki_strafe_ir += e_ir;

    left_font_motor.writeMicroseconds(1500 + speed_val - correction_val_gyro - correction_val_ir);
    left_rear_motor.writeMicroseconds(1500 - speed_val - correction_val_gyro - correction_val_ir);
    right_rear_motor.writeMicroseconds(1500 - speed_val - correction_val_gyro + correction_val_ir);
    right_font_motor.writeMicroseconds(1500 + speed_val - correction_val_gyro + correction_val_ir);
}

/*******************HELPER FUNCTIONS************************/
void align_against_wall(){
  if(!found_angle_offset){
      //calculate the misalignment angle of the robot using trig
      BluetoothSerial.print("LR1 Reading: ");
      BluetoothSerial.println(LR1mm_reading);
      BluetoothSerial.print("LR3 Reading: ");
      BluetoothSerial.println(LR3mm_reading);
      double IR_base_length = 100;
      double IR_height_offset = LR3mm_reading - LR1mm_reading;
      IR_angle_error = atan(IR_height_offset/IR_base_length) * (180/3.14);    //convert rad to deg
      align_state = ALIGN_ROBOT;
      BluetoothSerial.print("angle offset: ");
      BluetoothSerial.println(IR_angle_error);
      gyroAngle = 0;      //make sonar straight again
      found_angle_offset = true;
  }else{
      BluetoothSerial.print("Fixing angle...");
      double sonar_error;
      sonar_error = ClosedLoopTurn(130, IR_angle_error); //Aim about 8 degrees back from the actual measured angle
      if (abs(sonar_error) <= 5){
        wall_settled++;
        if (wall_settled == 10){
          wall_settled = 0;
          stop();
          delay(2000);
          BluetoothSerial.println("TURNING STOPPED");
          gyroAngle = 0;

          align_state = DUMMY;
        }
      }
      else{
        wall_settled = 0;
      }
  }
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

void Sonar()
{
    unsigned long t1, t2, pulse_width;
    double cm;
     
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

    
    (sonar_cm == 0 ? sonar_cm = cm : sonar_cm = KalmanSonar(cm));
    // BluetoothSerial.print("Raw Sonar Reading");
    // BluetoothSerial.println(cm);
    // BluetoothSerial.print("Last sonar reading");
    // BluetoothSerial.println(sonar_cm);
    // BluetoothSerial.println("");
}

