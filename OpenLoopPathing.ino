/***Test for open loop pathing***/
#include <Servo.h>  //Need for Servo pulse output
#include <SoftwareSerial.h>

#define NO_READ_GYRO  //Uncomment of GYRO is not attached.
#define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.

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

//State machine states
enum STATE {
  INITIALISING,
  RUNNING,
  STOPPED
};

//Refer to Shield Pinouts.jpg for pin locations

//Default motor control pins
const byte left_front = 47;
const byte left_rear = 46;
const byte right_rear = 50;
const byte right_front = 51;


//Default ultrasonic ranging sensor pins, these pins are defined my the Shield
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

double sonar_cm;

//Pathing variables
    enum pathing_state {
        FORWARD,
        BACKWARD,
        STRAFE_RIGHT
    };

    pathing_state path_state;
    pathing_state last_state;

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

Servo left_font_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29
Servo turret_motor;


int speed_val = 100;
int speed_change;

//IR Sensor equation variables
double MR1coeff = 12.452;
double MR1power = -0.889;

double MR2coeff = 12.887;
double MR2power = -0.889;

double LR1coeff = 61.823;
double LR1power = -1.032;

double LR3coeff = 911.866;
double LR3power = -1.508;

//IR Sensor distance variables
double MR1cm, MR2cm, LR1cm, LR3cm;
double MR1cm_reading, MR2cm_reading, LR1cm_reading, LR3cm_reading;
int array_index = 0;
int iterations = 20;
double MR1arr[20], MR2arr[20], LR1arr[20], LR3arr[20];

//Kalman variables
double MR1var, MR2var, LR1var, LR3var;
double sensor_noise = 1;
double process_noise = 10;

//Serial Pointer
HardwareSerial *SerialCom;

int pos = 0;
void setup(void)
{
  BluetoothSerial.begin(115200);
  turret_motor.attach(20);
  pinMode(LED_BUILTIN, OUTPUT);

  //Initialise sensor pins
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  SerialCom = &Serial;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code_25/01/2018");
  delay(1000);
  SerialCom->println("Setup....");

  pinMode(ECHO_PIN, INPUT); // Sets the echoPin as an Input
  delay(1000); //settling time but no really needed

  path_state = FORWARD;
  last_state = FORWARD;
}

void loop(void) //main loop
{
  static STATE machine_state = INITIALISING;
  //Finite-state machine Code
  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      break;
    case RUNNING: //Lipo Battery Volage OK
      machine_state =  running();
      break;
    case STOPPED: //Stop of Lipo Battery voltage is too low, to protect Battery
      machine_state = stopped();
      break;
  };
}


STATE initialising() {
  //initialising
  SerialCom->println("INITIALISING....");
  delay(1000); //One second delay to see the serial string "INITIALISING...."
  SerialCom->println("Enabling Motors...");
  enable_motors();
  initialise_IR_sensors();
  SerialCom->println("RUNNING STATE...");
  return RUNNING;
}

STATE running() {

  static unsigned long previous_millis;

  read_serial_command();
  fast_flash_double_LED_builtin();

  if (millis() - previous_millis > 500) {  //Arduino style 500ms timed execution statement
    previous_millis = millis();

    SerialCom->println("RUNNING---------");
    speed_change_smooth();
    Analog_Range_A4();

#ifndef NO_READ_GYRO
    GYRO_reading();
#endif

#ifndef NO_HC-SR04
    HC_SR04_range();
#endif

#ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK()) return STOPPED;
#endif


  //   turret_motor.write(pos);

  //   if (pos == 0)
  //   {
  //     pos = 45;
  //   }
  //   else
  //   {
  //     pos = 0;
  //   }
  // }
  //Read and print IR sensor values
  read_IR_sensors();
  filter_IR_reading();
  print_IR_values();
  // open_loop_path(sonar_cm);
  return RUNNING;
  }
}

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

  // Print out results
  if ( pulse_width > MAX_DIST ) {
    SerialCom->println("HC-SR04: Out of range");
  } else {
    BluetoothSerial.print(cm, DEC);
    BluetoothSerial.println(' ');
  }
  sonar_cm = cm;
}
#endif

void Analog_Range_A4()
{
  SerialCom->print("Analog Range A4:");
  SerialCom->println(analogRead(A4));
}

#ifndef NO_READ_GYRO
void GYRO_reading()
{
  SerialCom->print("GYRO A3:");
  SerialCom->println(analogRead(A3));
}
#endif

//Serial command pasing
void read_serial_command()
{
  if (SerialCom->available()) {
    char val = SerialCom->read();
    SerialCom->print("Speed:");
    SerialCom->print(speed_val);
    SerialCom->print(" ms ");

    //Perform an action depending on the command
    switch (val) {
      case 'w'://Move Forward
      case 'W':
        forward ();
        SerialCom->println("Forward");
        BluetoothSerial.println("Forward");
        
        break;
      case 's'://Move Backwards
      case 'S':
        reverse ();
        SerialCom->println("Backwards");
        BluetoothSerial.println("Backwards");
        break;
      case 'q'://Turn Left
      case 'Q':
        strafe_left();
        SerialCom->println("Strafe Left");
        BluetoothSerial.println("Strafe Left");
        break;
      case 'e'://Turn Right
      case 'E':
        strafe_right();
        SerialCom->println("Strafe Right");
        BluetoothSerial.println("Strafe Right");
        break;
      case 'a'://Turn Right
      case 'A':
        ccw();
        SerialCom->println("ccw");
        BluetoothSerial.println("ccw");
        break;
      case 'd'://Turn Right
      case 'D':
        cw();
        SerialCom->println("cw");
        BluetoothSerial.println("cw");
        break;
      case '-'://Turn Right
      case '_':
        speed_change = -100;
        SerialCom->println("-100");
        break;
      case '=':
      case '+':
        speed_change = 100;
        SerialCom->println("+");
        break;
      default:
        stop();
        SerialCom->println("stop");
        BluetoothSerial.println("stop");
        break;
    }

  }

}

//----------------------Motor moments------------------------
//The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control

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

// Open loop test script
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
      last_state = FORWARD;
      break;

    case BACKWARD:
      if(sonar_cm > 106.9){
          stop();
          delay(500);
          path_state = STRAFE_RIGHT;
      }else{
          reverse();
      }
      last_state = BACKWARD;
      break;

    case STRAFE_RIGHT:
      strafe_right();
      delay(2000);
      stop();
      delay(500);
      path_state = (last_state == FORWARD) ? BACKWARD : FORWARD;
      break;
  }
}

double read_IR(double coefficient, double power, double sensor_reading){
  double sensor_cm;
  BluetoothSerial.print("SENSOR READING:");
  BluetoothSerial.println(sensor_reading);
  sensor_cm = coefficient *1000*(pow(sensor_reading, power));
  return sensor_cm;
}

void read_IR_sensors(){
  BluetoothSerial.println("READ SENSOR START");
  MR1cm_reading = read_IR(MR1coeff, MR1power, analogRead(A4));
  MR2cm_reading = read_IR(MR2coeff, MR2power, analogRead(A6));
  LR1cm_reading = read_IR(LR1coeff, LR1power, analogRead(A5));
  LR3cm_reading = read_IR(LR3coeff, LR3power, analogRead(A7));
}

void initialise_IR_sensors(){
  double MR1sum, MR2sum, LR1sum, LR3sum;

  for (int i = 0; i<=iterations; i++){
    read_IR_sensors();
    MR1arr[i] = MR1cm_reading;
    MR2arr[i] = MR2cm_reading;
    LR1arr[i] = LR1cm_reading;
    LR3arr[i] = LR3cm_reading;
    delay(5);
  }
  MR1cm = average_array(MR1arr, 0);
  MR2cm = average_array(MR2arr, 0);
  LR1cm = average_array(LR1arr, 0);
  LR3cm = average_array(LR3arr, 0);

  //Set IR variance to 0
  MR1var = 0;
  MR2var = 0;
  LR1var = 0;
  LR3var = 0;
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

void print_IR_values(){
  BluetoothSerial.print("MR1 DISTANCE:");
  BluetoothSerial.println(MR1cm);
  BluetoothSerial.print("MR2 DISTANCE:");
  BluetoothSerial.println(MR2cm);
  BluetoothSerial.print("LR1 DISTANCE:");
  BluetoothSerial.println(LR1cm);
  BluetoothSerial.print("LR3 DISTANCE:");
  BluetoothSerial.println(LR3cm);
}

void filter_IR_reading(){
  //Add values to given array
  MR1arr[array_index] = MR1cm_reading;
  MR2arr[array_index] = MR2cm_reading;
  LR1arr[array_index] = LR1cm_reading;
  LR3arr[array_index] = LR3cm_reading;

  //Set next array index
  array_index++;
  if (array_index >=20){
    array_index = 0;
  }

  //Average these to final value 
  MR1cm = average_array(MR1arr, MR1cm);
  MR2cm = average_array(MR2arr, MR2cm);
  LR1cm = average_array(LR1arr, LR1cm);
  LR3cm = average_array(LR3arr, LR3cm);

  // MR1cm = IR_Kalman(MR1cm_reading, MR1cm, &MR1var);
  // MR2cm = IR_Kalman(MR2cm_reading, MR2cm, &MR2var);
  // LR1cm = IR_Kalman(LR1cm_reading, LR1cm, &LR1var);
  // LR3cm = IR_Kalman(LR3cm_reading, LR3cm, &LR3var);
}


double IR_Kalman(double distance_reading, double last_reading, double* last_var){
  double post_est, prior_var, post_var, kalman_gain;

  prior_var = *last_var + process_noise; //variation in last reading

  kalman_gain = prior_var/(prior_var + sensor_noise); //gain correction of prior variation in last reading
  post_est  = last_reading + kalman_gain*(distance_reading-last_reading);
  post_var = (1-kalman_gain)*prior_var;
  *last_var = post_var;
  return (post_est);
}
