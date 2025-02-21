/**Library**/

#include <QTRSensors.h>
#include <L298NX2.h>
/**Library**/
/**Motor Parameter**/

/*Right Motor*/
#define EN_A A1
#define IN1_A 5
#define IN2_A 6

//Speed Declaration
#define RIGHT_MOTOR_BASE_SPEED 185
#define RIGHT_MOTOR_MAXIMUM_SPEED 205
#define RIGHT_TURN_SPEED 130
/*Right Motor*/

/*Left Motor*/
#define EN_B A0
#define IN1_B 8
#define IN2_B 7

//Speed Declaration
#define LEFT_MOTOR_BASE_SPEED 185
#define LEFT_MOTOR_MAXIMUM_SPEED 205
#define LEFT_TURN_SPEED 130
/*Left Motor*/
L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);
int delSpeed = 0, left_motor_speed = 0, right_motor_speed = 0;

/**Motor Parameter**/
/**IR Sensor Parameter**/

QTRSensors irsensor;
const uint8_t NUMBER_OF_SENSOR = 7;
uint16_t sensorValues[NUMBER_OF_SENSOR];
int lastSensor;
#define THRESHOLD 800

/**IR Sensor Parameter**/
/**PID**/

#define KP 0.11
#define KI 0
#define KD 0

/**PID**/
/**Others Parameter**/

boolean start_follow = false;
int pushButton1 = A4; //Start & Stop
#define SET_POINT 3000
uint16_t position;
int line_error, last_line_error = 0;
int P, D;
float I;
bool line_track[NUMBER_OF_SENSOR];
#define ONE_INCH_DELAY 220
#define STOP_DELAY 50
int SUM_OF_SENSOR_VALUES;
char mode;
/**Others Parameter**/

void setup() {
  // put your setup code here, to run once:
  //Push

  pinMode(pushButton1, INPUT);

  motors.stop();
  Calibrate_Sensor(); //Calibration


}

void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(pushButton1) == HIGH)
  {
    start_follow = !start_follow;
    motors.stop();

    delay(2000);
  }

  if (line_track[NUMBER_OF_SENSOR] == true && start_follow == true)
  {
    irsensorread();
    if ((sensorValues[1] == 0 && sensorValues[6] == 0) && (sensorValues[1] == 1 || sensorValues[2] == 1 || sensorValues[3] == 1 || sensorValues[4] == 1 || sensorValues[5] == 1)) {
      PID_GO();
    }
    else {
      FOLLOW_THE_RULES();

    }
  }
}

/**IR Sensor Calibrate Function Start**/
void Calibrate_Sensor() {
  irsensor.setTypeRC();
  irsensor.setSensorPins((const uint8_t[]) {
    12, 11, 10, 9, 4, 3, 2
  }, NUMBER_OF_SENSOR);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 200; i++)
  { motors.setSpeedA(RIGHT_TURN_SPEED);
    motors.forwardA();
    motors.setSpeedB(LEFT_TURN_SPEED);
    motors.backwardB();
    irsensor.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
  motors.stop();
}
/**IR Sensor Calibrate Function End**/
/*Motor*/
void motor(int LEFT_MOTOR_SPEED, int RIGHT_MOTOR_SPEED)
{
  if (RIGHT_MOTOR_SPEED == 0)
  {
    motors.stopA();
  }
  if (RIGHT_MOTOR_SPEED > 0)
  {
    motors.forwardA();
  }
  else if (RIGHT_MOTOR_SPEED < 0)
  {
    motors.backwardA();

  }
  if (LEFT_MOTOR_SPEED == 0)
  {
    motors.stopB();
  }
  if (LEFT_MOTOR_SPEED > 0)
  {
    motors.forwardB();

  }
  else if (LEFT_MOTOR_SPEED < 0)
  {
    motors.backwardB();
  }

  if (abs(LEFT_MOTOR_SPEED) > LEFT_MOTOR_MAXIMUM_SPEED)
  {
    LEFT_MOTOR_SPEED = LEFT_MOTOR_MAXIMUM_SPEED;
  }

  if (abs(RIGHT_MOTOR_SPEED) > RIGHT_MOTOR_MAXIMUM_SPEED)
  {
    RIGHT_MOTOR_SPEED = RIGHT_MOTOR_MAXIMUM_SPEED;
  }
  motors.setSpeedA (abs(RIGHT_MOTOR_SPEED));
  motors.setSpeedB (abs(LEFT_MOTOR_SPEED));
}
/*Motor*/

/**IR Sensor READING Function Start**/
/**IR Sensor READING Function Start**/
void irsensorread() {
  irsensor.read(sensorValues);
  for (unsigned char index = 0; index < NUMBER_OF_SENSOR; index++)
  {
    line_track[index] = (sensorValues[index] > THRESHOLD) ? true : false;
    sensorValues[index] = (sensorValues[index] > THRESHOLD) ? 1 : 0;
  }
}
/**IR Sensor READING Function Start**/

/**PID Section**/
void PID_GO() {
  position = irsensor.readLineBlack(sensorValues);
  line_error = position - SET_POINT;
  P = line_error * KP;
  D = KD * (line_error - last_line_error);
  if (line_error > (SET_POINT - 200) && line_error < (SET_POINT + 200)) {
    I += (KI * line_error);
  }
  else {
    I = 0;
  }
  delSpeed = P + I + D;

  left_motor_speed = LEFT_MOTOR_BASE_SPEED + delSpeed;
  right_motor_speed = RIGHT_MOTOR_BASE_SPEED - delSpeed;

  motor(left_motor_speed, right_motor_speed);

  last_line_error = line_error;
}
/**PID Section**/
/*One Step Forward at Any Junction*/
void Go_One_Step() {
  motors.setSpeed(135);
  motors.forward();
  delay(ONE_INCH_DELAY);
  motors.setSpeed(0);
  motors.stop();
  delay(STOP_DELAY);
  irsensorread();
}
/*One Step Forward at Any Junction*/

/*Turn Left*/
void GO_LEFT() {
  while (sensorValues[1] != 1) {
    irsensorread();
    motors.setSpeedA(RIGHT_TURN_SPEED);
    motors.forwardA();
    motors.setSpeedB(LEFT_TURN_SPEED);
    motors.backwardB();
  }
  motors.setSpeed(0);
  motors.stop();
  delay(STOP_DELAY);
}
/*Turn Left*/

/*Turn Right*/
void GO_RIGHT() {
  while (sensorValues[5] != 1) {
    irsensorread();
    motors.setSpeedA(RIGHT_TURN_SPEED);
    motors.backwardA();
    motors.setSpeedB(LEFT_TURN_SPEED);
    motors.forwardB();
  }
  motors.setSpeed(0);
  motors.stop();
  delay(STOP_DELAY);
}
/*Turn Left*/
void U_TURN() {
  motors.setSpeed(0);
  motors.stop();
  delay(1000);
  irsensorread();
  while (sensorValues[1] != 1) {
    irsensorread();
    motors.setSpeedA(RIGHT_TURN_SPEED);
    motors.forwardA();
    motors.setSpeedB(LEFT_TURN_SPEED);
    motors.backwardB();
  }
  motors.setSpeed(0);
  motors.stop();
  delay(STOP_DELAY);
}

void FOLLOW_THE_RULES() {
  irsensorread();
  //  SUM_OF_SENSOR_VALUES = sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] + sensorValues[4];
  if (sensorValues[0] == 1 && sensorValues[6] == 1) {
    Go_One_Step();
    if (sensorValues[0] == 1 && sensorValues[6] == 1) {
      motors.setSpeed(0);
      motors.stop();
      start_follow = !start_follow;
//      trackFinish = !trackFinish;
      return;
    }
    if (sensorValues[0] == 0 && sensorValues[6] == 0 && (sensorValues[2]==1 || sensorValues[3]==1 ||sensorValues[4]==1)) {
      PID_GO();
    }
  }

  if (sensorValues[0] == 1) {
    Go_One_Step();
    if (sensorValues[0] == 1 && sensorValues[6] == 1) {
      motors.setSpeed(0);
      motors.stop();
      start_follow = !start_follow;
//      trackFinish = !trackFinish;
      return;
    }
    if ( sensorValues[2] == 1 || sensorValues[3] == 1 || sensorValues[4] == 1 ) {
      PID_GO();
    }
    else {
      GO_LEFT();
    }
  }
  if (sensorValues[6] == 1) {
    Go_One_Step();
    if (sensorValues[2] == 1 || sensorValues[3] == 1 || sensorValues[4] == 1) {
      PID_GO();
    }
    else {
      GO_RIGHT();
    }
  }
//  if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && sensorValues[6] == 0) {
//    U_TURN();
//  }
}
