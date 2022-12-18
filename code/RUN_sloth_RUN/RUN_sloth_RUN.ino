#include <AFMotor.h>    //Adafruit Motor Shield Library. First you must download and install AFMotor library
#include <QTRSensors.h> //Pololu QTR Sensor Library. First you must download and install QTRSensors library

AF_DCMotor motor1(4, MOTOR12_1KHZ ); //create motor #1 using M1 output on Motor Drive Shield, set to 1kHz PWM frequency
AF_DCMotor motor2(3, MOTOR12_1KHZ ); //create motor #2 using M2 output on Motor Drive Shield, set to 1kHz PWM frequency

#define KP 1  //experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define KD 5 //experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define M1_minumum_speed 100  //minimum speed of the Motor1
#define M2_minumum_speed 100  //minimum speed of the Motor2
#define M1_maksimum_speed 200 //max. speed of the Motor1
#define M2_maksimum_speed 200 //max. speed of the Motor2
#define MIDDLE_SENSOR 2       //number of middle sensor used
#define NUM_SENSORS 6         //number of sensors used
#define TIMEOUT 2500          //waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN QTR_NO_EMITTER_PIN         //emitterPin is the Arduino digital pin that controls whether the IR LEDs are on or off. Emitter is controlled by digital pin 2
#define calSpeed 80   // tune value motors will run while auto calibration sweeping turn (0-255)
#define turnSpeedSlow 50
#define LRturningSpeed 50
#define DEBUG 1

//sensors 0 through 5 are connected to analog inputs 0 through 5, respectively
QTRSensorsRC qtrrc((unsigned char[]) {
  A0, A1, A2, A3, A4, A5
} , NUM_SENSORS, TIMEOUT, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];
unsigned int line_position = 0; // value from 0-5000 to indicate position of line between sensor 0 - 5


void setup()
{
  Serial.begin(9600);
  delay(1500);
  calibration();
  set_motors(0, 0);
}

int lastError = 0;
int last_proportional = 0;
int integral = 0;

void loop()
{
  unsigned int sensors[5];
  int position = qtrrc.readLine(sensors); //get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  int error = position - 2500;

  int motorSpeed = KP * error + KD * (error - lastError);
  //int motorSpeed = KP * error;
  lastError = error;

  int leftMotorSpeed = M1_minumum_speed + motorSpeed;
  int rightMotorSpeed = M2_minumum_speed - motorSpeed;

  if (sensors[0] > 600 && sensors[1] > 600 && sensors[2] > 600 && sensors[3] > 600 && sensors[4] > 600 && sensors[5] > 600)
  {
    delay(100);
    qtrrc.readLine(sensors);
    if (sensors[0] > 600 && sensors[1] > 600 && sensors[2] > 600 && sensors[3] > 600 && sensors[4] > 600 && sensors[5] > 600)
    {
      // stop both motors
      motor1.run(RELEASE);
      motor2.run(RELEASE);
    }
  }
  else
  {
    set_motors(leftMotorSpeed, rightMotorSpeed);
  }
  // set motor speeds using the two motor speed variables above
  //

  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensors[i]);
    Serial.print('\t');
  }
  //Serial.println(); // uncomment this line if you are using raw values
  Serial.print(position); // comment this line out if you are using raw values
  Serial.print('\t');
  Serial.println(error);

}

void set_motors(int motor1speed, int motor2speed)
{
  if (motor1speed > M1_maksimum_speed ) motor1speed = M1_maksimum_speed;
  if (motor2speed > M2_maksimum_speed ) motor2speed = M2_maksimum_speed;
  if (motor1speed < 0) motor1speed = 0;
  if (motor2speed < 0) motor2speed = 0;
  motor1.setSpeed(motor1speed);
  motor2.setSpeed(motor2speed);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
}

//calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead
void calibration() {

  for (int i = 0; i <= 260; i++) // begin calibration cycle to last about 2.5 seconds (100*25ms/call)
  {
    // auto calibration sweeping left/right, tune 'calSpeed' motor speed at declaration
    if (i == 0 || i == 60 || i == 140 || i == 220) // slow sweeping turn right to pass sensors over line
    {
      motor1.setSpeed(calSpeed);
      motor1.run(BACKWARD);
      motor2.setSpeed(calSpeed);
      motor2.run(FORWARD);
    }
    else if (i == 20 || i == 100 || i == 180 || i == 260) // slow sweeping turn left to pass sensors over line
    {
      motor1.setSpeed(calSpeed);
      motor1.run(FORWARD);
      motor2.setSpeed(calSpeed);
      motor2.run(BACKWARD);
    }

    qtrrc.calibrate();  // reads all sensors with the define set 2500 microseconds (25 milliseconds) for sensor outputs to go low.

  }  // end calibration cycle

  // read calibrated sensor values and obtain a measure of the line position from 0 to 5000
  // To get raw sensor values, call:
  //  qtrrc.read(sensorValues); instead of unsigned int position = qtrrc.readLine(sensorValues);
  line_position = qtrrc.readLine(sensorValues);

  while (sensorValues[1] < 250) // wait for line position to near center
  {
    line_position = qtrrc.readLine(sensorValues);
  }

  // slow down speed
  motor1.setSpeed(turnSpeedSlow);
  motor2.setSpeed(turnSpeedSlow);

  // find center
  while (line_position > 3000 || line_position < 2000)  // wait for line position to find center
  {
    line_position = qtrrc.readLine(sensorValues);
  }

  // stop both motors
  motor1.run(RELEASE);
  motor2.run(RELEASE);

  // print calibration results
  // print the calibration minimum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  Serial.println("Calibration Complete");
  delay(1000);
}
