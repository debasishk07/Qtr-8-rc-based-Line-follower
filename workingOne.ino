#include <QTRSensors.h>

#define Kp 0.055
#define Kd 2.6
// I have made the Ki value to 0, because it doesnot impact much on the code or the test run of the vehicle.

#define rightMaxSpeed 180
#define leftMaxSpeed 180

#define rightBaseSpeed 50
#define leftBaseSpeed 50

#define rightMotor1 11
#define rightMotor2 12
#define rightMotorPWM 9

#define leftMotor1 6
#define leftMotor2 5
#define leftMotorPWM 3  

#define buttonPin 2 // Change this to the pin you connect the push button 
// This push button is used to start or stop the vehicle, press is once to start and again press once to stop.
// connect the pin 2 with ground with a swtich. if you have any confusion then refer the connection diagram.

QTRSensors qtr;

const uint8_t SensorCount = 8;
unsigned int sensorValues[SensorCount];

int lastError = 0;
bool isRunning = false;

void setup()
{
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);

  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);

  pinMode(buttonPin, INPUT_PULLUP);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  calibrate(); // Calibrate once during setup

  delay(1000);
}

void loop()
{
  static bool prevButtonState = HIGH;
  bool currentButtonState = digitalRead(buttonPin);

  if (currentButtonState == LOW && prevButtonState == HIGH)
  {
    // Button is pressed once
    isRunning = !isRunning; // Toggle the running state
    digitalWrite(LED_BUILTIN, isRunning ? LOW : HIGH);

    if (isRunning)
    {
      delay(500); // 0.5-second delay after pressing the button to start
    }
    else
    {
      delay(500); // 0.5-second delay after pressing the button to stop
      wait(); // Stop the motors
    }
  }

  prevButtonState = currentButtonState;

  if (isRunning)
  {
    unsigned int position = qtr.readLineBlack(sensorValues)-500;// adjust the subtracted values to get a tuned calibration.
    //you can also use the sensor calibration code to test the values from the qtr sensor
    int error = position - 4000;// then add the difference in it. The default value was 3500 as I have subtracted 500 from the
    // sensor value so I have added it to the sensor error position value.
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;
    int rightMotorSpeed = rightBaseSpeed + motorSpeed;
    int leftMotorSpeed = leftBaseSpeed - motorSpeed;

    if (rightMotorSpeed > rightMaxSpeed)
      rightMotorSpeed = rightMaxSpeed;
    if (leftMotorSpeed > leftMaxSpeed)
      leftMotorSpeed = leftMaxSpeed;
    if (rightMotorSpeed < 0)
      rightMotorSpeed = 0;
    if (leftMotorSpeed < 0)
      leftMotorSpeed = 0;

    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
    analogWrite(rightMotorPWM, rightMotorSpeed);

    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
    analogWrite(leftMotorPWM, leftMotorSpeed);
  }
}

void calibrate()
{
  digitalWrite(LED_BUILTIN, HIGH);
  for (int i = 0; i < 100; i++)
  {
    if (i < 25 || i >= 75)
    {
      turn_right();
    }
    else
    {
      turn_left();
    }
    qtr.calibrate();
    delay(20);
  }
  wait();
  digitalWrite(LED_BUILTIN, LOW);
}

void wait()
{
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);

  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);
}

void turn_left()
{
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);
  analogWrite(rightMotorPWM, rightBaseSpeed);

  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, HIGH);
  analogWrite(leftMotorPWM, leftBaseSpeed);
}

void turn_right()
{
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(rightMotorPWM, rightBaseSpeed);

  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM, leftBaseSpeed);
}
