/*
* Created by Mannan Simo. 22-07-2017
* mannan.simo.91@gmail.com
*/

#include <ArduinoJson.h>
#include <Wire.h>
#include <NewPing.h>
#include <Adafruit_NeoPixel.h>

#define DEBUG_MODE true

const String deviceId = "KITT";

// defines for ultrasonic range sensor
#define ECHO_PIN 2
#define TRIGGER_PIN 3
#define MAX_DISTANCE 200

// define for LEDs
#define RGB_PIN 7

// Maximal revolutions per minute of motor
#define MAX_RPM 600

#define WHEEL_DIAMETER 0.42

//defines for motors
#define PWMA 6  //Left Motor Speed pin (ENA)
#define AIN2 A0 //Motor-L driveForward (IN2).
#define AIN1 A1 //Motor-L driveBackward (IN1)
#define PWMB 5  //Right Motor Speed pin (ENB)
#define BIN1 A2 //Motor-R driveForward (IN3)
#define BIN2 A3 //Motor-R driveBackward (IN4)

// define for PCF8574 I2C extender
#define Addr 0x20

// defines for buzzer
#define beep_on PCF8574Write(Addr, 0xDF & PCF8574Read(Addr))
#define beep_off PCF8574Write(Addr, 0x20 | PCF8574Read(Addr))

// define for on-board LED pin
#define LED_ONBOARD_PIN 13

// definitions for JSON objects
#define JSON_OUTPUT_BUFFER_SIZE 210
#define JSON_INPUT_BUFFER_SIZE 50

// Buffer for output JSON object
StaticJsonDocument<JSON_OUTPUT_BUFFER_SIZE> outputJsonDoc;

// Buffer for input JSON object
StaticJsonDocument<JSON_INPUT_BUFFER_SIZE> inputJsonDoc;

// Create an object to work with ultrasonic sensor
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Create an object to work with on-board LEDs
Adafruit_NeoPixel RGB = Adafruit_NeoPixel(4, RGB_PIN, NEO_GRB + NEO_KHZ800);

// define new type for system state representation
typedef struct
{
  //max possible RPM of motors
  int maxRPM;
  // curent RPM of motors
  int currentRPMLeftMotor;
  // curent RPM of motors
  int currentRPMRightMotor;
  // store number of main loop iterations
  unsigned long interationCounter;
  // info from ultra sonic sensor
  unsigned int ultraSonicDistance;
  // info from infrared left sensor
  bool isObstacleFromRightSide;
  // info from infrared left sensor
  bool isObstacleFromLeftSide;
  // indication of current bluetooth connect
  bool isBluetoothConnected;
  // value of the PWM frequency for left motor
  byte leftMotorFrequency;
  // value of the PWM frequency for right motor
  byte rightMotorFrequency;
  // store the last command from joystick
  byte joystickInputCommand;
  // indication of the robot's movement fact
  bool isMoves;
  // direction: 1 - f, 2 -l, 3 - r, 4 - b, 0 - s
  byte movementDirection;
  // speed of left wheel in cm
  float leftWheelSpeed;
  // speed of right wheel in cm
  float rightWheelSpeed;
  // start time of last iteration in milliseconds
  unsigned long startIterationTime;
  // end time of last iteration in milliseconds
  unsigned long endIterationTime;
  // duration of last iteration in milliseconds
  int lastIterationTimeinMs;
  // JSON object for system's output
  JsonObject outputJson;
  // JSON object for input stream
  JsonObject inputJson;
  // indicate if new command was entered and not executed
  bool isNewCommand;
  // representation of user command
  byte userCommandCode;
  // state of the onboard LED
  byte onboardLedState;
} system_state;

system_state my_robot_state = {
    .maxRPM = MAX_RPM,
    .currentRPMLeftMotor = 0,
    .currentRPMRightMotor = 0,
    .interationCounter = 0,
    .ultraSonicDistance = 0,
    .isObstacleFromRightSide = false,
    .isObstacleFromLeftSide = false,
    .isBluetoothConnected = false,
    .leftMotorFrequency = 50,
    .rightMotorFrequency = 50,
    .joystickInputCommand = 0xFF,
    .isMoves = false,
    .movementDirection = 0,
    .leftWheelSpeed = 0,
    .rightWheelSpeed = 0,
    .startIterationTime = 0,
    .endIterationTime = 0,
    .lastIterationTimeinMs = 0,
    .outputJson = outputJsonDoc.to<JsonObject>(),
    .inputJson = inputJsonDoc.to<JsonObject>(),
    .isNewCommand = false,
    .userCommandCode = 0,
    .onboardLedState = LOW};

void updateOutputJson()
{
  // update the values in output JSON object
  my_robot_state.outputJson["deviceId"] = deviceId;
  my_robot_state.outputJson["maxRPM"] = my_robot_state.maxRPM;
  my_robot_state.outputJson["currentRPMLeftMotor"] = my_robot_state.currentRPMLeftMotor;
  my_robot_state.outputJson["currentRPMRightMotor"] = my_robot_state.currentRPMRightMotor;
  my_robot_state.outputJson["interationCounter"] = my_robot_state.interationCounter;
  my_robot_state.outputJson["ultraSonicDistance"] = my_robot_state.ultraSonicDistance;
  my_robot_state.outputJson["isObstacleFromRightSide"] = my_robot_state.isObstacleFromRightSide;
  my_robot_state.outputJson["isObstacleFromLeftSide"] = my_robot_state.isObstacleFromLeftSide;
  my_robot_state.outputJson["isBluetoothConnected"] = my_robot_state.isBluetoothConnected;
  my_robot_state.outputJson["leftMotorFrequency"] = my_robot_state.leftMotorFrequency;
  my_robot_state.outputJson["rightMotorFrequency"] = my_robot_state.rightMotorFrequency;
  my_robot_state.outputJson["joystickInputCommand"] = my_robot_state.joystickInputCommand;
  my_robot_state.outputJson["isMoves"] = my_robot_state.isMoves;
  my_robot_state.outputJson["movementDirection"] = my_robot_state.movementDirection;
  my_robot_state.outputJson["leftWheelSpeed"] = my_robot_state.leftWheelSpeed;
  my_robot_state.outputJson["rightWheelSpeed"] = my_robot_state.rightWheelSpeed;
  my_robot_state.outputJson["startIterationTime"] = my_robot_state.startIterationTime;
  my_robot_state.outputJson["endIterationTime"] = my_robot_state.endIterationTime;
  my_robot_state.outputJson["lastIterationTimeinMs"] = my_robot_state.lastIterationTimeinMs;
  my_robot_state.outputJson["onboardLedState"] = my_robot_state.onboardLedState;
}

void toggleOnboardLED(int freq)
{
  byte onboard_led_state = digitalRead(LED_ONBOARD_PIN);
  
  // change LED state every "freq" iterations
  if (my_robot_state.interationCounter % freq) {
    if (onboard_led_state == HIGH) {
      onboard_led_state = LOW;
    } else {
      onboard_led_state = HIGH;
    }

    my_robot_state.onboardLedState = onboard_led_state;

    digitalWrite(LED_ONBOARD_PIN, onboard_led_state);
  }
}

void updateCurrentRPM()
{
  my_robot_state.currentRPMLeftMotor = calculateCurrentRPMofMotor(my_robot_state.leftMotorFrequency, MAX_RPM);
  my_robot_state.currentRPMRightMotor = calculateCurrentRPMofMotor(my_robot_state.rightMotorFrequency, MAX_RPM);
}

void updateWheelsSpeed()
{
  float leftWhlSpd=0, rightWhlSpd=0;
  
  if (my_robot_state.isMoves)
  {
    leftWhlSpd = calculateSpeedWheelInCm(my_robot_state.currentRPMLeftMotor);
    rightWhlSpd = calculateSpeedWheelInCm(my_robot_state.currentRPMLeftMotor);
  }
  
  my_robot_state.leftWheelSpeed = leftWhlSpd;
  my_robot_state.rightWheelSpeed = rightWhlSpd;
}

float calculateSpeedWheelInCm(int rpm)
{
  return 3.14 * rpm * WHEEL_DIAMETER;
}

int calculateCurrentRPMofMotor(int motorFreq, int max_rpm)
{
  return round(max_rpm * (float(motorFreq) / 255));
}

void configureMotorPins(byte motorA1, byte motorA2, byte motorB1, byte motorB2)
{
  digitalWrite(AIN1, motorA1);
  digitalWrite(AIN2, motorA2);
  digitalWrite(BIN1, motorB1);
  digitalWrite(BIN2, motorB2);
}

void configureMotorPWMFreq(byte leftMotorFreq, byte rightMotorFreq)
{
  analogWrite(PWMA, leftMotorFreq);
  analogWrite(PWMB, rightMotorFreq);
}

void driveForward()
{
  // update status of the robot movement
  my_robot_state.isMoves = true;

  configureMotorPWMFreq(my_robot_state.leftMotorFrequency, my_robot_state.rightMotorFrequency);
  configureMotorPins(LOW, HIGH, LOW, HIGH);

  if (my_robot_state.leftMotorFrequency > my_robot_state.rightMotorFrequency)
  {
    // forward right
    my_robot_state.movementDirection = 3;
  }
  else if (my_robot_state.leftMotorFrequency < my_robot_state.rightMotorFrequency)
  {
    // forward left
    my_robot_state.movementDirection = 2;
  }
  else
  {
    // straight forward
    my_robot_state.movementDirection = 1;
  }

  my_robot_state.movementDirection = 1;
}

void driveBackward()
{
  // update status of the robot movement
  my_robot_state.isMoves = true;

  configureMotorPWMFreq(my_robot_state.leftMotorFrequency, my_robot_state.rightMotorFrequency);
  configureMotorPins(HIGH, LOW, HIGH, LOW);

  if (my_robot_state.leftMotorFrequency > my_robot_state.rightMotorFrequency)
  {
    // backward right
    my_robot_state.movementDirection = 3;
  }
  else if (my_robot_state.leftMotorFrequency < my_robot_state.rightMotorFrequency)
  {
    // backward left
    my_robot_state.movementDirection = 2;
  }
  else
  {
    // straight backward
    my_robot_state.movementDirection = 4;
  }
}

void rotateRight()
{
  // update status of the robot movement
  my_robot_state.isMoves = true;

  configureMotorPWMFreq(my_robot_state.leftMotorFrequency, my_robot_state.rightMotorFrequency);
  configureMotorPins(LOW, HIGH, HIGH, LOW);

  my_robot_state.movementDirection = 3;
}

void rotateLeft()
{
  // update status of the robot movement
  my_robot_state.isMoves = true;

  configureMotorPWMFreq(my_robot_state.leftMotorFrequency, my_robot_state.rightMotorFrequency);
  configureMotorPins(HIGH, LOW, LOW, HIGH);

  my_robot_state.movementDirection = 2;
}

void stopMovement()
{
  // update status of the robot movement
  my_robot_state.isMoves = false;

  configureMotorPWMFreq(0, 0);
  configureMotorPins(LOW, LOW, LOW, LOW);

  my_robot_state.movementDirection = 0;
}

void PCF8574Write(byte addr, byte data)
{
  Wire.beginTransmission(addr);
  Wire.write(data);
  Wire.endTransmission();
}

byte PCF8574Read(byte addr)
{
  int data = -1;
  Wire.requestFrom(addr, 1);
  if (Wire.available())
  {
    data = Wire.read();
  }
  return data;
}

unsigned int doDistanceMeasurementInCm()
{
  // read the value from ultra sonic sensor
  unsigned int ultraSonicDistance = sonar.convert_cm(sonar.ping_median(4));

  /*
  Basically when stuck at zero,
  the echo pin is turned to output,
  given a low pulse then turned to input.
  */
  pinMode(ECHO_PIN, OUTPUT);
  digitalWrite(ECHO_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);

  return ultraSonicDistance;
}

void updateDistanceMeasurementResult()
{
  my_robot_state.ultraSonicDistance = doDistanceMeasurementInCm();
}

void updateLedIndication()
{
  //default indication mode - LEDs are switched off
  RGB.setPixelColor(0, RGB.Color(0, 0, 0));
  RGB.setPixelColor(1, RGB.Color(0, 0, 0));
  RGB.setPixelColor(2, RGB.Color(0, 0, 0));
  RGB.setPixelColor(3, RGB.Color(0, 0, 0));

  if (10 < my_robot_state.ultraSonicDistance && my_robot_state.ultraSonicDistance <= 20)
  {
    // set 2 and 3 LEDs to orange
    RGB.setPixelColor(1, RGB.Color(255, 40, 0));
    RGB.setPixelColor(2, RGB.Color(255, 40, 0));
  }
  if (my_robot_state.ultraSonicDistance <= 10)
  {
    // set 2 and 3 LEDs to red
    RGB.setPixelColor(1, RGB.Color(255, 0, 0));
    RGB.setPixelColor(2, RGB.Color(255, 0, 0));
  }

  if (my_robot_state.isObstacleFromRightSide == true)
  {
    // set 1 LED to orange
    RGB.setPixelColor(0, RGB.Color(255, 40, 0));
  }

  if (my_robot_state.isObstacleFromLeftSide == true)
  {
    // set 4 LED to orange
    RGB.setPixelColor(3, RGB.Color(255, 40, 0)); 
  }

  RGB.show();
}

void updateSerialOutput()
{
  serializeJsonPretty(my_robot_state.outputJson, Serial);
}

void updateLastIterationTimeinMs()
{
  my_robot_state.lastIterationTimeinMs = my_robot_state.endIterationTime - my_robot_state.startIterationTime;
}

void updateObstacleAvoidingInfo()
{
  /*
  set Pin High and read Pin value
  */
  PCF8574Write(Addr, 0xC0 | PCF8574Read(Addr));
  int value = PCF8574Read(Addr) | 0x3F;

  if (value != 255)
  {
    if (value == 191)
    {
      // an obstacle from right side
      my_robot_state.isObstacleFromLeftSide = false;
      my_robot_state.isObstacleFromRightSide = true;
    }
    else if (value == 127)
    {
      // an obstacle from left side
      my_robot_state.isObstacleFromLeftSide = true;
      my_robot_state.isObstacleFromRightSide = false;
    }
    else if (value == 63)
    {
      // an obstacle from left and right side
      my_robot_state.isObstacleFromLeftSide = true;
      my_robot_state.isObstacleFromRightSide = true;
    }
  }
  else
  {
    // no obsticles are observed
    my_robot_state.isObstacleFromLeftSide = false;
    my_robot_state.isObstacleFromRightSide = false;
  }
}

void updateJoystickInputCommand()
{
  /*
  * set Pin High and read Pin value
  */
  PCF8574Write(Addr, 0x1F | PCF8574Read(Addr));
  my_robot_state.joystickInputCommand = PCF8574Read(Addr) | 0xE0;
}

void getUserCommand()
{
  if (Serial.available())
  {
    DeserializationError error = deserializeJson(inputJsonDoc, Serial);
    if (!error)
    {
      my_robot_state.inputJson = inputJsonDoc.as<JsonObject>();
      byte code = my_robot_state.inputJson["userCommandCode"];

      // if there any problem 0 will be given
      if (code != 0)
      {
        my_robot_state.isNewCommand = true;
        my_robot_state.userCommandCode = code;
      }
    }
  }
}

void executeUserCommand()
{
  if (my_robot_state.isNewCommand)
  {
    my_robot_state.isNewCommand = false;

    switch (my_robot_state.userCommandCode)
    {
    case 1:
      driveForward();
      break;
    case 2:
      rotateLeft();
      break;
    case 3:
      rotateRight();
      break;
    case 4:
      driveBackward();
      break;
    case 5:
      stopMovement();
      break;
    case 6:
      // complete status of a robot
      updateSerialOutput();
      break;
    default:
      break;
    }
  }
}

void obstacleAvoidanceAlgorithm()
{
  if (2 < my_robot_state.ultraSonicDistance && my_robot_state.ultraSonicDistance < 5)
  {
    stopMovement();
  }
}

void initMotor()
{
  pinMode(PWMA, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
}

void setup()
{
  Serial.begin(115200);
  /*
  * Start RGB library and initialize all pixels to 'off'
  */
  RGB.begin();
  RGB.show();

  Wire.begin();

  // initialize pins for motors
  initMotor();

  // initialize pin for on-board LED
  pinMode(LED_ONBOARD_PIN, OUTPUT);
}

void loop()
{ 
  my_robot_state.interationCounter++;

  // remember time of an iteration's start
  my_robot_state.startIterationTime = millis();

  // try to get command from user
  getUserCommand();

  // get value from the ultrasonic range sensor
  updateDistanceMeasurementResult();

  // get info from infrared obstacle avoiding sensors
  updateObstacleAvoidingInfo();

  // get the command from joystick
  updateJoystickInputCommand();

  // set the indication mode of LEDs array
  updateLedIndication();

  // get value about current revolutions per minute of motor
  updateCurrentRPM();

  // calculate current
  updateWheelsSpeed();

  // interpreted a command from user
  executeUserCommand();

  // avoid potentially dangerous conditions during movement
  obstacleAvoidanceAlgorithm();

  // remember time of an iteration's end
  my_robot_state.endIterationTime = millis();

  // calculate delta time of last iteration
  updateLastIterationTimeinMs();

  // serialize the current system state to output JSON object
  updateOutputJson();

#if DEBUG_MODE
  // print debug data to serial interface
  updateSerialOutput();
#endif
}
