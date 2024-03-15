#include <Adafruit_PWMServoDriver.h>

// PWM Driver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo Channels
#define xServoChannel 0
#define yServoChannel 1
#define xServoChannel2 2
#define yServoChannel2 3
#define pincerServoChannel 4

#define enablePin 10

// Joystick and Potentiometer Pins
#define xPin A0 // Joystick 1, x
#define yPin A1 // Joystick 1, y
#define xPin2 A2 // Joystick 2, x
#define yPin2 A3 // Joystick 2, y
#define pincerPin A4 // Potentiometer 1 for pincer
#define speedPin A5 // Potentiometer 2 for arm speed

// Vars for controlling the servos movement with the Joystics
float xRate = 0.0;
float xSum = 0.0;
float yRate = 0.0;
float ySum = 0.0;
float xRate2 = 0.0;
float xSum2 = 0.0;
float yRate2 = 0.0;
float ySum2 = 0.0;

// Pincer vars
int pincerPos = 0;
int pincerAng = 90;

// Arm speed vars
float MovSpeed = 1;
float JoystickRate = 1;
float temp = 1;

// Arm posicion vars
int xPos = 0;
int yPos = 0;
int xPos2 = 0;
int yPos2 = 0;

// Initial arm posicion in degrees
int xAng = 130;
int yAng = 80;
int xAng2 = 25;
int yAng2 = 155;

void setup() {
  // Initialize PWM Driver
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  // Move servos to initial posicion
  pwm.setPWM(xServoChannel, 0, xAng);
  pwm.setPWM(yServoChannel, 0, yAng);
  pwm.setPWM(xServoChannel2, 0, xAng2);
  pwm.setPWM(yServoChannel2, 0, yAng2);
  pwm.setPWM(pincerServoChannel, 0, pincerAng);

  // Init serial communication
  Serial.begin(9600);

  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, HIGH);

}

void loop() {
  // Read Joysticks and potentiometers status
  xPos = analogRead(xPin);
  yPos = analogRead(yPin);
  xPos2 = analogRead(xPin2);
  yPos2 = analogRead(yPin2);
  pincerPos = analogRead(pincerPin);
  MovSpeed = analogRead(speedPin);

  // Speed of the arm
  JoystickRate = round(5 * MovSpeed / 1023.0);

  // Joysticks rates
  xRate = ((float)xPos - 510.0) / 510.0;
  yRate = ((float)yPos - 510.0) / 510.0;
  xRate2 = ((float)xPos2 - 510.0) / 510.0;
  yRate2 = ((float)yPos2 - 510.0) / 510.0;

  // Calculate servo movements
  xSum = xRate * JoystickRate;
  ySum = yRate2 * JoystickRate;
  xSum2 = xRate2 * JoystickRate;
  ySum2 = yRate * JoystickRate;

  // Adjust angles and clamp
  xAng = xAng - round(xSum);
  yAng = yAng - round(ySum);
  xAng2 = xAng2 + round(xSum2);
  yAng2 = yAng2 + round(ySum2);

  xAng = constrain(xAng, 10, 135);
  yAng = constrain(yAng, 10, 170);
  xAng2 = constrain(xAng2, 5, 140);
  yAng2 = constrain(yAng2, 10, 160);

  // Calculate pincer angle
  pincerAng = round(170.0 * ((float)pincerPos / 1023.0));
  pincerAng = constrain(pincerAng, 40, 120);

  // Move the servos to the indicated angles
  pwm.setPWM(xServoChannel, 0, xAng);
  pwm.setPWM(yServoChannel, 0, yAng);
  pwm.setPWM(xServoChannel2, 0, xAng2);
  pwm.setPWM(yServoChannel2, 0, yAng2);
  pwm.setPWM(pincerServoChannel, 0, pincerAng);

  // Give servos time to move
  delay(15);
}
