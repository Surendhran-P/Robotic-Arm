#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create a servo driver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define the servo parameters
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 60 // Analog servos run at ~60 Hz updates

// Define the servo channels
#define SERVO_1 0
#define SERVO_2 1
#define SERVO_3 2

// Function prototype
void moveRoboticHand(int servo1Pos, int servo2Pos, int servo3Pos);

void setup() {

  Serial.begin(9600);
  
  // Initialize the servo driver
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~60 Hz updates
  
  pinMode(LED_BUILTIN, OUTPUT);
  delay(10);
  Serial.println("Type Something");
}

void loop() {

  if(Serial.available()){
    // Example: Move robotic hand to closed position
    moveRoboticHand(0, 180, 180);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100); // Delay for 1 second
  
    // Example: Move robotic hand to open position
    moveRoboticHand(90, 90, 90);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100); // Delay for 1 second
  }
}

// Function to move the robotic hand to specified positions
void moveRoboticHand(int servo1Pos, int servo2Pos, int servo3Pos) {
  // Move servos to specified positions
  pwm.setPWM(SERVO_1, 0, map(servo1Pos, 0, 180, SERVOMIN, SERVOMAX));
  pwm.setPWM(SERVO_2, 0, map(servo2Pos, 0, 180, SERVOMIN, SERVOMAX));
  pwm.setPWM(SERVO_3, 0, map(servo3Pos, 0, 180, SERVOMIN, SERVOMAX));
}
