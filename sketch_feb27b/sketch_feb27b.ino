#include <Wire.h> //I2C Library
#include <Adafruit_PWMServoDriver.h>//PWM breakout board library

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); 

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  430 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
//SERVOMAX - SERVOMIN = 280 pulse length count
//280 = 180 degrees
// Servo moves 0.643 degrees per pulse length count

//Link Lengths in mm
#define BaseOffset 89
#define ShoulderElbow 150
#define ElbowWrist 191.64
#define WristWrist 11.66
#define WristPenTip TBC

//Defines for servo motors, relating to connector position on breakout board
#define ShoulderRotate 0
#define ShoulderElevate 1
#define Elbow 2
#define WristRotate 3
#define WristElevate 4

//Array to hold current position of each servo
int positions[5] = {90,90,90,90,90};

//Function to move specified servo to specified degrees
void GoDegrees(int servo, int degrees){
  if(degrees>=0 && degrees <181){
    positions[servo] = degrees
    int Pulses = degrees/0.643;
    pwm.setPWM(servo, 0, SERVOMIN+Pulses);
  }
}

//Function to move specified servo by a specified number of degrees.
void MoveBy(int servo, int degrees){
  if(degrees>=0 && degrees <181){
    if 
    int Pulses = degrees/0.643;
    pwm.setPWM(servo, 0, SERVOMIN+Pulses);
  }
}

//Setup Function 
void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);
}

void loop() {
  GoDegrees(ShoulderRotate, 0);
  GoDegrees(ShoulderElevate, 0);
  GoDegrees(Elbow, 0);
  delay(1000);
  GoDegrees(ShoulderRotate, 90);
  GoDegrees(ShoulderElevate, 90);
  GoDegrees(Elbow, 90);
  delay(500);

/*
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    pwm.setPWM(ShoulderRotate, 0, pulselen);
  }
  Serial.println("8 channel Servo test!");
  delay(2000);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(ShoulderRotate, 0, pulselen);
  }
  delay(2000);
  Serial.println("8 channel Servo test done!");
  */
}

