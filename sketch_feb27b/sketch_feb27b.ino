#include <Wire.h> //I2C Library
#include <Adafruit_PWMServoDriver.h>//PWM breakout board library
#include <math.h> // Math Library

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); 

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define BIGSERVOMAX  430 // This is the 'maximum' pulse length count (out of 4096)
#define SMALLSERVOMAX 350 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
//For Big servos:
//BIGSERVOMAX - SERVOMIN = 280 pulse length count
//280 = 180 degrees
// Servo moves 0.643 degrees per pulse length count

//For Small Servos:
//350-150 = 200 = 90 degrees
//Servo moves 0.45 degrees per pulse length count
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


int positionNumber = 10;

float position[458][3] = 
{
  {1,1},
  {1,1},
  {1,1},
  {1,1},
  {1,1},
  {1,1},
  {1,1},
  {1,1},
  {1,1},
  {1,1},
};

float angles [10][4];




//Array to hold current position of each servo
int positions[5] = {90,90,90,90,90};
int targets[5];

//Function to move specified servo to specified degrees
void GoDegrees(int servo, int degrees){
  if(degrees>=0 && degrees <181){
    positions[servo] = degrees;
    if (servo<3 && servo>=0){
      int Pulses = degrees/0.643;
      pwm.setPWM(servo, 0, SERVOMIN+Pulses);
    }else if(servo == 3 || servo == 4){
      int Pulses = degrees/0.44;
      pwm.setPWM(servo, 0, SERVOMIN+Pulses);
    }
  }
}

//Setup Function 
void setup() {
  calculateAllAngles();
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);
}

void loop() {/*
  GoDegrees(ShoulderRotate, 90);
  GoDegrees(ShoulderElevate, 90);
  GoDegrees(Elbow, 90);
  GoDegrees(WristRotate, 90);
  GoDegrees(WristElevate, 90);
  delay(500);
    */
  for(int i = 0; i< positionNumber; i++){

    setAllServos(angles[i][0],angles[i][1],angles[i][2],angles[i][3],position[i][2]);
  }
  

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

void determineAngles(float * shoulderLiftAngle, float * shoulderRotAngle, float * elbowAngle, float * wristAngle, float x, float y){
  
  // This gives us the angle form 0 that the shoulder rotational joint should be at.
  *shoulderRotAngle = tan(x/y); 

  // This gives us the distance from the center of the base in mm
  float distance = x/sin(*shoulderRotAngle); 

  //This gives us the angle from horizontle of the shoulder angle.
  *shoulderLiftAngle = acos((ShoulderElbow*ShoulderElbow+distance*distance-ElbowWrist*ElbowWrist)/(2*ShoulderElbow*distance)); 

  //This converts shoulderLiftAngle to be the angle from vertical
  *shoulderLiftAngle = 90-*shoulderLiftAngle;

  //This gives us the angle between the Humerous and the forearm at the elbow
  *elbowAngle = acos((ShoulderElbow*ShoulderElbow-ElbowWrist*ElbowWrist-distance*distance)/(2*ShoulderElbow*ElbowWrist)); 

  //This gives us the angle of the wrist such that the pencil remains vertical.
  *wristAngle = 90-(180-*elbowAngle-*shoulderLiftAngle);
}

void calculateAllAngles(){
  for(int i = 0; i<positionNumber; i++){
    determineAngles(&angles[i][0],&angles[i][1],&angles[i][2],&angles[i][3],position[i][0],position[i][1]);
  }

}

void setAllServos(int angle1, int angle2, int angle3, int angle4, int angleDraw){
  GoDegrees(ShoulderRotate,angle1);
  GoDegrees(ShoulderElevate,angle2);
  GoDegrees(Elbow, angle3);
  GoDegrees(WristElevate, angle4);
  GoDegrees(WristRotate, angleDraw);
}


