//Include Files
#include <Wire.h> //I2C Library
#include <Adafruit_PWMServoDriver.h>//PWM breakout board library
#include <math.h> // Math Library

//set PWM driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); 

//Definitions
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define BIGSERVOMAX  430 // This is the 'maximum' pulse length count (out of 4096)
#define SMALLSERVOMAX 350 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

/*
//Control Theory:

For Big servos:
BIGSERVOMAX - SERVOMIN = 280 pulse length count
280 = 180 degrees
Servo moves 0.643 degrees per pulse length count

For Small Servos:
350-150 = 200 = 90 degrees
Servo moves 0.45 degrees per pulse length count

*/

//Arm Lengths in m^-5
#define BaseOffset 890
#define ShoulderElbow 1500
#define ElbowWrist 1916.4
#define WristWrist 116.6
#define WristPenTip TBC

#define L1 BaseOffset
#define L2 ShoulderElbow
#define L3 ElbowWrist


//Servo connector position on breakout board
#define ShoulderRotate 0
#define ShoulderElevate 1
#define Elbow 2
#define WristRotate 3
#define WristElevate 4

//Number of positions for drawing
#define positionNum 14

//Array of all xy coordinates
float position[positionNum][3] = 
{
        {810,1590,90},
        {810,1590,0},
        {630,1770,0},
        {670,1950,0},
        {930,1690,0},
        {990,1850,0},
        {850,1990,0},
        {960,2080,0},
        {640,2080,0},
        {500,1700,0},
        {820,2490,0},
        {1130,1710,0},
        {1000,2080,0},
        {1000,2080,90}
};

//Empty array to be filled with all the angles
float angles [positionNum][4] = {};

//Array holding all current angles
int servoAngles[5] = {90,90,90,90,90};


//Array keeps the min and max angle allowed by all the servos, in degrees
int MinMax[5][2] =
{
  {0, 180},
  {70, 150},
  {30, 130},
  {0, 157},
  {0, 156},
};


void changepos(int sRot, int sLift, int elbow, int wRot, int wLift){
  int tempAngles [5] = {servoAngles[0]*1.555,servoAngles[1]*1.555,servoAngles[2]*1.555,servoAngles[3]*2.222,servoAngles[4]*2.222};
  int targets [5] = {sRot*1.555, sLift*1.555, elbow*1.555, wRot*2.222, wLift*2.222}; 
  int sRotchange = abs(servoAngles[0]-sRot); 
  int sLiftchange = abs(servoAngles[1]-sLift); 
  int elbowchange = abs(servoAngles[2]-elbow); 
  int wRotchange = abs(servoAngles[3]-wRot); 
  int wLiftchange = abs(servoAngles[4]-wLift); 
  int pulseses = 0;
  int smallest = min(sRotchange,min(sLiftchange,min(elbowchange,min(wRotchange,wLiftchange))));
  if (smallest == sRotchange || smallest == sLiftchange || smallest == sRotchange){
    pulseses = smallest/0.643;  
  }else{
    pulseses = smallest/0.45; 
  }
  int steps[5] = {(sRotchange*1.555)/pulseses,(sLiftchange*1.555)/pulseses,(elbowchange*1.555)/pulseses,(wRotchange*2.222)/pulseses,(wLiftchange*2.222)/pulseses};
  for (int i = 0; i <5, i++;){
    if (tempAngles[i] - targets[i] > 0){
      steps[i] = steps[i]*-1; 
    }
  }
  if (pulseses < 50)pulseses = 50;
  while (tempAngles[0] >= targets[0]-3 && tempAngles[0] <= targets[0]+3 && tempAngles[1] >= targets[1]-3 && tempAngles[1] <= targets[1]+3 && tempAngles[2] >= targets[2]-3 && tempAngles[2] <= targets[2]+3 && tempAngles[3] >= targets[3]-3 && tempAngles[3] <= targets[3]+3 && tempAngles[4] >= targets[4]-3 && tempAngles[4] <= targets[4]+3){
    for (int j = 0; j <5, j++;){
      if (tempAngles[j] >= targets[j]-3 && tempAngles[j] <= targets[j]+3);else{   //if the servo isn't close to the target yet, move by step
        pwm.setPWM(j, 0, SERVOMIN + tempAngles[j] + steps[j]);
        tempAngles[j] += steps[j];
      }
    }
  }

}


void goPreciseDegrees(int servo, int newPos) {
  int steps = 500;
  int target = constrain(newPos, MinMax[servo][0], MinMax[servo][1]);
  if (target != newPos) {
    Serial.print("Servo ");
    Serial.print(servo);
    Serial.println(" out of bounds, adjusted to limit.");
  }

  float angleStep = max(abs(servoAngles[servo] - target) / steps, 1);
  int Pulses;
  int tempAngle = servoAngles[servo];

  for (int i = 0; i < steps; i++) {
    if (tempAngle < target) {
      tempAngle += angleStep;

    } else {
      tempAngle -= angleStep;
    }

    if (servo < 3) {  // Big Servos
      Pulses = tempAngle / 0.643;
    } else {  // Small Servos
      Pulses = tempAngle/ 0.45;
    }

    pwm.setPWM(servo, 0, SERVOMIN + Pulses);
    delay(20);
  }
  servoAngles[servo] = target; // Update the stored position at the end

  Serial.println("Servo moved successfully");
}

void goPreciseAll(int sRot, int sLift, int elbow, int wRot, int wLift){
  int steps = 500;
  
  //Set the targets for all 5 joints
  int sRotT = constrain(sRot, MinMax[0][0], MinMax[0][1]);
  int sLiftT = constrain(sLift, MinMax[1][0], MinMax[1][1]);
  int elbowT = constrain(elbow, MinMax[2][0], MinMax[2][1]);
  int wRotT = constrain(wRot, MinMax[3][0], MinMax[3][1]);
  int wLiftT = constrain(wLift, MinMax[4][0], MinMax[4][1]);
  if (sRotT != sRot) {
    Serial.println("Shoulder Rotation Joint out of bounds, adjusted to limit.");
  }
  if (sLiftT != sLift) {
    Serial.println("Shoulder Elevation Joint out of bounds, adjusted to limit.");
  }
    if (elbowT != elbow) {
    Serial.println("Elbow Joint out of bounds, adjusted to limit.");
  }
  if (wRotT != wRot) {
    Serial.println("Wrist Rotation Joint out of bounds, adjusted to limit.");
  }
  if (wLift != wLift) {
    Serial.println("Wrist Elevation Joint out of bounds, adjusted to limit.");
  }

  //Set up other variables
  int target[5] = {sRotT, sLiftT,elbowT,wRotT, wLiftT};
  float angleStep[5];
  int pulses[5];
  int tempAngle[5];

  for(int i = 0; i< 5; i++){
    angleStep[i] = max(abs(servoAngles[i] - target[i]) / steps, 1);
    tempAngle[i] = servoAngles[i];
  }

  for (int i = 0; i < steps; i++) {
    for( int j = 0; j< 5; j++){ //This repeats the code for each of the 5 servos
      if (tempAngle[j] < target[j]) {
        tempAngle[j] += angleStep[j];

      } else {
        tempAngle[j] -= angleStep[j];
      }

      if (j < 3) {  // Big Servos
        pulses[j] = tempAngle[j] / 0.643;
      } else {  // Small Servos
        pulses[j] = tempAngle[j]/ 0.45;
      }

      servoAngles[j] = target[j]; // Update the stored position at the end
      pwm.setPWM(j, 0, SERVOMIN + pulses[j]);
    }
    delay(20);
  }
}

float srtest = 0;
float sltest = 0;
float etest = 0;
float wrtest = 0;
float wltest = 0;
void determineAngles(float * shoulderRotAngle, float* shoulderLiftAngle, float * elbowAngle, float * wristAngle, float x, float y){
// This gives us the angle from 0 that the shoulder rotational joint should be at.
  *shoulderRotAngle = 180/M_PI *atan(x/y); 
  
  // This gives us the distance from the center of the base in mm
  float distance = sqrt(sq(x)+sq(y)); 

  //This gives us the angle from horizontle of the shoulder angle.
  *shoulderLiftAngle = (sq(L2)+sq(distance)-sq(L3))/(2*L2*distance);
  *shoulderLiftAngle = 180 - 180/M_PI *acos(*shoulderLiftAngle); 

  //This gives us the angle between the Humerous and the forearm at the elbow
  *elbowAngle = 180 - 180/M_PI *acos((sq(L2)+sq(L3)-sq(distance))/(2*L2*L3));

  //This gives us the angle of the wrist such that the pencil remains vertical.
  *wristAngle = 90 - 180/M_PI *acos((sq(L3)+sq(distance)-sq(L2))/(2*distance*L3));
}

void servoInit(){
  int Pulses = 90/0.643;
  pwm.setPWM(0, 0, SERVOMIN+Pulses);
  pwm.setPWM(1, 0, SERVOMIN+Pulses);
  pwm.setPWM(2, 0, SERVOMIN+Pulses);
  Pulses = 90/.44;
  pwm.setPWM(3, 0, SERVOMIN+Pulses);
  pwm.setPWM(4, 0, SERVOMIN+Pulses);
}


void calculateAllAngles(){
  for(int i = 0; i<positionNum; i++){
    determineAngles(&angles[i][0],&angles[i][1],&angles[i][2],&angles[i][3],position[i][0],position[i][1]);
  }

}



void setup() {
  //Initialize the serial output (UART)
  Serial.begin(9600);
  Serial.println("Roco224 Robotic Arm -- Hugo Cabret");

  //Initialize the pwm
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);

  //Set all servos to 90 degrees
  servoInit();

  calculateAllAngles();

  Serial.println("Setup Complete");
  Serial.println("Enter servo number, angle, and steps: (e.g. 2 90)");
  delay(3000);
}


void loop() {
  //determineAngles(&srtest, &sltest, &etest, &wltest, 2000, 2000);
  //Serial.println(srtest);
  //Serial.println(sltest);
  //Serial.println(etest);
  //Serial.println(wltest);
  //goPreciseAll(srtest, sltest, etest, 90, wltest);
  //delay(3000);
  
  delay(1000);
  for(int i = 0; i< positionNum; i++){
    goPreciseAll(angles[i][0],angles[i][1],angles[i][2],position[i][2],angles[i][3]);
    Serial.println(i);
  }
/*
  if (Serial.available()) {
    int servoNum = Serial.parseInt();
    int angle = Serial.parseInt();

    if (servoNum >= 0 && servoNum < 5 && angle >= 0 && angle <= 180) {
      goPreciseDegrees(servoNum, angle);
      Serial.print("Servo ");
      Serial.print(servoNum);
      Serial.print(" set to ");
      Serial.print(angle);
      Serial.println(" degrees in 500 steps");

    } else {
      Serial.println("Invalid input. Use: servo angle steps (e.g., 2 90)");
    }
  }
  */  
}
