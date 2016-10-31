
//Manipulator code
#include "Arduino.h"
#include <VarSpeedServo.h>
#include <math.h>

//Dimensions of the arm
const float A = 5.75;
const float B = 7.375;

//Arm Servo pins
#define Base_pin 2
#define Shoulder_pin 3
#define Elbow_pin 4
#define Wrist_pin 10
#define Gripper_pin 11
#define WristR_pin 12

//Onboard Speaker
#define Speaker_pin 5

//Radians to Degrees constant
const float rtod = 57.295779;

//Arm Speed Variables
float Speed = 1.0;
int sps = 3;   //not really used

//Servo Objects with Special library that needs to be downloaded
VarSpeedServo Elb;
VarSpeedServo Shldr;
VarSpeedServo Wrist;
VarSpeedServo Base;
VarSpeedServo Gripper;
VarSpeedServo WristR;

//Arm Current Pos
float X = 4;
float Y = 4;
float Z = 90;
int G = 90;
float WA = 0;
int WR = 90;

//Arm temp pos
float tmpx = 4;
float tmpy = 4;
float tmpz = 90;
int tmpg = 90;
int tmpwr = 90;
float tmpwa = 0;

//boolean mode = true;

float prevX;   //step movement
float prevY;
float prevZ;
float prevWa;
float prevG;

void setup()
{
  Serial.begin(9600);
  Base.attach(Base_pin);
  Shldr.attach(Shoulder_pin);
  Elb.attach(Elbow_pin);
  Wrist.attach(Wrist_pin);
  Gripper.attach(Gripper_pin);
  WristR.attach(WristR_pin);
}

int Arm(float x, float y, float z, int g, float wa, int wr) //Here's all the Inverse Kinematics to control the arm
{
  float M = sqrt((y*y)+(x*x));
  if(M <= 0)
    return 1;
  float A1 = atan(y/x);
  if(x <= 0)
    return 1;
  float A2 = acos((A*A-B*B+M*M)/((A*2)*M));
  float Elbow = acos((A*A+B*B-M*M)/((A*2)*B));
  float Shoulder = A1 + A2;
  Elbow = Elbow * rtod;
  Shoulder = Shoulder * rtod;
  if((int)Elbow <= 0 || (int)Shoulder <= 0)
    return 1;
  float Wris = abs(wa - Elbow - Shoulder) - 90;

  Elb.writeMicroseconds(map(180 - Elbow, 0, 180, 900, 2100  ));
  Shldr.writeMicroseconds(map(Shoulder, 0, 180, 900, 2100));

  Elb.write(180 - Elbow, 30);
  Shldr.write(Shoulder, 30);

  Wrist.write(180 - Wris, 30);
  Base.write(z, 30);
  WristR.write(wr, 30);
#ifndef FSRG
  Gripper.write(g, 30);
#endif
  Y = tmpy;
  X = tmpx;
  Z = tmpz;
  WA = tmpwa;
#ifndef FSRG
  G = tmpg;
#endif
  return 0; 
}

// Number of positions to cycle through
const int numPos = 16;

// XYZ position of the gripper in relation to the base & wrist angle
float posListXYZWa[][4] =
{
  {3,2,99,0},    //initial
  {3,5,96,0},   //2 position open
  {3,5,96,0},   //3 close
  {5,4,96,0},   //4 up
  {5,4,130,0},  //5 go right
  {3,5,137,0},  //6 do down
  {3,5,130,0},  //7 open 
  {3,5,130,0},  //8 go up 
  {4,5,130,0},
  {5,4,130,0},  //9 go down
  {4,4,130,0},   //in between
  {3,5,130,0},  //10 close
  {3,5,130,0},  //11 go up
  {5,4,130,0},   
  {5,4,33,0} ,       //12 go left      
  {5,4,33,0}      //14 open
};

// Gripper closing and wrist rotation
int posListGWr[][2] =
{
  {108,90},  //1
  {24,90},   //2
  {120,90},  //3
  {120,90},  //4 
  {120,90},  //5
  {120,90},  //6
  {0,90},    //7
  {0,90}, 
  {0,90},          //8
  {0,90},    //9
  {0,90},     //in between
  {96,90},   //10
  {96,90},   //11
  {96,90},   //12
  {96,90},    //13
  {30,90}   //14
};

// Delay between moves
long posListDelay[] =
{
  3000,
  2000,
  1000,
  1000,
  3000,
  1000,
  500,
  500,
  500,
  500,
  500,
  500,
  3000,
  2000,
  2000,
  2000,
  2000
};
long lastReferenceTime;

void loop()
{
  // Example - Follow set of positions
  for(int i=0; i<numPos; i++)
  {

    // Set positions from array
    tmpx = posListXYZWa[i][0];
    tmpy = posListXYZWa[i][1];
    tmpz = posListXYZWa[i][2];
    tmpg = posListGWr[i][0];
    tmpwa = posListXYZWa[i][3];
    tmpwr = posListGWr[i][1];
    
    // Display position
    Serial.print("tmpx = "); Serial.print(tmpx, DEC); Serial.print("\ttmpy = "); Serial.print(tmpy, DEC); Serial.print("\ttmpz = "); Serial.print(tmpz, DEC); Serial.print("\ttmpg = "); Serial.print(tmpg, DEC); Serial.print("\ttmpwa = "); Serial.print(tmpwa, DEC); Serial.print("\ttmpwr = "); Serial.println(tmpwr, DEC);
    Serial.print("Delay = "); Serial.println(posListDelay[i], DEC); Serial.println("");
    
    // Move arm
    Arm(tmpx, tmpy, tmpz, tmpg, tmpwa, tmpwr);
    
    // Pause for the required delay
    lastReferenceTime = millis();
    while(millis() <= (posListDelay[i] + lastReferenceTime)){};
  }
}
