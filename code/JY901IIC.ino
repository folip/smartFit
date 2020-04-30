#include <Wire.h>
#include "JY901.h"
CJY901 MPU0 = CJY901();
CJY901 MPU1 = CJY901();
//CJY901 MPU2 = CJY901();
//CJY901 MPU3 = CJY901();
//CJY901 MPU4 = CJY901();

#define selectOutPin0 2
#define selectOutPin1 3
#define selectOutPin2 4

#define motorOutPin0 5
#define motorOutPin1 6

#define pressureInPin A0
#define curvatureInPin A1

bool s0 = 0;
bool s1 = 0;
bool s2 = 0;
//channel 1
bool sx0 = 1;
bool sx1 = 0;
bool sx2 = 0;
//channel 2
bool bothOn = 0;
//measure two channels together?

int pressureVal = 0;
int pressureVal2 = 0;
int curvatureVal = 0; 

void setup() 
{
  Serial.begin(9600);
  IICIntl();
  pinMode(selectOutPin0,OUTPUT);
  pinMode(selectOutPin1,OUTPUT);
  pinMode(selectOutPin2,OUTPUT); // select signals
  pinMode(motorOutPin0, OUTPUT);
  pinMode(motorOutPin1, OUTPUT);
  selectChannel();
} 

void loop() 
{
  //print received data. Data was received in serialEvent;
  motorCoolDown();
  readCurvature();
  readPressure();
  readAngle();
  report();
  react();
  delay(100);
}

void IICIntl(){
  MPU0.StartIIC(0x50);
  MPU1.StartIIC(0x51);
}

void readAngle(){
  MPU0.GetAngle();
  MPU1.GetAngle();
}

void report(){
  Serial.print((float)MPU0.stcAngle.Angle[0]/32768*180);Serial.print(" ");
  Serial.print((float)MPU0.stcAngle.Angle[1]/32768*180);Serial.print(" ");
  Serial.print((float)MPU1.stcAngle.Angle[0]/32768*180);Serial.print(" ");
  Serial.print((float)MPU1.stcAngle.Angle[1]/32768*180);Serial.print(" ");
  Serial.print(curvatureVal);Serial.print(" ");
  Serial.print(pressureVal);Serial.print(" ");
  if(bothOn){
    Serial.print(pressureVal2);Serial.print(" ");
  }
  Serial.println("");
}

bool settingChannel1 = 0;
bool settingChannel2 = 0;
void react(){
    if(Serial.available() > 0){
      int serialData = Serial.read();
      Serial.print("char received ");
      Serial.println(char(serialData));
      if(1 == settingChannel1){
        if('0' == serialData){s2 = 0; s1 = 0; s0 = 0;}
        else if('1' == serialData){s2 = 0; s1 = 0; s0 = 1;}
        else if('2' == serialData){s2 = 0; s1 = 1; s0 = 0;}
        else if('3' == serialData){s2 = 0; s1 = 1; s0 = 1;}
        else if('4' == serialData){s2 = 1; s1 = 0; s0 = 0;}
        else if('5' == serialData){s2 = 1; s1 = 0; s0 = 1;}
        else if('6' == serialData){s2 = 1; s1 = 1; s0 = 0;}
        else if('7' == serialData){s2 = 1; s1 = 1; s0 = 1;}
        Serial.print(s2);Serial.print("-");
        Serial.print(s1);Serial.print("-");
        Serial.print(s0);Serial.print("-");
        Serial.println("Channel1_set.");
        

        selectChannel();
        settingChannel1 = 0;
        return;
      }
      if(1 == settingChannel2){
        if('0' == serialData){sx2 = 0; sx1 = 0; sx0 = 0;}
        else if('1' == serialData){sx2 = 0; sx1 = 0; sx0 = 1;}
        else if('2' == serialData){sx2 = 0; sx1 = 1; sx0 = 0;}
        else if('3' == serialData){sx2 = 0; sx1 = 1; sx0 = 1;}
        else if('4' == serialData){sx2 = 1; sx1 = 0; sx0 = 0;}
        else if('5' == serialData){sx2 = 1; sx1 = 0; sx0 = 1;}
        else if('6' == serialData){sx2 = 1; sx1 = 1; sx0 = 0;}
        else if('7' == serialData){sx2 = 1; sx1 = 1; sx0 = 1;}
        Serial.print(sx2);Serial.print("-");
        Serial.print(sx1);Serial.print("-");
        Serial.print(sx0);Serial.println("-Channel2_set.");
        settingChannel2 = 0;
        return;
      }
      switch(serialData){
        case(','):{
          settingChannel1 = 1;
          Serial.println("setting_channel_1");
          break;
        }
        case('.'):{
          settingChannel2 = 1;
          Serial.println("setting_channel_2");
          break;
        };
        case('c'):{
          bothOn = 0;
          Serial.println("channel_2_closed");
          selectChannel();
          break;
        };
        case('o'):{
          bothOn = 1;
          Serial.println("channel_2_open");
          break;
        };
        case('l'):{
          leftMotor();
          break;
        };
        case('r'):{
          rightMotor();
          break;
        };
        default: return;  
      }
   }
}

void readCurvature(){
  curvatureVal = analogRead(curvatureInPin);
  // and the curvature value;
}

void readPressure(){
  if(0 == bothOn){
    pressureVal = analogRead(pressureInPin);
  }
  else{
    pressureVal = analogRead(pressureInPin);
    selectChannelX();
    delay(25);
    pressureVal2 = analogRead(pressureInPin);
    selectChannel();
  }
}

void selectChannel(){
  digitalWrite(selectOutPin0,s0);
  digitalWrite(selectOutPin1,s1);
  digitalWrite(selectOutPin2,s2);
}

void selectChannelX(){
  digitalWrite(selectOutPin0,sx0);
  digitalWrite(selectOutPin1,sx1);
  digitalWrite(selectOutPin2,sx2);
}

void reportPC(){
  Serial.print(curvatureVal);Serial.print(" ");
  Serial.print(pressureVal);Serial.print(" ");
  if(bothOn){
    Serial.print(pressureVal2);Serial.print(" ");
  }
  Serial.println("");
}

float t = 0;
void reportTest(){
  Serial.print(int(500 + 500 * cos(t/57)));Serial.print(" ");
  Serial.print(int(500 + 500 * sin(t/57)));Serial.print(" ");
  if(bothOn){
    Serial.print(int(500 + 500 * sin(2*t/57)));Serial.print(" ");
  }
  Serial.println("");
  t++;
}

int tLM = 0;
void leftMotor(){
  tLM = 4;
  digitalWrite(motorOutPin0,HIGH);
  Serial.println("left_motor_activated");
}

int tRM = 0;
void rightMotor(){
  tRM = 4;
  Serial.println("right_motor_activated");
  digitalWrite(motorOutPin1,HIGH);
}

void motorCoolDown(){
  //Serial.println(tRM);
  if(tLM!=0) tLM--;
  if(tLM == 0) digitalWrite(motorOutPin0,LOW);
  if(tRM!=0) tRM--;
  if(tRM == 0) digitalWrite(motorOutPin1,LOW);
}
