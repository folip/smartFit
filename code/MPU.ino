#include <Kalman.h>
#include <Wire.h>
#include <Math.h>

//IO
float fRad2Deg = 57.295779513f; //将弧度转为角度的乘数
const int MPU = 0x68; //MPU-6050的I2C地址
const int nValCnt = 7; //一次读取寄存器的数量

//校准系列
const int nCalibTimes = 1000; //校准时读数的次数
int calibData[nValCnt]; //校准数据
int displayType = 0;
const int dCN = 1000;
int displayCount = dCN;
unsigned long displayStartT; // 注意数据的格式！！
//时间的分析
unsigned long nLastTime = 0; //上一次读数的时间
//只使用重力计算Roll Pitch，用于卡尔曼滤波
float fLastRoll = 0.0f; //上一次滤波得到的Roll角
float fLastPitch = 0.0f; //上一次滤波得到的Pitch角
Kalman kalmanRoll; //Roll角滤波器
Kalman kalmanPitch; //Pitch角滤波器

void setup() {
  Serial.begin(9600); //初始化串口，指定波特率
  Wire.begin(); //初始化Wire库
  WriteMPUReg(0x6B, 0); //启动MPU6050设备

  Calibration(); //执行校准
  nLastTime = micros(); //记录当前时间
}

void loop() {
  //基本没什么东西的循环1000次0.01s
  int readouts[nValCnt];
  ReadAccGyr(readouts); //读出测量值
  // 1000次花费时间0.28s
  float realVals[7];
  Rectify(readouts, realVals); //根据校准的偏移量进行纠正
  //1000次花费时间0.1s

  /*--------------------------------------时间分析模块------------------------------*/
  //计算两次测量的时间间隔dt，以秒为单位
  unsigned long nCurTime = micros();
  float dt = (double)(nCurTime - nLastTime) / 1000000.0;
  //micros返回的读数就是多少us 经测试没毛病
  //更新本次测的时间
  nLastTime = nCurTime;
  if(displayType == 1){
    if(displayCount == dCN){
      displayStartT = nCurTime;
      Serial.print("start time:");Serial.print(displayStartT);Serial.print("\n");
    }
    if(displayCount != 0){
     // Serial.print("dt: ");Serial.print(dt);Serial.print("\n");
      //Serial.print("current time:");Serial.print(nCurTime);Serial.print("\n");
      displayCount--;
    }
     else{
      Serial.print("1000 times finish\n");
      Serial.print("End time: ");Serial.print(nCurTime);Serial.print("\n");
      Serial.print("Div: ");Serial.print(nCurTime - displayStartT);Serial.print("\n");
      Serial.print("1000dt: ");Serial.print((double)(nCurTime - displayStartT) / 1000000.0);Serial.print("\n");
      displayType = 0;
      displayCount = dCN;
      //还原显示计数 
     }
  }

/*-------------------------------------------DMP---------------------------------------------*/  
  float halfT = dt/2;
  AHRSupdate(realVals[4],realVals[5],realVals[6],realVals[0],realVals[1],realVals[2],halfT);
  
/*----------------------只靠重力计算Row Pitch角-------------------------*/
  
  //计算加速度向量的模长，均以g为单位
  float fNorm = sqrt(realVals[0] * realVals[0] + realVals[1] * realVals[1] + realVals[2] * realVals[2]);
  float fRoll = GetRoll(realVals, fNorm); //计算Roll角
  if (realVals[1] > 0) {
    fRoll = -fRoll;
  }
  float fPitch = GetPitch(realVals, fNorm); //计算Pitch角
  if (realVals[0] < 0) {
    fPitch = -fPitch;
  }

  //对Roll角和Pitch角进行卡尔曼滤波
  float fNewRoll = kalmanRoll.getAngle(fRoll, realVals[4], dt);
  float fNewPitch = kalmanPitch.getAngle(fPitch, realVals[5], dt);
  //跟据滤波值计算角度速
  float fRollRate = (fNewRoll - fLastRoll) / dt;
  float fPitchRate = (fNewPitch - fLastPitch) / dt;
   //更新Roll角和Pitch角
  fLastRoll = fNewRoll;
  fLastPitch = fNewPitch;

/*-----------------------------------------串口输入控制--------------------------------------*/
  if(Serial.available() > 0){
      int serialData = Serial.read();
      if('a' == serialData) displayAcc(realVals);
      if('r' == serialData) {
          Serial.print("Roll:");
          Serial.print(fNewRoll); Serial.print('(');
          Serial.print(fRollRate); Serial.print("),\tPitch:");
          Serial.print(fNewPitch); Serial.print('(');
          Serial.print(fPitchRate); Serial.print(")\n");
          delay(10);
      }
      //旧时代的roll和pitch输出
      if('t' == serialData) displayTem(realVals);
      if('o' == serialData) displayOmega(realVals);
      if('c' == serialData) {
        Calibration();
        AHRSreset();
        /*Euler的测试
        displayType = 2;
        Euler(1,0,0,0);
        Euler(0.8404,0.1907,0.4700,0.1907);
        Euler(0.7861,0.1675,0.5709,0.1675);
        Euler(0.7071,0.0003,0.7071,0.0003);
        Euler(0.701,0,0.701,0);
        */
        displayType = 0;
      } 
      if('d' == serialData){
        displayType = 1;
      }
      if('e' == serialData){
        displayType = 0;
        displayCount = dCN;  
      }
      if('q' == serialData){
        displayType = 2;
        displayCount = 10;
      }
   }
}
/* -------------------------- MPU IO and print function-------------------------------*/ 

void displayAcc(float *realVals){
  Serial.print("Acc_x: ");Serial.print(realVals[0]);Serial.print("\n");
  Serial.print("Acc_y: ");Serial.print(realVals[1]);Serial.print("\n");
  Serial.print("Acc_z: ");Serial.print(realVals[2]);Serial.print("\n");
}

void displayTem(float * realVals){
  Serial.print("Temp: ");Serial.print(realVals[3]);Serial.print("\n");
}

void displayOmega(float *realVals){
  Serial.print("Omega_x: ");Serial.print(realVals[4]);Serial.print("\n");
  Serial.print("Omega_y: ");Serial.print(realVals[5]);Serial.print("\n");
  Serial.print("Omega_z: ");Serial.print(realVals[6]);Serial.print("\n");
}


//向MPU6050写入一个字节的数据
//指定寄存器地址与一个字节的值
void WriteMPUReg(int nReg, unsigned char nVal) {
  Wire.beginTransmission(MPU);
  Wire.write(nReg);
  Wire.write(nVal);
  Wire.endTransmission(true);
}

//从MPU6050读出一个字节的数据
//指定寄存器地址，返回读出的值
unsigned char ReadMPUReg(int nReg) {
  Wire.beginTransmission(MPU);
  Wire.write(nReg);
  Wire.requestFrom(MPU, 1, true);
  Wire.endTransmission(true);
  return Wire.read();
}

//从MPU6050读出加速度计三个分量、温度和三个角速度计
//保存在指定的数组中
void ReadAccGyr(int *pVals) {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.requestFrom(MPU, nValCnt * 2, true);
  Wire.endTransmission(true);
  for (long i = 0; i < nValCnt; ++i) {
    pVals[i] = Wire.read() << 8 | Wire.read();
  }
}

/* -------------------------- 校准与纠偏-------------------------------*/ 
//对大量读数进行统计，校准平均偏移量
void Calibration()
{
  float valSums[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0};
  //先求和
  for (int i = 0; i < nCalibTimes; ++i) {
    int mpuVals[nValCnt];
    ReadAccGyr(mpuVals);
    for (int j = 0; j < nValCnt; ++j) {
      valSums[j] += mpuVals[j];
    }
  }
  //再求平均
  for (int i = 0; i < nValCnt; ++i) {
    calibData[i] = valSums[i]/ nCalibTimes;
  }
  calibData[2] -= 16384; //设芯片Z轴竖直向上，设定静态工作点。 //!!!
  Serial.print("calibData\n");
  for(int i=0;i<7;i++){
    Serial.print("\n");
    Serial.print(calibData[i]); // print("  " ) 8224 //!!!
  }
}

//对读数进行纠正，消除偏移，并转换为物理量。公式见文档。
void Rectify(int *pReadout, float *pRealVals) {
  for (int i = 0; i < 3; ++i) {
    pRealVals[i] = (float)(pReadout[i] - calibData[i]) / 16384.0f;
  }
  pRealVals[3] = pReadout[3] / 340.0f + 36.53;
  for (int i = 4; i < 7; ++i) {
    pRealVals[i] = (float)(pReadout[i] - calibData[i]) / 131.0f;
  }
}

/* -------------------------- 纯重力法测姿态角-------------------------------*/ 
//算得Roll角。算法见文档。
float GetRoll(float *pRealVals, float fNorm) {
  float fNormXZ = sqrt(pRealVals[0] * pRealVals[0] + pRealVals[2] * pRealVals[2]);
  float fCos = fNormXZ / fNorm;
  return acos(fCos) * fRad2Deg;
}

//算得Pitch角。算法见文档。
float GetPitch(float *pRealVals, float fNorm) {
  float fNormYZ = sqrt(pRealVals[1] * pRealVals[1] + pRealVals[2] * pRealVals[2]);
  float fCos = fNormYZ / fNorm;
  return acos(fCos) * fRad2Deg;
}
/* -------------------------- DMP算法测姿态角-------------------------------*/ 

// 加速度计、陀螺仪数据融合，更新四元数
/*
   [gx,gy,gz]为陀螺仪的测量值
   [ax,at,az]为加速度的测量值
*/
//关键常量定义
#define Kp 2.0f
#define Ki 0.005f
//#define halfT 0.0018f

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float exInt = 0, eyInt = 0, ezInt = 0;
void AHRSreset(){
  q0 = 1, q1 = 0, q2 = 0, q3 = 0;
  exInt = 0, eyInt = 0, ezInt = 0;
  Serial.print("DMP reset done\n");
}

void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az,float halfT) 
{  
            float norm;               
            float hx, hy, hz; //转换到b系的重力加速度
            float vx, vy, vz, wx, wy, wz;  
            float ex, ey, ez;  
            
            // 定义一些辅助变量用于转换矩阵
            float q0q0 = q0*q0;  
            float q0q1 = q0*q1;  
            float q0q2 = q0*q2;  
            float q0q3 = q0*q3;  
            float q1q1 = q1*q1;  
            float q1q2 = q1*q2;  
            float q1q3 = q1*q3;  
            float q2q2 = q2*q2;  
            float q2q3 = q2*q3;  
            float q3q3 = q3*q3;  

            if(0 == displayType){
              Serial.print("Omega_x: ");Serial.print(gx);Serial.print("\n");
              Serial.print("Omega_y: ");Serial.print(gy);Serial.print("\n");
              Serial.print("Omega_z: ");Serial.print(gz);Serial.print("\n");
            }
            /*----------------------------重力补偿---------------------------*/     
            // 归一化加速度计和地磁计的度数 
            norm = sqrt(ax*ax + ay*ay + az*az);   
            ax = ax / norm;  
            ay = ay / norm;  
            az = az / norm;  
           
            /*                     
            //n系中重力加速度[0,0,1]转换到b系中得到三个分量[vx,vy,vz]        
            vx = 2*(q1q3 - q0q2);  
            vy = 2*(q0q1 + q2q3);  
            vz = q0q0 - q1q1 - q2q2 + q3q3;    
             
            //计算[ax,at,az] X [vx,vy,vz]，得到两个误差后求和
            ex = (ay*vz - az*vy);  
            ey = (az*vx - ax*vz);  
            ez = (ax*vy - ay*vx);  

            if(displayType == 2){
              Serial.print("error to correct with gravity");
              Serial.print(ex);Serial.print("\n");
              Serial.print(ey);Serial.print("\n");
              Serial.print(ez);Serial.print("\n");
            }
            
            //PI控制器中的积分部分
            exInt = exInt + ex*Ki;  
            eyInt = eyInt + ey*Ki;  
            ezInt = ezInt + ez*Ki;  
            
            //误差经过PI控制器后输出,然后补偿到角速度的三个分量，Kp、Ki是需要调节的参数
            gx = gx + Kp*ex + exInt;  
            gy = gy + Kp*ey + eyInt;  
            gz = gz + Kp*ez + ezInt;
            */               
             /*----------------------------四元数更新与归一化------------------------------*/   
            //一阶龙格库塔法更新四元数  
            q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;  
            q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;  
            q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;  
            q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;    
             
            // 归一化四元数
            norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);  
            q0 = q0 / norm;  
            q1 = q1 / norm;  
            q2 = q2 / norm;  
            q3 = q3 / norm;  

            if(displayType == 0){
              Serial.print("qs:  ");
              Serial.print(q0);Serial.print(" ");
              Serial.print(q1);Serial.print(" ");
              Serial.print(q2);Serial.print(" ");
              Serial.print(q3);Serial.print("\n");
              Euler(q0,q1,q2,q3);
              displayCount--;
              if(0 == displayCount){
                displayType = 0;
                displayCount = 1000;
              }
            }
} 

float eulerX, eulerY, eulerZ;
void Euler(float qw, float qx, float qy, float qz){
 /*  
    eulerX = atan2(2 * (qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz);
    eulerY = asin(-2 * (qx*qz - qw*qy));
    eulerZ = atan2(2 * (qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz);
 */

    eulerX = fRad2Deg * atan2(2 * (qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz);
    eulerY = fRad2Deg * asin(-2 * (qx*qz - qw*qy));
    eulerZ = fRad2Deg * atan2(2 * (qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz);
    
    if(0 == displayType){
      Serial.print("Eulers:  ");
      Serial.print(eulerX);Serial.print(" ");
      Serial.print(eulerY);Serial.print(" ");
      Serial.print(eulerZ);Serial.print("\n");
    }
}
// ref: https://blog.csdn.net/MOU_IT/article/details/80391216
