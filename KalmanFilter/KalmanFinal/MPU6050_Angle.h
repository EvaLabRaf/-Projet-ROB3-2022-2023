#ifndef MPU6050_ANGLE_H
#define MPU6050_ANGLE_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

class MPU6050_Angle
{
public:
  MPU6050_Angle();
  ~MPU6050_Angle();
  void begin();
  void readAngle();
  void readRowGyro();
  void readRowAcc();

private:
  static const int NumData = 7;
  static const int MPU_address = 0x68;          // Define MPU constant
  static const double pi;                       // Define pi constant
  int GyAccTemp[NumData];                       // Table with the raw values of the gyro and acc
  int GATCorr[NumData] = {0, 0, 0, 0, 0, 0, 0}; // Table with a correction that can be added to the read values
  double PitchRoll[3];                          // Table with the angles

  void ReadGY521(int *GyAccTempp, int *GATCorrr);
  void ComputeAngle(int *GyAccTempp, double *PitchRol);
  void endSerial();
};

#endif