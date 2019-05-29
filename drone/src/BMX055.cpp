#include <Arduino.h>
#include <Wire.h>
#include "BMX055.h"

// BMX055　加速度センサのI2Cアドレス
#define Addr_Accl 0x18
// BMX055　ジャイロセンサのI2Cアドレス
#define Addr_Gyro 0x68
// BMX055　磁気センサのI2Cアドレス
#define Addr_Mag 0x10


void BMX055::begin()
{
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x0F); // Select PMU_Range register
  Wire.write(0x03); // Range = +/- 2g
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x10); // Select PMU_BW register
  Wire.write(0x0A); // Bandwidth = 31.25 Hz
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x11); // Select PMU_LPW register
  Wire.write(0x00); // Normal mode, Sleep duration = 0.5ms
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x0F); // Select Range register
  Wire.write(0x02); // Full scale = +/- 500 degree/s
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x10); // Select Bandwidth register
  Wire.write(0x07); // ODR = 100 Hz
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x11); // Select LPM1 register
  Wire.write(0x00); // Normal mode, Sleep duration = 2ms
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B); // Select Mag register
  Wire.write(0x83); // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B); // Select Mag register
  Wire.write(0x01); // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4C); // Select Mag register
  Wire.write(0x07); // Normal Mode, ODR = 30 Hz
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4E); // Select Mag register
  Wire.write(0x84); // X, Y, Z-Axis enabled
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x51); // Select Mag register
  Wire.write(0x04); // No. of Repetitions for X-Y Axis = 9
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x52); // Select Mag register
  Wire.write(0x16); // No. of Repetitions for Z-Axis = 15
  Wire.endTransmission();
}

float BMX055::Accl(axis type)
{
  int data[6];
  for (int i = 0; i < 6; i++)
  {
    Wire.beginTransmission(Addr_Accl);
    Wire.write((2 + i)); // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Accl, 1); // Request 1 byte of data
    // Read 6 bytes of data
    // xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }

  float Accl = 0.00;
  switch (type) {
    // Convert the data to 12-bits
    case x:
      Accl = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
      break;

    case y:
      Accl = ((data[3] * 256) + (data[2] & 0xF0)) / 16;
      break;

    case z:
      Accl = ((data[5] * 256) + (data[4] & 0xF0)) / 16;
      break;
  }
  if (Accl > 2047)
    Accl -= 4096;
  Accl = Accl * 0.0098; // renge +-2g
  return Accl;
}


float BMX055::Gyro(axis type)
{
  int data[6];
  for (int i = 0; i < 6; i++)
  {
    Wire.beginTransmission(Addr_Gyro);
    Wire.write((2 + i)); // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Gyro, 1); // Request 1 byte of data
    // Read 6 bytes of data
    // xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  
  float Gyro = 0.00;
  switch (type) {
    // Convert the data
    case x:
      Gyro = (data[1] * 256) + data[0];
      break;

    case y:
      Gyro = (data[3] * 256) + data[2];
      break;

    case z:
      Gyro = (data[5] * 256) + data[4];
      break;
  }
  if (Gyro > 32767)
    Gyro -= 65536;
  Gyro = Gyro * 0.0038; //  Full scale = +/- 125 degree/s
  return Gyro;
}


int BMX055::Mag(axis type)
{
  int data[8];
  for (int i = 0; i < 8; i++)
  {
    Wire.beginTransmission(Addr_Mag);
    Wire.write((0x42 + i)); // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Mag, 1); // Request 1 byte of data
    // Read 6 bytes of data
    // xMag lsb, xMag msb, yMag lsb, yMag msb, zMag lsb, zMag msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  
  int Mag = 0;
  switch (type) {
    // Convert the data
    case x:
      Mag = ((data[1] << 8) | (data[0] >> 3));
      if (Mag > 4095)
        Mag -= 8192;
      break;

    case y:
      Mag = ((data[3] << 8) | (data[2] >> 3));
      if (Mag > 4095)
        Mag -= 8192;
      break;

    case z:
      Mag = ((data[5] << 8) | (data[4] >> 3));
      if (Mag > 16383)
        Mag -= 32768;
      break;
  }
  return Mag;
}
