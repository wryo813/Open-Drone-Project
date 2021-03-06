//このソースコードの9-166行目は、秋月電子通商のAE-BMX055のサンプルプログラム[http://akizukidenshi.com/download/ds/akizuki/BMX055_20180510.zip]から引用し、一部変更を加えたものである。

#include <Arduino.h>
#include <Wire.h>
#include "BMX055.h"

//加速度センサのI2cアドレス
#define Addr_Accl 0x18
//ジャイロセンサのI2cアドレス
#define Addr_Gyro 0x68
//地磁気センサのI2cアドレス
#define Addr_Mag 0x10

//BMX055の設定
void BMX055::begin() {
  //Wire(Arduino-I2C)の初期化
  Wire.begin();
  
  //加速度センサレンジ設定
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x0F); // Select PMU_Range register
  Wire.write(0x08); // Range = +/- 8g
  Wire.endTransmission();
  delay(100);
  //加速度センサのフィルタ帯域幅設定
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x10); // Select PMU_BW register
  Wire.write(0x0B); // Bandwidth = 62.5 Hz
  Wire.endTransmission();
  delay(100);
  //加速度センサの電源モード設定
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x11); // Select PMU_LPW register
  Wire.write(0x00); // Normal mode, Sleep duration = 0.5ms
  Wire.endTransmission();
  delay(100);
  //ジャイロセンサレンジ設定
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x0F); // Select Range register
  Wire.write(0x02); // Full scale = +/- 500 degree/s
  Wire.endTransmission();
  delay(100);
  //ジャイロセンサのフィルタ帯域幅設定
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x10); // Select Bandwidth register
  Wire.write(0x07); // ODR = 100 Hz
  Wire.endTransmission();
  delay(100);
  //ジャイロセンサの電源モード設定
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x11); // Select LPM1 register
  Wire.write(0x00); // Normal mode, Sleep duration = 2ms
  Wire.endTransmission();
  delay(100);
  //地磁気センサのソフトリセットとSPIの設定
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B); // Select Mag register
  Wire.write(0x83); // Soft reset
  Wire.endTransmission();
  delay(100);
  //地磁気センサのソフトリセット
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B); // Select Mag register
  Wire.write(0x01); // Soft reset
  Wire.endTransmission();
  delay(100);
  //地磁気センサのアウトプットデータレート設定
  //MadgwickFilterのサンプリンレートより大きくする必要がある
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4C); // Select Mag register
  Wire.write(0x06); // Normal Mode, ODR = 25 Hz
  Wire.endTransmission();
  //地磁気センサの有効にする軸の設定
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4E); // Select Mag register
  Wire.write(0x84); // X, Y, Z-Axis enabled
  Wire.endTransmission();
  //地磁気センサのX/Y軸の読み取り回数設定
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x51); // Select Mag register
  Wire.write(0x04); // No. of Repetitions for X-Y Axis = 9
  Wire.endTransmission();
  //地磁気センサのZ軸の読み取り回数設定
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x52); // Select Mag register
  Wire.write(0x16); // No. of Repetitions for Z-Axis = 15
  Wire.endTransmission();
}

//BMX055から加速度センサデータを取得
void BMX055::begin_Accl() {
  int data[6];
  for (int i = 0; i < 6; i++)  {
    Wire.beginTransmission(Addr_Accl);
    Wire.write((2 + i));// Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Accl, 1);// Request 1 byte of data
    // Read 6 bytes of data
    // xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data to 12-bits
  xAccl = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
  if (xAccl > 2047)  xAccl -= 4096;
  yAccl = ((data[3] * 256) + (data[2] & 0xF0)) / 16;
  if (yAccl > 2047)  yAccl -= 4096;
  zAccl = ((data[5] * 256) + (data[4] & 0xF0)) / 16;
  if (zAccl > 2047)  zAccl -= 4096;
  xAccl = xAccl * 0.0098; // renge +-8g
  yAccl = yAccl * 0.0098; // renge +-8g
  zAccl = zAccl * 0.0098; // renge +-8g
}

//BMX055からジャイロセンサデータを取得
void BMX055::begin_Gyro() {
  int data[6];
  for (int i = 0; i < 6; i++) {
    Wire.beginTransmission(Addr_Gyro);
    Wire.write((2 + i));    // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Gyro, 1);    // Request 1 byte of data
    // Read 6 bytes of data
    // xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data
  xGyro = (data[1] * 256) + data[0];
  if (xGyro > 32767)  xGyro -= 65536;
  yGyro = (data[3] * 256) + data[2];
  if (yGyro > 32767)  yGyro -= 65536;
  zGyro = (data[5] * 256) + data[4];
  if (zGyro > 32767)  zGyro -= 65536;

  xGyro = xGyro * 0.0038; //  Full scale = +/- 500 degree/s
  yGyro = yGyro * 0.0038; //  Full scale = +/- 500 degree/s
  zGyro = zGyro * 0.0038; //  Full scale = +/- 500 degree/s
}

//BMX055から地磁気センサデータを取得
void BMX055::begin_Mag() {
  int data[8];
  for (int i = 0; i < 8; i++) {
    Wire.beginTransmission(Addr_Mag);
    Wire.write((0x42 + i));    // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Mag, 1);    // Request 1 byte of data
    // Read 6 bytes of data
    // xMag lsb, xMag msb, yMag lsb, yMag msb, zMag lsb, zMag msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data
  xMag = ((data[1] << 8) | (data[0] >> 3));
  if (xMag > 4095)  xMag -= 8192;
  yMag = ((data[3] << 8) | (data[2] >> 3));
  if (yMag > 4095)  yMag -= 8192;
  zMag = ((data[5] << 8) | (data[4] >> 3));
  if (zMag > 16383)  zMag -= 32768;
}

//引数の軸の加速度センサデータを返す
float BMX055::Accl(axis type) {
  float Accl = 0.00;

  switch (type) {
    case x:
      Accl = xAccl;
      break;

    case y:
      Accl = yAccl;
      break;

    case z:
      Accl = zAccl;
      break;
  }
  return Accl;
}

//引数の軸のジャイロセンサデータを返す
float BMX055::Gyro(axis type) {
  float Gyro = 0.00;
  switch (type) {
    case x:
      Gyro = xGyro;
      break;

    case y:
      Gyro = yGyro;
      break;

    case z:
      Gyro = zGyro;
      break;
  }
  return Gyro;
}

//引数の軸の地磁気センサデータを返す
int BMX055::Mag(axis type) {
  int Mag = 0;
  switch (type) {
    case x:
      Mag = xMag;
      break;

    case y:
      Mag = yMag;
      break;

    case z:
      Mag = zMag;
      break;
  }
  return Mag;
}
