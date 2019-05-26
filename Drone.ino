#include <WiFi.h>
#include <WiFiUDP.h>
#include <Wire.h>
#include <MadgwickAHRS.h>
#include <SparkFunBQ27441.h>
#include "serial_cmd.h"

// BMX055　加速度センサのI2Cアドレス
#define Addr_Accl 0x18
// BMX055　ジャイロセンサのI2Cアドレス
#define Addr_Gyro 0x68
// BMX055　磁気センサのI2Cアドレス
#define Addr_Mag 0x10

//モーターピン設定
#define MOTOR1_PIN 12
#define MOTOR2_PIN 13
#define MOTOR3_PIN 14
#define MOTOR4_PIN 16

//PIDゲイン
#define P_GAIN 5
#define TARGET 0

//重力加速度
#define G 9.80665

//アクセスポイント設定
const char *APSSID = "ESP32_wifi";
const char *APPASS = "esp32pass";
unsigned int localPort = 8888;
WiFiUDP udp;
//受信データ
char packetBuffer[255];
static const char *udpReturnAddr = "192.168.4.2";
static const int udpReturnPort = 8889;

Madgwick MadgwickFilter;
unsigned long microsPerReading, microsPrevious;

const unsigned int BATTERY_CAPACITY = 1000;

// センサーの値を保存するグローバル関数
float xAccl = 0.00;
float yAccl = 0.00;
float zAccl = 0.00;
float xGyro = 0.00;
float yGyro = 0.00;
float zGyro = 0.00;
int xMag = 0;
int yMag = 0;
int zMag = 0;

float xf = 0;
float yf = 0;
float zf = 0;

float roll, pitch, yaw;
float motor1_angle_now, motor2_angle_now, motor3_angle_now, motor4_angle_now;

void setup()
{
  // デバック用シリアル通信は115200bps
  Serial.begin(115200);
  Serial.println("Seial OK");

  //アクセスポイント構築
  WiFi.softAP(APSSID, APPASS);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  Serial.println("Wi-Fi OK");
  //UDPサーバ構築
  udp.begin(localPort);
  Serial.println("UDP OK");

  //MadgwickFilterのサンプリンレートIMUのサンプリンレートよりも小さくする25Hz(MAX30Hz)
  MadgwickFilter.begin(25);
  // Wire(Arduino-I2C)の初期化
  Wire.begin();
  //BMX055 初期化
  BMX055_Init();
  Serial.println("IMU OK");

  //バッテリー残量IC初期化
  setupBQ27441();
  Serial.println("battery OK");

  delay(300);
}

void loop()
{
  float motor1_level;
  float motor2_level;
  float motor3_level;
  float motor4_level;
  float motor1_angle_deviation;
  float motor2_angle_deviation;
  float motor3_angle_deviation;
  float motor4_angle_deviation;
  int motor1_duty;
  int motor2_duty;
  int motor3_duty;
  int motor4_duty;

  get_imu_data();

  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    int len = udp.read(packetBuffer, packetSize);
    //終端文字設定
    if (len > 0) packetBuffer[len] = '\0';

    //Serial.print(udp.remoteIP());
    //Serial.print(" / ");
    //Serial.println(packetBuffer);
  }

  udp.beginPacket(udpReturnAddr, udpReturnPort);
  udp.print(printBatteryStats());
  udp.endPacket();

  motor1_angle_deviation = TARGET - motor1_angle_now;
  motor2_angle_deviation = TARGET - motor2_angle_now;
  motor3_angle_deviation = TARGET - motor3_angle_now;
  motor4_angle_deviation = TARGET - motor4_angle_now;

  motor1_level = P_GAIN * motor1_angle_deviation;
  motor2_level = P_GAIN * motor2_angle_deviation;
  motor3_level = P_GAIN * motor3_angle_deviation;
  motor4_level = P_GAIN * motor4_angle_deviation;

  motor1_duty = (int)motor1_level;
  motor2_duty = (int)motor2_level;
  motor3_duty = (int)motor3_level;
  motor4_duty = (int)motor4_level;

  if (motor1_duty >= 255)
    motor1_duty = 255;
  if (motor1_duty <= 0)
    motor1_duty = 0;
  if (motor2_duty >= 255)
    motor2_duty = 255;
  if (motor2_duty <= 0)
    motor2_duty = 0;
  if (motor3_duty >= 255)
    motor3_duty = 255;
  if (motor3_duty <= 0)
    motor3_duty = 0;
  if (motor4_duty >= 255)
    motor4_duty = 255;
  if (motor4_duty <= 0)
    motor4_duty = 0;

}

void get_imu_data()
{
  //BMX055 加速度の読み取り
  BMX055_Accl();
  //BMX055 ジャイロの読み取り
  BMX055_Gyro();
  //BMX055 磁気の読み取り
  BMX055_Mag();

  xf = (float)xMag;
  yf = (float)yMag;
  zf = (float)zMag;

  MadgwickFilter.update(xGyro, yGyro, zGyro, xAccl / G, yAccl / G, zAccl / G, xMag, yMag, zMag);

  roll  = MadgwickFilter.getRoll();
  pitch = MadgwickFilter.getPitch();
  yaw   = MadgwickFilter.getYaw();

  motor1_angle_now =  roll + pitch;
  motor2_angle_now = -roll + pitch;
  motor3_angle_now = -roll - pitch;
  motor4_angle_now =  roll - pitch;
}

//=====================================================================================//
void BMX055_Init()
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

//=====================================================================================//
void BMX055_Accl()
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
  // Convert the data to 12-bits
  xAccl = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
  if (xAccl > 2047)
    xAccl -= 4096;
  yAccl = ((data[3] * 256) + (data[2] & 0xF0)) / 16;
  if (yAccl > 2047)
    yAccl -= 4096;
  zAccl = ((data[5] * 256) + (data[4] & 0xF0)) / 16;
  if (zAccl > 2047)
    zAccl -= 4096;
  xAccl = xAccl * 0.0098; // renge +-2g
  yAccl = yAccl * 0.0098; // renge +-2g
  zAccl = zAccl * 0.0098; // renge +-2g
}

//=====================================================================================//
void BMX055_Gyro()
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
  // Convert the data
  xGyro = (data[1] * 256) + data[0];
  if (xGyro > 32767)
    xGyro -= 65536;
  yGyro = (data[3] * 256) + data[2];
  if (yGyro > 32767)
    yGyro -= 65536;
  zGyro = (data[5] * 256) + data[4];
  if (zGyro > 32767)
    zGyro -= 65536;

  xGyro = xGyro * 0.0038; //  Full scale = +/- 125 degree/s
  yGyro = yGyro * 0.0038; //  Full scale = +/- 125 degree/s
  zGyro = zGyro * 0.0038; //  Full scale = +/- 125 degree/s
}

//=====================================================================================//
void BMX055_Mag()
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
  // Convert the data
  xMag = ((data[1] << 8) | (data[0] >> 3));
  if (xMag > 4095)
    xMag -= 8192;
  yMag = ((data[3] << 8) | (data[2] >> 3));
  if (yMag > 4095)
    yMag -= 8192;
  zMag = ((data[5] << 8) | (data[4] >> 3));
  if (zMag > 16383)
    zMag -= 32768;
}


void setupBQ27441(void)
{
  // Use lipo.begin() to initialize the BQ27441-G1A and confirm that it's
  // connected and communicating.
  if (!lipo.begin()) // begin() will return true if communication is successful
  {
    // If communication fails, print an error message and loop forever.
    Serial.println("Error: Unable to communicate with BQ27441.");
    Serial.println("  Check wiring and try again.");
    Serial.println("  (Battery must be plugged into Battery Babysitter!)");
    while (1) ;
  }
  Serial.println("Connected to BQ27441!");

  // Uset lipo.setCapacity(BATTERY_CAPACITY) to set the design capacity
  // of your battery.
  lipo.setCapacity(BATTERY_CAPACITY);
}

String printBatteryStats()
{
  // Read battery stats from the BQ27441-G1A
  unsigned int soc = lipo.soc();  // Read state-of-charge (%)
  unsigned int volts = lipo.voltage(); // Read battery voltage (mV)
  int current = lipo.current(AVG); // Read average current (mA)
  unsigned int fullCapacity = lipo.capacity(FULL); // Read full capacity (mAh)
  unsigned int capacity = lipo.capacity(REMAIN); // Read remaining capacity (mAh)
  int power = lipo.power(); // Read average power draw (mW)
  int health = lipo.soh(); // Read state-of-health (%)
  int temp = lipo.temperature(BATTERY);
  int icTemp = lipo.temperature(INTERNAL_TEMP);

  // Now print out those values:
  String toPrint = String(soc) + "% | ";
  toPrint += String(volts) + " mV | ";
  toPrint += String(current) + " mA | ";
  toPrint += String(capacity) + " / ";
  toPrint += String(fullCapacity) + " mAh | ";
  toPrint += String(power) + " mW | ";
  toPrint += String(health) + "% | ";
  toPrint += String((temp / 10) - 273) + "℃ | ";
  toPrint += String((icTemp / 10) - 273) + "℃";

  return toPrint;
}
