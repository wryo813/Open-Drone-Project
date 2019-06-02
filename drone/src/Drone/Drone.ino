#include <WiFi.h>
#include <WiFiUDP.h>
#include <Wire.h>
#include <MadgwickAHRS.h>
#include <SparkFunBQ27441.h>
#include "serial_cmd.h"
#include "BMX055.h"


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


WiFiUDP udp;
Madgwick filter;
BMX055 IMU;

//アクセスポイント設定
const char *APSSID = "ESP32_wifi";
const char *APPASS = "esp32pass";
unsigned int localPort = 8888;

//受信データ
char packetBuffer[255];
static const char *udpReturnAddr = "192.168.4.2";
static const int udpReturnPort = 8889;

//バッテリー容量
const unsigned int BATTERY_CAPACITY = 1000;

// センサーの値を保存するグローバル関数


//MadgwickFilterの出力値
float roll = 0;
float pitch = 0;
float yaw = 0;

float dt, preTime;
float dps, a;

int i = 0;
int m1, m2, m3, m4, dps1;

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

  //MadgwickFilterのサンプリンレート。IMUのサンプリンレートよりも小さくする5Hz(MAX25Hz)
  filter.begin(8);
  // Wire(Arduino-I2C)の初期化
  Wire.begin();  //BMX055 初期化
  IMU.begin();
  Serial.println("IMU OK");

  //バッテリー残量IC初期化
  setupBQ27441();
  Serial.println("battery OK");

  delay(300);
  preTime = micros();
}

void loop()
{
  get_imu_data();

  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    int len = udp.read(packetBuffer, packetSize);

    //終端文字設定
    /*if (len > 0) packetBuffer[len] = '\0';
    Serial.print(udp.remoteIP());
    Serial.print(" / ");
    Serial.println(packetBuffer);*/
   
    //↓時間かかりすぎ 
    if (packetBuffer[i] == 'e') {
      packetBuffer[i] = '\0';
      //Serial.println(buf);

      m1 = atoi(strtok(packetBuffer, ","));
      m2 = atoi(strtok(NULL, ","));
      m3 = atoi(strtok(NULL, ","));
      m4 = atoi(strtok(NULL, ","));
      dps = atoi(strtok(NULL, ","));

      Serial.println(m1);
      Serial.println(m2);
      Serial.println(m3);
      Serial.println(m4);
      Serial.println(dps1);
      i = 0;
    } else {
      i++;
    }
    
    udp.beginPacket(udpReturnAddr, udpReturnPort);
    udp.print(printBatteryStats());
    udp.endPacket();
  }
}

//IMU生値をセンサーフュージョン
void get_imu_data()
{
  float gx = IMU.Gyro(x);
  float gy = IMU.Gyro(y);
  float gz = IMU.Gyro(z);
  float ax = IMU.Accl(x) / G;
  float ay = IMU.Accl(y) / G;
  float az = IMU.Accl(z) / G;
  float mx = IMU.Mag(x);
  float my = IMU.Mag(y);
  float mz = IMU.Mag(z);

  filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

  roll  = filter.getRoll();
  pitch = filter.getPitch();
  yaw   = filter.getYaw();

  //motor1_angle_now =  roll + pitch;
  //motor2_angle_now = -roll + pitch;
  //motor3_angle_now = -roll - pitch;
  //motor4_angle_now =  roll - pitch;
}

//角度を角速度へ変換
inline float degTodps(float deg)
{
  dt = (micros() - preTime) / 1000000;
  preTime = micros();

  dps = (deg - a) / dt;
  a = deg;

  if (dps >= 5000) {
    dps = dps - (360 / dt);
  }
  if (dps <= -5000) {
    dps = dps + (360 / dt);
  }

  return dps;
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
  String toPrint = String(soc) + ",";
  toPrint += String(volts) + ",";
  toPrint += String(current) + ",";
  toPrint += String(capacity) + ",";
  toPrint += String(fullCapacity) + ",";
  toPrint += String(power) + ",";
  toPrint += String(health) + ",";
  toPrint += String((temp / 10) - 273) + ",";
  toPrint += String((icTemp / 10) - 273) + ",";

  return toPrint;
}
