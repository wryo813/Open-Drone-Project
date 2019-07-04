/*
   メモ
   DTP603450を使ってモータを回転させると3Aでリミッターがかかって供給を停止する
   USB_STATUS_PINを使うとモーターからのノイズでモーターが一時的に停止することがある
   PW_SWITCH_PINを使うとモーターからのノイズで勝手にシャットダウンすることがある

   優先順位
   P_GAINの最適化
   PID制御の実装
   throttle_target_rawが0の時のみ電源スイッチを有効にする
   バッテリーステータスの送信
   高度センサ

   あきらめたこと
   カメラの実装
   DTP603450を使うこと
   USB使用時のモーター回転禁止
*/

#include <WiFi.h>
#include <WiFiUDP.h>
#include <MadgwickAHRS.h>
#include <SparkFunBQ27441.h>
#include "serial_cmd.h"
#include "BMX055.h"
#include "split.h"

//電源管理ピン設定
#define SHUTDOWN_PIN 25
#define PW_SWITCH_PIN 26
#define USB_STATUS_PIN 34
#define CHG_STATUS_PIN 35

//モーターピン設定
#define MOTOR1_PIN 27
#define MOTOR2_PIN 16
#define MOTOR3_PIN 4
#define MOTOR4_PIN 14

//バッテリー容量
#define BATTERY_CAPACITY 1000

/*
   PWM設定について
   f=1/T,T=1/fより
   1/80[MHz]=12.5[ns]
   PWM_FREQ[Hz]=1/(12.5[ns]×2^PWM_BIT[bit])
*/
//PWM周波数[Hz]
#define PWM_FREQ 19531
//PWM分解能[bit]
#define PWM_BIT 12

//PIDゲイン
#define P_GAIN 10
#define I_GAIN 1
#define D_GAIN 1
#define TARGET 0

//yaw軸調整
#define YAW_P_GAIN 10

//スロットル調整
#define T_GAIN 0.5

//重力加速度
#define G 9.80665

//受信したトークン数
#define n 6

WiFiUDP udp;
Madgwick filter;
BMX055 IMU;

//アクセスポイント設定
const char *APSSID = "ESP32_wifi";
const char *APPASS = "esp32pass";

//ドローンのポート
unsigned int localPort = 8888;

//受信データ
char packetBuffer[255];

//送信機のIPアドレス
static const char *udpReturnAddr = "192.168.4.2";
//送信機のポート
static const int udpReturnPort = 8889;

//MadgwickFilterの出力値
float roll = 0;
float pitch = 0;
float yaw = 0;

//モーター角度
float motor1_angle_now = 0;
float motor2_angle_now = 0;
float motor3_angle_now = 0;
float motor4_angle_now = 0;

//角度-角速度変換用
float dps, preDeg;

//ループ時間計測関数用
float loop_time, preTime;

//バッテリーが接続されると、Trueになる
bool battery_status = 0;

void setup()
{
  //GPIO入出力設定
  //電源関連
  pinMode(PW_SWITCH_PIN, INPUT);
  pinMode(USB_STATUS_PIN, INPUT);
  pinMode(CHG_STATUS_PIN, INPUT);
  pinMode(SHUTDOWN_PIN, OUTPUT);
  //モーターピン
  pinMode(MOTOR1_PIN, OUTPUT);
  pinMode(MOTOR2_PIN, OUTPUT);
  pinMode(MOTOR3_PIN, OUTPUT);
  pinMode(MOTOR4_PIN, OUTPUT);

  //使わないピン
  pinMode(13, INPUT_PULLUP);
  pinMode(17, INPUT_PULLUP);
  pinMode(32, INPUT_PULLUP);
  pinMode(33, INPUT_PULLUP);

  digitalWrite(MOTOR1_PIN, LOW);
  digitalWrite(MOTOR2_PIN, LOW);
  digitalWrite(MOTOR3_PIN, LOW);
  digitalWrite(MOTOR4_PIN, LOW);

  //PWM設定
  ledcSetup(0, PWM_FREQ, PWM_BIT);
  ledcSetup(1, PWM_FREQ, PWM_BIT);
  ledcSetup(2, PWM_FREQ, PWM_BIT);
  ledcSetup(3, PWM_FREQ, PWM_BIT);

  //ピンをチャンネルに接続
  ledcAttachPin(MOTOR1_PIN, 0);
  ledcAttachPin(MOTOR2_PIN, 1);
  ledcAttachPin(MOTOR3_PIN, 2);
  ledcAttachPin(MOTOR4_PIN, 3);

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
  filter.begin(25);

  // Wire(Arduino-I2C)の初期化
  Wire.begin();

  //BMX055 初期化
  IMU.begin();
  Serial.println("IMU OK");

  //バッテリー残量IC初期化
  setupBQ27441();
  Serial.println("battery OK");

  Serial.println("Setting complete");
  Serial.print("Setting time = ");
  Serial.print(micros());
  Serial.println(" us");

  delay(300);

  //電源スイッチ割込み(サブルーチン関数内で一定以上ループするとリブートする)
  // attachInterrupt(digitalPinToInterrupt(PW_SWITCH_PIN), shutdown_pw, RISING);

  preTime = micros();
}

void loop()
{
  int motor1_angle_target_raw = 0;
  int motor2_angle_target_raw = 0;
  int motor3_angle_target_raw = 0;
  int motor4_angle_target_raw = 0;
  int angular_velocity_target_raw = 0;
  int throttle_target_raw = 0;

  int motor1_angle_target = 0;
  int motor2_angle_target = 0;
  int motor3_angle_target = 0;
  int motor4_angle_target = 0;
  int angular_velocity_target = 0;
  unsigned int throttle_target = 0;

  int motor1_duty_raw = 0;
  int motor2_duty_raw = 0;
  int motor3_duty_raw = 0;
  int motor4_duty_raw = 0;

  int angular_velocity = 0;

  get_posture();

  int packetSize = udp.parsePacket();

  if (packetSize > 0)
  {
    int len = udp.read(packetBuffer, packetSize);
    String cmds[n] = {"\0"};

    //終端文字設定
    if (len > 0)
    {
      //分割された文字列を格納する配列
      packetBuffer[len] = '\0';
    }

    // 分割数 = 分割処理(文字列, 区切り文字, 配列)
    int index = split(packetBuffer, ',', cmds);

    //if (index != 6) {
    //通信失敗処理
    //}

    motor1_angle_target_raw = cmds[0].toInt();
    motor2_angle_target_raw = cmds[1].toInt();
    motor3_angle_target_raw = cmds[2].toInt();
    motor4_angle_target_raw = cmds[3].toInt();
    angular_velocity_target_raw = cmds[4].toInt();
    throttle_target_raw = cmds[5].toInt();
  }

  angular_velocity_target = angular_velocity_target_raw * 500 / 2048;

  //複数loopTime()を使うとうまくいかない
  angular_velocity = (angular_velocity_target - degTodps(yaw)) * YAW_P_GAIN;

  Serial.println(throttle_target_raw);

  /*if (throttle_target_raw > 0) {
    //目標姿勢角の上限を設定
    //motor1_angle_target = motor1_angle_target_raw * 3 / 130;
    //Serial.println(motor1_angle_target_raw);
    //スロットルをduty比に変換
    //throttle_target = throttle_target_raw / 10;

    //int motor1_duty_raw = throttle_target_raw + (motor1_angle_target_raw - motor1_angle_now) * P_GAIN;

    //if (motor1_duty_raw < 0) {
    //  motor1_duty_raw = 0;
    //}
    // Serial.println(throttle_target_raw);
    }*/

  motor1_angle_target = motor1_angle_target_raw * 30 / 1258;
  motor2_angle_target = motor2_angle_target_raw * 30 / 1258;
  motor3_angle_target = motor3_angle_target_raw * 30 / 1258;
  motor4_angle_target = motor4_angle_target_raw * 30 / 1258;

  motor1_duty_raw = throttle_target_raw * T_GAIN + (motor1_angle_target - motor1_angle_now) * P_GAIN + angular_velocity;
  motor2_duty_raw = throttle_target_raw * T_GAIN + (motor2_angle_target - motor2_angle_now) * P_GAIN - angular_velocity;
  motor3_duty_raw = throttle_target_raw * T_GAIN + (motor3_angle_target - motor3_angle_now) * P_GAIN + angular_velocity;
  motor4_duty_raw = throttle_target_raw * T_GAIN + (motor4_angle_target - motor4_angle_now) * P_GAIN - angular_velocity;

  if (motor1_duty_raw < 0)
    motor1_duty_raw = 0;
  if (motor2_duty_raw < 0)
    motor2_duty_raw = 0;
  if (motor3_duty_raw < 0)
    motor3_duty_raw = 0;
  if (motor4_duty_raw < 0)
    motor4_duty_raw = 0;

  if (throttle_target_raw <= 0)
  {
    motor1_duty_raw = 0;
    motor2_duty_raw = 0;
    motor3_duty_raw = 0;
    motor4_duty_raw = 0;
  }

  /*Serial.print(motor1_duty_raw);
    Serial.print(" ");
    Serial.print(motor2_duty_raw);
    Serial.print(" ");
    Serial.print(motor3_duty_raw);
    Serial.print(" ");
    Serial.println(motor4_duty_raw);
  */

  //モーターノイズが入ってうまく動かない
  /*if (digitalRead(USB_STATUS_PIN) == HIGH) {
    motor1_duty_raw = 0;
    motor2_duty_raw = 0;
    motor3_duty_raw = 0;
    motor4_duty_raw = 0;
    }*/

  ledcWrite(0, motor1_duty_raw);
  ledcWrite(1, motor2_duty_raw);
  ledcWrite(2, motor3_duty_raw);
  ledcWrite(3, motor4_duty_raw);

  //udp.beginPacket(udpReturnAddr, udpReturnPort);
  //udp.print(printBatteryStats());
  //udp.endPacket();
  //Serial.println("loopTime = " + (String)loopTime());

  /*if (digitalRead(PW_SWITCH_PIN) == HIGH) {
    shutdown_pw();
    }*/
}

void shutdown_pw()
{
  while (digitalRead(PW_SWITCH_PIN) == HIGH)
  {
  }
  digitalWrite(SHUTDOWN_PIN, HIGH);
}

//IMU生値をセンサーフュージョン
void get_posture()
{
  IMU.begin_Accl();
  IMU.begin_Gyro();
  IMU.begin_Mag();
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

  roll = filter.getRoll();
  pitch = filter.getPitch();
  yaw = filter.getYaw();

  motor1_angle_now = roll + pitch;
  motor2_angle_now = roll - pitch;
  motor3_angle_now = -roll - pitch;
  motor4_angle_now = -roll + pitch;
}

//ループ時間計測関数
inline float loopTime()
{
  loop_time = (micros() - preTime) / 1000000;
  preTime = micros();

  return loop_time;
}

//角度を角速度へ変換
inline float degTodps(float deg)
{
  dps = (deg - preDeg) / loopTime();
  preDeg = deg;

  if (dps >= 10000)
  {
    dps = dps - (360 / loopTime());
  }
  if (dps <= -(10000))
  {
    dps = dps + (360 / loopTime());
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
    while (1)
      ;
  }
  Serial.println("Connected to BQ27441!");

  // Uset lipo.setCapacity(BATTERY_CAPACITY) to set the design capacity
  // of your battery.
  battery_status = lipo.setCapacity(BATTERY_CAPACITY);
}

String printBatteryStats()
{
  // Read battery stats from the BQ27441-G1A
  unsigned int soc = lipo.soc();                   // Read state-of-charge (%)
  unsigned int volts = lipo.voltage();             // Read battery voltage (mV)
  int current = lipo.current(AVG);                 // Read average current (mA)
  unsigned int fullCapacity = lipo.capacity(FULL); // Read full capacity (mAh)
  unsigned int capacity = lipo.capacity(REMAIN);   // Read remaining capacity (mAh)
  int power = lipo.power();                        // Read average power draw (mW)
  int health = lipo.soh();                         // Read state-of-health (%)
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
