/*
   メモ
   オイラー角で姿勢角送信
   DTP603450を使ってモータを回転させると3Aでリミッターがかかって供給を停止する
   USB_STATUS_PINを使うとモーターからのノイズでモーターが一時的に停止することがある
   PW_SWITCH_PINを使うとモーターからのノイズで勝手にシャットダウンすることがある
   BQ27441が不安定

   TODO(優先順位)
   P_GAINの最適化
   PID制御の実装
   バッテリーステータスの送信
   高度センサ

   完了
   throttle_target_rawが0の時のみ電源スイッチを有効にする
   UDP通信で目標姿勢データと起動情報データの分解split関数を使う

   あきらめたこと
   電源スイッチ割込み
   カメラの実装
   DTP603450を使うこと
   USB使用時のモーター回転禁止
*/


#include <WiFi.h>
#include <WiFiUDP.h>
#include <MadgwickAHRS.h>
#include <SparkFunBQ27441.h>
#include "GPIO_config.h"
//#include "serial_cmd.h"
#include "BMX055.h"
#include "split.h"


//バッテリー容量[mAh]
#define BATTERY_CAPACITY 1000

//目標姿勢角最大値[deg]
#define MAX_POSTURE_ANGLE 15
//目標yaw軸角速度最大値[deg/s]
#define MAX_YAW_VELOCITY 500

//PIDゲイン
#define P_GAIN 2
#define I_GAIN 1
#define D_GAIN 0.7
#define TARGET 0

//yaw軸調整
#define YAW_P_GAIN 1
#define YAW_I_GAIN 1
#define YAW_D_GAIN 1

//スロットル調整
#define T_GAIN 1

//重力加速度[m/s^2]
#define G_ACCL 9.80665

//受信するトークン数
#define STRING_NUM 6


WiFiUDP udp;
Madgwick filter;
BMX055 IMU;


//アクセスポイント設定 パスワードは8文字以上必要
const char *ssid     = "OpenDroneV2";
const char *password = "ODPpassword";

//自身のIPアドレス/ゲートウェイ/サブネットマスク
IPAddress ip(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

//送信先ののIPアドレス
static const char *remote_ip    = "192.168.4.2";
//自身のポート
static const int local_UDP_port = 8888;
//送信機のポート
static const int rmote_UDP_port = 8889;

//ループ時間計測関数用
float preTime = 0;

//微分時間[s]
float dt = 0;

//前回のモーター偏差
float pre_motor1_deviation = 0;
float pre_motor2_deviation = 0;
float pre_motor3_deviation = 0;
float pre_motor4_deviation = 0;

//角度-角速度変換用
float pre_yaw = 0;

//バッテリーが接続されると、Trueになる
bool battery_status = 0;


void setup() {
  // デバック用シリアル通信は115200bps
  Serial.begin(115200);
  Serial.println("Seial OK");

  //GPIO,PWMの初期化
  GPIO_setup();

  //バッテリー残量IC初期化
  //setupBQ27441();
  //Serial.println("battery OK");

  //BMX055 初期化
  IMU.begin();
  Serial.println("IMU OK");

  //MadgwickFilterのサンプリンレート。IMUのサンプリンレートよりも小さくする5Hz(MAX25Hz)
  filter.begin(25);

  //アクセスポイント構築
  WiFi.softAPConfig(ip, gateway, subnet);
  WiFi.softAP(ssid, password);
  Serial.print("SSID = ");
  Serial.println(ssid);
  Serial.print("Password = ");
  Serial.println(password);

  //ドローンのIPアドレスをシリアルで表示
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  Serial.println("Wi-Fi OK");

  //UDPサーバ構築
  udp.begin(local_UDP_port);
  Serial.println("UDP OK");

  Serial.println("Setup complete");
  Serial.print("Setup time = ");
  Serial.print(micros());
  Serial.println(" us");

  delay(300);

  preTime = micros();
}


void loop() {
  //MadgwickFilterの出力値変数 0≦x＜360[deg]
  float roll  = 0;
  float pitch = 0;
  float yaw   = 0;

  //送信機からの目標姿勢角生値変数 -2047≦x≦2047
  float roll_target_raw = 0;
  float pitch_target_raw = 0;
  //送信機からのyaw軸速度目標生値変数 -2047≦x≦2047
  float yaw_velocity_target_raw = 0;
  //送信機からのスロットル目標生値変数 0≦x≦4095
  unsigned int throttle_target_raw = 0;

  //目標姿勢角 30≦x≦30[deg]
  float roll_target = 0;
  float pitch_target = 0;
  //目標yaw軸角速度変数 -500≦x≦500[deg/s]
  float yaw_velocity_target = 0;
  //目標スロットル変数 0≦x≦4095
  unsigned int throttle_target = 0;

  //モーター偏差
  float motor1_deviation = 0;
  float motor2_deviation = 0;
  float motor3_deviation = 0;
  float motor4_deviation = 0;

  //モーター偏差の微分
  float motor1_differential = 0;
  float motor2_differential = 0;
  float motor3_differential = 0;
  float motor4_differential = 0;

  //デューティー比生値変数
  int motor1_duty_raw = 0;
  int motor2_duty_raw = 0;
  int motor3_duty_raw = 0;
  int motor4_duty_raw = 0;
  //yaw軸回転操作量変数
  int yaw_operation = 0;

  //UDP到着データ種類判別・破損識別変数
  String data_type;
  //破損識別用最終データ変数
  String end_status;

  //IMUへ姿勢データ取得リクエスト
  IMU.begin_Accl();
  IMU.begin_Gyro();
  IMU.begin_Mag();

  //IMU生値をセンサーフュージョン
  filter.update(IMU.Gyro(x), IMU.Gyro(y), IMU.Gyro(z), IMU.Accl(x) / G_ACCL, IMU.Accl(y) / G_ACCL, IMU.Accl(z) / G_ACCL, IMU.Mag(x), IMU.Mag(y), IMU.Mag(z));

  //Madgwickフィルタで取得した姿勢角度を代入
  roll  = filter.getRoll();
  pitch = filter.getPitch();
  yaw   = filter.getYaw();

  dt = loopTime();

  //Serial.println(differential(yaw));

  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    //受信データ
    char packetBuffer[255];
    int len = udp.read(packetBuffer, packetSize);
    String cmds[STRING_NUM] = {"\0"};

    //終端文字設定
    if (len > 0) {
      //分割された文字列を格納する配列
      packetBuffer[len] = '\0';
    }

    //Serial.println(packetBuffer);

    // 分割数 = 分割処理(文字列, 区切り文字, 配列)
    int index = split(packetBuffer, ',', cmds);

    data_type = cmds[0];
    end_status = cmds[index - 1];

    //飛行データ取得成功
    if (data_type == "FLIGHT" && end_status == "END") {
      udp.beginPacket(remote_ip, rmote_UDP_port);
      udp.print("FLIGHT_OK");
      udp.endPacket();

      //取得した飛行データを変数に代入
      roll_target_raw         = cmds[1].toInt(); //-2047から2047
      pitch_target_raw        = cmds[2].toInt(); //-2047から2047
      yaw_velocity_target_raw = cmds[3].toInt(); //-2047から2047
      throttle_target_raw     = cmds[4].toInt(); //0から4095

      //目標pitch,roll,yaw角速度の最大値を設定
      roll_target         = roll_target_raw  * MAX_POSTURE_ANGLE / 2047;
      pitch_target        = pitch_target_raw * MAX_POSTURE_ANGLE / 2047;
      yaw_velocity_target = yaw_velocity_target_raw * MAX_YAW_VELOCITY / 2047;

      //スロットル最大値を設定
      throttle_target = throttle_target_raw;

      motor1_deviation = ( roll_target + pitch_target) - ( roll + pitch);
      motor2_deviation = ( roll_target - pitch_target) - ( roll - pitch);
      motor3_deviation = (-roll_target - pitch_target) - (-roll - pitch);
      motor4_deviation = (-roll_target + pitch_target) - (-roll + pitch);

      motor1_differential = (motor1_deviation - pre_motor1_deviation) / dt;
      pre_motor1_deviation = motor1_deviation;
      
      motor2_differential = (motor2_deviation - pre_motor2_deviation) / dt;
      pre_motor2_deviation = motor2_deviation;
      
      motor3_differential = (motor3_deviation - pre_motor3_deviation) / dt;
      pre_motor3_deviation = motor3_deviation;
      
      motor4_differential = (motor4_deviation - pre_motor4_deviation) / dt;
      pre_motor4_deviation = motor1_deviation;

      //Serial.println(motor1_deviation);
      Serial.println(motor1_differential);
      //yaw軸回転操作量
      //複数loopTime()を使うとうまくいかない
      //yaw_operation = (yaw_velocity_target - differential(yaw, pre_yaw));
      //Serial.println(yaw_operation);

      motor1_duty_raw = throttle_target * T_GAIN + motor1_deviation * P_GAIN + motor1_differential * D_GAIN; //+ yaw_operation * YAW_P_GAIN;
      motor2_duty_raw = throttle_target * T_GAIN + motor2_deviation * P_GAIN + motor2_differential * D_GAIN; //- yaw_operation * YAW_P_GAIN;
      motor3_duty_raw = throttle_target * T_GAIN + motor3_deviation * P_GAIN + motor3_differential * D_GAIN; //+ yaw_operation * YAW_P_GAIN;
      motor4_duty_raw = throttle_target * T_GAIN + motor4_deviation * P_GAIN + motor4_differential * D_GAIN; //- yaw_operation * YAW_P_GAIN;

      if (throttle_target_raw == 0) {
        motor1_duty_raw = 0;
        motor2_duty_raw = 0;
        motor3_duty_raw = 0;
        motor4_duty_raw = 0;
      }

    } else if (data_type == "CONFIG" && end_status == "END") {

    } else {
      motor1_duty_raw = 0;
      motor2_duty_raw = 0;
      motor3_duty_raw = 0;
      motor4_duty_raw = 0;
      udp.beginPacket(remote_ip, rmote_UDP_port);
      udp.print("ERROR");
      udp.endPacket();
    }
  }

  //モーターデューティー比を有効な範囲に収める
  motor1_duty_raw = constrain(motor1_duty_raw, 0, 4095);
  motor2_duty_raw = constrain(motor2_duty_raw, 0, 4095);
  motor3_duty_raw = constrain(motor3_duty_raw, 0, 4095);
  motor4_duty_raw = constrain(motor4_duty_raw, 0, 4095);

  //デューティー比は12bit(0≦duty≦4095)
  /*ledcWrite(0, motor1_duty_raw);
  ledcWrite(1, motor2_duty_raw);
  ledcWrite(2, motor3_duty_raw);
  ledcWrite(3, motor4_duty_raw);*/

  //Serial.println(motor1_duty_raw);

  /*Serial.print("motor1=" + (String)motor1_duty_raw);
    Serial.print(" motor2=" + (String)motor2_duty_raw);
    Serial.print(" motor3=" + (String)motor3_duty_raw);
    Serial.println(" motor4=" + (String)motor4_duty_raw);*/

  //udp.beginPacket(remote_ip, rmote_UDP_port);
  //udp.print(printBatteryStats());
  //udp.endPacket();
  //Serial.println("loopTime = " + (String)loopTime());

  if (throttle_target_raw == 0 && digitalRead(PW_SWITCH_PIN) == HIGH) {
    shutdown_pw();
  }
}


void shutdown_pw() {
  while (digitalRead(PW_SWITCH_PIN) == HIGH) {
  }
  digitalWrite(SHUTDOWN_PIN, HIGH);
}


//ループ時間計測関数
inline float loopTime() {
  float loop_time = 0;

  //今の時間から前回の時間を代入したpreTime引いたものを1000000で割って秒に変換
  loop_time = (micros() - preTime) / 1000000;
  //次の計測に使う時間
  preTime = micros();

  return loop_time;
}


//角度を角速度へ変換(preDegがグローバル変数なので同一ループ内で二回以上使えない)
inline float differential(float deg, float preDeg) {
  float dps = 0;
  dps = (deg - preDeg) / dt;
  preDeg = deg;

  /*if (dps >= 1000) {
    dps = dps - (360 / dt);
    }
    if (dps <= -1000) {
    dps = dps + (360 / dt);
    }*/
  return dps;
}


void setupBQ27441(void) {
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


String printBatteryStats() {
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
