/*
   メモ
   DTP603450を使ってモータを回転させると3Aでリミッターがかかって供給を停止する
   USB_STATUS_PINを使うとモーターからのノイズでモーターが一時的に停止することがある
   PW_SWITCH_PINを使うとモーターからのノイズで勝手にシャットダウンすることがある
   BQ27441が不安定

   TODO(優先順位)
   高度センサ

   完了
   オイラー角で姿勢角送信
   P_GAINの最適化
   PID制御の実装
   throttle_tgt_rawが0の時のみ電源スイッチを有効にする
   UDP通信で目標姿勢データと起動情報データの分解split関数を使う

   あきらめたこと
   バッテリーステータスの送信（IC割れた）
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
#define MAX_POSTURE_ANGLE 50
//目標yaw軸角速度最大値[deg/s]
#define MAX_YAW_VELOCITY 500

//PIDゲイン
#define P_GAIN 5.8 //PD制御時適切
#define I_GAIN 4   //5では振動した（まだ正確ではない）
#define D_GAIN 0.6 //PD制御時適切
#define TARGET 0

//YAW軸PIDパラメータ
#define YAW_P_GAIN 0
#define YAW_I_GAIN 0
#define YAW_D_GAIN 0

//スロットル調整
#define T_GAIN 1

//重力加速度[m/s^2]
#define G_ACCL 9.80665

//受信するトークン数
#define STRING_NUM 6

//アクセスポイント設定 パスワードは8文字以上必要
const char* ssid     = "OpenDroneV2";
const char* password = "ODPpassword";

//自身のIPアドレス/ゲートウェイ/サブネットマスク
IPAddress ip(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

//送信先ののIPアドレス
static const char *remote_ip = "192.168.4.2";
//自身のポート
static const int local_UDP_port = 8888;
//送信機のポート
static const int rmote_UDP_port = 8889;

//角度-角速度変換用
//float pre_yaw = 0;

//バッテリーが接続されると、Trueになる
bool battery_status = 0;


WiFiUDP udp;
Madgwick filter;
BMX055 IMU;


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

  //MadgwickFilterのサンプリンレート。IMUのサンプリンレートよりも小さくする25Hz(MAX25Hz)
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

  Serial.print("P_GAIN = ");
  Serial.println(P_GAIN);
  Serial.print("I_GAIN = ");
  Serial.println(I_GAIN);
  Serial.print("D_GAIN = ");
  Serial.println(D_GAIN);

  Serial.println("Setup complete");
  Serial.print("Setup time = ");
  Serial.print(micros());
  Serial.println(" us");

  delay(300);
}


void loop() {
  //スロットル目標値変数
  unsigned int throttle_tgt = 0;

  //デューティー比生値変数
  int motor1_duty_raw = 0;
  int motor2_duty_raw = 0;
  int motor3_duty_raw = 0;
  int motor4_duty_raw = 0;

  //IMUへ姿勢データ取得リクエスト
  IMU.begin_Accl();
  IMU.begin_Gyro();
  IMU.begin_Mag();

  //IMUから姿勢データを取得
  float gx = IMU.Gyro(x);
  float gy = IMU.Gyro(y);
  float gz = IMU.Gyro(z);
  float ax = IMU.Accl(x) / G_ACCL;
  float ay = IMU.Accl(y) / G_ACCL;
  float az = IMU.Accl(z) / G_ACCL;
  float mx = IMU.Mag(x);
  float my = IMU.Mag(y);
  float mz = IMU.Mag(z);

  //IMU生値をセンサーフュージョン
  filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

  //Madgwickフィルタで取得した姿勢角度を代入 0≦x＜360[deg]
  float roll  = filter.getRoll();
  float pitch = filter.getPitch();
  float yaw   = filter.getYaw();

  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    //受信データ
    char packetBuffer[255] = {0};

    int len = udp.read(packetBuffer, packetSize);
    String cmds[STRING_NUM] = {"\0"};

    //終端文字設定
    if (len > 0) {
      //分割された文字列を格納する配列
      packetBuffer[len] = '\0';
    }

    // 分割数 = 分割処理(文字列, 区切り文字, 配列)
    int index = split(packetBuffer, ',', cmds);

    //UDP到着データ種類判別・破損識別変数
    String data_type = cmds[0];
    //破損識別用最終データ変数
    String end_status = cmds[index - 1];

    //飛行データ取得成功
    if (data_type == "FLIGHT" && end_status == "END") {
      //コントローラーにフライトデータの送信が成功したことを通知
      /* udp.beginPacket(remote_ip, rmote_UDP_port);
        udp.print("FLIGHT_OK");
        udp.endPacket();*/

      //取得した飛行データを変数に代入
      //送信機からの目標姿勢角生値変数
      int roll_tgt_raw     = cmds[1].toInt();//-2047から2047
      int pitch_tgt_raw    = cmds[2].toInt();//-2047から2047
      int yaw_vel_tgt_raw  = cmds[3].toInt();//-2047から2047
      int throttle_tgt_raw = cmds[4].toInt();//0から4095

      //目標pitch,roll,yaw角速度の最大値を設定
      float roll_tgt    = (float)roll_tgt_raw    * MAX_POSTURE_ANGLE / 2047;
      float pitch_tgt   = (float)pitch_tgt_raw   * MAX_POSTURE_ANGLE / 2047;
      float yaw_vel_tgt = (float)yaw_vel_tgt_raw * MAX_YAW_VELOCITY  / 2047;
      //スロットル最大値を設定
      throttle_tgt = throttle_tgt_raw;

      //現在yaw軸dps算出用変数
      static float pre_yaw = 0;  

      //モーター偏差の積分
      static float motor1_intg = 0;
      static float motor2_intg = 0;
      static float motor3_intg = 0;
      static float motor4_intg = 0;
      static float yaw_vel_intg = 0;

      //前回のモーター偏差
      static float pre_motor1_devi = 0;
      static float pre_motor2_devi = 0;
      static float pre_motor3_devi = 0;
      static float pre_motor4_devi = 0;
      static float pre_yaw_val_davi = 0;

      //微分時間[s]
      float dt = loopTime();

      //現在YAW軸速度
      float yaw_vel = (yaw - pre_yaw) / dt;
      pre_yaw = yaw;

      //モーター偏差（目標モーター角度と現在のモーター角度の差）
      float motor1_devi = ( roll_tgt + pitch_tgt) - ( roll + pitch);
      float motor2_devi = ( roll_tgt - pitch_tgt) - ( roll - pitch);
      float motor3_devi = (-roll_tgt - pitch_tgt) - (-roll - pitch);
      float motor4_devi = (-roll_tgt + pitch_tgt) - (-roll + pitch);
      //YAW軸速度偏差（目標YAW軸速度と現在のYAW軸速度の差）
      float yaw_vel_davi = yaw_vel_tgt - yaw_vel;
      
      //モーター偏差の積分（モーター偏差とループ時間をかけた値の累計）
      motor1_intg += motor1_devi * dt;
      motor2_intg += motor2_devi * dt;
      motor3_intg += motor3_devi * dt;
      motor4_intg += motor4_devi * dt;
      //YAW軸速度偏差の積分（YAW軸速度偏差とループ時間をかけた値の累計）
      yaw_vel_intg += yaw_vel_davi * dt;

      //暴走の防止のためスロットルが0の時はモーター偏差の積分をリセット
      if (throttle_tgt == 0) {
        motor1_intg = 0;
        motor2_intg = 0;
        motor3_intg = 0;
        motor4_intg = 0;
        yaw_vel_intg = 0;
       }

      //モーター偏差の微分（1秒間あたりのモーター偏差の変化量）
      float motor1_diff = (motor1_devi - pre_motor1_devi) / dt;
      float motor2_diff = (motor2_devi - pre_motor2_devi) / dt;
      float motor3_diff = (motor3_devi - pre_motor3_devi) / dt;
      float motor4_diff = (motor4_devi - pre_motor4_devi) / dt;
      pre_motor1_devi = motor1_devi;
      pre_motor2_devi = motor2_devi;
      pre_motor3_devi = motor3_devi;
      pre_motor4_devi = motor4_devi;
      //YAW軸速度偏差の微分（1秒間あたりのYAW軸速度偏差の変化量）
      float yaw_vel_diff = (yaw_vel_davi - pre_yaw_val_davi) / dt;
      pre_yaw_val_davi = yaw_vel_davi;

      //YAW軸速度のPID制御
      float yaw_op = yaw_vel_davi * YAW_P_GAIN +  yaw_vel_intg * YAW_I_GAIN + yaw_vel_diff * YAW_D_GAIN;

      //
      motor1_duty_raw = throttle_tgt * T_GAIN + motor1_devi * P_GAIN + motor1_intg * I_GAIN + motor1_diff * D_GAIN + yaw_op;
      motor2_duty_raw = throttle_tgt * T_GAIN + motor2_devi * P_GAIN + motor2_intg * I_GAIN + motor2_diff * D_GAIN - yaw_op;
      motor3_duty_raw = throttle_tgt * T_GAIN + motor3_devi * P_GAIN + motor3_intg * I_GAIN + motor3_diff * D_GAIN + yaw_op;
      motor4_duty_raw = throttle_tgt * T_GAIN + motor4_devi * P_GAIN + motor4_intg * I_GAIN + motor4_diff * D_GAIN - yaw_op;

      if (throttle_tgt == 0) {
        motor1_duty_raw = 0;
        motor2_duty_raw = 0;
        motor3_duty_raw = 0;
        motor4_duty_raw = 0;
      }
      if (throttle_tgt > 0) {
        udp.beginPacket(remote_ip, rmote_UDP_port);
        udp.println((String)millis() + "," + (String)pitch_tgt + "," + (String)pitch);
        udp.endPacket();
      }

    }
    /*else if (data_type == "CONFIG" && end_status == "END") {}
      else {
      motor1_duty_raw = 0;
      motor2_duty_raw = 0;
      motor3_duty_raw = 0;
      motor4_duty_raw = 0;
      udp.beginPacket(remote_ip, rmote_UDP_port);
      udp.print("ERROR");
      udp.endPacket();
      }*/
  }

  //モーターデューティー比を有効な範囲に収める
  motor1_duty_raw = constrain(motor1_duty_raw, 0, 4095);
  motor2_duty_raw = constrain(motor2_duty_raw, 0, 4095);
  motor3_duty_raw = constrain(motor3_duty_raw, 0, 4095);
  motor4_duty_raw = constrain(motor4_duty_raw, 0, 4095);

  //デューティー比は12bit(0≦duty≦4095)
  ledcWrite(0, motor1_duty_raw);
  ledcWrite(1, motor2_duty_raw);
  ledcWrite(2, motor3_duty_raw);
  ledcWrite(3, motor4_duty_raw);

  //送信機から送られてきた目標スロットルが0かつ電源スイッチがスイッチが押されたときにshutdown_pw()を呼び出す
  if (throttle_tgt == 0 && digitalRead(PW_SWITCH_PIN) == HIGH) {
    shutdown_pw();
  }
}


//電源ぼスイッチが押された時にシャットダウンする関数
void shutdown_pw()
{
  //電源スイッチ押し付けるている間はシャットダウンしない
  //これがないと、電源スイッチを押し付けている間起動とシャットダウンを繰り返す。
  while (digitalRead(PW_SWITCH_PIN) == HIGH) {
  }
  //シャットダウンピンをHIGHにする
  digitalWrite(SHUTDOWN_PIN, HIGH);
}


//ループ時間計測関数
inline float loopTime()
{
  float loop_time = 0;
  static float preTime = 0;

  //今の時間から前回の時間を代入したpreTime引いたものを1000000で割って秒に変換
  loop_time = (micros() - preTime) / 1000000;
  //次の計測に使う時間
  preTime = micros();

  return loop_time;
}


//角度を角速度へ変換(preDegがグローバル変数なので同一ループ内で二回以上使えない)
/*inline float differential(float deg, float preDeg) {
  float dps = 0;
  dps = (deg - preDeg) / dt;
  preDeg = deg;

  /*if (dps >= 1000) {
    dps = dps - (360 / dt);
    }
    if (dps <= -1000) {
    dps = dps + (360 / dt);
    }
  return dps;
  }*/


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
