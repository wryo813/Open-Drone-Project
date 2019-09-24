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

   PIDパラメータ最適値
   P制御
   P_GAIN=3.80

   PD制御
   P_GAIN=5.80
   D_GAIN=0.60

   PID制御
   P_GAIN 5.80
   I_GAIN 3.70
   D_GAIN 0.87
*/

#include <WiFi.h>
#include <WiFiUDP.h>
#include <MadgwickAHRS.h>
#include <SparkFunBQ27441.h>
#include "GPIO_config.h"
#include "BMX055.h"
#include "split.h"

//バッテリー容量[mAh]
#define BATTERY_CAPACITY 1000

//目標姿勢角度最大値[deg]
#define MAX_POSTURE_ANGLE 40
//目標yaw軸角速度最大値[deg/s]
#define MAX_YAW_VELOCITY 500
//スロットル最大値
#define MAX_THROTTLE 4095

//PIDパラメータ
#define P_GAIN 4.80
#define I_GAIN 2.70
#define D_GAIN 0.87

//yaw軸PIDパラメータ
#define YAW_P_GAIN 0
#define YAW_I_GAIN 0
#define YAW_D_GAIN 0

//重力加速度[m/s^2]
#define G_ACCL 9.80665

//受信する分割文字列数
#define STRING_NUM 6

//アクセスポイント設定 パスワードは8文字以上必要
const char* ssid     = "OpenDroneV2";
const char* password = "ODPpassword";

//自身のIPアドレス
IPAddress ip(192, 168, 4, 1);
//自身のゲートウェイ
IPAddress gateway(192, 168, 4, 1);
//自身のサブネットマスク
IPAddress subnet(255, 255, 255, 0);

//送信先のIPアドレス
static const char *remote_ip = "192.168.4.2";
//自身のUDPポート
static const int local_UDP_port = 8888;
//送信先のUDPポート
static const int rmote_UDP_port = 8889;

WiFiUDP udp;
Madgwick filter;
BMX055 IMU;

void setup() {
  //デバック用シリアル通信は115200bps
  Serial.begin(115200);
  Serial.println("Seial OK");

  //GPIO,PWMの初期化
  GPIO_setup();

  //バッテリー残量IC初期化
  lipo.begin();
  lipo.setCapacity(BATTERY_CAPACITY);
  Serial.println("battery OK");

  //モーションセンサ(BMX055)の初期化
  IMU.begin();
  Serial.println("IMU OK");

  //MadgwickFilterのサンプリンレート[Hz]
  //モーションセンサの出力データレート以下にする25Hz(MAX25Hz)
  filter.begin(25);

  //アクセスポイント構築
  WiFi.softAPConfig(ip, gateway, subnet);
  WiFi.softAP(ssid, password);
  Serial.print("SSID = ");
  Serial.println(ssid);
  Serial.print("Password = ");
  Serial.println(password);
  //機体のIPアドレスを表示
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

  Serial.println("Setting completed");
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

  //モーションセンサへ姿勢データ取得リクエスト
  IMU.begin_Accl();
  IMU.begin_Gyro();
  IMU.begin_Mag();
  //モーションセンサから姿勢データを取得
  float gx = IMU.Gyro(x);
  float gy = IMU.Gyro(y);
  float gz = IMU.Gyro(z);
  //加速度センサの生値を重力加速度で割って[G]に変換
  float ax = IMU.Accl(x) / G_ACCL;
  float ay = IMU.Accl(y) / G_ACCL;
  float az = IMU.Accl(z) / G_ACCL;
  float mx = IMU.Mag(x);
  float my = IMU.Mag(y);
  float mz = IMU.Mag(z);

  //モーションセンサ生値をセンサーフュージョン
  filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
  //Madgwickフィルタで取得した姿勢角度を代入 0≦x＜360[deg]
  float roll  = filter.getRoll();
  float pitch = filter.getPitch();
  float yaw   = filter.getYaw();

  //UDP通信でパケットの存在を確認しサイズをpacketSizeに代入
  int packetSize = udp.parsePacket();
  //もしパケットを受信してたら
  if (packetSize > 0) {
    //UDP通信で受信した文字配列（文字列）を格納する配列
    char packetBuffer[255] = {0};
    //UDP通信での受信したサイズがpacketSizeの文字列をpacketBufferに代入し、受信した文字列の文字数をlenに代入。
    int len = udp.read(packetBuffer, packetSize);
    //分割した文字列を格納する配列をヌル文字で初期化
    String cmds[STRING_NUM] = {"\0"};
    //受信した文字列の最後にヌル文字を追加
    if (len > 0) packetBuffer[len] = '\0';

    //分割数 = 分割処理(文字列, 区切り文字, 配列)
    int index = split(packetBuffer, ',', cmds);
    //UDP到着データ種類判別・破損識別変数
    String data_type = cmds[0];
    //破損識別用最終データ変数
    String end_status = cmds[index - 1];
    //飛行データ取得成功
    if (data_type == "FLIGHT" && end_status == "END") {
      //取得した飛行データを変数に代入
      int roll_tgt_raw     = cmds[1].toInt();//-2047≦x≦2047
      int pitch_tgt_raw    = cmds[2].toInt();//-2047≦x≦2047
      int yaw_vel_tgt_raw  = cmds[3].toInt();//-2047≦x≦2047
      int throttle_tgt_raw = cmds[4].toInt();//0≦x≦4095

      //目標pitch軸角度,roll軸角度,yaw軸角速度の最大値を設定
      float roll_tgt    = (float)roll_tgt_raw    * MAX_POSTURE_ANGLE / 2047;
      float pitch_tgt   = (float)pitch_tgt_raw   * MAX_POSTURE_ANGLE / 2047;
      float yaw_vel_tgt = (float)yaw_vel_tgt_raw * MAX_YAW_VELOCITY  / 2047;
      //スロットル最大値を設定
      throttle_tgt      = throttle_tgt_raw       * MAX_THROTTLE      / 4095;

      //今回yaw軸角速度の算出に使う前回のyaw軸角度を格納する変数
      static float pre_yaw = 0;

      //モーター角度偏差の積分を格納する変数
      static float motor1_intg = 0;
      static float motor2_intg = 0;
      static float motor3_intg = 0;
      static float motor4_intg = 0;
      //yaw軸角速度偏差の積分を格納する変数
      static float yaw_vel_intg = 0;

      //前回のモーター角度偏差を格納する変数
      static float pre_motor1_devi = 0;
      static float pre_motor2_devi = 0;
      static float pre_motor3_devi = 0;
      static float pre_motor4_devi = 0;
      //前回のyaw軸角速度偏差を格納する変数
      static float pre_yaw_val_davi = 0;

      //ループ時間[s]
      float dt = loopTime();

      //yaw軸角速度[deg/s]
      float yaw_vel = (yaw - pre_yaw) / dt;
      //次回のyaw軸角速度算出に使う現在のyaw軸角度
      pre_yaw = yaw;
      //一回転したときに異常な値が出ないようにするための補正
      if (yaw_vel > 600) yaw_vel = yaw_vel - (360 / dt);
      else if (yaw_vel < -600) yaw_vel = yaw_vel + (360 / dt);

      //モーター角度偏差（目標モーター角度と現在のモーター角度の差）
      float motor1_devi = ( roll_tgt + pitch_tgt) - ( roll + pitch);
      float motor2_devi = ( roll_tgt - pitch_tgt) - ( roll - pitch);
      float motor3_devi = (-roll_tgt - pitch_tgt) - (-roll - pitch);
      float motor4_devi = (-roll_tgt + pitch_tgt) - (-roll + pitch);
      //yaw軸角速度偏差（目標yaw軸角速度と現在のyaw軸角速度の差）
      float yaw_vel_davi = yaw_vel_tgt - yaw_vel;

      //モーター角度偏差を時間で積分
      //モーター角度偏差とループ時間[s]をかけた値の累計を算出することで疑似的に積分している
      motor1_intg += motor1_devi * dt;
      motor2_intg += motor2_devi * dt;
      motor3_intg += motor3_devi * dt;
      motor4_intg += motor4_devi * dt;
      //yaw軸角速度偏差を時間で積分
      //yaw軸角速度偏差とループ時間[s]をかけた値の累計を算出することで疑似的に積分している
      yaw_vel_intg += yaw_vel_davi * dt;

      //モーターの暴走防止のため、スロットルが0の時はモーター角度偏差の積分とyaw軸角速度偏差の積分をリセット。
      if (throttle_tgt == 0) {
        motor1_intg = 0;
        motor2_intg = 0;
        motor3_intg = 0;
        motor4_intg = 0;
        yaw_vel_intg = 0;
      }

      //モーター角度偏差を時間で微分
      //モーター角度偏差の変化量をループ時間[s]で割ることで疑似的に微分している
      float motor1_diff = (motor1_devi - pre_motor1_devi) / dt;
      float motor2_diff = (motor2_devi - pre_motor2_devi) / dt;
      float motor3_diff = (motor3_devi - pre_motor3_devi) / dt;
      float motor4_diff = (motor4_devi - pre_motor4_devi) / dt;
      //次回のモーター角度偏差の微分に使う現在のモーター偏差
      pre_motor1_devi = motor1_devi;
      pre_motor2_devi = motor2_devi;
      pre_motor3_devi = motor3_devi;
      pre_motor4_devi = motor4_devi;
      //yaw軸角速度偏差を時間で微分
      //yaw軸角速度偏差の変化量をループ時間[s]で割ることで疑似的に微分している
      float yaw_vel_diff = (yaw_vel_davi - pre_yaw_val_davi) / dt;
      //次回のyaw軸角速度偏差の微分に使う現在のyaw軸角速度偏差
      pre_yaw_val_davi = yaw_vel_davi;

      //yaw軸角速度のPID制御
      float yaw_op = yaw_vel_davi * YAW_P_GAIN +  yaw_vel_intg * YAW_I_GAIN + yaw_vel_diff * YAW_D_GAIN;
      //モーター角度偏差のPID制御
      //yaw軸の制御のため、隣り合うモーターに異符号のyaw軸角速度偏差のPID制御を足す。
      motor1_duty_raw = throttle_tgt + motor1_devi * P_GAIN + motor1_intg * I_GAIN + motor1_diff * D_GAIN + yaw_op;
      motor2_duty_raw = throttle_tgt + motor2_devi * P_GAIN + motor2_intg * I_GAIN + motor2_diff * D_GAIN - yaw_op;
      motor3_duty_raw = throttle_tgt + motor3_devi * P_GAIN + motor3_intg * I_GAIN + motor3_diff * D_GAIN + yaw_op;
      motor4_duty_raw = throttle_tgt + motor4_devi * P_GAIN + motor4_intg * I_GAIN + motor4_diff * D_GAIN - yaw_op;

      //スロットルが0の時はモーターを回転させない
      if (throttle_tgt == 0) {
        motor1_duty_raw = 0;
        motor2_duty_raw = 0;
        motor3_duty_raw = 0;
        motor4_duty_raw = 0;
      }

      //UDP通信で送信するデータを生成
      String send_data  = "STATUS,";
      send_data += (String)millis() + ",";         //起動してからの経過時間
      send_data += (String)pitch_tgt + ",";        //目標pitch角度
      send_data += (String)pitch + ",";            //実測pitch角度
      send_data += (String)lipo.voltage() + ",";   //バッテリー電圧
      send_data += (String)lipo.current(AVG) + ",";//バッテリー電流
      send_data += (String)lipo.soc() + ",";       //バッテリー残量
      send_data += "END";

      //コントローラーにデータを送信
      udp.beginPacket(remote_ip, rmote_UDP_port);
      udp.print(send_data);
      udp.endPacket();
    }
  }

  //モーターのデューティー比を有効な範囲に収める
  motor1_duty_raw = constrain(motor1_duty_raw, 0, 4095);
  motor2_duty_raw = constrain(motor2_duty_raw, 0, 4095);
  motor3_duty_raw = constrain(motor3_duty_raw, 0, 4095);
  motor4_duty_raw = constrain(motor4_duty_raw, 0, 4095);

  //PWMを生成して出力
  //デューティー比は12bit 0≦x≦4095
  ledcWrite(0, motor1_duty_raw);
  ledcWrite(1, motor2_duty_raw);
  ledcWrite(2, motor3_duty_raw);
  ledcWrite(3, motor4_duty_raw);

  //目標スロットルが0かつ電源スイッチが押された時にshutdown_pw()を呼び出す
  if (throttle_tgt == 0 && digitalRead(PW_SWITCH_PIN) == HIGH) {
    shutdown_pw();
  }
}

//電源スイッチが押された時にシャットダウンする関数
void shutdown_pw() {
  //電源スイッチを押し続けている間はシャットダウンしない
  //これがないと、電源スイッチを押し続けている間起動とシャットダウンを繰り返す。
  while (digitalRead(PW_SWITCH_PIN) == HIGH);
  //シャットダウンピンをHIGHにする
  digitalWrite(SHUTDOWN_PIN, HIGH);
}

//ループ時間計測関数
//静的変数を使ってループ時間を計測しているので、二回以上呼び出すと正常に動作しない。
inline float loopTime() {
  float loop_time = 0;
  static float preTime = 0;

  //現在の時間から前回の時間を代入したpreTime引いたものを1000000で割って秒に変換
  loop_time = (micros() - preTime) / 1000000;
  //次の計測に使う現在の時間
  preTime = micros();

  return loop_time;
}
