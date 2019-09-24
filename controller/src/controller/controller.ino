#include <WiFi.h>
#include <WiFiUDP.h>
#include "GPIO_config.h"
#include "split.h"

//ジョイスティックの初期状態の平均値を求める際のジョイスティック電圧読み込み回数
#define AVG_CNT 100

//受信する分割文字列数
#define STRING_NUM 8

//接続先アクセスポイントのSSID/パスワード
const char ssid[] = "OpenDroneV2"; // SSID
const char pass[] = "ODPpassword"; // password

//自身のIPアドレス
IPAddress ip(192, 168, 4, 2);
//自身のゲートウェイ
IPAddress gateway(192, 168, 4, 1);
//自身のサブネットマスク
IPAddress subnet(255, 255, 255, 0);
//自身のDNSサーバ
IPAddress DNS(192, 168, 4, 1);

//送信先のIPアドレス
static const char *remote_ip    = "192.168.4.1";
//自身のUDPポート
static const int local_UDP_port = 8889;
//送信先のUDPポート
static const int rmote_UDP_port = 8888;

//初期状態のジョイスティックの値を格納する変数
int rxv_pro = 0;
int ryv_pro = 0;
int lxv_pro = 0;
int lyv_pro = 0;

WiFiUDP udp;

void setup() {
  //デバック用シリアル通信は115200bps
  Serial.begin(115200);
  
  //GPIOの初期化
  GPIO_setup();
  Serial.println("GPIO OK");
  
  //初期状態のジョイスティックの値を取得
  joystick_proofread();
  Serial.println("Joystick OK");

  //自身のIPアドレス、ゲートウェイ、サブネットマスク、DNSサーバの設定
  WiFi.config(ip, gateway, subnet, DNS);
  //アクセスポイントに設定した機体にWi-Fi接続
  WiFi.begin(ssid, pass);
  Serial.println("weiting");
  //接続されるまで待機
  while (WiFi.status() != WL_CONNECTED);
  Serial.println("conected");
  
  //UDPサーバ構築
  udp.begin(local_UDP_port);
  Serial.println("UDP OK");

  Serial.println("Setting completed");
  delay(300);
}

void loop() {
  //Wi-Fiが切断された時に再接続を開始
  if (WiFi.status() != WL_CONNECTED) {
    //完全にWi-Fiを切断
    WiFi.disconnect();
    delay(50);
    //アクセスポイントに再接続
    WiFi.begin(ssid, pass);
  }
  
  //初期状態のジョイスティックの値を引くことで、ジョイスティックが中心の時に0になる。
  int lxv = analogRead(JOYSTICK_LX_PIN) - lxv_pro;//-2047≦x≦2047
  int lyv = analogRead(JOYSTICK_LY_PIN) - lyv_pro;//0≦x≦4095
  int rxv = analogRead(JOYSTICK_RX_PIN) - rxv_pro;//-2047≦x≦2047
  int ryv = analogRead(JOYSTICK_RY_PIN) - ryv_pro;//-2047≦x≦2047

  //送信する姿勢角度
  //手動でジョイスティックの値を調整
  int roll_tgt_raw     = -ryv - 30;
  int pitch_tgt_raw    =  rxv - 30;
  int yaw_vel_tgt_raw  =  lxv - 30;
  int throttle_tgt_raw =  lyv - 100;
  //throttle_tgt_rawが0よりも小さいときは0を代入
  throttle_tgt_raw = max(throttle_tgt_raw, 0);

  //Mixスイッチがオフの時、姿勢角度とスロットルを0にする(緊急停止)。
  if (digitalRead(MIX_SW_PIN) == LOW) {
    roll_tgt_raw = 0;
    pitch_tgt_raw = 0;
    yaw_vel_tgt_raw = 0;
    throttle_tgt_raw = 0;
  }

  //UDP通信で送信するデータを生成
  String send_data  = "FLIGHT,";
  send_data += (String)roll_tgt_raw + ",";
  send_data += (String)pitch_tgt_raw + ",";
  send_data += (String)yaw_vel_tgt_raw + ",";
  send_data += (String)throttle_tgt_raw + ",";
  send_data += "END";

  //機体にデータを送信
  udp.beginPacket(remote_ip, rmote_UDP_port);
  udp.print(send_data);
  udp.endPacket();

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
    //機体データ取得成功
    if (data_type == "STATUS" && end_status == "END") {
      unsigned long time   = cmds[1].toInt();   //機体が起動してからの経過時間
      float recv_pitch_tgt = cmds[2].toFloat(); //目標pitch角度
      float pitch_tgt      = cmds[3].toFloat(); //実測pitch角度
      int battery_voltage  = cmds[4].toInt();   //バッテリー電圧
      int battery_current  = cmds[5].toInt();   //バッテリー電流
      int battery_soc      = cmds[6].toInt();   //バッテリー残量
      Serial.print(time);
      Serial.print(",");
      Serial.print(recv_pitch_tgt);
      Serial.print(",");
      Serial.print(pitch_tgt);
      Serial.print(",");
      Serial.print(battery_voltage );
      Serial.print(",");
      Serial.print(battery_current);
      Serial.print(",");
      Serial.println(battery_soc);
    }
  }
}

//ジョイスティックの初期状態の平均値を求める関数
void joystick_proofread() {
  int rxv_tmp = 0;
  int ryv_tmp = 0;
  int lxv_tmp = 0;
  int lyv_tmp = 0;

  for (int i = 0; i < AVG_CNT; i++) {
    lyv_tmp += analogRead(JOYSTICK_LY_PIN);
  }
  lyv_pro = lyv_tmp / AVG_CNT;

  for (int i = 0; i < AVG_CNT; i++) {
    lxv_tmp += analogRead(JOYSTICK_LX_PIN);
  }
  lxv_pro = lxv_tmp / AVG_CNT;

  for (int i = 0; i < AVG_CNT; i++) {
    rxv_tmp += analogRead(JOYSTICK_RX_PIN);
  }
  rxv_pro = rxv_tmp / AVG_CNT;

  for (int i = 0; i < AVG_CNT; i++) {
    ryv_tmp += analogRead(JOYSTICK_RY_PIN);
  }
  ryv_pro = ryv_tmp / AVG_CNT;
}
