//PIDパラメータ設定の為姿勢は仮で0を送信

#include <WiFi.h>
#include <WiFiUDP.h>
#include <Wire.h>
#include "split.h"
#include "GPIO_config.h"


#define AVG_CNT 100


//接続先アクセスポイントのSSID/パスポート
const char ssid[] = "OpenDroneV2"; // SSID
const char pass[] = "ODPpassword"; // password

//自身のIPアドレス、ゲートウェイ、サブネットマスク、DNSサーバ
IPAddress ip(192, 168, 4, 2);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress DNS(192, 168, 4, 1);

//送信先のIPアドレス
static const char *remote_ip    = "192.168.4.1";
//自身のポート
static const int local_UDP_port = 8889;
//送信先のポート
static const int rmote_UDP_port = 8888;

//ジョイスティックのキャリブレーション用変数
int rxv_pro = 0;
int ryv_pro = 0;
int lxv_pro = 0;
int lyv_pro = 0;


WiFiUDP udp;


void setup() {
  Serial.begin(115200);
  //GPIOの設定
  GPIO_setup();
  delay(500);
  //初期状態のジョイスティックの値を取得
  joystick_proofread();
  Serial.println("joystick_proofread");
  
  //自身のIPアドレス、ゲートウェイ、サブネットマスク、DNSサーバの設定
  WiFi.config(ip, gateway, subnet, DNS);
  //接続先のSSIDとパスワード設定
  WiFi.begin(ssid, pass);

  Serial.println("weiting");
  //Wi-Fiアクセスポイントに設定した機体に接続するまで待機
  while (WiFi.status() != WL_CONNECTED) {
  }

  Serial.println("conected");
  //UDPサーバ構築
  udp.begin(local_UDP_port);
}


void loop() {
  //初期状態のジョイスティックの値を引くことで、ジョイスティックが中心の時に0になる。
  int lxv = analogRead(JOYSTICK_LX_PIN) - lxv_pro - 30; //-2047から2047
  int lyv = analogRead(JOYSTICK_LY_PIN) - lyv_pro - 100; //0から4095
  int rxv = analogRead(JOYSTICK_RX_PIN) - rxv_pro - 30;//-2047から2047
  int ryv = analogRead(JOYSTICK_RY_PIN) - ryv_pro - 30; //-2047から2047

  //送信する姿勢角度
  int roll_tgt_raw     = -ryv; //-2047から2047
  int pitch_tgt_raw    = rxv; //-2047から2047
  int yaw_vel_tgt_raw  = lxv; //-2047から2047
  int throttle_tgt_raw = lyv; //0から4095

  //コントローラーのMixスイッチの状態を格納する変数
  bool stop_status = digitalRead(MIX_SW_PIN);

  //UDP通信で送信するデータを格納する変数
  String send_data = "";

  //Wi-Fiが切断されたときに再接続を開始
  if (WiFi.status() != WL_CONNECTED) {
    //完全にWi-Fiを切断
    WiFi.disconnect();
    delay(50);
    //アクセスポイントに再接続
    WiFi.begin(ssid, pass);
  }

  //throttle_tgt_rawが0よりも小さいときは0を代入
  throttle_tgt_raw = max(throttle_tgt_raw, 0);

  //Mixスイッチがオンの時、姿勢角度とスロットルを0にする(緊急停止)
  if (stop_status == 0) {
    roll_tgt_raw = 0;
    pitch_tgt_raw = 0;
    yaw_vel_tgt_raw = 0;
    throttle_tgt_raw = 0;
  }
  roll_tgt_raw = 0;
  pitch_tgt_raw = 0;
  yaw_vel_tgt_raw = 0;

  //UDP通信で送信するデータ
  send_data  = "FLIGHT,";
  send_data += (String)roll_tgt_raw + ",";
  send_data += (String)pitch_tgt_raw + ",";
  send_data += (String)yaw_vel_tgt_raw + ",";
  send_data += (String)throttle_tgt_raw + ",";
  send_data += "END";

  udp.beginPacket(remote_ip, rmote_UDP_port);
  //データタイプ,目標ロール,目標ピッチ,目標yaw速度,目標スロットル,最終データ
  udp.print(send_data);
  udp.endPacket();

  /* int packetSize = udp.parsePacket();
    if (packetSize > 0) {
     int len = udp.read(packetBuffer, packetSize);
     //  終端文字設定
     if (len > 0) packetBuffer[len] = '\0';

     //エラーが帰ってき場合、send_dataを再送する
     if (packetBuffer == "ERROR") {
       udp.beginPacket(remote_ip, rmote_UDP_port);
       udp.print(send_data);
       udp.endPacket();
     }
    }*/

  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    char packetBuffer[1000] = {0};

    int len = udp.read(packetBuffer, packetSize);
    //  終端文字設定
    if (len > 0) packetBuffer[len] = '\0';
    Serial.print(packetBuffer);
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
