//PIDパラメータ設定の為姿勢は仮で0を送信

#include <WiFi.h>
#include <WiFiUDP.h>
#include <Wire.h>
#include "split.h"
#include "GPIO_config.h"

#define JOYSTICK_LX_PIN 32
#define JOYSTICK_LY_PIN 33
#define JOYSTICK_RX_PIN 34
#define JOYSTICK_RY_PIN 35

#define Throttle_SW_PIN 26
#define Move_SW_PIN     25

#define AVG_CNT 100


WiFiUDP udp;

//接続先アクセスポイントのSSID/パスポート
const char ssid[] = "OpenDroneV2"; // SSID
const char pass[] = "ODPpassword";     // password

IPAddress ip(192, 168, 4, 2);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress DNS(192, 168, 4, 1);

//送信先のIPアドレス
static const char *remote_ip    = "192.168.4.1";
//自身のポート
static const int local_UDP_port = 8889; //自身のポート
//送信先のポート
static const int rmote_UDP_port = 8888;

int rxv_pro = 0;
int ryv_pro = 0;
int lxv_pro = 0;
int lyv_pro = 0;

void setup() {
  Serial.begin(115200);
  GPIO_setup();
  delay(500);
  joystick_proofread();
  Serial.println("joystick_proofread");

  WiFi.config(ip, gateway, subnet, DNS);
  WiFi.begin(ssid, pass);

  Serial.println("weiting");
  while (WiFi.status() != WL_CONNECTED) {
  }

  Serial.println("conected");
  udp.begin(local_UDP_port);
}

void loop() {
  int lxv = analogRead(JOYSTICK_LX_PIN) - lxv_pro - 30; //-2047から2047
  int lyv = analogRead(JOYSTICK_LY_PIN) - lyv_pro - 100; //0から4095
  int rxv = analogRead(JOYSTICK_RX_PIN) - rxv_pro - 30;//-2047から2047
  int ryv = analogRead(JOYSTICK_RY_PIN) - ryv_pro - 30; //-2047から2047

  int roll_target_raw              = -ryv; //-2047から2047
  int pitch_target_raw             =  rxv; //-2047から2047
  int yaw_velocity_target_raw      =  lxv; //-2047から2047
  int throttle_target_raw          =  lyv; //0から4095

  bool stop_status = digitalRead(MIX_SW_PIN);

  String send_data = "";

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    delay(50);
    WiFi.begin(ssid, pass);
  }

  //throttle_target_rawが0よりも小さいときは0を代入
  throttle_target_raw = max(throttle_target_raw, 0);

  if (stop_status == 0) {
    roll_target_raw = 0;
    pitch_target_raw = 0;
    yaw_velocity_target_raw = 0;
    throttle_target_raw = 0;
  }
  roll_target_raw = 0;
  pitch_target_raw = 0;
  yaw_velocity_target_raw = 0;

  //送信データ
  send_data  = "FLIGHT,";
  send_data += (String)roll_target_raw + ",";
  send_data += (String)pitch_target_raw + ",";
  send_data += (String)yaw_velocity_target_raw + ",";
  send_data += (String)throttle_target_raw + ",";
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
