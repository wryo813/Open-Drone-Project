#include <WiFi.h>
#include <WiFiUDP.h>
#include <Wire.h>
#include "split.h"

#define JOYSTICK_LX_PIN 32
#define JOYSTICK_LY_PIN 33
#define JOYSTICK_RX_PIN 34
#define JOYSTICK_RY_PIN 35

#define Throttle_SW_PIN 26
#define Move_SW_PIN     25

#define AVG_CNT 1000



const char ssid[] = "ESP32_wifi"; // SSID
const char pass[] = "esp32pass";  // password

static WiFiUDP udp;

char packetBuffer[255];
static const char *kRemoteIpadr = "192.168.4.1";
static const int kRmoteUdpPort = 8888; //送信先のポート
static const int kLocalPort = 8889;  //自身のポート


int rxv_pro = 0;
int ryv_pro = 0;
int lxv_pro = 0;
int lyv_pro = 0;

void setup() {
  Serial.begin(115200);
  joystick_proofread();

  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {

  }

  udp.begin(kLocalPort);
}

void loop() {
  int lxv = analogRead(JOYSTICK_LX_PIN) - lxv_pro; //-2047から2047
  int lyv = analogRead(JOYSTICK_LY_PIN) - lyv_pro; //0から4095
  int rxv = analogRead(JOYSTICK_RX_PIN) - rxv_pro; //-2047から2047
  int ryv = analogRead(JOYSTICK_RY_PIN) - ryv_pro; //-2047から2047

  int motor1_target_raw =  rxv - ryv; //-4094から4094
  int motor2_target_raw = -rxv - ryv; //-4094から4094
  int motor3_target_raw = -rxv + ryv; //-4094から4094
  int motor4_target_raw =  rxv + ryv; //-4094から4094
  int yaw_target_raw = lxv; //-2047から2047
  int throttle_target_raw = lyv; //0から4095

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    delay(50);
    WiFi.begin(ssid, pass);
  }

  if (throttle_target_raw < 0) {
    throttle_target_raw = 0;
  }

  udp.beginPacket(kRemoteIpadr, kRmoteUdpPort);
  udp.print("FLIGHT," + (String)motor1_target_raw + "," + (String)motor2_target_raw + "," + (String)motor3_target_raw + "," + (String)motor4_target_raw + "," + (String)yaw_target_raw + "," + (String)throttle_target_raw + ",END");
  udp.endPacket();

  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    int len = udp.read(packetBuffer, packetSize);
    //  終端文字設定
    if (len > 0) packetBuffer[len] = '\0';

    if (packetBuffer == "ERROR") {
      udp.beginPacket(kRemoteIpadr, kRmoteUdpPort);
      udp.print("FLIGHT," + (String)motor1_target_raw + "," + (String)motor2_target_raw + "," + (String)motor3_target_raw + "," + (String)motor4_target_raw + "," + (String)yaw_target_raw + "," + (String)throttle_target_raw + ",END");
      udp.endPacket();
    }
  }

  Serial.println("FLIGHT," + (String)motor1_target_raw + "," + (String)motor2_target_raw + "," + (String)motor3_target_raw + "," + (String)motor4_target_raw + "," + (String)yaw_target_raw + "," + (String)throttle_target_raw + ",END");

  delay(10);
}


void joystick_proofread() {
  int rxv_tmp = 0;
  int ryv_tmp = 0;
  int lxv_tmp = 0;
  int lyv_tmp = 0;

  delay(10);
  for (int i = 0; i < AVG_CNT; i++) {
    lyv_pro = analogRead(JOYSTICK_LY_PIN);
    lyv_tmp = lyv_tmp + lyv_pro;
  }
  lyv_pro = lyv_tmp / AVG_CNT;

  delay(10);
  for (int i = 0; i < AVG_CNT; i++) {
    lxv_pro = analogRead(JOYSTICK_LX_PIN);
    lxv_tmp = lxv_tmp + lxv_pro;
  }
  lxv_pro = lxv_tmp / AVG_CNT;

  delay(10);
  for (int i = 0; i < AVG_CNT; i++) {
    rxv_pro = analogRead(JOYSTICK_RX_PIN);
    rxv_tmp = rxv_tmp + rxv_pro;
  }
  rxv_pro = rxv_tmp / AVG_CNT;

  delay(10);
  for (int i = 0; i < AVG_CNT; i++) {
    ryv_pro = analogRead(JOYSTICK_RY_PIN);
    ryv_tmp = ryv_tmp + ryv_pro;
  }
  ryv_pro = ryv_tmp / AVG_CNT;
}
