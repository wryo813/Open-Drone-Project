#include <WiFi.h>
#include <WiFiUDP.h>
#include <Wire.h>

#define Throttle_X_PIN  32
#define Throttle_Y_PIN  33
#define Throttle_SW_PIN 26
#define Move_X_PIN      34
#define Move_Y_PIN      35
#define Move_SW_PIN     25


const char ssid[] = "ESP32_wifi"; // SSID
const char pass[] = "esp32pass";  // password

static WiFiUDP udp;

char packetBuffer[255];
static const char *kRemoteIpadr = "192.168.4.1";
static const int kRmoteUdpPort = 8888; //送信先のポート
static const int kLocalPort = 8889;  //自身のポート

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, pass);
    while ( WiFi.status() != WL_CONNECTED) {
    delay(500);
    }
    udp.begin(kLocalPort);
}

void loop() {
  int mvx = analogRead(Move_X_PIN);
  int mvy = analogRead(Move_Y_PIN);
  int tvx = analogRead(Throttle_X_PIN);
  int tvy = analogRead(Throttle_Y_PIN);

  mvx = mvx - 2047.5 + 121;
  mvy = mvy - 2047.5 + 111;

  tvx = tvx - 2047.5;

  mvx = min(mvx, 2047);
  mvy = min(mvy, 2047);

  float motor1 =  mvx - mvy;
  float motor2 = -mvx - mvy;
  float motor3 = -mvx + mvy;
  float motor4 =  mvx + mvy;

  int m1 = motor1 * 15 / 2047;
  int m2 = motor2 * 15 / 2047;
  int m3 = motor3 * 15 / 2047;
  int m4 = motor4 * 15 / 2047;

  int angular_velocity = tvx * 400/2047;

  udp.beginPacket(kRemoteIpadr, kRmoteUdpPort);
  udp.print((String)m1 + "," + (String)m2 + "," + (String)m3 + "," + (String)m4 + "," + (String)angular_velocity);
  udp.endPacket();

  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
  int len = udp.read(packetBuffer, packetSize);
  //  終端文字設定
  if (len > 0) packetBuffer[len] = '\0';
  Serial.print(udp.remoteIP());
  Serial.print(" / ");
  Serial.println(packetBuffer);
  }
  delay(10);
}
