#include <WiFi.h>
#include <WiFiUDP.h>
#include <Wire.h>

#define SVX_PIN 32
#define SVY_PIN 33
#define SVS_PIN
#define MVX_PIN 34
#define MVY_PIN 35
#define SVS_PIN

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
  int mvx = analogRead(MVX_PIN);
  int mvy = analogRead(MVY_PIN);
  mvx = mvx - 2047.5;
  mvy = mvy - 2047.5;

  int svy = analogRead(SVY_PIN);

  int motor1 =  mvx - mvy;
  int motor2 = -mvx - mvy;
  int motor3 = -mvx + mvy;
  int motor4 =  mvx + mvy;
  motor1 = motor1 * 15 / 2047;
  motor2 = motor2 * 15 / 2047;
  motor3 = motor3 * 15 / 2047;
  motor4 = motor4 * 15 / 2047;

  udp.beginPacket(kRemoteIpadr, kRmoteUdpPort);
  udp.print((String)motor1 + "," + (String)motor2 + "," + (String)motor3 + "," + (String)motor4 + ",");
  udp.endPacket();

  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    int len = udp.read(packetBuffer, packetSize);
    //終端文字設定
    if (len > 0) packetBuffer[len] = '\0';

    Serial.print(udp.remoteIP());
    Serial.print(" / ");
    Serial.println(packetBuffer);
  }
  delay(10);
}
