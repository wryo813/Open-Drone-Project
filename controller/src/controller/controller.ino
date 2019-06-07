#include <WiFi.h>
#include <WiFiUDP.h>
#include <Wire.h>

#define Throttle_X_PIN  32
#define Throttle_Y_PIN  33
#define Move_X_PIN      34
#define Move_Y_PIN      35

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


int mvx_pro = 0;
int mvy_pro = 0;
int tvx_pro = 0;
int tvy_pro = 0;

void setup() {
  Serial.begin(115200);
  joystick_proofread();

  WiFi.begin(ssid, pass);
    while ( WiFi.status() != WL_CONNECTED) {
    delay(500);
    }
    udp.begin(kLocalPort);
}

void loop() {
  int mvx = analogRead(Move_X_PIN) - mvx_pro;
  int mvy = analogRead(Move_Y_PIN) - mvy_pro;
  int tvx = analogRead(Throttle_X_PIN) - tvx_pro;
  int tvy = analogRead(Throttle_Y_PIN) - tvy_pro;

  int motor1_raw =  mvx - mvy;
  int motor2_raw = -mvx - mvy;
  int motor3_raw = -mvx + mvy;
  int motor4_raw =  mvx + mvy;

  int angular_velocity_raw = tvx;

  int throttle_raw = tvy;

  if(throttle_raw<0){
    throttle_raw = 0;
  }


  Serial.println((String)motor1_raw + "," + (String)motor2_raw + "," + (String)motor3_raw + "," + (String)motor4_raw + "," + (String)angular_velocity_raw + "," + (String)throttle_raw);

    udp.beginPacket(kRemoteIpadr, kRmoteUdpPort);
  udp.print((String)motor1_raw + "," + (String)motor2_raw + "," + (String)motor3_raw + "," + (String)motor4_raw + "," + (String)angular_velocity_raw + "," + (String)throttle_raw);
  udp.endPacket();

  /*int packetSize = udp.parsePacket();
    if (packetSize > 0) {
    int len = udp.read(packetBuffer, packetSize);
    //  終端文字設定
    if (len > 0) packetBuffer[len] = '\0';
    Serial.print(udp.remoteIP());
    Serial.print(" / ");
    Serial.println(packetBuffer);
    }*/
  delay(10);
}


void joystick_proofread()
{
  int mvx_tmp = 0;
  int mvy_tmp = 0;
  int tvx_tmp = 0;
  int tvy_tmp = 0;

  delay(10);
  for (int i = 0; i < AVG_CNT; i++) {
    tvy_pro = analogRead(Throttle_Y_PIN);
    tvy_tmp = tvy_tmp + tvy_pro;
  }
  tvy_pro = tvy_tmp / AVG_CNT;

  delay(10);
  for (int i = 0; i < AVG_CNT; i++) {
    tvx_pro = analogRead(Throttle_X_PIN);
    tvx_tmp = tvx_tmp + tvx_pro;
  }
  tvx_pro = tvx_tmp / AVG_CNT;

  delay(10);
  for (int i = 0; i < AVG_CNT; i++) {
    mvx_pro = analogRead(Move_X_PIN);
    mvx_tmp = mvx_tmp + mvx_pro;
  }
  mvx_pro = mvx_tmp / AVG_CNT;

  delay(10);
  for (int i = 0; i < AVG_CNT; i++) {
    mvy_pro = analogRead(Move_Y_PIN);
    mvy_tmp = mvy_tmp + mvy_pro;
  }
  mvy_pro = mvy_tmp / AVG_CNT;
}
