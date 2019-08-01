#ifndef GPIO_config_h
#define GPIO_config_h

#include <Arduino.h>

#define JOYSTICK_LX_PIN 32 //左ジョイスティックx軸
#define JOYSTICK_LY_PIN 33 //左ジョイスティックy軸
#define JOYSTICK_RX_PIN 34 //右ジョイスティックx軸
#define JOYSTICK_RY_PIN 35 //右ジョイスティックy軸

#define MIX_SW_PIN      26 //Mixスイッチ
#define GYRO_SW_PIN     25 //5CH Gyroスイッチ

#define GEAR_PIT        36 //Gear/Pitスイッチ

//GPIO設定
void GPIO_setup(){
  //GPIO入出力設定
  //ジョイスティック接続ピン
  pinMode(JOYSTICK_LX_PIN, INPUT);
  pinMode(JOYSTICK_LY_PIN, INPUT);
  pinMode(JOYSTICK_RX_PIN, INPUT);
  pinMode(JOYSTICK_RY_PIN, INPUT);

  //スイッチ
  pinMode(MIX_SW_PIN, INPUT);
  pinMode(GYRO_SW_PIN, INPUT);

  //可変抵抗
  pinMode(GEAR_PIT, INPUT);
  
  Serial.println("GPIO OK");
}

#endif
