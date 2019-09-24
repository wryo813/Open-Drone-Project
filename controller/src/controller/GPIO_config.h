#ifndef GPIO_config_h
#define GPIO_config_h

#include <Arduino.h>

//ジョイスティックピン設定
#define JOYSTICK_LX_PIN 32 //左ジョイスティックx軸
#define JOYSTICK_LY_PIN 33 //左ジョイスティックy軸
#define JOYSTICK_RX_PIN 34 //右ジョイスティックx軸
#define JOYSTICK_RY_PIN 35 //右ジョイスティックy軸

//スイッチピン設定
#define MIX_SW_PIN      26 //Mixスイッチ
#define GYRO_SW_PIN     25 //5CH Gyroスイッチ

//可変抵抗ピン設定
#define GEAR_PIT_PIN    36 //Gear/Pit可変抵抗

//GPIO設定
void GPIO_setup(){
  //GPIO入出力設定
  //ジョイスティックピン
  pinMode(JOYSTICK_LX_PIN, INPUT);
  pinMode(JOYSTICK_LY_PIN, INPUT);
  pinMode(JOYSTICK_RX_PIN, INPUT);
  pinMode(JOYSTICK_RY_PIN, INPUT);

  //スイッチピン
  pinMode(MIX_SW_PIN,  INPUT);
  pinMode(GYRO_SW_PIN, INPUT);

  //可変抵抗ピン
  pinMode(GEAR_PIT_PIN, INPUT);
  
  Serial.println("GPIO OK");
}

#endif
