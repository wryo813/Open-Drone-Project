#ifndef GPIO_config_h
#define GPIO_config_h

#include <Arduino.h>

#define JOYSTICK_LX_PIN 32 //左ジョイスティックx軸
#define JOYSTICK_LY_PIN 33 //左ジョイスティックy軸
#define JOYSTICK_RX_PIN 34 //右ジョイスティックx軸
#define JOYSTICK_RY_PIN 35 //右ジョイスティックy軸

#define Throttle_SW_PIN 26
#define Move_SW_PIN     25


void GPIO_setup(){
  //GPIO入出力設定
  //ジョイスティック接続ピン
  pinMode(JOYSTICK_LX_PIN, INPUT);
  pinMode(JOYSTICK_LY_PIN, INPUT);
  pinMode(JOYSTICK_RX_PIN, INPUT);
  pinMode(JOYSTICK_RY_PIN, INPUT);
  
  Serial.println("GPIO OK");
}

#endif
