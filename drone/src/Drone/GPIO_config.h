#ifndef GPIO_config_h
#define GPIO_config_h

#include <Arduino.h>

//電源管理ピン設定
#define SHUTDOWN_PIN 25
#define PW_SWITCH_PIN 26
#define USB_STATUS_PIN 34
#define CHG_STATUS_PIN 35

//モーターピン設定
#define MOTOR1_PIN 27
#define MOTOR2_PIN 16
#define MOTOR3_PIN 4
#define MOTOR4_PIN 14

//PWM設定
/*
   PWM設定について
   f=1/T,T=1/fより
   1/80[MHz]=12.5[ns]
   PWM_FREQ[Hz]=1/(12.5[ns]×2^PWM_BIT[bit])
*/
//PWM周波数[Hz]
#define PWM_FREQ 19531
//PWM分解能[bit]
#define PWM_BIT 12


void GPIO_setup(){
  //GPIO入出力設定
  //電源関連
  pinMode(PW_SWITCH_PIN, INPUT);
  pinMode(USB_STATUS_PIN, INPUT);
  pinMode(CHG_STATUS_PIN, INPUT);
  pinMode(SHUTDOWN_PIN, OUTPUT);
  //モーターピン
  pinMode(MOTOR1_PIN, OUTPUT);
  pinMode(MOTOR2_PIN, OUTPUT);
  pinMode(MOTOR3_PIN, OUTPUT);
  pinMode(MOTOR4_PIN, OUTPUT);
  //未使用ピン
  pinMode(13, INPUT_PULLUP);
  pinMode(17, INPUT_PULLUP);
  pinMode(32, INPUT_PULLUP);
  pinMode(33, INPUT_PULLUP);

  //モーターピンの初期化
  digitalWrite(MOTOR1_PIN, LOW);
  digitalWrite(MOTOR2_PIN, LOW);
  digitalWrite(MOTOR3_PIN, LOW);
  digitalWrite(MOTOR4_PIN, LOW);

  //PWM設定
  ledcSetup(0, PWM_FREQ, PWM_BIT);
  ledcSetup(1, PWM_FREQ, PWM_BIT);
  ledcSetup(2, PWM_FREQ, PWM_BIT);
  ledcSetup(3, PWM_FREQ, PWM_BIT);

  //ピンをチャンネルに接続
  ledcAttachPin(MOTOR1_PIN, 0);
  ledcAttachPin(MOTOR2_PIN, 1);
  ledcAttachPin(MOTOR3_PIN, 2);
  ledcAttachPin(MOTOR4_PIN, 3);

  Serial.println("GPIO OK");
}

#endif
