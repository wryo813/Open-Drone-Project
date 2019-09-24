#ifndef GPIO_config_h
#define GPIO_config_h

#include <Arduino.h>

//電源管理ピン設定
#define SHUTDOWN_PIN   25
#define PW_SWITCH_PIN  26
#define USB_STATUS_PIN 34
#define CHG_STATUS_PIN 35

//モーターピン設定
#define MOTOR1_PIN     27
#define MOTOR2_PIN     16
#define MOTOR3_PIN     4
#define MOTOR4_PIN     14

//PWM設定
/*
   PWM周波数とPWM分解能の設定
   T:周期[s],f:周波数[Hz],PWM_BIT:PWM分解能[bit],PWM_FREQ:PWM周波数[Hz] とする
   PWMのクロックタイマの周波数が80MHzの時、PWMのクロックタイマの周期は、
   T=1/f=1/(80×10^6)=12.5×10^(-9)s
   PWMのクロックタイマの周期とPWM分解能をかけた値の逆数がPWM周波数
   f=1/T=1/(12.5×10^(-9)×2^PWM_BIT[bit])=PWM_FREQ[Hz]
*/
//PWM周波数[Hz]
#define PWM_FREQ 19531
//PWM分解能[bit]
#define PWM_BIT 12

//GPIOの設定
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
