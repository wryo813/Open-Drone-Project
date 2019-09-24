#ifndef IMU_h
#define IMU_h

#include <Arduino.h>

typedef enum {
  x,
  y,
  z
} axis;

class BMX055 {
  private:
    float xAccl = 0.00;
    float yAccl = 0.00;
    float zAccl = 0.00;
    float xGyro = 0.00;
    float yGyro = 0.00;
    float zGyro = 0.00;
    int   xMag  = 0;
    int   yMag  = 0;
    int   zMag  = 0;

  public:
    //BMX055の初期化
    void begin();
    //加速度センサにデータ取得リクエスト
    void begin_Accl();
    //ジャイロセンサにデータ取得リクエスト
    void begin_Gyro();
    //地磁気センサにデータ取得リクエスト
    void begin_Mag();
    //引数の軸に応じて加速度センサの値を返す
    float Accl(axis type);
    //引数の軸に応じてジャイロセンサの値を返す
    float Gyro(axis type);
    //引数の軸に応じて地磁気センサの値を返す
    int Mag(axis type);
};

#endif
