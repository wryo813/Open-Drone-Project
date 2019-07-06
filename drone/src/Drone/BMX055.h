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
    void begin();

    void begin_Accl();
    void begin_Gyro();
    void begin_Mag();
    float Accl(axis type);
    float Gyro(axis type);
    int Mag(axis type);
};

#endif
