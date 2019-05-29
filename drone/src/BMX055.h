#ifndef IMU_h
#define IMU_h

#include <Arduino.h>

typedef enum{
  x,
  y,
  z
}axis;

class BMX055 {
  private:
  
  public:
    void begin();
    float Accl(axis type);
    float Gyro(axis type);
    int Mag(axis type);
};


#endif
