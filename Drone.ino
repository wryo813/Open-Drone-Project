#include <Wire.h>
#include <MadgwickAHRS.h>

// BMX055　加速度センサのI2Cアドレス
#define Addr_Accl 0x19  // (JP1,JP2,JP3 = Openの時)
// BMX055　ジャイロセンサのI2Cアドレス
#define Addr_Gyro 0x69  // (JP1,JP2,JP3 = Openの時)
// BMX055　磁気センサのI2Cアドレス
#define Addr_Mag 0x13   // (JP1,JP2,JP3 = Openの時)

#define MOTOR1_PIN 12
#define MOTOR2_PIN 13
#define MOTOR3_PIN 14
#define MOTOR4_PIN 16

#define P_GAIN 5
#define TARGET 0

Madgwick MadgwickFilter;
unsigned long microsPerReading, microsPrevious;

// センサーの値を保存するグローバル関数
float xAccl = 0.00;
float yAccl = 0.00;
float zAccl = 0.00;
float xGyro = 0.00;
float yGyro = 0.00;
float zGyro = 0.00;
int   xMag  = 0;
int   yMag  = 0;
int   zMag  = 0;

float xf = 0;
float yf = 0;
float zf = 0;

float roll, pitch, yaw;
float roll_raw, pitch_raw, yaw_raw;
float motor1_angle_now, motor2_angle_now, motor3_angle_now, motor4_angle_now;
void setup()
{

  MadgwickFilter.begin(22); //100Hz
  // Wire(Arduino-I2C)の初期化
  Wire.begin();
  // デバック用シリアル通信は115200bps
  Serial.begin(115200);
  //BMX055 初期化
  BMX055_Init();
  delay(300);

}

void loop()
{


  if ( Serial.available() > 0 ) {
    String str = Serial.readStringUntil('\n');

    if ( str == "") {
      Serial.println(">");
    }

    else if ( str == "start" ) {
      Serial.println(">" + str);
      for (;;) {
        String cmd;
        float motor1_level;
        float motor2_level;
        float motor3_level;
        float motor4_level;
        float motor1_angle_deviation;
        float motor2_angle_deviation;
        float motor3_angle_deviation;
        float motor4_angle_deviation;
        int motor1_duty;
        int motor2_duty;
        int motor3_duty;
        int motor4_duty;
        get_imu_data();

        motor1_angle_deviation = TARGET - motor1_angle_now;
        motor2_angle_deviation = TARGET - motor2_angle_now;
        motor3_angle_deviation = TARGET - motor3_angle_now;
        motor4_angle_deviation = TARGET - motor4_angle_now;
        
        motor1_level = P_GAIN * motor1_angle_deviation;
        motor2_level = P_GAIN * motor2_angle_deviation;
        motor3_level = P_GAIN * motor3_angle_deviation;
        motor4_level = P_GAIN * motor4_angle_deviation;

        motor1_duty = (int) motor1_level;
        motor2_duty = (int) motor2_level;
        motor3_duty = (int) motor3_level;
        motor4_duty = (int) motor4_level;
        
        
        if( motor1_duty >= 255 ) motor1_duty = 255;
        if( motor1_duty <= 0 ) motor1_duty = 0;
        if( motor2_duty >= 255 ) motor2_duty = 255;
        if( motor2_duty <= 0 ) motor2_duty = 0;
        if( motor3_duty >= 255 ) motor3_duty = 255;
        if( motor3_duty <= 0 ) motor3_duty = 0;
        if( motor4_duty >= 255 ) motor4_duty = 255;
        if( motor4_duty <= 0 ) motor4_duty = 0;


        analogWrite(MOTOR1_PIN, motor1_duty);
        analogWrite(MOTOR2_PIN, motor2_duty);
        analogWrite(MOTOR3_PIN, motor3_duty);
        analogWrite(MOTOR4_PIN, motor4_duty);
        
        
        Serial.print(motor1_duty);
        Serial.print(" ");
        Serial.print(motor2_duty);
        Serial.print(" ");
        Serial.print(motor3_duty);
        Serial.print(" ");
        Serial.println(motor4_duty);

        
        if (Serial.available() > 0) {
          cmd = Serial.readStringUntil('\n');
        }
        if (cmd == "stop") {
          Serial.println(">" + cmd);
          break;
        }
        delay(50);
      }
    }

    else if ( str == "get_all" ) {
      Serial.println(">" + str);
      for (;;) {
        String cmd;
        get_imu_data();
        Serial.print("Orientation: ");
        Serial.print("YAW=");
        Serial.print(yaw);
        Serial.print(" ");
        Serial.print("PITCH=");
        Serial.print(pitch);
        Serial.print(" ");
        Serial.print("ROOL=");
        Serial.print(roll);
        Serial.print(" ");
        Serial.print("MOTOR1_ANGLE_NOW=");
        Serial.print(motor1_angle_now);
        Serial.print(" ");
        Serial.print("MOTOR2_ANGLE_NOW=");
        Serial.print(motor2_angle_now);
        Serial.print(" ");
        Serial.print("MOTOR3_ANGLE_NOW=");
        Serial.print(motor3_angle_now);
        Serial.print(" ");
        Serial.print("MOTOR3_ANGLE_NOW=");
        Serial.println(motor4_angle_now);
        
        if (Serial.available() > 0) {
          cmd = Serial.readStringUntil('\n');
        }
        if (cmd == "stop") {
          Serial.println(">" + cmd);
          break;
        }
        delay(50);
      }
    }

    else if ( str == "log_all" ) {
    }

    else {
      Serial.println(">" + str);
      Serial.println("[ERROR]");
    }
  }
}

void get_imu_data() {
  //BMX055 加速度の読み取り
  BMX055_Accl();
  //BMX055 ジャイロの読み取り
  BMX055_Gyro();
  //BMX055 磁気の読み取り
  BMX055_Mag();

  xf = (float) xMag;
  yf = (float) yMag;
  zf = (float) zMag;

  MadgwickFilter.updateIMU(xGyro, yGyro, zGyro, xAccl, yAccl, zAccl);
  roll_raw     = MadgwickFilter.getRoll();
  pitch_raw    = MadgwickFilter.getPitch();
  yaw_raw      = MadgwickFilter.getYaw();

  roll         = roll_raw;
  pitch        = -pitch_raw;
  yaw          = -yaw_raw;

  motor1_angle_now = roll + pitch;
  motor2_angle_now = -roll + pitch;
  motor3_angle_now = - roll - pitch;  
  motor4_angle_now = roll - pitch;
}

//=====================================================================================//
void BMX055_Init()
{
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x0F); // Select PMU_Range register
  Wire.write(0x03);   // Range = +/- 2g
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x10);  // Select PMU_BW register
  Wire.write(0x08);  // Bandwidth = 7.81 Hz
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x11);  // Select PMU_LPW register
  Wire.write(0x00);  // Normal mode, Sleep duration = 0.5ms
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x0F);  // Select Range register
  Wire.write(0x04);  // Full scale = +/- 125 degree/s
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x10);  // Select Bandwidth register
  Wire.write(0x07);  // ODR = 100 Hz
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x11);  // Select LPM1 register
  Wire.write(0x00);  // Normal mode, Sleep duration = 2ms
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x83);  // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x01);  // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4C);  // Select Mag register
  Wire.write(0x00);  // Normal Mode, ODR = 10 Hz
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4E);  // Select Mag register
  Wire.write(0x84);  // X, Y, Z-Axis enabled
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x51);  // Select Mag register
  Wire.write(0x04);  // No. of Repetitions for X-Y Axis = 9
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x52);  // Select Mag register
  Wire.write(0x16);  // No. of Repetitions for Z-Axis = 15
  Wire.endTransmission();
}
//=====================================================================================//
void BMX055_Accl()
{
  int data[6];
  for (int i = 0; i < 6; i++)
  {
    Wire.beginTransmission(Addr_Accl);
    Wire.write((2 + i));// Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Accl, 1);// Request 1 byte of data
    // Read 6 bytes of data
    // xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data to 12-bits
  xAccl = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
  if (xAccl > 2047)  xAccl -= 4096;
  yAccl = ((data[3] * 256) + (data[2] & 0xF0)) / 16;
  if (yAccl > 2047)  yAccl -= 4096;
  zAccl = ((data[5] * 256) + (data[4] & 0xF0)) / 16;
  if (zAccl > 2047)  zAccl -= 4096;
  xAccl = xAccl * 0.0098; // renge +-2g
  yAccl = yAccl * 0.0098; // renge +-2g
  zAccl = zAccl * 0.0098; // renge +-2g
}
//=====================================================================================//
void BMX055_Gyro()
{
  int data[6];
  for (int i = 0; i < 6; i++)
  {
    Wire.beginTransmission(Addr_Gyro);
    Wire.write((2 + i));    // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Gyro, 1);    // Request 1 byte of data
    // Read 6 bytes of data
    // xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data
  xGyro = (data[1] * 256) + data[0];
  if (xGyro > 32767)  xGyro -= 65536;
  yGyro = (data[3] * 256) + data[2];
  if (yGyro > 32767)  yGyro -= 65536;
  zGyro = (data[5] * 256) + data[4];
  if (zGyro > 32767)  zGyro -= 65536;

  xGyro = xGyro * 0.0038; //  Full scale = +/- 125 degree/s
  yGyro = yGyro * 0.0038; //  Full scale = +/- 125 degree/s
  zGyro = zGyro * 0.0038; //  Full scale = +/- 125 degree/s
}
//=====================================================================================//
void BMX055_Mag()
{
  int data[8];
  for (int i = 0; i < 8; i++)
  {
    Wire.beginTransmission(Addr_Mag);
    Wire.write((0x42 + i));    // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Mag, 1);    // Request 1 byte of data
    // Read 6 bytes of data
    // xMag lsb, xMag msb, yMag lsb, yMag msb, zMag lsb, zMag msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data
  xMag = ((data[1] << 8) | (data[0] >> 3));
  if (xMag > 4095)  xMag -= 8192;
  yMag = ((data[3] << 8) | (data[2] >> 3));
  if (yMag > 4095)  yMag -= 8192;
  zMag = ((data[5] << 8) | (data[4] >> 3));
  if (zMag > 16383)  zMag -= 32768;
}
