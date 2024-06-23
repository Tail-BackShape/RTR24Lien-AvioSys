
#include<Wire.h>
#include<MadgwickAHRS.h>
Madgwick MadgwickFilter;
#include <Ticker.h>
Ticker magdwickticker;
#define MagdwickHz 100 //周波数。１秒間に何回データを読み込むかの値
const float magdwickinterval = 1000 / MagdwickHz;

// BMX055
#define Addr_Accl 0x19  // 加速度センサのI2Cアドレス
#define Addr_Gyro 0x69  // ジャイロセンサのI2Cアドレス
#define Addr_Mag 0x13   // 磁気センサのI2Cアドレス

// センサーの値を保存するグローバル変数
float ax = 0.00;
float ay = 0.00;
float az = 0.00;
float gx = 0.00;
float gy = 0.00;
float gz = 0.00;
int mx = 0.00;
int my = 0.00;
int mz = 0.00;
float pitch = 0.00;
float roll = 0.00;
float yaw = 0.00;

void BMX055_Init() //BMX055の初期化
{
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
  Wire.write(0x00);  // Normal Mode, ODR = 10 Hz 0x00 //100Hz 0x07
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4E);  // Select Mag register
  Wire.write(0x84);  // X, Y, Z-is enabled
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x51);  // Select Mag register
  Wire.write(0x04);  // No. of Repetitions for X-Y is = 9
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x52);  // Select Mag register
  Wire.write(0x16);  // No. of Repetitions for Z-Axis = 15
  Wire.endTransmission();
}

void BMX055_All() //BMX055から全データとMadgwickフィルタの結果を取得する
{
  unsigned int data[8];

  //加速度データを取得する
  for (int i = 0; i < 6; i++)
  {
    Wire.beginTransmission(Addr_Accl);
    Wire.write((2 + i));// Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Accl, 1);// Request 1 byte of data
    // Read 6 bytes of data
    // ax lsb, ax msb, ay lsb, ay msb, az lsb, az msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data to 12-bits
  ax = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
  if (ax > 2047)  ax -= 4096;
  ay = ((data[3] * 256) + (data[2] & 0xF0)) / 16;
  if (ay > 2047)  ay -= 4096;
  az = ((data[5] * 256) + (data[4] & 0xF0)) / 16;
  if (az > 2047)  az -= 4096;
  ax = ax * 0.0098; // range = +/-2g
  ay = ay * 0.0098; // range = +/-2g
  az = az * 0.0098; // range = +/-2g
  // 上記の乗数は秋月のマニュアル通りですが、誤植とのことです。
  // 正しくは、mg換算で0.98、G換算で0.00098
  // とのことです。

  //ジャイロデータを取得する
  for (int i = 0; i < 6; i++)
  {
    Wire.beginTransmission(Addr_Gyro);
    Wire.write((2 + i));    // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Gyro, 1);    // Request 1 byte of data
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data
  gx = (data[1] * 256) + data[0];
  if (gx > 32767)  gx -= 65536;
  gy = (data[3] * 256) + data[2];
  if (gy > 32767)  gy -= 65536;
  gz = (data[5] * 256) + data[4];
  if (gz > 32767)  gz -= 65536;

  gx = gx * 0.0038; //  Full scale = +/- 125 degree/s
  gy = gy * 0.0038; //  Full scale = +/- 125 degree/s
  gz = gz * 0.0038; //  Full scale = +/- 125 degree/s

  //コンパスデータを取得する
  for (int i = 0; i < 8; i++)
  {
    Wire.beginTransmission(Addr_Mag);
    Wire.write((0x42 + i));    // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Mag, 1);    // Request 1 byte of data
    // Read 6 bytes of data
    // mx lsb, mx msb, my lsb, my msb, mz lsb, mz msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data
  mx = ((data[1] << 5) | (data[0] >> 3));
  if (mx > 4095)  mx -= 8192;
  my = ((data[3] << 5) | (data[2] >> 3));
  if (my > 4095)  my -= 8192;
  mz = ((data[5] << 7) | (data[4] >> 1));
  if (mz > 16383)  mz -= 32768;

  MadgwickFilter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
  float pitch = MadgwickFilter.getPitch();
  float roll  = MadgwickFilter.getRoll();
  float yaw   = MadgwickFilter.getYaw();
  Serial.print(pitch);
  Serial.print(",");
  Serial.print(roll);
  Serial.print(",");
  Serial.print(yaw);
  Serial.println("");
}

void setup()
{
  MadgwickFilter.begin(MagdwickHz);
  Wire.begin();//I2Cの初期化
  Serial.begin(115200); //デバッグ用シリアル通信
  BMX055_Init(); //BMX055の初期化
  //#define MagdwickHz で設定した周波数にてセンサー読み取りを割り込み実行
  magdwickticker.attach_ms(magdwickinterval, BMX055_All);
  delay(300);
}

void loop()
{
}
