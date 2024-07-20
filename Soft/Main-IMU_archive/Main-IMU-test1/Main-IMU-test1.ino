// ESP32 SCL->GPIO22
// ESP32 SDA->GPIO21

/*
SDcard
2.DAT3>GPIO4
3.CMD>GPIO23
4.VDD>3.3v
5.CLk>GPIO18
6.VSS>GND
7.DAT0>GPIO19
*/

#include <CAN.h>
#include <Wire.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <freertos/FreeRTOS.h>
#include <MadgwickAHRS.h>
Madgwick MadgwickFilter;

// I2C address
const int ADT_addr = 0x48;
const int RTC_addr = 0x68;

const int BMX_Acc = 0x19;
const int BMX_Gyro = 0x69;
const int BMX_Mag = 0x13;

// Sensor status
int SDstatus = 0;   // 0: not initialized, 1: initialized
int hh, mm, ss, ms; // RTC time

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

// function prototype
byte read_RTC(byte);
int BCD_to_int(byte);

// RTOSTasks Prototype Declaration
void read_Angle(void);
void time_update(void);
void SD_write(void);
void GetBoardTemp(void);

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  Serial.println("Start");

  MadgwickFilter.begin(100);
  BMX055_Init(); // BMX055の初期化

  /*
  if (!SD.begin())
  {
    Serial.println("Card Mount Failed");
    return;
  }
  else
  {
    SDstatus = 1;
  }
  */

  /* create task */
  /*xTaskCreatePinnedToCore(SD_write,   // function
                          "SD_write", // function name
                          4096,       // stack size
                          NULL,       // piont
                          1,          // priority
                          NULL,       // task handle
                          1);         // core number
                          */

  xTaskCreatePinnedToCore(time_update,   // function
                          "time_update", // function name
                          4096,          // stack size
                          NULL,          // piont
                          2,             // priority
                          NULL,          // task handle
                          1);

  xTaskCreatePinnedToCore(GetBoardTemp,   // function
                          "GetBoardTemp", // function name
                          4096,           // stack size
                          NULL,           // piont
                          3,              // priority
                          NULL,           // task handle
                          1);             // core number

  xTaskCreatePinnedToCore(GetIMUdata,   // function
                          "GetIMUdata", // function name
                          4096,         // stack size
                          NULL,         // piont
                          1,            // priority
                          NULL,         // task handle
                          0);           // core number

  xTaskCreatePinnedToCore(showIMUdata,   // function
                          "showIMUdata", // function name
                          4096,          // stack size
                          NULL,          // piont
                          2,             // priority
                          NULL,          // task handle
                          0);            // core number
}

/*
void SD_write(void *param)
{
  int DoCycleMs = 50; //(20hz)

  while (1)
  {
    ms = 0; // reset ms

    // write to SD
    while (ms < 1000)
    {
      File file = SD.open("/MainIMU.txt", FILE_APPEND);
      if (!file)
      {
        Serial.println("Failed to open file");
        return;
      }

      file.print(hh);
      file.print(":");
      file.print(mm);
      file.print(":");
      file.print(ss);
      file.print(".");
      file.print(ms);

      file.close();

      vTaskDelay(50);
      ms += DoCycleMs;
      vTaskDelay(DoCycleMs);
    }
  }
}
*/

byte read_RTC(byte addr)
{
  Wire.beginTransmission(RTC_addr);
  Wire.write(addr);
  Wire.endTransmission(false);
  Wire.requestFrom(RTC_addr, 1, true);
  byte data = Wire.read();
  return data;
}

int BCD_to_int(byte BCD)
{
  uint8_t ones = BCD & B1111;     // 9
  uint8_t tens = (BCD >> 4) * 10; // 50
  uint8_t data = tens + ones;

  return data;
}

void time_update(void *param)
{
  // int DoCycleMs = 100; //(10hz)
  while (1)
  {
    // read rtc time
    // format is BCD
    byte sec_r = read_RTC(0x00);
    byte min_r = read_RTC(0x01);
    byte hour_r = read_RTC(0x02);
    // byte day_r = read_RTC(0x03);
    byte date_r = read_RTC(0x04);
    byte month_r = read_RTC(0x05);
    byte year_r = read_RTC(0x06);

    // convert BCD to int
    int sec = BCD_to_int(sec_r);
    int min = BCD_to_int(min_r);
    int hour = BCD_to_int(hour_r);
    // int day = BCD_to_int(day_r); // san, mon...
    int date = BCD_to_int(date_r);
    int month = BCD_to_int(month_r);
    int year = BCD_to_int(year_r);

    /*
        Serial.print(year);
        Serial.print("/");
        Serial.print(month);
        Serial.print("/");
        Serial.print(date);
        Serial.print("/");
        Serial.print(hour);
        Serial.print(":");
        Serial.print(min);
        Serial.print(":");
        Serial.println(sec);
    */
    vTaskDelay(1000);
  }
}

void GetBoardTemp(void *param)
{
  while (1)
  {
    uint16_t uiVal; // 2バイト(16ビット)の領域
    float fVal;
    int iVal;

    Wire.requestFrom(ADT_addr, 2); // request 2 bytes from ADT

    uiVal = (uint8_t)Wire.read() << 8; // read 1 byte and shift to upper byte
    uiVal |= Wire.read();              // read 1 byte and OR to lower byte

    uiVal >>= 3; // シフトで13bit化

    if (uiVal & 0x1000) // check sign bit
    {
      iVal = uiVal - 0x2000; // minus value
    }
    else
    {
      iVal = uiVal; // plus value
    }

    fVal = (float)iVal / 16.0; // change to float
    // Serial.println(fVal, 4);   // send to serial monitor
    vTaskDelay(1000); // wait 1 sec
  }
}

void showIMUdata(void *param)
{
  while (1)
  {
    Serial.print(pitch);
    Serial.print(",");
    Serial.print(roll);
    Serial.print(",");
    Serial.print(yaw);
    Serial.println("");
    vTaskDelay(10);
  }
}

void GetIMUdata(void *param)
{
  while (1)
  {
    unsigned int data[8];

    // get Acc
    for (int i = 0; i < 6; i++)
    {
      Wire.beginTransmission(BMX_Acc);
      Wire.write((2 + i)); // Select data register
      Wire.endTransmission();
      Wire.requestFrom(BMX_Acc, 1); // Request 1 byte of data
      // Read 6 bytes of data
      // ax lsb, ax msb, ay lsb, ay msb, az lsb, az msb
      if (Wire.available() == 1)
        data[i] = Wire.read();
    }
    // Convert the data to 12-bits
    ax = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
    if (ax > 2047)
      ax -= 4096;
    ay = ((data[3] * 256) + (data[2] & 0xF0)) / 16;
    if (ay > 2047)
      ay -= 4096;
    az = ((data[5] * 256) + (data[4] & 0xF0)) / 16;
    if (az > 2047)
      az -= 4096;
    ax = ax * 0.00098; // range = +/-2g
    ay = ay * 0.00098; // range = +/-2g
    az = az * 0.00098; // range = +/-2g
    // 上記の乗数は秋月のマニュアル通りですが、誤植とのことです。
    // 正しくは、mg換算で0.98、G換算で0.00098
    // とのことです。

    // get Gyro
    for (int i = 0; i < 6; i++)
    {
      Wire.beginTransmission(BMX_Gyro);
      Wire.write((2 + i)); // Select data register
      Wire.endTransmission();
      Wire.requestFrom(BMX_Gyro, 1); // Request 1 byte of data
      if (Wire.available() == 1)
        data[i] = Wire.read();
    }
    // Convert the data
    gx = (data[1] * 256) + data[0];
    if (gx > 32767)
      gx -= 65536;
    gy = (data[3] * 256) + data[2];
    if (gy > 32767)
      gy -= 65536;
    gz = (data[5] * 256) + data[4];
    if (gz > 32767)
      gz -= 65536;

    gx = gx * 0.0038; //  Full scale = +/- 125 degree/s
    gy = gy * 0.0038; //  Full scale = +/- 125 degree/s
    gz = gz * 0.0038; //  Full scale = +/- 125 degree/s

    // get Mag
    for (int i = 0; i < 8; i++)
    {
      Wire.beginTransmission(BMX_Mag);
      Wire.write((0x42 + i)); // Select data register
      Wire.endTransmission();
      Wire.requestFrom(BMX_Mag, 1); // Request 1 byte of data
      // Read 6 bytes of data
      // mx lsb, mx msb, my lsb, my msb, mz lsb, mz msb
      if (Wire.available() == 1)
        data[i] = Wire.read();
    }
    // Convert the data
    mx = ((data[1] << 5) | (data[0] >> 3));
    if (mx > 4095)
      mx -= 8192;
    my = ((data[3] << 5) | (data[2] >> 3));
    if (my > 4095)
      my -= 8192;
    mz = ((data[5] << 7) | (data[4] >> 1));
    if (mz > 16383)
      mz -= 32768;

    MadgwickFilter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    pitch = MadgwickFilter.getPitch();
    roll = MadgwickFilter.getRoll();
    yaw = MadgwickFilter.getYaw();
  }
}

void BMX055_Init() // BMX055の初期化
{
  Wire.beginTransmission(BMX_Acc);
  Wire.write(0x0F); // Select PMU_Range register
  Wire.write(0x03); // Range = +/- 2g
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(BMX_Acc);
  Wire.write(0x10); // Select PMU_BW register
  Wire.write(0x08); // Bandwidth = 7.81 Hz
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(BMX_Acc);
  Wire.write(0x11); // Select PMU_LPW register
  Wire.write(0x00); // Normal mode, Sleep duration = 0.5ms
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(BMX_Gyro);
  Wire.write(0x0F); // Select Range register
  Wire.write(0x04); // Full scale = +/- 125 degree/s
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(BMX_Gyro);
  Wire.write(0x10); // Select Bandwidth register
  Wire.write(0x07); // ODR = 100 Hz
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(BMX_Gyro);
  Wire.write(0x11); // Select LPM1 register
  Wire.write(0x00); // Normal mode, Sleep duration = 2ms
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(BMX_Mag);
  Wire.write(0x4B); // Select Mag register
  Wire.write(0x83); // Soft reset
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(BMX_Mag);
  Wire.write(0x4B); // Select Mag register
  Wire.write(0x01); // Soft reset
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(BMX_Mag);
  Wire.write(0x4C); // Select Mag register
  Wire.write(0x00); // Normal Mode, ODR = 10 Hz 0x00 //100Hz 0x07
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(BMX_Mag);
  Wire.write(0x4E); // Select Mag register
  Wire.write(0x84); // X, Y, Z-is enabled
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(BMX_Mag);
  Wire.write(0x51); // Select Mag register
  Wire.write(0x04); // No. of Repetitions for X-Y is = 9
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(BMX_Mag);
  Wire.write(0x52); // Select Mag register
  Wire.write(0x16); // No. of Repetitions for Z-Axis = 15
  Wire.endTransmission();
}

void loop()
{
}
