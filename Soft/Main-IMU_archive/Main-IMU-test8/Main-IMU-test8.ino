// write data to SD
// file format changed txt to csv
// timestamp is used file name.

// Send data to CAN bus
// Get Data And Send to CAN bus
// CANBUSマイナス処理,送信データに符号データを含ませる。

// 5.各データをそれぞれ新しいタスクの関数内でCAN送信

// 6.ノード基板の送信部分を作成

// 7.SDwrite以外の機能をすべて実装

// 8.SDwrite実装したら終わり（のはず）

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
int SDstatus = 0;                       // 0: failed, 1: success
int year, month, date, hour, minu, sec; // RTC time

// MAIN
volatile float pitch = 0;
volatile float roll = 0;
volatile float yaw = 0;
volatile float ax = 0;
volatile float ay = 0;
volatile float az = 0;
volatile float gx = 0;
volatile float gy = 0;
volatile float gz = 0;
volatile float mx = 0;
volatile float my = 0;
volatile float mz = 0;
volatile float PcbTemp = 0;

// Node(OTHER)
float SpacePres = 0.00;
float SpaceTemp = 0.00;
float Pitot1 = 0.00;
float Pitot2 = 0.00;
float Pitot3 = 0.00;

int CANstatus = 1;      // 0: failed, 1: success
uint8_t sgnfcntDgt = 5; // 有効数字

// CAN address(MAIN)
const int pitchCANaddr = 0x01;
const int rollCANaddr = 0x02;
const int yawCANaddr = 0x03;
const int axCANaddr = 0x04;
const int ayCANaddr = 0x05;
const int azCANaddr = 0x06;
const int gxCANaddr = 0x07;
const int gyCANaddr = 0x08;
const int gzCANaddr = 0x09;
const int mxCANaddr = 0x0A;
const int myCANaddr = 0x0B;
const int mzCANaddr = 0x0C;
const int PcbTempCANaddr = 0x0D;
const int SDstatusCANaddr = 0x0E;

// CAN address(NODE)
const int SpacePresCANaddr = 0x0F;
const int SpaceTempCANaddr = 0x10;
const int Pitot1CANaddr = 0x12;
const int Pitot2CANaddr = 0x13;
const int Pitot3CANaddr = 0x14;

// function prototype
byte read_RTC(byte);
int BCD_to_int(byte);
void CAN_SEND(byte, int, uint8_t, uint8_t);
void chng(byte, int32_t, byte);

// RTOSTasks Prototype Declaration
void read_Angle(void);
void time_update(void);
void SD_write(void);
void GetBoardTemp(void);
void GetIMUdata(void);
void SEND_IMU(void);
void SEND_PcbTemp(void);
void CAN_RECEIVE(void);

void setup()
{
  // I2C setup
  Wire.begin();

  // Serial setup
  Serial.begin(115200);
  Serial.println("Start");

  // IMU setup
  MadgwickFilter.begin(100);
  BMX055_Init(); // BMX055の初期化

  // SD setup
  if (!SD.begin(4))
  {
    Serial.println("Card Mount Failed");
  }
  else
  {
    SDstatus = 1;
  }

  // CAN setup (500kbps)
  CAN.setPins(33, 32);
  if (!CAN.begin(500E3))
  {
    CANstatus = 0; // failed
  }

  /* create task */
  xTaskCreateUniversal(SD_write,   // function
                       "SD_write", // function name
                       4096,       // stack size
                       NULL,       // piont
                       5,          // priority
                       NULL,       // task handle
                       1);         // core number

  xTaskCreateUniversal(time_update,   // function
                       "time_update", // function name
                       4096,          // stack size
                       NULL,          // piont
                       2,             // priority
                       NULL,          // task handle
                       0);            // core number

  xTaskCreateUniversal(GetBoardTemp,   // function
                       "GetBoardTemp", // function name
                       4096,           // stack size
                       NULL,           // piont
                       2,              // priority
                       NULL,           // task handle
                       0);             // core number

  xTaskCreateUniversal(GetIMUdata,   // function
                       "GetIMUdata", // function name
                       4096,         // stack size
                       NULL,         // piont
                       3,            // priority
                       NULL,         // task handle
                       0);           // core number

  xTaskCreateUniversal(SEND_IMU,   // function
                       "SEND_IMU", // function name
                       4096,       // stack size
                       NULL,       // piont
                       2,          // priority
                       NULL,       // task handle
                       0);         // core number

  xTaskCreateUniversal(SEND_PcbTemp,   // function
                       "SEND_PcbTemp", // function name
                       4096,           // stack size
                       NULL,           // piont
                       1,              // priority
                       NULL,           // task handle
                       0);             // core number
  /*
  xTaskCreateUniversal(showIMUdata,   // function
                       "showIMUdata", // function name
                       4096,          // stack size
                       NULL,          // piont
                       1,             // priority
                       NULL,          // task handle
                       1);            // core number
                       */

  // 7.new
  xTaskCreateUniversal(CAN_RECEIVE,   // function
                       "CAN_RECEIVE", // function name
                       4096,          // stack size
                       NULL,          // piont
                       3,             // priority
                       NULL,          // task handle
                       0);            // core number
}

void SD_write(void *param)
{
  vTaskDelay(2500);
  int ms = 0;
  int DoCycleMs = 50; //(20hz)
  static int previousSec = -1;
  String filename = "/" + String(month) + "-" + String(date) + "-" + String(hour) + "-" + String(minu) + "-" + String(sec) + ".csv";
  File file = SD.open(filename, FILE_APPEND);
  file.println("timestamp, pitch, roll, yaw, ax, ay, az, gx, gy, gz, mx, my, mz, PcbTemp, SpacePres, SpaceTemp, Pitot1, Pitot2, Pitot3");
  file.close();
  while (1)
  {
    if (SDstatus == 1)
    {

      // write to SD
      while (ms < 1000)
      {

        File file = SD.open(filename, FILE_APPEND);
        if (!file)
        {
          Serial.println("Failed to open file");
          return;
        }

        if (sec != previousSec) // 秒が更新されたかチェック
        {
          ms = 0;            // secが更新されたらmsをリセット
          previousSec = sec; // 現在のsecをpreviousSecに更新
        }

        // write timestamp
        file.print(year);
        file.print("/");
        file.print(month);
        file.print("/");
        file.print(date);
        file.print("/");
        file.print(hour);
        file.print(":");
        file.print(minu);
        file.print(":");
        file.print(sec);
        file.print(".");
        file.print(ms);
        file.print(", ");

        // write AHRS
        file.print(pitch);
        file.print(", ");
        file.print(roll);
        file.print(", ");
        file.print(yaw);
        file.print(", ");
        file.print(ax);
        file.print(", ");
        file.print(ay);
        file.print(", ");
        file.print(az);
        file.print(", ");
        file.print(gx);
        file.print(", ");
        file.print(gy);
        file.print(", ");
        file.print(gz);
        file.print(", ");
        file.print(mx);
        file.print(", ");
        file.print(my);
        file.print(", ");
        file.print(mz);
        file.print(", ");

        // write PCB temp
        file.print(PcbTemp, 4);

        // Write Node data
        file.print(", ");
        file.print(SpacePres);
        file.print(", ");
        file.print(SpaceTemp);
        file.print(", ");
        file.print(Pitot1);
        file.print(", ");
        file.print(Pitot2);
        file.print(", ");
        file.print(Pitot3);

        file.println();

        file.close();

        Serial.print("pitch: ");
        Serial.println(pitch);
        Serial.print("roll: ");
        Serial.println(roll);
        Serial.print("yaw: ");
        Serial.println(yaw);
        Serial.print("ax: ");
        Serial.println(ax);
        Serial.print("ay: ");
        Serial.println(ay);
        Serial.print("az: ");
        Serial.println(az);
        Serial.print("gx: ");
        Serial.println(gx);
        Serial.print("gy: ");
        Serial.println(gy);
        Serial.print("gz: ");
        Serial.println(gz);
        Serial.print("mx: ");
        Serial.println(mx);
        Serial.print("my: ");
        Serial.println(my);
        Serial.print("mz: ");
        Serial.println(mz);
        Serial.print("PcbTemp: ");
        Serial.println(PcbTemp);
        Serial.print("SpacePres: ");
        Serial.println(SpacePres);
        Serial.print("SpaceTemp: ");
        Serial.println(SpaceTemp);
        Serial.print("Pitot1: ");
        Serial.println(Pitot1);
        Serial.print("Pitot2: ");
        Serial.println(Pitot2);
        Serial.print("Pitot3: ");
        Serial.println(Pitot3);

        ms += DoCycleMs;
        vTaskDelay(DoCycleMs);
      }
    }
  }
}

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
    sec = BCD_to_int(sec_r);
    minu = BCD_to_int(min_r);
    hour = BCD_to_int(hour_r);
    // day = BCD_to_int(day_r); // san, mon...
    date = BCD_to_int(date_r);
    month = BCD_to_int(month_r);
    year = BCD_to_int(year_r);

    /*
        Serial.print(year);
        Serial.print("/");
        Serial.print(month);
        Serial.print("/");
        Serial.print(date);
        Serial.print("/");
        Serial.print(hour);
        Serial.print(":");
        Serial.print(minu);
        Serial.print(":");
        Serial.println(sec);
    */
    vTaskDelay(500);
  }
}

void GetBoardTemp(void *param)
{
  while (1)
  {
    uint16_t uiVal; // 2バイト(16ビット)の領域
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

    PcbTemp = (float)iVal / 16.0; // change to float
    // Serial.println(PcbTemp, 4);   // send to serial monitor
    // check
    /*
    Serial.print(PcbTempCANaddr, HEX);
    Serial.print(",");
    Serial.print(PcbTempInt);
    Serial.print(",");
    Serial.print("0");
    Serial.print(",");
    Serial.println(sgnfcntDgt);
    */
    vTaskDelay(500); // wait 0.5 sec
  }
}

void SEND_PcbTemp(void *param)
{
  while (1)
  {
    // data sign check
    uint8_t PcbTempSign = PcbTemp < 0 ? 1 : 0;

    uint32_t PcbTempInt = (int)(abs(PcbTemp) * (pow(10, sgnfcntDgt)));

    CAN_SEND(PcbTempCANaddr, PcbTempInt, PcbTempSign, sgnfcntDgt); // sing 0:plus, 1:minus
    vTaskDelay(500);
  }
}

// show other data for debug
/*
void showIMUdata(void *param)
{
  while (1)
  {
    Serial.print(pitch);
    Serial.print(",");
    Serial.print(roll);
    Serial.print(",");
    Serial.print(yaw);
    Serial.print(", ");
    //===============//
    Serial.print(year);
    Serial.print("/");
    Serial.print(month);
    Serial.print("/");
    Serial.print(date);
    Serial.print("/");
    Serial.print(hour);
    Serial.print(":");
    Serial.print(minu);
    Serial.print(":");
    Serial.print(sec);
    Serial.print(", ");
    Serial.println(PcbTemp, 4);

    vTaskDelay(10);
  }
}
*/

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
    // change mg to G 0.00098

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

    vTaskDelay(1);
  }
}

void SEND_IMU(void *param)
{
  while (1)
  {
    // data sign check
    uint8_t axSign = ax < 0 ? 1 : 0;
    uint8_t aySign = ay < 0 ? 1 : 0;
    uint8_t azSign = az < 0 ? 1 : 0;
    uint8_t gxSign = gx < 0 ? 1 : 0;
    uint8_t gySign = gy < 0 ? 1 : 0;
    uint8_t gzSign = gz < 0 ? 1 : 0;
    uint8_t mxSign = mx < 0 ? 1 : 0;
    uint8_t mySign = my < 0 ? 1 : 0;
    uint8_t mzSign = mz < 0 ? 1 : 0;
    uint8_t pitchSign = pitch < 0 ? 1 : 0;
    uint8_t rollSign = roll < 0 ? 1 : 0;
    uint8_t yawSign = yaw < 0 ? 1 : 0;

    uint32_t axInt = (int)(abs(ax) * (pow(10, sgnfcntDgt)));
    uint32_t ayInt = (int)(abs(ay) * (pow(10, sgnfcntDgt)));
    uint32_t azInt = (int)(abs(az) * (pow(10, sgnfcntDgt)));
    uint32_t gxInt = (int)(abs(gx) * (pow(10, sgnfcntDgt)));
    uint32_t gyInt = (int)(abs(gy) * (pow(10, sgnfcntDgt)));
    uint32_t gzInt = (int)(abs(gz) * (pow(10, sgnfcntDgt)));
    uint32_t mxInt = (int)(abs(mx) * (pow(10, sgnfcntDgt)));
    uint32_t myInt = (int)(abs(my) * (pow(10, sgnfcntDgt)));
    uint32_t mzInt = (int)(abs(mz) * (pow(10, sgnfcntDgt)));
    uint32_t pitchInt = (int)(abs(pitch) * (pow(10, sgnfcntDgt)));
    uint32_t rollInt = (int)(abs(roll) * (pow(10, sgnfcntDgt)));
    uint32_t yawInt = (int)(abs(yaw) * (pow(10, sgnfcntDgt)));

    CAN_SEND(axCANaddr, axInt, axSign, sgnfcntDgt); // sing 0:plus, 1:minus
    // Serial.print("ax: ");
    // Serial.println(ax);
    CAN_SEND(ayCANaddr, ayInt, aySign, sgnfcntDgt); // sing 0:plus, 1:minus
    // Serial.print("ay: ");
    // Serial.println(ay);
    CAN_SEND(azCANaddr, azInt, azSign, sgnfcntDgt); // sing 0:plus, 1:minus
    // Serial.print("az: ");
    // Serial.println(az);
    CAN_SEND(gxCANaddr, gxInt, gxSign, sgnfcntDgt); // sing 0:plus, 1:minus
    // Serial.print("gx: ");
    // Serial.println(gx);
    CAN_SEND(gyCANaddr, gyInt, gySign, sgnfcntDgt); // sing 0:plus, 1:minus
    // Serial.print("gy: ");
    // Serial.println(gy);
    CAN_SEND(gzCANaddr, gzInt, gzSign, sgnfcntDgt); // sing 0:plus, 1:minus
    // Serial.print("gz: ");
    // Serial.println(gz);
    CAN_SEND(mxCANaddr, mxInt, mxSign, sgnfcntDgt); // sing 0:plus, 1:minus
    // Serial.print("mx: ");
    // Serial.println(mx);
    CAN_SEND(myCANaddr, myInt, mySign, sgnfcntDgt); // sing 0:plus, 1:minus
    // Serial.print("my: ");
    // Serial.println(my);
    CAN_SEND(mzCANaddr, mzInt, mzSign, sgnfcntDgt); // sing 0:plus, 1:minus
    // Serial.print("mz: ");
    // Serial.println(mz);
    CAN_SEND(pitchCANaddr, pitchInt, pitchSign, sgnfcntDgt); // sing 0:plus, 1:minus
    // Serial.print("pitch: ");
    // Serial.println(pitch);
    CAN_SEND(rollCANaddr, rollInt, rollSign, sgnfcntDgt); // sing 0:plus, 1:minus
    // Serial.print("roll: ");
    // Serial.println(roll);
    CAN_SEND(yawCANaddr, yawInt, yawSign, sgnfcntDgt); // sing 0:plus, 1:minus
    // Serial.print("yaw: ");
    // Serial.println(yaw);
    vTaskDelay(50);
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

// 32bit仮定でバイト送れるようにする。
void CAN_SEND(byte CANaddr, uint32_t data, byte sign, byte exp)
{
  // SEND 32bit data
  // devide data to 4byte
  CAN.beginPacket(CANaddr);
  CAN.write(sign); // 0:plus, 1:minus
  CAN.write(exp);  // 10^exp
  // Serial.println(CANaddr, HEX);
  // Serial.println(sign);
  // Serial.println(exp);
  for (int i = 3; i >= 0; i--)
  {
    byte dataByte = (byte)(data >> (i * 8));
    CAN.write(dataByte);
    // Serial.println(dataByte);
  }
  CAN.endPacket();
  // Serial.println("done");
}

void CAN_RECEIVE(void *param)
{
  while (1)
  {
    if (CANstatus == 1)
    {
      if (CAN.parsePacket())
      {
        byte CANaddr = CAN.packetId(); // パケットのID（アドレス）を取得
        byte sign = CAN.read();        // 符号を取得
        byte exp = CAN.read();         // 指数を取得

        int32_t data = 0;
        for (int i = 0; i < 4; i++)
        {
          byte dataByte = CAN.read(); // データの各バイトを受信
          data = (data << 8) | dataByte;
        }

        if (sign == 1)
        {
          data = -data;
        }

        chng(CANaddr, data, exp);
      }
    }
    vTaskDelay(1); // wait 1ms
  }
}

void chng(byte CANaddr, int32_t data, byte exp)
{
  float fData = (float)(data) * (pow(10, (-1 * exp)));
  switch (CANaddr)
  {
  case pitchCANaddr:
    pitch = fData;
    // Serial.print("pitch: ");
    // Serial.println(pitch);
    break;
  case rollCANaddr:
    roll = fData;
    // Serial.print("roll: ");
    //  Serial.println(roll);
    break;
  case yawCANaddr:
    yaw = fData;
    //  Serial.print("yaw: ");
    //  Serial.println(yaw);
    break;
  case axCANaddr:
    ax = fData;
    //  Serial.print("ax: ");
    // Serial.println(ax);
    break;
  case ayCANaddr:
    ay = fData;
    // Serial.print("ay: ");
    // Serial.println(ay);
    break;
  case azCANaddr:
    az = fData;
    // Serial.print("az: ");
    // Serial.println(az);
    break;
  case gxCANaddr:
    gx = fData;
    // Serial.print("gx: ");
    // Serial.println(gx);
    break;
  case gyCANaddr:
    gy = fData;
    // Serial.print("gy: ");
    // Serial.println(gy);
    break;
  case gzCANaddr:
    gz = fData;
    // Serial.print("gz: ");
    // Serial.println(gz);
    break;
  case mxCANaddr:
    mx = fData;
    // Serial.print("mx: ");
    // Serial.println(mx);
    break;
  case myCANaddr:
    my = fData;
    // Serial.print("my: ");
    // Serial.println(my);
    break;
  case mzCANaddr:
    mz = fData;
    // Serial.print("mz: ");
    // Serial.println(mz);
    break;
  case PcbTempCANaddr:
    PcbTemp = fData;
    // Serial.print("PcbTemp: ");
    // Serial.println(PcbTemp);
    break;
  case SpacePresCANaddr:
    SpacePres = fData;
    // Serial.print("SpacePres: ");
    // Serial.println(SpacePres);
    break;
  case SpaceTempCANaddr:
    SpaceTemp = fData;
    // Serial.print("SpaceTemp: ");
    // Serial.println(SpaceTemp);
    break;
  case Pitot1CANaddr:
    Pitot1 = fData;
    // Serial.print("Pitot1: ");
    // Serial.println(Pitot1);
    break;
  case Pitot2CANaddr:
    Pitot2 = fData;
    // Serial.print("Pitot2: ");
    // Serial.println(Pitot2);
    break;
  case Pitot3CANaddr:
    Pitot3 = fData;
    // Serial.print("Pitot3: ");
    // Serial.println(Pitot3);
    break;
  default:
    break;
  }
}

void loop()
{
  delay(1); // for WDT
}
