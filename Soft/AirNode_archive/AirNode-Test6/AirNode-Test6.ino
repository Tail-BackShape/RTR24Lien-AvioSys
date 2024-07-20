// write data to SD
// file format changed txt to csv
// timestamp is used file name.

// Send data to CAN bus
// Get Data And Send to CAN bus
// CANBUSマイナス処理,送信データに符号データを含ませる。

// 5.各データをそれぞれ新しいタスクの関数内でCAN送信

// 6.ノード基板の送信部分を作成

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
#include <Adafruit_BMP280.h>
#include <Adafruit_ADS1X15.h>
Adafruit_BMP280 bmp;  // use I2C interface
Adafruit_ADS1115 ads; // Use this for the 16-bit version

// I2C address
const int RTC_addr = 0x68;
const int BMP280_addr = 0x76;

// Sensor status
int SDstatus = 0;                       // 0: failed, 1: success
int BMP280_status = 1;                  // 0: failed, 1: success
int year, month, date, hour, minu, sec; // RTC time

/*
// Main(OTHER)
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
float PcbTemp = 0.00;
*/
// Node(THIS)
float SpacePres = 0.00;
float SpaceTemp = 0.00;
float Alt = 0.00;
float Pitot1 = 0.00;
float Pitot2 = 0.00;
float Pitot3 = 0.00;

uint8_t CANstatus = 1;  // 0: failed, 1: success
uint8_t sgnfcntDgt = 5; // 有効数字

/*
// CAN address(MAIN)
byte pitchCANaddr = 0x01;
byte rollCANaddr = 0x02;
byte yawCANaddr = 0x03;
byte axCANaddr = 0x04;
byte ayCANaddr = 0x05;
byte azCANaddr = 0x06;
byte gxCANaddr = 0x07;
byte gyCANaddr = 0x08;
byte gzCANaddr = 0x09;
byte mxCANaddr = 0x0A;
byte myCANaddr = 0x0B;
byte mzCANaddr = 0x0C;
byte PcbTempCANaddr = 0x0D;
byte SDstatusCANaddr = 0x0E;
*/

// CAN address(THIS-NODE)
byte SpacePresCANaddr = 0x0F;
byte SpaceTempCANaddr = 0x10;
byte AltCANaddr = 0x11;
byte Pitot1CANaddr = 0x12;
byte Pitot2CANaddr = 0x13;
byte Pitot3CANaddr = 0x14;

// function prototype
byte read_RTC(byte);
int BCD_to_int(byte);
void CAN_SEND(byte, uint32_t, uint8_t, uint8_t);

// RTOSTasks Prototype Declaration
void read_Angle(void);
void time_update(void);
// void SD_write(void);
void Get_SpaceData(void); // Pressure and Temp
void SEND_SpaceData(void);
void Get_PitotData(void); // 3 Pitot tube Volts
void SEND_PitotData(void);

void setup()
{
  // I2C setup
  Wire.begin();

  // Serial setup
  Serial.begin(115200);
  Serial.println("Start");

  // BMP280 setup
  bmp.begin(BMP280_addr);

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  // ADS1115 setup
  // 後でステータス追加
  ads.setGain(GAIN_ONE); // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  ads.begin();

  // SD setup
  if (!SD.begin())
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
  /*
  xTaskCreateUniversal(SD_write,   // function
                       "SD_write", // function name
                       4096,       // stack size
                       NULL,       // piont
                       1,          // priority
                       NULL,       // task handle
                       1);         // core number
                       */

  xTaskCreateUniversal(time_update,   // function
                       "time_update", // function name
                       4096,          // stack size
                       NULL,          // piont
                       2,             // priority
                       NULL,          // task handle
                       1);            // core number

  xTaskCreateUniversal(Get_SpaceData,   // function
                       "Get_SpaceData", // function name
                       4096,            // stack size
                       NULL,            // piont
                       2,               // priority
                       NULL,            // task handle
                       1);              // core number

  xTaskCreateUniversal(SEND_SpaceData,   // function
                       "SEND_SpaceData", // function name
                       4096,             // stack size
                       NULL,             // piont
                       2,                // priority
                       NULL,             // task handle
                       1);               // core number

  // this
  xTaskCreateUniversal(Get_PitotData,   // function
                       "Get_PitotData", // function name
                       8192,            // stack size
                       NULL,            // piont
                       2,               // priority
                       NULL,            // task handle
                       0);              // core number

  xTaskCreateUniversal(SEND_PitotData,   // function
                       "SEND_PitotData", // function name
                       4096,             // stack size
                       NULL,             // piont
                       2,                // priority
                       NULL,             // task handle
                       1);               // core number
}

/*
void SD_write(void *param)
{
  vTaskDelay(1500);
  int DoCycleMs = 50; //(20hz)
  String filename = "/" + String(month) + "-" + String(date) + "-" + String(hour) + "-" + String(minu) + "-" + String(sec) + ".csv";
  File file = SD.open(filename, FILE_APPEND);
  file.println("timestamp, pitch, roll, yaw, ax, ay, az, gx, gy, gz, mx, my, mz, PcbTemp");
  file.close();
  while (1)
  {
    if (SDstatus == 1)
    {
      int ms = 0; // reset ms

      // write to SD
      while (ms < 1000)
      {
        File file = SD.open(filename, FILE_APPEND);
        if (!file)
        {
          Serial.println("Failed to open file");
          return;
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
        file.println();

        file.close();

        vTaskDelay(50);
        ms += DoCycleMs;
        vTaskDelay(DoCycleMs);
      }
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
    vTaskDelay(1000);
  }
}

void Get_SpaceData(void *param)
{
  while (1)
  {
    if (BMP280_status == 1)
    {
      SpaceTemp = bmp.readTemperature();
      SpacePres = bmp.readPressure();
      Serial.print("Temp: ");
      Serial.println(SpaceTemp, 6);
      Serial.print("Pres: ");
      Serial.println(SpacePres, 6);
    }
    vTaskDelay(200);
  }
}

void SEND_SpaceData(void *param)
{
  uint8_t ASPsgnfcntDgt = 4;
  while (1)
  {
    uint8_t SpaceTempSign = SpaceTemp < 0 ? 1 : 0;
    uint8_t SpacePresSign = SpacePres < 0 ? 1 : 0;

    uint32_t SpaceTempInt = (int)(abs(SpaceTemp) * (pow(10, sgnfcntDgt)));
    uint32_t SpacePresInt = (int)(abs(SpacePres) * (pow(10, ASPsgnfcntDgt)));

    CAN_SEND(SpaceTempCANaddr, SpaceTempInt, SpaceTempSign, sgnfcntDgt);
    CAN_SEND(SpacePresCANaddr, SpacePresInt, SpacePresSign, ASPsgnfcntDgt);
    vTaskDelay(200);
  }
}

void Get_PitotData(void *param)
{
  int16_t adc0, adc1, adc2;
  while (1)
  {
    // Get Pitot ADC data
    adc0 = ads.readADC_SingleEnded(0);
    adc1 = ads.readADC_SingleEnded(1);
    adc2 = ads.readADC_SingleEnded(2);

    // Convert to Volts
    Pitot1 = ads.computeVolts(adc0);
    Pitot2 = ads.computeVolts(adc1);
    Pitot3 = ads.computeVolts(adc2);

    Serial.print("Pitot1: ");
    Serial.println(Pitot1, 6);
    Serial.print("Pitot2: ");
    Serial.println(Pitot2, 6);
    Serial.print("Pitot3: ");
    Serial.println(Pitot3, 6);

    vTaskDelay(200);
  }
}

void SEND_PitotData(void *param)
{
  while (1)
  {
    uint8_t Pitot1Sign = Pitot1 < 0 ? 1 : 0;
    uint8_t Pitot2Sign = Pitot2 < 0 ? 1 : 0;
    uint8_t Pitot3Sign = Pitot3 < 0 ? 1 : 0;

    uint32_t Pitot1Int = (int)(abs(Pitot1) * (pow(10, sgnfcntDgt)));
    uint32_t Pitot2Int = (int)(abs(Pitot2) * (pow(10, sgnfcntDgt)));
    uint32_t Pitot3Int = (int)(abs(Pitot3) * (pow(10, sgnfcntDgt)));

    CAN_SEND(Pitot1CANaddr, Pitot1Int, Pitot1Sign, sgnfcntDgt);
    CAN_SEND(Pitot2CANaddr, Pitot2Int, Pitot2Sign, sgnfcntDgt);
    CAN_SEND(Pitot3CANaddr, Pitot3Int, Pitot3Sign, sgnfcntDgt);
    vTaskDelay(200);
  }
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

void loop()
{
  delay(1); // for WDT
}
