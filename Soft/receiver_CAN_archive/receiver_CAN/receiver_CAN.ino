#include <CAN.h>

// CAN address
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

// rData
float pitch = 0;
float roll = 0;
float yaw = 0;
float ax = 0;
float ay = 0;
float az = 0;
float gx = 0;
float gy = 0;
float gz = 0;
float mx = 0;
float my = 0;
float mz = 0;
float PcbTemp = 0;

void CAN_SEND(byte CANaddr, int data, uint8_t sign, uint8_t exp);
void CAN_RECEIVE();
void chng(byte CANaddr, uint32_t data, byte exp);

void setup()
{
  Serial.begin(115200);

  CAN.setPins(33, 32);
  if (!CAN.begin(500E3))
  {
    Serial.println("Starting CAN failed!");
  }
}

void loop()
{
  CAN_RECEIVE();
}

void CAN_RECEIVE()
{

  if (CAN.parsePacket())
  {
    byte CANaddr = CAN.packetId(); // パケットのID（アドレス）を取得
    byte sign = CAN.read();        // 符号を取得
    byte exp = CAN.read();         // 指数を取得

    // Serial.println(CANaddr, HEX);
    // Serial.println(sign);
    // Serial.println(exp);

    uint32_t data = 0;
    for (int i = 0; i < 4; i++)
    {
      byte dataByte = CAN.read(); // データの各バイトを受信
      data = (data << 8) | dataByte;
      // Serial.println(dataByte);
    }

    if (sign == 1)
    {
      data = -data;
    }

    chng(CANaddr, data, exp);

    // Serial.println("Data received:");
    // Serial.println(PcbTemp, exp);
    // Serial.println("done");
  }
}

void chng(byte CANaddr, uint32_t data, byte exp)
{
  float fData = data * (pow(10, (-1 * exp)));
  switch (CANaddr)
  {
  case pitchCANaddr:
    pitch = fData;
    break;
  case rollCANaddr:
    roll = fData;
    break;
  case yawCANaddr:
    yaw = fData;
    break;
  case axCANaddr:
    ax = fData;
    break;
  case ayCANaddr:
    ay = fData;
    break;
  case azCANaddr:
    az = fData;
    break;
  case gxCANaddr:
    gx = fData;
    break;
  case gyCANaddr:
    gy = fData;
    break;
  case gzCANaddr:
    gz = fData;
    break;
  case mxCANaddr:
    mx = fData;
    break;
  case myCANaddr:
    my = fData;
    break;
  case mzCANaddr:
    mz = fData;
    break;
  case PcbTempCANaddr:
    PcbTemp = fData;
    break;
  default:
    break;
  }
}
