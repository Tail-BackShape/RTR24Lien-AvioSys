#include <CAN.h>

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
const int AltCANaddr = 0x11;
const int Pitot1CANaddr = 0x12;
const int Pitot2CANaddr = 0x13;
const int Pitot3CANaddr = 0x14;

// MAIN
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

// Node(THIS)
float SpacePres = 0.00;
float SpaceTemp = 0.00;
float Alt = 0.00;
float Pitot1 = 0.00;
float Pitot2 = 0.00;
float Pitot3 = 0.00;

void CAN_SEND(byte CANaddr, int data, uint8_t sign, uint8_t exp);
void CAN_RECEIVE();
void chng(byte CANaddr, int32_t data, byte exp);

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

void chng(byte CANaddr, int32_t data, byte exp)
{
  float fData = (float)(data) * (pow(10, (-1 * exp)));
  switch (CANaddr)
  {
  case pitchCANaddr:
    pitch = fData;
    Serial.print("pitch: ");
    Serial.println(pitch);
    break;
  case rollCANaddr:
    roll = fData;
    Serial.print("roll: ");
    Serial.println(roll);
    break;
  case yawCANaddr:
    yaw = fData;
    Serial.print("yaw: ");
    Serial.println(yaw);
    break;
  case axCANaddr:
    ax = fData;
    Serial.print("ax: ");
    Serial.println(ax);
    break;
  case ayCANaddr:
    ay = fData;
    Serial.print("ay: ");
    Serial.println(ay);
    break;
  case azCANaddr:
    az = fData;
    Serial.print("az: ");
    Serial.println(az);
    break;
  case gxCANaddr:
    gx = fData;
    Serial.print("gx: ");
    Serial.println(gx);
    break;
  case gyCANaddr:
    gy = fData;
    Serial.print("gy: ");
    Serial.println(gy);
    break;
  case gzCANaddr:
    gz = fData;
    Serial.print("gz: ");
    Serial.println(gz);
    break;
  case mxCANaddr:
    mx = fData;
    Serial.print("mx: ");
    Serial.println(mx);
    break;
  case myCANaddr:
    my = fData;
    Serial.print("my: ");
    Serial.println(my);
    break;
  case mzCANaddr:
    mz = fData;
    Serial.print("mz: ");
    Serial.println(mz);
    break;
  case PcbTempCANaddr:
    PcbTemp = fData;
    Serial.print("PcbTemp: ");
    Serial.println(PcbTemp);
    break;
  case SpacePresCANaddr:
    SpacePres = fData;
    Serial.print("SpacePres: ");
    Serial.println(SpacePres);
    break;
  case SpaceTempCANaddr:
    SpaceTemp = fData;
    Serial.print("SpaceTemp: ");
    Serial.println(SpaceTemp);
    break;
  case AltCANaddr:
    Alt = fData;
    Serial.print("Alt: ");
    Serial.println(Alt);
    break;
  case Pitot1CANaddr:
    Pitot1 = fData;
    Serial.print("Pitot1: ");
    Serial.println(Pitot1);
    break;
  case Pitot2CANaddr:
    Pitot2 = fData;
    Serial.print("Pitot2: ");
    Serial.println(Pitot2);
    break;
  case Pitot3CANaddr:
    Pitot3 = fData;
    Serial.print("Pitot3: ");
    Serial.println(Pitot3);
    break;
  default:
    break;
  }
}
