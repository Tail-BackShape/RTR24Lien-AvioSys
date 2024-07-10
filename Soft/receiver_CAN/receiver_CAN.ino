#include <CAN.h>

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

  if (CAN.parsePacket())
  {
    byte CANaddr = CAN.packetId(); // パケットのID（アドレス）を取得
    byte sign = CAN.read();        // 符号を取得
    byte exp = CAN.read();         // 指数を取得

    Serial.println(CANaddr, HEX);
    Serial.println(sign);
    Serial.println(exp);

    uint32_t data = 0;
    for (int i = 0; i < 4; i++)
    {
      byte dataByte = CAN.read(); // データの各バイトを受信
      data = (data << 8) | dataByte;
      Serial.println(dataByte);
    }

    Serial.println("Data received:");
    Serial.println(data);
    Serial.println("done");
  }
  else
  {
    Serial.println("No packet received");
  }
}
