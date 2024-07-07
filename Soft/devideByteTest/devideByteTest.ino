#include <CAN.h>

void setup()
{
  CAN.setPins(33, 32);
  // start the CAN bus at 500 kbps
  if (!CAN.begin(500E3))
  {
    Serial.println("Starting CAN failed!");
    while (1)
      ;
  }
  Serial.begin(115200);
}

void loop()
{
  // put your main code here, to run repeatedly:
  CAN_SEND(0x100, 3.14159);
  delayMicroseconds(1000000);
}

void CAN_SEND(byte CANaddr, uint32_t data)
{
  // SEND 32bit data
  // devide data to 4byte
  CAN.beginPacket(CANaddr);

  for (int i = 3; i >= 0; i--)
  {
    byte dataByte = (byte)(data >> (8 * i));
    Serial.print(dataByte, HEX);
    CAN.write(dataByte);
  }

  CAN.endPacket();
  Serial.println("done");
}
