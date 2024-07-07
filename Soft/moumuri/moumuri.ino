#include <CAN.h>

union BytesToFloat {
    float floatValue;
    uint8_t bytes[4];
};

void setup() {
    Serial.begin(115200);

    // Initialize CAN at 500 kbps
    CAN.setPins(33, 32);
    if (!CAN.begin(500E3)) {
        Serial.println("Starting CAN failed!");
        while (1);
    }
    Serial.println("CAN init ok");
}

void loop() {
    // Check if a CAN message is available
    int packetSize = CAN.parsePacket();
    if (packetSize) {
        if (CAN.packetId() == 0x100) { // Check if the message ID is 0x100
            BytesToFloat converter;
            int i = 0;
            while (CAN.available() && i < 4) {
                converter.bytes[i++] = CAN.read();
            }

            if (i == 4) { // Ensure that 4 bytes were read
                Serial.print("Received float value: ");
                Serial.println(converter.floatValue);
            }
        }
    }
}
