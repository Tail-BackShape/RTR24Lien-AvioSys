#include <CAN.h>

union FloatToBytes {
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
    // Example float value to send
    float exampleValue = 23.45;

    // Convert float to bytes
    FloatToBytes converter;
    converter.floatValue = exampleValue;

    // Create a CAN message
    CAN.beginPacket(0x100); // Set the CAN ID
    CAN.write(converter.bytes, sizeof(converter.bytes)); // Write the byte array to the packet
    CAN.endPacket(); // Send the packet

    Serial.print("Sent float value: ");
    Serial.println(exampleValue);

    delay(1000); // Wait for a second before sending the next message
}
