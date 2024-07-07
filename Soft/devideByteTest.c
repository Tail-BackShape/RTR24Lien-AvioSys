#include <stdio.h>
#include <inttypes.h>

void divideBytes(byte addr, uint32_t data)
{
    printf("addr: %d\n", addr);
    printf("data: %d\n", data);

    // devide data to 4byte
    for (int i = 0; i < 4; i++)
    {
        byte dataByte = (byte)(data >> (i * 8));
        printf("dataByte: %d\n", dataByte);
    }

    return 0;
}

int main()
{
    uint32_t data = scanf("%d", &data);
    byte addr = scanf("%d", &addr);
    divideBytes(addr, data);
    return 0;
}
