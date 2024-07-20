#include <freertos/FreeRTOS.h>
#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads; // Use this for the 16-bit version

float Pitot1 = 0.00;
float Pitot2 = 0.00;
float Pitot3 = 0.00;

void setup()
{
  Serial.begin(115200);

  ads.setGain(GAIN_ONE); // 1x gain +/- 4.096V 1 bit = 2mV 0.125mV
  ads.begin();

  xTaskCreateUniversal(Get_PitotData,   // function
                       "Get_PitotData", // function name
                       8192,            // stack size
                       NULL,            // piont
                       2,               // priority
                       NULL,            // task handle
                       0);              // core number
}
void loop()
{
  delay(2);
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

    Serial.print(Pitot1, 6);
    Serial.print(",");
    Serial.print(Pitot2, 6);
    Serial.print(",");
    Serial.print(Pitot3, 6);
    Serial.println();
    vTaskDelay(200);
  }
}
