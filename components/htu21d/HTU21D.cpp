#include "HTU21D.h"

void HTU21D::init()
{
    // Initialization data goes here
}

bool HTU21D::readTemperature(float * temperature)
{
    uint8_t writeData = TRIGGER_TEMP_MEASURE;
    uint8_t readData[2]; 

    esp_err_t errRc = ESP_OK;
   // Write to register TRIGGER_TEMP_MEASURE and
   // wait for 60ms before reading the temperature 
   // value
   if ( (errRc = m_i2c.write(m_address, &writeData, 1, true)) != ESP_OK)
       return false;

   vTaskDelay(60 / portTICK_RATE_MS );
   if (( errRc = m_i2c.read(m_address, &readData[0], 2, true) ) != ESP_OK)
       return false;

   // Process temperature value

   // Data[0] = high byte, Data[1] = low byte
   unsigned int rawTemperature = ((unsigned int) readData[0] << 8) | (unsigned int) readData[1];
   rawTemperature &= 0xFFFC;

   float temp = (float) rawTemperature / 65536.0; // 2^16 = 65536
   *temperature = -46.85 + (175.72 * temp); // From page 14 of HTU21D specs

   return true;
}

bool HTU21D::readHumidity(float * humidity)
{
    uint8_t writeData = TRIGGER_HUMD_MEASURE;
    uint8_t readData[2]; 

    esp_err_t errRc = ESP_OK;
    // Write to register TRIGGER_HUMD_MEASURE and
    // wait for 60ms before reading the temperature 
    // value
    if ( (errRc = m_i2c.write(m_address, &writeData, 1, true)) != ESP_OK)
        return false;

    vTaskDelay(60 / portTICK_RATE_MS );
    if (( errRc = m_i2c.read(m_address, &readData[0], 2, true) ) != ESP_OK)
        return false;

    // Process humidity value

    // Data[0] = high byte, Data[1] = low byte
    unsigned int rawHumidity = ((unsigned int) readData[0] << 8) | (unsigned int) readData[1];
    rawHumidity &= 0xFFFC;

    float temp = rawHumidity / (float) 65536;
    *humidity = -6 + (125 * temp); // From page 14.

    return true;

}

