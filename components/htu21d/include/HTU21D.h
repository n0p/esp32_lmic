#ifndef _HTU21D_H_
#define _HTU21D_H_

#include "I2CMaster.h"

#define HTU21D_I2C_ADDRESS  0x40
#define TRIGGER_TEMP_MEASURE  0xE3
#define TRIGGER_HUMD_MEASURE  0xE5

class HTU21D {

    public:
        HTU21D(I2CMaster & i2c, uint8_t address = HTU21D_I2C_ADDRESS)
            :m_i2c(i2c),
            m_address(address)
        {}
        ~HTU21D() {}

        void init();
        bool readTemperature(float * temperature);
        bool readHumidity(float * humidity);
    private:
        I2CMaster & m_i2c;
        uint8_t m_address;

};

#endif

