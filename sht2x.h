#ifndef SHT2_H
#define SHT2_H

#include <stdint.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <util/delay.h>
#include "twi_master_driver.h"


class SHT2
{
  public:
    #define CHECKSUM_ERROR 0x80
    typedef enum{
    TRIG_T_MEASUREMENT_HM    = 0xE3, // command trig. temp meas. hold master
    TRIG_RH_MEASUREMENT_HM   = 0xE5, // command trig. humidity meas. hold master
    TRIG_T_MEASUREMENT_POLL  = 0xF3, // command trig. temp meas. no hold master
    TRIG_RH_MEASUREMENT_POLL = 0xF5, // command trig. humidity meas. no hold master
    USER_REG_W               = 0xE6, // command writing user register
    USER_REG_R               = 0xE7, // command reading user register
    SOFT_RESET               = 0xFE  // command soft reset
    }etSHT2xCommand;

    typedef enum {
      SHT2x_RES_12_14BIT       = 0x00, // RH=12bit, T=14bit
      SHT2x_RES_8_12BIT        = 0x01, // RH= 8bit, T=12bit
      SHT2x_RES_10_13BIT       = 0x80, // RH=10bit, T=13bit
      SHT2x_RES_11_11BIT       = 0x81, // RH=11bit, T=11bit
      SHT2x_RES_MASK           = 0x81  // Mask for res. bits (7,0) in user reg.
    } etSHT2xResolution;

    typedef enum {
      SHT2x_EOB_ON             = 0x40, // end of battery
      SHT2x_EOB_MASK           = 0x40, // Mask for EOB bit(6) in user reg.
    } etSHT2xEob;

    typedef enum {
      SHT2x_HEATER_ON          = 0x04, // heater on
      SHT2x_HEATER_OFF         = 0x00, // heater off
      SHT2x_HEATER_MASK        = 0x04, // Mask for Heater bit(2) in user reg.
    } etSHT2xHeater;

    // measurement signal selection
    typedef enum{
      HUMIDITY,
      TEMP
    }etSHT2xMeasureType;

    typedef enum{
      I2C_ADR_W                = 128,   // sensor I2C address + write bit
      I2C_ADR_R                = 129    // sensor I2C address + read bit
    }etI2cHeader;


    SHT2(void);
    SHT2(TWI_MasterDriver_t *mytwi,uint8_t address);
    ~SHT2();
    uint8_t write(uint8_t *data,uint8_t num);
    uint8_t read(uint8_t* data, uint8_t num);
    uint8_t writeRead(uint8_t* writeData, uint8_t writeNum, uint8_t* readData, uint8_t readNum);
    uint8_t CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum);
    uint8_t ReadUserRegister(uint8_t *pRegisterValue);
    uint8_t WriteUserRegister(uint8_t pRegisterValue);
    uint8_t startMeasurementPoll(etSHT2xMeasureType eSHT2xMeasureType);
    uint8_t getMeasurementPoll(int16_t* pMeasurand);
    uint8_t SoftReset();
    float CalcRH(uint16_t u16sRH);
    float CalcTemperatureC(uint16_t u16sT);
    uint8_t getSerialNumber(uint8_t u8SerialNumber[]);

  protected:

  private:
    uint8_t twiAdress;
    TWI_MasterDriver_t *twi;
    const uint16_t POLYNOMIAL = 0x131;  //P(x)=x^8+x^5+x^4+1 = 100110001
    SHT2( const SHT2 &c );
    SHT2& operator=( const SHT2 &c );

};

#endif // SHT2_H
