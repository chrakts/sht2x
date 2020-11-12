#ifndef SHTC3_H
#define SHTC3_H

#include "twi_master_driver.h"

#define TWI_ADDRESS                 0x70
#define SHTC3_SLEEP                 0xB098
#define SHTC3_WAKEUP                0x3517
#define SHTC3_RESET                 0x805D
#define SHTC3_GETID                 0xEFC8
#define SHTC3_NORMAL_CLST_T_FIRST   0x7CA2
#define SHTC3_NORMAL_CLST_RH_FIRST  0x5C24
#define SHTC3_NORMAL_T_FIRST        0x7866
#define SHTC3_NORMAL_RH_FIRST       0x58E0
#define SHTC3_LOPO_CLST_T_FIRST     0x6458
#define SHTC3_LOPO_CLST_RH_FIRST    0x44DE
#define SHTC3_LOPO_T_FIRST          0x609C
#define SHTC3_LOPO_RH_FIRST         0x401A

class shtc3
{
  public:
    shtc3();
    virtual ~shtc3();
    void begin(TWI_MasterDriver_t *twi);
    void setMode(uint16_t mode);
    void wakeup();
    void sleep();
    bool startMeasure();
    void reset();
    bool command(uint16_t com);
    uint16_t getID();
    bool getResults(double &temperature, double &humidity);
    bool verifyCRC(uint8_t *data);
    uint8_t calculateCrc8(uint8_t crc8, uint8_t data);
    uint16_t get16Value(uint8_t *data);
  protected:

  private:
    TWI_MasterDriver_t *_twi;
    uint16_t _mode;
};

#endif // SHTC3_H
