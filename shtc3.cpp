#include "shtc3.h"

shtc3::shtc3()
{
  //ctor
}

shtc3::~shtc3()
{
  //dtor
}

void shtc3::begin(TWI_MasterDriver_t *twi)
{
  _twi = twi;
  //sleep();
}

void shtc3::setMode(uint16_t mode)
{
  _mode = mode;
}

void shtc3::wakeup()
{
  command(SHTC3_WAKEUP);
}

void shtc3::sleep()
{
  command(SHTC3_SLEEP);
}

void shtc3::reset()
{
  command(SHTC3_RESET);
}

bool shtc3::startMeasure()
{
  return( command(_mode) );
}

uint16_t shtc3::getID()
{
uint8_t data[2];
  data[0] = SHTC3_GETID>>8;
  data[1] = SHTC3_GETID&0xff;
  TWI_MasterWriteRead(_twi,TWI_ADDRESS,data,2,3);
	while (_twi->status != TWIM_STATUS_READY)
  {

	}
  if( verifyCRC(_twi->readData) )
  {
    return( get16Value(_twi->readData) );
  }
  else
    return(0);
}

bool shtc3::command(uint16_t com)
{
uint8_t data[2];
	while (_twi->status != TWIM_STATUS_READY)
  {

	}
  data[0] = uint8_t(com>>8);
  data[1] = uint8_t(com&0xff);
  return( TWI_MasterWrite(_twi,TWI_ADDRESS,data,2) );
}

uint16_t shtc3::get16Value(uint8_t *data)
{
uint16_t result;
  result=(uint16_t)data[0];
  result<<=8;
  result |= (uint16_t)data[1];
  return(result);
}

bool shtc3::readResults()
{
   return( TWI_MasterRead(_twi,TWI_ADDRESS,6) );
}

bool shtc3::getResults(volatile double &temperature, volatile double &humidity)
{
	while (_twi->status != TWIM_STATUS_READY)
  {

	}
  temperature = (double) get16Value(_twi->readData);
  humidity    = (double) get16Value( &(_twi->readData[3]) );
  temperature = temperature/374.491428571-45.0;
  humidity    = humidity/655.36;
}

bool shtc3::verifyCRC(uint8_t *data)
{
uint8_t crc;
  crc = calculateCrc8(0xff, data,2);
  return(data[2]==crc);
}

uint8_t shtc3::calculateCrc8(uint8_t crc, const void *data, uint8_t data_len)
{
    const unsigned char *d = (const unsigned char *)data;
    unsigned int i;
    bool bit;
    unsigned char c;

    while (data_len--) {
        c = *d++;
        for (i = 0x80; i > 0; i >>= 1) {
            bit = crc & 0x80;
            if (c & i) {
                bit = !bit;
            }
            crc <<= 1;
            if (bit) {
                crc ^= 0x31;
            }
        }
        crc &= 0xff;
    }
    return crc & 0xff;
}

double shtc3::calcDewPoint(double t,double h)
{
double k,dew_point ;
	k = (log10(h)-2)/0.4343 + (17.62*t)/(243.12+t);
	dew_point = 243.12*k/(17.62-k);
	return dew_point;
}

double shtc3::calcAbsHumi(double t,double h)
{
const double a = 7.5, b = 237.3 ;
const double R = 8314.3, mw = 18.016;

double DD;
	DD = h/100 * 6.1078 * pow(10,(a*t)/(b+t));
	return(10e5 * mw/R * DD/(t+273.15));
}
