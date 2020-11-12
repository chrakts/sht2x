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
  sleep();
  _twi = twi;
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
  command(SHTC3_GETID);
  TWI_MasterRead(_twi,TWI_ADDRESS,3);
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
  data[0] = uint8_t(com>>8);
  data[1] = uint8_t(com&0xff);
  return( TWI_MasterWrite(_twi,TWI_ADDRESS,data,2) );
}

uint16_t shtc3::get16Value(uint8_t *data)
{
uint16_t result;
  result=(uint16_t)data[0];
  result<<=8;
  result |= data[1];
  return(result);
}

bool shtc3::getResults(double &temperature, double &humidity)
{
  TWI_MasterRead(_twi,TWI_ADDRESS,6);
  temperature = (double) get16Value(_twi->readData);
  humidity    = (double) get16Value( &(_twi->readData[3]) );
  temperature = temperature/374.491428571-45.0;
  humidity    = humidity/655.36;
}

bool shtc3::verifyCRC(uint8_t *data)
{
uint8_t crc;
  crc = calculateCrc8(0xff, data[0]);
  crc = calculateCrc8(crc, data[1]);
  return(data[2]==crc);
}

uint8_t shtc3::calculateCrc8(uint8_t crc8, uint8_t data)
{
  crc8 = crc8 ^ data;
  for (int i = 0; i < 8; i++)
  {
    if (crc8 & 1)
    {
      crc8 = (crc8 >> 1) ^ 0x8c;
    }
    else
    {
      crc8 = (crc8 >> 1);
    }
  }
  return crc8;
}
