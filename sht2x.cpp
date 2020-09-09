#include "sht2x.h"

SHT2::SHT2(void)
{
}

SHT2::SHT2(TWI_MasterDriver_t *mytwi,uint8_t address)
{
  twi = mytwi;
  twiAdress = address;
}

SHT2::~SHT2()
{
  //dtor
}

/** @brief (one liner)
  *
  * (documentation goes here)
  */
uint8_t SHT2::write(uint8_t* data, uint8_t num)
{
   TWI_MasterWrite(twi,twiAdress,data,num);
   while (twi->status != TWIM_STATUS_READY) {
		/* Wait until transaction is complete. */
	 }
	 return(twi->result);
}


/** @brief (one liner)
  *
  * (documentation goes here)
  */
uint8_t SHT2::read(uint8_t* data, uint8_t num)
{
uint8_t i;
  TWI_MasterRead(twi,twiAdress,num);
  while (twi->status != TWIM_STATUS_READY) {
	/* Wait until transaction is complete. */
	}
	for(i=0;i<num;i++)
    data[i] = twi->readData[i];
  return(twi->result);
}

uint8_t SHT2::writeRead(uint8_t* writeData, uint8_t writeNum, uint8_t* readData, uint8_t readNum)
{
uint8_t i;
  TWI_MasterWriteRead(twi,twiAdress,writeData,writeNum,readNum);
  while (twi->status != TWIM_STATUS_READY) {
  /* Wait until transaction is complete. */
  }
	for(i=0;i<readNum;i++)
    readData[i] = twi->readData[i];
  return(twi->result);
}
/*! \brief calculates checksum
 *
 *  calculates checksum for n bytes of data and compares
 *  it with expected checksum
 *
 *  \param data[] checksum is built based on this data
 *  \param nbrOfBytes   checksum is built for n bytes of data
 *  \param checksum     expected checksum
 *
 *  \retval CHECKSUM_ERROR  checksum does not match
 *  \retval 0 checksum matches
 */
uint8_t SHT2::CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum)
{
  uint8_t crc = 0;
  uint8_t byteCtr;
  //calculates 8-Bit checksum with given polynomial
  for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
  {
    crc ^= (data[byteCtr]);
    for (uint8_t bit = 8; bit > 0; --bit)
    {
      if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
      else crc = (crc << 1);
    }
  }
  if (crc != checksum) return CHECKSUM_ERROR;
  else return 0;
}

/** @brief (one liner)
  *
  * (documentation goes here)
  */
uint8_t SHT2::ReadUserRegister(uint8_t* pRegisterValue)
{
  uint8_t error=0;    //variable for error code
  uint8_t command;
  uint8_t answer[2];

  command = USER_REG_R;
  error = write(&command,1);
  read(answer,2);
  error += CheckCrc (&answer[0],1,answer[1]);
  return error;

/*  I2c_StartCondition();
  error |= I2c_WriteByte (I2C_ADR_W);
  error |= I2c_WriteByte (USER_REG_R);
  I2c_StartCondition();
  error |= I2c_WriteByte (I2C_ADR_R);
  *pRegisterValue = I2c_ReadByte(ACK);
  checksum=I2c_ReadByte(NO_ACK);
  error |= SHT2x_CheckCrc (pRegisterValue,1,checksum);
  I2c_StopCondition();
  return error;*/
}

/** @brief (one liner)
  *
  * (documentation goes here)
  */
uint8_t SHT2::WriteUserRegister(uint8_t pRegisterValue)
{
  uint8_t command[2];
  command[0] = USER_REG_W;
  command[1] = pRegisterValue;
  return write(command,2);

/*
  u8t error=0;   //variable for error code

  I2c_StartCondition();
  error |= I2c_WriteByte (I2C_ADR_W);
  error |= I2c_WriteByte (USER_REG_W);
  error |= I2c_WriteByte (*pRegisterValue);
  I2c_StopCondition();
  return error;
*/
}


/** @brief (one liner)
  *
  * (documentation goes here)
  */
uint8_t SHT2::getSerialNumber(uint8_t u8SerialNumber[])
{
  uint8_t commandData[2] = {0xFA,0x0F};
  uint8_t readData[8];
  uint8_t  error=0;                          //error variable

  error = writeRead(commandData,2,readData,8);
  u8SerialNumber[5] = readData[0];
  u8SerialNumber[4] = readData[2];
  u8SerialNumber[3] = readData[4];
  u8SerialNumber[2] = readData[6];
  commandData[0] = 0xFC;
  commandData[0] = 0xC9;
  error |= writeRead(commandData,2,readData,6);
  u8SerialNumber[1] = readData[0];
  u8SerialNumber[0] = readData[1];
  u8SerialNumber[7] = readData[3];
  u8SerialNumber[6] = readData[4];

  return error;

}

/** @brief (one liner)
  *
  * (documentation goes here)
  */
float SHT2::CalcTemperatureC(uint16_t u16sT)
{
  float temperatureC;            // variable for result

  //u16sT &= ~0x0003;           // clear bits [1..0] (status bits)
  u16sT >>= 2;
  //-- calculate temperature [°C] --
  temperatureC= -46.85 + 0.010725098 *(float)u16sT; //T= -46.85 + 175.72 * ST/2^16
  return temperatureC;
}

/** @brief (one liner)
  *
  * (documentation goes here)
  */
float SHT2::CalcRH(uint16_t u16sRH)
{
float humidityRH;              // variable for result

  u16sRH &= ~0x0003;          // clear bits [1..0] (status bits)
  //-- calculate relative humidity [%RH] --
  humidityRH = -6.0 + 125.0/65536 * (float)u16sRH; // RH= -6 + 125 * SRH/2^16
  return humidityRH;
}

/** @brief (one liner)
  *
  * (documentation goes here)
  */
uint8_t SHT2::SoftReset()
{
  uint8_t command,error;
  command = SOFT_RESET;
  error = write(&command,1);
  _delay_ms(15);
  return(error);

  /*
  u8t  error=0;           //error variable
  I2c_StartCondition();
  error |= I2c_WriteByte (I2C_ADR_W); // I2C Adr
  error |= I2c_WriteByte (SOFT_RESET);                            // Command
  I2c_StopCondition();

  DelayMicroSeconds(15000); // wait till sensor has restarted

  return error;
  */
}

/** @brief (one liner)
  *
  * (documentation goes here)
  */
uint8_t SHT2::MeasureHM(etSHT2xMeasureType eSHT2xMeasureType, int16_t* pMeasurand)
{

}

/** @brief (one liner)
  *
  * (documentation goes here)
  */
uint8_t SHT2::startMeasurementPoll(etSHT2xMeasureType eSHT2xMeasureType)
{
  uint8_t  data;    //data array for checksum verification
  uint8_t  error=0;    //error variable

  switch(eSHT2xMeasureType)
  { case  HUMIDITY: data = TRIG_RH_MEASUREMENT_POLL; break;
    case TEMP     : data = TRIG_T_MEASUREMENT_POLL; break;
    default: ;
  }
  error = write(&data,1);
  return(error);
}

/** @brief (one liner)
  *
  * (documentation goes here)
  */
uint8_t SHT2::getMeasurementPoll(int16_t* pMeasurand)
{
  uint8_t  data[3];    //data array for checksum verification
  uint8_t  error=0;    //error variable

  error = write(NULL,0);
  switch(error)
  {
    case TWIM_RESULT_NACK_RECEIVED:
      return(0);
    break;
    case TWIM_RESULT_OK:
      read(data,3);
      *pMeasurand = data[0];
      *pMeasurand = *pMeasurand<<8;
      *pMeasurand += data[1];
      error |= CheckCrc (data,2,data[2]);
      return(error);
    break;
    default:
      return(0);
    break;
  }
}

