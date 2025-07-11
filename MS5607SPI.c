/*
   MS5607-02 SPI library for ARM STM32F103xx Microcontrollers - Main source file
   05/01/2020 by Joao Pedro Vilas <joaopedrovbs@gmail.com>
   Changelog:
     2012-05-23 - initial release.
*/
/* ============================================================================================
 MS5607-02 device SPI library code for ARM STM32F103xx is placed under the MIT license
Copyright (c) 2020 João Pedro Vilas Boas

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 ================================================================================================
 */

#include "MS5607SPI.h"

/* Private SPI Handler */
static SPI_HandleTypeDef *hspi;

/* Private GPIO CS Pin Variables */
static GPIO_TypeDef *CS_GPIO_Port;
static uint16_t CS_Pin;

/* SPI Transmission Data */
static uint8_t SPITransmitData;

/* Private OSR Instantiations */
static uint8_t Pressure_OSR =  OSR_256;
static uint8_t Temperature_OSR =  OSR_256;

/* Private data holders */

/* PROM data structure */
static struct promData promData;
/* Unconpensated values structure */
static struct MS5607UncompensatedValues uncompValues;
/* Compensated values structure */
static struct MS5607Readings readings;

/**
 * ================================================================================================
 * __weak functions for user to override if required
 * ================================================================================================
*/
__weak void MS5607EnableCSB(void);
__weak void MS5607DisableCSB(void);
__weak void MS5607Delay(uint32_t delay);
__weak HAL_StatusTypeDef MS5607SendCommand(uint8_t CMD);
__weak HAL_StatusTypeDef MS5607ReadData(uint8_t CMD, uint8_t *data, uint16_t bytesToRead, uint16_t timeout);


/*
 * @brief  Resets the Chip Select pin to select the MS5607 device
 * @param  None
 * @retval None
 */
__weak void MS5607EnableCSB(void)
{
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief  Sets the Chip Select pin to deselect the MS5607 device
 * @param  None
 * @retval None
 */
__weak void MS5607DisableCSB(void)
{
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief  Delays the execution for a given amount of milliseconds
 * @param  delay: Amount of milliseconds to delay
 * @retval None
 */
__weak void MS5607Delay(uint32_t delay)
{
  HAL_Delay(delay);
}

/**
 * @brief Sends command to the MS5607 device
 * @param uint8_t CMD: Command to send to the device
 * @return HAL_StatusTypeDef: Status of the transmission
 */
__weak HAL_StatusTypeDef MS5607SendCommand(uint8_t CMD)
{
  HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, &CMD, 1, 10);
  return status;
}

/**
 * @brief Reads data from the MS5607 device
 * @param uint8_t CMD: Command to read data from the device
 * @param uint8_t *data: Pointer to the data buffer to store the read data
 * @param uint16_t bytesToRead: Number of bytes to read from the device
 * @param uint16_t timeout: Timeout for the read operation
 * @return HAL_StatusTypeDef: Status of the read operation
 */
__weak HAL_StatusTypeDef MS5607ReadData(uint8_t CMD, uint8_t *data, uint16_t bytesToRead, uint16_t timeout) {
  HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, &CMD, 1, 10);
  if (status == HAL_OK) {
	status = HAL_SPI_Receive(hspi, data, bytesToRead, 10);
  }
  return status;
}


/**
 * ================================================================================================
 * Private functions do not modify this section
 * ================================================================================================
*/

/**
 * Reset and prepare for general usage.
 * This will reset the device and perform the PROM reading to find the conversion values and if
 * the communication is working.
*/
MS5607StateTypeDef MS5607_Init(SPI_HandleTypeDef *hspix, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
  hspi = hspix;
  CS_GPIO_Port = GPIOx;
  CS_Pin = GPIO_Pin;

  SPITransmitData = RESET_COMMAND;

  MS5607EnableCSB();
  MS5607SendCommand(SPITransmitData);
  MS5607Delay(3); // Wait for 3ms as per datasheet
  MS5607DisableCSB();

  MS5607PromRead(&promData);

  if (promData.reserved == 0x00 || promData.reserved == 0xff)
    return MS5607_STATE_FAILED;
  else
    return MS5607_STATE_READY;
}

/* Performs a reading on the devices PROM. */
void MS5607PromRead(struct promData *prom){
  uint8_t   address;
  uint16_t  *structPointer;

  /* As the PROM is made of 8 16bit addresses I used a pointer for acessing the data structure */
  structPointer = (uint16_t *) prom;

  for (address = 0; address < 8; address++) {
    SPITransmitData = PROM_READ(address);
    MS5607EnableCSB();
    MS5607ReadData(SPITransmitData, (uint8_t *) structPointer, 2, 10);
    MS5607DisableCSB();
    structPointer++;
  }

  /* Byte swap on 16bit integers*/
  structPointer = (uint16_t *) prom;
  for (address = 0; address < 8; address++) {
    uint8_t   *toSwap = (uint8_t *) structPointer;
    uint8_t secondByte = toSwap[0];
    toSwap[0] = toSwap[1];
    toSwap[1] = secondByte;
    structPointer++;
  }
}

/* Performs a reading on the devices PROM. */
void MS5607UncompensatedRead(struct MS5607UncompensatedValues *uncompValues){

  /*Sensor reply data buffer*/
  uint8_t reply[3];

  MS5607EnableCSB();
  /* Assemble the conversion command based on previously set OSR */
  SPITransmitData = CONVERT_D1_COMMAND | Pressure_OSR;
  MS5607SendCommand(SPITransmitData);

  if(Pressure_OSR == 0x00)
	MS5607Delay(1);
  else if(Pressure_OSR == 0x02)
	MS5607Delay(2);
  else if(Pressure_OSR == 0x04)
	MS5607Delay(3);
  else if(Pressure_OSR == 0x06)
    MS5607Delay(5);
  else
    MS5607Delay(10);

  MS5607DisableCSB();

  /* Performs the reading of the 24 bits from the ADC */

  SPITransmitData = READ_ADC_COMMAND;
  MS5607EnableCSB();
  MS5607ReadData(READ_ADC_COMMAND, reply, 3, 10);
  MS5607DisableCSB();

  /* Tranfer the 24bits read into a 32bit int */
  uncompValues->pressure = ((uint32_t) reply[0] << 16) | ((uint32_t) reply[1] << 8) | (uint32_t) reply[2];

  MS5607EnableCSB();

  /* Assemble the conversion command based on previously set OSR */
  SPITransmitData = CONVERT_D2_COMMAND | Temperature_OSR;
  MS5607SendCommand(SPITransmitData);

  if(Temperature_OSR == 0x00)
    MS5607Delay(1);
  else if(Temperature_OSR == 0x02)
    MS5607Delay(2);
  else if(Temperature_OSR == 0x04)
    MS5607Delay(3);
  else if(Temperature_OSR == 0x06)
    MS5607Delay(5);
  else
    MS5607Delay(10);

  MS5607DisableCSB();


  MS5607EnableCSB();

  SPITransmitData = READ_ADC_COMMAND;
  MS5607ReadData(READ_ADC_COMMAND, reply, 3, 10);

  MS5607DisableCSB();

  /* Assemble the conversion command based on previously set OSR */
  uncompValues->temperature = ((uint32_t) reply[0] << 16) | ((uint32_t) reply[1] << 8) | (uint32_t) reply[2];
}

/* Performs the data conversion according to the MS5607 datasheet */
void MS5607Convert(struct MS5607UncompensatedValues *sample, struct MS5607Readings *value){
  int32_t dT;
  int32_t TEMP;
  int64_t OFF;
  int64_t SENS;

  dT = sample->temperature - ((int32_t) (promData.tref << 8));

  TEMP = 2000 + (((int64_t) dT * promData.tempsens) >> 23);

  OFF = ((int64_t) promData.off << 17) + (((int64_t) promData.tco * dT) >> 6);
  SENS = ((int64_t) promData.sens << 16) + (((int64_t) promData.tcs * dT) >> 7);

  /**/
  if (TEMP < 2000) {
    int32_t T2 = ((int64_t) dT * (int64_t) dT) >> 31;
    int32_t TEMPM = TEMP - 2000;
    int64_t OFF2 = (61 * (int64_t) TEMPM * (int64_t) TEMPM) >> 4;
    int64_t SENS2 = 2 * (int64_t) TEMPM * (int64_t) TEMPM;
    if (TEMP < -1500) {
      int32_t TEMPP = TEMP + 1500;
      int32_t TEMPP2 = TEMPP * TEMPP;
      OFF2 = OFF2 + (int64_t) 15 * TEMPP2;
      SENS2 = SENS2 + (int64_t) 8 * TEMPP2;
    }
    TEMP -= T2;
    OFF -= OFF2;
    SENS -= SENS2;
  }

  value->pressure = ((((int64_t) sample->pressure * SENS) >> 21) - OFF) >> 15;
  value->temperature = TEMP;
}

/* Performs the sensor reading updating the data structures */
void MS5607Update(void){
  MS5607UncompensatedRead(&uncompValues);
  MS5607Convert(&uncompValues, &readings);
}

/* Gets the temperature from the sensor reading */
double MS5607GetTemperatureC(void){
  return (double)readings.temperature/(double)100.0;
}

/* Gets the pressure from the sensor reading */
int32_t MS5607GetPressurePa(void){
  return readings.pressure;
}

/* Sets the OSR for temperature */
void MS5607SetTemperatureOSR(MS5607OSRFactors tOSR){
  Temperature_OSR = tOSR;
}

/* Sets the OSR for pressure */
void MS5607SetPressureOSR(MS5607OSRFactors pOSR){
  Pressure_OSR = pOSR;
}
