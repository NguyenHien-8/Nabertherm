/*
 * @file MAX31856.c
 * Author: TRAN NGUYEN HIEN
 * Email: trannguyenhien29085@gmail.com
 */

#include "MAX31856.h"
#include <math.h>

/* ==================================================================== */
/* PRIVATE LOW-LEVEL READ/WRITE FUNCTIONS                               */
/* ==================================================================== */

static void MAX31856_WriteRegister8(MAX31856_HandleTypeDef *dev, uint8_t addr, uint8_t data) {
  addr |= 0x80; // MSB=1 for write, make sure top bit is set
  uint8_t buffer[2] = {addr, data};

  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(dev->hspi, buffer, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

static void MAX31856_ReadRegisterN(MAX31856_HandleTypeDef *dev, uint8_t addr, uint8_t *buffer, uint8_t n) {
  addr &= 0x7F; // MSB=0 for read, make sure top bit is not set

  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(dev->hspi, &addr, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(dev->hspi, buffer, n, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

static uint8_t MAX31856_ReadRegister8(MAX31856_HandleTypeDef *dev, uint8_t addr) {
  uint8_t ret = 0;
  MAX31856_ReadRegisterN(dev, addr, &ret, 1);
  return ret;
}

static uint16_t MAX31856_ReadRegister16(MAX31856_HandleTypeDef *dev, uint8_t addr) {
  uint8_t buffer[2] = {0, 0};
  MAX31856_ReadRegisterN(dev, addr, buffer, 2);

  uint16_t ret = buffer[0];
  ret <<= 8;
  ret |= buffer[1];

  return ret;
}

static uint32_t MAX31856_ReadRegister24(MAX31856_HandleTypeDef *dev, uint8_t addr) {
  uint8_t buffer[3] = {0, 0, 0};
  MAX31856_ReadRegisterN(dev, addr, buffer, 3);

  uint32_t ret = buffer[0];
  ret <<= 8;
  ret |= buffer[1];
  ret <<= 8;
  ret |= buffer[2];

  return ret;
}

/* ==================================================================== */
/*  MAIN API FUNCTIONS                                                  */
/* ==================================================================== */

bool MAX31856_Init(MAX31856_HandleTypeDef *dev, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin) {
  if (dev == NULL || hspi == NULL || cs_port == NULL) return false;

  dev->hspi = hspi;
  dev->cs_port = cs_port;
  dev->cs_pin = cs_pin;
  dev->initialized = true;

  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

  // assert on any fault
  MAX31856_WriteRegister8(dev, MAX31856_MASK_REG, 0x0);

  // enable open circuit fault detection
  MAX31856_WriteRegister8(dev, MAX31856_CR0_REG, MAX31856_CR0_OCFAULT0);

  // set cold junction temperature offset to zero
  MAX31856_WriteRegister8(dev, MAX31856_CJTO_REG, 0x0);

  // set Type K by default
  MAX31856_SetThermocoupleType(dev, MAX31856_TCTYPE_K);

  // set One-Shot conversion mode
  MAX31856_SetConversionMode(dev, MAX31856_ONESHOT);

  return true;
}

/**************************************************************************/
/*!
    @brief  Set temperature conversion mode
    @param mode The conversion mode
*/
/**************************************************************************/
void MAX31856_SetConversionMode(MAX31856_HandleTypeDef *dev, max31856_conversion_mode_t mode) {
  dev->conversionMode = mode;
  uint8_t t = MAX31856_ReadRegister8(dev, MAX31856_CR0_REG); // get current register

  if (dev->conversionMode == MAX31856_CONTINUOUS) {
    t |= MAX31856_CR0_AUTOCONVERT;  // turn on automatic
    t &= ~MAX31856_CR0_1SHOT;       // turn off one-shot
  } else {
    t &= ~MAX31856_CR0_AUTOCONVERT; // turn off automatic
    t |= MAX31856_CR0_1SHOT;        // turn on one-shot
  }

  MAX31856_WriteRegister8(dev, MAX31856_CR0_REG, t); // write value back to register
}

/**************************************************************************/
/*!
    @brief  Get temperature conversion mode
    @returns The conversion mode
*/
/**************************************************************************/
max31856_conversion_mode_t MAX31856_GetConversionMode(MAX31856_HandleTypeDef *dev) {
  return dev->conversionMode;
}

/**************************************************************************/
/*!
    @brief  Set which kind of Thermocouple (K, J, T, etc) to detect & decode
    @param type The enumeration type of the thermocouple
*/
/**************************************************************************/
void MAX31856_SetThermocoupleType(MAX31856_HandleTypeDef *dev, max31856_thermocoupletype_t type) {
  uint8_t t = MAX31856_ReadRegister8(dev, MAX31856_CR1_REG);
  t &= 0xF0; // mask off bottom 4 bits
  t |= (uint8_t)type & 0x0F;
  MAX31856_WriteRegister8(dev, MAX31856_CR1_REG, t);
}

max31856_thermocoupletype_t MAX31856_GetThermocoupleType(MAX31856_HandleTypeDef *dev) {
  uint8_t t = MAX31856_ReadRegister8(dev, MAX31856_CR1_REG);
  t &= 0x0F;
  return (max31856_thermocoupletype_t)(t);
}

/**************************************************************************/
/*!
    @brief  Read the fault register (8 bits)
    @returns 8 bits of fault register data
*/
/**************************************************************************/
uint8_t MAX31856_ReadFault(MAX31856_HandleTypeDef *dev) {
  return MAX31856_ReadRegister8(dev, MAX31856_SR_REG);
}

/**************************************************************************/
/*!
    @brief  Sets the threshhold for internal chip temperature range
    for fault detection. NOT the thermocouple temperature range!
    @param  low Low (min) temperature, signed 8 bit so -128 to 127 degrees C
    @param  high High (max) temperature, signed 8 bit so -128 to 127 degrees C
*/
/**************************************************************************/
void MAX31856_SetColdJunctionFaultThreshholds(MAX31856_HandleTypeDef *dev, int8_t low, int8_t high) {
  MAX31856_WriteRegister8(dev, MAX31856_CJLF_REG, low);
  MAX31856_WriteRegister8(dev, MAX31856_CJHF_REG, high);
}

/**************************************************************************/
/*!
    @brief  Sets the mains noise filter. Can be set to 50 or 60hz.
    Defaults to 60hz. You need to call this if you live in a 50hz country.
    @param  noiseFilter One of MAX31856_NOISE_FILTER_50HZ or
   MAX31856_NOISE_FILTER_60HZ
*/
/**************************************************************************/
void MAX31856_SetNoiseFilter(MAX31856_HandleTypeDef *dev, max31856_noise_filter_t noiseFilter) {
  uint8_t t = MAX31856_ReadRegister8(dev, MAX31856_CR0_REG);
  if (noiseFilter == MAX31856_NOISE_FILTER_50HZ) {
    t |= 0x01;
  } else {
    t &= 0xFE;
  }
  MAX31856_WriteRegister8(dev, MAX31856_CR0_REG, t);
}

/**************************************************************************/
/*!
    @brief  Sets the threshhold for thermocouple temperature range
    for fault detection. NOT the internal chip temperature range!
    @param  flow Low (min) temperature, floating point
    @param  fhigh High (max) temperature, floating point
*/
/**************************************************************************/
void MAX31856_SetTempFaultThreshholds(MAX31856_HandleTypeDef *dev, float flow, float fhigh) {
  int16_t low, high;

  flow *= 16;
  low = (int16_t)flow;

  fhigh *= 16;
  high = (int16_t)fhigh;

  MAX31856_WriteRegister8(dev, MAX31856_LTHFTH_REG, high >> 8);
  MAX31856_WriteRegister8(dev, MAX31856_LTHFTL_REG, high);

  MAX31856_WriteRegister8(dev, MAX31856_LTLFTH_REG, low >> 8);
  MAX31856_WriteRegister8(dev, MAX31856_LTLFTL_REG, low);
}

/**************************************************************************/
/*!
    @brief  Begin a one-shot (read temperature only upon request) measurement.
    Value must be read later, not returned here!
*/
/**************************************************************************/
void MAX31856_TriggerOneShot(MAX31856_HandleTypeDef *dev) {
  if (dev->conversionMode == MAX31856_CONTINUOUS)
    return;

  uint8_t t = MAX31856_ReadRegister8(dev, MAX31856_CR0_REG); // get current register
  t &= ~MAX31856_CR0_AUTOCONVERT;                            // turn off autoconvert
  t |= MAX31856_CR0_1SHOT;                                   // turn on one-shot
  MAX31856_WriteRegister8(dev, MAX31856_CR0_REG, t);         // write value back to register
}

/**************************************************************************/
/*!
    @brief  Return status of temperature conversion.
    @returns true if conversion complete, otherwise false
*/
/**************************************************************************/
bool MAX31856_ConversionComplete(MAX31856_HandleTypeDef *dev) {
  if (dev->conversionMode == MAX31856_CONTINUOUS)
    return true;
  return !(MAX31856_ReadRegister8(dev, MAX31856_CR0_REG) & MAX31856_CR0_1SHOT);
}

/**************************************************************************/
/*!
    @brief  Return cold-junction (internal chip) temperature
    @returns Floating point temperature in Celsius
*/
/**************************************************************************/
float MAX31856_ReadCJTemperature(MAX31856_HandleTypeDef *dev) {
  return MAX31856_ReadRegister16(dev, MAX31856_CJTH_REG) / 256.0f;
}

/**************************************************************************/
/*!
    @brief  Return hot-junction (thermocouple) temperature
    @returns Floating point temperature in Celsius
*/
/**************************************************************************/
float MAX31856_ReadThermocoupleTemperature(MAX31856_HandleTypeDef *dev) {
  if (dev->conversionMode == MAX31856_ONESHOT) {
    MAX31856_TriggerOneShot(dev);

    uint32_t start = HAL_GetTick();
    while (!MAX31856_ConversionComplete(dev)) {
      if (HAL_GetTick() - start > 250) {
        return NAN; // Timeout
      }
      HAL_Delay(10);
    }
  }
  int32_t temp24 = MAX31856_ReadRegister24(dev, MAX31856_LTCBH_REG);

  if (temp24 & 0x800000) {
    temp24 |= 0xFF000000;
  }

  temp24 >>= 5;
  return temp24 * 0.0078125f;
}
