/*
 * @file MAX31856.c
 * Author: TRAN NGUYEN HIEN
 * Reliability update:
 * - No HAL_MAX_DELAY
 * - Every SPI transaction has a finite timeout
 * - Each complete transaction may retry once after a transient failure
 * - CS is always released even on timeout/error
 * - Communication failures are latched and visible to the application
 */

#include "MAX31856.h"
#include <math.h>
#include <stddef.h>

/* ==================================================================== */
/* PRIVATE LOW-LEVEL READ/WRITE FUNCTIONS                               */
/* ==================================================================== */

static void MAX31856_RecordCommunicationError(MAX31856_HandleTypeDef *dev,
                                              HAL_StatusTypeDef status) {
  if (dev == NULL) return;

  dev->communication_error = true;
  dev->last_hal_status = status;
  if (dev->communication_error_count < UINT32_MAX) {
    dev->communication_error_count++;
  }
}

static bool MAX31856_WriteRegister8(MAX31856_HandleTypeDef *dev,
                                    uint8_t addr,
                                    uint8_t data) {
  if (dev == NULL || dev->hspi == NULL || dev->cs_port == NULL) return false;

  uint8_t buffer[2] = {(uint8_t)(addr | 0x80U), data};
  HAL_StatusTypeDef status = HAL_ERROR;

  for (uint32_t attempt = 0U; attempt < MAX31856_SPI_MAX_ATTEMPTS; attempt++) {
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);

    status = HAL_SPI_Transmit(dev->hspi, buffer, 2U, MAX31856_SPI_TIMEOUT_MS);

    /* CS must be released on every path, including HAL_TIMEOUT/HAL_ERROR. */
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

    if (status == HAL_OK) {
      dev->last_hal_status = HAL_OK;
      return true;
    }

    /* Let the peripheral/bus settle before one bounded retry. */
    if ((attempt + 1U) < MAX31856_SPI_MAX_ATTEMPTS) {
      HAL_Delay(1U);
    }
  }

  MAX31856_RecordCommunicationError(dev, status);
  return false;
}

static bool MAX31856_ReadRegisterN(MAX31856_HandleTypeDef *dev,
                                   uint8_t addr,
                                   uint8_t *buffer,
                                   uint8_t n) {
  if (dev == NULL || dev->hspi == NULL || dev->cs_port == NULL ||
      buffer == NULL || n == 0U) {
    return false;
  }

  addr &= 0x7FU;
  HAL_StatusTypeDef status = HAL_ERROR;

  for (uint32_t attempt = 0U; attempt < MAX31856_SPI_MAX_ATTEMPTS; attempt++) {
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);

    status = HAL_SPI_Transmit(dev->hspi, &addr, 1U, MAX31856_SPI_TIMEOUT_MS);
    if (status == HAL_OK) {
      status = HAL_SPI_Receive(dev->hspi, buffer, n, MAX31856_SPI_TIMEOUT_MS);
    }

    /* Always deassert CS so a failed transaction cannot leave MAX31856 selected. */
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

    if (status == HAL_OK) {
      dev->last_hal_status = HAL_OK;
      return true;
    }

    if ((attempt + 1U) < MAX31856_SPI_MAX_ATTEMPTS) {
      HAL_Delay(1U);
    }
  }

  MAX31856_RecordCommunicationError(dev, status);
  return false;
}

static bool MAX31856_ReadRegister8(MAX31856_HandleTypeDef *dev,
                                   uint8_t addr,
                                   uint8_t *value) {
  if (value == NULL) return false;
  *value = 0U;
  return MAX31856_ReadRegisterN(dev, addr, value, 1U);
}

static bool MAX31856_ReadRegister16(MAX31856_HandleTypeDef *dev,
                                    uint8_t addr,
                                    uint16_t *value) {
  if (value == NULL) return false;

  uint8_t buffer[2] = {0U, 0U};
  *value = 0U;

  if (!MAX31856_ReadRegisterN(dev, addr, buffer, 2U)) return false;

  *value = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
  return true;
}

static bool MAX31856_ReadRegister24(MAX31856_HandleTypeDef *dev,
                                    uint8_t addr,
                                    uint32_t *value) {
  if (value == NULL) return false;

  uint8_t buffer[3] = {0U, 0U, 0U};
  *value = 0U;

  if (!MAX31856_ReadRegisterN(dev, addr, buffer, 3U)) return false;

  *value = ((uint32_t)buffer[0] << 16) |
           ((uint32_t)buffer[1] << 8) |
           (uint32_t)buffer[2];
  return true;
}

/* ==================================================================== */
/* COMMUNICATION HEALTH API                                             */
/* ==================================================================== */

void MAX31856_ClearCommunicationError(MAX31856_HandleTypeDef *dev) {
  if (dev == NULL) return;
  dev->communication_error = false;
  dev->last_hal_status = HAL_OK;
}

bool MAX31856_HasCommunicationError(const MAX31856_HandleTypeDef *dev) {
  return (dev == NULL) ? true : dev->communication_error;
}

HAL_StatusTypeDef MAX31856_GetLastHALStatus(const MAX31856_HandleTypeDef *dev) {
  return (dev == NULL) ? HAL_ERROR : dev->last_hal_status;
}

uint32_t MAX31856_GetCommunicationErrorCount(const MAX31856_HandleTypeDef *dev) {
  return (dev == NULL) ? 0U : dev->communication_error_count;
}

/* ==================================================================== */
/* MAIN API FUNCTIONS                                                    */
/* ==================================================================== */

bool MAX31856_Init(MAX31856_HandleTypeDef *dev,
                   SPI_HandleTypeDef *hspi,
                   GPIO_TypeDef *cs_port,
                   uint16_t cs_pin) {
  if (dev == NULL || hspi == NULL || cs_port == NULL) return false;

  dev->hspi = hspi;
  dev->cs_port = cs_port;
  dev->cs_pin = cs_pin;
  dev->conversionMode = MAX31856_ONESHOT;
  dev->initialized = false;
  dev->communication_error = false;
  dev->last_hal_status = HAL_OK;
  dev->communication_error_count = 0U;

  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

  /* Configure a known-safe baseline. Each call is checked through the latched
   * communication_error flag. Initialization succeeds only if all SPI accesses
   * actually complete.
   */
  if (!MAX31856_WriteRegister8(dev, MAX31856_MASK_REG, 0x00U)) return false;
  if (!MAX31856_WriteRegister8(dev, MAX31856_CR0_REG, MAX31856_CR0_OCFAULT0)) return false;
  if (!MAX31856_WriteRegister8(dev, MAX31856_CJTO_REG, 0x00U)) return false;

  MAX31856_SetThermocoupleType(dev, MAX31856_TCTYPE_K);
  if (dev->communication_error) return false;

  MAX31856_SetConversionMode(dev, MAX31856_ONESHOT);
  if (dev->communication_error) return false;

  dev->initialized = true;
  return true;
}

void MAX31856_SetConversionMode(MAX31856_HandleTypeDef *dev,
                                max31856_conversion_mode_t mode) {
  if (dev == NULL) return;

  uint8_t t = 0U;
  if (!MAX31856_ReadRegister8(dev, MAX31856_CR0_REG, &t)) return;

  if (mode == MAX31856_CONTINUOUS) {
    t |= MAX31856_CR0_AUTOCONVERT;
    t &= (uint8_t)~MAX31856_CR0_1SHOT;
  } else {
    t &= (uint8_t)~MAX31856_CR0_AUTOCONVERT;
    t |= MAX31856_CR0_1SHOT;
  }

  if (MAX31856_WriteRegister8(dev, MAX31856_CR0_REG, t)) {
    dev->conversionMode = mode;
  }
}

max31856_conversion_mode_t MAX31856_GetConversionMode(MAX31856_HandleTypeDef *dev) {
  return (dev == NULL) ? MAX31856_ONESHOT : dev->conversionMode;
}

void MAX31856_SetThermocoupleType(MAX31856_HandleTypeDef *dev,
                                  max31856_thermocoupletype_t type) {
  if (dev == NULL) return;

  uint8_t t = 0U;
  if (!MAX31856_ReadRegister8(dev, MAX31856_CR1_REG, &t)) return;

  t &= 0xF0U;
  t |= ((uint8_t)type & 0x0FU);
  (void)MAX31856_WriteRegister8(dev, MAX31856_CR1_REG, t);
}

max31856_thermocoupletype_t MAX31856_GetThermocoupleType(MAX31856_HandleTypeDef *dev) {
  uint8_t t = 0U;
  if (!MAX31856_ReadRegister8(dev, MAX31856_CR1_REG, &t)) {
    return MAX31856_TCTYPE_K;
  }
  return (max31856_thermocoupletype_t)(t & 0x0FU);
}

uint8_t MAX31856_ReadFault(MAX31856_HandleTypeDef *dev) {
  uint8_t value = 0U;
  if (!MAX31856_ReadRegister8(dev, MAX31856_SR_REG, &value)) {
    /* 0xFF is intentionally fail-safe: even legacy application code that does
     * not inspect communication_error will treat this as a sensor fault.
     */
    return 0xFFU;
  }
  return value;
}

void MAX31856_SetColdJunctionFaultThreshholds(MAX31856_HandleTypeDef *dev,
                                               int8_t low,
                                               int8_t high) {
  if (dev == NULL) return;
  if (!MAX31856_WriteRegister8(dev, MAX31856_CJLF_REG, (uint8_t)low)) return;
  (void)MAX31856_WriteRegister8(dev, MAX31856_CJHF_REG, (uint8_t)high);
}

void MAX31856_SetNoiseFilter(MAX31856_HandleTypeDef *dev,
                             max31856_noise_filter_t noiseFilter) {
  if (dev == NULL) return;

  uint8_t t = 0U;
  if (!MAX31856_ReadRegister8(dev, MAX31856_CR0_REG, &t)) return;

  if (noiseFilter == MAX31856_NOISE_FILTER_50HZ) {
    t |= 0x01U;
  } else {
    t &= 0xFEU;
  }

  (void)MAX31856_WriteRegister8(dev, MAX31856_CR0_REG, t);
}

void MAX31856_SetTempFaultThreshholds(MAX31856_HandleTypeDef *dev,
                                      float flow,
                                      float fhigh) {
  if (dev == NULL) return;

  int16_t low = (int16_t)(flow * 16.0f);
  int16_t high = (int16_t)(fhigh * 16.0f);

  if (!MAX31856_WriteRegister8(dev, MAX31856_LTHFTH_REG, (uint8_t)(high >> 8))) return;
  if (!MAX31856_WriteRegister8(dev, MAX31856_LTHFTL_REG, (uint8_t)high)) return;
  if (!MAX31856_WriteRegister8(dev, MAX31856_LTLFTH_REG, (uint8_t)(low >> 8))) return;
  (void)MAX31856_WriteRegister8(dev, MAX31856_LTLFTL_REG, (uint8_t)low);
}

void MAX31856_TriggerOneShot(MAX31856_HandleTypeDef *dev) {
  if (dev == NULL || dev->conversionMode == MAX31856_CONTINUOUS) return;

  uint8_t t = 0U;
  if (!MAX31856_ReadRegister8(dev, MAX31856_CR0_REG, &t)) return;

  t &= (uint8_t)~MAX31856_CR0_AUTOCONVERT;
  t |= MAX31856_CR0_1SHOT;
  (void)MAX31856_WriteRegister8(dev, MAX31856_CR0_REG, t);
}

bool MAX31856_ConversionComplete(MAX31856_HandleTypeDef *dev) {
  if (dev == NULL) return false;
  if (dev->conversionMode == MAX31856_CONTINUOUS) return true;

  uint8_t t = 0U;
  if (!MAX31856_ReadRegister8(dev, MAX31856_CR0_REG, &t)) return false;
  return ((t & MAX31856_CR0_1SHOT) == 0U);
}

float MAX31856_ReadCJTemperature(MAX31856_HandleTypeDef *dev) {
  uint16_t raw = 0U;
  if (!MAX31856_ReadRegister16(dev, MAX31856_CJTH_REG, &raw)) return NAN;
  return (float)raw / 256.0f;
}

float MAX31856_ReadThermocoupleTemperature(MAX31856_HandleTypeDef *dev) {
  if (dev == NULL) return NAN;

  if (dev->conversionMode == MAX31856_ONESHOT) {
    MAX31856_TriggerOneShot(dev);
    if (dev->communication_error) return NAN;

    uint32_t start = HAL_GetTick();
    while (!MAX31856_ConversionComplete(dev)) {
      if (dev->communication_error) return NAN;
      if ((uint32_t)(HAL_GetTick() - start) > 250U) return NAN;
      HAL_Delay(10U);
    }
  }

  uint32_t temp24 = 0U;
  if (!MAX31856_ReadRegister24(dev, MAX31856_LTCBH_REG, &temp24)) return NAN;

  int32_t signed_temp24 = (int32_t)temp24;
  if ((temp24 & 0x800000U) != 0U) {
    signed_temp24 |= (int32_t)0xFF000000U;
  }

  signed_temp24 >>= 5;
  return (float)signed_temp24 * 0.0078125f;
}
