/*
 * @file MAX31856.h
 * Author: TRAN NGUYEN HIEN
 * Reliability update:
 * - Finite SPI timeouts (no HAL_MAX_DELAY)
 * - Bounded retry
 * - Latched communication error/status diagnostics
 */

#ifndef MAX31856_H
#define MAX31856_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* SPI reliability settings.
 * A MAX31856 transfer is only a few bytes, so 10 ms is already very generous.
 * Two complete transaction attempts tolerate a transient EMI disturbance while
 * still guaranteeing that control returns to the main loop quickly.
 */
#define MAX31856_SPI_TIMEOUT_MS        10U
#define MAX31856_SPI_MAX_ATTEMPTS       2U

/* The registers of MAX31856 */
#define MAX31856_CR0_REG 0x00
#define MAX31856_CR0_AUTOCONVERT 0x80
#define MAX31856_CR0_1SHOT       0x40
#define MAX31856_CR0_OCFAULT1    0x20
#define MAX31856_CR0_OCFAULT0    0x10
#define MAX31856_CR0_CJ          0x08
#define MAX31856_CR0_FAULT       0x04
#define MAX31856_CR0_FAULTCLR    0x02

#define MAX31856_CR1_REG   0x01
#define MAX31856_MASK_REG  0x02
#define MAX31856_CJHF_REG  0x03
#define MAX31856_CJLF_REG  0x04
#define MAX31856_LTHFTH_REG 0x05
#define MAX31856_LTHFTL_REG 0x06
#define MAX31856_LTLFTH_REG 0x07
#define MAX31856_LTLFTL_REG 0x08
#define MAX31856_CJTO_REG    0x09
#define MAX31856_CJTH_REG    0x0A
#define MAX31856_CJTL_REG    0x0B
#define MAX31856_LTCBH_REG   0x0C
#define MAX31856_LTCBM_REG   0x0D
#define MAX31856_LTCBL_REG   0x0E
#define MAX31856_SR_REG      0x0F

/* ERROR FLAG */
#define MAX31856_FAULT_CJRANGE 0x80
#define MAX31856_FAULT_TCRANGE 0x40
#define MAX31856_FAULT_CJHIGH  0x20
#define MAX31856_FAULT_CJLOW   0x10
#define MAX31856_FAULT_TCHIGH  0x08
#define MAX31856_FAULT_TCLOW   0x04
#define MAX31856_FAULT_OVUV    0x02
#define MAX31856_FAULT_OPEN    0x01

typedef enum {
  MAX31856_NOISE_FILTER_50HZ,
  MAX31856_NOISE_FILTER_60HZ
} max31856_noise_filter_t;

typedef enum {
  MAX31856_TCTYPE_B = 0b0000,
  MAX31856_TCTYPE_E = 0b0001,
  MAX31856_TCTYPE_J = 0b0010,
  MAX31856_TCTYPE_K = 0b0011,
  MAX31856_TCTYPE_N = 0b0100,
  MAX31856_TCTYPE_R = 0b0101,
  MAX31856_TCTYPE_S = 0b0110,
  MAX31856_TCTYPE_T = 0b0111,
  MAX31856_VMODE_G8 = 0b1000,
  MAX31856_VMODE_G32 = 0b1100,
} max31856_thermocoupletype_t;

typedef enum {
  MAX31856_ONESHOT,
  MAX31856_ONESHOT_NOWAIT,
  MAX31856_CONTINUOUS
} max31856_conversion_mode_t;

typedef struct {
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef      *cs_port;
  uint16_t           cs_pin;
  max31856_conversion_mode_t conversionMode;
  bool               initialized;

  /* Reliability diagnostics.
   * communication_error is latched until MAX31856_ClearCommunicationError().
   * This prevents a later successful SPI transaction from hiding a failure that
   * occurred earlier in the same application-level operation.
   */
  bool               communication_error;
  HAL_StatusTypeDef  last_hal_status;
  uint32_t           communication_error_count;
} MAX31856_HandleTypeDef;

bool MAX31856_Init(MAX31856_HandleTypeDef *dev, SPI_HandleTypeDef *hspi,
                   GPIO_TypeDef *cs_port, uint16_t cs_pin);

void MAX31856_SetConversionMode(MAX31856_HandleTypeDef *dev, max31856_conversion_mode_t mode);
max31856_conversion_mode_t MAX31856_GetConversionMode(MAX31856_HandleTypeDef *dev);

void MAX31856_SetThermocoupleType(MAX31856_HandleTypeDef *dev, max31856_thermocoupletype_t type);
max31856_thermocoupletype_t MAX31856_GetThermocoupleType(MAX31856_HandleTypeDef *dev);

uint8_t MAX31856_ReadFault(MAX31856_HandleTypeDef *dev);

void MAX31856_TriggerOneShot(MAX31856_HandleTypeDef *dev);
bool MAX31856_ConversionComplete(MAX31856_HandleTypeDef *dev);

float MAX31856_ReadCJTemperature(MAX31856_HandleTypeDef *dev);
float MAX31856_ReadThermocoupleTemperature(MAX31856_HandleTypeDef *dev);

void MAX31856_SetTempFaultThreshholds(MAX31856_HandleTypeDef *dev, float flow, float fhigh);
void MAX31856_SetColdJunctionFaultThreshholds(MAX31856_HandleTypeDef *dev, int8_t low, int8_t high);
void MAX31856_SetNoiseFilter(MAX31856_HandleTypeDef *dev, max31856_noise_filter_t noiseFilter);

/* Communication health API. */
void MAX31856_ClearCommunicationError(MAX31856_HandleTypeDef *dev);
bool MAX31856_HasCommunicationError(const MAX31856_HandleTypeDef *dev);
HAL_StatusTypeDef MAX31856_GetLastHALStatus(const MAX31856_HandleTypeDef *dev);
uint32_t MAX31856_GetCommunicationErrorCount(const MAX31856_HandleTypeDef *dev);

#ifdef __cplusplus
}
#endif

#endif /* MAX31856_H */
