/**
  ******************************************************************************
  * @file    battery.h
  * @brief   Battery voltage sensing via ADC1 + circular DMA.
  *          Performs VREFINT-based VDDA calibration, 8-sample averaging and
  *          undervoltage latch with hysteresis + duration.
  ******************************************************************************
  */
#ifndef __BATTERY_H
#define __BATTERY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>

/* Exported functions --------------------------------------------------------*/
void     battery_init(void);
void     battery_deinit(void);
void     battery_resume(void);   /* Re-arm ADC+DMA after an aborted shutdown (keeps the UV latch) */
void     battery_process(void);

uint32_t battery_get_mv(void);
uint32_t battery_get_vdda_mv(void);
bool     battery_is_undervoltage(void);
bool     battery_data_valid(void);

#ifdef __cplusplus
}
#endif

#endif /* __BATTERY_H */
