/**
  ******************************************************************************
  * @file    ws2812.h
  * @brief   Single WS2812 indicator via SPI2 blocking transmit.
  *          GRB encoding for WS2812C-2020-V6, brightness scaling and enable
  *          (power) control.
  ******************************************************************************
  */
#ifndef __WS2812_H
#define __WS2812_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported functions --------------------------------------------------------*/
void ws2812_init(void);
void ws2812_send(uint8_t r, uint8_t g, uint8_t b); /* Blocking: encode + transmit + reset */
void ws2812_enable(void);
void ws2812_disable(void);
void ws2812_off(void);  /* Send off frame + latch, then power down */

#ifdef __cplusplus
}
#endif

#endif /* __WS2812_H */
