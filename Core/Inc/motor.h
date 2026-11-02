/**
  ******************************************************************************
  * @file    motor.h
  * @brief   Motor control: boost enable, inverted PWM (0..999), five levels
  *          (stop + silent + three speeds), non-blocking soft-start ramp and
  *          safe-stop sequence.
  ******************************************************************************
  */
#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "app.h"
#include <stdbool.h>

/* Exported functions --------------------------------------------------------*/
void         motor_init(void);
void         motor_set_level(motor_level_t level);
motor_level_t motor_get_level(void);
void         motor_process(void);
bool         motor_is_idle(void);
void         motor_shutdown(void);   /* Blocking safe-stop, used only in shutdown path */

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
