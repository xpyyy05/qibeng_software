/**
  ******************************************************************************
  * @file    power.h
  * @brief   Supply source parsing (N_CHRG / N_DONE debounce), idle/sleep
  *          preconditions and Shutdown entry with WKUP1 configuration.
  ******************************************************************************
  */
#ifndef __POWER_H
#define __POWER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "app.h"
#include <stdbool.h>

/* Exported functions --------------------------------------------------------*/
void          power_init(void);
void          power_process(void);
charge_state_t power_get_charge_state(void);
power_source_t power_get_source(void);
bool          power_is_external(void);

void          power_record_wake_source(void);
bool          power_woke_from_shutdown(void);

void          power_enter_shutdown(void);  /* Non-returning under normal conditions */

#ifdef __cplusplus
}
#endif

#endif /* __POWER_H */
