/**
  ******************************************************************************
  * @file    app.h
  * @brief   Top-level application state machine interface.
  *          Defines the shared enums and the central system state structure.
  ******************************************************************************
  */
#ifndef __APP_H
#define __APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/
/* Motor speed levels (0 = stop, 4 = full speed).
 * Level 1 is the silent gear at ~16.5% speed; levels 2..4 are the original
 * ~33%, ~66% and 100% speeds. */
typedef enum {
    MOTOR_LEVEL_0 = 0,
    MOTOR_LEVEL_1,   /* silent, ~16.5% */
    MOTOR_LEVEL_2,   /* ~33% */
    MOTOR_LEVEL_3,   /* ~66% */
    MOTOR_LEVEL_4    /* 100% */
} motor_level_t;

/* Charging / supply state decoded from N_CHRG and N_DONE. */
typedef enum {
    CHARGE_STATE_ON_BATTERY = 0,
    CHARGE_STATE_CHARGING,
    CHARGE_STATE_DONE,
    CHARGE_STATE_INVALID
} charge_state_t;

/* Battery voltage state (only "present" vs "confirmed undervoltage"). */
typedef enum {
    BATTERY_STATE_PRESENT = 0,
    BATTERY_STATE_UNDERVOLTAGE
} battery_state_t;

/* Power source arbitration. */
typedef enum {
    POWER_SOURCE_BATTERY = 0,
    POWER_SOURCE_EXTERNAL
} power_source_t;

/* Central system state. Owned and maintained by app.c. */
typedef struct {
    uint32_t       battery_mv;        /* Calibrated battery voltage, mV (x2 divider applied) */
    uint32_t       vdda_mv;           /* Real-time VDDA derived from VREFINT + factory cal */
    motor_level_t  motor_level;       /* User-selected motor level (0 after undervoltage latch) */
    charge_state_t charge_state;      /* Decoded charging/supply state */
    battery_state_t battery_state;    /* Present vs confirmed undervoltage */
    power_source_t power_source;      /* Battery vs external supply */
    uint8_t        wake_button_pressed; /* Latched key event (informational) */
    uint8_t        woke_from_shutdown;  /* Set when boot follows a Shutdown wake-up */
    uint32_t       last_activity_ms;    /* Tick of last activity (key / level / supply change) */
} app_state_t;

/* Exported variables --------------------------------------------------------*/
extern app_state_t g_app;

/* Exported functions --------------------------------------------------------*/
void app_init(void);
void app_process(void);
void app_resume_after_aborted_shutdown(void);  /* Restore LED state after a cancelled Shutdown entry */

#ifdef __cplusplus
}
#endif

#endif /* __APP_H */
