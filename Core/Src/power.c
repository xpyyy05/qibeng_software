/**
  ******************************************************************************
  * @file    power.c
  * @brief   Supply source parsing and low-power entry.
  *
  *          Debounces N_CHRG / N_DONE (both active-low, internal pull-up) into a
  *          charge_state_t and a power_source_t. External supply is any state
  *          where N_CHRG or N_DONE is low; battery supply requires both high.
  *
  *          power_enter_shutdown() performs the ordered low-power sequence:
  *          motor stop -> WS2812 off -> ADC/DMA stop -> EXTI teardown -> PWR
  *          pull-downs for PA0/PA5/PA8 + APC -> WKUP1 high-active -> clear WKUP
  *          flag -> enter Standby. (STM32G030 has no Shutdown mode; Standby is the
  *          equivalent lowest-power wake-on-PA0 state.)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "power.h"
#include "battery.h"
#include "motor.h"
#include "ws2812.h"

/* Private defines -----------------------------------------------------------*/
#define CHARGE_DEBOUNCE_MS 30U

/* Private variables ---------------------------------------------------------*/
static charge_state_t g_charge_state = CHARGE_STATE_ON_BATTERY;
static power_source_t g_power_source = POWER_SOURCE_BATTERY;

static uint8_t  g_debounce_raw;
static uint8_t  g_debounce_stable;
static uint32_t g_debounce_start_ms;
static bool     g_debouncing;

static bool g_woke_from_shutdown;

/* Private functions ---------------------------------------------------------*/
static void power_read_raw(uint8_t *chrg_active, uint8_t *done_active)
{
    GPIO_PinState nchrg = HAL_GPIO_ReadPin(N_CHRG_GPIO_Port, N_CHRG_Pin);
    GPIO_PinState ndone = HAL_GPIO_ReadPin(N_DONE_GPIO_Port, N_DONE_Pin);

    *chrg_active = (nchrg == GPIO_PIN_RESET) ? 1U : 0U;  /* active low */
    *done_active = (ndone == GPIO_PIN_RESET) ? 1U : 0U;
}

static void power_apply_state(uint8_t raw)
{
    uint8_t chrg = (uint8_t)(raw & 1U);
    uint8_t done = (uint8_t)((raw >> 1U) & 1U);

    if ((chrg != 0U) && (done != 0U))
    {
        g_charge_state = CHARGE_STATE_INVALID;
    }
    else if (chrg != 0U)
    {
        g_charge_state = CHARGE_STATE_CHARGING;
    }
    else if (done != 0U)
    {
        g_charge_state = CHARGE_STATE_DONE;
    }
    else
    {
        g_charge_state = CHARGE_STATE_ON_BATTERY;
    }

    if ((chrg != 0U) || (done != 0U))
    {
        g_power_source = POWER_SOURCE_EXTERNAL;
    }
    else
    {
        g_power_source = POWER_SOURCE_BATTERY;
    }
}

/* Exported functions --------------------------------------------------------*/
void power_init(void)
{
    uint8_t chrg;
    uint8_t done;
    uint8_t raw;

    g_charge_state = CHARGE_STATE_ON_BATTERY;
    g_power_source = POWER_SOURCE_BATTERY;
    g_debouncing   = false;
    g_debounce_start_ms = 0U;

    power_read_raw(&chrg, &done);
    raw = (uint8_t)((done << 1U) | chrg);

    g_debounce_raw    = raw;
    g_debounce_stable = raw;
    power_apply_state(raw);
}

void power_process(void)
{
    uint8_t chrg;
    uint8_t done;
    uint8_t raw;
    uint32_t now = HAL_GetTick();

    power_read_raw(&chrg, &done);
    raw = (uint8_t)((done << 1U) | chrg);

    if (raw != g_debounce_raw)
    {
        g_debounce_raw = raw;
        g_debouncing   = true;
        g_debounce_start_ms = now;
    }

    if (g_debouncing && ((uint32_t)(now - g_debounce_start_ms) >= CHARGE_DEBOUNCE_MS))
    {
        g_debouncing    = false;
        g_debounce_stable = raw;
        power_apply_state(raw);
    }
}

charge_state_t power_get_charge_state(void)
{
    return g_charge_state;
}

power_source_t power_get_source(void)
{
    return g_power_source;
}

bool power_is_external(void)
{
    return (g_power_source == POWER_SOURCE_EXTERNAL);
}

void power_record_wake_source(void)
{
    /* Read the standby flag BEFORE clearing it so the app can distinguish a
     * Standby wake-up (reset) from a normal power-on reset. */
    g_woke_from_shutdown = (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != 0U);

    /* Clear standby and wakeup flags for a clean boot state. */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF);
}

bool power_woke_from_shutdown(void)
{
    return g_woke_from_shutdown;
}

void power_enter_shutdown(void)
{
    /* 0. Cheap entry guard: abort before any teardown if PA0 is not yet low.
     *    The sleep precondition already checked this; aborting leaves the
     *    system fully functional. */
    if (HAL_GPIO_ReadPin(KEY_EXTI_GPIO_Port, KEY_EXTI_Pin) != GPIO_PIN_RESET)
    {
        return;
    }

    /* 1. Motor safe-stop (blocking): PWM=999, 100ms, boost off, timer stop. */
    motor_shutdown();

    /* 2. WS2812: send off frame + latch, then cut its power. */
    ws2812_off();

    /* 3. Stop continuous ADC + circular DMA. */
    battery_deinit();

    /* 4. FINAL PA0 confirmation, immediately before the non-blocking commit
     *    steps. This closes the ~100ms race: if the user pressed (or a bounce
     *    occurred) during the motor stop above, PA0 is now high and arming the
     *    high-active WKUP1 would cause an instant wake. Re-arm the runtime and
     *    return instead. The EXTI is still armed, so the button keeps working. */
    if (HAL_GPIO_ReadPin(KEY_EXTI_GPIO_Port, KEY_EXTI_Pin) != GPIO_PIN_RESET)
    {
        motor_init();
        battery_resume();
        app_resume_after_aborted_shutdown();
        return;
    }

    /* 5. Tear down the runtime EXTI: disable the IRQ and clear pending. */
    HAL_NVIC_DisableIRQ(KEY_EXTI_EXTI_IRQn);
    __HAL_GPIO_EXTI_CLEAR_IT(KEY_EXTI_Pin);

    /* 6. Standby pull-downs (PA0, PA5, PA8) + APC so the weak pull-downs are
     *    applied while in Standby. PA5 already has an external pull-down, but
     *    enabling its PWR pull-down as well keeps the state consistent. */
    HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_A,
                                 PWR_GPIO_BIT_0 | PWR_GPIO_BIT_5 | PWR_GPIO_BIT_8);
    HAL_PWREx_EnablePullUpPullDownConfig();

    /* 7. Enable WKUP1 (PA0), high-level detection. */
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);

    /* 8. Clear any stale wakeup flag immediately before entering Standby. */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF);

    /* 9. Enter the lowest-power state. STM32G030 has no Shutdown mode
     *    (PWR_SHDW_SUPPORT is undefined on this part), so Standby is used:
     *    it is the equivalent wake-on-PA0 state and keeps the same SBF/WUF
     *    wake-source flags. Wake-up follows the reset path, not the next line. */
    HAL_PWR_EnterSTANDBYMode();

    /* Should be unreachable. */
    while (1)
    {
    }
}
