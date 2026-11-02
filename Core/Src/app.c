/**
  ******************************************************************************
  * @file    app.c
  * @brief   Top-level state machine, periodic scheduling and event handling.
  *
  *          The main loop calls app_process() which performs, in order:
  *          key debounce, supply-state debounce, battery snapshot processing,
  *          motor arbitration + ramp, LED refresh, and idle/sleep check.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app.h"
#include "battery.h"
#include "motor.h"
#include "ws2812.h"
#include "power.h"

/* Private defines -----------------------------------------------------------*/
#define KEY_DEBOUNCE_MS    30U
#define ACTIVITY_IDLE_MS   15000U
#define LEVEL4_RUNTIME_MS  300000U   /* highest gear auto-downshift after 5 min */

/* Indicator colors (full brightness; scaled by the ws2812 module). */
#define LED_ORANGE_R 255U
#define LED_ORANGE_G 165U
#define LED_ORANGE_B 0U

/* Private types -------------------------------------------------------------*/
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_t;

/* Private variables ---------------------------------------------------------*/
app_state_t g_app;

/* Key event produced by the EXTI callback, debounced in the main loop. */
static volatile uint32_t g_key_event_pending;
static volatile uint32_t g_key_event_ms;
static bool             g_await_key_release;
static bool             g_key_was_high;

/* LED change tracking (avoids redundant SPI frames). */
static bool    g_led_on;
static bool    g_led_initialized;
static uint8_t g_led_last_r;
static uint8_t g_led_last_g;
static uint8_t g_led_last_b;

/* Previous supply state, used to detect changes for the idle timer. */
static charge_state_t g_prev_charge_state;
static power_source_t g_prev_power_source;

/* Highest-gear dwell timer (level 4 -> level 3 after 5 minutes). */
static bool     g_level4_timing;
static uint32_t g_level4_start_ms;

/* Private functions ---------------------------------------------------------*/
static bool power_inhibited(void)
{
    return (g_app.power_source == POWER_SOURCE_BATTERY) &&
           battery_is_undervoltage();
}

static void app_activity_tick(void)
{
    g_app.last_activity_ms = HAL_GetTick();
}

/* Key press debounce and event generation. */
static void key_process(void)
{
    uint32_t now = HAL_GetTick();
    bool key_high = (HAL_GPIO_ReadPin(KEY_EXTI_GPIO_Port, KEY_EXTI_Pin) == GPIO_PIN_SET);

    /* After a Shutdown wake-up, ignore key events until PA0 is seen released. */
    if (g_await_key_release)
    {
        if (!key_high)
        {
            g_await_key_release = false;
            g_key_was_high = false;
        }
        g_key_event_pending = 0U;
        return;
    }

    /* Runtime fallback for a missed EXTI edge. It does not replace the EXTI
     * wake path; it only makes normal-operation key handling robust when a
     * high-rate peripheral interrupt delays edge servicing. */
    if (key_high && !g_key_was_high && (g_key_event_pending == 0U))
    {
        g_key_event_pending = 1U;
        g_key_event_ms = now;
    }
    g_key_was_high = key_high;

    if (g_key_event_pending == 0U)
    {
        return;
    }

    if (!key_high)
    {
        /* Released before the debounce window: bounce or very short press. */
        g_key_event_pending = 0U;
        return;
    }

    if ((uint32_t)(now - g_key_event_ms) >= KEY_DEBOUNCE_MS)
    {
        g_key_event_pending = 0U;
        g_app.wake_button_pressed = 1U;

        /* A valid short press always refreshes the idle timer. */
        app_activity_tick();

        /* Cycle 0 -> 1 -> 2 -> 3 -> 4 -> 0 (stop -> silent -> 33% -> 66% ->
         * 100%), unless battery undervoltage inhibits. */
        motor_level_t next = (motor_level_t)(((uint32_t)g_app.motor_level + 1U) % 5U);

        if ((next != MOTOR_LEVEL_0) && power_inhibited())
        {
            /* Undervoltage while on battery: reject the non-zero request and
             * stay at 0. The motor remains stopped until the user presses again
             * after the condition clears. */
            next = MOTOR_LEVEL_0;
        }

        g_app.motor_level = next;
        motor_set_level(next);
    }
}

/* Central safety arbitration: on battery + confirmed undervoltage, force stop
 * and latch level 0. Handles both the running case and the "just unplugged
 * while already undervoltage" case without re-waiting 2 seconds. */
static void motor_arbitration(void)
{
    if (power_inhibited() && (g_app.motor_level != MOTOR_LEVEL_0))
    {
        g_app.motor_level = MOTOR_LEVEL_0;
        motor_set_level(MOTOR_LEVEL_0);
    }
}

/* Highest-gear protection: after 5 minutes at level 4, drop back to level 3.
 * The timer starts when level 4 is entered and is cleared on any other level,
 * so re-entering level 4 always starts a fresh 5-minute window. */
static void level4_timeout_process(void)
{
    uint32_t now = HAL_GetTick();

    if (g_app.motor_level != MOTOR_LEVEL_4)
    {
        g_level4_timing = false;
        return;
    }

    if (!g_level4_timing)
    {
        g_level4_timing   = true;
        g_level4_start_ms = now;
        return;
    }

    if ((uint32_t)(now - g_level4_start_ms) >= LEVEL4_RUNTIME_MS)
    {
        g_level4_timing   = false;
        g_app.motor_level = MOTOR_LEVEL_3;
        motor_set_level(MOTOR_LEVEL_3);
    }
}

static rgb_t compute_base_color(void)
{
    rgb_t c;

    switch (g_app.charge_state)
    {
        case CHARGE_STATE_INVALID:
            c.r = LED_ORANGE_R; c.g = LED_ORANGE_G; c.b = LED_ORANGE_B;
            break;

        case CHARGE_STATE_CHARGING:
        case CHARGE_STATE_DONE:
            c.r = 0U; c.g = 255U; c.b = 0U;   /* green */
            break;

        case CHARGE_STATE_ON_BATTERY:
        default:
            if (battery_is_undervoltage())
            {
                c.r = 255U; c.g = 0U; c.b = 0U;   /* red */
            }
            else
            {
                c.r = 0U; c.g = 0U; c.b = 255U;   /* blue */
            }
            break;
    }

    return c;
}

static uint32_t blink_half_period_ms(motor_level_t level)
{
    switch (level)
    {
        case MOTOR_LEVEL_1: return 2000U;   /* 0.25 Hz (silent) */
        case MOTOR_LEVEL_2: return 1000U;   /* 0.5 Hz */
        case MOTOR_LEVEL_3: return 500U;    /* 1 Hz   */
        case MOTOR_LEVEL_4: return 250U;    /* 2 Hz   */
        case MOTOR_LEVEL_0:
        default:            return 0U;      /* solid  */
    }
}

static void led_update(void)
{
    rgb_t base = compute_base_color();
    uint32_t half = blink_half_period_ms(g_app.motor_level);
    bool on = (half == 0U) || (((HAL_GetTick() / half) % 2U) == 0U);

    uint8_t r = on ? base.r : 0U;
    uint8_t g = on ? base.g : 0U;
    uint8_t b = on ? base.b : 0U;

    if (g_led_initialized && (r == g_led_last_r) &&
        (g == g_led_last_g) && (b == g_led_last_b))
    {
        return;
    }

    if (!g_led_on)
    {
        ws2812_enable();
        g_led_on = true;
    }

    ws2812_send(r, g, b);

    g_led_last_r = r;
    g_led_last_g = g;
    g_led_last_b = b;
    g_led_initialized = true;
}

static bool sleep_conditions_met(void)
{
    uint32_t now = HAL_GetTick();

    if (g_app.motor_level != MOTOR_LEVEL_0)
    {
        return false;
    }
    if (power_is_external())
    {
        return false;
    }
    if (HAL_GPIO_ReadPin(KEY_EXTI_GPIO_Port, KEY_EXTI_Pin) != GPIO_PIN_RESET)
    {
        return false;   /* PA0 not yet released */
    }
    if (!motor_is_idle())
    {
        return false;   /* wait for the safe-stop 100ms hold to finish */
    }
    if ((uint32_t)(now - g_app.last_activity_ms) < ACTIVITY_IDLE_MS)
    {
        return false;
    }

    return true;
}

/* EXTI rising callback (weak override) --------------------------------------*/
#if !defined(TEST_WS2812)
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY_EXTI_Pin)
    {
        g_key_event_pending = 1U;
        g_key_event_ms = HAL_GetTick();
    }
}
#endif

/* Exported functions --------------------------------------------------------*/
void app_init(void)
{
    /* Record the wake source (and clear PWR flags) before anything else. */
    power_record_wake_source();

    /* Safe defaults. BOOST_EN and WS2812_EN are already low from MX_GPIO_Init. */
    g_app.battery_mv        = 0U;
    g_app.vdda_mv           = 0U;
    g_app.motor_level       = MOTOR_LEVEL_0;
    g_app.charge_state      = CHARGE_STATE_ON_BATTERY;
    g_app.battery_state     = BATTERY_STATE_PRESENT;
    g_app.power_source      = POWER_SOURCE_BATTERY;
    g_app.wake_button_pressed = 0U;
    g_app.woke_from_shutdown  = power_woke_from_shutdown();
    g_app.last_activity_ms    = HAL_GetTick();

    g_level4_timing   = false;
    g_level4_start_ms = 0U;

    g_key_event_pending = 0U;
    g_key_event_ms      = 0U;
    g_await_key_release = (g_app.woke_from_shutdown != 0U);
    g_key_was_high      = (HAL_GPIO_ReadPin(KEY_EXTI_GPIO_Port, KEY_EXTI_Pin) == GPIO_PIN_SET);

    g_led_on          = false;
    g_led_initialized = false;
    g_led_last_r      = 0x00U;   /* force a first send */
    g_led_last_g      = 0xFFU;
    g_led_last_b      = 0xFFU;

    /* Module initialization: battery ADC+DMA, motor PWM, WS2812, supply. */
    battery_init();
    motor_init();
    ws2812_init();
    power_init();

    /* Sync the central state after module init. */
    g_app.charge_state = power_get_charge_state();
    g_app.power_source = power_get_source();
    g_prev_charge_state = g_app.charge_state;
    g_prev_power_source = g_app.power_source;
}

void app_resume_after_aborted_shutdown(void)
{
    /* The cancelled Shutdown entry powered down the indicator. Reset the LED
     * change tracking so the next led_update() re-powers and re-sends the
     * current color instead of leaving the indicator dark. */
    g_led_on          = false;
    g_led_initialized = false;

    /* Restart the idle timer so a bounce/edge that aborted Shutdown cannot
     * immediately re-trigger the sleep path. */
    app_activity_tick();
}

void app_process(void)
{
    /* 1. Key debounce and events. */
    key_process();

    /* 2. Supply state debounce and idle-timer refresh on change. */
    power_process();
    if ((power_get_charge_state() != g_prev_charge_state) ||
        (power_get_source() != g_prev_power_source))
    {
        g_prev_charge_state = power_get_charge_state();
        g_prev_power_source = power_get_source();
        g_app.charge_state  = g_prev_charge_state;
        g_app.power_source  = g_prev_power_source;
        app_activity_tick();
    }

    /* 3. Battery snapshot and undervoltage state. */
    battery_process();
    if (battery_data_valid())
    {
        g_app.battery_mv  = battery_get_mv();
        g_app.vdda_mv     = battery_get_vdda_mv();
        g_app.battery_state = battery_is_undervoltage()
                                ? BATTERY_STATE_UNDERVOLTAGE
                                : BATTERY_STATE_PRESENT;
    }

    /* 4. Motor arbitration, highest-gear timeout and non-blocking ramp /
     *    safe-stop. */
    motor_arbitration();
    level4_timeout_process();
    motor_process();

    /* 5. LED refresh (color + blink phase). */
    led_update();

    /* 6. Idle / sleep check. */
    if (sleep_conditions_met())
    {
        power_enter_shutdown();   /* non-returning */
    }
}
