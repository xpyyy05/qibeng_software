/**
  ******************************************************************************
  * @file    motor.c
  * @brief   Motor control.
  *
  *          N_MOTOR_PWM (TIM1_CH4) is an inverted speed command: CCR=999 is
  *          stop (100% high), CCR=0 is full speed (0% high). Boost is enabled
  *          directly (BOOST_EN high) before ramping; the motor PWM itself uses
  *          a non-blocking soft-start ramp (10 CCR counts / 1ms).
  *
  *          Safe-stop sequence: drive CCR=999 immediately, hold ~100ms, then
  *          drop BOOST_EN. It runs as a state machine (motor_process) in normal
  *          operation and as a blocking sequence (motor_shutdown) in the
  *          shutdown path only.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "motor.h"
#include "tim.h"

/* Private defines -----------------------------------------------------------*/
/* Inverted PWM: speed fraction = (999 - CCR) / 999, so the level targets below
 * are ~16.5%, ~33%, ~66% and 100% of full speed. */
#define PWM_STOP_CCR      999U
#define MOTOR_LEVEL1_CCR  834U   /* ~16.5% speed: silent gear */
#define MOTOR_LEVEL2_CCR  666U   /* ~33% speed */
#define MOTOR_LEVEL3_CCR  333U   /* ~66% speed */
#define MOTOR_LEVEL4_CCR  0U     /* 100% speed: PWM low for the whole period */

#define RAMP_STEP_CCR     10U
#define RAMP_STEP_MS      1U
#define STOP_HOLD_MS      100U

/* Private types -------------------------------------------------------------*/
typedef enum {
    MOTOR_STATE_OFF = 0,      /* boost low, CCR=999, idle */
    MOTOR_STATE_RAMPING,      /* boost high, CCR ramping down toward target */
    MOTOR_STATE_RUNNING,      /* boost high, CCR at target (non-zero) */
    MOTOR_STATE_STOPPING      /* CCR=999 applied, waiting 100ms before boost off */
} motor_state_t;

/* Private variables ---------------------------------------------------------*/
static motor_state_t m_state;
static motor_level_t m_target_level;
static uint32_t      m_current_ccr;
static uint32_t      m_target_ccr;
static uint32_t      m_last_step_ms;
static uint32_t      m_stop_start_ms;
static bool          m_boost_on;

/* Private functions ---------------------------------------------------------*/
static uint32_t level_to_ccr(motor_level_t level)
{
    switch (level)
    {
        case MOTOR_LEVEL_1: return MOTOR_LEVEL1_CCR;
        case MOTOR_LEVEL_2: return MOTOR_LEVEL2_CCR;
        case MOTOR_LEVEL_3: return MOTOR_LEVEL3_CCR;
        case MOTOR_LEVEL_4: return MOTOR_LEVEL4_CCR;
        case MOTOR_LEVEL_0:
        default:            return PWM_STOP_CCR;
    }
}

static void motor_set_ccr(uint32_t ccr)
{
    m_current_ccr = ccr;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ccr);
}

static void motor_set_boost(bool on)
{
    m_boost_on = on;
    HAL_GPIO_WritePin(BOOST_EN_GPIO_Port, BOOST_EN_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* Exported functions --------------------------------------------------------*/
void motor_init(void)
{
    m_state        = MOTOR_STATE_OFF;
    m_target_level = MOTOR_LEVEL_0;
    m_current_ccr  = PWM_STOP_CCR;
    m_target_ccr   = PWM_STOP_CCR;
    m_boost_on     = false;

    motor_set_boost(false);

    /* Drive the stop command before enabling the PWM output so the very first
     * cycle is already zero-speed (inverted PWM safety default). */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PWM_STOP_CCR);

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
}

void motor_set_level(motor_level_t level)
{
    m_target_level = level;

    if (level == MOTOR_LEVEL_0)
    {
        /* Stop now; cancel any ramp. Boost is dropped after the 100ms hold. */
        motor_set_ccr(PWM_STOP_CCR);

        if ((m_state != MOTOR_STATE_OFF) && (m_state != MOTOR_STATE_STOPPING))
        {
            m_state = MOTOR_STATE_STOPPING;
            m_stop_start_ms = HAL_GetTick();
        }
        return;
    }

    /* Non-zero level: enable boost first, then ramp the PWM toward target. */
    if (!m_boost_on)
    {
        motor_set_boost(true);
    }

    m_target_ccr = level_to_ccr(level);

    if (m_target_ccr < m_current_ccr)
    {
        /* Speed increase -> non-blocking soft-start ramp (CCR decreasing). */
        m_state = MOTOR_STATE_RAMPING;
        m_last_step_ms = HAL_GetTick();
    }
    else
    {
        /* Slowdown (or same target) -> apply immediately; this is safe. */
        motor_set_ccr(m_target_ccr);
        m_state = MOTOR_STATE_RUNNING;
    }
}

motor_level_t motor_get_level(void)
{
    return m_target_level;
}

void motor_process(void)
{
    uint32_t now = HAL_GetTick();

    if (m_state == MOTOR_STATE_RAMPING)
    {
        if ((uint32_t)(now - m_last_step_ms) >= RAMP_STEP_MS)
        {
            m_last_step_ms = now;

            if (m_current_ccr > m_target_ccr)
            {
                uint32_t next;

                /* Clamp before subtracting: with target 0 the last step would
                 * otherwise underflow (9 - 10 wraps in uint32_t) and stick in
                 * RAMPING. Since current > target here, the difference is safe. */
                if ((m_current_ccr - m_target_ccr) <= RAMP_STEP_CCR)
                {
                    next = m_target_ccr;
                }
                else
                {
                    next = m_current_ccr - RAMP_STEP_CCR;
                }
                motor_set_ccr(next);

                if (m_current_ccr == m_target_ccr)
                {
                    m_state = MOTOR_STATE_RUNNING;
                }
            }
            else
            {
                m_state = MOTOR_STATE_RUNNING;
            }
        }
    }
    else if (m_state == MOTOR_STATE_STOPPING)
    {
        if ((uint32_t)(now - m_stop_start_ms) >= STOP_HOLD_MS)
        {
            motor_set_boost(false);
            m_state = MOTOR_STATE_OFF;
        }
    }
    /* OFF and RUNNING states need no periodic work. */
}

bool motor_is_idle(void)
{
    return (m_state == MOTOR_STATE_OFF);
}

void motor_shutdown(void)
{
    /* Blocking safe-stop for the Shutdown path only:
     * stop PWM, hold ~100ms, then disable boost and stop the timer. */
    motor_set_ccr(PWM_STOP_CCR);
    HAL_Delay(STOP_HOLD_MS);
    motor_set_boost(false);
    m_state        = MOTOR_STATE_OFF;
    m_target_level = MOTOR_LEVEL_0;

    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
}
