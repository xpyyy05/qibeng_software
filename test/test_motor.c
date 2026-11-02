/**
  ******************************************************************************
  * @file    test_motor.c
  * @brief   Unit test: motor speed cycling with soft-start.
  *
  *          On power-up the motor cycles 0% -> 16.5% -> 33% -> 66% -> 100% -> 0% ...
  *          with each speed held for 3000ms. The test drives the production
  *          motor module only: BOOST_EN is raised for every non-zero level and
  *          dropped at 0% (safe-stop: stop PWM, hold ~100ms, then boost off),
  *          and the soft-start ramp (10 CCR counts / 1ms toward the target) is
  *          run by motor_process() in the main loop. Non-blocking: timing comes
  *          from HAL_GetTick().
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "test.h"
#include "motor.h"

/* Private defines -----------------------------------------------------------*/
#define LEVEL_HOLD_MS 3000U   /* each speed is held for 3000ms */

/* Private variables ---------------------------------------------------------*/
static const motor_level_t test_levels[] = {
    MOTOR_LEVEL_0,   /* 0%    (CCR=999, boost off)       */
    MOTOR_LEVEL_1,   /* ~16.5% (CCR=834, silent)         */
    MOTOR_LEVEL_2,   /* ~33%  (CCR=666)                  */
    MOTOR_LEVEL_3,   /* ~66%  (CCR=333)                  */
    MOTOR_LEVEL_4    /* 100%  (CCR=0)                    */
};
#define TEST_LEVEL_COUNT (sizeof(test_levels) / sizeof(test_levels[0]))

static uint32_t test_level_index;
static uint32_t test_level_changed_ms;

/* Exported functions --------------------------------------------------------*/
void test_motor_init(void)
{
    test_level_index      = 0U;
    test_level_changed_ms = HAL_GetTick();

    motor_init();                    /* boost off, PWM=999, TIM1 started */
    motor_set_level(MOTOR_LEVEL_0);  /* stay at 0% */
}

void test_motor_process(void)
{
    uint32_t now = HAL_GetTick();

    /* Drive the motor module's soft-start / safe-stop state machine. */
    motor_process();

    if ((uint32_t)(now - test_level_changed_ms) < LEVEL_HOLD_MS)
    {
        return;
    }

    test_level_index = (test_level_index + 1U) % (uint32_t)TEST_LEVEL_COUNT;
    motor_set_level(test_levels[test_level_index]);
    test_level_changed_ms = now;
}
